/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   poweraid RAID 换盘/重建公共层实现，详见 poweraid_raid_common_rebuild.h。
 */

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/thread.h"
#include "spdk/util.h"

#include "../bdev_raid.h"
#include "poweraid_raid_common.h"
#include "poweraid_raid_common_rebuild.h"
#include "poweraid_raid_common_merge.h"
#include "poweraid_raid_common_sb.h"

SPDK_LOG_REGISTER_COMPONENT(poweraid_rebuild)

/* ===== 窗口门控 ===== */

enum poweraid_raid_common_gate
poweraid_raid_common_rebuild_gate_classify(struct poweraid_raid_common_raid *raid,
		struct raid_bdev_io_channel *raid_ch, uint64_t stripe_index,
		struct raid_bdev_io_channel **eff_ch)
{
	struct raid_bdev_io_channel *shadow;
	uint32_t data_chunks;
	uint64_t offset, stripe_end;

	*eff_ch = raid_ch;

	shadow = raid_bdev_channel_get_processed_channel(raid_ch);
	if (shadow == NULL) {
		uint8_t i;

		/* 该 channel 上无活动进程或 shadow 尚未装好。成员缺失（热拔后、
		 * 换盘前）或目标槽位 cbdev 已 FAULTED（rebuild_starting 已借开
		 * 新盘 desc、但各 IO channel 的 shadow 还没装完的窗口）时，
		 * 写一个 stripe 总要落数据条带与 P/Q，此时直读目标槽位会 EIO，
		 * 无法在本层安全完成。统一 GATE_WAIT：bdev 层以 NOMEM 重试排队，
		 * 等 shadow 就绪、重建窗口越过该 stripe 后放行；读路径不走门控。*/
		for (i = 0; i < raid->num_base_bdevs; i++) {
			struct poweraid_raid_common_bdev *cbdev = raid->base_bdevs[i];

			if (raid->raid_bdev->base_bdev_info[i].desc == NULL ||
			    (cbdev != NULL && poweraid_raid_state_test(&cbdev->state,
							     POWERAID_BDEV_ST_FAULTED))) {
				return POWERAID_RAID_COMMON_GATE_WAIT;
			}
		}
		return POWERAID_RAID_COMMON_GATE_PASS;
	}

	data_chunks = raid->num_base_bdevs - raid->num_parity;
	stripe_end = (stripe_index + 1) * (uint64_t)raid->strip_size * data_chunks;
	offset = raid_bdev_channel_get_process_offset(raid_ch);

	if (offset != UINT64_MAX && stripe_end <= offset) {
		/* stripe 已完整越过重建窗口：shadow channel 把目标槽位路由到 target_ch */
		*eff_ch = shadow;
		return POWERAID_RAID_COMMON_GATE_PROCESSED;
	}

	return POWERAID_RAID_COMMON_GATE_WAIT;
}

/* ===== 卷降级状态刷新（app 线程视图：desc==NULL 即缺盘）===== */

static void
rebuild_refresh_degraded(struct poweraid_raid_common_raid *raid)
{
	struct raid_bdev *raid_bdev = raid->raid_bdev;
	uint8_t i, n_fault = 0;

	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid_bdev->base_bdev_info[i].desc == NULL) {
			n_fault++;
		}
	}

	poweraid_raid_state_clear(&raid->state,
				   POWERAID_RAID_ST_DEGRADED |
				   POWERAID_RAID_ST_DEGRADED2);
	if (n_fault >= 2) {
		poweraid_raid_state_set(&raid->state,
					POWERAID_RAID_ST_DEGRADED |
					POWERAID_RAID_ST_DEGRADED2);
	} else if (n_fault == 1) {
		poweraid_raid_state_set(&raid->state, POWERAID_RAID_ST_DEGRADED);
	}
}

/* ===== 替换盘格式化（sb + PPL 区，app 线程异步状态机）=====
 *
 * 框架在 hook_rebuild_starting 后立即启动重建引擎。引擎写数据区
 * （data_offset_blocks 起），格式化写 LBA0 sb 与 1MiB 处 PPL super，
 * 区域互不重叠，可并行；重建完成与格式化完成两个事件在
 * rebuild_finalize_if_done 汇聚，二者都结束才清 FAULTED/RECON。
 */
struct replace_format_op {
	struct poweraid_raid_common_raid		*raid;
	struct poweraid_raid_common_bdev	*cbdev;
	struct spdk_bdev_desc			*desc;
	struct spdk_io_channel			*ch;
	const void				*sb_buf;
	uint32_t				sb_size;
	uint64_t				ppl_region_offset;
	uint64_t				ppl_region_size;
	struct spdk_bdev_io_wait_entry		wait_entry;
};

static void replace_format_kick_sb_write(struct replace_format_op *op);

static void
replace_format_finish(struct replace_format_op *op, int status)
{
	struct poweraid_raid_common_raid *raid = op->raid;
	struct poweraid_raid_common_bdev *cbdev = op->cbdev;

	if (status != 0) {
		/* 与建卷路径一致：格式化失败不阻塞重建，降级为无 PPL 保护 */
		SPDK_ERRLOG("replace: format slot %u failed (%d), continuing degraded\n",
			    cbdev->slot, status);
	}

	cbdev->format_done = true;
	poweraid_raid_state_set(&cbdev->state,
				POWERAID_BDEV_ST_ONLINE |
				POWERAID_BDEV_ST_MD_VALID);

	/* 若重建引擎已先于格式化结束，在此完成状态收敛；否则由 process_complete 收敛 */
	if (raid->raid_bdev->process == NULL) {
		poweraid_raid_state_clear(&cbdev->state,
					  POWERAID_BDEV_ST_FAULTED |
					  POWERAID_BDEV_ST_RECON);
		poweraid_raid_state_clear(&raid->state, POWERAID_RAID_ST_RECON);
		rebuild_refresh_degraded(raid);
	}

	free(op);
}

static void
replace_format_ppl_init_cb(int status, void *cb_arg)
{
	struct replace_format_op *op = cb_arg;

	if (status != 0) {
		SPDK_ERRLOG("replace: ppl_init slot %u failed (%d)\n",
			    op->cbdev->slot, status);
	}

	replace_format_finish(op, status);
}

static void
replace_format_sb_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct replace_format_op *op = cb_arg;
	struct poweraid_raid_common_ppl_ctx *ppl_ctx;
	int status = 0;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		SPDK_ERRLOG("replace: sb write IO failed slot %u\n", op->cbdev->slot);
		replace_format_finish(op, -EIO);
		return;
	}

	ppl_ctx = poweraid_raid_common_ppl_alloc(op->desc, op->ch,
			op->raid->block_size,
			op->ppl_region_offset, op->ppl_region_size);
	if (ppl_ctx == NULL) {
		replace_format_finish(op, -ENOMEM);
		return;
	}
	op->cbdev->ppl_ctx = ppl_ctx;
	poweraid_raid_common_ppl_init(ppl_ctx, replace_format_ppl_init_cb, op);
}

static void
replace_format_kick_sb_write(struct replace_format_op *op)
{
	struct spdk_bdev *bdev = spdk_bdev_desc_get_bdev(op->desc);
	uint32_t num_blocks = op->sb_size / op->raid->block_size;
	int rc;

	rc = spdk_bdev_write_blocks(op->desc, op->ch, (void *)op->sb_buf, 0,
				    num_blocks, replace_format_sb_write_cb, op);
	if (rc == -ENOMEM) {
		op->wait_entry.bdev = bdev;
		op->wait_entry.cb_fn = (spdk_bdev_io_wait_cb)replace_format_kick_sb_write;
		op->wait_entry.cb_arg = op;
		spdk_bdev_queue_io_wait(bdev, op->ch, &op->wait_entry);
		return;
	}
	if (rc != 0) {
		SPDK_ERRLOG("replace: sb write_blocks slot %u rc=%d\n",
			    op->cbdev->slot, rc);
		replace_format_finish(op, rc);
	}
}

static void
replace_format_start(struct poweraid_raid_common_raid *raid,
		     struct poweraid_raid_common_bdev *cbdev)
{
	struct replace_format_op *op;
	uint32_t sb_size;

	op = calloc(1, sizeof(*op));
	if (op == NULL) {
		SPDK_ERRLOG("replace: format op alloc failed slot %u\n", cbdev->slot);
		return;
	}
	op->raid = raid;
	op->cbdev = cbdev;
	op->desc = (struct spdk_bdev_desc *)cbdev->desc;
	op->ch = (struct spdk_io_channel *)cbdev->ch;

	op->sb_buf = poweraid_raid_common_sb_get_write_buffer(raid, &sb_size);
	if (op->sb_buf == NULL || sb_size == 0 ||
	    sb_size % raid->block_size != 0) {
		SPDK_ERRLOG("replace: sb buffer unavailable slot %u\n", cbdev->slot);
		free(op);
		return;
	}
	op->sb_size = sb_size;

	if (poweraid_raid_common_sb_get_ppl_region(raid, &op->ppl_region_offset,
			&op->ppl_region_size) != 0) {
		SPDK_ERRLOG("replace: no PPL region geometry slot %u\n", cbdev->slot);
		free(op);
		return;
	}

	replace_format_kick_sb_write(op);
}

/* ===== hook 1：成员盘移除 ===== */

void
poweraid_raid_common_hook_base_bdev_removed(struct raid_bdev *raid_bdev, uint8_t slot)
{
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	struct poweraid_raid_common_bdev *cbdev;

	if (raid == NULL || slot >= raid->num_base_bdevs) {
		return;
	}
	cbdev = raid->base_bdevs[slot];
	if (cbdev == NULL) {
		return;
	}

	if (cbdev->ch != NULL) {
		spdk_put_io_channel((struct spdk_io_channel *)cbdev->ch);
		cbdev->ch = NULL;
	}
	if (cbdev->ppl_ctx != NULL) {
		poweraid_raid_common_ppl_free(cbdev->ppl_ctx);
		cbdev->ppl_ctx = NULL;
	}
	cbdev->desc = NULL;
	cbdev->format_done = false;
	poweraid_raid_state_clear(&cbdev->state,
				  POWERAID_BDEV_ST_ONLINE |
				  POWERAID_BDEV_ST_OPEN |
				  POWERAID_BDEV_ST_MD_VALID |
				  POWERAID_BDEV_ST_RECON);
	poweraid_raid_state_set(&cbdev->state, POWERAID_BDEV_ST_FAULTED);

	rebuild_refresh_degraded(raid);

	SPDK_NOTICELOG("replace: base bdev slot %u removed from raid %s\n",
		       slot, raid->name);
}

/* ===== hook 2：新盘成为 rebuild 目标 ===== */

void
poweraid_raid_common_hook_rebuild_starting(struct raid_bdev *raid_bdev,
		struct raid_base_bdev_info *target)
{
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	uint8_t slot = raid_bdev_base_bdev_slot(target);
	struct poweraid_raid_common_bdev *cbdev;
	struct spdk_io_channel *ch;

	if (raid == NULL || slot >= raid->num_base_bdevs) {
		return;
	}

	cbdev = raid->base_bdevs[slot];
	if (cbdev == NULL) {
		cbdev = calloc(1, sizeof(*cbdev));
		if (cbdev == NULL) {
			SPDK_ERRLOG("replace: alloc common bdev slot %u failed\n", slot);
			return;
		}
		cbdev->slot = slot;
		cbdev->raid = raid;
		raid->base_bdevs[slot] = cbdev;
	}

	/* 释放/覆盖旧故障盘残留（正常路径 hook_base_bdev_removed 已清理，兜底）*/
	if (cbdev->ch != NULL) {
		spdk_put_io_channel((struct spdk_io_channel *)cbdev->ch);
		cbdev->ch = NULL;
	}
	if (cbdev->ppl_ctx != NULL) {
		poweraid_raid_common_ppl_free(cbdev->ppl_ctx);
		cbdev->ppl_ctx = NULL;
	}

	ch = spdk_bdev_get_io_channel(target->desc);
	if (ch == NULL) {
		SPDK_ERRLOG("replace: get_io_channel slot %u failed\n", slot);
		return;
	}

	cbdev->desc = target->desc;  /* 框架借用指针，不持有 */
	cbdev->ch = ch;
	cbdev->format_done = false;
	spdk_uuid_copy(&cbdev->uuid, &target->uuid);
	poweraid_raid_state_clear(&cbdev->state, POWERAID_BDEV_ST_ONLINE);
	poweraid_raid_state_set(&cbdev->state,
				POWERAID_BDEV_ST_OPEN |
				POWERAID_BDEV_ST_RECON |
				POWERAID_BDEV_ST_FAULTED);  /* 重建完成前读仍走降级 */

	poweraid_raid_state_set(&raid->state, POWERAID_RAID_ST_RECON);

	SPDK_NOTICELOG("replace: rebuild starting slot %u bdev %s raid %s\n",
		       slot, target->name, raid->name);
	/* 根因修复：把运行时新成员的 data_offset/data_size 统一到模块 PPL/MWL
	 * 预留口径（superblock=false 时框架给的是 0），必须在重建引擎启动前完成，
	 * 否则 helper 业务读(LBA0) 与 raw 重建写(reserve) 物理偏移不一致。*/
	poweraid_raid_common_sb_unify_member_data_offset(raid_bdev, target);

	/* 异步格式化新盘 sb + PPL 区（与数据区重建区域不重叠）*/
	replace_format_start(raid, cbdev);
}

/* ===== hook 3：rebuild 进程结束 ===== */

void
poweraid_raid_common_hook_process_complete(struct raid_bdev *raid_bdev,
		struct raid_base_bdev_info *target, int status)
{
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	uint8_t slot = raid_bdev_base_bdev_slot(target);
	struct poweraid_raid_common_bdev *cbdev;

	if (raid == NULL || slot >= raid->num_base_bdevs) {
		return;
	}
	cbdev = raid->base_bdevs[slot];

	if (status != 0) {
		/* 失败：框架随后移除目标盘并触发 base_bdev_removed hook；
		 * RECON 位在此提前清除。*/
		SPDK_ERRLOG("replace: rebuild slot %u failed status=%d\n", slot, status);
		poweraid_raid_state_clear(&raid->state, POWERAID_RAID_ST_RECON);
		if (cbdev != NULL) {
			poweraid_raid_state_clear(&cbdev->state,
						  POWERAID_BDEV_ST_RECON);
		}
		return;
	}

	if (cbdev == NULL) {
		return;
	}

	SPDK_NOTICELOG("replace: rebuild slot %u complete, format_done=%d\n",
		       slot, cbdev->format_done != false);

	/* 格式化已结束则立即收敛；否则等待 replace_format_finish 收敛 */
	if (cbdev->format_done) {
		poweraid_raid_state_clear(&cbdev->state,
					  POWERAID_BDEV_ST_FAULTED |
					  POWERAID_BDEV_ST_RECON);
		poweraid_raid_state_set(&cbdev->state,
					POWERAID_BDEV_ST_ONLINE |
					POWERAID_BDEV_ST_MD_VALID);
		poweraid_raid_state_clear(&raid->state, POWERAID_RAID_ST_RECON);
		rebuild_refresh_degraded(raid);
	}
}

/* ===== hook 4：重建窗口在某 channel 上推进/结束 ===== */

void
poweraid_raid_common_hook_window_advanced(struct raid_bdev *raid_bdev,
		struct raid_bdev_io_channel *raid_ch, bool ended, int status)
{
	struct poweraid_raid_common_io_channel *mod_ch;

	if (status != 0) {
		return;
	}

	mod_ch = raid_bdev_channel_get_module_ctx(raid_ch);
	if (mod_ch == NULL) {
		return;
	}

	(void)ended;
        /* 重放该 channel merge 层被门控延迟的条目；条目内部重新分类，
         * 仍未越过窗口的继续留在 pending 列表等下次推进（poller 也会重试）。*/
        poweraid_raid_common_merge_replay_gated(mod_ch);
}

/* ===== rebuild process 引擎（5f/6f 共用；模块差异由 ops.recover_missing 屏蔽）=====
 *
 * 框架语义（见 bdev_raid.c）：
 *  - 每个 process_req 携带一块 max_window 大小的 DMA iov 缓冲；模块消耗一个 req
 *    返回“消费的 raid 数据块数”，完成后异步 raid_bdev_process_request_complete。
 *  - 每个 RAID stripe 只向目标成员写一条 strip，故消费粒度 = 一个 stripe
 *    （strip_size * data_chunks 个 raid LBA），offset 按 stripe 对齐。
 *  - recover_missing 把目标 strip 重构进 process_req->iov，再写入目标盘物理位置
 *    （公共层真实偏移 data_offset_blocks + stripe*strip_size）。
 */

struct poweraid_raid_common_rebuild_req {
	struct raid_bdev_process_request	*process_req;
	struct poweraid_raid_common_raid	*raid;
	uint64_t				stripe_index;
	uint32_t				strip_bytes;
	int					complete_status;
	struct spdk_thread			*thread;
	struct spdk_bdev_io_wait_entry		write_wait;
};

static void
rebuild_complete_msg(void *arg)
{
	struct poweraid_raid_common_rebuild_req *rreq = arg;
	struct raid_bdev_process_request *process_req = rreq->process_req;
	int status = rreq->complete_status;

	free(rreq);
	raid_bdev_process_request_complete(process_req, status);
}

static void
rebuild_complete_deferred(struct poweraid_raid_common_rebuild_req *rreq, int status)
{
	/* 框架要求 complete 一定在 submit 返回之后（window_remaining 先加后减）。
	 * recover_missing 可能在全失败路径同步回调，统一经线程消息延迟完成。*/
	rreq->complete_status = status;
	spdk_thread_send_msg(rreq->thread, rebuild_complete_msg, rreq);
}

static void rebuild_submit_write(void *arg);

static void
rebuild_write_completed(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct poweraid_raid_common_rebuild_req *rreq = cb_arg;
	struct raid_bdev_process_request *process_req = rreq->process_req;

	spdk_bdev_free_io(bdev_io);
	free(rreq);
	raid_bdev_process_request_complete(process_req, success ? 0 : -EIO);
}

static void
rebuild_write_wait_cb(void *arg)
{
	rebuild_submit_write(arg);
}

static void
rebuild_submit_write(void *arg)
{
	struct poweraid_raid_common_rebuild_req *rreq = arg;
	struct poweraid_raid_common_raid *raid = rreq->raid;
	struct raid_bdev_process_request *process_req = rreq->process_req;
	struct raid_base_bdev_info *target = process_req->target;
	uint64_t write_offset;
	int rc;

	/* 成员盘物理偏移：元数据区 + stripe*strip_size（base_info->data_offset==0 的
	 * 配置下，所有 poweraid 成员写均使用公共层记录的真实偏移）。*/
	write_offset = raid->data_offset_blocks +
		       rreq->stripe_index * (uint64_t)raid->strip_size;

	rc = spdk_bdev_write_blocks(target->desc, process_req->target_ch,
				    process_req->iov.iov_base, write_offset,
				    raid->strip_size,
				    rebuild_write_completed, rreq);
	if (rc == -ENOMEM) {
		rreq->write_wait.bdev = spdk_bdev_desc_get_bdev(target->desc);
		rreq->write_wait.cb_fn = (spdk_bdev_io_wait_cb)rebuild_write_wait_cb;
		rreq->write_wait.cb_arg = rreq;
		spdk_bdev_queue_io_wait(rreq->write_wait.bdev,
					process_req->target_ch, &rreq->write_wait);
		return;
	}
	if (rc != 0) {
		/* 走到这里 recover 已异步完成（submit 早已返回），可直接完成。*/
		free(rreq);
		raid_bdev_process_request_complete(process_req, rc);
	}
}

static void
rebuild_recover_done(int status, void *cb_arg)
{
	struct poweraid_raid_common_rebuild_req *rreq = cb_arg;

	if (status != 0) {
		rebuild_complete_deferred(rreq, status);
		return;
	}

	rebuild_submit_write(rreq);
}

int
poweraid_raid_common_submit_process_request(struct raid_bdev_process_request *process_req,
					    struct raid_bdev_io_channel *raid_ch)
{
	struct raid_bdev *raid_bdev = process_req->target->raid_bdev;
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	uint32_t data_chunks = raid->num_base_bdevs - raid->num_parity;
	uint32_t stripe_blocks = raid->strip_size * data_chunks;
	uint32_t strip_bytes = raid->strip_size * raid->block_size;
	uint64_t stripe_index;
	uint8_t target_slot;
	struct poweraid_raid_common_rebuild_req *rreq;
	int rc;

	if (process_req->num_blocks < stripe_blocks) {
		/* 窗口尾部不足一个 stripe（正常几何下不应发生）：框架将结束窗口。*/
		return 0;
	}

	stripe_index = process_req->offset_blocks / stripe_blocks;
	target_slot = raid_bdev_base_bdev_slot(process_req->target);

	if (raid->ops.recover_missing == NULL) {
		SPDK_ERRLOG("poweraid rebuild: module lacks recover_missing op\n");
		return -ENOTSUP;
	}

	rreq = calloc(1, sizeof(*rreq));
	if (rreq == NULL) {
		return -ENOMEM;
	}
	rreq->process_req = process_req;
	rreq->raid = raid;
	rreq->stripe_index = stripe_index;
	rreq->strip_bytes = strip_bytes;
	rreq->thread = spdk_get_thread();

	process_req->iov.iov_len = strip_bytes;

	rc = raid->ops.recover_missing(raid, raid_ch, stripe_index, target_slot,
				       process_req->iov.iov_base, strip_bytes,
				       rebuild_recover_done, rreq);
	if (rc != 0) {
		free(rreq);
		return rc;
	}

	/* 每个 req 处理一个 stripe：消费 stripe_blocks 个 raid LBA。*/
	return (int)stripe_blocks;
}
