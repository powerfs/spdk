/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   poweraid_raid5f 模块入口：注册到 SPDK bdev_raid 框架
 *   阶段 2 子任务 D：start() 创建 FSM raid 对象 + 桥接 raid_bdev + 触发 CREATE_DSC。
 */

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/log.h"
#include "spdk/string.h"
#include "spdk/util.h"
#include "spdk/thread.h"
#include "spdk/accel.h"

#include "../bdev_raid.h"
#include "poweraid_raid5f.h"
#include "poweraid_raid5f_rmw.h"
#include "poweraid_raid5f_merge.h"

SPDK_LOG_REGISTER_COMPONENT(poweraid_raid5f)

/* ===== stripe_request 池化管理（参考 raid5f stripe_request alloc/free）===== */

static struct poweraid_raid5f_req *
poweraid_raid5f_stripe_request_alloc(struct poweraid_raid5f_io_channel *ch,
				     enum poweraid_raid5f_stripe_type type)
{
	struct poweraid_raid5f_req *req;

	req = calloc(1, sizeof(*req));
	if (!req) {
		return NULL;
	}
	req->type = type;
	req->ch = ch;
	return req;
}

static void
poweraid_raid5f_stripe_request_free(struct poweraid_raid5f_req *req)
{
	if (!req) {
		return;
	}
	/* D-5 将在此释放 parity_buf / src_bufs 数组等动态分配的 IO 缓冲 */
	free(req);
}

/* ===== per-thread IO channel（参考 raid5f_ioch_create/destroy L997-1049）===== */

static int
poweraid_raid5f_ioch_create(void *io_device, void *ctx_buf)
{
	struct poweraid_raid5f_raid *raid = io_device;
	struct poweraid_raid5f_io_channel *ch = ctx_buf;
	struct poweraid_raid5f_req *req;
	int i;

	TAILQ_INIT(&ch->free_write_stripe_requests);
	TAILQ_INIT(&ch->free_reconstruct_stripe_requests);
	TAILQ_INIT(&ch->xor_retry_queue);

	for (i = 0; i < POWERAID_RAID5F_MAX_STRIPES; i++) {
		req = poweraid_raid5f_stripe_request_alloc(ch,
				POWERAID_RAID5F_STRIPE_REQ_WRITE);
		if (!req) {
			goto err;
		}
		TAILQ_INSERT_HEAD(&ch->free_write_stripe_requests, req, link);
	}

	for (i = 0; i < POWERAID_RAID5F_MAX_STRIPES; i++) {
		req = poweraid_raid5f_stripe_request_alloc(ch,
				POWERAID_RAID5F_STRIPE_REQ_RECONSTRUCT);
		if (!req) {
			goto err;
		}
		TAILQ_INSERT_HEAD(&ch->free_reconstruct_stripe_requests, req, link);
	}

	ch->accel_ch = spdk_accel_get_io_channel();
	if (!ch->accel_ch) {
		SPDK_ERRLOG("poweraid_raid5f: failed to get accel io channel\n");
		goto err;
	}

	/* 合并层初始化（阶段 3b）*/
	if (poweraid_raid5f_merge_init(&ch->merge_ctx, raid, ch) != 0) {
		SPDK_ERRLOG("poweraid_raid5f: merge_init failed\n");
		spdk_put_io_channel(ch->accel_ch);
		ch->accel_ch = NULL;
		goto err;
	}

	SPDK_DEBUGLOG(poweraid_raid5f, "ioch_create: raid=%p ch=%p\n", raid, ch);
	return 0;

err:
	SPDK_ERRLOG("poweraid_raid5f: ioch_create failed\n");
	while ((req = TAILQ_FIRST(&ch->free_write_stripe_requests))) {
		TAILQ_REMOVE(&ch->free_write_stripe_requests, req, link);
		poweraid_raid5f_stripe_request_free(req);
	}
	while ((req = TAILQ_FIRST(&ch->free_reconstruct_stripe_requests))) {
		TAILQ_REMOVE(&ch->free_reconstruct_stripe_requests, req, link);
		poweraid_raid5f_stripe_request_free(req);
	}
	return -ENOMEM;
}

static void
poweraid_raid5f_ioch_destroy(void *io_device, void *ctx_buf)
{
	struct poweraid_raid5f_io_channel *ch = ctx_buf;
	struct poweraid_raid5f_req *req;

	assert(TAILQ_EMPTY(&ch->xor_retry_queue));

	/* 合并层销毁（先于 stripe_request 池，确保 pending IO 完成）*/
	poweraid_raid5f_merge_destroy(&ch->merge_ctx);

	while ((req = TAILQ_FIRST(&ch->free_write_stripe_requests))) {
		TAILQ_REMOVE(&ch->free_write_stripe_requests, req, link);
		poweraid_raid5f_stripe_request_free(req);
	}

	while ((req = TAILQ_FIRST(&ch->free_reconstruct_stripe_requests))) {
		TAILQ_REMOVE(&ch->free_reconstruct_stripe_requests, req, link);
		poweraid_raid5f_stripe_request_free(req);
	}

	if (ch->accel_ch) {
		spdk_put_io_channel(ch->accel_ch);
		ch->accel_ch = NULL;
	}

	SPDK_DEBUGLOG(poweraid_raid5f, "ioch_destroy: ch=%p\n", ch);
}

/* ===== 模块生命周期 ===== */

/* start()：raid_bdev 框架在 base bdev 就绪后调用。
 * 1. 分配 struct poweraid_raid5f_raid（FSM 对象），从 raid_bdev 填充字段。
 * 2. 分配 base_bdevs[]，按 base_bdev_info 创建 poweraid_raid5f_bdev（desc/slot/uuid）。
 * 3. raid_bdev->module_private = raid（桥接）。
 * 4. 计算 stripe 几何，设置 raid_bdev->bdev blockcnt/write_unit/optimal_io_boundary。
 * 5. 触发 RAID FSM EV_CREATE_DSC（→ sb_alloc → sb_init → OPEN_BDEVS → ... → ONLINE）。
 * 参考：raid5f.c raid5f_start（L1051-1101）。
 */
int
poweraid_raid5f_start(struct raid_bdev *raid_bdev)
{
	struct poweraid_raid5f_raid *raid;
	struct raid_base_bdev_info *base_info;
	uint64_t min_blockcnt = UINT64_MAX;
	uint64_t base_data_size, total_stripes, stripe_blocks;
	uint32_t data_chunks;
	uint8_t i;

	SPDK_NOTICELOG("poweraid_raid5f: start raid=%s num_base_bdevs=%u\n",
		       raid_bdev->bdev.name, raid_bdev->num_base_bdevs);

	if (raid_bdev->num_base_bdevs < 3) {
		SPDK_ERRLOG("poweraid_raid5f: need >= 3 base bdevs (got %u)\n",
			    raid_bdev->num_base_bdevs);
		return -EINVAL;
	}

	raid = calloc(1, sizeof(*raid));
	if (!raid) {
		SPDK_ERRLOG("poweraid_raid5f: alloc raid failed\n");
		return -ENOMEM;
	}
	raid->raid_bdev = raid_bdev;
	raid->level = (uint32_t)raid_bdev->level;
	raid->strip_size = raid_bdev->strip_size;
	raid->block_size = raid_bdev->bdev.blocklen;
	raid->num_base_bdevs = raid_bdev->num_base_bdevs;
	raid->delay_us = MERGE_DELAY_US_DEFAULT;  /* Stage 3c：默认 1ms，RPC 可改 */
	spdk_uuid_copy(&raid->uuid, &raid_bdev->bdev.uuid);
	snprintf(raid->name, sizeof(raid->name), "%s", raid_bdev->bdev.name);

	/* 分配 base_bdevs 数组 */
	raid->base_bdevs = calloc(raid->num_base_bdevs, sizeof(*raid->base_bdevs));
	if (!raid->base_bdevs) {
		SPDK_ERRLOG("poweraid_raid5f: alloc base_bdevs failed\n");
		free(raid);
		return -ENOMEM;
	}

	/* 桥接每个 base_bdev_info → poweraid_raid5f_bdev；求 min data_size */
	i = 0;
	RAID_FOR_EACH_BASE_BDEV(raid_bdev, base_info) {
		struct poweraid_raid5f_bdev *bdev = calloc(1, sizeof(*bdev));
		if (!bdev) {
			SPDK_ERRLOG("poweraid_raid5f: alloc bdev[%u] failed\n", i);
			/* 回滚 */
			while (i > 0) {
				free(raid->base_bdevs[--i]);
			}
			free(raid->base_bdevs);
			free(raid);
			return -ENOMEM;
		}
		bdev->slot = i;
		bdev->raid = raid;
		bdev->desc = base_info->desc;
		bdev->ch = NULL;  /* OPEN 阶段填充（子任务 D-4 IO channel 基础设施）*/
		spdk_uuid_copy(&bdev->uuid, &base_info->uuid);
		raid->base_bdevs[i] = bdev;
		if (base_info->desc) {
			uint64_t bs = base_info->data_size ? base_info->data_size :
				    spdk_bdev_desc_get_bdev(base_info->desc)->blockcnt;
			min_blockcnt = spdk_min(min_blockcnt, bs);
		}
		i++;
	}

	/* stripe 几何：RAID5 data_chunks = N-1 */
	data_chunks = raid->num_base_bdevs - 1;

	/* 数据区保留：LBA0 sb + PPL 区（固定 1MiB+4MiB）。
	 * 4K 块下 = 1280 块；要求按 strip 对齐。*/
	{
		uint64_t reserve = (POWERAID_RAID5F_PPL_REGION_OFFSET +
				    POWERAID_RAID5F_PPL_REGION_SIZE) /
				   raid->block_size;
		if (reserve % raid->strip_size != 0) {
			SPDK_ERRLOG("poweraid_raid5f: PPL reserve %"PRIu64
				    " not strip-aligned (strip=%u)\n",
				    reserve, raid->strip_size);
			return -EINVAL;
		}
		raid->data_offset_blocks = reserve;
	}
	if (min_blockcnt <= raid->data_offset_blocks) {
		SPDK_ERRLOG("poweraid_raid5f: base bdev too small (%"PRIu64
			    " <= reserve %"PRIu64")\n",
			    min_blockcnt, raid->data_offset_blocks);
		return -EINVAL;
	}
	base_data_size = ((min_blockcnt - raid->data_offset_blocks) /
			  raid->strip_size) * raid->strip_size;
	total_stripes = base_data_size / raid->strip_size;
	stripe_blocks = raid->strip_size * data_chunks;

	raid_bdev->bdev.blockcnt = stripe_blocks * total_stripes;
	/* 几何门控（阶段 3a）：
	 * optimal_io_boundary=strip_size + split_on_optimal_io_boundary：
	 *   读写均在 strip 边界拆分，保证模块收到的 IO 不跨 strip。
	 *   - 写：每个 IO ≤ strip_size，部分 stripe → RMW 路径。
	 *     全 stripe 写也被拆成多个 strip 写走 RMW；阶段 3b Merge 优化合并。
	 *   - 读：每个 IO ≤ strip_size，单 chunk 直读路径可处理。
	 * write_unit_size=strip_size：要求写 strip 对齐，sub-strip 写被拒绝。*/
	raid_bdev->bdev.optimal_io_boundary = raid->strip_size;
	raid_bdev->bdev.split_on_optimal_io_boundary = true;
	raid_bdev->bdev.write_unit_size = raid->strip_size;
	raid_bdev->bdev.split_on_write_unit = true;

	raid->raid_size = raid_bdev->bdev.blockcnt;
	raid_bdev->module_private = raid;

	/* 注册 io device（参考 raid5f L1097-1098），get_io_channel 时按此 ctx 分配 */
	spdk_io_device_register(raid, poweraid_raid5f_ioch_create,
				poweraid_raid5f_ioch_destroy,
				sizeof(struct poweraid_raid5f_io_channel), NULL);

	SPDK_NOTICELOG("poweraid_raid5f: raid=%p stripe_blocks=%"PRIu64
		       " total_stripes=%"PRIu64" blockcnt=%"PRIu64"\n",
		       raid, stripe_blocks, total_stripes, raid_bdev->bdev.blockcnt);

	/* 触发 FSM：CREATE_DSC → sb_alloc/init → OPEN_BDEVS → ... → ONLINE → recovery */
	poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_RAID, raid,
				   POWERAID_RAID_EV_CREATE_DSC);
	return 0;
}

/* io_device_unregister 完成回调：所有 io channel 释放后释放 raid 对象并通知框架 stop 完成。
 * 参考 raid5f_io_device_unregister_done（L1103-1111）。*/
static void
poweraid_raid5f_io_device_unregister_done(void *io_device)
{
	struct poweraid_raid5f_raid *raid = io_device;
	struct raid_bdev *raid_bdev = raid->raid_bdev;
	uint8_t i;

	/* 释放每盘 ppl_ctx（stop 早期触发 OFFLINE 时已释放，此处兜底）*/
	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL) {
			if (raid->base_bdevs[i]->ppl_ctx != NULL) {
				poweraid_raid5f_ppl_free(raid->base_bdevs[i]->ppl_ctx);
			}
			if (raid->base_bdevs[i]->ch != NULL) {
				spdk_put_io_channel(
					(struct spdk_io_channel *)raid->base_bdevs[i]->ch);
			}
			free(raid->base_bdevs[i]);
		}
	}
	free(raid->base_bdevs);

	/* sb_ctx 在 sb_alloc/sb_free 生命周期内管理；stop 时若仍有引用则释放 */
	if (raid->sb_ctx != NULL) {
		poweraid_raid5f_sb_free(raid);
	}

	SPDK_NOTICELOG("poweraid_raid5f: io_device unregistered, raid=%p\n", raid);
	free(raid);

	/* 通知 raid_bdev 框架 stop 异步完成 */
	raid_bdev_module_stop_done(raid_bdev);
}

/* stop()：框架在移除 raid 时调用。触发 OFFLINE，注销 io device，返回 false（异步）。
 * 参考 raid5f_stop（L1113-1121）：先 unregister io device，完成回调中 free raid + stop_done。
 */
bool
poweraid_raid5f_stop(struct raid_bdev *raid_bdev)
{
	struct poweraid_raid5f_raid *raid = raid_bdev->module_private;
	uint8_t i;

	SPDK_NOTICELOG("poweraid_raid5f: stop raid=%s\n", raid_bdev->bdev.name);

	if (raid == NULL) {
		return true;  /* 无 FSM 对象，同步完成 */
	}

	/* 触发 OFFLINE：FSM 清理运行时状态（停止 IO 接受、刷 PPL 等）*/
	poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_RAID, raid,
				   POWERAID_RAID_EV_OFFLINE);

	/* 释放每盘 ppl_ctx（OFFLINE FSM 内部也会处理，此处确保释放）*/
	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL && raid->base_bdevs[i]->ppl_ctx != NULL) {
			poweraid_raid5f_ppl_free(raid->base_bdevs[i]->ppl_ctx);
			raid->base_bdevs[i]->ppl_ctx = NULL;
		}
	}

	/* 注销 io device：所有 io channel 释放后回调 io_device_unregister_done */
	spdk_io_device_unregister(raid, poweraid_raid5f_io_device_unregister_done);
	return false;  /* 异步：stop_done 在 unregister_done 回调中调用 */
}

/* D-6 read 完成回调：直接读单 chunk 成功后完成 raid_io */
static void
poweraid_raid5f_read_complete_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct raid_bdev_io *raid_io = cb_arg;

	if (!success) {
		SPDK_ERRLOG("poweraid_raid5f: read failed\n");
	}
	raid_bdev_io_complete(raid_io, success ?
			       SPDK_BDEV_IO_STATUS_SUCCESS :
			       SPDK_BDEV_IO_STATUS_FAILED);
	spdk_bdev_free_io(bdev_io);
}

/* recovery 用：读某 stripe 的某个 data chunk（整 strip），供三分支 hash 判定。
 * chunk_idx 为 data 序号（0..N-2），内部映射到物理盘。*/
struct recovery_read_ctx {
	poweraid_raid5f_recovery_read_data_cb	cb;
	void					*cb_arg;
	void					*buf;
	size_t					len;
};

static void
recovery_strip_read_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct recovery_read_ctx *rctx = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		spdk_dma_free(rctx->buf);
		rctx->cb(-EIO, NULL, 0, rctx->cb_arg);
		free(rctx);
		return;
	}
	rctx->cb(0, rctx->buf, rctx->len, rctx->cb_arg);
	spdk_dma_free(rctx->buf);
	free(rctx);
}

void
poweraid_raid5f_recovery_read_strip(struct poweraid_raid5f_raid *raid,
				    uint64_t stripe_id, uint32_t chunk_idx,
				    uint32_t chunk_len_blocks,
				    poweraid_raid5f_recovery_read_data_cb cb,
				    void *cb_arg)
{
	struct poweraid_raid5f_bdev *bdev;
	struct recovery_read_ctx *rctx;
	uint32_t data_chunks = raid->num_base_bdevs - 1;
	uint8_t p_idx = data_chunks - (stripe_id % raid->num_base_bdevs);
	uint8_t phys = (chunk_idx < p_idx) ? chunk_idx : chunk_idx + 1;
	uint64_t offset;
	int rc;

	if (chunk_idx >= data_chunks || phys >= raid->num_base_bdevs) {
		cb(-EINVAL, NULL, 0, cb_arg);
		return;
	}
	bdev = raid->base_bdevs[phys];
	if (bdev == NULL || bdev->desc == NULL || bdev->ch == NULL) {
		cb(-ENODEV, NULL, 0, cb_arg);
		return;
	}

	rctx = calloc(1, sizeof(*rctx));
	if (rctx == NULL) {
		cb(-ENOMEM, NULL, 0, cb_arg);
		return;
	}
	rctx->cb = cb;
	rctx->cb_arg = cb_arg;
	rctx->len = (size_t)chunk_len_blocks * raid->block_size;
	rctx->buf = spdk_dma_malloc(rctx->len, 0x1000, NULL);
	if (rctx->buf == NULL) {
		free(rctx);
		cb(-ENOMEM, NULL, 0, cb_arg);
		return;
	}

	offset = raid->data_offset_blocks + stripe_id * raid->strip_size;
	rc = spdk_bdev_read_blocks((struct spdk_bdev_desc *)bdev->desc,
				   (struct spdk_io_channel *)bdev->ch,
				   rctx->buf, offset, chunk_len_blocks,
				   recovery_strip_read_io_cb, rctx);
	if (rc != 0) {
		spdk_dma_free(rctx->buf);
		free(rctx);
		cb(rc, NULL, 0, cb_arg);
	}
}

/* D-6 layout-aware read：直接读目标 chunk（非降级路径）。
 * 降级路径（目标盘故障→重建）留阶段 3 接入 recovery_run。 */
static int
poweraid_raid5f_submit_read_request(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid5f_raid *raid = raid_bdev->module_private;
	uint32_t data_chunks = raid->num_base_bdevs - 1;
	uint32_t stripe_blocks = raid->strip_size * data_chunks;
	uint64_t stripe_index = raid_io->offset_blocks / stripe_blocks;
	uint64_t stripe_offset = raid_io->offset_blocks % stripe_blocks;
	uint8_t chunk_data_idx = stripe_offset >> raid_bdev->strip_size_shift;
	uint8_t p_idx = data_chunks - (stripe_index % raid->num_base_bdevs);
	uint8_t chunk_idx = (chunk_data_idx < p_idx) ? chunk_data_idx : (chunk_data_idx + 1);
	struct raid_base_bdev_info *base_info = &raid_bdev->base_bdev_info[chunk_idx];
	struct spdk_io_channel *base_ch;
	uint64_t chunk_offset = stripe_offset - (chunk_data_idx << raid_bdev->strip_size_shift);
	uint64_t base_offset = raid->data_offset_blocks +
			       (stripe_index << raid_bdev->strip_size_shift) + chunk_offset;
	struct spdk_bdev_ext_io_opts io_opts = {0};

	io_opts.size = sizeof(io_opts);
	io_opts.memory_domain = raid_io->memory_domain;
	io_opts.memory_domain_ctx = raid_io->memory_domain_ctx;
	io_opts.metadata = raid_io->md_buf;

	base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, chunk_idx);
	if (base_ch == NULL || base_info->desc == NULL) {
		/* 目标盘故障 → 降级重建路径（阶段 3 完整接入 recovery_run） */
		SPDK_WARNLOG("poweraid_raid5f: degraded read chunk=%u, not yet supported\n",
			     chunk_idx);
		return -ENODEV;
	}

	return raid_bdev_readv_blocks_ext(base_info, base_ch,
					 raid_io->iovs, raid_io->iovcnt,
					 base_offset, raid_io->num_blocks,
					 poweraid_raid5f_read_complete_cb, raid_io,
					 &io_opts);
}

void
poweraid_raid5f_submit_rw_request(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid5f_raid *raid = raid_bdev->module_private;
	struct poweraid_raid5f_io_channel *ch;
	struct poweraid_raid5f_req *req;
	uint32_t data_chunks = raid->num_base_bdevs - 1;
	uint32_t stripe_blocks = raid->strip_size * data_chunks;
	uint64_t stripe_index, stripe_offset;
	uint32_t strip_size_bytes;
	uint32_t blocklen = raid->block_size;
	uint32_t i;
	int rc;

	/* 卷未 ONLINE，或 ONLINE 但 recovery/parity fixup 仍在跑（RESTORING）时，
	 * 拒绝 IO。返回 NOMEM 由 bdev 层自动排队重试，恢复结束后自然放行。*/
	if (poweraid_raid_state_test((uint64_t *)&raid->state,
				     POWERAID_RAID_ST_RESTORING) ||
	    !poweraid_raid_state_test((uint64_t *)&raid->state,
				      POWERAID_RAID_ST_ONLINE)) {
		SPDK_WARNLOG("poweraid_raid5f: IO before online/restoring, rejected\n");
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_NOMEM);
		return;
	}

	switch (raid_io->type) {
	case SPDK_BDEV_IO_TYPE_READ:
		assert(raid_io->num_blocks <= raid->strip_size);
		rc = poweraid_raid5f_submit_read_request(raid_io);
		if (rc != 0) {
			raid_bdev_io_complete(raid_io, rc == -ENOMEM ?
					      SPDK_BDEV_IO_STATUS_NOMEM :
					      SPDK_BDEV_IO_STATUS_FAILED);
		}
		return;

	case SPDK_BDEV_IO_TYPE_WRITE:
		break;  /* 继续下方 full-stripe 写路径 */

	default:
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	/* 写分流：完整 stripe → 全 stripe 写路径（5 步 barrier）；
	 * 部分 stripe（框架 split_on_write_unit/optimal_io_boundary 保证 strip 对齐且不跨 stripe）
	 * → 合并层（merge_submit，收集同 stripe 多 strip，1ms 超时或全覆盖后 flush）。*/
	stripe_index = raid_io->offset_blocks / stripe_blocks;
	stripe_offset = raid_io->offset_blocks % stripe_blocks;
	if (stripe_offset != 0 || raid_io->num_blocks != stripe_blocks) {
		int merge_rc = poweraid_raid5f_merge_submit(raid_io);
		if (merge_rc != 0) {
			raid_bdev_io_complete(raid_io, merge_rc == -ENOMEM ?
					      SPDK_BDEV_IO_STATUS_NOMEM :
					      SPDK_BDEV_IO_STATUS_FAILED);
		}
		return;
	}

	ch = raid_bdev_channel_get_module_ctx(raid_io->raid_ch);
	if (ch == NULL) {
		SPDK_ERRLOG("poweraid_raid5f: no module channel\n");
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	/* 从 free 池取一个 write stripe_request */
	req = TAILQ_FIRST(&ch->free_write_stripe_requests);
	if (req == NULL) {
		SPDK_ERRLOG("poweraid_raid5f: no free stripe_request\n");
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_NOMEM);
		return;
	}
	TAILQ_REMOVE(&ch->free_write_stripe_requests, req, link);

	/* 填充 req */
	req->type = POWERAID_RAID5F_STRIPE_REQ_WRITE;
	req->raid_io = raid_io;
	req->stripe_index = stripe_index;
	req->raid = raid;
	/* req->io 用于 IO_COMPLETE 调 raid_bdev_io_complete */
	req->io = raid_io;

	strip_size_bytes = raid->strip_size * blocklen;

	/* 分配 parity 缓冲（spdk_dma_malloc 保证 DMA 对齐）*/
	req->parity_buf_alloc = spdk_dma_malloc(strip_size_bytes, 0, NULL);
	if (req->parity_buf_alloc == NULL) {
		SPDK_ERRLOG("poweraid_raid5f: alloc parity_buf failed\n");
		goto err_free_req;
	}
	req->parity_buf = req->parity_buf_alloc;

	/* 分配全 stripe 数据缓冲，拷贝 raid_io->iovs */
	req->data_buf = spdk_dma_malloc(stripe_blocks * blocklen, 0, NULL);
	if (req->data_buf == NULL) {
		SPDK_ERRLOG("poweraid_raid5f: alloc data_buf failed\n");
		goto err_free_parity;
	}

	/* 拷贝 raid_io->iovs 到 data_buf（简化实现：非零拷贝但保证正确性）。
	 * spdk_iovcpy 返回值为已拷贝字节数（非剩余），全量拷贝才视为成功。*/
	rc = (int)spdk_iovcpy(raid_io->iovs, raid_io->iovcnt,
		 &(struct iovec){ .iov_base = req->data_buf,
				  .iov_len = stripe_blocks * blocklen }, 1);
	if (rc != (int)(stripe_blocks * blocklen)) {
		SPDK_ERRLOG("poweraid_raid5f: iovcpy short copied=%d expect=%"PRIu64"\n",
			    rc, (uint64_t)stripe_blocks * blocklen);
		goto err_free_data;
	}

	/* 构造 src_bufs 数组（每个 data chunk 一个指针，指向 data_buf 内对应区域）*/
	req->src_bufs = calloc(data_chunks, sizeof(void *));
	if (req->src_bufs == NULL) {
		SPDK_ERRLOG("poweraid_raid5f: alloc src_bufs failed\n");
		goto err_free_data;
	}
	for (i = 0; i < data_chunks; i++) {
		req->src_bufs[i] = (char *)req->data_buf + i * strip_size_bytes;
	}
	req->n_src = data_chunks;
	req->xor_len = strip_size_bytes;

	SPDK_DEBUGLOG(poweraid_raid5f, "submit_write: req=%p stripe=%"PRIu64
		      " parity_buf=%p data_buf=%p\n",
		      req, stripe_index, req->parity_buf, req->data_buf);

	/* 触发 REQ FSM：ASSIGN → CALC → WRITE_FULL → ... → IO_COMPLETE → DESTROY */
	poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
				   POWERAID_REQ_EV_ASSIGN);
	return;

err_free_data:
	spdk_dma_free(req->data_buf);
	req->data_buf = NULL;
err_free_parity:
	spdk_dma_free(req->parity_buf_alloc);
	req->parity_buf_alloc = NULL;
	req->parity_buf = NULL;
err_free_req:
	/* 回收 req 到 free 池 */
	TAILQ_INSERT_HEAD(&ch->free_write_stripe_requests, req, link);
	raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_NOMEM);
}

struct spdk_io_channel *
poweraid_raid5f_get_io_channel(struct raid_bdev *raid_bdev)
{
	struct poweraid_raid5f_raid *raid = raid_bdev->module_private;

	return spdk_get_io_channel(raid);
}

int
poweraid_raid5f_submit_process_request(struct raid_bdev_process_request *process_req,
				       struct raid_bdev_io_channel *raid_ch)
{
	/* TODO 阶段 3：rebuild/scrub process */
	return 0;
}

/* ===== FLUSH / UNMAP 处理（阶段 3b）=====
 * FLUSH：先 drain 合并层 pending entries（写路径自带 FUA/flush barrier，
 *         drain 后数据已在持久存储上），然后完成 flush IO。
 * UNMAP：阶段 4 实现，暂不支持。
 */
static void
poweraid_raid5f_submit_null_payload_request(struct raid_bdev_io *raid_io)
{
	switch (raid_io->type) {
	case SPDK_BDEV_IO_TYPE_FLUSH:
		poweraid_raid5f_merge_flush_all(raid_io);
		break;

	case SPDK_BDEV_IO_TYPE_UNMAP:
		/* TODO 阶段 4：TRIM 支持 */
		SPDK_WARNLOG("poweraid_raid5f: UNMAP not yet supported\n");
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		break;

	default:
		SPDK_ERRLOG("poweraid_raid5f: invalid null payload io type %u\n",
			    raid_io->type);
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		break;
	}
}

/* ===== 模块注册（参考 raid5f.c g_raid5f_module）===== */
struct raid_bdev_module g_poweraid_raid5f_module = {
	.level = SPDK_BDEV_RAID_LEVEL_RAID5F,  /* 阶段 1 复用 5F，后续可注册 RAID6 */
	.base_bdevs_min = 3,
	.base_bdevs_constraint = {CONSTRAINT_MAX_BASE_BDEVS_REMOVED, 1},
	.start = poweraid_raid5f_start,
	.stop = poweraid_raid5f_stop,
	.submit_rw_request = poweraid_raid5f_submit_rw_request,
	.get_io_channel = poweraid_raid5f_get_io_channel,
	.submit_process_request = poweraid_raid5f_submit_process_request,
	.submit_null_payload_request = poweraid_raid5f_submit_null_payload_request,
};
RAID_MODULE_REGISTER(&g_poweraid_raid5f_module)
