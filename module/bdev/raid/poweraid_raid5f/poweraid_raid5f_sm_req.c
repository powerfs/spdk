/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   REQ 层 FSM handler（20 个事件，对应 poweraid_raid5f_sm.h 中
 *   enum poweraid_raid5f_req_event 全部真实事件）
 *
 *   阶段 1：全部为 stub，仅 SPDK_DEBUGLOG 打印 + 立即返回。
 *   后续阶段逐个填充实际逻辑（参考 XISRC xnr_req_* 函数族）。
 *
 *   详见 raid5f-enhanced-design.md 第 3.10.6 节
 */

#include "spdk/stdinc.h"
#include "spdk/log.h"
#include "spdk/thread.h"
#include "spdk/accel.h"

#include "poweraid_raid5f.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_sm_req);

/* REQ 层状态位图为 uint32_t（节省内存），打印时用 PRIx32 */
#define DEFINE_REQ_HANDLER(name)                                        \
void                                                                     \
poweraid_raid5f_sm_req_##name(struct poweraid_raid5f_req *req,           \
			      enum poweraid_raid5f_req_event event)          \
{                                                                        \
	SPDK_DEBUGLOG(raid5f_sm_req, "%s: req=%p state=0x%" PRIx32       \
		       " event=%u\n", #name, req,                         \
		       req ? req->state : 0u, (uint32_t)event);           \
	(void)event;                                                    \
	/* TODO 阶段 2+：实现实际状态转换 */                               \
}

/* ===== 生命周期 stub（3 个；assign/io_complete/destroy 见下方实现）===== */
DEFINE_REQ_HANDLER(reassign)
DEFINE_REQ_HANDLER(complete_service_req)
DEFINE_REQ_HANDLER(wait_md)

/* ===== 读路径（6 个）===== */
DEFINE_REQ_HANDLER(read_restripe)
DEFINE_REQ_HANDLER(read_full)
DEFINE_REQ_HANDLER(read_all_full)
DEFINE_REQ_HANDLER(read0)
DEFINE_REQ_HANDLER(read1)
DEFINE_REQ_HANDLER(read_recon1)

/* ===== 写路径 stub（5 个；write_full + write_parity 见下方实现，calc 见下方实现）===== */
DEFINE_REQ_HANDLER(write_all_full)
DEFINE_REQ_HANDLER(write_recon)
DEFINE_REQ_HANDLER(write)
DEFINE_REQ_HANDLER(write_all)
DEFINE_REQ_HANDLER(write1)

#undef DEFINE_REQ_HANDLER

/* ===== 阶段 1.3 关键 handler 实现（REQ 层 4 个）=====
 * 借鉴 XISRC xnr_req_* 函数族 + raid5f stripe_request 生命周期。
 *
 * 注意：REQ 状态位图为 uint32_t（见 sm.h POWERAID_REQ_ST_*），
 * 现有 poweraid_raid_state_set/clear 宏参数为 uint64_t*，类型不匹配。
 * 故 REQ 层直接使用 __atomic_fetch_or/and（ACQ_REL）于 uint32_t。
 */

/* ASSIGN：分配 req 对象，绑定 raid_io，置 ASSIGNED，触发 CALC。
 * 参考：XISRC xnr_req_assign + raid5f_submit_rw_request 分配 stripe_request（L816）。
 */
void
poweraid_raid5f_sm_req_assign(struct poweraid_raid5f_req *req,
			      enum poweraid_raid5f_req_event event)
{
	SPDK_DEBUGLOG(raid5f_sm_req, "assign: req=%p io=%p event=%u\n", req,
		      req ? req->io : NULL, (uint32_t)event);
	(void)event;

	if (req == NULL) {
		return;
	}

	__atomic_fetch_or(&req->state, POWERAID_REQ_ST_ASSIGNED,
			  __ATOMIC_ACQ_REL);

	/* 阶段 1：统一进入 CALC；阶段 2 按读/写类型分流到 READ0/READ1 或 WRITE_FULL */
	poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
				   POWERAID_REQ_EV_CALC);
}

/* 同步 XOR fallback：accel 资源不足或不可用时保证前进。
 * 将 n_src 个 src_bufs XOR 到 parity_buf（uint64_t 粒度 + 尾部逐字节）。
 * xor_len 须为 8 的倍数（stripe block 对齐，NVMe 块 ≥ 512）。*/
static void
poweraid_raid5f_req_xor_sync(struct poweraid_raid5f_req *req)
{
	uint64_t *dst = req->parity_buf;
	uint64_t len = req->xor_len / sizeof(uint64_t);
	uint32_t i;
	uint64_t j;

	for (j = 0; j < len; j++) {
		uint64_t v = 0;
		for (i = 0; i < req->n_src; i++) {
			const uint64_t *src = req->src_bufs[i];
			v ^= src[j];
		}
		dst[j] = v;
	}
}

/* CALC：XOR/RS 校验计算。
 * 阶段 2 D-5：若 req 携带 src_bufs/parity_buf（由 submit_rw_request 填充），
 *           通过 spdk_accel_submit_xor 异步计算 parity，完成后按 req->type 分流：
 *           - WRITE：触发 WRITE_FULL（PPL 5 步 barrier）
 *           - RECONSTRUCT：触发 IO_COMPLETE
 *           - accel 资源不足（-ENOMEM）：fallback 到同步 XOR。
 *           无 buffer（阶段 1 兼容）：直接 IO_COMPLETE。
 */
static void
poweraid_raid5f_req_xor_cb(void *cb_arg, int status)
{
	struct poweraid_raid5f_req *req = cb_arg;

	if (status != 0) {
		SPDK_ERRLOG("req calc: accel xor failed (%d), fallback sync\n", status);
		poweraid_raid5f_req_xor_sync(req);
	}
	/* XOR 完成后按写/读类型分流 */
	if (req->type == POWERAID_RAID5F_STRIPE_REQ_WRITE) {
		poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
					   POWERAID_REQ_EV_WRITE_FULL);
	} else {
		poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
					   POWERAID_REQ_EV_IO_COMPLETE);
	}
}

void
poweraid_raid5f_sm_req_calc(struct poweraid_raid5f_req *req,
			    enum poweraid_raid5f_req_event event)
{
	struct spdk_io_channel *accel_ch;
	int rc;

	SPDK_DEBUGLOG(raid5f_sm_req, "calc: req=%p io=%p event=%u\n", req,
		      req ? req->io : NULL, (uint32_t)event);
	(void)event;

	if (req == NULL) {
		return;
	}

	__atomic_fetch_or(&req->state, POWERAID_REQ_ST_CALC, __ATOMIC_ACQ_REL);

	/* 无 buffer（阶段 1 兼容路径）：直接完成 */
	if (req->src_bufs == NULL || req->parity_buf == NULL ||
	    req->n_src == 0 || req->xor_len == 0) {
		poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
					   POWERAID_REQ_EV_IO_COMPLETE);
		return;
	}

	accel_ch = spdk_accel_get_io_channel();
	if (accel_ch == NULL) {
		SPDK_WARNLOG("req calc: no accel_ch, sync xor\n");
		poweraid_raid5f_req_xor_sync(req);
		if (req->type == POWERAID_RAID5F_STRIPE_REQ_WRITE) {
			poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
						   POWERAID_REQ_EV_WRITE_FULL);
		} else {
			poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
						   POWERAID_REQ_EV_IO_COMPLETE);
		}
		return;
	}

	rc = spdk_accel_submit_xor(accel_ch, req->parity_buf, req->src_bufs,
				   req->n_src, req->xor_len,
				   poweraid_raid5f_req_xor_cb, req);
	spdk_put_io_channel(accel_ch);

	if (rc == 0) {
		/* 异步完成，等 cb */
		return;
	}

	/* 提交失败（含 -ENOMEM）：同步 fallback，保证前进 */
	SPDK_WARNLOG("req calc: submit_xor rc=%d, sync fallback\n", rc);
	poweraid_raid5f_req_xor_sync(req);
	if (req->type == POWERAID_RAID5F_STRIPE_REQ_WRITE) {
		poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
					   POWERAID_REQ_EV_WRITE_FULL);
	} else {
		poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
					   POWERAID_REQ_EV_IO_COMPLETE);
	}
}

/* ===== D-5: WRITE_FULL + PPL 5 步 barrier =====
 * Step 1: ppl_append_record（FUA intent 落盘）
 * Step 2: 写 data chunks + parity chunk 到所有 base bdev
 * Step 3: flush 所有 base bdev
 * Step 4: ppl_commit（FUA 标记 committed）
 * Step 5: IO_COMPLETE
 *
 * 参考raid5f raid5f_stripe_request_submit_chunks + raid5f_chunk_write_complete。
 */

/* 前向声明 */
static void poweraid_raid5f_req_start_flushes(struct poweraid_raid5f_req *req);
static void poweraid_raid5f_req_chunk_io_cb(struct spdk_bdev_io *bdev_io,
					   bool success, void *cb_arg);

/* Step 1 完成：PPL append 回调 → 进入 Step 2（写 data+parity）*/
static void
poweraid_raid5f_req_ppl_append_done(int status, uint64_t seq, void *cb_arg)
{
	struct poweraid_raid5f_req *req = cb_arg;
	struct raid_bdev_io *raid_io = req->raid_io;
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid5f_raid *raid = req->raid;
	uint32_t strip_size_bytes = raid->strip_size * raid->block_size;
	uint8_t p_idx, data_chunks = raid->num_base_bdevs - 1;
	uint8_t i, chunk_idx;
	int rc;

	SPDK_DEBUGLOG(raid5f_sm_req, "ppl_append_done: req=%p status=%d seq=%"PRIu64"\n",
		      req, status, seq);

	if (status != 0) {
		SPDK_ERRLOG("ppl append failed (%d), continue write without PPL\n", status);
	}
	req->ppl_seq = (status == 0) ? seq : 0;

	/* Step 2: 写 data chunks + parity chunk 到所有 base bdev */
	req->base_bdev_io_remaining = raid->num_base_bdevs;
	req->base_bdev_io_status = 0;

	/* RAID5 left-symmetric parity 位置 */
	p_idx = data_chunks - (req->stripe_index % raid->num_base_bdevs);

	for (i = 0; i < raid->num_base_bdevs; i++) {
		struct raid_base_bdev_info *base_info = &raid_bdev->base_bdev_info[i];
		struct spdk_io_channel *base_ch;
		struct spdk_bdev_ext_io_opts io_opts = {0};
		struct iovec iov;
		void *buf;
		uint64_t base_offset;

		io_opts.size = sizeof(io_opts);

		base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, i);
		if (base_ch == NULL || base_info->desc == NULL) {
			SPDK_ERRLOG("write_full: no channel for chunk %u\n", i);
			req->base_bdev_io_status = -EIO;
			if (--req->base_bdev_io_remaining == 0) {
				poweraid_raid5f_req_start_flushes(req);
			}
			continue;
		}

		if (i == p_idx) {
			buf = req->parity_buf_alloc;
		} else {
			/* data chunk j → physical chunk i，data_buf 按 data chunk 序号排列 */
			chunk_idx = (i < p_idx) ? i : (i - 1);
			buf = (char *)req->data_buf + chunk_idx * strip_size_bytes;
		}

		iov.iov_base = buf;
		iov.iov_len = strip_size_bytes;
		base_offset = raid->data_offset_blocks +
			       req->stripe_index * raid->strip_size;

		/* 回调：decrement remaining，全部完成后 → flush */
		rc = raid_bdev_writev_blocks_ext(base_info, base_ch, &iov, 1,
						 base_offset, raid->strip_size,
						 poweraid_raid5f_req_chunk_io_cb, req, &io_opts);
		if (rc != 0) {
			if (rc == -ENOMEM) {
				/* TODO: queue IO wait；阶段 2 简化为失败 */
			}
			SPDK_ERRLOG("write_full: write chunk %u failed rc=%d\n", i, rc);
			req->base_bdev_io_status = rc;
			if (--req->base_bdev_io_remaining == 0) {
				poweraid_raid5f_req_start_flushes(req);
			}
		}
	}
}

/* Step 2/3 共用回调：base bdev IO（write 或 flush）完成。
 * spdk_bdev_io_completion_cb 签名：success 为布尔值（非 errno）。*/
static void
poweraid_raid5f_req_chunk_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct poweraid_raid5f_req *req = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		req->base_bdev_io_status = -EIO;
		SPDK_ERRLOG("chunk io failed (remaining=%u)\n",
			    req->base_bdev_io_remaining - 1);
	}

	if (--req->base_bdev_io_remaining == 0) {
		/* 当前步骤全部完成：判断是 write 还是 flush */
		if (req->base_bdev_io_status == 0 &&
		    !poweraid_raid_state_test((uint64_t *)&req->state,
					      POWERAID_REQ_ST_WRITE1)) {
			/* write 阶段完成 → 进入 flush 阶段 */
			poweraid_raid5f_req_start_flushes(req);
		} else if (poweraid_raid_state_test((uint64_t *)&req->state,
						    POWERAID_REQ_ST_WRITE1)) {
			/* flush 阶段完成 → 进入 Step 4 (ppl_commit) */
			poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
						   POWERAID_REQ_EV_WRITE_PARITY);
		} else {
			/* write 阶段有失败 → 跳过 flush，直接完成 */
			poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
						   POWERAID_REQ_EV_IO_COMPLETE);
		}
	}
}

/* Step 3：flush 所有 base bdev */
static void
poweraid_raid5f_req_start_flushes(struct poweraid_raid5f_req *req)
{
	struct raid_bdev_io *raid_io = req->raid_io;
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid5f_raid *raid = req->raid;
	uint8_t i;
	int rc;

	if (req->base_bdev_io_status != 0) {
		/* write 有失败，跳过 flush 直接完成 */
		poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
					   POWERAID_REQ_EV_IO_COMPLETE);
		return;
	}

	/* 标记进入 WRITE1（flush）阶段 */
	__atomic_fetch_or(&req->state, POWERAID_REQ_ST_WRITE1, __ATOMIC_ACQ_REL);
	req->base_bdev_io_remaining = raid->num_base_bdevs;

	for (i = 0; i < raid->num_base_bdevs; i++) {
		struct raid_base_bdev_info *base_info = &raid_bdev->base_bdev_info[i];
		struct spdk_io_channel *base_ch;
		uint64_t base_offset;

		base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, i);
		if (base_ch == NULL || base_info->desc == NULL) {
			if (--req->base_bdev_io_remaining == 0) {
				/* 全部 flush 完成 → Step 4 */
				poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
						   POWERAID_REQ_EV_WRITE_PARITY);
			}
			continue;
		}

		base_offset = raid->data_offset_blocks +
			       req->stripe_index * raid->strip_size;
		rc = raid_bdev_flush_blocks(base_info, base_ch, base_offset,
					    raid->strip_size,
					    poweraid_raid5f_req_chunk_io_cb, req);
		if (rc != 0) {
			SPDK_ERRLOG("flush chunk %u failed rc=%d\n", i, rc);
			req->base_bdev_io_status = rc;
			if (--req->base_bdev_io_remaining == 0) {
				poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
						   POWERAID_REQ_EV_WRITE_PARITY);
			}
		}
	}
}

/* Step 4：ppl_commit 回调 → Step 5 IO_COMPLETE */
static void
poweraid_raid5f_req_ppl_commit_done(int status, void *cb_arg)
{
	struct poweraid_raid5f_req *req = cb_arg;

	if (status != 0) {
		SPDK_ERRLOG("ppl commit failed (%d)\n", status);
	}
	poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
				   POWERAID_REQ_EV_IO_COMPLETE);
}

/* WRITE_FULL handler：入口，触发 Step 1（ppl_append_record）*/
void
poweraid_raid5f_sm_req_write_full(struct poweraid_raid5f_req *req,
				  enum poweraid_raid5f_req_event event)
{
	struct raid_bdev_io *raid_io = req->raid_io;
	struct poweraid_raid5f_raid *raid = req->raid;
	struct poweraid_raid5f_ppl_ctx *ppl_ctx = NULL;
	struct spdk_io_channel *ppl_ch = NULL;
	uint8_t i;
	uint64_t chunk_bitmap = 0;

	SPDK_DEBUGLOG(raid5f_sm_req, "write_full: req=%p stripe=%"PRIu64"\n",
		      req, req->stripe_index);
	(void)event;

	/* 阶段 2 简化：PPL 固定追加到第一块成员盘的 PPL 区（启动时只需 replay 单盘；
	 * barrier 仍 flush 全部成员盘）。channel 必须取自当前 IO 线程——fio plugin 的
	 * 配置线程与 IO 线程分离，ctx->ch 属于配置线程，数据面不可用。*/
	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL &&
		    raid->base_bdevs[i]->ppl_ctx != NULL) {
			ppl_ctx = raid->base_bdevs[i]->ppl_ctx;
			ppl_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, i);
			break;
		}
	}

	/* 构造 chunk_bitmap：标记本次写的所有 data chunk */
	for (i = 0; i < raid->num_base_bdevs - 1; i++) {
		chunk_bitmap |= (1ULL << i);
	}

	if (ppl_ctx != NULL && ppl_ch != NULL) {
		/* Step 1: PPL append（FUA 落盘 intent）*/
		/* old_data_hash=0（全 stripe 写无旧数据），new_data_hash 用 data_buf 算 */
		uint64_t new_hash = poweraid_raid5f_ppl_data_hash(
			req->data_buf, (raid->num_base_bdevs - 1) *
			raid->strip_size * raid->block_size);

		__atomic_fetch_or(&req->state, POWERAID_REQ_ST_WRITE,
				  __ATOMIC_ACQ_REL);
		poweraid_raid5f_ppl_append_record(ppl_ctx, ppl_ch,
						  req->stripe_index,
						  chunk_bitmap, 0 /* old_data_hash */,
						  new_hash,
						  poweraid_raid5f_req_ppl_append_done, req);
	} else {
		/* 无 PPL：直接进入 Step 2（写 data+parity），降级但保证数据可用 */
		SPDK_WARNLOG("write_full: no ppl_ctx, write without PPL protection\n");
		poweraid_raid5f_req_ppl_append_done(0, 0, req);
	}
}

/* WRITE_PARITY handler（复用为 Step 4：ppl_commit 入口）*/
void
poweraid_raid5f_sm_req_write_parity(struct poweraid_raid5f_req *req,
				    enum poweraid_raid5f_req_event event)
{
	struct poweraid_raid5f_raid *raid = req->raid;
	struct raid_bdev_io *raid_io = req->raid_io;
	struct poweraid_raid5f_ppl_ctx *ppl_ctx = NULL;
	struct spdk_io_channel *ppl_ch = NULL;
	uint8_t i;

	SPDK_DEBUGLOG(raid5f_sm_req, "write_parity(commit): req=%p ppl_seq=%"PRIu64"\n",
		      req, req->ppl_seq);
	(void)event;

	if (req->ppl_seq == 0) {
		/* 无 PPL record（append 失败或无 ppl_ctx），直接完成 */
		poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
					   POWERAID_REQ_EV_IO_COMPLETE);
		return;
	}

	/* 与 append 相同：固定第一块成员盘 PPL + 当前 IO 线程 channel */
	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL &&
		    raid->base_bdevs[i]->ppl_ctx != NULL) {
			ppl_ctx = raid->base_bdevs[i]->ppl_ctx;
			ppl_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, i);
			break;
		}
	}

	if (ppl_ctx != NULL && ppl_ch != NULL) {
		poweraid_raid5f_ppl_commit(ppl_ctx, ppl_ch, req->ppl_seq,
					   poweraid_raid5f_req_ppl_commit_done, req);
	} else {
		poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
					   POWERAID_REQ_EV_IO_COMPLETE);
	}
}

/* IO_COMPLETE：调 raid_bdev_io_complete 完成 IO，触发 DESTROY。
 * 参考：raid5f raid5f_stripe_request_complete → raid_bdev_io_complete（bdev_raid.c L605）。
 */
void
poweraid_raid5f_sm_req_io_complete(struct poweraid_raid5f_req *req,
				   enum poweraid_raid5f_req_event event)
{
	(void)event;

	if (req == NULL) {
		return;
	}

	__atomic_fetch_or(&req->state, POWERAID_REQ_ST_IO_COMPLETE,
			  __ATOMIC_ACQ_REL);

	if (req->io != NULL) {
		enum spdk_bdev_io_status status = (req->base_bdev_io_status == 0) ?
			SPDK_BDEV_IO_STATUS_SUCCESS : SPDK_BDEV_IO_STATUS_FAILED;
		raid_bdev_io_complete((struct raid_bdev_io *)req->io, status);
	}

	poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
				   POWERAID_REQ_EV_DESTROY);
}

/* DESTROY：释放 req 拥有的缓冲，回收到 io channel free 池。
 * 参考：XISRC xnr_req_destroy + raid5f_stripe_request_release（IO channel 池回收）。
 */
void
poweraid_raid5f_sm_req_destroy(struct poweraid_raid5f_req *req,
			       enum poweraid_raid5f_req_event event)
{
	SPDK_DEBUGLOG(raid5f_sm_req, "destroy: req=%p event=%u\n", req,
		      (uint32_t)event);
	(void)event;

	if (req == NULL) {
		return;
	}

	__atomic_fetch_or(&req->state, POWERAID_REQ_ST_DESTROYING,
			  __ATOMIC_ACQ_REL);

	/* 释放 D-5 分配的 IO 缓冲 */
	if (req->parity_buf_alloc != NULL) {
		spdk_dma_free(req->parity_buf_alloc);
		req->parity_buf_alloc = NULL;
	}
	if (req->data_buf != NULL) {
		spdk_dma_free(req->data_buf);
		req->data_buf = NULL;
	}
	if (req->src_bufs != NULL) {
		free(req->src_bufs);
		req->src_bufs = NULL;
	}

	/* 重置 req 状态，回收到 free 池 */
	req->state = 0;
	req->io = NULL;
	req->raid_io = NULL;
	req->parity_buf = NULL;
	req->n_src = 0;
	req->xor_len = 0;
	req->ppl_seq = 0;
	req->base_bdev_io_remaining = 0;
	req->base_bdev_io_status = 0;

	if (req->ch != NULL) {
		if (req->type == POWERAID_RAID5F_STRIPE_REQ_WRITE) {
			TAILQ_INSERT_HEAD(&req->ch->free_write_stripe_requests,
					   req, link);
		} else {
			TAILQ_INSERT_HEAD(&req->ch->free_reconstruct_stripe_requests,
					   req, link);
		}
	}
}

/* ===== 全局 REQ FSM 分派表 ===== */
poweraid_raid5f_req_handler_t
poweraid_raid5f_req_fsm[POWERAID_REQ_EV_COUNT] = {
	[POWERAID_REQ_EV_ASSIGN]                 = poweraid_raid5f_sm_req_assign,
	[POWERAID_REQ_EV_REASSIGN]               = poweraid_raid5f_sm_req_reassign,
	[POWERAID_REQ_EV_COMPLETE_SERVICE_REQ]   = poweraid_raid5f_sm_req_complete_service_req,
	[POWERAID_REQ_EV_IO_COMPLETE]            = poweraid_raid5f_sm_req_io_complete,
	[POWERAID_REQ_EV_WAIT_MD]                = poweraid_raid5f_sm_req_wait_md,
	[POWERAID_REQ_EV_DESTROY]               = poweraid_raid5f_sm_req_destroy,
	[POWERAID_REQ_EV_READ_RESTRIPE]         = poweraid_raid5f_sm_req_read_restripe,
	[POWERAID_REQ_EV_READ_FULL]             = poweraid_raid5f_sm_req_read_full,
	[POWERAID_REQ_EV_READ_ALL_FULL]         = poweraid_raid5f_sm_req_read_all_full,
	[POWERAID_REQ_EV_READ0]                 = poweraid_raid5f_sm_req_read0,
	[POWERAID_REQ_EV_READ1]                 = poweraid_raid5f_sm_req_read1,
	[POWERAID_REQ_EV_READ_RECON1]           = poweraid_raid5f_sm_req_read_recon1,
	[POWERAID_REQ_EV_WRITE_FULL]            = poweraid_raid5f_sm_req_write_full,
	[POWERAID_REQ_EV_WRITE_PARITY]          = poweraid_raid5f_sm_req_write_parity,
	[POWERAID_REQ_EV_WRITE_ALL_FULL]        = poweraid_raid5f_sm_req_write_all_full,
	[POWERAID_REQ_EV_WRITE_RECON]           = poweraid_raid5f_sm_req_write_recon,
	[POWERAID_REQ_EV_WRITE]                 = poweraid_raid5f_sm_req_write,
	[POWERAID_REQ_EV_WRITE_ALL]             = poweraid_raid5f_sm_req_write_all,
	[POWERAID_REQ_EV_WRITE1]                = poweraid_raid5f_sm_req_write1,
	[POWERAID_REQ_EV_CALC]                 = poweraid_raid5f_sm_req_calc,
};

SPDK_STATIC_ASSERT(SPDK_COUNTOF(poweraid_raid5f_req_fsm) ==
		   POWERAID_REQ_EV_COUNT,
		   "req fsm table size mismatch with enum");
