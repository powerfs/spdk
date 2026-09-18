/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   合并层 + 延迟合并实现（Stage 3b）
 *
 *   收集同一 stripe 的多个部分 stripe 写，凑齐后一次性提交：
 *     - 全 chunk 覆盖 → 全 stripe 写路径（carrier approach + REQ FSM）
 *     - 部分 chunk 覆盖 → RMW（poweraid_raid_common_rmw_submit_merged）
 *
 *   收集阶段数据仅在内存（chunk_bufs），不写 PPL。
 *   最终写路径（全 stripe 或 RMW）包含 PPL。
 *   断电时收集中的数据丢失，磁盘数据一致（无残留 PPL record）。
 *
 *   Per-IO-channel TAILQ + 线性扫描，poller 200μs 扫描，1ms 超时 flush。
 *
 *   详见 raid5f-enhanced-design.md 第 3.2 节（阶段 3b）。
 */

#include "spdk/stdinc.h"
#include "spdk/log.h"
#include "spdk/string.h"
#include "spdk/util.h"
#include "spdk/bdev_module.h"
#include "spdk/thread.h"
#include "spdk/env.h"

#include "../bdev_raid.h"
#include "poweraid_raid_common.h"
#include "poweraid_raid_common_merge.h"
#include "poweraid_raid_common_rmw.h"
#include "poweraid_raid_common_rebuild.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_merge);

/* ===== 前向声明 ===== */
static int merge_poller_fn(void *arg);
static void merge_flush_entry(struct merge_entry *entry);
static void merge_carrier_complete_cb(struct raid_bdev_io *carrier,
				       enum spdk_bdev_io_status status);
static void merge_rmw_complete_cb(int status, void *cb_arg);
static void merge_check_drain(struct merge_ctx *mctx);

/* ===== entry 分配/释放 ===== */

static struct merge_entry *
merge_entry_alloc(struct merge_ctx *mctx,
		   struct raid_bdev_io_channel *raid_ch,
		   struct poweraid_raid_common_io_channel *mod_ch,
		   struct poweraid_raid_common_raid *raid,
		   uint64_t stripe_index)
{
	uint32_t data_chunks = raid->num_base_bdevs - raid->num_parity;
	uint8_t p_idx, q_idx;
	poweraid_raid_common_get_parity_idx(raid, stripe_index, &p_idx, &q_idx);
	uint32_t strip_bytes = raid->strip_size * raid->block_size;
	struct merge_entry *entry;
	uint32_t i;

	entry = calloc(1, sizeof(*entry));
	if (entry == NULL) {
		return NULL;
	}

	entry->chunk_bufs = calloc(data_chunks, sizeof(void *));
	if (entry->chunk_bufs == NULL) {
		free(entry);
		return NULL;
	}

	/* 为每个 chunk 预分配 strip_bytes 缓冲（避免 submit 路径 ENOMEM）*/
	for (i = 0; i < data_chunks; i++) {
		entry->chunk_bufs[i] = spdk_dma_malloc(strip_bytes, 0, NULL);
		if (entry->chunk_bufs[i] == NULL) {
			goto err_free_bufs;
		}
	}

	entry->raid = raid;
	entry->raid_ch = raid_ch;
	entry->eff_raid_ch = raid_ch;
	entry->mod_ch = mod_ch;
	entry->mctx = mctx;
	entry->stripe_index = stripe_index;
	entry->p_idx = p_idx;
	entry->data_chunks = data_chunks;
	entry->strip_bytes = strip_bytes;
	entry->chunk_bitmap = 0;
	entry->create_time_ticks = spdk_get_ticks();
	entry->flushing = false;
	TAILQ_INIT(&entry->pending_ios);

	return entry;

err_free_bufs:
	for (i = 0; i < data_chunks; i++) {
		if (entry->chunk_bufs[i] != NULL) {
			spdk_dma_free(entry->chunk_bufs[i]);
		}
	}
	free(entry->chunk_bufs);
	free(entry);
	return NULL;
}

static void
merge_entry_free(struct merge_entry *entry)
{
	struct merge_pending_io *pio;
	uint32_t i;

	/* 释放 chunk_bufs */
	if (entry->chunk_bufs != NULL) {
		for (i = 0; i < entry->data_chunks; i++) {
			if (entry->chunk_bufs[i] != NULL) {
				spdk_dma_free(entry->chunk_bufs[i]);
				entry->chunk_bufs[i] = NULL;
			}
		}
		free(entry->chunk_bufs);
		entry->chunk_bufs = NULL;
	}

	/* pending_ios 应已在完成回调中清空，此处兜底 */
	while ((pio = TAILQ_FIRST(&entry->pending_ios)) != NULL) {
		TAILQ_REMOVE(&entry->pending_ios, pio, link);
		/* 不应到这里；若到了说明 pending_ios 未被完成 */
		SPDK_WARNLOG("merge: orphan pending_io %p in entry %p\n", pio, entry);
		free(pio);
	}

	free(entry);
}

/* 从 pending_list 移除并释放 entry（flush 完成回调中调用）*/
static void
merge_entry_remove_and_free(struct merge_entry *entry)
{
	struct merge_ctx *mctx = entry->mctx;

	TAILQ_REMOVE(&mctx->pending_list, entry, link);
	mctx->num_pending--;
	merge_entry_free(entry);
}

/* ===== pending_io 完成辅助 ===== */

static void
merge_complete_pending_ios(struct merge_entry *entry, enum spdk_bdev_io_status status)
{
	struct merge_pending_io *pio;

	while ((pio = TAILQ_FIRST(&entry->pending_ios)) != NULL) {
		struct raid_bdev_io *raid_io = pio->raid_io;

		TAILQ_REMOVE(&entry->pending_ios, pio, link);
		free(pio);

		/* 清除 carrier 的 completion_cb（carrier 即第一个 pending_io 的 raid_io）*/
		raid_io->completion_cb = NULL;
		raid_io->module_private = NULL;

		raid_bdev_io_complete(raid_io, status);
	}
}

/* ===== 全 stripe 写路径（carrier approach）===== */

/* carrier 完成回调：REQ FSM IO_COMPLETE → raid_bdev_io_complete → 此回调。
 * 完成所有 pending_ios，释放 entry，drain 检查。*/
static void
merge_carrier_complete_cb(struct raid_bdev_io *carrier,
			   enum spdk_bdev_io_status status)
{
	struct merge_entry *entry = carrier->module_private;
	struct merge_ctx *mctx = entry->mctx;

	SPDK_DEBUGLOG(raid5f_merge, "carrier complete: entry=%p stripe=%"PRIu64
		      " status=%u\n", entry, entry->stripe_index, (uint32_t)status);

	/* 完成所有 pending_ios（包括 carrier 自身）*/
	merge_complete_pending_ios(entry, status);

	/* 释放 entry */
	merge_entry_remove_and_free(entry);

	/* 减少 inflight 计数，drain 检查 */
	__atomic_fetch_sub(&mctx->inflight_flushes, 1, __ATOMIC_ACQ_REL);
	merge_check_drain(mctx);
}

static int
merge_full_stripe_write(struct merge_entry *entry)
{
	struct poweraid_raid_common_raid *raid = entry->raid;
	struct poweraid_raid_common_io_channel *ch = entry->mod_ch;
	struct poweraid_raid_common_req *req;
	struct raid_bdev_io *carrier;
	struct merge_pending_io *first_pio;
	uint32_t strip_bytes = entry->strip_bytes;
	uint32_t i;

	/* 从 free 池取一个 write stripe_request */
	req = TAILQ_FIRST(&ch->free_write_stripe_requests);
	if (req == NULL) {
		/* 池暂时耗尽：返回 ENOMEM，由 merge_flush_entry 延迟重试（Stage 3c）*/
		SPDK_DEBUGLOG(raid5f_merge, "merge full: no free stripe_request, defer\n");
		return -ENOMEM;
	}
	TAILQ_REMOVE(&ch->free_write_stripe_requests, req, link);

	/* 取第一个 pending_io 的 raid_io 作为 carrier */
	first_pio = TAILQ_FIRST(&entry->pending_ios);
	carrier = first_pio->raid_io;

	/* 设置 carrier 完成回调 */
	carrier->completion_cb = merge_carrier_complete_cb;
	carrier->module_private = entry;

	/* 填充 req */
	req->type = POWERAID_RAID_COMMON_STRIPE_REQ_WRITE;
	req->raid_io = carrier;
	req->eff_raid_ch = entry->eff_raid_ch;
	req->stripe_index = entry->stripe_index;
	req->raid = raid;
	req->io = carrier;  /* IO_COMPLETE 调 raid_bdev_io_complete 用 */

	/* 从缓冲池取 parity 缓冲 */
	req->parity_buf_alloc = poweraid_raid_common_get_parity_buf(ch);
	if (req->parity_buf_alloc == NULL) {
		SPDK_ERRLOG("merge full: alloc parity_buf failed\n");
		goto err_free_req;
	}
	req->parity_buf = req->parity_buf_alloc;

	/* RAID6: 从缓冲池取 Q 缓冲 */
	if (raid->num_parity >= 2) {
		req->q_buf_alloc = poweraid_raid_common_get_q_buf(ch);
		if (req->q_buf_alloc == NULL) {
			SPDK_ERRLOG("merge full: alloc q_buf failed\n");
			goto err_free_parity;
		}
		req->q_buf = req->q_buf_alloc;
	}

	/* 从缓冲池取全 stripe 数据缓冲 */
	req->data_buf = poweraid_raid_common_get_data_buf(ch);
	if (req->data_buf == NULL) {
		SPDK_ERRLOG("merge full: alloc data_buf failed\n");
		goto err_free_q;
	}

	/* 从 chunk_bufs 拷贝到 data_buf（按 data chunk 序号排列）*/
	for (i = 0; i < entry->data_chunks; i++) {
		memcpy((char *)req->data_buf + i * strip_bytes,
		       entry->chunk_bufs[i], strip_bytes);
	}

	/* 构造 src_bufs 数组 */
	req->src_bufs = calloc(entry->data_chunks, sizeof(void *));
	if (req->src_bufs == NULL) {
		SPDK_ERRLOG("merge full: alloc src_bufs failed\n");
		goto err_free_data;
	}
	for (i = 0; i < entry->data_chunks; i++) {
		req->src_bufs[i] = (char *)req->data_buf + i * strip_bytes;
	}
	req->n_src = entry->data_chunks;
	req->xor_len = strip_bytes;

	SPDK_DEBUGLOG(raid5f_merge, "merge full: req=%p stripe=%"PRIu64
		      " carrier=%p\n", req, entry->stripe_index, carrier);

	/* 触发 REQ FSM：ASSIGN → CALC → WRITE_FULL → ... → IO_COMPLETE */
	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_REQ, req,
				   POWERAID_REQ_EV_ASSIGN);
	return 0;

err_free_data:
	poweraid_raid_common_put_data_buf(ch, req->data_buf);
	req->data_buf = NULL;
err_free_q:
	if (req->q_buf_alloc != NULL) {
		poweraid_raid_common_put_q_buf(ch, req->q_buf_alloc);
		req->q_buf_alloc = NULL;
		req->q_buf = NULL;
	}
err_free_parity:
	poweraid_raid_common_put_parity_buf(ch, req->parity_buf_alloc);
	req->parity_buf_alloc = NULL;
	req->parity_buf = NULL;
err_free_req:
	TAILQ_INSERT_HEAD(&ch->free_write_stripe_requests, req, link);
	return -ENOMEM;
}

/* ===== RMW 路径 ===== */

static void
merge_rmw_complete_cb(int status, void *cb_arg)
{
	struct merge_entry *entry = cb_arg;
	struct merge_ctx *mctx = entry->mctx;
	enum spdk_bdev_io_status io_status = (status == 0) ?
		SPDK_BDEV_IO_STATUS_SUCCESS : SPDK_BDEV_IO_STATUS_FAILED;

	SPDK_DEBUGLOG(raid5f_merge, "rmw complete: entry=%p stripe=%"PRIu64
		      " status=%d\n", entry, entry->stripe_index, status);

	merge_complete_pending_ios(entry, io_status);
	merge_entry_remove_and_free(entry);

	__atomic_fetch_sub(&mctx->inflight_flushes, 1, __ATOMIC_ACQ_REL);
	merge_check_drain(mctx);
}

static int
merge_rmw_dispatch(struct merge_entry *entry)
{
	int rc;

	SPDK_DEBUGLOG(raid5f_merge, "rmw dispatch: entry=%p stripe=%"PRIu64
		      " bitmap=0x%"PRIx64"\n", entry, entry->stripe_index,
		      entry->chunk_bitmap);

	rc = poweraid_raid_common_rmw_submit_merged(entry->eff_raid_ch,
					       entry->raid,
					       entry->stripe_index,
					       entry->chunk_bitmap,
					       entry->chunk_bufs,
					       merge_rmw_complete_cb,
					       entry);
	if (rc != 0) {
		SPDK_ERRLOG("rmw dispatch failed rc=%d\n", rc);
	}
	return rc;
}

/* ===== flush entry ===== */

static void
merge_flush_entry(struct merge_entry *entry)
{
	struct merge_ctx *mctx = entry->mctx;
	struct raid_bdev_io_channel *eff_ch;
	enum poweraid_raid_common_gate gate;
	uint32_t num_modified;
	int rc;

	if (entry->flushing) {
		return;
	}

	/* 重建窗口门控：未越过窗口的 stripe 必须等重构完成（引擎通过窗口推进
	 * hook 触发重放，poller 也会周期重试）。此处不标记 flushing/不计 inflight。*/
	gate = poweraid_raid_common_rebuild_gate_classify(entry->raid, entry->raid_ch,
			entry->stripe_index, &eff_ch);
	if (gate == POWERAID_RAID_COMMON_GATE_WAIT) {
		/* pending IOs 已在 bdev 层 io_submitted 链表上（merge_submit 返回 0
		 * 后 raid_io 归 merge 所有，未完成）。若仅 return 不 flush，raid_io
		 * 永不离开 io_submitted → 框架 quiesce drain 死锁（窗口永远锁不住，
		 * 重建引擎永远不被调用）。
		 *
		 * 将 pending IOs 以 NOMEM 交还 bdev 层：
		 * 1. raid_bdev_io_complete(NOMEM) → bdev 层从 io_submitted 摘除 →
		 *    入 nomem 队列（不计 outstanding）
		 * 2. quiesce drain 成功 → 窗口锁定 → 重建引擎启动 → stripe 重构
		 * 3. 重建推进 → unquiesce → nomem 重试的 IO 命中 LBA range lock（已解锁）
		 *    → 提交到模块 → gate PASS（窗口已越过）→ 正常写入 */
		SPDK_DEBUGLOG(raid5f_merge, "flush gated, NOMEM pending ios stripe=%"PRIu64"\n",
			      entry->stripe_index);
		merge_complete_pending_ios(entry, SPDK_BDEV_IO_STATUS_NOMEM);
		merge_entry_remove_and_free(entry);
		merge_check_drain(mctx);
		return;
	}
	entry->eff_raid_ch = eff_ch;

	entry->flushing = true;

	/* 增加 inflight 计数 */
	__atomic_fetch_add(&mctx->inflight_flushes, 1, __ATOMIC_ACQ_REL);

	num_modified = (uint32_t)__builtin_popcountll(entry->chunk_bitmap);

	SPDK_DEBUGLOG(raid5f_merge, "flush entry: stripe=%"PRIu64
		      " bitmap=0x%"PRIx64" num_modified=%u data_chunks=%u\n",
		      entry->stripe_index, entry->chunk_bitmap,
		      num_modified, entry->data_chunks);

	if (num_modified == entry->data_chunks) {
		/* 全 chunk 覆盖 → 全 stripe 写 */
		rc = merge_full_stripe_write(entry);
	} else {
		/* 部分 chunk → RMW */
		rc = merge_rmw_dispatch(entry);
	}

	if (rc != 0) {
		if (rc == -ENOMEM) {
			/* 资源暂时不足（stripe_request 池耗尽等）：保持 entry pending，
			 * 由 poller 200μs 后重试。inflight 写完成后必然释放 req，有进展保证。
			 * 注意不得 fail pending IO——bdev 层对 merge 内部发起的 flush 无重试路径。*/
			entry->flushing = false;
			__atomic_fetch_sub(&mctx->inflight_flushes, 1, __ATOMIC_ACQ_REL);
			SPDK_DEBUGLOG(raid5f_merge, "flush entry deferred (ENOMEM), stripe=%"PRIu64"\n",
				      entry->stripe_index);
			return;
		}
		/* 其他错误：完成所有 pending_io 为 FAILED，释放 entry */
		SPDK_ERRLOG("flush entry failed rc=%d, failing pending ios\n", rc);
		merge_complete_pending_ios(entry, SPDK_BDEV_IO_STATUS_FAILED);
		merge_entry_remove_and_free(entry);
		__atomic_fetch_sub(&mctx->inflight_flushes, 1, __ATOMIC_ACQ_REL);
		merge_check_drain(mctx);
	}
}

/* ===== poller ===== */

static int
merge_poller_fn(void *arg)
{
	struct merge_ctx *mctx = arg;
	struct merge_entry *entry, *tmp;
	uint64_t now = spdk_get_ticks();
	uint64_t delay_ticks = (mctx->delay_us * spdk_get_ticks_hz()) / 1000000ULL;
	int flushed = 0;

	TAILQ_FOREACH_SAFE(entry, &mctx->pending_list, link, tmp) {
		if (entry->flushing) {
			continue;
		}
		if ((now - entry->create_time_ticks) >= delay_ticks) {
			merge_flush_entry(entry);
			flushed++;
		}
	}

	return flushed;  /* 返回有工作量则 >0，SPDK poller 据此判断是否繁忙 */
}

/* ===== drain 检查 ===== */

static void
merge_check_drain(struct merge_ctx *mctx)
{
	if (mctx->pending_flush_io != NULL &&
	    mctx->num_pending == 0 &&
	    __atomic_load_n(&mctx->inflight_flushes, __ATOMIC_ACQUIRE) == 0) {
		struct raid_bdev_io *flush_io = mctx->pending_flush_io;
		mctx->pending_flush_io = NULL;
		raid_bdev_io_complete(flush_io, SPDK_BDEV_IO_STATUS_SUCCESS);
	}
}

/* ===== 公开 API ===== */

int
poweraid_raid_common_merge_init(struct merge_ctx *mctx,
			    struct poweraid_raid_common_raid *raid,
			    struct poweraid_raid_common_io_channel *mod_ch)
{
	memset(mctx, 0, sizeof(*mctx));
	TAILQ_INIT(&mctx->pending_list);
	mctx->raid = raid;
	mctx->mod_ch = mod_ch;
	mctx->delay_us = raid->delay_us;  /* Stage 3c：raid 级配置，RPC 可改 */
	mctx->num_pending = 0;
	mctx->inflight_flushes = 0;
	mctx->pending_flush_io = NULL;

	mctx->poller = SPDK_POLLER_REGISTER(merge_poller_fn, mctx,
					    MERGE_POLLER_PERIOD_US);
	if (mctx->poller == NULL) {
		SPDK_ERRLOG("merge_init: poller register failed\n");
		return -ENOMEM;
	}

	SPDK_DEBUGLOG(raid5f_merge, "merge_init: mctx=%p raid=%p delay_us=%lu\n",
		      mctx, raid, mctx->delay_us);
	return 0;
}

void
poweraid_raid_common_merge_destroy(struct merge_ctx *mctx)
{
	struct merge_entry *entry;
	struct merge_pending_io *pio;

	SPDK_DEBUGLOG(raid5f_merge, "merge_destroy: mctx=%p num_pending=%u\n",
		      mctx, mctx->num_pending);

	if (mctx->poller != NULL) {
		spdk_poller_unregister(&mctx->poller);
	}

	/* fail 所有 pending entries */
	while ((entry = TAILQ_FIRST(&mctx->pending_list)) != NULL) {
		TAILQ_REMOVE(&mctx->pending_list, entry, link);
		mctx->num_pending--;

		/* 完成所有 pending_ios 为 FAILED */
		while ((pio = TAILQ_FIRST(&entry->pending_ios)) != NULL) {
			struct raid_bdev_io *raid_io = pio->raid_io;

			TAILQ_REMOVE(&entry->pending_ios, pio, link);
			free(pio);
			raid_io->completion_cb = NULL;
			raid_io->module_private = NULL;
			raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		}
		merge_entry_free(entry);
	}

	/* 若有等待 drain 的 flush IO，完成它 */
	if (mctx->pending_flush_io != NULL) {
		struct raid_bdev_io *flush_io = mctx->pending_flush_io;
		mctx->pending_flush_io = NULL;
		raid_bdev_io_complete(flush_io, SPDK_BDEV_IO_STATUS_SUCCESS);
	}
}

int
poweraid_raid_common_merge_submit(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	struct poweraid_raid_common_io_channel *ch;
	uint32_t data_chunks = raid->num_base_bdevs - raid->num_parity;
	uint32_t stripe_blocks = raid->strip_size * data_chunks;
	uint64_t stripe_index = raid_io->offset_blocks / stripe_blocks;
	uint64_t stripe_offset = raid_io->offset_blocks % stripe_blocks;
	uint8_t chunk_data_idx = stripe_offset >> raid_bdev->strip_size_shift;
	uint32_t strip_bytes = raid->strip_size * raid->block_size;
	struct merge_ctx *mctx;
	struct merge_entry *entry;
	struct merge_pending_io *pio;
	int rc;

	ch = raid_bdev_channel_get_module_ctx(raid_io->raid_ch);
	if (ch == NULL) {
		SPDK_ERRLOG("merge_submit: no module channel\n");
		return -ENODEV;
	}
	mctx = &ch->merge_ctx;

	/* 重建窗口门控必须在吸收 raid_io 之前：一旦插入 pending entry，raid_io
	 * 所有权转给 merge，在 flush 完成前一直是模块在途 IO。框架 quiesce
	 * 窗口要等在途 IO 全部 drain 才放重建引擎进来，而 flush 又在等窗口推进
	 * —— 内部持有的 gated IO 会与之死锁（quiesce 回调永不触发）。
	 * 故这里直接 -ENOMEM，把 IO 交还 bdev 层 nomem 队列（不计 outstanding，
	 * 且会被 LBA range lock 自动挂起/解锁后重投），窗口推进后自然重试。*/
	{
		struct raid_bdev_io_channel *eff_ch;
		enum poweraid_raid_common_gate gate =
			poweraid_raid_common_rebuild_gate_classify(raid, raid_io->raid_ch,
					stripe_index, &eff_ch);
		if (gate == POWERAID_RAID_COMMON_GATE_WAIT) {
			return -ENOMEM;
		}
	}

	/* 防御性校验：strip 对齐 */
	if (stripe_offset % raid->strip_size != 0 ||
	    raid_io->num_blocks % raid->strip_size != 0) {
		SPDK_ERRLOG("merge_submit: not strip-aligned offset=%"PRIu64
			    " num=%"PRIu64"\n", raid_io->offset_blocks, raid_io->num_blocks);
		return -EINVAL;
	}

	/* 分配 pending_io */
	pio = calloc(1, sizeof(*pio));
	if (pio == NULL) {
		return -ENOMEM;
	}
	pio->raid_io = raid_io;

	/* 线性扫描查找同 stripe entry */
	TAILQ_FOREACH(entry, &mctx->pending_list, link) {
		if (entry->stripe_index == stripe_index && !entry->flushing) {
			break;
		}
	}

	if (entry == NULL) {
		/* 检查 pending 上限 */
		if (mctx->num_pending >= MERGE_MAX_PENDING) {
			/* 超限：直接走 RMW 单 strip 路径（不合并）*/
			free(pio);
			return poweraid_raid_common_rmw_submit(raid_io);
		}

		/* 创建新 entry */
		entry = merge_entry_alloc(mctx, raid_io->raid_ch, ch, raid, stripe_index);
		if (entry == NULL) {
			free(pio);
			return -ENOMEM;
		}
		TAILQ_INSERT_TAIL(&mctx->pending_list, entry, link);
		mctx->num_pending++;
	}

	/* 拷贝 raid_io 数据到 chunk_bufs[chunk_data_idx] */
	rc = (int)spdk_iovcpy(raid_io->iovs, raid_io->iovcnt,
		 &(struct iovec){
			.iov_base = entry->chunk_bufs[chunk_data_idx],
			.iov_len = strip_bytes,
		 }, 1);
	if (rc != (int)strip_bytes) {
		SPDK_ERRLOG("merge_submit: iovcpy short copied=%d expect=%u\n",
			    rc, strip_bytes);
		/* 若 entry 是新建的（只有这一个 IO），移除并释放 */
		if (TAILQ_EMPTY(&entry->pending_ios)) {
			merge_entry_remove_and_free(entry);
		}
		free(pio);
		return -EIO;
	}

	/* 更新 bitmap */
	entry->chunk_bitmap |= (1ULL << chunk_data_idx);

	/* 挂载 pending_io */
	TAILQ_INSERT_TAIL(&entry->pending_ios, pio, link);

	SPDK_DEBUGLOG(raid5f_merge, "submit: stripe=%"PRIu64" chunk=%u"
		      " bitmap=0x%"PRIx64" num_pending=%u\n",
		      stripe_index, chunk_data_idx, entry->chunk_bitmap,
		      (uint32_t)__builtin_popcountll(entry->chunk_bitmap));

	/* 全 chunk 覆盖 → 立即 flush；delay=0（merge OFF）→ 每个 IO 立即 flush（纯 RMW 基线）*/
	if ((uint32_t)__builtin_popcountll(entry->chunk_bitmap) == entry->data_chunks ||
	    mctx->delay_us == 0) {
		merge_flush_entry(entry);
	}

	return 0;
}

void
poweraid_raid_common_merge_flush_all(struct raid_bdev_io *raid_io)
{
	struct poweraid_raid_common_io_channel *ch;
	struct merge_ctx *mctx;
	struct merge_entry *entry, *tmp;

	ch = raid_bdev_channel_get_module_ctx(raid_io->raid_ch);
	if (ch == NULL) {
		SPDK_ERRLOG("merge_flush_all: no module channel\n");
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}
	mctx = &ch->merge_ctx;

	SPDK_DEBUGLOG(raid5f_merge, "flush_all: num_pending=%u inflight=%d\n",
		      mctx->num_pending,
		      __atomic_load_n(&mctx->inflight_flushes, __ATOMIC_ACQUIRE));

	/* flush 所有 pending entries */
	TAILQ_FOREACH_SAFE(entry, &mctx->pending_list, link, tmp) {
		if (!entry->flushing) {
			merge_flush_entry(entry);
		}
	}

	/* 若仍有 inflight flush 或 pending entries，排队 flush IO 等 drain */
	if (__atomic_load_n(&mctx->inflight_flushes, __ATOMIC_ACQUIRE) > 0 ||
	    mctx->num_pending > 0) {
		mctx->pending_flush_io = raid_io;
	} else {
		/* 无 pending，立即完成 flush */
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_SUCCESS);
	}
}

void
poweraid_raid_common_merge_replay_gated(struct poweraid_raid_common_io_channel *mod_ch)
{
	struct merge_ctx *mctx = &mod_ch->merge_ctx;
	struct merge_entry *entry, *tmp;

	/* 被重建窗口门控的条目重新分类并尝试提交；仍被门控的留在 pending 列表。
	 * 窗口结束（ended=true）时 shadow 为 NULL，所有条目必然放行。*/
	TAILQ_FOREACH_SAFE(entry, &mctx->pending_list, link, tmp) {
		if (!entry->flushing) {
			merge_flush_entry(entry);
		}
	}
}
