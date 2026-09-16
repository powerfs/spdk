/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   合并层 + 延迟合并实现（Stage 3b）
 *
 *   收集同一 stripe 的多个部分 stripe 写，凑齐后一次性提交：
 *     - 全 chunk 覆盖 → 全 stripe 写路径（carrier approach + REQ FSM）
 *     - 部分 chunk 覆盖 → RMW（poweraid_raid5f_rmw_submit_merged）
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
#include "poweraid_raid5f.h"
#include "poweraid_raid5f_merge.h"
#include "poweraid_raid5f_rmw.h"

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
		   struct poweraid_raid5f_io_channel *mod_ch,
		   struct poweraid_raid5f_raid *raid,
		   uint64_t stripe_index)
{
	uint32_t data_chunks = raid->num_base_bdevs - 1;
	uint8_t p_idx = data_chunks - (stripe_index % raid->num_base_bdevs);
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
	struct poweraid_raid5f_raid *raid = entry->raid;
	struct poweraid_raid5f_io_channel *ch = entry->mod_ch;
	struct poweraid_raid5f_req *req;
	struct raid_bdev_io *carrier;
	struct merge_pending_io *first_pio;
	uint32_t strip_bytes = entry->strip_bytes;
	uint32_t stripe_bytes = strip_bytes * entry->data_chunks;
	uint32_t i;

	/* 从 free 池取一个 write stripe_request */
	req = TAILQ_FIRST(&ch->free_write_stripe_requests);
	if (req == NULL) {
		SPDK_ERRLOG("merge full: no free stripe_request\n");
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
	req->type = POWERAID_RAID5F_STRIPE_REQ_WRITE;
	req->raid_io = carrier;
	req->stripe_index = entry->stripe_index;
	req->raid = raid;
	req->io = carrier;  /* IO_COMPLETE 调 raid_bdev_io_complete 用 */

	/* 分配 parity 缓冲 */
	req->parity_buf_alloc = spdk_dma_malloc(strip_bytes, 0, NULL);
	if (req->parity_buf_alloc == NULL) {
		SPDK_ERRLOG("merge full: alloc parity_buf failed\n");
		goto err_free_req;
	}
	req->parity_buf = req->parity_buf_alloc;

	/* 分配全 stripe 数据缓冲 */
	req->data_buf = spdk_dma_malloc(stripe_bytes, 0, NULL);
	if (req->data_buf == NULL) {
		SPDK_ERRLOG("merge full: alloc data_buf failed\n");
		goto err_free_parity;
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
	poweraid_raid5f_sm_process(POWERAID_FSM_LAYER_REQ, req,
				   POWERAID_REQ_EV_ASSIGN);
	return 0;

err_free_data:
	spdk_dma_free(req->data_buf);
	req->data_buf = NULL;
err_free_parity:
	spdk_dma_free(req->parity_buf_alloc);
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

	rc = poweraid_raid5f_rmw_submit_merged(entry->raid_ch,
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
	uint32_t num_modified;
	int rc;

	if (entry->flushing) {
		return;
	}
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
		/* flush 启动失败：完成所有 pending_io 为 FAILED，释放 entry */
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
poweraid_raid5f_merge_init(struct merge_ctx *mctx,
			    struct poweraid_raid5f_raid *raid,
			    struct poweraid_raid5f_io_channel *mod_ch)
{
	memset(mctx, 0, sizeof(*mctx));
	TAILQ_INIT(&mctx->pending_list);
	mctx->raid = raid;
	mctx->mod_ch = mod_ch;
	mctx->delay_us = MERGE_DELAY_US_DEFAULT;
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
poweraid_raid5f_merge_destroy(struct merge_ctx *mctx)
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
poweraid_raid5f_merge_submit(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid5f_raid *raid = raid_bdev->module_private;
	struct poweraid_raid5f_io_channel *ch;
	uint32_t data_chunks = raid->num_base_bdevs - 1;
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
			return poweraid_raid5f_rmw_submit(raid_io);
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

	/* 全 chunk 覆盖 → 立即 flush */
	if ((uint32_t)__builtin_popcountll(entry->chunk_bitmap) == entry->data_chunks) {
		merge_flush_entry(entry);
	}

	return 0;
}

void
poweraid_raid5f_merge_flush_all(struct raid_bdev_io *raid_io)
{
	struct poweraid_raid5f_io_channel *ch;
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
