/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   合并层 + 延迟合并（Stage 3b）
 *
 *   收集同一 stripe 的多个部分 stripe 写，凑齐后一次性提交：
 *     - 全 chunk 覆盖 → 全 stripe 写路径（无 RMW）
 *     - 部分 chunk 覆盖 → RMW（合并后的 bitmap）
 *
 *   收集阶段数据仅在内存中，不写 PPL。最终写路径（全 stripe 或 RMW）包含 PPL。
 *   断电时收集中的数据丢失，磁盘数据一致（无残留 PPL record）。
 *
 *   Per-IO-channel TAILQ + 线性扫描（SPDK 无 rbtree，IO 深度有界）。
 *   Poller 200μs 扫描，超时 1ms flush。
 *
 *   详见 raid5f-enhanced-design.md 第 3.2 节（阶段 3b）。
 */

#ifndef POWERAID_RAID5F_MERGE_H
#define POWERAID_RAID5F_MERGE_H

#include "spdk/stdinc.h"
#include "spdk/queue.h"
#include "spdk/thread.h"

struct raid_bdev_io;
struct raid_bdev_io_channel;
struct poweraid_raid5f_raid;
struct poweraid_raid5f_io_channel;

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 常量 ===== */
#define MERGE_DELAY_US_DEFAULT     1000  /* 默认延迟 flush 阈值（1ms）*/
#define MERGE_POLLER_PERIOD_US     200   /* poller 扫描周期（200μs）*/
#define MERGE_MAX_PENDING          64    /* 每 channel 最大 pending entry 数 */

/* ===== 数据结构 ===== */

/* 挂在 merge_entry->pending_ios 上的待完成 IO */
struct merge_pending_io {
	struct raid_bdev_io            *raid_io;
	TAILQ_ENTRY(merge_pending_io)   link;
};

/* Per-stripe 合并条目 */
struct merge_entry {
	struct poweraid_raid5f_raid     *raid;
	struct raid_bdev_io_channel     *raid_ch;      /* 框架 IO channel */
	struct poweraid_raid5f_io_channel *mod_ch;     /* 模块 IO channel（free 池用）*/
	struct merge_ctx               *mctx;          /* 回指，drain 检查用 */
	uint64_t                         stripe_index;
	uint8_t                          p_idx;
	uint32_t                         data_chunks;
	uint32_t                         strip_bytes;

	uint64_t                         chunk_bitmap;  /* bit i = chunk i 已有数据 */
	void                           **chunk_bufs;    /* [data_chunks]，NULL = 未写 */

	uint64_t                         create_time_ticks;  /* spdk_get_ticks() */
	bool                             flushing;

	TAILQ_HEAD(, merge_pending_io)   pending_ios;
	TAILQ_ENTRY(merge_entry)         link;
};

/* Per-IO-channel 合并上下文 */
struct merge_ctx {
	struct poweraid_raid5f_raid     *raid;
	struct poweraid_raid5f_io_channel *mod_ch;

	TAILQ_HEAD(, merge_entry)        pending_list;
	uint32_t                         num_pending;

	struct spdk_poller              *poller;
	uint64_t                         delay_us;      /* 默认 1000（1ms）*/

	int32_t                          inflight_flushes; /* 原子，正在 flush 的 entry 数 */
	struct raid_bdev_io             *pending_flush_io; /* 等待 drain 的 flush IO */
};

/* ===== API ===== */

/**
 * 初始化 merge_ctx（ioch_create 调用）。
 * 注册 poller，初始化 TAILQ。
 */
int poweraid_raid5f_merge_init(struct merge_ctx *mctx,
			       struct poweraid_raid5f_raid *raid,
			       struct poweraid_raid5f_io_channel *mod_ch);

/**
 * 销毁 merge_ctx（ioch_destroy 调用）。
 * 注销 poller，fail 所有 pending IO。
 */
void poweraid_raid5f_merge_destroy(struct merge_ctx *mctx);

/**
 * 提交一个部分 stripe 写到合并层。
 * raid_io 几何由 submit_rw_request 校验（strip 对齐，不跨 stripe）。
 * 异步完成：通过 raid_bdev_io_complete 通知框架。
 * 返回 0 表示已接受，非 0 表示立即失败（调用方负责 complete）。
 */
int poweraid_raid5f_merge_submit(struct raid_bdev_io *raid_io);

/**
 * Flush 所有 pending entry（FLUSH IO 到达时调用）。
 * 若有 inflight flush 则排队 flush IO，等 drain 后完成。
 */
void poweraid_raid5f_merge_flush_all(struct raid_bdev_io *raid_io);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID5F_MERGE_H */
