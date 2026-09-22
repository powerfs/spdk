/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   PPL（Partial Parity Log）—— write hole 根治方案
 *
 *   详见 raid5f-enhanced-design.md 第 3.1 节（设计）与第 3.7 节（FUA barrier 序列）。
 *
 *   落盘布局（每盘 MD 区前 32 MiB 内，PPL 子区由 sb v2 ext 的 ppl_region_offset/size 描述）：
 *     [PPL super(64B)] [record0(64B)] [record1(64B)] ... [recordN(64B)]
 *
 *   阶段 2 简化策略（正确性优先，性能优化延后）：
 *     - 每条 record 写一个 block_size 对齐的 slot（record 置于 slot 起始 64B，余下清零），
 *       保证单 record 独立 FUA 落盘，无需读改写。
 *     - append = spdk_bdev_write_blocks + spdk_bdev_flush_blocks（等价 FUA，bdev 无关）。
 *     - commit = 更新 super.commit_seq + 写 super slot + FUA。
 *     - 日志槽位按 block_size 步进，最大 record 数 = region_size / block_size。
 *       4 MiB / 512B = 8192；4 MiB / 4096B = 1024（阶段 2 测试足够）。
 *     - 满一轮后回收已 commit 的 slot（head 前进）。
 *     - backpressure（issue #14）：日志满且有 in-flight 未 commit 时，append 请求入队等待，
 *       commit 释放 slot 后自动唤醒重发，不再降级写无 PPL 保护。
 *     - 并发安全：slot / seq 在 append_record 入口同步预占，避免异步写完成前并发 append 拿到相同 slot。
 *   TODO（后续阶段）：按 64B 紧凑打包 record + 整块缓存批量 FUA，降低写放大。
 */

#ifndef POWERAID_RAID_COMMON_PPL_H
#define POWERAID_RAID_COMMON_PPL_H

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 常量 ===== */

#define POWERAID_RAID_COMMON_PPL_REC_MAGIC    0x504C5052u  /* "PPLR" little-endian */
#define POWERAID_RAID_COMMON_PPL_SUPER_MAGIC  0x50505352u  /* "PPSR" little-endian */

/* PPL super 大小（与 record 一致，64B）*/
#define POWERAID_RAID_COMMON_PPL_SUPER_SIZE   64
/* PPL record 大小 */
#define POWERAID_RAID_COMMON_PPL_REC_SIZE     64

/* 默认 PPL 区几何（与 sb v2 ext 默认值一致，可被 sb ext 覆盖）*/
#define POWERAID_RAID_COMMON_PPL_DEFAULT_REGION_OFFSET  4096ULL          /* 4 KiB */
#define POWERAID_RAID_COMMON_PPL_DEFAULT_REGION_SIZE    (4ULL * 1024 * 1024)  /* 4 MiB */

/* record flags */
#define POWERAID_RAID_COMMON_PPL_REC_F_VALID     (1u << 0)  /* record 有效（magic+crc 通过）*/
#define POWERAID_RAID_COMMON_PPL_REC_F_COMMITTED (1u << 1)  /* data+parity 已写并 flush（用于 in-place 标记，阶段 2 用 super.commit_seq 判定，此 flag 备用）*/

/* ===== 落盘结构 ===== */

/*
 * PPL record（64B），对应 MD 设计 L120。
 * data_hash[16] 拆为 old_data_hash(8) + new_data_hash(8)（各 64-bit hash），
 * 支持 MD L125-130 三分支恢复：data 匹配 new → 补 parity；匹配 old → 跳过；都不符 → scrub。
 */
struct poweraid_raid_common_ppl_record {
	uint32_t	magic;            /* POWERAID_RAID_COMMON_PPL_REC_MAGIC */
	uint32_t	flags;             /* POWERAID_RAID_COMMON_PPL_REC_F_* */
	uint64_t	seq;               /* 全局单调递增序号（由 PPL super.next_seq 分配）*/
	uint64_t	stripe_id;         /* 目标 stripe */
	uint64_t	chunk_bitmap;      /* 本次写涉及的 data chunk 位图（bit i = chunk i）*/
	uint64_t	old_data_hash;     /* 旧 data 的 64-bit hash（XOR 各涉及的 chunk）*/
	uint64_t	new_data_hash;     /* 新 data 的 64-bit hash */
	uint32_t	crc32c;            /* 覆盖本 record 除 crc32c 字段外全部字节 */
	uint32_t	reserved;          /* 预留对齐 */
	uint64_t	ts;                /* 时间戳（unix 秒，time(NULL)）*/
};
SPDK_STATIC_ASSERT(sizeof(struct poweraid_raid_common_ppl_record) ==
		   POWERAID_RAID_COMMON_PPL_REC_SIZE, "incorrect ppl record size");

/*
 * PPL super（64B），位于 PPL 区起始。
 * head_seq / tail_seq / commit_seq 均为 record seq（非槽位下标）。
 */
struct poweraid_raid_common_ppl_super {
	uint32_t	magic;            /* POWERAID_RAID_COMMON_PPL_SUPER_MAGIC */
	uint32_t	version;          /* 1 */
	uint64_t	head_seq;         /* 日志中最旧有效 record 的 seq（回收起点）*/
	uint64_t	tail_seq;         /* 下一条待写 record 的 seq（== next_seq 快照）*/
	uint64_t	commit_seq;       /* 已 commit 的最大 record seq（commit_seq <= tail_seq）*/
	uint64_t	next_seq;         /* 下一条 record 的 seq（单调递增，回卷时由 seq+1 自然溢出处理）*/
	uint32_t	crc32c;           /* 覆盖本 super 除 crc32c 字段外全部字节 */
	uint32_t	num_records;      /* 当前日志中有效 record 数（tail - head）*/
	uint64_t	reserved[2];      /* 预留对齐，凑满 64B */
};
SPDK_STATIC_ASSERT(sizeof(struct poweraid_raid_common_ppl_super) ==
		   POWERAID_RAID_COMMON_PPL_SUPER_SIZE, "incorrect ppl super size");

/* ===== 上下文（opaque）===== */
struct poweraid_raid_common_ppl_ctx;
struct poweraid_raid_common_ppl_gc;

/* in-flight record（已 append，待 commit），挂在 ctx->inflight 链表 */
struct poweraid_raid_common_ppl_inflight {
	uint64_t				seq;
	uint64_t				stripe_id;
	TAILQ_ENTRY(poweraid_raid_common_ppl_inflight)	link;
};

/* ===== 回调原型 ===== */
typedef void (*poweraid_raid_common_ppl_init_cb)(int status, void *cb_arg);
typedef void (*poweraid_raid_common_ppl_append_cb)(int status, uint64_t seq, void *cb_arg);
typedef void (*poweraid_raid_common_ppl_commit_cb)(int status, void *cb_arg);

/*
 * recovery 回调：load_replay 完成后返回按 seq 升序排列的未 commit record 数组。
 * 调用方负责 free(records)（用 poweraid_raid_common_ppl_free_replay_result）。
 * status==0 且 records!=NULL 时 num_records>0；status!=0 时 records 为 NULL。
 */
typedef void (*poweraid_raid_common_ppl_replay_cb)(int status,
		struct poweraid_raid_common_ppl_record *records, uint32_t num_records,
		void *cb_arg);

/* data flush 完成回调（group commit 阶段 C1）*/
typedef void (*poweraid_raid_common_ppl_data_flush_cb)(int status, void *cb_arg);

/* ===== Group Commit 协调器（阶段 C1）=====
 *
 * 三处屏障批合并，不改盘上格式：
 *   1. append group：K 条 record 流水线写，1 次 range flush
 *   2. data flush group：K 个 stripe 的 N 盘 flush 合并为每盘 1 次 range flush
 *   3. commit group：K 个 commit 合并为 1 次 super 写+flush
 *
 * 并发模型：per-thread gc_ctx（挂 io_channel），无锁访问；
 * 共享 ppl_ctx 的 slot/seq/super 用 slot_lock 保护。
 * idle 快路径：仅 1 个 pending 且无 inflight → 立即 flush，d1 无回归。
 */

#define POWERAID_RAID_COMMON_PPL_GC_K           32    /* 批合并阈值，与 iodepth 匹配 */
#define POWERAID_RAID_COMMON_PPL_GC_T_APPEND_US 100   /* append group 超时触发（µs）*/
#define POWERAID_RAID_COMMON_PPL_GC_T_DATA_US   50    /* data flush group 超时（µs）*/
#define POWERAID_RAID_COMMON_PPL_GC_T_COMMIT_US 50    /* commit group 超时（µs）*/
#define POWERAID_RAID_COMMON_PPL_GC_POLLER_US   50    /* poller 周期（µs）*/
#define POWERAID_RAID_COMMON_PPL_GC_MAX_BDEVS   16    /* data flush 按盘聚合最大盘数 */

/* per-disk data flush range（IO 写过的盘区间）*/
struct poweraid_raid_common_ppl_data_range {
	uint8_t				phys;   /* 物理盘索引 */
	uint64_t			offset; /* stripe 相对偏移（blocks）*/
	uint64_t			length; /* 长度（blocks）*/
};

/* group commit entry：贯穿 append → data_flush → commit 三阶段 */
struct poweraid_raid_common_ppl_gc_entry {
	uint64_t			seq;
	uint32_t			slot;
	uint64_t			slot_byte_off;
	uint64_t			stripe_id;
	uint64_t			chunk_bitmap;
	uint64_t			old_data_hash;
	uint64_t			new_data_hash;
	struct spdk_io_channel		*ch;  /* 调用方线程 channel（append write/flush 用）*/

	/* append 阶段 */
	struct poweraid_raid_common_ppl_gc	*gc;  /* 所属 group commit 协调器 */
	poweraid_raid_common_ppl_append_cb	append_cb;
	void				*append_cb_arg;
	uint64_t			append_enqueue_ticks;
	bool				append_write_failed;
	bool				waiting_for_slot;  /* PPL 满，等待 slot 释放 */
	void				*append_buf;  /* DMA scratch for record write */

	/* data flush 阶段 */
	struct poweraid_raid_common_ppl_data_range	ranges[POWERAID_RAID_COMMON_PPL_GC_MAX_BDEVS];
	uint32_t			num_ranges;
	poweraid_raid_common_ppl_data_flush_cb	data_flush_cb;
	void				*data_flush_cb_arg;
	uint64_t			data_flush_enqueue_ticks;

	/* commit 阶段 */
	poweraid_raid_common_ppl_commit_cb	commit_cb;
	void				*commit_cb_arg;

	TAILQ_ENTRY(poweraid_raid_common_ppl_gc_entry)	link;
};

/* per-thread group commit 协调器（嵌入 io_channel）*/
struct poweraid_raid_common_ppl_gc {
	struct poweraid_raid_common_ppl_ctx	*ppl_ctx;
	struct spdk_poller			*poller;

	/* opaque raid 引用：data flush 阶段发 per-disk range flush 用。
	 * raid_bdev = struct raid_bdev*（base_bdev_info[] 来源）
	 * raid_ch   = struct raid_bdev_io_channel*（base_channel[] 来源）
	 * 用 void* 避免 ppl.h 依赖 bdev_raid.h，ppl.c 内部强转。*/
	void					*raid_bdev;
	void					*raid_ch;

	/* append group */
	TAILQ_HEAD(, poweraid_raid_common_ppl_gc_entry)	append_pending;
	uint32_t				append_pending_count;
	bool					append_inflight;
	uint64_t				append_oldest_ticks;
	/* 已入队但 record 写尚未提交的 entry 数（waiting_for_slot 场景）。
	 * append flush 组必须等组内所有 record 写都完成后才能发出，
	 * 否则在无 FUA/flush 能力的盘上 flush 会立即完成并提前释放 append_buf，
	 * 与仍在途的 record 写形成 UAF（载荷被回收内存覆盖，record 落盘全零，
	 * 且破坏 record 先于数据持久化的顺序保证）。*/
	uint32_t				append_unsubmitted_count;
	/* 已提交但尚未完成的 record 写数量 */
	uint32_t				append_write_inflight;

	/* data flush group */
	TAILQ_HEAD(, poweraid_raid_common_ppl_gc_entry)	data_flush_pending;
	uint32_t				data_flush_pending_count;
	bool					data_flush_inflight;
	uint64_t				data_flush_oldest_ticks;
	struct {
		uint64_t			min_off;
		uint64_t			max_off;
		bool				active;
	} data_flush_ranges[POWERAID_RAID_COMMON_PPL_GC_MAX_BDEVS];
	uint32_t				data_flush_remaining;
	bool					data_flush_group_failed;

	/* commit group */
	TAILQ_HEAD(, poweraid_raid_common_ppl_gc_entry)	commit_pending;
	uint32_t				commit_pending_count;
	bool					commit_inflight;
	uint64_t				commit_oldest_ticks;
};

/* ===== API ===== */

/**
 * 为单个 base_bdev 分配 PPL 上下文。
 * region_offset/region_size 来自 sb v2 ext（默认 4096 / 4 MiB）。
 * block_size 为 base_bdev 块大小（决定 record slot 步进）。
 */
struct poweraid_raid_common_ppl_ctx *poweraid_raid_common_ppl_alloc(
	void *bdev_desc, struct spdk_io_channel *ch,
	uint32_t block_size,
	uint64_t region_offset, uint64_t region_size);

/**
 * 释放 PPL 上下文（不含磁盘操作，仅内存）。
 */
void poweraid_raid_common_ppl_free(struct poweraid_raid_common_ppl_ctx *ctx);

/**
 * 初始化磁盘 PPL 区：写 super（head=tail=commit=next_seq=1）+ FUA。
 * 用于新卷创建或首次格式化。异步。
 */
void poweraid_raid_common_ppl_init(struct poweraid_raid_common_ppl_ctx *ctx,
			      poweraid_raid_common_ppl_init_cb cb, void *cb_arg);

/**
 * 追加一条 PPL record（FUA 落盘，保证 intend 持久后再写 data）。
 * seq 由 ctx 内部从 super.next_seq 分配并回传。
 * 对应写路径第 1 步（MD L273）。
 */
void poweraid_raid_common_ppl_append_record(
	struct poweraid_raid_common_ppl_ctx *ctx, struct spdk_io_channel *ch,
	uint64_t stripe_id, uint64_t chunk_bitmap,
	uint64_t old_data_hash, uint64_t new_data_hash,
	poweraid_raid_common_ppl_append_cb cb, void *cb_arg);

/**
 * commit 到指定 seq（含）：更新 super.commit_seq + FUA。
 * 对应写路径第 5 步（MD L277）。data+parity 已 flush 后调用。
 */
void poweraid_raid_common_ppl_commit(struct poweraid_raid_common_ppl_ctx *ctx,
			       struct spdk_io_channel *ch, uint64_t seq,
			       poweraid_raid_common_ppl_commit_cb cb, void *cb_arg);

/**
 * 启动时扫描 PPL 区，按 seq 升序返回未 commit（seq > commit_seq）的有效 record。
 * 对应恢复流程 MD L126-130。调用方用 old/new hash 三分支判定后处理。
 * 子任务 C 串联调用。
 */
void poweraid_raid_common_ppl_load_replay(struct poweraid_raid_common_ppl_ctx *ctx,
				     poweraid_raid_common_ppl_replay_cb cb, void *cb_arg);

/**
 * 释放 load_replay 回调返回的 record 数组。
 */
void poweraid_raid_common_ppl_free_replay_result(struct poweraid_raid_common_ppl_record *records);

/**
 * 数据块 64-bit hash（写路径与恢复路径共用，保证一致）。
 * 实现：两段 crc32c（不同 seed）拼成 64-bit，碰撞率足够低。
 * - 写路径（子任务 D）：append 前对旧 data / 新 data 各算一次填入 record。
 * - 恢复路径（子任务 C）：读当前 data 算一次，与 old/new 比对定分支。
 */
uint64_t poweraid_raid_common_ppl_data_hash(const void *buf, size_t len);

/**
 * 增量 hash 上下文：多 chunk 数据须用同一上下文逐段 update，
 * final 得到的 64-bit 与一次性 data_hash(全量拼接) 等价。
 * 写路径按 chunk 顺序 update（old 读盘顺序 / new 写缓冲顺序需一致）。
 */
struct poweraid_raid_common_ppl_hash_ctx {
	uint32_t	lo;
	uint32_t	hi;
};
void poweraid_raid_common_ppl_hash_init(struct poweraid_raid_common_ppl_hash_ctx *c);
void poweraid_raid_common_ppl_hash_update(struct poweraid_raid_common_ppl_hash_ctx *c,
				     const void *buf, size_t len);
uint64_t poweraid_raid_common_ppl_hash_final(struct poweraid_raid_common_ppl_hash_ctx *c);

/* ===== Group Commit API（阶段 C1）===== */

/**
 * 初始化 per-thread gc_ctx（ioch_create 调用）。
 * 注册 poller，初始化 TAILQ。
 * raid_bdev / raid_ch 为 opaque 指针（data flush per-disk flush 用）。
 */
int poweraid_raid_common_ppl_gc_init(struct poweraid_raid_common_ppl_gc *gc,
				  struct poweraid_raid_common_ppl_ctx *ppl_ctx,
				  void *raid_bdev, void *raid_ch);

/**
 * 销毁 gc_ctx（ioch_destroy 调用）。
 * 注销 poller，fail 所有 pending entry。
 */
void poweraid_raid_common_ppl_gc_destroy(struct poweraid_raid_common_ppl_gc *gc);

/**
 * Group commit append：替代 ppl_append_record 的逐条 flush。
 * - 分配 entry（*entry 返回给调用方，贯穿后续 data_flush/commit 阶段）
 * - spinlock reserve slot/seq，issue write（不 flush），入队 append_pending
 * - 触发条件满足时 range flush，完成后 inline 调 cb
 * - idle 快路径：仅 1 pending 且无 inflight → 立即 flush（d1 无回归）
 *
 * 注意：cb 在 entry 完成 append flush 后被调用（可能 inline 也可能 poller 触发后）。
 * 调用方在 cb 中开始 data writes，完成后调 ppl_gc_data_flush_enqueue。
 */
void poweraid_raid_common_ppl_gc_append(
	struct poweraid_raid_common_ppl_ctx *ctx,
	struct poweraid_raid_common_ppl_gc *gc,
	struct spdk_io_channel *ch,
	uint64_t stripe_id, uint64_t chunk_bitmap,
	uint64_t old_data_hash, uint64_t new_data_hash,
	struct poweraid_raid_common_ppl_gc_entry **entry,
	poweraid_raid_common_ppl_append_cb cb, void *cb_arg);

/**
 * Group commit data flush enqueue：替代 rmw_start_flushes / req_start_flushes。
 * - entry 为 ppl_gc_append 返回的同一 entry
 * - ranges 描述本 IO 写过的盘区间（write_full=全部 N 盘，RMW=n_modified+num_parity 盘）
 * - 入队 data_flush_pending，按盘聚合 min/max offset
 * - 触发条件满足时每盘 1 次 range flush，全部完成后 inline 调 cb
 */
void poweraid_raid_common_ppl_gc_data_flush_enqueue(
	struct poweraid_raid_common_ppl_gc *gc,
	struct poweraid_raid_common_ppl_gc_entry *entry,
	const struct poweraid_raid_common_ppl_data_range *ranges,
	uint32_t num_ranges,
	poweraid_raid_common_ppl_data_flush_cb cb, void *cb_arg);

/**
 * Group commit commit：替代 ppl_commit 的逐条 super 写+flush。
 * - entry 为同一 entry（携带 append 分配的 seq）
 * - 入队 commit_pending
 * - 触发条件满足时 slot_lock 取锁、commit_seq=max、1 次 super 写+flush、释放锁
 * - 完成后 inline 调 cb，entry 被 gc 释放
 */
void poweraid_raid_common_ppl_gc_commit(
	struct poweraid_raid_common_ppl_ctx *ctx,
	struct poweraid_raid_common_ppl_gc *gc,
	struct spdk_io_channel *ch,
	struct poweraid_raid_common_ppl_gc_entry *entry,
	poweraid_raid_common_ppl_commit_cb cb, void *cb_arg);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID_COMMON_PPL_H */
