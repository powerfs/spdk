/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   MWL（Mirror Write Log）—— raid1f 意图日志
 *
 *   详见 spec.md AC-3 / AC-10 与 raid1f-mirror-write-log 设计。
 *
 *   落盘布局（每盘 MD 区内，MWL 子区由 sb v2 ext 的 mwl_region_offset/size 描述；
 *   默认与 PPL 同构：1MiB 起 4MiB）：
 *     [MWL super(64B)] [record0(64B)] [record1(64B)] ... [recordN(64B)]
 *
 *   纪律（与 PPL 对齐，正确性优先，性能优化延后）：
 *     - 每条 record 占一个 block_size 对齐 slot（record 64B 置于 slot 起始，余清零），
 *       单 record 独立 FUA 落盘，无需读改写。
 *     - append = spdk_bdev_write_blocks + spdk_bdev_flush_blocks（等价 FUA，bdev 无关）。
 *     - commit = 更新 super.commit_seq + 写 super slot + FUA。
 *     - slot 0 = super，record slot 从 1 起；满一轮后回收已 commit slot（head 前进或回卷）。
 *   TODO（后续阶段）：64B 紧凑打包 + 批量 FUA，降低写放大。
 *
 *   与 PPL 的关系：CRC/hash 计算复用 PPL 已导出的工具（poweraid_raid_common_ppl_data_hash
 *   与 poweraid_raid_common_ppl_hash_*）；本文件不复制其实现，仅按 MWL 字段布局定义
 *   自己的 record/super 结构与对应的打包/校验函数（字段不同故不能直接共用）。
 */

#ifndef POWERAID_RAID_COMMON_MWL_H
#define POWERAID_RAID_COMMON_MWL_H

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 常量 ===== */

#define POWERAID_RAID_COMMON_MWL_REC_MAGIC    0x524C574Du  /* "MWLR" little-endian */
#define POWERAID_RAID_COMMON_MWL_SUPER_MAGIC  0x534C574Du  /* "MWLS" little-endian */

#define POWERAID_RAID_COMMON_MWL_SUPER_SIZE   64
#define POWERAID_RAID_COMMON_MWL_REC_SIZE     64

/* record flags */
#define POWERAID_RAID_COMMON_MWL_REC_F_VALID        (1u << 0)  /* record 有效（magic+crc 通过）*/
#define POWERAID_RAID_COMMON_MWL_REC_F_ACTING_LEAD  (1u << 1)  /* 由 acting lead（非 slot0）产生，
								* acting_slot 字段记录实际 lead 槽位 */

/* ===== 落盘结构 ===== */

/*
 * MWL record（64B）。镜像写意图记录，对应 spec AC-3。
 * 与 PPL record 字段不同（无 stripe/chunk_bitmap，改用 lba/num_blocks），
 * 但 hash 字段语义一致：old/new 各 64-bit，replay 时按 lead 三分支判定。
 * acting_slot：lead(slot0) 在场时为 0；缺时为最低在场槽位，并置 F_ACTING_LEAD。
 */
struct poweraid_raid_common_mwl_record {
	uint32_t	magic;            /* POWERAID_RAID_COMMON_MWL_REC_MAGIC */
	uint32_t	flags;             /* POWERAID_RAID_COMMON_MWL_REC_F_* */
	uint64_t	seq;               /* 全局单调递增序号（由 MWL super.next_seq 分配）*/
	uint64_t	lba;               /* 目标 LBA（块单位，raid1f 数据区相对）*/
	uint64_t	num_blocks;        /* 写范围长度（块）*/
	uint64_t	old_data_hash;     /* 旧 data 的 64-bit hash（复用 PPL data_hash）*/
	uint64_t	new_data_hash;     /* 新 data 的 64-bit hash */
	uint8_t		acting_slot;       /* 实际 lead 槽位（0=slot0 在场）*/
	uint8_t		reserved[3];       /* 对齐预留 */
	uint32_t	crc32c;            /* 覆盖本 record 除 crc32c 字段外全部字节 */
	uint64_t	ts;                /* 时间戳（unix 秒，time(NULL)）*/
};
SPDK_STATIC_ASSERT(sizeof(struct poweraid_raid_common_mwl_record) ==
		   POWERAID_RAID_COMMON_MWL_REC_SIZE, "incorrect mwl record size");

/*
 * MWL super（64B），位于 MWL 区起始（slot 0）。
 * head_seq / tail_seq / commit_seq 均为 record seq（非槽位下标）。
 * acting_lead_slot：当前实际承担 lead 的槽位；slot0 在场时恒为 0，
 * 缺席时为最低在场槽位（与 record 的 acting_slot 一致）。
 */
struct poweraid_raid_common_mwl_super {
	uint32_t	magic;            /* POWERAID_RAID_COMMON_MWL_SUPER_MAGIC */
	uint32_t	version;          /* 1 */
	uint64_t	head_seq;         /* 日志中最旧未回收 record 的 seq（回收起点）*/
	uint64_t	tail_seq;         /* 下一条待写 record 的 seq（== next_seq 快照）*/
	uint64_t	commit_seq;       /* 已 commit 的最大 record seq（commit_seq <= tail_seq）*/
	uint64_t	next_seq;         /* 下一条 record 的 seq（单调递增）*/
	uint64_t	acting_lead_slot; /* 当前 acting lead 槽位（0=slot0）*/
	uint32_t	crc32c;           /* 覆盖本 super 除 crc32c 字段外全部字节 */
	uint32_t	num_records;      /* 当前日志中有效 record 数（tail - head）*/
	uint64_t	reserved;         /* 预留对齐，凑满 64B */
};
SPDK_STATIC_ASSERT(sizeof(struct poweraid_raid_common_mwl_super) ==
		   POWERAID_RAID_COMMON_MWL_SUPER_SIZE, "incorrect mwl super size");

/* ===== 上下文（opaque）===== */
struct poweraid_raid_common_mwl_ctx;

/* in-flight record（已 append 待 commit），挂在 ctx->inflight 链表 */
struct poweraid_raid_common_mwl_inflight {
	uint64_t				seq;
	uint64_t				lba;
	uint64_t				num_blocks;
	uint8_t					acting_slot;
	TAILQ_ENTRY(poweraid_raid_common_mwl_inflight)	link;
};

/* ===== 回调原型 ===== */
typedef void (*poweraid_raid_common_mwl_init_cb)(int status, void *cb_arg);
typedef void (*poweraid_raid_common_mwl_append_cb)(int status, uint64_t seq, void *cb_arg);
typedef void (*poweraid_raid_common_mwl_commit_cb)(int status, void *cb_arg);

/*
 * recovery 回调：load_replay 完成后返回按 seq 升序排列的未 commit record 数组。
 * 调用方负责用 poweraid_raid_common_mwl_free_replay_result 释放。
 * status==0 且 records!=NULL 时 num_records>0；status==0 且 records==NULL 时无未 commit 记录；
 * status!=0 时 records 为 NULL。
 */
typedef void (*poweraid_raid_common_mwl_replay_cb)(int status,
		struct poweraid_raid_common_mwl_record *records, uint32_t num_records,
		void *cb_arg);

/* ===== API ===== */

/**
 * 为单个 base_bdev 分配 MWL 上下文。
 * region_offset/region_size 来自 sb v2 ext（默认 1MiB / 4MiB）。
 * block_size 为 base_bdev 块大小（决定 record slot 步进）。
 */
struct poweraid_raid_common_mwl_ctx *poweraid_raid_common_mwl_alloc(
	void *bdev_desc, struct spdk_io_channel *ch,
	uint32_t block_size,
	uint64_t region_offset, uint64_t region_size);

/**
 * 释放 MWL 上下文（不含磁盘操作，仅内存）。
 */
void poweraid_raid_common_mwl_free(struct poweraid_raid_common_mwl_ctx *ctx);

/**
 * 初始化磁盘 MWL 区：写 super（head=tail=commit=next_seq=1, acting_lead_slot=0）+ FUA。
 * 用于新卷创建或首次格式化。异步。
 */
void poweraid_raid_common_mwl_init(struct poweraid_raid_common_mwl_ctx *ctx,
			      poweraid_raid_common_mwl_init_cb cb, void *cb_arg);

/**
 * 追加一条 MWL record（FUA 落盘，保证意图持久后再写 data）。
 * seq 由 ctx 内部从 super.next_seq 分配并回传。
 * acting_slot：本记录的实际 lead 槽位；slot0 在场时传 0 并自动清 F_ACTING_LEAD，
 * 缺席时传最低在场槽位并自动置 F_ACTING_LEAD。
 * old/new_data_hash 由调用方用 poweraid_raid_common_ppl_data_hash 计算（复用 PPL 工具）。
 * 对应写路径第 1 步。
 */
void poweraid_raid_common_mwl_append_record(
	struct poweraid_raid_common_mwl_ctx *ctx, struct spdk_io_channel *ch,
	uint64_t lba, uint64_t num_blocks,
	uint64_t old_data_hash, uint64_t new_data_hash,
	uint8_t acting_slot,
	poweraid_raid_common_mwl_append_cb cb, void *cb_arg);

/**
 * commit 到指定 seq（含）：更新 super.commit_seq + FUA。
 * 对应写路径第 4 步。data 已 flush 后调用。
 */
void poweraid_raid_common_mwl_commit(struct poweraid_raid_common_mwl_ctx *ctx,
				struct spdk_io_channel *ch, uint64_t seq,
				poweraid_raid_common_mwl_commit_cb cb, void *cb_arg);

/**
 * 启动时扫描 MWL 区，按 seq 升序返回未 commit（seq > commit_seq）的有效 record。
 * replay 流程：lead(slot0) 在场时按其记录的 new/old hash 三分支处理；
 * 缺席时 acting lead 的记录为权威源。
 */
void poweraid_raid_common_mwl_load_replay(struct poweraid_raid_common_mwl_ctx *ctx,
				     poweraid_raid_common_mwl_replay_cb cb, void *cb_arg);

/**
 * 释放 load_replay 回调返回的 record 数组。
 */
void poweraid_raid_common_mwl_free_replay_result(struct poweraid_raid_common_mwl_record *records);

/**
 * 查询当前 acting lead 槽位（用于上层与 record/super 对齐）。
 */
uint8_t poweraid_raid_common_mwl_get_acting_lead_slot(struct poweraid_raid_common_mwl_ctx *ctx);

/**
 * 设置 acting lead 槽位（slot0 补回后归位为 0；slot0 缺席时设最低在场槽位）。
 * 仅更新内存 super 镜像，不落盘；下次 commit 时一并 FUA。
 */
void poweraid_raid_common_mwl_set_acting_lead_slot(struct poweraid_raid_common_mwl_ctx *ctx,
	uint8_t slot);

/**
 * 查询当前 commit_seq（用于上层判断 replay 边界）。
 */
uint64_t poweraid_raid_common_mwl_get_commit_seq(struct poweraid_raid_common_mwl_ctx *ctx);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID_COMMON_MWL_H */
