/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   启动恢复：扫描 PPL 区未 commit record，按 MD L125-130 三分支判定。
 *
 *   三分支（每个未 commit record）：
 *     - 当前 data hash == new_data_hash → 数据已落盘，parity 可能 stale → 重算并补写 parity
 *     - 当前 data hash == old_data_hash   → 数据未落盘（IO 未到达）→ 跳过
 *     - 都不匹配                          → 部分写 / 损坏 → 标记 inconsistent，触发 scrub
 *
 *   数据读取通过 read_data_fn 回调进行（layout 无关），由集成层（子任务 D）提供
 *   真实的 chunk→bdev/offset 映射，保证正确性。
 *
 *   恢复期间置 RAID_ST_RESTORING，完成后清除。
 */

#ifndef POWERAID_RAID_COMMON_RECOVERY_H
#define POWERAID_RAID_COMMON_RECOVERY_H

#include "spdk/stdinc.h"
#include "poweraid_raid_common_ppl.h"
#include "poweraid_raid_common_sm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 恢复动作（三分支结果）*/
enum poweraid_raid_common_recovery_action {
	POWERAID_RECOVERY_ACT_NONE = 0,         /* data==old → 跳过（IO 未到达）*/
	POWERAID_RECOVERY_ACT_REWRITE_PARITY,     /* data==new → 补写 parity */
	POWERAID_RECOVERY_ACT_INCONSISTENT,       /* 都不匹配 → 标记 faulty，触发 scrub */
};

struct poweraid_raid_common_recovery_result {
	uint64_t	seq;
	uint64_t	stripe_id;
	uint64_t	chunk_bitmap;
	uint64_t	cur_data_hash;  /* 恢复时读盘算出的当前 data hash */
	enum poweraid_raid_common_recovery_action action;
};

/* 读单个 data chunk 的回调签名（layout 无关）。
 * chunk_idx 为 record.chunk_bitmap 中置位的位（升序），chunk_len_blocks = strip_size。
 * cb 返回 buf（len 字节，>= chunk_len_blocks*block_size）。status!=0 表示读失败。 */
typedef void (*poweraid_raid_common_recovery_read_data_cb)(int status, const void *buf,
		size_t len, void *cb_arg);
typedef void (*poweraid_raid_common_recovery_read_data_fn)(
	struct poweraid_raid_common_raid *raid,
	uint64_t stripe_id, uint32_t chunk_idx, uint32_t chunk_len_blocks,
	poweraid_raid_common_recovery_read_data_cb cb, void *cb_arg);

typedef void (*poweraid_raid_common_recovery_done_cb)(int status,
		const struct poweraid_raid_common_recovery_result *results, uint32_t num_results,
		void *cb_arg);

/**
 * 运行恢复：加载 PPL → 逐 record 读 data chunk → 三分支判定 → 返回结果。
 * ppl_ctx 为某 base_bdev 的 PPL 上下文（通常 parity 盘）。
 * read_fn 由集成层提供；为 NULL 时仅做 PPL 扫描，所有 record 标 NONE（安全降级）。
 * 恢复期间置 raid->state |= RAID_ST_RESTORING，完成时清除。
 */
void poweraid_raid_common_recovery_run(
	struct poweraid_raid_common_ppl_ctx *ppl_ctx,
	struct poweraid_raid_common_raid *raid,
	poweraid_raid_common_recovery_read_data_fn read_fn,
	poweraid_raid_common_recovery_done_cb cb, void *cb_arg);

/**
 * 释放 recovery_run 回调返回的 result 数组。
 */
void poweraid_raid_common_recovery_free_result(struct poweraid_raid_common_recovery_result *results);

/**
 * 集成层提供的 read_data_fn 实现（poweraid_raid5f.c）：按 RAID5F 布局
 * 读指定 stripe 的某个 data chunk（整 strip）。
 */
void poweraid_raid_common_recovery_read_strip(
	struct poweraid_raid_common_raid *raid,
	uint64_t stripe_id, uint32_t chunk_idx, uint32_t chunk_len_blocks,
	poweraid_raid_common_recovery_read_data_cb cb, void *cb_arg);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID_COMMON_RECOVERY_H */
