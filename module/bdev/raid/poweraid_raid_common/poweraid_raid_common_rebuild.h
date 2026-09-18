/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   poweraid RAID 换盘/重建公共层（Stage 4 步骤 7）
 *
 *   两部分：
 *
 *   1. 模块生命周期 hook（注册到 struct raid_bdev_module，5f/6f 共用）
 *      - base_bdev_removed       成员盘被移除（热拔/IO 失败踢出）
 *      - base_bdev_rebuild_starting 空白新盘成为 rebuild 目标
 *      - process_complete        rebuild 结束（成功/失败）
 *      - process_window_advanced 重建窗口在某 IO channel 上推进/结束
 *
 *   2. 重建窗口门控（gate）
 *      poweraid 的写路径（全 stripe / RMW / merge）绕过框架 split 逻辑，
 *      直接按 stripe 写成员盘。重建期间必须按窗口边界门控：
 *        GATE_PASS      该 channel 无活动进程，正常提交；
 *        GATE_PROCESSED stripe 完全落在已处理窗口内，改走 shadow channel
 *                       （目标槽位路由到 target_ch），可正常提交；
 *        GATE_WAIT      stripe 未被窗口覆盖，延迟提交（merge 留在 pending
 *                       列表由 poller/窗口推进回调重放；直写路径返回 NOMEM
 *                                             由 bdev 层排队重试）。
 *
 *      正确性论证：重建引擎对未处理 stripe 从存活盘重构目标盘；该 stripe 的
 *      任何前台写（无论是否覆盖目标数据块）都必须等重构完成后再落盘，否则
 *      旧数据/旧校验会污染新盘或产生 stale。统一规则：进程活动期间，stripe
 *      完全越过窗口才放行，且必须经 shadow channel 路由。
 */

#ifndef POWERAID_RAID_COMMON_REBUILD_H
#define POWERAID_RAID_COMMON_REBUILD_H

#include "spdk/stdinc.h"

#include "../bdev_raid.h"
#include "poweraid_raid_common_sm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 窗口门控结论 */
enum poweraid_raid_common_gate {
	POWERAID_RAID_COMMON_GATE_PASS = 0,	/* 无进程：使用原 channel */
	POWERAID_RAID_COMMON_GATE_PROCESSED,	/* 已越过窗口：使用 eff_ch（shadow） */
	POWERAID_RAID_COMMON_GATE_WAIT,		/* 未越过窗口：延迟提交 */
};

/**
 * 判定某 stripe 在给定 raid channel 上的门控状态。
 * eff_ch 输出实际应用于成员盘 channel 查询的 raid channel（PASS 时==raid_ch，
 * PROCESSED 时为框架 shadow channel，WAIT 时不允许提交）。
 */
enum poweraid_raid_common_gate
poweraid_raid_common_rebuild_gate_classify(struct poweraid_raid_common_raid *raid,
		struct raid_bdev_io_channel *raid_ch, uint64_t stripe_index,
		struct raid_bdev_io_channel **eff_ch);

/* ===== struct raid_bdev_module hook 实现（5f/6f 直接注册）===== */

void poweraid_raid_common_hook_base_bdev_removed(struct raid_bdev *raid_bdev,
		uint8_t slot);

void poweraid_raid_common_hook_rebuild_starting(struct raid_bdev *raid_bdev,
		struct raid_base_bdev_info *target);

void poweraid_raid_common_hook_process_complete(struct raid_bdev *raid_bdev,
		struct raid_base_bdev_info *target, int status);

void poweraid_raid_common_hook_window_advanced(struct raid_bdev *raid_bdev,
		struct raid_bdev_io_channel *raid_ch, bool ended, int status);

/* rebuild process 引擎：5f/6f 模块的 submit_process_request 直接转发于此。
 * 依赖模块注册 ops.recover_missing 完成条带重构。*/
int poweraid_raid_common_submit_process_request(
		struct raid_bdev_process_request *process_req,
		struct raid_bdev_io_channel *raid_ch);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID_COMMON_REBUILD_H */
