/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   poweraid RAID1F —— 带 MWL（Mirror Write Log）掉电一致性的 N 副本镜像
 *
 *   级别：SPDK_BDEV_RAID_LEVEL_RAID1F(97)，名称 "raid1f"/"1f"
 *   骨架说明（Stage A / Task 1）：
 *     - 模块私有上下文复用 struct poweraid_raid_common_raid（module_private），
 *       使 poweraid RPC（get_info/replace）可直接识别本模块；
 *     - 读写路径与上游 raid1 等价（多副本并行写、读均衡/失败切换/回写纠正）；
 *     - MWL 意图日志（append FUA -> lead -> 其余 -> commit）在后续任务接入，
 *       在此之前本模块不提供掉电一致性保证。
 */

#ifndef POWERAID_RAID1F_H
#define POWERAID_RAID1F_H

#include "spdk/stdinc.h"
#include "../poweraid_raid_common/poweraid_raid_common_sb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* lead 槽位常量 POWERAID_RAID1F_LEAD_SLOT 定义于 common sb.h（MWL 层共用）。 */

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID1F_H */
