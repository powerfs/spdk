/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   增强 RAID5F 模块：poweraid_raid5f 公共接口
 */

#ifndef POWERAID_RAID5F_H
#define POWERAID_RAID5F_H

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/uuid.h"

#include "../bdev_raid.h"
#include "poweraid_raid5f_sm.h"
#include "poweraid_raid5f_sb.h"
#include "poweraid_raid5f_ppl.h"
#include "poweraid_raid5f_recovery.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 模块初始化：注册到 SPDK bdev_raid 框架。
 * 调用 RAID_MODULE_REGISTER(&g_poweraid_raid5f_module) 自动注册。
 */
extern struct raid_bdev_module g_poweraid_raid5f_module;

/* 模块生命周期 stub（阶段 1 仅占位，后续阶段填充）*/
int poweraid_raid5f_start(struct raid_bdev *raid_bdev);
bool poweraid_raid5f_stop(struct raid_bdev *raid_bdev);
void poweraid_raid5f_submit_rw_request(struct raid_bdev_io *raid_io);
struct spdk_io_channel *poweraid_raid5f_get_io_channel(struct raid_bdev *raid_bdev);
int poweraid_raid5f_submit_process_request(struct raid_bdev_process_request *process_req,
		struct raid_bdev_io_channel *raid_ch);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID5F_H */
