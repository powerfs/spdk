/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   增强 RAID6F 模块：poweraid_raid6f 薄壳公共接口
 *
 *   Stage 4 (RAID6 P+Q)：本头文件仅声明 6f 模块注册相关符号，
 *   其余公共数据结构与 API 来自 poweraid_raid_common/。
 */

#ifndef POWERAID_RAID6F_H
#define POWERAID_RAID6F_H

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/uuid.h"

#include "../bdev_raid.h"
#include "../poweraid_raid_common/poweraid_raid_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 模块初始化：注册到 SPDK bdev_raid 框架。
 * 调用 RAID_MODULE_REGISTER(&g_poweraid_raid6f_module) 自动注册。
 */
extern struct raid_bdev_module g_poweraid_raid6f_module;

/* 模块生命周期 */
int poweraid_raid6f_start(struct raid_bdev *raid_bdev);
bool poweraid_raid6f_stop(struct raid_bdev *raid_bdev);
void poweraid_raid6f_submit_rw_request(struct raid_bdev_io *raid_io);
struct spdk_io_channel *poweraid_raid6f_get_io_channel(struct raid_bdev *raid_bdev);
int poweraid_raid6f_submit_process_request(struct raid_bdev_process_request *process_req,
		struct raid_bdev_io_channel *raid_ch);

/* CALC: P (XOR) + Q (GF8 encode) */
void poweraid_raid6f_calc_parity(struct poweraid_raid_common_req *req,
				 poweraid_raid_calc_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID6F_H */
