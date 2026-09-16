/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   poweraid_raid_common 公共层伞头文件
 *
 *   5f/6f 薄壳与公共层内部文件均通过此头文件获取公共数据结构与 API。
 *   取代原 poweraid_raid5f.h 的伞头角色（Stage 4 公共层抽取）。
 */

#ifndef POWERAID_RAID_COMMON_H
#define POWERAID_RAID_COMMON_H

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/uuid.h"

#include "../bdev_raid.h"
#include "poweraid_raid_common_sm.h"
#include "poweraid_raid_common_sb.h"
#include "poweraid_raid_common_ppl.h"
#include "poweraid_raid_common_recovery.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 公共层 API 由各子头文件声明（sm/sb/ppl/recovery/rmw/merge/gf8/rpc）。*/

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID_COMMON_H */
