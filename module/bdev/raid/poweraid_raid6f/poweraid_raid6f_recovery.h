/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   RAID6F 降级读 + 重建 strip 恢复（Stage 4 步骤 7 从 poweraid_raid6f.c 抽出）
 *
 *   三类重建方法（目标 data chunk 故障）：
 *     P_XOR    : P 存活  → D_t = P XOR 其他存活 D
 *     Q_SINGLE : P 也故障 → D_t = α^-t × (Q XOR Σ α^i D_i)
 *     DUAL     : 两块 data 均故障 → gf8_decode_2 解 2×2
 *
 *   poweraid_raid6f_recover_strip 是重建引擎专用入口：目标槽位在 stripe 中
 *   可能承担 data/P/Q 任意角色，并允许另一块成员盘同时故障（双盘故障串行重建）。
 */

#ifndef POWERAID_RAID6F_RECOVERY_H
#define POWERAID_RAID6F_RECOVERY_H

#include "spdk/stdinc.h"

#include "../bdev_raid.h"
#include "../poweraid_raid_common/poweraid_raid_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 某物理槽在给定 raid channel 上是否故障（desc 缺失/通道缺失/FAULTED 位）*/
bool poweraid_raid6f_slot_faulted(struct poweraid_raid_common_raid *raid,
				  struct raid_bdev_io_channel *raid_ch, uint8_t phys);

/* 扫描通道视图刷新卷 DEGRADED/DEGRADED2 状态位 */
void poweraid_raid6f_refresh_degraded_state(struct poweraid_raid_common_raid *raid,
		struct raid_bdev_io_channel *raid_ch);

/* 将某物理槽标记 FAULTED 并刷新降级状态（直读 IO error 入口）*/
void poweraid_raid6f_mark_slot_faulted(struct poweraid_raid_common_raid *raid,
				       struct raid_bdev_io_channel *raid_ch,
				       uint8_t phys);

/* 降级读入口（shell read 路径调用）*/
int poweraid_raid6f_submit_degraded_read(struct raid_bdev_io *raid_io,
		uint64_t stripe_index,
		uint8_t p_idx, uint8_t q_idx,
		uint8_t target_phys, uint8_t target_data,
		uint64_t chunk_offset_blocks);

/* shell 的 layout-aware 读入口（recovery 完成重试需要，非 static）*/
int poweraid_raid6f_submit_read_request(struct raid_bdev_io *raid_io);

/**
 * 重建引擎：重构 stripe 中 target_phys 槽位的整条 strip，写入调用方提供的
 * DMA 缓冲 out_buf（长度 strip_len == raid->strip_size*block_size，4K 对齐）。
 * 允许另一块成员盘同时故障（RAID6 双盘故障串行重建场景）。
 *
 * 异步：成功/失败均经 cb(status) 通知；返回 0 表示已接受，<0 立即失败。
 */
int poweraid_raid6f_recover_strip(struct poweraid_raid_common_raid *raid,
				  struct raid_bdev_io_channel *raid_ch,
				  uint64_t stripe_index, uint8_t target_phys,
				  void *out_buf, uint32_t strip_len,
				  void (*cb)(int status, void *cb_arg),
				  void *cb_arg);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID6F_RECOVERY_H */
