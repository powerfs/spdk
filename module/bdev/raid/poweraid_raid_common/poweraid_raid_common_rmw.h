/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   RMW（Read-Modify-Write）部分 stripe 写路径
 *
 *   框架 split_on_write_unit / split_on_optimal_io_boundary 已保证写 IO：
 *     - 不跨 stripe 边界
 *     - strip 对齐（offset 与 num_blocks 均 strip_size 整数倍）
 *   故模块收到的写 IO 只有两种：
 *     A) 完整 stripe（offset_blocks % stripe_blocks == 0 且 num_blocks == stripe_blocks）
 *        → 走全 stripe 写路径（poweraid_raid_common_sm_req WRITE_FULL，5 步 barrier）
 *     B) 部分 stripe（非 A）→ 走 RMW（本文件）
 *
 *   RMW 7 步（op + 回调状态机，与 REQ FSM 独立）：
 *     1. 读所有被改 data chunk 的旧数据 + 旧 parity（并行）
 *     2. 算新 parity（XOR syndrome）+ old/new data hash
 *     3. PPL append（FUA intent，含 chunk_bitmap / old_hash / new_hash）
 *     4. 写新 data + 新 parity（并行）
 *     5. flush 被写盘（并行）
 *     6. PPL commit（FUA 标记 committed）
 *     7. complete
 *
 *   PPL record 格式与全 stripe 写一致（old/new hash 严格语义），recovery 零改动。
 *
 *   详见 raid5f-enhanced-design.md 第 3.2 节（阶段 3a）。
 */

#ifndef POWERAID_RAID_COMMON_RMW_H
#define POWERAID_RAID_COMMON_RMW_H

#include "spdk/stdinc.h"

struct raid_bdev_io;
struct raid_bdev_io_channel;
struct poweraid_raid_common_raid;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 提交一个部分 stripe 写 IO（RMW 路径，直接入口）。
 *
 * 调用前已由 submit_rw_request 校验：
 *   - raid 在线（ONLINE && !RESTORING）
 *   - IO 类型为 WRITE
 *   - 非完整 stripe（走 RMW）
 *
 * raid_io 几何（由框架 split 保证，调用方无需再校验）：
 *   - strip 对齐（offset_blocks % strip_size == 0，num_blocks % strip_size == 0）
 *   - IO 不跨 stripe 边界
 *
 * 异步完成：通过 raid_bdev_io_complete 通知 raid_bdev 框架。
 * 返回 0 表示已接受（异步完成），非 0 表示立即失败（调用方负责 complete）。
 */
int poweraid_raid_common_rmw_submit(struct raid_bdev_io *raid_io);

/**
 * 提交合并后的 RMW IO（合并层入口，阶段 3b）。
 *
 * 支持 chunk_bitmap 中非连续的 set bit（合并层多 strip 写同一 stripe 时可能不连续）。
 * 按 chunk_bitmap set bit 升序处理（与 recovery 读盘顺序一致）。
 *
 * \param raid_ch 框架 IO channel（base channel 获取用）
 * \param raid RAID 模块私有数据
 * \param stripe_index stripe 序号
 * \param chunk_bitmap 被改 data chunk 位图（set bit = 被改）
 * \param new_chunk_bufs[data_chunks] 各 chunk 的新数据缓冲（仅 bitmap set bit 对应项有效）
 * \param cb 完成回调（status==0 成功，<0 失败）
 * \param cb_arg 回调参数
 * \return 0 成功（异步完成 via cb），非 0 立即失败
 */
int poweraid_raid_common_rmw_submit_merged(struct raid_bdev_io_channel *raid_ch,
				       struct poweraid_raid_common_raid *raid,
				       uint64_t stripe_index,
				       uint64_t chunk_bitmap,
				       void **new_chunk_bufs,
				       void (*cb)(int status, void *cb_arg),
				       void *cb_arg);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID_COMMON_RMW_H */
