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
 *        → 走全 stripe 写路径（poweraid_raid5f_sm_req WRITE_FULL，5 步 barrier）
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

#ifndef POWERAID_RAID5F_RMW_H
#define POWERAID_RAID5F_RMW_H

#include "spdk/stdinc.h"

struct raid_bdev_io;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 提交一个部分 stripe 写 IO（RMW 路径）。
 *
 * 调用前已由 submit_rw_request 校验：
 *   - raid 在线（ONLINE && !RESTORING）
 *   - IO 类型为 WRITE
 *   - 非完整 stripe（走 RMW）
 *
 * raid_io 几何（由框架 split 保证，调用方无需再校验）：
 *   - stripe_offset = offset_blocks % stripe_blocks，strip 对齐
 *   - num_blocks 为 strip_size 整数倍
 *   - IO 不跨 stripe 边界
 *   - 被改 data chunk 连续：[start_chunk, start_chunk + num_modified)
 *     其中 start_chunk = stripe_offset / strip_size，
 *          num_modified = num_blocks / strip_size，且 num_modified < data_chunks
 *
 * 异步完成：通过 raid_bdev_io_complete 通知 raid_bdev 框架。
 * 返回 0 表示已接受（异步完成），非 0 表示立即失败（调用方负责 complete）。
 */
int poweraid_raid5f_rmw_submit(struct raid_bdev_io *raid_io);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID5F_RMW_H */
