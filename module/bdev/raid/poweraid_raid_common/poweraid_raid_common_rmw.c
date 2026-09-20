/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   RMW（Read-Modify-Write）部分 stripe 写路径实现。
 *   详见 poweraid_raid_common_rmw.h 与 raid5f-enhanced-design.md 第 3.2 节。
 *
 *   op + 回调状态机（参考 parity_fixup 模式），与 REQ FSM 独立：
 *     submit → READ_OLD(data+parity 并行) → CALC → PPL_APPEND
 *            → WRITE(data+parity 并行) → FLUSH(并行) → PPL_COMMIT → COMPLETE
 *
 *   阶段 3b：支持非连续 chunk_bitmap（合并层多 strip 写同一 stripe 时可能不连续）。
 *   按 chunk_bitmap set bit 升序迭代（与 recovery 读盘顺序一致）。
 *
 *   ENOMEM：简化为失败（与 write_full 一致），由 bdev 层重试整个 raid_io。
 *     PPL record 已 append 但未 commit，recovery 三分支判定处理（data==old → NONE）。
 *
 *   两种入口：
 *     A) poweraid_raid_common_rmw_submit(raid_io) — 直接单 strip 写（raid_io 完成回调）
 *     B) poweraid_raid_common_rmw_submit_merged(...) — 合并层调用（自定义完成回调）
 *
 *   data_idx → 物理盘映射：phys = (data_idx < p_idx) ? data_idx : data_idx + 1
 *     （left-symmetric，与 recovery_read_strip 一致）
 */

#include "spdk/stdinc.h"
#include "spdk/log.h"
#include "spdk/string.h"
#include "spdk/util.h"
#include "spdk/bdev_module.h"

#include "../bdev_raid.h"
#include "poweraid_raid_common.h"
#include "poweraid_raid_common_rmw.h"
#include "poweraid_raid_common_ppl.h"
#include "poweraid_raid_common_gf8.h"
#include "poweraid_raid_common_rebuild.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_rmw);

/* ===== op 状态机 ===== */
enum rmw_state {
	RMW_S_READ_OLD = 0,
	RMW_S_CALC,
	RMW_S_PPL_APPEND,
	RMW_S_WRITE,
	RMW_S_FLUSH,
	RMW_S_PPL_COMMIT,
	RMW_S_DONE,
};

struct rmw_op {
	struct raid_bdev_io			*raid_io;	/* NULL 当合并层调用 */
	struct raid_bdev_io_channel		*raid_ch;	/* 框架 IO channel（合并层路径 raid_io=NULL 时使用）*/
	struct poweraid_raid_common_raid		*raid;
	struct raid_bdev			*raid_bdev;	/* 缓存，避免重复解引用 */

	uint64_t				stripe_index;
	uint8_t					p_idx;		/* P parity 物理盘号 */
	uint8_t					q_idx;		/* Q parity 物理盘号 */
	uint64_t				chunk_bitmap;	/* set bit i = data chunk i 被改 */
	uint32_t				num_modified;	/* popcount(chunk_bitmap)，预计算 */

	uint32_t				strip_bytes;

	/* 缓冲：old_data/new_data 按 chunk_bitmap set bit 升序排列（buf_idx 0..num_modified-1） */
	void					*old_data_buf;	/* num_modified * strip_bytes */
	void					*new_data_buf;	/* num_modified * strip_bytes */
	void					*parity_buf;	/* strip_bytes：读旧 P → 原地 XOR 成新 P */
	void					*q_buf;		/* strip_bytes：读旧 Q → 原地 XOR 成新 Q（num_parity=2 时）*/

	/* PPL */
	struct poweraid_raid_common_ppl_ctx		*ppl_ctx;
	struct spdk_io_channel			*ppl_ch;
	uint64_t				ppl_seq;	/* 0 表示无 PPL 保护 */

	/* 状态机 */
	enum rmw_state				state;
	uint32_t				io_remaining;
	int					io_status;

	/* 完成回调（合并层路径用；直接路径为 NULL → raid_bdev_io_complete）*/
	void					(*complete_cb)(int status, void *cb_arg);
	void					*complete_cb_arg;

	/* sub-strip 写支持（issue #14）：raid_io 数据只覆盖 strip 一部分，
	 * 需在读旧 data 后用旧 data 填充 new_data_buf 再覆盖 sub-strip 新数据。*/
	bool					is_sub_strip;
	uint32_t				sub_strip_offset_bytes;	/* strip 内偏移 */
	uint32_t				sub_strip_len_bytes;	/* sub-strip 数据长度 */
};

/* 前向声明 */
static void rmw_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void rmw_start_writes(struct rmw_op *op);
static void rmw_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void rmw_start_flushes(struct rmw_op *op);
static void rmw_flush_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void rmw_ppl_append_done(int status, uint64_t seq, void *cb_arg);
static void rmw_start_commit(struct rmw_op *op);
static void rmw_ppl_commit_done(int status, void *cb_arg);
static void rmw_calc_and_next(struct rmw_op *op);

/* ===== 释放与完成 ===== */

static void
rmw_op_free(struct rmw_op *op)
{
	if (op->old_data_buf != NULL) {
		spdk_dma_free(op->old_data_buf);
	}
	if (op->new_data_buf != NULL) {
		spdk_dma_free(op->new_data_buf);
	}
	if (op->parity_buf != NULL) {
		spdk_dma_free(op->parity_buf);
	}
	if (op->q_buf != NULL) {
		spdk_dma_free(op->q_buf);
	}
	free(op);
}

static void
rmw_op_finish(struct rmw_op *op, enum spdk_bdev_io_status status)
{
	SPDK_DEBUGLOG(raid5f_rmw, "rmw finish: op=%p status=%u stripe=%"PRIu64
		      " bitmap=0x%"PRIx64" num_modified=%u\n",
		      op, status, op->stripe_index, op->chunk_bitmap, op->num_modified);

	if (op->complete_cb != NULL) {
		int s = (status == SPDK_BDEV_IO_STATUS_SUCCESS) ? 0 : -EIO;
		void (*cb)(int, void *) = op->complete_cb;
		void *cb_arg = op->complete_cb_arg;
		rmw_op_free(op);
		cb(s, cb_arg);
	} else if (op->raid_io != NULL) {
		struct raid_bdev_io *raid_io = op->raid_io;
		rmw_op_free(op);
		raid_bdev_io_complete(raid_io, status);
	} else {
		/* 无回调且无 raid_io：不应发生，释放并警告 */
		SPDK_ERRLOG("rmw finish: no completion path, op=%p\n", op);
		rmw_op_free(op);
	}
}

/* ===== 几何辅助 ===== */

/* data chunk 序号 → 物理盘号（left-symmetric，兼容 RAID5/6）*/
static inline uint8_t
rmw_data_to_phys(uint8_t data_idx, uint8_t p_idx, uint8_t q_idx,
		 uint8_t num_base_bdevs)
{
	return poweraid_raid_common_data_to_phys(data_idx, p_idx, q_idx,
						 num_base_bdevs);
}

/* 在当前 raid 状态下是否可继续（OFFLINE 则中止）*/
static inline bool
rmw_can_proceed(struct rmw_op *op)
{
	return !poweraid_raid_state_test(&op->raid->state, POWERAID_RAID_ST_OFFLINE);
}

/* 查找首个带 ppl_ctx 的成员盘 + 对应 IO 线程 base channel（与 write_full 一致）*/
static void
rmw_find_ppl(struct rmw_op *op)
{
	struct poweraid_raid_common_raid *raid = op->raid;
	uint8_t i;

	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL &&
		    raid->base_bdevs[i]->ppl_ctx != NULL) {
			op->ppl_ctx = raid->base_bdevs[i]->ppl_ctx;
			op->ppl_ch = raid_bdev_channel_get_base_channel(
				op->raid_ch, i);
			break;
		}
	}
}

/* ===== Step 1：读旧 data + 旧 parity（并行）===== */

static void
rmw_start_reads(struct rmw_op *op)
{
	struct raid_bdev *raid_bdev = op->raid_bdev;
	struct poweraid_raid_common_raid *raid = op->raid;
	/* helper 内部已加 base_info->data_offset，此处传 stripe 相对偏移 */
	uint64_t base_offset = op->stripe_index * raid->strip_size;
	uint64_t bits = op->chunk_bitmap;
	uint32_t buf_idx = 0;
	int rc;

	op->state = RMW_S_READ_OLD;
	op->io_remaining = op->num_modified + raid->num_parity; /* data chunks + P(+Q) */
	op->io_status = 0;

	SPDK_DEBUGLOG(raid5f_rmw, "rmw reads: op=%p stripe=%"PRIu64" base_offset=%"PRIu64
		      " p_idx=%u bitmap=0x%"PRIx64" num_modified=%u\n",
		      op, op->stripe_index, base_offset, op->p_idx,
		      op->chunk_bitmap, op->num_modified);

	/* 读旧 data chunks（按 chunk_bitmap set bit 升序）*/
	while (bits) {
		uint8_t data_idx = __builtin_ctzll(bits);
		bits &= bits - 1;
		uint8_t phys = rmw_data_to_phys(data_idx, op->p_idx, op->q_idx,
						op->raid_bdev->num_base_bdevs);
		struct raid_base_bdev_info *base_info = &raid_bdev->base_bdev_info[phys];
		struct spdk_io_channel *base_ch;
		struct spdk_bdev_ext_io_opts io_opts = {0};
		struct iovec iov = {
			.iov_base = (char *)op->old_data_buf + buf_idx * op->strip_bytes,
			.iov_len = op->strip_bytes,
		};

		io_opts.size = sizeof(io_opts);
		base_ch = raid_bdev_channel_get_base_channel(op->raid_ch, phys);
		if (base_ch == NULL || base_info->desc == NULL) {
			SPDK_ERRLOG("rmw read: no channel for data chunk %u (phys %u)\n",
				    data_idx, phys);
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_calc_and_next(op);
			}
			buf_idx++;
			continue;
		}

		rc = raid_bdev_readv_blocks_ext(base_info, base_ch, &iov, 1,
						base_offset, raid->strip_size,
						rmw_read_cb, op, &io_opts);
		if (rc != 0) {
			SPDK_ERRLOG("rmw read: data chunk %u failed rc=%d\n", data_idx, rc);
			op->io_status = rc;
			if (--op->io_remaining == 0) {
				rmw_calc_and_next(op);
			}
		}
		buf_idx++;
	}

	/* 读旧 parity（p_idx 盘）*/
	{
		struct raid_base_bdev_info *p_info = &raid_bdev->base_bdev_info[op->p_idx];
		struct spdk_io_channel *p_ch;
		struct spdk_bdev_ext_io_opts io_opts = {0};
		struct iovec iov = {
			.iov_base = op->parity_buf,
			.iov_len = op->strip_bytes,
		};

		io_opts.size = sizeof(io_opts);
		p_ch = raid_bdev_channel_get_base_channel(op->raid_ch, op->p_idx);
		if (p_ch == NULL || p_info->desc == NULL) {
			SPDK_ERRLOG("rmw read: no channel for P parity (p_idx %u)\n",
				    op->p_idx);
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_calc_and_next(op);
			}
		} else {
			rc = raid_bdev_readv_blocks_ext(p_info, p_ch, &iov, 1,
							base_offset, raid->strip_size,
							rmw_read_cb, op, &io_opts);
			if (rc != 0) {
				SPDK_ERRLOG("rmw read: P parity failed rc=%d\n", rc);
				op->io_status = rc;
				if (--op->io_remaining == 0) {
					rmw_calc_and_next(op);
				}
			}
		}
	}

	/* RAID6: 读旧 Q（q_idx 盘）*/
	if (raid->num_parity > 1) {
		struct raid_base_bdev_info *q_info = &raid_bdev->base_bdev_info[op->q_idx];
		struct spdk_io_channel *q_ch;
		struct spdk_bdev_ext_io_opts io_opts = {0};
		struct iovec iov = {
			.iov_base = op->q_buf,
			.iov_len = op->strip_bytes,
		};

		io_opts.size = sizeof(io_opts);
		q_ch = raid_bdev_channel_get_base_channel(op->raid_ch, op->q_idx);
		if (q_ch == NULL || q_info->desc == NULL) {
			SPDK_ERRLOG("rmw read: no channel for Q parity (q_idx %u)\n",
				    op->q_idx);
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_calc_and_next(op);
			}
		} else {
			rc = raid_bdev_readv_blocks_ext(q_info, q_ch, &iov, 1,
							base_offset, raid->strip_size,
							rmw_read_cb, op, &io_opts);
			if (rc != 0) {
				SPDK_ERRLOG("rmw read: Q parity failed rc=%d\n", rc);
				op->io_status = rc;
				if (--op->io_remaining == 0) {
					rmw_calc_and_next(op);
				}
			}
		}
	}
}

static void
rmw_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct rmw_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		SPDK_ERRLOG("rmw read cb: failure (remaining=%u)\n",
			    op->io_remaining - 1);
		op->io_status = -EIO;
	}

	if (--op->io_remaining == 0) {
		rmw_calc_and_next(op);
	}
}

/* ===== Step 2：CALC（新 parity + old/new hash）→ Step 3 PPL_APPEND ===== */

static void
rmw_calc_and_next(struct rmw_op *op)
{
	uint32_t strip_u64 = op->strip_bytes / sizeof(uint64_t);
	uint64_t *parity = op->parity_buf;
	struct poweraid_raid_common_raid *raid = op->raid;
	struct poweraid_raid_common_ppl_hash_ctx old_h, new_h;
	uint64_t old_hash, new_hash;
	uint64_t bits;
	uint32_t buf_idx, j;
	int rc;

	if (!rmw_can_proceed(op)) {
		SPDK_WARNLOG("rmw: raid offline during reads, abort op=%p\n", op);
		rmw_op_finish(op, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}
	if (op->io_status != 0) {
		SPDK_ERRLOG("rmw: reads failed (%d), abort op=%p\n",
			    op->io_status, op);
		rmw_op_finish(op, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	op->state = RMW_S_CALC;

	/* sub-strip 写：读旧 data 完成后，用旧 data 填充 new_data_buf，
	 * 再用 raid_io 数据覆盖 sub-strip 区域，形成完整 strip 新数据。
	 * 之后 XOR 计算 parity 与 strip 对齐路径完全一致。*/
	if (op->is_sub_strip) {
		memcpy(op->new_data_buf, op->old_data_buf, op->strip_bytes);
		rc = (int)spdk_iovcpy(op->raid_io->iovs, op->raid_io->iovcnt,
			&(struct iovec){
				.iov_base = (char *)op->new_data_buf + op->sub_strip_offset_bytes,
				.iov_len = op->sub_strip_len_bytes,
			}, 1);
		if (rc != (int)op->sub_strip_len_bytes) {
			SPDK_ERRLOG("rmw sub-strip iovcpy short copied=%d expect=%u\n",
				    rc, op->sub_strip_len_bytes);
			rmw_op_finish(op, SPDK_BDEV_IO_STATUS_FAILED);
			return;
		}
	}

	/* 新 parity = 旧 parity ^ (旧 data[i] ^ 新 data[i]) for each modified i
	 * 原地更新 parity_buf：先已是旧 parity，XOR 进 (old ^ new) 即得新 parity。
	 * 按 chunk_bitmap set bit 升序迭代（与 recovery 读盘顺序一致）。*/
	bits = op->chunk_bitmap;
	buf_idx = 0;
	while (bits) {
		uint8_t data_idx = __builtin_ctzll(bits);
		const uint64_t *oldp = (const uint64_t *)
			((char *)op->old_data_buf + buf_idx * op->strip_bytes);
		const uint64_t *newp = (const uint64_t *)
			((char *)op->new_data_buf + buf_idx * op->strip_bytes);
		for (j = 0; j < strip_u64; j++) {
			parity[j] ^= oldp[j] ^ newp[j];
		}
		/* RAID6: Q delta = old_Q ^ mul(old_data, α^i) ^ mul(new_data, α^i)
		 * 利用 gf8 线性性：mul(a^b, c) = mul(a,c) ^ mul(b,c) */
		if (raid->num_parity > 1) {
			uint8_t coeff = poweraid_raid_common_gf8_exp[data_idx];
			poweraid_raid_common_gf8_mul_const_xor(
				oldp, coeff, op->q_buf, op->strip_bytes);
			poweraid_raid_common_gf8_mul_const_xor(
				newp, coeff, op->q_buf, op->strip_bytes);
		}
		bits &= bits - 1;
		buf_idx++;
	}

	/* old/new hash：按 chunk_bitmap set bit 升序 */
	poweraid_raid_common_ppl_hash_init(&old_h);
	poweraid_raid_common_ppl_hash_init(&new_h);
	bits = op->chunk_bitmap;
	buf_idx = 0;
	while (bits) {
		poweraid_raid_common_ppl_hash_update(&old_h,
			(char *)op->old_data_buf + buf_idx * op->strip_bytes,
			op->strip_bytes);
		poweraid_raid_common_ppl_hash_update(&new_h,
			(char *)op->new_data_buf + buf_idx * op->strip_bytes,
			op->strip_bytes);
		bits &= bits - 1;
		buf_idx++;
	}
	old_hash = poweraid_raid_common_ppl_hash_final(&old_h);
	new_hash = poweraid_raid_common_ppl_hash_final(&new_h);

	SPDK_DEBUGLOG(raid5f_rmw, "rmw calc: op=%p old_hash=0x%"PRIx64
		      " new_hash=0x%"PRIx64" bitmap=0x%"PRIx64"\n",
		      op, old_hash, new_hash, op->chunk_bitmap);

	/* Step 3: PPL append（FUA intent）*/
	op->state = RMW_S_PPL_APPEND;
	if (op->ppl_ctx != NULL && op->ppl_ch != NULL) {
		poweraid_raid_common_ppl_append_record(op->ppl_ctx, op->ppl_ch,
						 op->stripe_index, op->chunk_bitmap,
						 old_hash, new_hash,
						 rmw_ppl_append_done, op);
	} else {
		SPDK_WARNLOG("rmw: no ppl_ctx, write without PPL protection op=%p\n", op);
		rmw_ppl_append_done(0, 0, op);
	}
}

static void
rmw_ppl_append_done(int status, uint64_t seq, void *cb_arg)
{
	struct rmw_op *op = cb_arg;

	SPDK_DEBUGLOG(raid5f_rmw, "rmw ppl_append_done: op=%p status=%d seq=%"PRIu64"\n",
		      op, status, seq);

	if (status != 0) {
		/* PPL append 失败仅由真实 IO 错误或 ctx 销毁（-ECANCELED）引起；
		 * -ENOSPC 由 backpressure 队列吸收，不会到达此处。降级写无 PPL 保护。*/
		SPDK_ERRLOG("rmw: ppl append failed (%d), degrade write without PPL\n", status);
	}
	op->ppl_seq = (status == 0) ? seq : 0;

	if (!rmw_can_proceed(op)) {
		SPDK_WARNLOG("rmw: raid offline after ppl_append, abort op=%p\n", op);
		rmw_op_finish(op, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	rmw_start_writes(op);
}

/* ===== Step 4：写新 data + 新 parity（并行）===== */

static void
rmw_start_writes(struct rmw_op *op)
{
	struct raid_bdev *raid_bdev = op->raid_bdev;
	struct poweraid_raid_common_raid *raid = op->raid;
	/* helper 内部已加 base_info->data_offset，此处传 stripe 相对偏移 */
	uint64_t base_offset = op->stripe_index * raid->strip_size;
	uint64_t bits = op->chunk_bitmap;
	uint32_t buf_idx = 0;
	int rc;

	op->state = RMW_S_WRITE;
	op->io_remaining = op->num_modified + raid->num_parity; /* data chunks + P(+Q) */
	op->io_status = 0;

	while (bits) {
		uint8_t data_idx = __builtin_ctzll(bits);
		bits &= bits - 1;
		uint8_t phys = rmw_data_to_phys(data_idx, op->p_idx, op->q_idx,
						op->raid_bdev->num_base_bdevs);
		struct raid_base_bdev_info *base_info = &raid_bdev->base_bdev_info[phys];
		struct spdk_io_channel *base_ch;
		struct spdk_bdev_ext_io_opts io_opts = {0};
		struct iovec iov = {
			.iov_base = (char *)op->new_data_buf + buf_idx * op->strip_bytes,
			.iov_len = op->strip_bytes,
		};

		io_opts.size = sizeof(io_opts);
		base_ch = raid_bdev_channel_get_base_channel(op->raid_ch, phys);
		if (base_ch == NULL || base_info->desc == NULL) {
			SPDK_ERRLOG("rmw write: no channel for data chunk %u (phys %u)\n",
				    data_idx, phys);
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_start_flushes(op);
			}
			buf_idx++;
			continue;
		}

		rc = raid_bdev_writev_blocks_ext(base_info, base_ch, &iov, 1,
						 base_offset, raid->strip_size,
						 rmw_write_cb, op, &io_opts);
		if (rc != 0) {
			SPDK_ERRLOG("rmw write: data chunk %u failed rc=%d\n", data_idx, rc);
			op->io_status = rc;
			if (--op->io_remaining == 0) {
				rmw_start_flushes(op);
			}
		}
		buf_idx++;
	}

	/* 写新 P parity（p_idx 盘）*/
	{
		struct raid_base_bdev_info *p_info = &raid_bdev->base_bdev_info[op->p_idx];
		struct spdk_io_channel *p_ch;
		struct spdk_bdev_ext_io_opts io_opts = {0};
		struct iovec iov = {
			.iov_base = op->parity_buf,
			.iov_len = op->strip_bytes,
		};

		io_opts.size = sizeof(io_opts);
		p_ch = raid_bdev_channel_get_base_channel(op->raid_ch, op->p_idx);
		if (p_ch == NULL || p_info->desc == NULL) {
			SPDK_ERRLOG("rmw write: no channel for P parity (p_idx %u)\n",
				    op->p_idx);
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_start_flushes(op);
			}
		} else {
			rc = raid_bdev_writev_blocks_ext(p_info, p_ch, &iov, 1,
							 base_offset, raid->strip_size,
							 rmw_write_cb, op, &io_opts);
			if (rc != 0) {
				SPDK_ERRLOG("rmw write: P parity failed rc=%d\n", rc);
				op->io_status = rc;
				if (--op->io_remaining == 0) {
					rmw_start_flushes(op);
				}
			}
		}
	}

	/* RAID6: 写新 Q parity（q_idx 盘）*/
	if (raid->num_parity > 1) {
		struct raid_base_bdev_info *q_info = &raid_bdev->base_bdev_info[op->q_idx];
		struct spdk_io_channel *q_ch;
		struct spdk_bdev_ext_io_opts io_opts = {0};
		struct iovec iov = {
			.iov_base = op->q_buf,
			.iov_len = op->strip_bytes,
		};

		io_opts.size = sizeof(io_opts);
		q_ch = raid_bdev_channel_get_base_channel(op->raid_ch, op->q_idx);
		if (q_ch == NULL || q_info->desc == NULL) {
			SPDK_ERRLOG("rmw write: no channel for Q parity (q_idx %u)\n",
				    op->q_idx);
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_start_flushes(op);
			}
		} else {
			rc = raid_bdev_writev_blocks_ext(q_info, q_ch, &iov, 1,
							 base_offset, raid->strip_size,
							 rmw_write_cb, op, &io_opts);
			if (rc != 0) {
				SPDK_ERRLOG("rmw write: Q parity failed rc=%d\n", rc);
				op->io_status = rc;
				if (--op->io_remaining == 0) {
					rmw_start_flushes(op);
				}
			}
		}
	}
}

static void
rmw_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct rmw_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		SPDK_ERRLOG("rmw write cb: failure (remaining=%u)\n",
			    op->io_remaining - 1);
		op->io_status = -EIO;
	}

	if (--op->io_remaining == 0) {
		rmw_start_flushes(op);
	}
}

/* ===== Step 5：flush 被写盘（并行）===== */

static void
rmw_start_flushes(struct rmw_op *op)
{
	struct raid_bdev *raid_bdev = op->raid_bdev;
	struct poweraid_raid_common_raid *raid = op->raid;
	/* helper 内部已加 base_info->data_offset，此处传 stripe 相对偏移 */
	uint64_t base_offset = op->stripe_index * raid->strip_size;
	uint64_t bits = op->chunk_bitmap;
	int rc;

	if (!rmw_can_proceed(op)) {
		SPDK_WARNLOG("rmw: raid offline before flush, abort op=%p\n", op);
		rmw_op_finish(op, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}
	if (op->io_status != 0) {
		SPDK_ERRLOG("rmw: writes failed (%d), skip flush op=%p\n",
			    op->io_status, op);
		/* 写失败：若有 ppl_seq 则不 commit（record 留 uncommitted，
		 * recovery 按 data==old → NONE 处理），直接失败完成。*/
		rmw_op_finish(op, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	op->state = RMW_S_FLUSH;
	op->io_remaining = op->num_modified + raid->num_parity; /* data + P(+Q) */
	op->io_status = 0;

	while (bits) {
		uint8_t data_idx = __builtin_ctzll(bits);
		bits &= bits - 1;
		uint8_t phys = rmw_data_to_phys(data_idx, op->p_idx, op->q_idx,
						op->raid_bdev->num_base_bdevs);
		struct raid_base_bdev_info *base_info = &raid_bdev->base_bdev_info[phys];
		struct spdk_io_channel *base_ch;

		base_ch = raid_bdev_channel_get_base_channel(op->raid_ch, phys);
		if (base_ch == NULL || base_info->desc == NULL) {
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_start_commit(op);
			}
			continue;
		}

		rc = raid_bdev_flush_blocks(base_info, base_ch, base_offset,
					    raid->strip_size,
					    rmw_flush_cb, op);
		if (rc != 0) {
			SPDK_ERRLOG("rmw flush: data chunk %u failed rc=%d\n",
				    data_idx, rc);
			op->io_status = rc;
			if (--op->io_remaining == 0) {
				rmw_start_commit(op);
			}
		}
	}

	/* flush P parity 盘 */
	{
		struct raid_base_bdev_info *p_info = &raid_bdev->base_bdev_info[op->p_idx];
		struct spdk_io_channel *p_ch;

		p_ch = raid_bdev_channel_get_base_channel(op->raid_ch, op->p_idx);
		if (p_ch == NULL || p_info->desc == NULL) {
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_start_commit(op);
			}
		} else {
			rc = raid_bdev_flush_blocks(p_info, p_ch, base_offset,
						    raid->strip_size,
						    rmw_flush_cb, op);
			if (rc != 0) {
				SPDK_ERRLOG("rmw flush: P parity failed rc=%d\n", rc);
				op->io_status = rc;
				if (--op->io_remaining == 0) {
					rmw_start_commit(op);
				}
			}
		}
	}

	/* RAID6: flush Q parity 盘 */
	if (raid->num_parity > 1) {
		struct raid_base_bdev_info *q_info = &raid_bdev->base_bdev_info[op->q_idx];
		struct spdk_io_channel *q_ch;

		q_ch = raid_bdev_channel_get_base_channel(op->raid_ch, op->q_idx);
		if (q_ch == NULL || q_info->desc == NULL) {
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_start_commit(op);
			}
		} else {
			rc = raid_bdev_flush_blocks(q_info, q_ch, base_offset,
						    raid->strip_size,
						    rmw_flush_cb, op);
			if (rc != 0) {
				SPDK_ERRLOG("rmw flush: Q parity failed rc=%d\n", rc);
				op->io_status = rc;
				if (--op->io_remaining == 0) {
					rmw_start_commit(op);
				}
			}
		}
	}
}

static void
rmw_flush_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct rmw_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		SPDK_ERRLOG("rmw flush cb: failure (remaining=%u)\n",
			    op->io_remaining - 1);
		op->io_status = -EIO;
	}

	if (--op->io_remaining == 0) {
		rmw_start_commit(op);
	}
}

/* ===== Step 6/7：PPL commit → COMPLETE ===== */

static void
rmw_start_commit(struct rmw_op *op)
{
	if (!rmw_can_proceed(op)) {
		SPDK_WARNLOG("rmw: raid offline before commit, abort op=%p\n", op);
		rmw_op_finish(op, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}
	if (op->io_status != 0) {
		/* flush 失败：不 commit（record 留 uncommitted，
		 * recovery 按 data 状态三分支处理）*/
		SPDK_ERRLOG("rmw: flush failed (%d), skip commit op=%p\n",
			    op->io_status, op);
		rmw_op_finish(op, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	op->state = RMW_S_PPL_COMMIT;
	if (op->ppl_seq != 0 && op->ppl_ctx != NULL && op->ppl_ch != NULL) {
		poweraid_raid_common_ppl_commit(op->ppl_ctx, op->ppl_ch, op->ppl_seq,
					   rmw_ppl_commit_done, op);
	} else {
		/* 无 PPL record（append 失败或无 ppl_ctx）：直接完成 */
		rmw_ppl_commit_done(0, op);
	}
}

static void
rmw_ppl_commit_done(int status, void *cb_arg)
{
	struct rmw_op *op = cb_arg;
	enum spdk_bdev_io_status io_status;

	SPDK_DEBUGLOG(raid5f_rmw, "rmw commit_done: op=%p status=%d io_status=%d\n",
		      op, status, op->io_status);

	if (status != 0) {
		SPDK_ERRLOG("rmw: ppl commit failed (%d)\n", status);
	}

	/* 最终状态：base IO 失败则 FAILED，否则 SUCCESS（PPL 失败不影响数据，仅降级保护）*/
	io_status = (op->io_status == 0) ? SPDK_BDEV_IO_STATUS_SUCCESS :
		    SPDK_BDEV_IO_STATUS_FAILED;
	rmw_op_finish(op, io_status);
}

/* ===== 内部初始化：分配 op + 缓冲 + PPL ctx（不启动状态机）===== */

static int
rmw_op_alloc(struct rmw_op *op)
{
	uint32_t num_modified = op->num_modified;
	uint32_t strip_bytes = op->strip_bytes;

	op->old_data_buf = spdk_dma_malloc((size_t)num_modified * strip_bytes,
					   0x1000, NULL);
	if (op->old_data_buf == NULL) {
		return -ENOMEM;
	}
	op->new_data_buf = spdk_dma_malloc((size_t)num_modified * strip_bytes,
					   0x1000, NULL);
	if (op->new_data_buf == NULL) {
		spdk_dma_free(op->old_data_buf);
		op->old_data_buf = NULL;
		return -ENOMEM;
	}
	op->parity_buf = spdk_dma_malloc(strip_bytes, 0x1000, NULL);
	if (op->parity_buf == NULL) {
		spdk_dma_free(op->new_data_buf);
		op->new_data_buf = NULL;
		spdk_dma_free(op->old_data_buf);
		op->old_data_buf = NULL;
		return -ENOMEM;
	}

	/* RAID6: Q parity 缓冲 */
	if (op->raid->num_parity > 1) {
		op->q_buf = spdk_dma_malloc(strip_bytes, 0x1000, NULL);
		if (op->q_buf == NULL) {
			spdk_dma_free(op->parity_buf);
			op->parity_buf = NULL;
			spdk_dma_free(op->new_data_buf);
			op->new_data_buf = NULL;
			spdk_dma_free(op->old_data_buf);
			op->old_data_buf = NULL;
			return -ENOMEM;
		}
	} else {
		op->q_buf = NULL;
	}

	rmw_find_ppl(op);
	return 0;
}

/* ===== 入口 A：直接单 strip 写（raid_io 完成回调）===== */

int
poweraid_raid_common_rmw_submit(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	uint32_t data_chunks = raid->num_base_bdevs - raid->num_parity;
	uint32_t stripe_blocks = raid->strip_size * data_chunks;
	uint64_t stripe_index = raid_io->offset_blocks / stripe_blocks;
	uint64_t stripe_offset = raid_io->offset_blocks % stripe_blocks;
	uint32_t strip_size_bytes = raid->strip_size * raid->block_size;
	uint32_t start_chunk = stripe_offset / raid->strip_size;
	uint32_t num_modified;
	bool is_sub_strip;
	uint8_t p_idx, q_idx;
	poweraid_raid_common_get_parity_idx(raid, stripe_index, &p_idx, &q_idx);
	uint64_t chunk_bitmap = 0;
	struct rmw_op *op;
	uint32_t i;
	int rc;

	/* sub-strip 写判断：非 strip 对齐时走 sub-strip 路径（issue #14）*/
	is_sub_strip = (stripe_offset % raid->strip_size != 0 ||
			raid_io->num_blocks % raid->strip_size != 0);
	if (is_sub_strip) {
		/* sub-strip 写只涉及 1 个 chunk（框架 split_on_optimal_io_boundary 保证不跨 strip）*/
		num_modified = 1;
	} else {
		num_modified = raid_io->num_blocks / raid->strip_size;
	}

	/* 校验几何 */
	if (is_sub_strip) {
		/* sub-strip：raid_io 必须在单个 strip 范围内 */
		if (raid_io->num_blocks > raid->strip_size) {
			SPDK_ERRLOG("rmw: sub-strip num_blocks too large=%"PRIu64"\n",
				    raid_io->num_blocks);
			return -EINVAL;
		}
	} else {
		if (num_modified == 0 || num_modified >= data_chunks) {
			SPDK_ERRLOG("rmw: invalid num_modified=%u (data_chunks=%u)\n",
				    num_modified, data_chunks);
			return -EINVAL;
		}
	}

	/* 重建窗口门控：stripe 未被重构覆盖前禁止 RMW（读旧数据/写新校验都会
	 * 与引擎竞争目标盘）。-ENOMEM 让 bdev 层排队稍后重试整个 raid_io。
	 * 已越过窗口时成员盘 channel 走框架 shadow channel（目标槽位路由 target_ch）。*/
	struct raid_bdev_io_channel *eff_ch;
	enum poweraid_raid_common_gate gate;

	gate = poweraid_raid_common_rebuild_gate_classify(raid,
			raid_io->raid_ch, stripe_index, &eff_ch);
	if (gate == POWERAID_RAID_COMMON_GATE_WAIT) {
		SPDK_DEBUGLOG(raid5f_rmw, "rmw gated by rebuild window stripe=%"PRIu64"\n",
			      stripe_index);
		return -ENOMEM;
	}

	/* 构建 chunk_bitmap（连续 range）*/
	for (i = 0; i < num_modified; i++) {
		chunk_bitmap |= (1ULL << (start_chunk + i));
	}

	op = calloc(1, sizeof(*op));
	if (op == NULL) {
		SPDK_ERRLOG("rmw: alloc op failed\n");
		return -ENOMEM;
	}
	op->raid_io = raid_io;
	op->raid_ch = eff_ch;
	op->raid = raid;
	op->raid_bdev = raid_bdev;
	op->stripe_index = stripe_index;
	op->p_idx = p_idx;
	op->q_idx = q_idx;
	op->chunk_bitmap = chunk_bitmap;
	op->num_modified = num_modified;
	op->strip_bytes = strip_size_bytes;
	op->complete_cb = NULL;
	op->complete_cb_arg = NULL;
	op->is_sub_strip = is_sub_strip;
	if (is_sub_strip) {
		op->sub_strip_offset_bytes =
			(stripe_offset % raid->strip_size) * raid->block_size;
		op->sub_strip_len_bytes = raid_io->num_blocks * raid->block_size;
	}

	SPDK_DEBUGLOG(raid5f_rmw, "rmw submit: op=%p stripe=%"PRIu64" p_idx=%u"
		       " bitmap=0x%"PRIx64" num_modified=%u sub_strip=%d\n",
		       op, stripe_index, p_idx, chunk_bitmap, num_modified, is_sub_strip);

	rc = rmw_op_alloc(op);
	if (rc != 0) {
		free(op);
		return rc;
	}

	if (is_sub_strip) {
		/* sub-strip 写：不在 submit 时拷贝 raid_io 数据（只有 sub-strip 部分），
		 * 在读旧 data 完成后（rmw_calc_and_next）用旧 data 填充 new_data_buf
		 * 再覆盖 sub-strip 新数据。*/
	} else {
		/* strip 对齐写：从 raid_io iovs 拷贝新数据到 new_data_buf */
		rc = (int)spdk_iovcpy(raid_io->iovs, raid_io->iovcnt,
			&(struct iovec){ .iov_base = op->new_data_buf,
					 .iov_len = (size_t)num_modified * strip_size_bytes },
			1);
		if (rc != (int)((size_t)num_modified * strip_size_bytes)) {
			SPDK_ERRLOG("rmw: iovcpy short copied=%d expect=%zu\n",
				    rc, (size_t)num_modified * strip_size_bytes);
			rmw_op_free(op);
			return -EIO;
		}
	}

	/* 启动状态机：Step 1 读旧 data + 旧 parity */
	rmw_start_reads(op);
	return 0;
}

/* ===== 入口 B：合并层调用（自定义完成回调，非连续 chunk_bitmap）===== */

int
poweraid_raid_common_rmw_submit_merged(
	struct raid_bdev_io_channel *raid_ch,
	struct poweraid_raid_common_raid *raid,
	uint64_t stripe_index,
	uint64_t chunk_bitmap,
	void **new_chunk_bufs,
	void (*cb)(int status, void *cb_arg),
	void *cb_arg)
{
	uint32_t data_chunks = raid->num_base_bdevs - raid->num_parity;
	uint8_t p_idx, q_idx;
	poweraid_raid_common_get_parity_idx(raid, stripe_index, &p_idx, &q_idx);
	uint32_t strip_size_bytes = raid->strip_size * raid->block_size;
	uint32_t num_modified = (uint32_t)__builtin_popcountll(chunk_bitmap);
	struct rmw_op *op;
	uint64_t bits;
	uint32_t buf_idx;
	int rc;

	if (chunk_bitmap == 0 || num_modified >= data_chunks) {
		SPDK_ERRLOG("rmw_merged: invalid bitmap=0x%"PRIx64" num=%u dc=%u\n",
			    chunk_bitmap, num_modified, data_chunks);
		return -EINVAL;
	}

	op = calloc(1, sizeof(*op));
	if (op == NULL) {
		return -ENOMEM;
	}
	op->raid_io = NULL;
	op->raid_ch = raid_ch;
	op->raid = raid;
	op->raid_bdev = raid->raid_bdev;
	op->stripe_index = stripe_index;
	op->p_idx = p_idx;
	op->q_idx = q_idx;
	op->chunk_bitmap = chunk_bitmap;
	op->num_modified = num_modified;
	op->strip_bytes = strip_size_bytes;
	op->complete_cb = cb;
	op->complete_cb_arg = cb_arg;

	SPDK_DEBUGLOG(raid5f_rmw, "rmw_merged: op=%p stripe=%"PRIu64" p_idx=%u"
		       " bitmap=0x%"PRIx64" num_modified=%u\n",
		       op, stripe_index, p_idx, chunk_bitmap, num_modified);

	rc = rmw_op_alloc(op);
	if (rc != 0) {
		free(op);
		return rc;
	}

	/* 从 new_chunk_bufs 拷贝新数据到 new_data_buf（按 bit 升序）*/
	bits = chunk_bitmap;
	buf_idx = 0;
	while (bits) {
		uint32_t chunk = __builtin_ctzll(bits);
		bits &= bits - 1;
		memcpy((char *)op->new_data_buf + buf_idx * strip_size_bytes,
		       new_chunk_bufs[chunk], strip_size_bytes);
		buf_idx++;
	}

	/* 启动状态机 */
	rmw_start_reads(op);
	return 0;
}
