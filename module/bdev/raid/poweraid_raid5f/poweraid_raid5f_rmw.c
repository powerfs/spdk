/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   RMW（Read-Modify-Write）部分 stripe 写路径实现。
 *   详见 poweraid_raid5f_rmw.h 与 raid5f-enhanced-design.md 第 3.2 节。
 *
 *   op + 回调状态机（参考 parity_fixup 模式），与 REQ FSM 独立：
 *     submit → READ_OLD(data+parity 并行) → CALC → PPL_APPEND
 *            → WRITE(data+parity 并行) → FLUSH(并行) → PPL_COMMIT → COMPLETE
 *
 *   ENOMEM：简化为失败（与 write_full 一致），由 bdev 层重试整个 raid_io。
 *     PPL record 已 append 但未 commit，recovery 三分支判定处理（data==old → NONE）。
 *
 *   几何（框架 split 已保证）：
 *     - stripe_offset = offset_blocks % stripe_blocks，strip 对齐
 *     - num_blocks = num_modified * strip_size，num_modified < data_chunks
 *     - 被改 data chunk 连续：[start_chunk, start_chunk + num_modified)
 *     - data_idx → 物理盘映射：phys = (data_idx < p_idx) ? data_idx : data_idx + 1
 *       （left-symmetric，与 recovery_read_strip 一致）
 */

#include "spdk/stdinc.h"
#include "spdk/log.h"
#include "spdk/string.h"
#include "spdk/util.h"
#include "spdk/bdev_module.h"

#include "../bdev_raid.h"
#include "poweraid_raid5f.h"
#include "poweraid_raid5f_rmw.h"
#include "poweraid_raid5f_ppl.h"

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
	struct raid_bdev_io			*raid_io;
	struct poweraid_raid5f_raid		*raid;

	uint64_t				stripe_index;
	uint8_t					p_idx;		/* parity 物理盘号 */
	uint32_t				start_chunk;	/* 第一个被改 data chunk 序号 */
	uint32_t				num_modified;	/* 被改 data chunk 数 */
	uint64_t				chunk_bitmap;	/* PPL record 用 */

	uint32_t				strip_bytes;

	/* 缓冲：old_data/new_data 按 modified 序号（0..num_modified-1）排列 */
	void					*old_data_buf;	/* num_modified * strip_bytes */
	void					*new_data_buf;	/* num_modified * strip_bytes（从 raid_io iovs 拷贝）*/
	void					*parity_buf;	/* strip_bytes：读旧 parity → 原地 XOR 成新 parity */

	/* PPL */
	struct poweraid_raid5f_ppl_ctx		*ppl_ctx;
	struct spdk_io_channel			*ppl_ch;
	uint64_t				ppl_seq;	/* 0 表示无 PPL 保护 */

	/* 状态机 */
	enum rmw_state				state;
	uint32_t				io_remaining;
	int					io_status;
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
	free(op);
}

static void
rmw_op_finish(struct rmw_op *op, enum spdk_bdev_io_status status)
{
	struct raid_bdev_io *raid_io = op->raid_io;

	SPDK_DEBUGLOG(raid5f_rmw, "rmw finish: op=%p status=%u stripe=%"PRIu64
		      " start_chunk=%u num_modified=%u\n",
		      op, status, op->stripe_index, op->start_chunk, op->num_modified);
	rmw_op_free(op);
	raid_bdev_io_complete(raid_io, status);
}

/* ===== 几何辅助 ===== */

/* data chunk 序号 → 物理盘号（left-symmetric）*/
static inline uint8_t
rmw_data_to_phys(uint8_t data_idx, uint8_t p_idx)
{
	return (data_idx < p_idx) ? data_idx : (data_idx + 1);
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
	struct raid_bdev_io *raid_io = op->raid_io;
	struct poweraid_raid5f_raid *raid = op->raid;
	uint8_t i;

	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL &&
		    raid->base_bdevs[i]->ppl_ctx != NULL) {
			op->ppl_ctx = raid->base_bdevs[i]->ppl_ctx;
			op->ppl_ch = raid_bdev_channel_get_base_channel(
				raid_io->raid_ch, i);
			break;
		}
	}
}

/* ===== Step 1：读旧 data + 旧 parity（并行）===== */

static void
rmw_start_reads(struct rmw_op *op)
{
	struct raid_bdev_io *raid_io = op->raid_io;
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid5f_raid *raid = op->raid;
	uint64_t base_offset = raid->data_offset_blocks +
			       op->stripe_index * raid->strip_size;
	uint32_t i;
	int rc;

	op->state = RMW_S_READ_OLD;
	op->io_remaining = op->num_modified + 1; /* data chunks + parity */
	op->io_status = 0;

	SPDK_DEBUGLOG(raid5f_rmw, "rmw reads: op=%p stripe=%"PRIu64" base_offset=%"PRIu64
		      " p_idx=%u num_modified=%u\n",
		      op, op->stripe_index, base_offset, op->p_idx, op->num_modified);

	/* 读旧 data chunks */
	for (i = 0; i < op->num_modified; i++) {
		uint8_t data_idx = op->start_chunk + i;
		uint8_t phys = rmw_data_to_phys(data_idx, op->p_idx);
		struct raid_base_bdev_info *base_info = &raid_bdev->base_bdev_info[phys];
		struct spdk_io_channel *base_ch;
		struct spdk_bdev_ext_io_opts io_opts = {0};
		struct iovec iov = {
			.iov_base = (char *)op->old_data_buf + i * op->strip_bytes,
			.iov_len = op->strip_bytes,
		};

		io_opts.size = sizeof(io_opts);
		base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, phys);
		if (base_ch == NULL || base_info->desc == NULL) {
			SPDK_ERRLOG("rmw read: no channel for data chunk %u (phys %u)\n",
				    data_idx, phys);
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_calc_and_next(op);
			}
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
		p_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, op->p_idx);
		if (p_ch == NULL || p_info->desc == NULL) {
			SPDK_ERRLOG("rmw read: no channel for parity (p_idx %u)\n",
				    op->p_idx);
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_calc_and_next(op);
			}
			return;
		}

		rc = raid_bdev_readv_blocks_ext(p_info, p_ch, &iov, 1,
						base_offset, raid->strip_size,
						rmw_read_cb, op, &io_opts);
		if (rc != 0) {
			SPDK_ERRLOG("rmw read: parity failed rc=%d\n", rc);
			op->io_status = rc;
			if (--op->io_remaining == 0) {
				rmw_calc_and_next(op);
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
	struct poweraid_raid5f_ppl_hash_ctx old_h, new_h;
	uint64_t old_hash, new_hash;
	uint32_t i, j;

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

	/* 新 parity = 旧 parity ^ (旧 data[i] ^ 新 data[i]) for each modified i
	 * 原地更新 parity_buf：先已是旧 parity，XOR 进 (old ^ new) 即得新 parity。*/
	for (i = 0; i < op->num_modified; i++) {
		const uint64_t *oldp = (const uint64_t *)
			((char *)op->old_data_buf + i * op->strip_bytes);
		const uint64_t *newp = (const uint64_t *)
			((char *)op->new_data_buf + i * op->strip_bytes);
		for (j = 0; j < strip_u64; j++) {
			parity[j] ^= oldp[j] ^ newp[j];
		}
	}

	/* old/new hash：按 modified chunk 升序（与 recovery 读盘顺序一致）*/
	poweraid_raid5f_ppl_hash_init(&old_h);
	poweraid_raid5f_ppl_hash_init(&new_h);
	for (i = 0; i < op->num_modified; i++) {
		poweraid_raid5f_ppl_hash_update(&old_h,
			(char *)op->old_data_buf + i * op->strip_bytes,
			op->strip_bytes);
		poweraid_raid5f_ppl_hash_update(&new_h,
			(char *)op->new_data_buf + i * op->strip_bytes,
			op->strip_bytes);
	}
	old_hash = poweraid_raid5f_ppl_hash_final(&old_h);
	new_hash = poweraid_raid5f_ppl_hash_final(&new_h);

	SPDK_DEBUGLOG(raid5f_rmw, "rmw calc: op=%p old_hash=0x%"PRIx64
		      " new_hash=0x%"PRIx64" bitmap=0x%"PRIx64"\n",
		      op, old_hash, new_hash, op->chunk_bitmap);

	/* Step 3: PPL append（FUA intent）*/
	op->state = RMW_S_PPL_APPEND;
	if (op->ppl_ctx != NULL && op->ppl_ch != NULL) {
		poweraid_raid5f_ppl_append_record(op->ppl_ctx, op->ppl_ch,
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
		SPDK_ERRLOG("rmw: ppl append failed (%d), continue without PPL\n", status);
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
	struct raid_bdev_io *raid_io = op->raid_io;
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid5f_raid *raid = op->raid;
	uint64_t base_offset = raid->data_offset_blocks +
			       op->stripe_index * raid->strip_size;
	uint32_t i;
	int rc;

	op->state = RMW_S_WRITE;
	op->io_remaining = op->num_modified + 1; /* data chunks + parity */
	op->io_status = 0;

	for (i = 0; i < op->num_modified; i++) {
		uint8_t data_idx = op->start_chunk + i;
		uint8_t phys = rmw_data_to_phys(data_idx, op->p_idx);
		struct raid_base_bdev_info *base_info = &raid_bdev->base_bdev_info[phys];
		struct spdk_io_channel *base_ch;
		struct spdk_bdev_ext_io_opts io_opts = {0};
		struct iovec iov = {
			.iov_base = (char *)op->new_data_buf + i * op->strip_bytes,
			.iov_len = op->strip_bytes,
		};

		io_opts.size = sizeof(io_opts);
		base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, phys);
		if (base_ch == NULL || base_info->desc == NULL) {
			SPDK_ERRLOG("rmw write: no channel for data chunk %u (phys %u)\n",
				    data_idx, phys);
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_start_flushes(op);
			}
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
	}

	/* 写新 parity（p_idx 盘）*/
	{
		struct raid_base_bdev_info *p_info = &raid_bdev->base_bdev_info[op->p_idx];
		struct spdk_io_channel *p_ch;
		struct spdk_bdev_ext_io_opts io_opts = {0};
		struct iovec iov = {
			.iov_base = op->parity_buf,
			.iov_len = op->strip_bytes,
		};

		io_opts.size = sizeof(io_opts);
		p_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, op->p_idx);
		if (p_ch == NULL || p_info->desc == NULL) {
			SPDK_ERRLOG("rmw write: no channel for parity (p_idx %u)\n",
				    op->p_idx);
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_start_flushes(op);
			}
			return;
		}

		rc = raid_bdev_writev_blocks_ext(p_info, p_ch, &iov, 1,
						 base_offset, raid->strip_size,
						 rmw_write_cb, op, &io_opts);
		if (rc != 0) {
			SPDK_ERRLOG("rmw write: parity failed rc=%d\n", rc);
			op->io_status = rc;
			if (--op->io_remaining == 0) {
				rmw_start_flushes(op);
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
	struct raid_bdev_io *raid_io = op->raid_io;
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid5f_raid *raid = op->raid;
	uint64_t base_offset = raid->data_offset_blocks +
			       op->stripe_index * raid->strip_size;
	uint32_t i;
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
	op->io_remaining = op->num_modified + 1;
	op->io_status = 0;

	for (i = 0; i < op->num_modified; i++) {
		uint8_t data_idx = op->start_chunk + i;
		uint8_t phys = rmw_data_to_phys(data_idx, op->p_idx);
		struct raid_base_bdev_info *base_info = &raid_bdev->base_bdev_info[phys];
		struct spdk_io_channel *base_ch;

		base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, phys);
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

	/* flush parity 盘 */
	{
		struct raid_base_bdev_info *p_info = &raid_bdev->base_bdev_info[op->p_idx];
		struct spdk_io_channel *p_ch;

		p_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, op->p_idx);
		if (p_ch == NULL || p_info->desc == NULL) {
			op->io_status = -ENODEV;
			if (--op->io_remaining == 0) {
				rmw_start_commit(op);
			}
			return;
		}

		rc = raid_bdev_flush_blocks(p_info, p_ch, base_offset,
					    raid->strip_size,
					    rmw_flush_cb, op);
		if (rc != 0) {
			SPDK_ERRLOG("rmw flush: parity failed rc=%d\n", rc);
			op->io_status = rc;
			if (--op->io_remaining == 0) {
				rmw_start_commit(op);
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
		poweraid_raid5f_ppl_commit(op->ppl_ctx, op->ppl_ch, op->ppl_seq,
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

/* ===== 入口：提交 RMW IO ===== */

int
poweraid_raid5f_rmw_submit(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid5f_raid *raid = raid_bdev->module_private;
	uint32_t data_chunks = raid->num_base_bdevs - 1;
	uint32_t stripe_blocks = raid->strip_size * data_chunks;
	uint64_t stripe_index = raid_io->offset_blocks / stripe_blocks;
	uint64_t stripe_offset = raid_io->offset_blocks % stripe_blocks;
	uint32_t strip_size_bytes = raid->strip_size * raid->block_size;
	uint32_t start_chunk = stripe_offset / raid->strip_size;
	uint32_t num_modified = raid_io->num_blocks / raid->strip_size;
	uint8_t p_idx = data_chunks - (stripe_index % raid->num_base_bdevs);
	uint64_t chunk_bitmap = 0;
	struct rmw_op *op;
	uint32_t i;
	int rc;

	/* 校验几何（框架 split 保证，防御性断言）*/
	if (stripe_offset % raid->strip_size != 0 ||
	    raid_io->num_blocks % raid->strip_size != 0) {
		SPDK_ERRLOG("rmw: not strip-aligned offset=%"PRIu64" num=%"PRIu64"\n",
			    raid_io->offset_blocks, raid_io->num_blocks);
		return -EINVAL;
	}
	if (num_modified == 0 || num_modified >= data_chunks) {
		SPDK_ERRLOG("rmw: invalid num_modified=%u (data_chunks=%u)\n",
			    num_modified, data_chunks);
		return -EINVAL;
	}

	op = calloc(1, sizeof(*op));
	if (op == NULL) {
		SPDK_ERRLOG("rmw: alloc op failed\n");
		return -ENOMEM;
	}
	op->raid_io = raid_io;
	op->raid = raid;
	op->stripe_index = stripe_index;
	op->p_idx = p_idx;
	op->start_chunk = start_chunk;
	op->num_modified = num_modified;
	op->strip_bytes = strip_size_bytes;

	/* chunk_bitmap：被改 data chunk 位图 */
	for (i = 0; i < num_modified; i++) {
		chunk_bitmap |= (1ULL << (start_chunk + i));
	}
	op->chunk_bitmap = chunk_bitmap;

	SPDK_NOTICELOG("rmw submit: op=%p stripe=%"PRIu64" p_idx=%u"
		       " start_chunk=%u num_modified=%u bitmap=0x%"PRIx64"\n",
		       op, stripe_index, p_idx, start_chunk, num_modified,
	       chunk_bitmap);

	/* 分配缓冲 */
	op->old_data_buf = spdk_dma_malloc((size_t)num_modified * strip_size_bytes,
					   0x1000, NULL);
	if (op->old_data_buf == NULL) {
		rc = -ENOMEM;
		goto err_free_op;
	}
	op->new_data_buf = spdk_dma_malloc((size_t)num_modified * strip_size_bytes,
					   0x1000, NULL);
	if (op->new_data_buf == NULL) {
		rc = -ENOMEM;
		goto err_free_old;
	}
	op->parity_buf = spdk_dma_malloc(strip_size_bytes, 0x1000, NULL);
	if (op->parity_buf == NULL) {
		rc = -ENOMEM;
		goto err_free_new;
	}

	/* 从 raid_io iovs 拷贝新数据（modified chunks 连续，按序拷贝）*/
	rc = (int)spdk_iovcpy(raid_io->iovs, raid_io->iovcnt,
		&(struct iovec){ .iov_base = op->new_data_buf,
				 .iov_len = (size_t)num_modified * strip_size_bytes },
		1);
	if (rc != (int)((size_t)num_modified * strip_size_bytes)) {
		SPDK_ERRLOG("rmw: iovcpy short copied=%d expect=%zu\n",
			    rc, (size_t)num_modified * strip_size_bytes);
		rc = -EIO;
		goto err_free_parity;
	}

	/* 查找 PPL ctx（与 write_full 一致：首个带 ppl_ctx 的成员盘）*/
	rmw_find_ppl(op);

	/* 启动状态机：Step 1 读旧 data + 旧 parity */
	rmw_start_reads(op);
	return 0;

err_free_parity:
	spdk_dma_free(op->parity_buf);
	op->parity_buf = NULL;
err_free_new:
	spdk_dma_free(op->new_data_buf);
	op->new_data_buf = NULL;
err_free_old:
	spdk_dma_free(op->old_data_buf);
	op->old_data_buf = NULL;
err_free_op:
	free(op);
	return rc;
}
