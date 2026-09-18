/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   MWL（Mirror Write Log）实现。
 *
 *   详见 poweraid_raid_common_mwl.h 与 raid1f-mirror-write-log spec。
 *
 *   纪律与 PPL 对称（正确性优先）：
 *     - 线性日志，无回卷/回收（满则 -ENOSPC；阶段 2 测试场景容量足够）。
 *       注：与 PPL 相同，tail_slot 到顶后若 inflight 为空则回卷到 1。
 *     - 每 record 占一个 block_size slot（record 64B 置于 slot 起始，余清零），
 *       单 record 独立 FUA，无需读改写。
 *     - FUA = spdk_bdev_write_blocks + spdk_bdev_flush_blocks（bdev 无关）。
 *     - per-op DMA scratch buffer（spdk_dma_malloc），cb 中释放，避免并发冲突。
 *     - super 与 record 共享同一 MWL 区：slot 0 = super，slot 1..N = records。
 *
 *   与 PPL 的复用：data_hash 工具直接调用 poweraid_raid_common_ppl_data_hash 与
 *   poweraid_raid_common_ppl_hash_*（见 ppl.h 导出）；CRC 计算用同一 spdk_crc32c_update
 *   底层 API，但因 MWL record/super 字段布局与 PPL 不同，本文件定义自己的打包/校验函数。
 *
 *   TODO（后续阶段）：64B 紧凑打包 + 批量 FUA + 环形回卷回收，降低写放大。
 */

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/crc32.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/util.h"

#include "poweraid_raid_common_mwl.h"
#include "poweraid_raid_common_sb.h"   /* POWERAID_RAID1F_LEAD_SLOT */
#include "poweraid_raid_common_ppl.h"  /* 复用 data_hash 工具 */

SPDK_LOG_REGISTER_COMPONENT(raid_mwl);

/* ===== 内部上下文 ===== */

struct poweraid_raid_common_mwl_ctx {
	void				*bdev_desc;     /* struct spdk_bdev_desc* */
	struct spdk_io_channel		*ch;
	uint32_t			block_size;
	uint64_t			region_offset;  /* 字节，MWL 区在盘上的起始 */
	uint64_t			region_size;     /* 字节 */
	uint32_t			block_shift;     /* log2(block_size) */
	uint32_t			max_slots;       /* region 内 block 槽位数（含 super slot 0）*/

	/* 内存 super 镜像（append/commit 时更新并写盘；load_replay 时从盘加载）*/
	struct poweraid_raid_common_mwl_super	super;

	/* 线性日志写指针：下一条 record 写入的 slot 下标（1..max_slots-1；0=super）*/
	uint32_t			tail_slot;

	/* in-flight（已 append 待 commit）记录链表，保证 commit 顺序 */
	TAILQ_HEAD(, poweraid_raid_common_mwl_inflight)	inflight;
};

/* ===== append 异步操作 ===== */
enum mwl_append_state {
	MWL_APPEND_WRITE = 0,
	MWL_APPEND_FLUSH,
	MWL_APPEND_DONE,
};

struct mwl_append_op {
	struct poweraid_raid_common_mwl_ctx	*ctx;
	struct spdk_io_channel		*ch;   /* 调用方线程的 bdev channel（数据面）*/
	struct poweraid_raid_common_mwl_record rec;
	uint8_t				state;
	int				status;
	uint64_t			seq;
	uint32_t			slot;            /* 写入的 slot 下标 */
	uint64_t			slot_byte_off;    /* slot 在盘上的字节偏移 */
	void				*buf;             /* block_size DMA scratch */
	poweraid_raid_common_mwl_append_cb	cb;
	void				*cb_arg;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== commit 异步操作 ===== */
enum mwl_commit_state {
	MWL_COMMIT_WRITE = 0,
	MWL_COMMIT_FLUSH,
	MWL_COMMIT_DONE,
};

struct mwl_commit_op {
	struct poweraid_raid_common_mwl_ctx	*ctx;
	struct spdk_io_channel		*ch;
	uint8_t				state;
	int				status;
	uint64_t			commit_seq;
	void				*buf;             /* block_size DMA scratch（写 super）*/
	poweraid_raid_common_mwl_commit_cb	cb;
	void				*cb_arg;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== init 异步操作 ===== */
enum mwl_init_state {
	MWL_INIT_WRITE = 0,
	MWL_INIT_FLUSH,
	MWL_INIT_DONE,
};

struct mwl_init_op {
	struct poweraid_raid_common_mwl_ctx	*ctx;
	uint8_t				state;
	int				status;
	void				*buf;             /* block_size DMA scratch（写 super）*/
	poweraid_raid_common_mwl_init_cb	cb;
	void				*cb_arg;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== load_replay 异步操作 ===== */
struct mwl_replay_op {
	struct poweraid_raid_common_mwl_ctx	*ctx;
	poweraid_raid_common_mwl_replay_cb	cb;
	void				*cb_arg;
	void				*buf;             /* 当前 chunk 读缓冲 */
	uint32_t			buf_blocks;       /* buf 容纳的 block 数 */
	uint32_t			cur_slot;         /* 下一个待读 slot */
	uint32_t			end_slot;         /* 扫描终止 slot */
	struct poweraid_raid_common_mwl_record *result;
	uint32_t			result_cap;
	uint32_t			result_cnt;
	uint32_t			bad_record_cnt;   /* 坏 record 计数（CRC/magic 失败）*/
	int				status;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* super 读专用 op（load_replay 第一阶段）*/
struct mwl_super_load_op {
	struct poweraid_raid_common_mwl_ctx	*ctx;
	poweraid_raid_common_mwl_replay_cb	cb;
	void				*cb_arg;
	void				*buf;             /* block_size，读 super */
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== 内部辅助 ===== */

static inline uint64_t
slot_to_byte_offset(struct poweraid_raid_common_mwl_ctx *ctx, uint32_t slot)
{
	return ctx->region_offset + (uint64_t)slot * ctx->block_size;
}

static inline uint64_t
byte_to_block_offset(struct poweraid_raid_common_mwl_ctx *ctx, uint64_t byte_off)
{
	return byte_off >> ctx->block_shift;
}

/* crc 覆盖 record 除 crc32c 字段外全部字节（两段：crc32c 前一段 + crc32c 后一段）*/
static uint32_t
mwl_rec_calc_crc(const struct poweraid_raid_common_mwl_record *rec)
{
	uint32_t crc;
	crc = spdk_crc32c_update(rec,
				 offsetof(struct poweraid_raid_common_mwl_record, crc32c), 0);
	crc = spdk_crc32c_update((const uint8_t *)rec +
				 offsetof(struct poweraid_raid_common_mwl_record, crc32c) +
				 sizeof(uint32_t),
				 sizeof(*rec) -
				 offsetof(struct poweraid_raid_common_mwl_record, crc32c) -
				 sizeof(uint32_t),
				 crc);
	return crc;
}

static inline void
mwl_rec_set_crc(struct poweraid_raid_common_mwl_record *rec)
{
	rec->crc32c = 0;
	rec->crc32c = mwl_rec_calc_crc(rec);
}

static inline bool
mwl_rec_check(const struct poweraid_raid_common_mwl_record *rec)
{
	if (rec->magic != POWERAID_RAID_COMMON_MWL_REC_MAGIC) {
		return false;
	}
	return mwl_rec_calc_crc(rec) == rec->crc32c;
}

static uint32_t
mwl_super_calc_crc(const struct poweraid_raid_common_mwl_super *s)
{
	uint32_t crc;
	crc = spdk_crc32c_update(s,
				 offsetof(struct poweraid_raid_common_mwl_super, crc32c), 0);
	crc = spdk_crc32c_update((const uint8_t *)s +
				offsetof(struct poweraid_raid_common_mwl_super, crc32c) +
				sizeof(uint32_t),
				sizeof(*s) -
				offsetof(struct poweraid_raid_common_mwl_super, crc32c) -
				sizeof(uint32_t),
				crc);
	return crc;
}

static inline void
mwl_super_set_crc(struct poweraid_raid_common_mwl_super *s)
{
	s->crc32c = 0;
	s->crc32c = mwl_super_calc_crc(s);
}

static inline bool
mwl_super_check(const struct poweraid_raid_common_mwl_super *s)
{
	if (s->magic != POWERAID_RAID_COMMON_MWL_SUPER_MAGIC) {
		return false;
	}
	return mwl_super_calc_crc(s) == s->crc32c;
}

static inline int
block_shift_of(uint32_t block_size)
{
	switch (block_size) {
	case 512:   return 9;
	case 1024:  return 10;
	case 2048:  return 11;
	case 4096:  return 12;
	case 8192:  return 13;
	default:    return -1;
	}
}

/* ===== 公共 API：alloc / free ===== */

struct poweraid_raid_common_mwl_ctx *
poweraid_raid_common_mwl_alloc(void *bdev_desc, struct spdk_io_channel *ch,
			  uint32_t block_size,
			  uint64_t region_offset, uint64_t region_size)
{
	struct poweraid_raid_common_mwl_ctx *ctx;
	int shift;

	if (bdev_desc == NULL || ch == NULL || block_size == 0 ||
	    region_size < (uint64_t)block_size * 2 || region_offset == 0) {
		SPDK_ERRLOG("mwl_alloc: invalid args (bs=%u rs=%"PRIu64" off=%"PRIu64")\n",
			    block_size, region_size, region_offset);
		return NULL;
	}
	shift = block_shift_of(block_size);
	if (shift < 0) {
		SPDK_ERRLOG("mwl_alloc: unsupported block_size %u\n", block_size);
		return NULL;
	}

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		SPDK_ERRLOG("mwl_alloc: calloc ctx failed\n");
		return NULL;
	}
	ctx->bdev_desc = bdev_desc;
	ctx->ch = ch;
	ctx->block_size = block_size;
	ctx->block_shift = (uint32_t)shift;
	ctx->region_offset = region_offset;
	ctx->region_size = region_size;
	ctx->max_slots = region_size / block_size;
	ctx->tail_slot = 1;  /* slot 0 = super，record 从 slot 1 起 */
	TAILQ_INIT(&ctx->inflight);

	/* 内存 super 初始（写盘由 mwl_init 触发；加载由 mwl_load_replay 触发）*/
	memset(&ctx->super, 0, sizeof(ctx->super));
	ctx->super.magic = POWERAID_RAID_COMMON_MWL_SUPER_MAGIC;
	ctx->super.version = 1;
	ctx->super.head_seq = 1;
	ctx->super.tail_seq = 1;
	ctx->super.commit_seq = 0;
	ctx->super.next_seq = 1;
	ctx->super.acting_lead_slot = 0;  /* 默认 slot0 为 lead */
	ctx->super.num_records = 0;

	return ctx;
}

void
poweraid_raid_common_mwl_free(struct poweraid_raid_common_mwl_ctx *ctx)
{
	struct poweraid_raid_common_mwl_inflight *inf;

	if (ctx == NULL) {
		return;
	}
	while ((inf = TAILQ_FIRST(&ctx->inflight)) != NULL) {
		TAILQ_REMOVE(&ctx->inflight, inf, link);
		free(inf);
	}
	free(ctx);
}

uint8_t
poweraid_raid_common_mwl_get_acting_lead_slot(struct poweraid_raid_common_mwl_ctx *ctx)
{
	return ctx ? (uint8_t)ctx->super.acting_lead_slot : 0;
}

void
poweraid_raid_common_mwl_set_acting_lead_slot(struct poweraid_raid_common_mwl_ctx *ctx, uint8_t slot)
{
	if (ctx == NULL) {
		return;
	}
	ctx->super.acting_lead_slot = (uint64_t)slot;
}

uint64_t
poweraid_raid_common_mwl_get_commit_seq(struct poweraid_raid_common_mwl_ctx *ctx)
{
	return ctx ? ctx->super.commit_seq : 0;
}

/* ===== 通用 wait helper：资源可用时重新进入对应 loop 重试提交 ===== */
static void
mwl_arm_wait(struct spdk_bdev_io_wait_entry *w, void *bdev_desc,
	     struct spdk_io_channel *ch, spdk_bdev_io_wait_cb retry_fn, void *op)
{
	w->bdev = spdk_bdev_desc_get_bdev(bdev_desc);
	w->cb_fn = retry_fn;
	w->cb_arg = op;
	spdk_bdev_queue_io_wait(bdev_desc, ch, w);
}

/* ===== init：格式化 MWL 区（写 super + FUA）===== */

static void mwl_init_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void mwl_init_loop(struct mwl_init_op *op);

static void
mwl_init_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct mwl_init_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		op->status = -EIO;
		op->state = MWL_INIT_DONE;
		mwl_init_loop(op);
		return;
	}
	op->state++;
	mwl_init_loop(op);
}

static void
mwl_init_loop(struct mwl_init_op *op)
{
	struct poweraid_raid_common_mwl_ctx *ctx = op->ctx;
	int rc;

	while (op->state < MWL_INIT_DONE) {
		switch (op->state) {
		case MWL_INIT_WRITE:
			memcpy(op->buf, &ctx->super, sizeof(ctx->super));
			rc = spdk_bdev_write_blocks(ctx->bdev_desc, ctx->ch, op->buf,
						    byte_to_block_offset(ctx, ctx->region_offset),
						    1, mwl_init_io_cb, op);
			if (rc == -ENOMEM) {
				mwl_arm_wait(&op->wait_entry, ctx->bdev_desc, ctx->ch,
					     (spdk_bdev_io_wait_cb)mwl_init_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = MWL_INIT_DONE;
				continue;
			}
			return;  /* 等 cb */

		case MWL_INIT_FLUSH:
			rc = spdk_bdev_flush_blocks(ctx->bdev_desc, ctx->ch,
						    byte_to_block_offset(ctx, ctx->region_offset),
						    1, mwl_init_io_cb, op);
			if (rc == -ENOMEM) {
				mwl_arm_wait(&op->wait_entry, ctx->bdev_desc, ctx->ch,
					     (spdk_bdev_io_wait_cb)mwl_init_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = MWL_INIT_DONE;
				continue;
			}
			return;  /* 等 cb */

		default:
			op->status = -EINVAL;
			op->state = MWL_INIT_DONE;
			break;
		}
	}

	/* DONE */
	if (op->status == 0) {
		SPDK_NOTICELOG("poweraid_raid_common_mwl: init ok (super @ slot 0, acting_lead=%u)\n",
			       (unsigned)ctx->super.acting_lead_slot);
	}
	op->cb(op->status, op->cb_arg);
	spdk_free(op->buf);
	free(op);
}

void
poweraid_raid_common_mwl_init(struct poweraid_raid_common_mwl_ctx *ctx,
			 poweraid_raid_common_mwl_init_cb cb, void *cb_arg)
{
	struct mwl_init_op *op;

	if (ctx == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, cb_arg); }
		return;
	}

	op = calloc(1, sizeof(*op));
	if (!op) {
		cb(-ENOMEM, cb_arg);
		return;
	}
	op->ctx = ctx;
	op->cb = cb;
	op->cb_arg = cb_arg;
	op->status = 0;
	op->state = MWL_INIT_WRITE;

	op->buf = spdk_dma_malloc(ctx->block_size, 0x1000, NULL);
	if (!op->buf) {
		free(op);
		cb(-ENOMEM, cb_arg);
		return;
	}
	memset(op->buf, 0, ctx->block_size);

	mwl_super_set_crc(&ctx->super);
	mwl_init_loop(op);
}

/* ===== append：写 record + FUA ===== */

static void mwl_append_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void mwl_append_loop(struct mwl_append_op *op);

static void
mwl_append_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct mwl_append_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		op->status = -EIO;
		op->state = MWL_APPEND_DONE;
		mwl_append_loop(op);
		return;
	}
	op->state++;
	mwl_append_loop(op);
}

static void
mwl_append_loop(struct mwl_append_op *op)
{
	struct poweraid_raid_common_mwl_ctx *ctx = op->ctx;
	int rc;

	while (op->state < MWL_APPEND_DONE) {
		switch (op->state) {
		case MWL_APPEND_WRITE:
			rc = spdk_bdev_write_blocks(ctx->bdev_desc, op->ch, op->buf,
						    byte_to_block_offset(ctx, op->slot_byte_off),
						    1, mwl_append_io_cb, op);
			if (rc == -ENOMEM) {
				mwl_arm_wait(&op->wait_entry, ctx->bdev_desc, op->ch,
					     (spdk_bdev_io_wait_cb)mwl_append_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = MWL_APPEND_DONE;
				continue;
			}
			return;  /* 等 cb */

		case MWL_APPEND_FLUSH:
			rc = spdk_bdev_flush_blocks(ctx->bdev_desc, op->ch,
						    byte_to_block_offset(ctx, op->slot_byte_off),
						    1, mwl_append_io_cb, op);
			if (rc == -ENOMEM) {
				mwl_arm_wait(&op->wait_entry, ctx->bdev_desc, op->ch,
					     (spdk_bdev_io_wait_cb)mwl_append_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = MWL_APPEND_DONE;
				continue;
			}
			return;  /* 等 cb */

		default:
			op->status = -EINVAL;
			op->state = MWL_APPEND_DONE;
			break;
		}
	}

	/* DONE */
	if (op->status == 0) {
		struct poweraid_raid_common_mwl_inflight *inf;
		ctx->super.tail_seq = op->seq + 1;
		ctx->super.next_seq = op->seq + 1;
		ctx->super.num_records++;
		ctx->tail_slot++;
		inf = calloc(1, sizeof(*inf));
		if (inf) {
			inf->seq = op->seq;
			inf->lba = op->rec.lba;
			inf->num_blocks = op->rec.num_blocks;
			inf->acting_slot = op->rec.acting_slot;
			TAILQ_INSERT_TAIL(&ctx->inflight, inf, link);
		}
		op->cb(0, op->seq, op->cb_arg);
	} else {
		op->cb(op->status, 0, op->cb_arg);
	}
	spdk_free(op->buf);
	free(op);
}

void
poweraid_raid_common_mwl_append_record(struct poweraid_raid_common_mwl_ctx *ctx,
				  struct spdk_io_channel *ch,
				  uint64_t lba, uint64_t num_blocks,
				  uint64_t old_data_hash, uint64_t new_data_hash,
				  uint8_t acting_slot,
				  poweraid_raid_common_mwl_append_cb cb, void *cb_arg)
{
	struct mwl_append_op *op;
	uint64_t seq;
	uint32_t slot;
	uint32_t flags;

	if (ctx == NULL || ch == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, 0, cb_arg); }
		return;
	}
	if (ctx->tail_slot >= ctx->max_slots) {
		/* 环形回收：所有已 append 的 record 均已 commit（inflight 空）后，
		 * slot 回卷到 1。seq 仍由 super.next_seq 单调分配，与 slot 解耦；
		 * 盘上未及覆盖的陈旧 record 在 replay 时按 seq <= commit_seq 过滤。*/
		if (!TAILQ_EMPTY(&ctx->inflight)) {
			SPDK_ERRLOG("mwl_append: log full with inflight (ctx=%p tail=%u max=%u)\n",
				    ctx, ctx->tail_slot, ctx->max_slots);
			cb(-ENOSPC, 0, cb_arg);
			return;
		}
		SPDK_NOTICELOG("poweraid_raid_common_mwl: ring recycle (next_seq=%"PRIu64")\n",
			       ctx->super.next_seq);
		ctx->tail_slot = 1;
		/* head_seq 推进到当前 commit_seq+1，表示已 commit 的全部回收 */
		ctx->super.head_seq = ctx->super.commit_seq + 1;
	}

	seq = ctx->super.next_seq;
	slot = ctx->tail_slot;

	op = calloc(1, sizeof(*op));
	if (!op) {
		cb(-ENOMEM, 0, cb_arg);
		return;
	}
	op->ctx = ctx;
	op->ch = ch;
	op->cb = cb;
	op->cb_arg = cb_arg;
	op->seq = seq;
	op->slot = slot;
	op->slot_byte_off = slot_to_byte_offset(ctx, slot);
	op->state = MWL_APPEND_WRITE;
	op->status = 0;

	flags = POWERAID_RAID_COMMON_MWL_REC_F_VALID;
	if (acting_slot != POWERAID_RAID1F_LEAD_SLOT) {
		flags |= POWERAID_RAID_COMMON_MWL_REC_F_ACTING_LEAD;
	}

	memset(&op->rec, 0, sizeof(op->rec));
	op->rec.magic = POWERAID_RAID_COMMON_MWL_REC_MAGIC;
	op->rec.flags = flags;
	op->rec.seq = seq;
	op->rec.lba = lba;
	op->rec.num_blocks = num_blocks;
	op->rec.old_data_hash = old_data_hash;
	op->rec.new_data_hash = new_data_hash;
	op->rec.acting_slot = acting_slot;
	op->rec.ts = (uint64_t)time(NULL);
	mwl_rec_set_crc(&op->rec);

	/* 同步内存 super 的 acting_lead_slot，确保 append 后 commit 时一致 */
	ctx->super.acting_lead_slot = (uint64_t)acting_slot;

	op->buf = spdk_dma_malloc(ctx->block_size, 0x1000, NULL);
	if (!op->buf) {
		free(op);
		cb(-ENOMEM, 0, cb_arg);
		return;
	}
	memset(op->buf, 0, ctx->block_size);
	memcpy(op->buf, &op->rec, sizeof(op->rec));

	mwl_append_loop(op);
}

/* ===== commit：更新 super.commit_seq + FUA ===== */

static void mwl_commit_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void mwl_commit_loop(struct mwl_commit_op *op);

static void
mwl_commit_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct mwl_commit_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		op->status = -EIO;
		op->state = MWL_COMMIT_DONE;
	}
	op->state++;
	mwl_commit_loop(op);
}

static void
mwl_commit_loop(struct mwl_commit_op *op)
{
	struct poweraid_raid_common_mwl_ctx *ctx = op->ctx;
	int rc;

	while (op->state < MWL_COMMIT_DONE) {
		switch (op->state) {
		case MWL_COMMIT_WRITE:
			memcpy(op->buf, &ctx->super, sizeof(ctx->super));
			rc = spdk_bdev_write_blocks(ctx->bdev_desc, op->ch, op->buf,
						    byte_to_block_offset(ctx, ctx->region_offset),
						    1, mwl_commit_io_cb, op);
			if (rc == -ENOMEM) {
				mwl_arm_wait(&op->wait_entry, ctx->bdev_desc, op->ch,
					     (spdk_bdev_io_wait_cb)mwl_commit_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = MWL_COMMIT_DONE;
				continue;
			}
			return;

		case MWL_COMMIT_FLUSH:
			rc = spdk_bdev_flush_blocks(ctx->bdev_desc, op->ch,
						    byte_to_block_offset(ctx, ctx->region_offset),
						    1, mwl_commit_io_cb, op);
			if (rc == -ENOMEM) {
				mwl_arm_wait(&op->wait_entry, ctx->bdev_desc, op->ch,
					     (spdk_bdev_io_wait_cb)mwl_commit_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = MWL_COMMIT_DONE;
				continue;
			}
			return;

		default:
			op->status = -EINVAL;
			op->state = MWL_COMMIT_DONE;
			break;
		}
	}

	/* DONE */
	if (op->status == 0) {
		struct poweraid_raid_common_mwl_inflight *inf, *tmp;
		TAILQ_FOREACH_SAFE(inf, &ctx->inflight, link, tmp) {
			if (inf->seq <= op->commit_seq) {
				TAILQ_REMOVE(&ctx->inflight, inf, link);
				free(inf);
			}
		}
	}
	op->cb(op->status, op->cb_arg);
	spdk_free(op->buf);
	free(op);
}

void
poweraid_raid_common_mwl_commit(struct poweraid_raid_common_mwl_ctx *ctx,
			  struct spdk_io_channel *ch, uint64_t seq,
			  poweraid_raid_common_mwl_commit_cb cb, void *cb_arg)
{
	struct mwl_commit_op *op;

	if (ctx == NULL || ch == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, cb_arg); }
		return;
	}
	if (seq < ctx->super.commit_seq) {
		SPDK_WARNLOG("mwl_commit: seq=%"PRIu64" < commit_seq=%"PRIu64" (skip)\n",
			     seq, ctx->super.commit_seq);
		cb(0, cb_arg);
		return;
	}

	ctx->super.commit_seq = seq;
	mwl_super_set_crc(&ctx->super);

	op = calloc(1, sizeof(*op));
	if (!op) {
		cb(-ENOMEM, cb_arg);
		return;
	}
	op->ctx = ctx;
	op->ch = ch;
	op->cb = cb;
	op->cb_arg = cb_arg;
	op->commit_seq = seq;
	op->state = MWL_COMMIT_WRITE;
	op->status = 0;

	op->buf = spdk_dma_malloc(ctx->block_size, 0x1000, NULL);
	if (!op->buf) {
		free(op);
		cb(-ENOMEM, cb_arg);
		return;
	}
	memset(op->buf, 0, ctx->block_size);

	mwl_commit_loop(op);
}

/* ===== load_replay：先读 super，再分 chunk 扫 records ===== */

static void mwl_replay_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void mwl_replay_loop(struct mwl_replay_op *op);

static int
mwl_replay_insert(struct mwl_replay_op *op, const struct poweraid_raid_common_mwl_record *rec)
{
	struct poweraid_raid_common_mwl_record *arr;
	uint32_t i, j;

	if (op->result_cnt >= op->result_cap) {
		uint32_t new_cap = op->result_cap ? op->result_cap * 2 : 64;
		arr = realloc(op->result, new_cap * sizeof(*arr));
		if (!arr) {
			return -ENOMEM;
		}
		op->result = arr;
		op->result_cap = new_cap;
	}

	for (i = 0; i < op->result_cnt; i++) {
		if (op->result[i].seq > rec->seq) {
			break;
		}
	}
	for (j = op->result_cnt; j > i; j--) {
		op->result[j] = op->result[j - 1];
	}
	op->result[i] = *rec;
	op->result_cnt++;
	return 0;
}

static void
mwl_replay_scan_chunk(struct mwl_replay_op *op, uint32_t slots_in_buf)
{
	uint8_t *p = op->buf;
	uint32_t i;

	for (i = 0; i < slots_in_buf; i++) {
		const struct poweraid_raid_common_mwl_record *rec =
			(const struct poweraid_raid_common_mwl_record *)p;
		if (rec->magic == POWERAID_RAID_COMMON_MWL_REC_MAGIC) {
			if (mwl_rec_check(rec)) {
				if (rec->seq > op->ctx->super.commit_seq) {
					if (mwl_replay_insert(op, rec) != 0) {
						op->status = -ENOMEM;
						return;
					}
				}
			} else {
				/* magic 命中但 CRC 失败：坏 record，计数并跳过 */
				op->bad_record_cnt++;
				SPDK_WARNLOG("mwl_replay: bad record CRC at slot=%u seq=%"PRIu64
					     " (skip, bad_cnt=%u)\n",
					     op->cur_slot + i, rec->seq, op->bad_record_cnt);
			}
		}
		/* magic 不匹配视为空槽，跳过 */
		p += op->ctx->block_size;
	}
}

static void
mwl_replay_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct mwl_replay_op *op = cb_arg;
	uint32_t remaining, slots_in_buf;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		op->status = -EIO;
		op->cur_slot = op->end_slot;
		mwl_replay_loop(op);
		return;
	}

	remaining = op->end_slot - op->cur_slot;
	slots_in_buf = (remaining < op->buf_blocks) ? remaining : op->buf_blocks;
	mwl_replay_scan_chunk(op, slots_in_buf);
	op->cur_slot += slots_in_buf;

	mwl_replay_loop(op);
}

static void
mwl_replay_loop(struct mwl_replay_op *op)
{
	struct poweraid_raid_common_mwl_ctx *ctx = op->ctx;
	uint32_t remaining, slots_this;
	uint64_t byte_off;
	int rc;

	if (op->status != 0 || op->cur_slot >= op->end_slot) {
		if (op->status == 0 && op->result_cnt > 0) {
			SPDK_NOTICELOG("poweraid_raid_common_mwl: replay found %u uncommitted records"
				       " (bad=%u)\n", op->result_cnt, op->bad_record_cnt);
			op->cb(0, op->result, op->result_cnt, op->cb_arg);
		} else if (op->status == 0) {
			SPDK_NOTICELOG("poweraid_raid_common_mwl: replay clean (no uncommitted, bad=%u)\n",
				       op->bad_record_cnt);
			op->cb(0, NULL, 0, op->cb_arg);
		} else {
			SPDK_ERRLOG("poweraid_raid_common_mwl: replay failed (%d, bad=%u)\n",
				    op->status, op->bad_record_cnt);
			free(op->result);
			op->cb(op->status, NULL, 0, op->cb_arg);
		}
		spdk_free(op->buf);
		free(op);
		return;
	}

	remaining = op->end_slot - op->cur_slot;
	slots_this = (remaining < op->buf_blocks) ? remaining : op->buf_blocks;
	byte_off = slot_to_byte_offset(ctx, op->cur_slot);

	rc = spdk_bdev_read_blocks(ctx->bdev_desc, ctx->ch, op->buf,
				   byte_to_block_offset(ctx, byte_off),
				   slots_this, mwl_replay_read_cb, op);
	if (rc == -ENOMEM) {
		mwl_arm_wait(&op->wait_entry, ctx->bdev_desc, ctx->ch,
			     (spdk_bdev_io_wait_cb)mwl_replay_loop, op);
		return;
	} else if (rc != 0) {
		op->status = rc;
		op->cur_slot = op->end_slot;
		mwl_replay_loop(op);
		return;
	}
	/* 等 cb */
}

static void mwl_super_load_submit(void *arg);

static void
mwl_super_load_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct mwl_super_load_op *sop = cb_arg;
	struct poweraid_raid_common_mwl_ctx *ctx = sop->ctx;
	const struct poweraid_raid_common_mwl_super *s;
	struct mwl_replay_op *op;
	uint32_t chunk_blocks;
	uint32_t slots_this;
	uint64_t byte_off;
	int rc;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		SPDK_ERRLOG("mwl_replay: read super failed\n");
		sop->cb(-EIO, NULL, 0, sop->cb_arg);
		spdk_free(sop->buf);
		free(sop);
		return;
	}

	s = (const struct poweraid_raid_common_mwl_super *)sop->buf;
	if (s->magic != POWERAID_RAID_COMMON_MWL_SUPER_MAGIC || !mwl_super_check(s)) {
		SPDK_NOTICELOG("mwl_replay: no valid super (fresh disk?) → clean\n");
		sop->cb(0, NULL, 0, sop->cb_arg);
		spdk_free(sop->buf);
		free(sop);
		return;
	}

	memcpy(&ctx->super, s, sizeof(ctx->super));
	SPDK_NOTICELOG("mwl_replay: super loaded commit_seq=%"PRIu64" next_seq=%"PRIu64
		       " tail_seq=%"PRIu64" num_records=%"PRIu32" acting_lead=%"PRIu64"\n",
		       ctx->super.commit_seq, ctx->super.next_seq,
		       ctx->super.tail_seq, ctx->super.num_records, ctx->super.acting_lead_slot);

	op = calloc(1, sizeof(*op));
	if (!op) {
		sop->cb(-ENOMEM, NULL, 0, sop->cb_arg);
		spdk_free(sop->buf);
		free(sop);
		return;
	}
	op->ctx = ctx;
	op->cb = sop->cb;
	op->cb_arg = sop->cb_arg;
	op->status = 0;
	op->cur_slot = 1;
	/* 线性日志全扫至 max_slots，靠 magic+crc 过滤空槽。
	 * tail_slot 在内存中未持久化（仅 super 里隐含 next_seq）；阶段 2 不读 tail_slot，
	 * 全扫 + magic 过滤即可正确识别有效 record。*/
	op->end_slot = ctx->max_slots;

	chunk_blocks = (uint32_t)((1ULL << 20) / ctx->block_size);
	if (chunk_blocks == 0) {
		chunk_blocks = 1;
	}
	op->buf_blocks = chunk_blocks;
	op->buf = spdk_dma_malloc((uint64_t)chunk_blocks * ctx->block_size, 0x1000, NULL);
	if (!op->buf) {
		free(op);
		sop->cb(-ENOMEM, NULL, 0, sop->cb_arg);
		spdk_free(sop->buf);
		free(sop);
		return;
	}

	byte_off = slot_to_byte_offset(ctx, op->cur_slot);
	/* 第一次读限制不超过剩余 slot，避免小 region 越界（与 mwl_replay_loop 的 slots_this 一致）*/
	slots_this = (op->end_slot - op->cur_slot < chunk_blocks) ?
		     (op->end_slot - op->cur_slot) : chunk_blocks;
	rc = spdk_bdev_read_blocks(ctx->bdev_desc, ctx->ch, op->buf,
				   byte_to_block_offset(ctx, byte_off),
				   slots_this, mwl_replay_read_cb, op);
	if (rc == -ENOMEM) {
		mwl_arm_wait(&op->wait_entry, ctx->bdev_desc, ctx->ch,
			     (spdk_bdev_io_wait_cb)mwl_replay_loop, op);
		/* sop 的 cb 已转交给 op；释放 sop 的 buf 并 sop 本身（与 PPL 同模式）*/
		spdk_free(sop->buf);
		free(sop);
		return;
	} else if (rc != 0) {
		spdk_free(op->buf);
		free(op);
		sop->cb(rc, NULL, 0, sop->cb_arg);
		spdk_free(sop->buf);
		free(sop);
		return;
	}
	/* 提交成功，释放 super 读 op（replay op 已接管 cb/cb_arg/buf）*/
	spdk_free(sop->buf);
	free(sop);
	/* 等 record chunk 读 cb */
}

static void
mwl_super_load_submit(void *arg)
{
	struct mwl_super_load_op *sop = arg;
	struct poweraid_raid_common_mwl_ctx *ctx = sop->ctx;
	int rc;

	rc = spdk_bdev_read_blocks(ctx->bdev_desc, ctx->ch, sop->buf,
				   byte_to_block_offset(ctx, ctx->region_offset),
				   1, mwl_super_load_cb, sop);
	if (rc == -ENOMEM) {
		mwl_arm_wait(&sop->wait_entry, ctx->bdev_desc, ctx->ch,
			     (spdk_bdev_io_wait_cb)mwl_super_load_submit, sop);
	} else if (rc != 0) {
		sop->cb(rc, NULL, 0, sop->cb_arg);
		spdk_free(sop->buf);
		free(sop);
	}
}

void
poweraid_raid_common_mwl_load_replay(struct poweraid_raid_common_mwl_ctx *ctx,
				poweraid_raid_common_mwl_replay_cb cb, void *cb_arg)
{
	struct mwl_super_load_op *sop;

	if (ctx == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, NULL, 0, cb_arg); }
		return;
	}

	sop = calloc(1, sizeof(*sop));
	if (!sop) {
		cb(-ENOMEM, NULL, 0, cb_arg);
		return;
	}
	sop->ctx = ctx;
	sop->cb = cb;
	sop->cb_arg = cb_arg;

	sop->buf = spdk_dma_malloc(ctx->block_size, 0x1000, NULL);
	if (!sop->buf) {
		free(sop);
		cb(-ENOMEM, NULL, 0, cb_arg);
		return;
	}
	memset(sop->buf, 0, ctx->block_size);

	mwl_super_load_submit(sop);
}

void
poweraid_raid_common_mwl_free_replay_result(struct poweraid_raid_common_mwl_record *records)
{
	free(records);
}
