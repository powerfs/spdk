/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   PPL（Partial Parity Log）实现。
 *
 *   详见 poweraid_raid5f_ppl.h 与 raid5f-enhanced-design.md 第 3.1 / 3.7 节。
 *
 *   阶段 2 策略（正确性优先）：
 *     - 线性日志，无回卷/回收（满则 -ENOSPC；测试场景容量足够）。
 *     - 每 record 占一个 block_size slot（record 64B 置于 slot 起始，余清零），
 *       单 record 独立 FUA，无需读改写。
 *     - FUA = spdk_bdev_write_blocks + spdk_bdev_flush_blocks（bdev 无关）。
 *     - per-op DMA scratch buffer（spdk_dma_malloc），cb 中释放，避免并发冲突。
 *     - super 与 record 共享同一 PPL 区：slot 0 = super，slot 1..N = records。
 *
 *   TODO（后续阶段）：64B 紧凑打包 + 批量 FUA + 环形回卷回收，降低写放大。
 */

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/crc32.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/util.h"

#include "poweraid_raid5f_ppl.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_ppl);

/* ===== 内部上下文 ===== */

struct poweraid_raid5f_ppl_ctx {
	void				*bdev_desc;     /* struct spdk_bdev_desc* */
	struct spdk_io_channel		*ch;
	uint32_t			block_size;
	uint64_t			region_offset;  /* 字节，PPL 区在盘上的起始 */
	uint64_t			region_size;     /* 字节 */
	uint32_t			block_shift;     /* log2(block_size) */
	uint32_t			max_slots;       /* region 内 block 槽位数（含 super slot 0）*/

	/* 内存 super 镜像（append/commit 时更新并写盘；load_replay 时从盘加载）*/
	struct poweraid_raid5f_ppl_super	super;

	/* 线性日志写指针：下一条 record 写入的 slot 下标（1..max_slots-1；0=super）*/
	uint32_t			tail_slot;

	/* in-flight（已 append 待 commit）记录链表，保证 commit 顺序 */
	TAILQ_HEAD(, poweraid_raid5f_ppl_inflight)	inflight;
};

/* ===== append 异步操作 ===== */
enum ppl_append_state {
	PPL_APPEND_WRITE = 0,
	PPL_APPEND_FLUSH,
	PPL_APPEND_DONE,
};

struct ppl_append_op {
	struct poweraid_raid5f_ppl_ctx	*ctx;
	struct spdk_io_channel		*ch;   /* 调用方线程的 bdev channel（数据面）*/
	struct poweraid_raid5f_ppl_record rec;
	uint8_t				state;
	int				status;
	uint64_t			seq;
	uint32_t			slot;            /* 写入的 slot 下标 */
	uint64_t			slot_byte_off;    /* slot 在盘上的字节偏移 */
	void				*buf;             /* block_size DMA scratch */
	poweraid_raid5f_ppl_append_cb	cb;
	void				*cb_arg;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== commit 异步操作 ===== */
enum ppl_commit_state {
	PPL_COMMIT_WRITE = 0,
	PPL_COMMIT_FLUSH,
	PPL_COMMIT_DONE,
};

struct ppl_commit_op {
	struct poweraid_raid5f_ppl_ctx	*ctx;
	struct spdk_io_channel		*ch;   /* 调用方线程的 bdev channel（数据面）*/
	uint8_t				state;
	int				status;
	uint64_t			commit_seq;
	void				*buf;             /* block_size DMA scratch（写 super）*/
	poweraid_raid5f_ppl_commit_cb	cb;
	void				*cb_arg;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== init 异步操作 ===== */
enum ppl_init_state {
	PPL_INIT_WRITE = 0,
	PPL_INIT_FLUSH,
	PPL_INIT_DONE,
};

struct ppl_init_op {
	struct poweraid_raid5f_ppl_ctx	*ctx;
	uint8_t				state;
	int				status;
	void				*buf;             /* block_size DMA scratch（写 super）*/
	poweraid_raid5f_ppl_init_cb	cb;
	void				*cb_arg;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== load_replay 异步操作 ===== */
struct ppl_replay_op {
	struct poweraid_raid5f_ppl_ctx	*ctx;
	poweraid_raid5f_ppl_replay_cb	cb;
	void				*cb_arg;
	void				*buf;             /* 当前 chunk 读缓冲 */
	uint32_t			buf_blocks;       /* buf 容纳的 block 数 */
	uint32_t			cur_slot;         /* 下一个待读 slot */
	uint32_t			end_slot;         /* 扫描终止 slot */
	struct poweraid_raid5f_ppl_record *result;
	uint32_t			result_cap;
	uint32_t			result_cnt;
	int				status;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* super 读专用 op（load_replay 第一阶段）*/
struct ppl_super_load_op {
	struct poweraid_raid5f_ppl_ctx	*ctx;
	poweraid_raid5f_ppl_replay_cb	cb;
	void				*cb_arg;
	void				*buf;             /* block_size，读 super */
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== 内部辅助 ===== */

static inline uint64_t
slot_to_byte_offset(struct poweraid_raid5f_ppl_ctx *ctx, uint32_t slot)
{
	return ctx->region_offset + (uint64_t)slot * ctx->block_size;
}

static inline uint64_t
byte_to_block_offset(struct poweraid_raid5f_ppl_ctx *ctx, uint64_t byte_off)
{
	return byte_off >> ctx->block_shift;
}

/* crc 覆盖 record 除 crc32c 字段外全部字节（两段：crc32c 前一段 + crc32c 后一段）*/
static uint32_t
ppl_rec_calc_crc(const struct poweraid_raid5f_ppl_record *rec)
{
	uint32_t crc;
	crc = spdk_crc32c_update(rec,
				 offsetof(struct poweraid_raid5f_ppl_record, crc32c), 0);
	crc = spdk_crc32c_update((const uint8_t *)rec +
				 offsetof(struct poweraid_raid5f_ppl_record, crc32c) +
				 sizeof(uint32_t),
				 sizeof(*rec) -
				 offsetof(struct poweraid_raid5f_ppl_record, crc32c) -
				 sizeof(uint32_t),
				 crc);
	return crc;
}

static inline void
ppl_rec_set_crc(struct poweraid_raid5f_ppl_record *rec)
{
	rec->crc32c = 0;
	rec->crc32c = ppl_rec_calc_crc(rec);
}

static inline bool
ppl_rec_check(const struct poweraid_raid5f_ppl_record *rec)
{
	if (rec->magic != POWERAID_RAID5F_PPL_REC_MAGIC) {
		return false;
	}
	return ppl_rec_calc_crc(rec) == rec->crc32c;
}

static uint32_t
ppl_super_calc_crc(const struct poweraid_raid5f_ppl_super *s)
{
	uint32_t crc;
	crc = spdk_crc32c_update(s,
				 offsetof(struct poweraid_raid5f_ppl_super, crc32c), 0);
	crc = spdk_crc32c_update((const uint8_t *)s +
				offsetof(struct poweraid_raid5f_ppl_super, crc32c) +
				sizeof(uint32_t),
				sizeof(*s) -
				offsetof(struct poweraid_raid5f_ppl_super, crc32c) -
				sizeof(uint32_t),
				crc);
	return crc;
}

static inline void
ppl_super_set_crc(struct poweraid_raid5f_ppl_super *s)
{
	s->crc32c = 0;
	s->crc32c = ppl_super_calc_crc(s);
}

static inline bool
ppl_super_check(const struct poweraid_raid5f_ppl_super *s)
{
	if (s->magic != POWERAID_RAID5F_PPL_SUPER_MAGIC) {
		return false;
	}
	return ppl_super_calc_crc(s) == s->crc32c;
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

struct poweraid_raid5f_ppl_ctx *
poweraid_raid5f_ppl_alloc(void *bdev_desc, struct spdk_io_channel *ch,
			  uint32_t block_size,
			  uint64_t region_offset, uint64_t region_size)
{
	struct poweraid_raid5f_ppl_ctx *ctx;
	int shift;

	if (bdev_desc == NULL || ch == NULL || block_size == 0 ||
	    region_size < (uint64_t)block_size * 2 || region_offset == 0) {
		SPDK_ERRLOG("ppl_alloc: invalid args (bs=%u rs=%"PRIu64" off=%"PRIu64")\n",
			    block_size, region_size, region_offset);
		return NULL;
	}
	shift = block_shift_of(block_size);
	if (shift < 0) {
		SPDK_ERRLOG("ppl_alloc: unsupported block_size %u\n", block_size);
		return NULL;
	}

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		SPDK_ERRLOG("ppl_alloc: calloc ctx failed\n");
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

	/* 内存 super 初始（写盘由 ppl_init 触发；加载由 ppl_load_replay 触发）*/
	memset(&ctx->super, 0, sizeof(ctx->super));
	ctx->super.magic = POWERAID_RAID5F_PPL_SUPER_MAGIC;
	ctx->super.version = 1;
	ctx->super.head_seq = 1;
	ctx->super.tail_seq = 1;
	ctx->super.commit_seq = 0;
	ctx->super.next_seq = 1;
	ctx->super.num_records = 0;

	return ctx;
}

void
poweraid_raid5f_ppl_free(struct poweraid_raid5f_ppl_ctx *ctx)
{
	struct poweraid_raid5f_ppl_inflight *inf;

	if (ctx == NULL) {
		return;
	}
	while ((inf = TAILQ_FIRST(&ctx->inflight)) != NULL) {
		TAILQ_REMOVE(&ctx->inflight, inf, link);
		free(inf);
	}
	free(ctx);
}

/* ===== 通用 wait helper：资源可用时重新进入对应 loop 重试提交 ===== */
static void
ppl_arm_wait(struct spdk_bdev_io_wait_entry *w, void *bdev_desc,
	     struct spdk_io_channel *ch, spdk_bdev_io_wait_cb retry_fn, void *op)
{
	w->bdev = spdk_bdev_desc_get_bdev(bdev_desc);
	w->cb_fn = retry_fn;
	w->cb_arg = op;
	spdk_bdev_queue_io_wait(bdev_desc, ch, w);
}

/* ===== init：格式化 PPL 区（写 super + FUA）===== */

static void ppl_init_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void ppl_init_loop(struct ppl_init_op *op);

static void
ppl_init_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ppl_init_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		op->status = -EIO;
		op->state = PPL_INIT_DONE;
		ppl_init_loop(op);
		return;
	}
	op->state++;
	ppl_init_loop(op);
}

static void
ppl_init_loop(struct ppl_init_op *op)
{
	struct poweraid_raid5f_ppl_ctx *ctx = op->ctx;
	int rc;

	while (op->state < PPL_INIT_DONE) {
		switch (op->state) {
		case PPL_INIT_WRITE:
			memcpy(op->buf, &ctx->super, sizeof(ctx->super));
			rc = spdk_bdev_write_blocks(ctx->bdev_desc, ctx->ch, op->buf,
						    byte_to_block_offset(ctx, ctx->region_offset),
						    1, ppl_init_io_cb, op);
			if (rc == -ENOMEM) {
				ppl_arm_wait(&op->wait_entry, ctx->bdev_desc, ctx->ch,
					     (spdk_bdev_io_wait_cb)ppl_init_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = PPL_INIT_DONE;
				continue;
			}
			return;  /* 等 cb */

		case PPL_INIT_FLUSH:
			rc = spdk_bdev_flush_blocks(ctx->bdev_desc, ctx->ch,
						    byte_to_block_offset(ctx, ctx->region_offset),
						    1, ppl_init_io_cb, op);
			if (rc == -ENOMEM) {
				ppl_arm_wait(&op->wait_entry, ctx->bdev_desc, ctx->ch,
					     (spdk_bdev_io_wait_cb)ppl_init_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = PPL_INIT_DONE;
				continue;
			}
			return;  /* 等 cb */

		default:
			op->status = -EINVAL;
			op->state = PPL_INIT_DONE;
			break;
		}
	}

	/* DONE */
	if (op->status == 0) {
		SPDK_NOTICELOG("poweraid_raid5f_ppl: init ok (super @ slot 0)\n");
	}
	op->cb(op->status, op->cb_arg);
	spdk_free(op->buf);
	free(op);
}

void
poweraid_raid5f_ppl_init(struct poweraid_raid5f_ppl_ctx *ctx,
			 poweraid_raid5f_ppl_init_cb cb, void *cb_arg)
{
	struct ppl_init_op *op;

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
	op->state = PPL_INIT_WRITE;

	op->buf = spdk_dma_malloc(ctx->block_size, 0x1000, NULL);
	if (!op->buf) {
		free(op);
		cb(-ENOMEM, cb_arg);
		return;
	}
	memset(op->buf, 0, ctx->block_size);

	ppl_super_set_crc(&ctx->super);
	ppl_init_loop(op);
}

/* ===== append：写 record + FUA ===== */

static void ppl_append_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void ppl_append_loop(struct ppl_append_op *op);

static void
ppl_append_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ppl_append_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		op->status = -EIO;
		op->state = PPL_APPEND_DONE;
		ppl_append_loop(op);
		return;
	}
	op->state++;
	ppl_append_loop(op);
}

static void
ppl_append_loop(struct ppl_append_op *op)
{
	struct poweraid_raid5f_ppl_ctx *ctx = op->ctx;
	int rc;

	while (op->state < PPL_APPEND_DONE) {
		switch (op->state) {
		case PPL_APPEND_WRITE:
			rc = spdk_bdev_write_blocks(ctx->bdev_desc, op->ch, op->buf,
						    byte_to_block_offset(ctx, op->slot_byte_off),
						    1, ppl_append_io_cb, op);
			if (rc == -ENOMEM) {
				ppl_arm_wait(&op->wait_entry, ctx->bdev_desc, op->ch,
					     (spdk_bdev_io_wait_cb)ppl_append_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = PPL_APPEND_DONE;
				continue;
			}
			return;  /* 等 cb */

		case PPL_APPEND_FLUSH:
			rc = spdk_bdev_flush_blocks(ctx->bdev_desc, op->ch,
						    byte_to_block_offset(ctx, op->slot_byte_off),
						    1, ppl_append_io_cb, op);
			if (rc == -ENOMEM) {
				ppl_arm_wait(&op->wait_entry, ctx->bdev_desc, op->ch,
					     (spdk_bdev_io_wait_cb)ppl_append_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = PPL_APPEND_DONE;
				continue;
			}
			return;  /* 等 cb */

		default:
			op->status = -EINVAL;
			op->state = PPL_APPEND_DONE;
			break;
		}
	}

	/* DONE */
	if (op->status == 0) {
		struct poweraid_raid5f_ppl_inflight *inf;
		ctx->super.tail_seq = op->seq + 1;
		ctx->super.next_seq = op->seq + 1;
		ctx->super.num_records++;
		ctx->tail_slot++;
		inf = calloc(1, sizeof(*inf));
		if (inf) {
			inf->seq = op->seq;
			inf->stripe_id = op->rec.stripe_id;
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
poweraid_raid5f_ppl_append_record(struct poweraid_raid5f_ppl_ctx *ctx,
				  struct spdk_io_channel *ch,
				  uint64_t stripe_id, uint64_t chunk_bitmap,
				  uint64_t old_data_hash, uint64_t new_data_hash,
				  poweraid_raid5f_ppl_append_cb cb, void *cb_arg)
{
	struct ppl_append_op *op;
	uint64_t seq;
	uint32_t slot;

	if (ctx == NULL || ch == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, 0, cb_arg); }
		return;
	}
	if (ctx->tail_slot >= ctx->max_slots) {
		/* 环形回收：所有已 append 的 record 均已 commit（inflight 空）后，
		 * slot 回卷到 1。seq 仍由 super.next_seq 单调分配，与 slot 解耦；
		 * 盘上未及覆盖的陈旧 record 在 replay 时按 seq <= commit_seq 过滤。*/
		if (!TAILQ_EMPTY(&ctx->inflight)) {
			SPDK_ERRLOG("ppl_append: log full with inflight (ctx=%p tail=%u max=%u)\n",
				    ctx, ctx->tail_slot, ctx->max_slots);
			cb(-ENOSPC, 0, cb_arg);
			return;
		}
		SPDK_NOTICELOG("poweraid_raid5f_ppl: ring recycle (next_seq=%"PRIu64")\n",
			       ctx->super.next_seq);
		ctx->tail_slot = 1;
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
	op->state = PPL_APPEND_WRITE;
	op->status = 0;

	memset(&op->rec, 0, sizeof(op->rec));
	op->rec.magic = POWERAID_RAID5F_PPL_REC_MAGIC;
	op->rec.flags = POWERAID_RAID5F_PPL_REC_F_VALID;
	op->rec.seq = seq;
	op->rec.stripe_id = stripe_id;
	op->rec.chunk_bitmap = chunk_bitmap;
	op->rec.old_data_hash = old_data_hash;
	op->rec.new_data_hash = new_data_hash;
	op->rec.ts = (uint64_t)time(NULL);
	ppl_rec_set_crc(&op->rec);

	op->buf = spdk_dma_malloc(ctx->block_size, 0x1000, NULL);
	if (!op->buf) {
		free(op);
		cb(-ENOMEM, 0, cb_arg);
		return;
	}
	memset(op->buf, 0, ctx->block_size);
	memcpy(op->buf, &op->rec, sizeof(op->rec));

	ppl_append_loop(op);
}

/* ===== commit：更新 super.commit_seq + FUA ===== */

static void ppl_commit_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void ppl_commit_loop(struct ppl_commit_op *op);

static void
ppl_commit_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ppl_commit_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		op->status = -EIO;
		op->state = PPL_COMMIT_DONE;
	}
	op->state++;
	ppl_commit_loop(op);
}

static void
ppl_commit_loop(struct ppl_commit_op *op)
{
	struct poweraid_raid5f_ppl_ctx *ctx = op->ctx;
	int rc;

	while (op->state < PPL_COMMIT_DONE) {
		switch (op->state) {
		case PPL_COMMIT_WRITE:
			memcpy(op->buf, &ctx->super, sizeof(ctx->super));
			rc = spdk_bdev_write_blocks(ctx->bdev_desc, op->ch, op->buf,
						    byte_to_block_offset(ctx, ctx->region_offset),
						    1, ppl_commit_io_cb, op);
			if (rc == -ENOMEM) {
				ppl_arm_wait(&op->wait_entry, ctx->bdev_desc, op->ch,
					     (spdk_bdev_io_wait_cb)ppl_commit_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = PPL_COMMIT_DONE;
				continue;
			}
			return;

		case PPL_COMMIT_FLUSH:
			rc = spdk_bdev_flush_blocks(ctx->bdev_desc, op->ch,
						    byte_to_block_offset(ctx, ctx->region_offset),
						    1, ppl_commit_io_cb, op);
			if (rc == -ENOMEM) {
				ppl_arm_wait(&op->wait_entry, ctx->bdev_desc, op->ch,
					     (spdk_bdev_io_wait_cb)ppl_commit_loop, op);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = PPL_COMMIT_DONE;
				continue;
			}
			return;

		default:
			op->status = -EINVAL;
			op->state = PPL_COMMIT_DONE;
			break;
		}
	}

	/* DONE */
	if (op->status == 0) {
		struct poweraid_raid5f_ppl_inflight *inf, *tmp;
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
poweraid_raid5f_ppl_commit(struct poweraid_raid5f_ppl_ctx *ctx,
			  struct spdk_io_channel *ch, uint64_t seq,
			  poweraid_raid5f_ppl_commit_cb cb, void *cb_arg)
{
	struct ppl_commit_op *op;

	if (ctx == NULL || ch == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, cb_arg); }
		return;
	}
	if (seq < ctx->super.commit_seq) {
		SPDK_WARNLOG("ppl_commit: seq=%"PRIu64" < commit_seq=%"PRIu64" (skip)\n",
			     seq, ctx->super.commit_seq);
		cb(0, cb_arg);
		return;
	}

	ctx->super.commit_seq = seq;
	ppl_super_set_crc(&ctx->super);

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
	op->state = PPL_COMMIT_WRITE;
	op->status = 0;

	op->buf = spdk_dma_malloc(ctx->block_size, 0x1000, NULL);
	if (!op->buf) {
		free(op);
		cb(-ENOMEM, cb_arg);
		return;
	}
	memset(op->buf, 0, ctx->block_size);

	ppl_commit_loop(op);
}

/* ===== load_replay：先读 super，再分 chunk 扫 records ===== */

static void ppl_replay_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void ppl_replay_loop(struct ppl_replay_op *op);

static int
ppl_replay_insert(struct ppl_replay_op *op, const struct poweraid_raid5f_ppl_record *rec)
{
	struct poweraid_raid5f_ppl_record *arr;
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
ppl_replay_scan_chunk(struct ppl_replay_op *op, uint32_t slots_in_buf)
{
	uint8_t *p = op->buf;
	uint32_t i;

	for (i = 0; i < slots_in_buf; i++) {
		const struct poweraid_raid5f_ppl_record *rec =
			(const struct poweraid_raid5f_ppl_record *)p;
		if (rec->magic == POWERAID_RAID5F_PPL_REC_MAGIC && ppl_rec_check(rec)) {
			if (rec->seq > op->ctx->super.commit_seq) {
				if (ppl_replay_insert(op, rec) != 0) {
					op->status = -ENOMEM;
					return;
				}
			}
		}
		p += op->ctx->block_size;
	}
}

static void
ppl_replay_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ppl_replay_op *op = cb_arg;
	uint32_t remaining, slots_in_buf;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		op->status = -EIO;
		op->cur_slot = op->end_slot;
		ppl_replay_loop(op);
		return;
	}

	remaining = op->end_slot - op->cur_slot;
	slots_in_buf = (remaining < op->buf_blocks) ? remaining : op->buf_blocks;
	ppl_replay_scan_chunk(op, slots_in_buf);
	op->cur_slot += slots_in_buf;

	ppl_replay_loop(op);
}

static void
ppl_replay_loop(struct ppl_replay_op *op)
{
	struct poweraid_raid5f_ppl_ctx *ctx = op->ctx;
	uint32_t remaining, slots_this;
	uint64_t byte_off;
	int rc;

	if (op->status != 0 || op->cur_slot >= op->end_slot) {
		if (op->status == 0 && op->result_cnt > 0) {
			SPDK_NOTICELOG("poweraid_raid5f_ppl: replay found %u uncommitted records\n",
				       op->result_cnt);
			op->cb(0, op->result, op->result_cnt, op->cb_arg);
		} else if (op->status == 0) {
			SPDK_NOTICELOG("poweraid_raid5f_ppl: replay clean (no uncommitted)\n");
			op->cb(0, NULL, 0, op->cb_arg);
		} else {
			SPDK_ERRLOG("poweraid_raid5f_ppl: replay failed (%d)\n", op->status);
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
				   slots_this, ppl_replay_read_cb, op);
	if (rc == -ENOMEM) {
		ppl_arm_wait(&op->wait_entry, ctx->bdev_desc, ctx->ch,
			     (spdk_bdev_io_wait_cb)ppl_replay_loop, op);
		return;
	} else if (rc != 0) {
		op->status = rc;
		op->cur_slot = op->end_slot;
		ppl_replay_loop(op);
		return;
	}
	/* 等 cb */
}

static void
ppl_super_load_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ppl_super_load_op *sop = cb_arg;
	struct poweraid_raid5f_ppl_ctx *ctx = sop->ctx;
	const struct poweraid_raid5f_ppl_super *s;
	struct ppl_replay_op *op;
	uint32_t chunk_blocks;
	uint64_t byte_off;
	int rc;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		SPDK_ERRLOG("ppl_replay: read super failed\n");
		sop->cb(-EIO, NULL, 0, sop->cb_arg);
		spdk_free(sop->buf);
		free(sop);
		return;
	}

	s = (const struct poweraid_raid5f_ppl_super *)sop->buf;
	if (s->magic != POWERAID_RAID5F_PPL_SUPER_MAGIC || !ppl_super_check(s)) {
		SPDK_NOTICELOG("ppl_replay: no valid super (fresh disk?) → clean\n");
		sop->cb(0, NULL, 0, sop->cb_arg);
		spdk_free(sop->buf);
		free(sop);
		return;
	}

	memcpy(&ctx->super, s, sizeof(ctx->super));
	SPDK_NOTICELOG("ppl_replay: super loaded commit_seq=%"PRIu64" next_seq=%"PRIu64
		       " tail_seq=%"PRIu64" num_records=%"PRIu64"\n",
		       ctx->super.commit_seq, ctx->super.next_seq,
		       ctx->super.tail_seq, ctx->super.num_records);

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
	rc = spdk_bdev_read_blocks(ctx->bdev_desc, ctx->ch, op->buf,
				   byte_to_block_offset(ctx, byte_off),
				   chunk_blocks, ppl_replay_read_cb, op);
	if (rc == -ENOMEM) {
		ppl_arm_wait(&op->wait_entry, ctx->bdev_desc, ctx->ch,
			     (spdk_bdev_io_wait_cb)ppl_replay_loop, op);
		/* sop 仍持有 cb 兜底，但 IO 已挂载到 op 的 wait；释放 sop 的 buf 并保留 sop
		 * 直到 replay 完成（cb 由 ppl_replay_loop 调用）。为避免泄漏，把 sop 的
		 * buf 释放并将 cb 转交给 op（op 已拷贝 sop->cb/cb_arg），可安全释放 sop。*/
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

static void ppl_super_load_submit(void *arg);

static void
ppl_super_load_submit(void *arg)
{
	struct ppl_super_load_op *sop = arg;
	struct poweraid_raid5f_ppl_ctx *ctx = sop->ctx;
	int rc;

	rc = spdk_bdev_read_blocks(ctx->bdev_desc, ctx->ch, sop->buf,
				   byte_to_block_offset(ctx, ctx->region_offset),
				   1, ppl_super_load_cb, sop);
	if (rc == -ENOMEM) {
		ppl_arm_wait(&sop->wait_entry, ctx->bdev_desc, ctx->ch,
			     (spdk_bdev_io_wait_cb)ppl_super_load_submit, sop);
	} else if (rc != 0) {
		sop->cb(rc, NULL, 0, sop->cb_arg);
		spdk_free(sop->buf);
		free(sop);
	}
}

void
poweraid_raid5f_ppl_load_replay(struct poweraid_raid5f_ppl_ctx *ctx,
				poweraid_raid5f_ppl_replay_cb cb, void *cb_arg)
{
	struct ppl_super_load_op *sop;

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

	ppl_super_load_submit(sop);
}

void
poweraid_raid5f_ppl_free_replay_result(struct poweraid_raid5f_ppl_record *records)
{
	free(records);
}

uint64_t
poweraid_raid5f_ppl_data_hash(const void *buf, size_t len)
{
	uint32_t lo, hi;
	/* 两段 crc32c，不同 seed，拼成 64-bit hash */
	lo = spdk_crc32c_update(buf, len, 0);
	hi = spdk_crc32c_update(buf, len, ~0u);
	return ((uint64_t)hi << 32) | (uint64_t)lo;
}

void
poweraid_raid5f_ppl_hash_init(struct poweraid_raid5f_ppl_hash_ctx *c)
{
	c->lo = 0;
	c->hi = ~0u;
}

void
poweraid_raid5f_ppl_hash_update(struct poweraid_raid5f_ppl_hash_ctx *c,
				const void *buf, size_t len)
{
	c->lo = spdk_crc32c_update(buf, len, c->lo);
	c->hi = spdk_crc32c_update(buf, len, c->hi);
}

uint64_t
poweraid_raid5f_ppl_hash_final(struct poweraid_raid5f_ppl_hash_ctx *c)
{
	return ((uint64_t)c->hi << 32) | (uint64_t)c->lo;
}
