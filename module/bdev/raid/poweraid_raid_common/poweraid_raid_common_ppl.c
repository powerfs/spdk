/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   PPL（Partial Parity Log）实现。
 *
 *   详见 poweraid_raid_common_ppl.h 与 raid5f-enhanced-design.md 第 3.1 / 3.7 节。
 *
 *   阶段 2 策略（正确性优先）：
 *     - 线性日志，满则 backpressure（issue #14）：append 请求入队等待 commit 释放 slot。
 *     - 每 record 占一个 block_size slot（record 64B 置于 slot 起始，余清零），
 *       单 record 独立 FUA，无需读改写。
 *     - FUA = spdk_bdev_write_blocks + spdk_bdev_flush_blocks（bdev 无关）。
 *     - per-op DMA scratch buffer（spdk_dma_malloc），cb 中释放，避免并发冲突。
 *     - super 与 record 共享同一 PPL 区：slot 0 = super，slot 1..N = records。
 *     - slot / seq 在 append_record 入口同步预占（并发安全），异步写完成时仅插 inflight。
 *
 *   TODO（后续阶段）：64B 紧凑打包 + 批量 FUA + 环形回卷回收，降低写放大。
 */

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/crc32.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/util.h"

#include "../bdev_raid.h"
#include "poweraid_raid_common_ppl.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_ppl);

/* ===== 内部上下文 ===== */

struct poweraid_raid_common_ppl_ctx {
	void				*bdev_desc;     /* struct spdk_bdev_desc* */
	struct spdk_io_channel		*ch;
	uint32_t			block_size;
	uint64_t			region_offset;  /* 字节，PPL 区在盘上的起始 */
	uint64_t			region_size;     /* 字节 */
	uint32_t			block_shift;     /* log2(block_size) */
	uint32_t			max_slots;       /* region 内 block 槽位数（含 super slot 0）*/

	/* 内存 super 镜像（append/commit 时更新并写盘；load_replay 时从盘加载）*/
	struct poweraid_raid_common_ppl_super	super;

	/* 线性日志写指针：下一条 record 写入的 slot 下标（1..max_slots-1；0=super）。
	 * 同步预占：在 append_record 入口即递增，避免并发 append 拿到相同 slot。*/
	uint32_t			tail_slot;

	/* in-flight（已 append 待 commit）记录链表，保证 commit 顺序 */
	TAILQ_HEAD(, poweraid_raid_common_ppl_inflight)	inflight;

	/* backpressure 等待队列：PPL 满时挂起 append 请求，commit 释放 slot 后唤醒。
	 * 解决 issue #14：高并发写时 PPL 满（-ENOSPC）降级无 PPL 保护的问题。*/
	TAILQ_HEAD(, poweraid_raid_common_ppl_waiter)	waiters;

	/* slot_lock：保护 tail_slot/next_seq/super 修改、inflight/waiters 链表操作。
	 * 修复多线程并发 append/commit 的 race（阶段 C1 顺带修复）。*/
	pthread_spinlock_t		slot_lock;
};

/* backpressure 等待者：PPL 满时暂存 append 参数，commit 唤醒后重发 append */
struct poweraid_raid_common_ppl_waiter {
	struct spdk_io_channel		*ch;
	uint64_t			stripe_id;
	uint64_t			chunk_bitmap;
	uint64_t			old_data_hash;
	uint64_t			new_data_hash;
	poweraid_raid_common_ppl_append_cb	cb;
	void				*cb_arg;
	TAILQ_ENTRY(poweraid_raid_common_ppl_waiter)	link;
};

/* ===== append 异步操作 ===== */
enum ppl_append_state {
	PPL_APPEND_WRITE = 0,
	PPL_APPEND_FLUSH,
	PPL_APPEND_DONE,
};

struct ppl_append_op {
	struct poweraid_raid_common_ppl_ctx	*ctx;
	struct spdk_io_channel		*ch;   /* 调用方线程的 bdev channel（数据面）*/
	struct poweraid_raid_common_ppl_record rec;
	uint8_t				state;
	int				status;
	uint64_t			seq;
	uint32_t			slot;            /* 写入的 slot 下标 */
	uint64_t			slot_byte_off;    /* slot 在盘上的字节偏移 */
	void				*buf;             /* block_size DMA scratch */
	poweraid_raid_common_ppl_append_cb	cb;
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
	struct poweraid_raid_common_ppl_ctx	*ctx;
	struct spdk_io_channel		*ch;   /* 调用方线程的 bdev channel（数据面）*/
	uint8_t				state;
	int				status;
	uint64_t			commit_seq;
	void				*buf;             /* block_size DMA scratch（写 super）*/
	poweraid_raid_common_ppl_commit_cb	cb;
	void				*cb_arg;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== init 异步操作 ===== */
enum ppl_init_state {
	PPL_INIT_ZEROES = 0,  /* 清零整个 PPL region，防止旧 record 残留（阶段 C1 根因修复）*/
	PPL_INIT_WRITE,
	PPL_INIT_FLUSH,
	PPL_INIT_DONE,
};

struct ppl_init_op {
	struct poweraid_raid_common_ppl_ctx	*ctx;
	uint8_t				state;
	int				status;
	void				*buf;             /* block_size DMA scratch（写 super）*/
	poweraid_raid_common_ppl_init_cb	cb;
	void				*cb_arg;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== load_replay 异步操作 ===== */
struct ppl_replay_op {
	struct poweraid_raid_common_ppl_ctx	*ctx;
	poweraid_raid_common_ppl_replay_cb	cb;
	void				*cb_arg;
	void				*buf;             /* 当前 chunk 读缓冲 */
	uint32_t			buf_blocks;       /* buf 容纳的 block 数 */
	uint32_t			cur_slot;         /* 下一个待读 slot */
	uint32_t			end_slot;         /* 扫描终止 slot */
	struct poweraid_raid_common_ppl_record *result;
	uint32_t			result_cap;
	uint32_t			result_cnt;
	int				status;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* super 读专用 op（load_replay 第一阶段）*/
struct ppl_super_load_op {
	struct poweraid_raid_common_ppl_ctx	*ctx;
	poweraid_raid_common_ppl_replay_cb	cb;
	void				*cb_arg;
	void				*buf;             /* block_size，读 super */
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== 内部辅助 ===== */

static inline uint64_t
slot_to_byte_offset(struct poweraid_raid_common_ppl_ctx *ctx, uint32_t slot)
{
	return ctx->region_offset + (uint64_t)slot * ctx->block_size;
}

static inline uint64_t
byte_to_block_offset(struct poweraid_raid_common_ppl_ctx *ctx, uint64_t byte_off)
{
	return byte_off >> ctx->block_shift;
}

/* crc 覆盖 record 除 crc32c 字段外全部字节（两段：crc32c 前一段 + crc32c 后一段）*/
static uint32_t
ppl_rec_calc_crc(const struct poweraid_raid_common_ppl_record *rec)
{
	uint32_t crc;
	crc = spdk_crc32c_update(rec,
				 offsetof(struct poweraid_raid_common_ppl_record, crc32c), 0);
	crc = spdk_crc32c_update((const uint8_t *)rec +
				 offsetof(struct poweraid_raid_common_ppl_record, crc32c) +
				 sizeof(uint32_t),
				 sizeof(*rec) -
				 offsetof(struct poweraid_raid_common_ppl_record, crc32c) -
				 sizeof(uint32_t),
				 crc);
	return crc;
}

static inline void
ppl_rec_set_crc(struct poweraid_raid_common_ppl_record *rec)
{
	rec->crc32c = 0;
	rec->crc32c = ppl_rec_calc_crc(rec);
}

static inline bool
ppl_rec_check(const struct poweraid_raid_common_ppl_record *rec)
{
	if (rec->magic != POWERAID_RAID_COMMON_PPL_REC_MAGIC) {
		return false;
	}
	return ppl_rec_calc_crc(rec) == rec->crc32c;
}

static uint32_t
ppl_super_calc_crc(const struct poweraid_raid_common_ppl_super *s)
{
	uint32_t crc;
	crc = spdk_crc32c_update(s,
				 offsetof(struct poweraid_raid_common_ppl_super, crc32c), 0);
	crc = spdk_crc32c_update((const uint8_t *)s +
				offsetof(struct poweraid_raid_common_ppl_super, crc32c) +
				sizeof(uint32_t),
				sizeof(*s) -
				offsetof(struct poweraid_raid_common_ppl_super, crc32c) -
				sizeof(uint32_t),
				crc);
	return crc;
}

static inline void
ppl_super_set_crc(struct poweraid_raid_common_ppl_super *s)
{
	s->crc32c = 0;
	s->crc32c = ppl_super_calc_crc(s);
}

static inline bool
ppl_super_check(const struct poweraid_raid_common_ppl_super *s)
{
	if (s->magic != POWERAID_RAID_COMMON_PPL_SUPER_MAGIC) {
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

struct poweraid_raid_common_ppl_ctx *
poweraid_raid_common_ppl_alloc(void *bdev_desc, struct spdk_io_channel *ch,
			  uint32_t block_size,
			  uint64_t region_offset, uint64_t region_size)
{
	struct poweraid_raid_common_ppl_ctx *ctx;
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
	TAILQ_INIT(&ctx->waiters);
	pthread_spin_init(&ctx->slot_lock, PTHREAD_PROCESS_PRIVATE);

	/* 内存 super 初始（写盘由 ppl_init 触发；加载由 ppl_load_replay 触发）*/
	memset(&ctx->super, 0, sizeof(ctx->super));
	ctx->super.magic = POWERAID_RAID_COMMON_PPL_SUPER_MAGIC;
	ctx->super.version = 1;
	ctx->super.head_seq = 1;
	ctx->super.tail_seq = 1;
	ctx->super.commit_seq = 0;
	ctx->super.next_seq = 1;
	ctx->super.num_records = 0;

	return ctx;
}

void
poweraid_raid_common_ppl_free(struct poweraid_raid_common_ppl_ctx *ctx)
{
	struct poweraid_raid_common_ppl_inflight *inf;
	struct poweraid_raid_common_ppl_waiter *w;

	if (ctx == NULL) {
		return;
	}
	while ((inf = TAILQ_FIRST(&ctx->inflight)) != NULL) {
		TAILQ_REMOVE(&ctx->inflight, inf, link);
		free(inf);
	}
	/* 唤醒所有等待者并报错（ctx 即将销毁，不能再 append）*/
	while ((w = TAILQ_FIRST(&ctx->waiters)) != NULL) {
		TAILQ_REMOVE(&ctx->waiters, w, link);
		w->cb(-ECANCELED, 0, w->cb_arg);
		free(w);
	}
	pthread_spin_destroy(&ctx->slot_lock);
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

/* ===== init：格式化 PPL 区（清零 region + 写 super + FUA）===== */

static void ppl_init_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void ppl_init_loop(struct ppl_init_op *op);

/* 清零 record 区域完成回调。
 * 根因修复（阶段 C1）：ppl_init 之前只写 super（slot 0），不清零 record 区域，
 * 旧磁盘残留的 record 会被 recovery 误读为 uncommitted，执行 REWRITE_PARITY 破坏全新卷 parity。
 * 清零整个 PPL region 后再写 super，确保旧数据被彻底清除。*/
static void
ppl_init_zeroes_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ppl_init_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		SPDK_ERRLOG("ppl_init: write_zeroes PPL region failed\n");
		op->status = -EIO;
		op->state = PPL_INIT_DONE;
		ppl_init_loop(op);
		return;
	}
	op->state++;
	ppl_init_loop(op);
}

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
	struct poweraid_raid_common_ppl_ctx *ctx = op->ctx;
	int rc;

	while (op->state < PPL_INIT_DONE) {
		switch (op->state) {
		case PPL_INIT_ZEROES:
			/* 清零整个 PPL region（slot 0..max_slots-1），防止旧磁盘 record 残留。
			 * 旧 record 的 magic+crc 仍然有效，recovery 会误读为 uncommitted record，
			 * 对全新卷执行 REWRITE_PARITY 破坏 parity。必须先清零再写 super。*/
			rc = spdk_bdev_write_zeroes_blocks(ctx->bdev_desc, ctx->ch,
							   byte_to_block_offset(ctx, ctx->region_offset),
							   ctx->max_slots, ppl_init_zeroes_cb, op);
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
		SPDK_NOTICELOG("poweraid_raid_common_ppl: init ok (super @ slot 0)\n");
	}
	op->cb(op->status, op->cb_arg);
	spdk_free(op->buf);
	free(op);
}

void
poweraid_raid_common_ppl_init(struct poweraid_raid_common_ppl_ctx *ctx,
			 poweraid_raid_common_ppl_init_cb cb, void *cb_arg)
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
	op->state = PPL_INIT_ZEROES;

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
	struct poweraid_raid_common_ppl_ctx *ctx = op->ctx;
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
		struct poweraid_raid_common_ppl_inflight *inf;
		/* tail_slot / next_seq 已在 append_record 入口同步预占，此处仅插 inflight */
		inf = calloc(1, sizeof(*inf));
		if (inf) {
			inf->seq = op->seq;
			inf->stripe_id = op->rec.stripe_id;
			pthread_spin_lock(&ctx->slot_lock);
			TAILQ_INSERT_TAIL(&ctx->inflight, inf, link);
			pthread_spin_unlock(&ctx->slot_lock);
		}
		op->cb(0, op->seq, op->cb_arg);
	} else {
		/* 写失败：slot/seq 已预占，留洞（replay 时 magic+crc 过滤），不影响正确性 */
		op->cb(op->status, 0, op->cb_arg);
	}
	spdk_free(op->buf);
	free(op);
}

void
poweraid_raid_common_ppl_append_record(struct poweraid_raid_common_ppl_ctx *ctx,
				  struct spdk_io_channel *ch,
				  uint64_t stripe_id, uint64_t chunk_bitmap,
				  uint64_t old_data_hash, uint64_t new_data_hash,
				  poweraid_raid_common_ppl_append_cb cb, void *cb_arg)
{
	struct ppl_append_op *op;
	struct poweraid_raid_common_ppl_waiter *w;
	uint64_t seq;
	uint32_t slot;

	if (ctx == NULL || ch == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, 0, cb_arg); }
		return;
	}

	/* slot_lock 保护 tail_slot/next_seq/super/inflight/waiters（阶段 C1 修复并发 bug）*/
	pthread_spin_lock(&ctx->slot_lock);
	if (ctx->tail_slot >= ctx->max_slots) {
		/* 环形回收：所有已 append 的 record 均已 commit（inflight 空）后，
		 * slot 回卷到 1。seq 仍由 super.next_seq 单调分配，与 slot 解耦；
		 * 盘上未及覆盖的陈旧 record 在 replay 时按 seq <= commit_seq 过滤。*/
		if (!TAILQ_EMPTY(&ctx->inflight)) {
			/* backpressure：PPL 满且有 in-flight 未 commit。
			 * 不再降级写无 PPL 保护（issue #14），而是挂起请求，
			 * 等 commit 释放 slot 后由 ppl_wake_waiters 唤醒重发。*/
			w = calloc(1, sizeof(*w));
			if (!w) {
				pthread_spin_unlock(&ctx->slot_lock);
				cb(-ENOMEM, 0, cb_arg);
				return;
			}
			w->ch = ch;
			w->stripe_id = stripe_id;
			w->chunk_bitmap = chunk_bitmap;
			w->old_data_hash = old_data_hash;
			w->new_data_hash = new_data_hash;
			w->cb = cb;
			w->cb_arg = cb_arg;
			TAILQ_INSERT_TAIL(&ctx->waiters, w, link);
			pthread_spin_unlock(&ctx->slot_lock);
			SPDK_NOTICELOG("ppl_append: log full, backpressure queueing "
				       "(ctx=%p tail=%u max=%u waiters++\n",
				       ctx, ctx->tail_slot, ctx->max_slots);
			return;
		}
		SPDK_NOTICELOG("poweraid_raid_common_ppl: ring recycle (next_seq=%"PRIu64")\n",
			       ctx->super.next_seq);
		ctx->tail_slot = 1;
	}

	/* 同步预占 slot + seq：避免并发 append 在异步写完成前拿到相同 slot/seq。
	 * 此处递增后即使写失败也只留洞，不影响正确性（replay magic+crc 过滤）。*/
	seq = ctx->super.next_seq;
	slot = ctx->tail_slot;
	ctx->tail_slot++;
	ctx->super.next_seq = seq + 1;
	ctx->super.tail_seq = seq + 1;
	ctx->super.num_records++;
	pthread_spin_unlock(&ctx->slot_lock);

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
	op->rec.magic = POWERAID_RAID_COMMON_PPL_REC_MAGIC;
	op->rec.flags = POWERAID_RAID_COMMON_PPL_REC_F_VALID;
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

/*
 * backpressure 唤醒：commit 释放 in-flight slot 后，检查等待队列。
 * 若有空闲 slot（tail_slot < max_slots）或可回卷（inflight 空 → tail_slot=1），
 * 逐个唤醒等待者重发 append。append_record 同步预占 slot，确保不会过度唤醒。
 */
static void
ppl_wake_waiters(struct poweraid_raid_common_ppl_ctx *ctx)
{
	struct poweraid_raid_common_ppl_waiter *w;

	while (1) {
		pthread_spin_lock(&ctx->slot_lock);
		w = TAILQ_FIRST(&ctx->waiters);
		if (w == NULL) {
			pthread_spin_unlock(&ctx->slot_lock);
			break;
		}
		if (ctx->tail_slot >= ctx->max_slots) {
			if (TAILQ_EMPTY(&ctx->inflight)) {
				SPDK_NOTICELOG("ppl_wake: ring recycle (next_seq=%"PRIu64")\n",
					       ctx->super.next_seq);
				ctx->tail_slot = 1;
			} else {
				pthread_spin_unlock(&ctx->slot_lock);
				break;  /* 仍然满，等待下次 commit */
			}
		}
		TAILQ_REMOVE(&ctx->waiters, w, link);
		pthread_spin_unlock(&ctx->slot_lock);

		SPDK_DEBUGLOG(raid5f_ppl, "ppl_wake: re-issue append stripe=%"PRIu64
			      " bitmap=0x%"PRIx64"\n", w->stripe_id, w->chunk_bitmap);
		/* 重发 append：同步预占 slot（内部加 slot_lock），若再次满则重新入队 */
		poweraid_raid_common_ppl_append_record(ctx, w->ch,
						       w->stripe_id, w->chunk_bitmap,
						       w->old_data_hash, w->new_data_hash,
						       w->cb, w->cb_arg);
		free(w);
	}
}

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
	struct poweraid_raid_common_ppl_ctx *ctx = op->ctx;
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
		struct poweraid_raid_common_ppl_inflight *inf, *tmp;
		uint32_t removed = 0;
		pthread_spin_lock(&ctx->slot_lock);
		TAILQ_FOREACH_SAFE(inf, &ctx->inflight, link, tmp) {
			if (inf->seq <= op->commit_seq) {
				TAILQ_REMOVE(&ctx->inflight, inf, link);
				free(inf);
				removed++;
			}
		}
		/* 更新 num_records / head_seq 以反映已 commit 的回收量 */
		if (removed > 0) {
			ctx->super.num_records = (ctx->super.num_records > removed)
				? (ctx->super.num_records - removed) : 0;
			ctx->super.head_seq = op->commit_seq + 1;
		}
		pthread_spin_unlock(&ctx->slot_lock);
		/* backpressure 唤醒：释放 slot 后通知等待者 */
		ppl_wake_waiters(ctx);
	}
	op->cb(op->status, op->cb_arg);
	spdk_free(op->buf);
	free(op);
}

void
poweraid_raid_common_ppl_commit(struct poweraid_raid_common_ppl_ctx *ctx,
			  struct spdk_io_channel *ch, uint64_t seq,
			  poweraid_raid_common_ppl_commit_cb cb, void *cb_arg)
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

	pthread_spin_lock(&ctx->slot_lock);
	ctx->super.commit_seq = seq;
	ppl_super_set_crc(&ctx->super);
	pthread_spin_unlock(&ctx->slot_lock);

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
ppl_replay_insert(struct ppl_replay_op *op, const struct poweraid_raid_common_ppl_record *rec)
{
	struct poweraid_raid_common_ppl_record *arr;
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
		const struct poweraid_raid_common_ppl_record *rec =
			(const struct poweraid_raid_common_ppl_record *)p;
		if (rec->magic == POWERAID_RAID_COMMON_PPL_REC_MAGIC &&
		    ppl_rec_check(rec)) {
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
	struct poweraid_raid_common_ppl_ctx *ctx = op->ctx;
	uint32_t remaining, slots_this;
	uint64_t byte_off;
	int rc;

	if (op->status != 0 || op->cur_slot >= op->end_slot) {
		if (op->status == 0 && op->result_cnt > 0) {
			SPDK_NOTICELOG("poweraid_raid_common_ppl: replay found %u uncommitted records\n",
				       op->result_cnt);
			op->cb(0, op->result, op->result_cnt, op->cb_arg);
		} else if (op->status == 0) {
			SPDK_NOTICELOG("poweraid_raid_common_ppl: replay clean (no uncommitted)\n");
			op->cb(0, NULL, 0, op->cb_arg);
		} else {
			SPDK_ERRLOG("poweraid_raid_common_ppl: replay failed (%d)\n", op->status);
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
	struct poweraid_raid_common_ppl_ctx *ctx = sop->ctx;
	const struct poweraid_raid_common_ppl_super *s;
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

	s = (const struct poweraid_raid_common_ppl_super *)sop->buf;
	if (s->magic != POWERAID_RAID_COMMON_PPL_SUPER_MAGIC || !ppl_super_check(s)) {
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
	struct poweraid_raid_common_ppl_ctx *ctx = sop->ctx;
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
poweraid_raid_common_ppl_load_replay(struct poweraid_raid_common_ppl_ctx *ctx,
				poweraid_raid_common_ppl_replay_cb cb, void *cb_arg)
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
poweraid_raid_common_ppl_free_replay_result(struct poweraid_raid_common_ppl_record *records)
{
	free(records);
}

uint64_t
poweraid_raid_common_ppl_data_hash(const void *buf, size_t len)
{
	uint32_t lo, hi;
	/* 两段 crc32c，不同 seed，拼成 64-bit hash */
	lo = spdk_crc32c_update(buf, len, 0);
	hi = spdk_crc32c_update(buf, len, ~0u);
	return ((uint64_t)hi << 32) | (uint64_t)lo;
}

void
poweraid_raid_common_ppl_hash_init(struct poweraid_raid_common_ppl_hash_ctx *c)
{
	c->lo = 0;
	c->hi = ~0u;
}

void
poweraid_raid_common_ppl_hash_update(struct poweraid_raid_common_ppl_hash_ctx *c,
				const void *buf, size_t len)
{
	c->lo = spdk_crc32c_update(buf, len, c->lo);
	c->hi = spdk_crc32c_update(buf, len, c->hi);
}

uint64_t
poweraid_raid_common_ppl_hash_final(struct poweraid_raid_common_ppl_hash_ctx *c)
{
	return ((uint64_t)c->hi << 32) | (uint64_t)c->lo;
}

/* ===== Group Commit 实现（阶段 C1）=====
 *
 * 三处屏障批合并，per-thread gc_ctx 无锁 + 共享 ppl_ctx slot_lock。
 * 详见 poweraid_raid_common_ppl.h 注释。
 */

/* 命名 TAILQ head 类型：TAILQ_LAST 需要具名 head type，匿名 TAILQ_HEAD 不行。
 * gc_append_flush_op / gc_data_flush_op / gc_commit_op 中的 group 字段布局一致，
 * 均为 TAILQ_HEAD(, poweraid_raid_common_ppl_gc_entry)，可安全强转。*/
TAILQ_HEAD(gc_entry_list, poweraid_raid_common_ppl_gc_entry);

/* 前向声明 */
static int ppl_gc_poller(void *arg);
static void ppl_gc_append_flush_retry(void *arg);
static void ppl_gc_data_flush_check_trigger(struct poweraid_raid_common_ppl_gc *gc);
static void ppl_gc_commit_check_trigger(struct poweraid_raid_common_ppl_gc *gc,
					struct poweraid_raid_common_ppl_ctx *ctx);
static void ppl_gc_append_check_trigger(struct poweraid_raid_common_ppl_gc *gc, bool force);
static void ppl_gc_append_resume_waiting(struct poweraid_raid_common_ppl_gc *gc);

/* ===== append group ===== */

/* 单条 record write 完成 cb：仅标记失败，不推进状态（flush 完成才推进）*/

static void
ppl_gc_append_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct poweraid_raid_common_ppl_gc_entry *entry = cb_arg;
	struct poweraid_raid_common_ppl_gc *gc = entry->gc;

	spdk_bdev_free_io(bdev_io);
	assert(gc->append_write_inflight > 0);
	gc->append_write_inflight--;
	if (!success) {
		entry->append_write_failed = true;
		SPDK_ERRLOG("gc_append: write failed slot=%u seq=%"PRIu64"\n",
			    entry->slot, entry->seq);
	}

	/* record 写全部完成后，append flush 组才允许发出 */
	ppl_gc_append_check_trigger(gc, false);
}

/* range flush op：跟踪当前 flush group */
struct gc_append_flush_op {
	struct poweraid_raid_common_ppl_gc		*gc;
	struct spdk_io_channel				*ch;
	TAILQ_HEAD(, poweraid_raid_common_ppl_gc_entry)	group;
	uint32_t					count;
	uint64_t					flush_off;
	uint64_t					flush_len;
	struct spdk_bdev_io_wait_entry			wait_entry;
};

static void
ppl_gc_append_flush_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct gc_append_flush_op *op = cb_arg;
	struct poweraid_raid_common_ppl_gc *gc = op->gc;
	struct poweraid_raid_common_ppl_gc_entry *entry, *tmp;
	struct poweraid_raid_common_ppl_inflight *inf;

	/* 提交即失败的路径（ENOMEM 重试耗尽/EINVAL）会以 bdev_io==NULL 直接调本回调 */
	if (bdev_io != NULL) {
		spdk_bdev_free_io(bdev_io);
	}

	if (!success) {
		SPDK_ERRLOG("gc_append: range flush failed (count=%u)\n", op->count);
	}

	TAILQ_FOREACH_SAFE(entry, &op->group, link, tmp) {
		spdk_free(entry->append_buf);
		entry->append_buf = NULL;

		if (success && !entry->append_write_failed) {
			/* 成功：插入 inflight（slot_lock 保护），回调，entry 保留供后续 data_flush/commit */
			inf = calloc(1, sizeof(*inf));
			if (inf) {
				inf->seq = entry->seq;
				inf->stripe_id = entry->stripe_id;
				pthread_spin_lock(&gc->ppl_ctx->slot_lock);
				TAILQ_INSERT_TAIL(&gc->ppl_ctx->inflight, inf, link);
				pthread_spin_unlock(&gc->ppl_ctx->slot_lock);
			}
			entry->append_cb(0, entry->seq, entry->append_cb_arg);
			/* entry 不释放：贯穿后续 data_flush / commit 阶段 */
		} else {
			/* 失败：回调后释放 entry（调用方收到 error 后设 gc_entry=NULL，不再使用）*/
			entry->append_cb(-EIO, 0, entry->append_cb_arg);
			free(entry);
		}
	}

	gc->append_inflight = false;

	/* 链式：flush 完成后若 pending 非空，立即触发下一 group */
	if (!TAILQ_EMPTY(&gc->append_pending)) {
		ppl_gc_append_check_trigger(gc, false);
	}

	free(op);
}

/* idle 快路径判断：仅 1 pending 且无 inflight 且无 commit 活动 */
static bool
ppl_gc_append_idle_fastpath(const struct poweraid_raid_common_ppl_gc *gc)
{
	return gc->append_pending_count == 1 &&
	       !gc->append_inflight &&
	       gc->commit_pending_count == 0 &&
	       !gc->commit_inflight;
}

/* ENOMEM 重试：op 仍持有 group entries，重新发 range flush。
 * 作为 spdk_bdev_io_wait_cb 被 spdk_bdev_queue_io_wait 回调。*/
static void
ppl_gc_append_flush_retry(void *arg)
{
	struct gc_append_flush_op *op = arg;
	struct poweraid_raid_common_ppl_gc *gc = op->gc;
	struct poweraid_raid_common_ppl_ctx *ctx = gc->ppl_ctx;
	int rc;

	rc = spdk_bdev_flush_blocks(ctx->bdev_desc, op->ch,
				    byte_to_block_offset(ctx, op->flush_off),
				    op->flush_len >> ctx->block_shift,
				    ppl_gc_append_flush_cb, op);
	if (rc == -ENOMEM) {
		/* 仍资源不足：重新 arm wait */
		op->wait_entry.bdev = spdk_bdev_desc_get_bdev(ctx->bdev_desc);
		op->wait_entry.cb_fn = ppl_gc_append_flush_retry;
		op->wait_entry.cb_arg = op;
		spdk_bdev_queue_io_wait(ctx->bdev_desc, op->ch, &op->wait_entry);
	} else if (rc != 0) {
		SPDK_ERRLOG("gc_append: flush retry failed rc=%d\n", rc);
		ppl_gc_append_flush_cb(NULL, false, op);
	}
}

/* 检查 append 触发条件，满足则发 range flush */
static void
ppl_gc_append_check_trigger(struct poweraid_raid_common_ppl_gc *gc, bool force)
{
	struct poweraid_raid_common_ppl_ctx *ctx = gc->ppl_ctx;
	struct gc_append_flush_op *op;
	struct poweraid_raid_common_ppl_gc_entry *first, *last;
	uint64_t flush_off;
	uint64_t flush_len;
	int rc;

	if (gc->append_inflight || gc->append_pending_count == 0) {
		return;
	}

	/* 必须等组内所有 record 写都已提交且完成后再 flush：
	 * - 无 FUA/flush 能力的盘上 flush 会立即完成，ppl_gc_append_flush_cb
	 *   随即释放 append_buf；若 record 写仍在途，其 DMA 载荷会被后续
	 *   malloc/memset 回收覆盖（实测落盘为全零 record），形成 UAF；
	 * - 同时必须保证 record 先于后续 data 写持久化的顺序。*/
	if (gc->append_unsubmitted_count != 0 || gc->append_write_inflight != 0) {
		return;
	}

	/* 触发条件：K 达到 / idle 快路径 / poller T_append 超时强制（force）。
	 * force 仍受上面的未完成 record 写门控约束，不能提前 flush。*/
	bool trigger = force ||
		       (gc->append_pending_count >= POWERAID_RAID_COMMON_PPL_GC_K) ||
		       ppl_gc_append_idle_fastpath(gc);
	if (!trigger) {
		return;
	}

	/* 收集当前 pending 全部 entry 到 flush op */
	op = calloc(1, sizeof(*op));
	if (!op) {
		SPDK_ERRLOG("gc_append: flush op alloc failed, retry next poll\n");
		return;
	}
	op->gc = gc;
	op->ch = TAILQ_FIRST(&gc->append_pending)->ch;
	TAILQ_INIT(&op->group);
	TAILQ_SWAP(&gc->append_pending, &op->group, poweraid_raid_common_ppl_gc_entry, link);
	/* TAILQ_SWAP 不清空源 head，手动清 */
	TAILQ_INIT(&gc->append_pending);
	op->count = gc->append_pending_count;
	gc->append_pending_count = 0;
	gc->append_inflight = true;


	/* 计算 flush range [first_slot, last_slot]。
	 * ring 回卷时（last slot 号 < first，entry 按入队顺序排列）两段不连续，
	 * 直接相减会下溢成超长范围导致 spdk_bdev_flush_blocks 返回 -EINVAL。
	 * NVMe FLUSH 本身不带范围（刷整个 namespace），回卷时刷整个 PPL region
	 * 与刷两个分段语义等价，且不会触及 region 之外的数据。*/
	first = TAILQ_FIRST(&op->group);
	last = TAILQ_LAST(&op->group, gc_entry_list);
	if (last->slot >= first->slot) {
		op->flush_off = first->slot_byte_off;
		op->flush_len = (last->slot_byte_off - first->slot_byte_off) + ctx->block_size;
	} else {
		op->flush_off = ctx->region_offset;
		op->flush_len = ctx->region_size;
	}

	SPDK_DEBUGLOG(raid5f_ppl, "gc_append flush: count=%u range=[%lu, +%lu)\n",
		      op->count, (unsigned long)op->flush_off, (unsigned long)op->flush_len);

	rc = spdk_bdev_flush_blocks(ctx->bdev_desc, op->ch,
				    byte_to_block_offset(ctx, op->flush_off),
				    op->flush_len >> ctx->block_shift,
				    ppl_gc_append_flush_cb, op);
	if (rc == -ENOMEM) {
		/* 资源不足：arm wait，retry 回调直接重发 flush（op 仍持有 entries）*/
		op->wait_entry.bdev = spdk_bdev_desc_get_bdev(ctx->bdev_desc);
		op->wait_entry.cb_fn = ppl_gc_append_flush_retry;
		op->wait_entry.cb_arg = op;
		spdk_bdev_queue_io_wait(ctx->bdev_desc, op->ch, &op->wait_entry);
		return;
	} else if (rc != 0) {
		SPDK_ERRLOG("gc_append: flush failed rc=%d\n", rc);
		/* 直接调 flush_cb 模拟 flush 失败，cb 会 fail 全组并清 inflight */
		ppl_gc_append_flush_cb(NULL, false, op);
		return;
	}
}

/* 为 waiting_for_slot 的 entry 尝试预留 slot 并发 write */
static void
ppl_gc_append_resume_waiting(struct poweraid_raid_common_ppl_gc *gc)
{
	struct poweraid_raid_common_ppl_ctx *ctx = gc->ppl_ctx;
	struct poweraid_raid_common_ppl_gc_entry *entry, *tmp;
	struct poweraid_raid_common_ppl_record rec;
	uint64_t seq;
	uint32_t slot;
	int rc;

	TAILQ_FOREACH_SAFE(entry, &gc->append_pending, link, tmp) {
		if (!entry->waiting_for_slot) {
			continue;
		}

		pthread_spin_lock(&ctx->slot_lock);
		if (ctx->tail_slot >= ctx->max_slots) {
			if (!TAILQ_EMPTY(&ctx->inflight)) {
				pthread_spin_unlock(&ctx->slot_lock);
				continue;  /* 仍然满 */
			}
			ctx->tail_slot = 1;
		}
		seq = ctx->super.next_seq;
		slot = ctx->tail_slot;
		ctx->tail_slot++;
		ctx->super.next_seq = seq + 1;
		ctx->super.tail_seq = seq + 1;
		ctx->super.num_records++;
		pthread_spin_unlock(&ctx->slot_lock);

		entry->seq = seq;
		entry->slot = slot;
		entry->slot_byte_off = slot_to_byte_offset(ctx, slot);
		entry->waiting_for_slot = false;

		/* 构造 record 并发 write */
		memset(&rec, 0, sizeof(rec));
		rec.magic = POWERAID_RAID_COMMON_PPL_REC_MAGIC;
		rec.flags = POWERAID_RAID_COMMON_PPL_REC_F_VALID;
		rec.seq = seq;
		rec.stripe_id = entry->stripe_id;
		rec.chunk_bitmap = entry->chunk_bitmap;
		rec.old_data_hash = entry->old_data_hash;
		rec.new_data_hash = entry->new_data_hash;
		rec.ts = (uint64_t)time(NULL);
		ppl_rec_set_crc(&rec);

		entry->append_buf = spdk_dma_malloc(ctx->block_size, 0x1000, NULL);
		if (!entry->append_buf) {
			entry->append_write_failed = true;
			gc->append_unsubmitted_count--;  /* 无在途写，直接允许组触发 */
			continue;
		}
		memset(entry->append_buf, 0, ctx->block_size);
		memcpy(entry->append_buf, &rec, sizeof(rec));

		rc = spdk_bdev_write_blocks(ctx->bdev_desc, entry->ch,
					    entry->append_buf,
					    byte_to_block_offset(ctx, entry->slot_byte_off),
					    1, ppl_gc_append_write_cb, entry);
		if (rc != 0) {
			SPDK_ERRLOG("gc_append: write failed rc=%d (waiting entry)\n", rc);
			entry->append_write_failed = true;
			/* 提交失败：无在途写，直接允许组触发 */
			gc->append_unsubmitted_count--;
		} else {
			gc->append_unsubmitted_count--;
			gc->append_write_inflight++;
		}
	}

	/* 可能本批 waiting entry 是最后的未提交写，尝试触发 append flush */
	ppl_gc_append_check_trigger(gc, false);
}

void
poweraid_raid_common_ppl_gc_append(
	struct poweraid_raid_common_ppl_ctx *ctx,
	struct poweraid_raid_common_ppl_gc *gc,
	struct spdk_io_channel *ch,
	uint64_t stripe_id, uint64_t chunk_bitmap,
	uint64_t old_data_hash, uint64_t new_data_hash,
	struct poweraid_raid_common_ppl_gc_entry **entry_out,
	poweraid_raid_common_ppl_append_cb cb, void *cb_arg)
{
	struct poweraid_raid_common_ppl_gc_entry *entry;
	struct poweraid_raid_common_ppl_record rec;
	uint64_t seq;
	uint32_t slot;
	bool slot_reserved = false;
	int rc;

	if (ctx == NULL || gc == NULL || ch == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, 0, cb_arg); }
		return;
	}

	/* gc 未初始化（poller 为 NULL）→ fallback 到逐条 append_record。
	 * entry_out 设 NULL，调用方据此走非 gc 路径的 data flush / commit。*/
	if (gc->poller == NULL) {
		*entry_out = NULL;
		poweraid_raid_common_ppl_append_record(ctx, ch, stripe_id, chunk_bitmap,
						       old_data_hash, new_data_hash, cb, cb_arg);
		return;
	}

	entry = calloc(1, sizeof(*entry));
	if (!entry) {
		cb(-ENOMEM, 0, cb_arg);
		return;
	}
	entry->stripe_id = stripe_id;
	entry->chunk_bitmap = chunk_bitmap;
	entry->old_data_hash = old_data_hash;
	entry->new_data_hash = new_data_hash;
	entry->ch = ch;
	entry->gc = gc;
	entry->append_cb = cb;
	entry->append_cb_arg = cb_arg;
	entry->append_enqueue_ticks = spdk_get_ticks();
	*entry_out = entry;

	/* 尝试预留 slot/seq */
	pthread_spin_lock(&ctx->slot_lock);
	if (ctx->tail_slot >= ctx->max_slots) {
		if (!TAILQ_EMPTY(&ctx->inflight)) {
			/* PPL 满：entry 挂入 append_pending，标记 waiting_for_slot，
			 * poller 在 commit 释放 slot 后重试 */
			entry->waiting_for_slot = true;
		} else {
			ctx->tail_slot = 1;  /* ring recycle */
		}
	}
	if (!entry->waiting_for_slot) {
		seq = ctx->super.next_seq;
		slot = ctx->tail_slot;
		ctx->tail_slot++;
		ctx->super.next_seq = seq + 1;
		ctx->super.tail_seq = seq + 1;
		ctx->super.num_records++;
		entry->seq = seq;
		entry->slot = slot;
		entry->slot_byte_off = slot_to_byte_offset(ctx, slot);
		slot_reserved = true;
	}
	pthread_spin_unlock(&ctx->slot_lock);

	/* 入队 append_pending */
	TAILQ_INSERT_TAIL(&gc->append_pending, entry, link);
	gc->append_pending_count++;
	/* 新 entry 的 record 写尚未提交（waiting_for_slot 时由 resume 路径提交）*/
	gc->append_unsubmitted_count++;
	if (gc->append_pending_count == 1) {
		gc->append_oldest_ticks = entry->append_enqueue_ticks;
	}

	if (slot_reserved) {
		/* 构造 record 并发 write（不 flush）*/
		memset(&rec, 0, sizeof(rec));
		rec.magic = POWERAID_RAID_COMMON_PPL_REC_MAGIC;
		rec.flags = POWERAID_RAID_COMMON_PPL_REC_F_VALID;
		rec.seq = seq;
		rec.stripe_id = stripe_id;
		rec.chunk_bitmap = chunk_bitmap;
		rec.old_data_hash = old_data_hash;
		rec.new_data_hash = new_data_hash;
		rec.ts = (uint64_t)time(NULL);
		ppl_rec_set_crc(&rec);

		entry->append_buf = spdk_dma_malloc(ctx->block_size, 0x1000, NULL);
		if (!entry->append_buf) {
			entry->append_write_failed = true;
			gc->append_unsubmitted_count--;  /* 无在途写，直接允许组触发 */
		} else {
			memset(entry->append_buf, 0, ctx->block_size);
			memcpy(entry->append_buf, &rec, sizeof(rec));

			rc = spdk_bdev_write_blocks(ctx->bdev_desc, ch,
						    entry->append_buf,
						    byte_to_block_offset(ctx, entry->slot_byte_off),
						    1, ppl_gc_append_write_cb, entry);
			if (rc != 0) {
				SPDK_ERRLOG("gc_append: write_blocks rc=%d\n", rc);
				entry->append_write_failed = true;
				/* 提交失败：无在途写，直接允许组触发 */
				gc->append_unsubmitted_count--;
			} else {
				gc->append_unsubmitted_count--;
				gc->append_write_inflight++;
			}
		}
	}

	/* 检查触发条件 */
	ppl_gc_append_check_trigger(gc, false);
}

/* ===== data flush group ===== */

struct gc_data_flush_op {
	struct poweraid_raid_common_ppl_gc		*gc;
	TAILQ_HEAD(, poweraid_raid_common_ppl_gc_entry)	group;
	uint32_t					count;
	uint32_t					remaining;  /* 未完成 per-disk flush 数 */
	bool						failed;
	/* per-disk range 聚合（从 gc->data_flush_ranges 拷贝，避免 inflight 期间被新 enqueue 污染）*/
	struct {
		uint64_t				min_off;
		uint64_t				max_off;
		bool					active;
	} ranges[POWERAID_RAID_COMMON_PPL_GC_MAX_BDEVS];
};

static void
ppl_gc_data_flush_done(struct gc_data_flush_op *op)
{
	struct poweraid_raid_common_ppl_gc *gc = op->gc;
	struct poweraid_raid_common_ppl_gc_entry *entry, *tmp;
	int status = op->failed ? -EIO : 0;

	SPDK_DEBUGLOG(raid5f_ppl, "gc_data_flush_done: count=%u failed=%d status=%d\n",
		      op->count, op->failed, status);

	TAILQ_FOREACH_SAFE(entry, &op->group, link, tmp) {
		entry->data_flush_cb(status, entry->data_flush_cb_arg);
		/* entry 不释放：贯穿 commit 阶段 */
	}

	gc->data_flush_inflight = false;
	/* 注：gc->data_flush_ranges 已在 trigger 时拷贝到 op 并重置，此处不再清 */

	/* 链式 */
	if (!TAILQ_EMPTY(&gc->data_flush_pending)) {
		ppl_gc_data_flush_check_trigger(gc);
	}

	free(op);
}

static void
ppl_gc_data_flush_per_disk_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct gc_data_flush_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	SPDK_DEBUGLOG(raid5f_ppl, "gc_data_flush per_disk_cb: success=%d remaining=%u\n",
		      success, op->remaining - 1);
	if (!success) {
		op->failed = true;
		SPDK_ERRLOG("gc_data_flush: per-disk flush failed (remaining=%u)\n",
			    op->remaining - 1);
	}

	if (--op->remaining == 0) {
		ppl_gc_data_flush_done(op);
	}
}

static void
ppl_gc_data_flush_check_trigger(struct poweraid_raid_common_ppl_gc *gc)
{
	struct gc_data_flush_op *op;
	struct raid_bdev *raid_bdev = gc->raid_bdev;
	struct raid_bdev_io_channel *raid_ch = gc->raid_ch;
	uint32_t i, num_disks;
	int rc;

	if (gc->data_flush_inflight || gc->data_flush_pending_count == 0) {
		return;
	}

	/* raid_bdev / raid_ch 未设置（理论不应到达，首次 IO 即填充）→ fail 全组 */
	if (raid_bdev == NULL || raid_ch == NULL) {
		SPDK_ERRLOG("gc_data_flush: raid_bdev or raid_ch is NULL\n");
		/* fail 所有 pending entry */
		{
			struct poweraid_raid_common_ppl_gc_entry *entry, *tmp;
			TAILQ_FOREACH_SAFE(entry, &gc->data_flush_pending, link, tmp) {
				TAILQ_REMOVE(&gc->data_flush_pending, entry, link);
				entry->data_flush_cb(-ENODEV, entry->data_flush_cb_arg);
			}
			gc->data_flush_pending_count = 0;
			memset(gc->data_flush_ranges, 0, sizeof(gc->data_flush_ranges));
		}
		return;
	}

	bool trigger = (gc->data_flush_pending_count >= POWERAID_RAID_COMMON_PPL_GC_K) ||
		       (gc->data_flush_pending_count == 1 &&
			!gc->data_flush_inflight &&
			gc->append_pending_count == 0 &&
			!gc->append_inflight &&
			gc->commit_pending_count == 0 &&
			!gc->commit_inflight);

	SPDK_DEBUGLOG(raid5f_ppl, "gc_data_flush trigger: pending=%u inflight=%d append_pend=%u append_inf=%d commit_pend=%u commit_inf=%d trigger=%d\n",
		      gc->data_flush_pending_count, gc->data_flush_inflight,
		      gc->append_pending_count, gc->append_inflight,
		      gc->commit_pending_count, gc->commit_inflight, trigger);
	/* T_data 超时强制触发：避免低并发下 batch 不满而 stall */
	if (!trigger && gc->data_flush_oldest_ticks != 0) {
		uint64_t t_data = (POWERAID_RAID_COMMON_PPL_GC_T_DATA_US *
				   spdk_get_ticks_hz()) / 1000000ULL;
		if ((spdk_get_ticks() - gc->data_flush_oldest_ticks) >= t_data) {
			trigger = true;
		}
	}
	if (!trigger) {
		return;
	}

	/* 收集 group */
	op = calloc(1, sizeof(*op));
	if (!op) {
		SPDK_ERRLOG("gc_data_flush: op alloc failed\n");
		return;
	}
	op->gc = gc;
	TAILQ_INIT(&op->group);
	TAILQ_SWAP(&gc->data_flush_pending, &op->group, poweraid_raid_common_ppl_gc_entry, link);
	TAILQ_INIT(&gc->data_flush_pending);
	op->count = gc->data_flush_pending_count;
	gc->data_flush_pending_count = 0;
	gc->data_flush_oldest_ticks = 0;
	gc->data_flush_inflight = true;
	op->failed = false;

	/* 拷贝 per-disk range 聚合到 op，并立即重置 gc->data_flush_ranges。
	 * 这样 inflight 期间新 enqueue 的 entry 的 range 不会被丢失（下一 group 独立聚合）。*/
	memcpy(op->ranges, gc->data_flush_ranges, sizeof(op->ranges));
	memset(gc->data_flush_ranges, 0, sizeof(gc->data_flush_ranges));

	/* 统计有多少个活跃盘需要 flush */
	num_disks = 0;
	for (i = 0; i < POWERAID_RAID_COMMON_PPL_GC_MAX_BDEVS; i++) {
		if (op->ranges[i].active) {
			num_disks++;
		}
	}
	op->remaining = num_disks;

	SPDK_DEBUGLOG(raid5f_ppl, "gc_data_flush: group formed count=%u num_disks=%u\n",
		      op->count, num_disks);

	if (num_disks == 0) {
		/* 无盘需 flush（理论不应发生），直接完成 */
		ppl_gc_data_flush_done(op);
		return;
	}

	/* 每盘发一次 range flush：通过 raid_bdev->base_bdev_info[phys] 获取 desc，
	 * 通过 raid_bdev_channel_get_base_channel(raid_ch, phys) 获取 base channel。
	 * offset/length 为 stripe 相对偏移（blocks），raid_bdev_flush_blocks 内部加 data_offset。*/
	for (i = 0; i < POWERAID_RAID_COMMON_PPL_GC_MAX_BDEVS; i++) {
		struct raid_base_bdev_info *base_info;
		struct spdk_io_channel *base_ch;
		uint64_t off, len;

		if (!op->ranges[i].active) {
			continue;
		}
		base_info = &raid_bdev->base_bdev_info[i];
		base_ch = raid_bdev_channel_get_base_channel(raid_ch, i);
		off = op->ranges[i].min_off;
		len = op->ranges[i].max_off - off;

		if (base_ch == NULL || base_info->desc == NULL) {
			SPDK_ERRLOG("gc_data_flush: disk %u unavailable\n", i);
			op->failed = true;
			if (--op->remaining == 0) {
				ppl_gc_data_flush_done(op);
			}
			continue;
		}

		rc = raid_bdev_flush_blocks(base_info, base_ch, off, len,
					    ppl_gc_data_flush_per_disk_cb, op);
		SPDK_DEBUGLOG(raid5f_ppl, "gc_data_flush: disk %u flush off=%lu len=%lu rc=%d remaining=%u\n",
			      i, (unsigned long)off, (unsigned long)len, rc, op->remaining);
		if (rc != 0) {
			SPDK_ERRLOG("gc_data_flush: disk %u flush rc=%d\n", i, rc);
			op->failed = true;
			if (--op->remaining == 0) {
				ppl_gc_data_flush_done(op);
			}
		}
	}
}

void
poweraid_raid_common_ppl_gc_data_flush_enqueue(
	struct poweraid_raid_common_ppl_gc *gc,
	struct poweraid_raid_common_ppl_gc_entry *entry,
	const struct poweraid_raid_common_ppl_data_range *ranges,
	uint32_t num_ranges,
	poweraid_raid_common_ppl_data_flush_cb cb, void *cb_arg)
{
	uint32_t i;

	SPDK_DEBUGLOG(raid5f_ppl, "gc_data_flush_enqueue: gc=%p entry=%p num_ranges=%u pending=%u inflight=%d\n",
		      gc, entry, num_ranges, gc ? gc->data_flush_pending_count : 0,
		      gc ? gc->data_flush_inflight : -1);

	if (gc == NULL || entry == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, cb_arg); }
		return;
	}

	/* 复制 ranges 到 entry */
	entry->num_ranges = num_ranges;
	for (i = 0; i < num_ranges && i < POWERAID_RAID_COMMON_PPL_GC_MAX_BDEVS; i++) {
		entry->ranges[i] = ranges[i];
	}
	entry->data_flush_cb = cb;
	entry->data_flush_cb_arg = cb_arg;
	entry->data_flush_enqueue_ticks = spdk_get_ticks();

	/* 入队 */
	TAILQ_INSERT_TAIL(&gc->data_flush_pending, entry, link);
	gc->data_flush_pending_count++;
	if (gc->data_flush_pending_count == 1) {
		gc->data_flush_oldest_ticks = entry->data_flush_enqueue_ticks;
	}

	/* 按盘聚合 min/max offset */
	for (i = 0; i < num_ranges && i < POWERAID_RAID_COMMON_PPL_GC_MAX_BDEVS; i++) {
		uint8_t phys = ranges[i].phys;
		uint64_t off = ranges[i].offset;
		uint64_t end = off + ranges[i].length;
		if (phys >= POWERAID_RAID_COMMON_PPL_GC_MAX_BDEVS) {
			continue;
		}
		if (!gc->data_flush_ranges[phys].active) {
			gc->data_flush_ranges[phys].active = true;
			gc->data_flush_ranges[phys].min_off = off;
			gc->data_flush_ranges[phys].max_off = end;
		} else {
			if (off < gc->data_flush_ranges[phys].min_off) {
				gc->data_flush_ranges[phys].min_off = off;
			}
			if (end > gc->data_flush_ranges[phys].max_off) {
				gc->data_flush_ranges[phys].max_off = end;
			}
		}
	}

	ppl_gc_data_flush_check_trigger(gc);
}

/* ===== commit group ===== */

struct gc_commit_op {
	struct poweraid_raid_common_ppl_gc		*gc;
	struct poweraid_raid_common_ppl_ctx		*ctx;
	struct spdk_io_channel				*ch;
	TAILQ_HEAD(, poweraid_raid_common_ppl_gc_entry)	group;
	uint32_t					count;
	uint64_t					commit_seq;
	uint8_t						state;  /* 0=write, 1=flush, 2=done */
	int						status;
	void						*buf;
	struct spdk_bdev_io_wait_entry			wait_entry;
};

enum {
	GC_COMMIT_WRITE = 0,
	GC_COMMIT_FLUSH,
	GC_COMMIT_DONE,
};

static void
ppl_gc_commit_loop(struct gc_commit_op *op);

static void
ppl_gc_commit_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct gc_commit_op *op = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		op->status = -EIO;
		op->state = GC_COMMIT_DONE;
	} else {
		op->state++;
	}
	ppl_gc_commit_loop(op);
}

static void
ppl_gc_commit_loop(struct gc_commit_op *op)
{
	struct poweraid_raid_common_ppl_ctx *ctx = op->ctx;
	int rc;

	while (op->state < GC_COMMIT_DONE) {
		switch (op->state) {
		case GC_COMMIT_WRITE:
			memcpy(op->buf, &ctx->super, sizeof(ctx->super));
			SPDK_DEBUGLOG(raid5f_ppl, "gc_commit WRITE: commit_seq=%"PRIu64" next_seq=%"PRIu64
				      " tail_seq=%"PRIu64" num_records=%u count=%u\n",
				      ctx->super.commit_seq, ctx->super.next_seq,
				      ctx->super.tail_seq, ctx->super.num_records, op->count);
			rc = spdk_bdev_write_blocks(ctx->bdev_desc, op->ch, op->buf,
						    byte_to_block_offset(ctx, ctx->region_offset),
						    1, ppl_gc_commit_io_cb, op);
			if (rc == -ENOMEM) {
				op->wait_entry.bdev = spdk_bdev_desc_get_bdev(ctx->bdev_desc);
				op->wait_entry.cb_fn = (spdk_bdev_io_wait_cb)ppl_gc_commit_loop;
				op->wait_entry.cb_arg = op;
				spdk_bdev_queue_io_wait(ctx->bdev_desc, op->ch, &op->wait_entry);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = GC_COMMIT_DONE;
				continue;
			}
			return;

		case GC_COMMIT_FLUSH:
			rc = spdk_bdev_flush_blocks(ctx->bdev_desc, op->ch,
						    byte_to_block_offset(ctx, ctx->region_offset),
						    1, ppl_gc_commit_io_cb, op);
			if (rc == -ENOMEM) {
				op->wait_entry.bdev = spdk_bdev_desc_get_bdev(ctx->bdev_desc);
				op->wait_entry.cb_fn = (spdk_bdev_io_wait_cb)ppl_gc_commit_loop;
				op->wait_entry.cb_arg = op;
				spdk_bdev_queue_io_wait(ctx->bdev_desc, op->ch, &op->wait_entry);
				return;
			} else if (rc != 0) {
				op->status = rc;
				op->state = GC_COMMIT_DONE;
				continue;
			}
			return;

		default:
			op->status = -EINVAL;
			op->state = GC_COMMIT_DONE;
			break;
		}
	}

	/* DONE */
	if (op->status == 0) {
		struct poweraid_raid_common_ppl_inflight *inf, *tmp;
		uint32_t removed = 0;
		pthread_spin_lock(&ctx->slot_lock);
		TAILQ_FOREACH_SAFE(inf, &ctx->inflight, link, tmp) {
			if (inf->seq <= op->commit_seq) {
				TAILQ_REMOVE(&ctx->inflight, inf, link);
				free(inf);
				removed++;
			}
		}
		if (removed > 0) {
			ctx->super.num_records = (ctx->super.num_records > removed)
				? (ctx->super.num_records - removed) : 0;
			ctx->super.head_seq = op->commit_seq + 1;
		}
		pthread_spin_unlock(&ctx->slot_lock);
		ppl_wake_waiters(ctx);
	}

	/* 回调全组 */
	{
		struct poweraid_raid_common_ppl_gc_entry *entry, *tmp;
		int status = op->status;
		TAILQ_FOREACH_SAFE(entry, &op->group, link, tmp) {
			entry->commit_cb(status, entry->commit_cb_arg);
			free(entry);
		}
	}

	op->gc->commit_inflight = false;

	/* 链式 */
	if (!TAILQ_EMPTY(&op->gc->commit_pending)) {
		ppl_gc_commit_check_trigger(op->gc, op->ctx);
	}

	spdk_free(op->buf);
	free(op);
}

static void
ppl_gc_commit_check_trigger(struct poweraid_raid_common_ppl_gc *gc,
			    struct poweraid_raid_common_ppl_ctx *ctx)
{
	struct gc_commit_op *op;
	struct poweraid_raid_common_ppl_gc_entry *entry, *first;
	uint64_t max_seq = 0;

	if (gc->commit_inflight || gc->commit_pending_count == 0) {
		return;
	}

	bool trigger = (gc->commit_pending_count >= POWERAID_RAID_COMMON_PPL_GC_K) ||
	       (gc->commit_pending_count == 1 &&
			!gc->commit_inflight &&
			gc->append_pending_count == 0 &&
			!gc->append_inflight &&
			gc->data_flush_pending_count == 0 &&
			!gc->data_flush_inflight);

	/* T_commit 超时强制触发：避免低并发下 batch 不满而 stall */
	if (!trigger && gc->commit_oldest_ticks != 0) {
		uint64_t t_commit = (POWERAID_RAID_COMMON_PPL_GC_T_COMMIT_US *
				     spdk_get_ticks_hz()) / 1000000ULL;
		if ((spdk_get_ticks() - gc->commit_oldest_ticks) >= t_commit) {
			trigger = true;
		}
	}
	if (!trigger) {
		return;
	}

	op = calloc(1, sizeof(*op));
	if (!op) {
		SPDK_ERRLOG("gc_commit: op alloc failed\n");
		return;
	}
	op->gc = gc;
	op->ctx = ctx;
	op->ch = TAILQ_FIRST(&gc->commit_pending)->ch;
	TAILQ_INIT(&op->group);
	TAILQ_SWAP(&gc->commit_pending, &op->group, poweraid_raid_common_ppl_gc_entry, link);
	TAILQ_INIT(&gc->commit_pending);
	op->count = gc->commit_pending_count;
	gc->commit_pending_count = 0;
	gc->commit_oldest_ticks = 0;
	gc->commit_inflight = true;
	op->state = GC_COMMIT_WRITE;
	op->status = 0;

	/* commit_seq = max(seq in group) */
	TAILQ_FOREACH(entry, &op->group, link) {
		if (entry->seq > max_seq) {
			max_seq = entry->seq;
		}
	}
	op->commit_seq = max_seq;

	/* 更新 super.commit_seq（slot_lock 保护）*/
	pthread_spin_lock(&ctx->slot_lock);
	ctx->super.commit_seq = max_seq;
	ppl_super_set_crc(&ctx->super);
	pthread_spin_unlock(&ctx->slot_lock);

	op->buf = spdk_dma_malloc(ctx->block_size, 0x1000, NULL);
	if (!op->buf) {
		SPDK_ERRLOG("gc_commit: buf alloc failed\n");
		/* fail 全组 */
		TAILQ_FOREACH_SAFE(entry, &op->group, link, first) {
			entry->commit_cb(-ENOMEM, entry->commit_cb_arg);
			free(entry);
		}
		gc->commit_inflight = false;
		free(op);
		return;
	}
	memset(op->buf, 0, ctx->block_size);

	ppl_gc_commit_loop(op);
}

void
poweraid_raid_common_ppl_gc_commit(
	struct poweraid_raid_common_ppl_ctx *ctx,
	struct poweraid_raid_common_ppl_gc *gc,
	struct spdk_io_channel *ch,
	struct poweraid_raid_common_ppl_gc_entry *entry,
	poweraid_raid_common_ppl_commit_cb cb, void *cb_arg)
{
	if (ctx == NULL || gc == NULL || entry == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, cb_arg); }
		return;
	}

	SPDK_DEBUGLOG(raid5f_ppl, "gc_commit: entry=%p seq=%"PRIu64" commit_pending=%u commit_inflight=%d\n",
		      entry, entry->seq, gc->commit_pending_count, gc->commit_inflight);

	entry->ch = ch;
	entry->commit_cb = cb;
	entry->commit_cb_arg = cb_arg;

	TAILQ_INSERT_TAIL(&gc->commit_pending, entry, link);
	gc->commit_pending_count++;
	if (gc->commit_pending_count == 1) {
		gc->commit_oldest_ticks = spdk_get_ticks();
	}

	ppl_gc_commit_check_trigger(gc, ctx);
}

/* ===== poller ===== */

static int
ppl_gc_poller(void *arg)
{
	struct poweraid_raid_common_ppl_gc *gc = arg;
	uint64_t now = spdk_get_ticks();
	uint64_t hz = spdk_get_ticks_hz();
	uint64_t t_append = (POWERAID_RAID_COMMON_PPL_GC_T_APPEND_US * hz) / 1000000ULL;

	/* append T_append 超时 */
	if (!gc->append_inflight && gc->append_pending_count > 0) {
		/* 先尝试恢复 waiting_for_slot 的 entry */
		ppl_gc_append_resume_waiting(gc);
		/* T_append 超时必须以 force 透传：否则组内停留 1..K-1 个 entry 且
		 * 上层无新 IO 补满 K 时（如 fio qd 恰好停在 K-1），flush 永不
		 * 发出，append 回调不返回，形成自死锁。*/
		if ((now - gc->append_oldest_ticks) >= t_append) {
			ppl_gc_append_check_trigger(gc, true);
		} else {
			ppl_gc_append_check_trigger(gc, false);
		}
	}

	/* data flush / commit 超时由各自 trigger 内部检查 T_data / T_commit，
	 * poller 只需在有 pending 且无 inflight 时调用 trigger。*/
	if (!gc->data_flush_inflight && gc->data_flush_pending_count > 0) {
		ppl_gc_data_flush_check_trigger(gc);
	}

	if (!gc->commit_inflight && gc->commit_pending_count > 0) {
		ppl_gc_commit_check_trigger(gc, gc->ppl_ctx);
	}

	return 0;
}

/* ===== init / destroy ===== */

int
poweraid_raid_common_ppl_gc_init(struct poweraid_raid_common_ppl_gc *gc,
			      struct poweraid_raid_common_ppl_ctx *ppl_ctx,
			      void *raid_bdev, void *raid_ch)
{
	if (gc == NULL || ppl_ctx == NULL) {
		return -EINVAL;
	}

	memset(gc, 0, sizeof(*gc));
	gc->ppl_ctx = ppl_ctx;
	gc->raid_bdev = raid_bdev;
	gc->raid_ch = raid_ch;
	TAILQ_INIT(&gc->append_pending);
	TAILQ_INIT(&gc->data_flush_pending);
	TAILQ_INIT(&gc->commit_pending);
	gc->append_pending_count = 0;
	gc->append_inflight = false;
	gc->append_unsubmitted_count = 0;
	gc->append_write_inflight = 0;
	gc->data_flush_pending_count = 0;
	gc->data_flush_inflight = false;
	gc->commit_pending_count = 0;
	gc->commit_inflight = false;

	gc->poller = SPDK_POLLER_REGISTER(ppl_gc_poller, gc,
					  POWERAID_RAID_COMMON_PPL_GC_POLLER_US);
	if (gc->poller == NULL) {
		SPDK_ERRLOG("gc_init: poller register failed\n");
		return -ENOMEM;
	}

	SPDK_DEBUGLOG(raid5f_ppl, "gc_init: gc=%p ppl_ctx=%p raid_bdev=%p raid_ch=%p\n",
		      gc, ppl_ctx, raid_bdev, raid_ch);
	return 0;
}

void
poweraid_raid_common_ppl_gc_destroy(struct poweraid_raid_common_ppl_gc *gc)
{
	struct poweraid_raid_common_ppl_gc_entry *entry, *tmp;

	if (gc == NULL) {
		return;
	}

	if (gc->poller) {
		spdk_poller_unregister(&gc->poller);
	}

	/* fail 所有 pending entry */
	TAILQ_FOREACH_SAFE(entry, &gc->append_pending, link, tmp) {
		TAILQ_REMOVE(&gc->append_pending, entry, link);
		if (entry->append_buf) {
			spdk_free(entry->append_buf);
		}
		entry->append_cb(-ECANCELED, 0, entry->append_cb_arg);
		free(entry);
	}
	gc->append_pending_count = 0;

	TAILQ_FOREACH_SAFE(entry, &gc->data_flush_pending, link, tmp) {
		TAILQ_REMOVE(&gc->data_flush_pending, entry, link);
		entry->data_flush_cb(-ECANCELED, entry->data_flush_cb_arg);
		free(entry);
	}
	gc->data_flush_pending_count = 0;

	TAILQ_FOREACH_SAFE(entry, &gc->commit_pending, link, tmp) {
		TAILQ_REMOVE(&gc->commit_pending, entry, link);
		entry->commit_cb(-ECANCELED, entry->commit_cb_arg);
		free(entry);
	}
	gc->commit_pending_count = 0;

	SPDK_DEBUGLOG(raid5f_ppl, "gc_destroy: gc=%p\n", gc);
}
