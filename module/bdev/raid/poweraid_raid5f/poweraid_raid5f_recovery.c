/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   启动恢复实现。详见 poweraid_raid5f_recovery.h 与 raid5f-enhanced-design.md L125-130。
 *
 *   异步状态机：
 *     ppl_load_replay → 逐 record → 逐 chunk（按 bitmap 升序）read_fn 读 →
 *       ppl_hash_update → 全 chunk 读完后 hash_final → 三分支判定 → 存 result → 下一 record
 *     全部完成 → 清 RAID_ST_RESTORING → cb 返回 result 数组
 */

#include "spdk/stdinc.h"
#include "spdk/log.h"

#include "poweraid_raid5f_recovery.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_recovery);

struct poweraid_raid5f_recovery_op {
	struct poweraid_raid5f_ppl_ctx		*ppl_ctx;
	struct poweraid_raid5f_raid		*raid;
	poweraid_raid5f_recovery_read_data_fn	read_fn;
	poweraid_raid5f_recovery_done_cb	cb;
	void					*cb_arg;

	/* PPL 扫描结果（ppl 拥有，完成后 ppl_free_replay_result）*/
	struct poweraid_raid5f_ppl_record	*records;
	uint32_t				num_records;

	/* 恢复结果数组（调用方用 recovery_free_result 释放）*/
	struct poweraid_raid5f_recovery_result	*results;

	/* 迭代游标 */
	uint32_t				cur_record;  /* 当前处理的 record 下标 */
	uint32_t				cur_bit;     /* 当前处理的 chunk bit（0..63）*/

	/* 当前 record 的增量 hash */
	struct poweraid_raid5f_ppl_hash_ctx	hash;
};

static void recovery_process_record(struct poweraid_raid5f_recovery_op *op);
static void recovery_read_next_chunk(struct poweraid_raid5f_recovery_op *op);
static void recovery_read_data_cb(int status, const void *buf, size_t len, void *cb_arg);
static void recovery_ppl_replay_cb(int status,
		struct poweraid_raid5f_ppl_record *records, uint32_t num_records,
		void *cb_arg);

static void
recovery_finish(struct poweraid_raid5f_recovery_op *op, int status)
{
	/* RESTORING 的清理由集成层（sm_raid online 流程）在 recovery 回调及其
	 * 后续 parity fixup 全部结束后统一清除，避免数据面在 fixup 期间提前开门。*/
	if (status != 0 || op->num_records == 0) {
		op->cb(status, NULL, 0, op->cb_arg);
		if (op->records) {
			poweraid_raid5f_ppl_free_replay_result(op->records);
		}
		free(op->results);
		free(op);
		return;
	}
	op->cb(0, op->results, op->num_records, op->cb_arg);
	poweraid_raid5f_ppl_free_replay_result(op->records);
	/* results 由调用方持有（cb 已收到指针），不在此释放；调用方用 recovery_free_result */
	free(op);
}

static void
recovery_process_record(struct poweraid_raid5f_recovery_op *op)
{
	struct poweraid_raid5f_ppl_record *rec;

	if (op->cur_record >= op->num_records) {
		/* 全部 record 处理完 */
		recovery_finish(op, 0);
		return;
	}

	rec = &op->records[op->cur_record];
	op->cur_bit = 0;
	poweraid_raid5f_ppl_hash_init(&op->hash);

	/* 若 read_fn 为 NULL（集成层未就绪），安全降级：标 NONE 并跳过读盘 */
	if (op->read_fn == NULL) {
		struct poweraid_raid5f_recovery_result *r = &op->results[op->cur_record];
		r->seq = rec->seq;
		r->stripe_id = rec->stripe_id;
		r->chunk_bitmap = rec->chunk_bitmap;
		r->cur_data_hash = 0;
		r->action = POWERAID_RECOVERY_ACT_NONE;
		SPDK_NOTICELOG("recovery: rec seq=%"PRIu64" stripe=%"PRIu64" → NONE (no read_fn)\n",
			       rec->seq, rec->stripe_id);
		op->cur_record++;
		recovery_process_record(op);
		return;
	}

	recovery_read_next_chunk(op);
}

static void
recovery_read_next_chunk(struct poweraid_raid5f_recovery_op *op)
{
	struct poweraid_raid5f_ppl_record *rec = &op->records[op->cur_record];
	uint64_t bit_mask;

	/* 跳过未置位的 bit */
	while (op->cur_bit < 64) {
		bit_mask = 1ULL << op->cur_bit;
		if (rec->chunk_bitmap & bit_mask) {
			break;
		}
		op->cur_bit++;
	}

	if (op->cur_bit >= 64) {
		/* 当前 record 全部 chunk 已读 → 判定 */
		uint64_t cur_hash = poweraid_raid5f_ppl_hash_final(&op->hash);
		struct poweraid_raid5f_recovery_result *r = &op->results[op->cur_record];
		r->seq = rec->seq;
		r->stripe_id = rec->stripe_id;
		r->chunk_bitmap = rec->chunk_bitmap;
		r->cur_data_hash = cur_hash;

		if (cur_hash == rec->new_data_hash) {
			r->action = POWERAID_RECOVERY_ACT_REWRITE_PARITY;
			SPDK_NOTICELOG("recovery: rec seq=%"PRIu64" stripe=%"PRIu64" → REWRITE_PARITY (data==new)\n",
				       rec->seq, rec->stripe_id);
		} else if (cur_hash == rec->old_data_hash) {
			r->action = POWERAID_RECOVERY_ACT_NONE;
			SPDK_NOTICELOG("recovery: rec seq=%"PRIu64" stripe=%"PRIu64" → NONE (data==old)\n",
				       rec->seq, rec->stripe_id);
		} else {
			r->action = POWERAID_RECOVERY_ACT_INCONSISTENT;
			SPDK_WARNLOG("recovery: rec seq=%"PRIu64" stripe=%"PRIu64" → INCONSISTENT "
				     "(cur=%016"PRIx64" old=%016"PRIx64" new=%016"PRIx64")\n",
				     rec->seq, rec->stripe_id,
				     cur_hash, rec->old_data_hash, rec->new_data_hash);
		}
		op->cur_record++;
		recovery_process_record(op);
		return;
	}

	/* 读当前 bit 对应的 chunk */
	op->read_fn(op->raid, rec->stripe_id, op->cur_bit,
		    op->raid->strip_size, recovery_read_data_cb, op);
}

static void
recovery_read_data_cb(int status, const void *buf, size_t len, void *cb_arg)
{
	struct poweraid_raid5f_recovery_op *op = cb_arg;
	struct poweraid_raid5f_ppl_record *rec = &op->records[op->cur_record];

	if (status != 0 || buf == NULL || len == 0) {
		/* 读失败 → 标记 inconsistent，跳到下一条 */
		struct poweraid_raid5f_recovery_result *r = &op->results[op->cur_record];
		r->seq = rec->seq;
		r->stripe_id = rec->stripe_id;
		r->chunk_bitmap = rec->chunk_bitmap;
		r->cur_data_hash = 0;
		r->action = POWERAID_RECOVERY_ACT_INCONSISTENT;
		SPDK_WARNLOG("recovery: rec seq=%"PRIu64" read chunk %u failed (%d) → INCONSISTENT\n",
			     rec->seq, op->cur_bit, status);
		op->cur_record++;
		recovery_process_record(op);
		return;
	}

	poweraid_raid5f_ppl_hash_update(&op->hash, buf, len);
	op->cur_bit++;
	recovery_read_next_chunk(op);
}

static void
recovery_ppl_replay_cb(int status,
		struct poweraid_raid5f_ppl_record *records, uint32_t num_records,
		void *cb_arg)
{
	struct poweraid_raid5f_recovery_op *op = cb_arg;

	if (status != 0) {
		SPDK_ERRLOG("recovery: ppl_load_replay failed (%d)\n", status);
		recovery_finish(op, status);
		return;
	}
	if (num_records == 0) {
		SPDK_NOTICELOG("recovery: no uncommitted records → clean\n");
		recovery_finish(op, 0);
		return;
	}

	op->records = records;
	op->num_records = num_records;
	op->results = calloc(num_records, sizeof(*op->results));
	if (!op->results) {
		poweraid_raid5f_ppl_free_replay_result(records);
		recovery_finish(op, -ENOMEM);
		return;
	}
	op->cur_record = 0;
	recovery_process_record(op);
}

void
poweraid_raid5f_recovery_run(struct poweraid_raid5f_ppl_ctx *ppl_ctx,
			     struct poweraid_raid5f_raid *raid,
			     poweraid_raid5f_recovery_read_data_fn read_fn,
			     poweraid_raid5f_recovery_done_cb cb, void *cb_arg)
{
	struct poweraid_raid5f_recovery_op *op;

	if (ppl_ctx == NULL || raid == NULL || cb == NULL) {
		if (cb) { cb(-EINVAL, NULL, 0, cb_arg); }
		return;
	}

	op = calloc(1, sizeof(*op));
	if (!op) {
		cb(-ENOMEM, NULL, 0, cb_arg);
		return;
	}
	op->ppl_ctx = ppl_ctx;
	op->raid = raid;
	op->read_fn = read_fn;
	op->cb = cb;
	op->cb_arg = cb_arg;

	poweraid_raid_state_set(&raid->state, POWERAID_RAID_ST_RESTORING);
	SPDK_NOTICELOG("recovery: start (RESTORING set), ppl_ctx=%p\n", ppl_ctx);

	poweraid_raid5f_ppl_load_replay(ppl_ctx, recovery_ppl_replay_cb, op);
}

void
poweraid_raid5f_recovery_free_result(struct poweraid_raid5f_recovery_result *results)
{
	free(results);
}
