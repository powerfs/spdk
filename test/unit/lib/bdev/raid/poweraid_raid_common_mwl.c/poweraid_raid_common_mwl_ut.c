/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   poweraid_raid_common_mwl 单元测试。
 *
 *   覆盖 TR-3.1（record 打包/CRC、seq 单调、slot 回卷 ≥2 轮回收）与
 *   TR-3.3（坏 record CRC / 坏 super CRC 安全处理）。
 *
 *   模式参照 bdev_raid_sb_ut.c：用 g_buf 模拟盘 MWL 区，mock spdk_bdev_*
 *   同步调用 cb 完成 IO；不依赖真实 bdev 设备。
 */

#include "spdk/stdinc.h"
#include "spdk_internal/cunit.h"
#include "spdk/env.h"
#include "spdk_internal/mock.h"

#include "common/lib/test_env.c"
#include "bdev/raid/poweraid_raid_common/poweraid_raid_common_mwl.c"

#define TEST_BUF_ALIGN	64

/* ===== stubs / mocks ===== */

DEFINE_STUB(spdk_bdev_queue_io_wait, int, (struct spdk_bdev *bdev, struct spdk_io_channel *ch,
		struct spdk_bdev_io_wait_entry *entry), 0);
DEFINE_STUB(spdk_bdev_get_name, const char *, (const struct spdk_bdev *bdev), "test_bdev");
DEFINE_STUB(spdk_bdev_get_buf_align, size_t, (const struct spdk_bdev *bdev), TEST_BUF_ALIGN);

/* ===== 测试全局 ===== */

static void *g_buf;              /* 模拟盘整片 MWL 区 */
static uint64_t g_buf_size;      /* g_buf 字节大小 */
static uint32_t g_block_size;
static uint64_t g_region_offset;
static uint64_t g_region_size;
static struct spdk_bdev g_bdev;
static struct spdk_bdev_desc *g_desc = (struct spdk_bdev_desc *)0xcafe;
static struct spdk_io_channel *g_ch = (struct spdk_io_channel *)0xbeef;

/* ===== mock 实现：直接对 g_buf 读写，同步调 cb ===== */

struct spdk_bdev *
spdk_bdev_desc_get_bdev(struct spdk_bdev_desc *desc)
{
	return &g_bdev;
}

void
spdk_bdev_free_io(struct spdk_bdev_io *bdev_io)
{
	free(bdev_io);
}

static struct spdk_bdev_io *
alloc_bdev_io(bool success)
{
	struct spdk_bdev_io *bdev_io;

	bdev_io = calloc(1, sizeof(*bdev_io));
	SPDK_CU_ASSERT_FATAL(bdev_io != NULL);
	bdev_io->bdev = &g_bdev;
	/* 用 internal.cb 字段不直接访问，借用 u.bdev 的方式：自建 spdk_bdev_io 太重，
	 * 这里用一个简化的 callback 触发机制：直接同步调用 cb_fn/cb_arg（伪造）。
	 * 但 mwl 的 io_cb 签名是 (bdev_io, success, cb_arg)，我们需要保存这两个。
	 * 简化：返回一个非空指针，由调用方在 _submit_*_blocks 里立即同步调用 cb。*/
	return bdev_io;
}

int
spdk_bdev_write_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		       void *buf, uint64_t offset_blocks, uint64_t num_blocks,
		       spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	uint64_t byte_off = offset_blocks * g_block_size;
	uint64_t nbytes = num_blocks * g_block_size;
	struct spdk_bdev_io *bdev_io;

	CU_ASSERT(byte_off + nbytes <= g_buf_size);
	memcpy((uint8_t *)g_buf + byte_off, buf, nbytes);

	bdev_io = alloc_bdev_io(true);
	cb(bdev_io, true, cb_arg);
	return 0;
}

int
spdk_bdev_flush_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		       uint64_t offset_blocks, uint64_t num_blocks,
		       spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev_io *bdev_io = alloc_bdev_io(true);
	cb(bdev_io, true, cb_arg);
	return 0;
}

int
spdk_bdev_read_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		      void *buf, uint64_t offset_blocks, uint64_t num_blocks,
		      spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	uint64_t byte_off = offset_blocks * g_block_size;
	uint64_t nbytes = num_blocks * g_block_size;
	struct spdk_bdev_io *bdev_io;

	CU_ASSERT(byte_off + nbytes <= g_buf_size);
	memcpy(buf, (uint8_t *)g_buf + byte_off, nbytes);

	bdev_io = alloc_bdev_io(true);
	cb(bdev_io, true, cb_arg);
	return 0;
}

/* ===== 同步等待异步回调完成（单线程内，cb 在 submit 内同步触发，无需 poll）===== */

/* ===== 测试 setup/cleanup ===== */

static int
test_setup_default(void)
{
	g_block_size = 512;
	g_region_offset = 4096;  /* 非 0（alloc 校验拒绝 0，与 PPL 一致）*/
	g_region_size = 4 * 1024 * 1024;  /* 4 MiB，max_slots=8192（含 super）*/
	g_buf_size = g_region_offset + g_region_size;

	g_bdev.blocklen = g_block_size;
	g_bdev.md_len = 0;

	g_buf = spdk_dma_zmalloc(g_buf_size, TEST_BUF_ALIGN, NULL);
	if (!g_buf) {
		return -ENOMEM;
	}

	return 0;
}

static int
test_setup_small(void)
{
	/* 小 region 用于回卷测试：4 slots = 1 super + 3 records */
	g_block_size = 512;
	g_region_offset = 4096;
	g_region_size = g_block_size * 4;  /* 4 slots */
	g_buf_size = g_region_offset + g_region_size;

	g_bdev.blocklen = g_block_size;
	g_bdev.md_len = 0;

	g_buf = spdk_dma_zmalloc(g_buf_size, TEST_BUF_ALIGN, NULL);
	if (!g_buf) {
		return -ENOMEM;
	}

	return 0;
}

static int
test_cleanup(void)
{
	spdk_dma_free(g_buf);
	g_buf = NULL;
	return 0;
}

/* ===== 辅助：从 g_buf 读 super/record ===== */

static const struct poweraid_raid_common_mwl_super *
mwl_super_at_buf(void)
{
	return (const struct poweraid_raid_common_mwl_super *)
	       ((uint8_t *)g_buf + g_region_offset);
}

static const struct poweraid_raid_common_mwl_record *
mwl_record_at_slot(uint32_t slot)
{
	return (const struct poweraid_raid_common_mwl_record *)
	       ((uint8_t *)g_buf + g_region_offset + (uint64_t)slot * g_block_size);
}

/* ===== 回调桥接 ===== */

static void
init_cb(int status, void *cb_arg)
{
	int *out = cb_arg;
	*out = status;
}

static void
append_cb(int status, uint64_t seq, void *cb_arg)
{
	struct { int status; uint64_t seq; } *out = cb_arg;
	out->status = status;
	out->seq = seq;
}

static void
commit_cb(int status, void *cb_arg)
{
	int *out = cb_arg;
	*out = status;
}

struct replay_result {
	int status;
	struct poweraid_raid_common_mwl_record *records;
	uint32_t num_records;
};

static void
replay_cb(int status, struct poweraid_raid_common_mwl_record *records, uint32_t num_records,
	  void *cb_arg)
{
	struct replay_result *out = cb_arg;
	out->status = status;
	out->records = records;
	out->num_records = num_records;
}

/* ===== 测试用例 ===== */

/* TR-3.1: record 打包 / CRC 校验通过与拒绝 */
static void
test_mwl_record_packing_crc(void)
{
	struct poweraid_raid_common_mwl_record rec;

	memset(&rec, 0, sizeof(rec));
	rec.magic = POWERAID_RAID_COMMON_MWL_REC_MAGIC;
	rec.flags = POWERAID_RAID_COMMON_MWL_REC_F_VALID;
	rec.seq = 42;
	rec.lba = 0x1000;
	rec.num_blocks = 8;
	rec.old_data_hash = 0xDEADBEEF;
	rec.new_data_hash = 0xCAFEBABE;
	rec.acting_slot = 0;
	rec.ts = 1700000000;
	mwl_rec_set_crc(&rec);

	/* CRC 通过 */
	CU_ASSERT(mwl_rec_check(&rec) == true);

	/* 篡改 seq 字段后 CRC 失败 */
	rec.seq = 999;
	CU_ASSERT(mwl_rec_check(&rec) == false);

	/* 恢复 seq，CRC 再次通过 */
	rec.seq = 42;
	CU_ASSERT(mwl_rec_check(&rec) == true);

	/* 篡改 magic 后 magic 校验先失败 */
	rec.magic = 0;
	CU_ASSERT(mwl_rec_check(&rec) == false);
}

/* TR-3.1: super 打包 / CRC 校验通过与拒绝 */
static void
test_mwl_super_packing_crc(void)
{
	struct poweraid_raid_common_mwl_super s;

	memset(&s, 0, sizeof(s));
	s.magic = POWERAID_RAID_COMMON_MWL_SUPER_MAGIC;
	s.version = 1;
	s.head_seq = 1;
	s.tail_seq = 10;
	s.commit_seq = 8;
	s.next_seq = 10;
	s.acting_lead_slot = 0;
	s.num_records = 1;
	mwl_super_set_crc(&s);

	CU_ASSERT(mwl_super_check(&s) == true);

	/* 篡改 commit_seq 后 CRC 失败 */
	s.commit_seq = 999;
	CU_ASSERT(mwl_super_check(&s) == false);

	/* 恢复 */
	s.commit_seq = 8;
	CU_ASSERT(mwl_super_check(&s) == true);

	/* 篡改 magic */
	s.magic = 0;
	CU_ASSERT(mwl_super_check(&s) == false);
}

/* TR-3.1: alloc + init 写出 valid super 到 g_buf */
static void
test_mwl_alloc_init(void)
{
	struct poweraid_raid_common_mwl_ctx *ctx;
	int rc;

	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	rc = -1;
	poweraid_raid_common_mwl_init(ctx, init_cb, &rc);
	CU_ASSERT(rc == 0);

	/* g_buf slot 0 应有 valid super */
	CU_ASSERT(mwl_super_at_buf()->magic == POWERAID_RAID_COMMON_MWL_SUPER_MAGIC);
	CU_ASSERT(mwl_super_check(mwl_super_at_buf()) == true);
	CU_ASSERT(mwl_super_at_buf()->head_seq == 1);
	CU_ASSERT(mwl_super_at_buf()->tail_seq == 1);
	CU_ASSERT(mwl_super_at_buf()->commit_seq == 0);
	CU_ASSERT(mwl_super_at_buf()->next_seq == 1);
	CU_ASSERT(mwl_super_at_buf()->acting_lead_slot == 0);

	poweraid_raid_common_mwl_free(ctx);

	/* 清 g_buf 供下一个用例 */
	memset(g_buf, 0, g_buf_size);
}

/* TR-3.1: append 多条，seq 单调递增，每条落盘后 magic+CRC 通过 */
static void
test_mwl_append_seq_monotonic(void)
{
	struct poweraid_raid_common_mwl_ctx *ctx;
	int rc;
	struct { int status; uint64_t seq; } ares;
	uint64_t prev_seq = 0;
	uint32_t i;

	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	rc = -1;
	poweraid_raid_common_mwl_init(ctx, init_cb, &rc);
	CU_ASSERT(rc == 0);

	for (i = 0; i < 5; i++) {
		ares.status = -1;
		ares.seq = 0;
		poweraid_raid_common_mwl_append_record(ctx, g_ch,
						       0x1000 + i * 8, 8,
						       0xAA00 + i, 0xBB00 + i,
						       0 /* acting_slot=0 */,
						       append_cb, &ares);
		CU_ASSERT(ares.status == 0);
		CU_ASSERT(ares.seq == prev_seq + 1);
		prev_seq = ares.seq;

		/* 验证落盘 record CRC 通过 */
		CU_ASSERT(mwl_record_at_slot(i + 1)->magic ==
			  POWERAID_RAID_COMMON_MWL_REC_MAGIC);
		CU_ASSERT(mwl_rec_check(mwl_record_at_slot(i + 1)) == true);
		CU_ASSERT(mwl_record_at_slot(i + 1)->seq == prev_seq);
		CU_ASSERT(mwl_record_at_slot(i + 1)->lba == 0x1000 + i * 8);
		/* acting_slot=0 不置 F_ACTING_LEAD */
		CU_ASSERT((mwl_record_at_slot(i + 1)->flags &
			   POWERAID_RAID_COMMON_MWL_REC_F_ACTING_LEAD) == 0);
	}

	/* ctx 内存 super 已推进 */
	CU_ASSERT(ctx->super.next_seq == 6);
	CU_ASSERT(ctx->super.tail_seq == 6);
	CU_ASSERT(ctx->super.num_records == 5);
	CU_ASSERT(ctx->super.commit_seq == 0);  /* 未 commit */

	poweraid_raid_common_mwl_free(ctx);
	memset(g_buf, 0, g_buf_size);
}

/* TR-3.1: acting_lead 标记：acting_slot=1 置 F_ACTING_LEAD */
static void
test_mwl_acting_lead_flag(void)
{
	struct poweraid_raid_common_mwl_ctx *ctx;
	int rc;
	struct { int status; uint64_t seq; } ares;

	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	rc = -1;
	poweraid_raid_common_mwl_init(ctx, init_cb, &rc);
	CU_ASSERT(rc == 0);

	/* acting_slot=1 → F_ACTING_LEAD 置位，super.acting_lead_slot 同步 */
	ares.status = -1;
	poweraid_raid_common_mwl_append_record(ctx, g_ch, 0x2000, 16,
					      0x1111, 0x2222,
					      1 /* acting_slot=1 */,
					      append_cb, &ares);
	CU_ASSERT(ares.status == 0);
	CU_ASSERT(ares.seq == 1);
	CU_ASSERT((mwl_record_at_slot(1)->flags &
		   POWERAID_RAID_COMMON_MWL_REC_F_ACTING_LEAD) != 0);
	CU_ASSERT(mwl_record_at_slot(1)->acting_slot == 1);
	CU_ASSERT(ctx->super.acting_lead_slot == 1);

	/* acting_slot=0 → F_ACTING_LEAD 清除，super.acting_lead_slot=0 */
	ares.status = -1;
	poweraid_raid_common_mwl_append_record(ctx, g_ch, 0x3000, 16,
					      0x3333, 0x4444,
					      0, append_cb, &ares);
	CU_ASSERT(ares.status == 0);
	CU_ASSERT((mwl_record_at_slot(2)->flags &
		   POWERAID_RAID_COMMON_MWL_REC_F_ACTING_LEAD) == 0);
	CU_ASSERT(ctx->super.acting_lead_slot == 0);

	poweraid_raid_common_mwl_free(ctx);
	memset(g_buf, 0, g_buf_size);
}

/* TR-3.1: append → commit 部分 → load_replay 返回未 commit 的按 seq 升序 */
static void
test_mwl_commit_replay(void)
{
	struct poweraid_raid_common_mwl_ctx *ctx;
	int rc;
	struct { int status; uint64_t seq; } ares;
	struct replay_result rres;
	uint32_t i;

	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	rc = -1;
	poweraid_raid_common_mwl_init(ctx, init_cb, &rc);
	CU_ASSERT(rc == 0);

	/* append 5 条 (seq 1..5) */
	for (i = 0; i < 5; i++) {
		ares.status = -1;
		poweraid_raid_common_mwl_append_record(ctx, g_ch, i * 16, 8,
						      0xA0 + i, 0xB0 + i, 0,
						      append_cb, &ares);
		CU_ASSERT(ares.status == 0);
	}

	/* commit 到 seq=3 */
	rc = -1;
	poweraid_raid_common_mwl_commit(ctx, g_ch, 3, commit_cb, &rc);
	CU_ASSERT(rc == 0);
	CU_ASSERT(ctx->super.commit_seq == 3);

	/* 掉盘：模拟重启 → 新 ctx，从 g_buf load_replay */
	poweraid_raid_common_mwl_free(ctx);
	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	memset(&rres, 0, sizeof(rres));
	poweraid_raid_common_mwl_load_replay(ctx, replay_cb, &rres);
	CU_ASSERT(rres.status == 0);
	CU_ASSERT(rres.num_records == 2);
	SPDK_CU_ASSERT_FATAL(rres.records != NULL);

	/* 按 seq 升序：4, 5 */
	CU_ASSERT(rres.records[0].seq == 4);
	CU_ASSERT(rres.records[1].seq == 5);
	CU_ASSERT(rres.records[0].lba == 3 * 16);
	CU_ASSERT(rres.records[1].lba == 4 * 16);

	/* super 已加载，commit_seq=3 */
	CU_ASSERT(ctx->super.commit_seq == 3);
	CU_ASSERT(ctx->super.next_seq == 6);

	poweraid_raid_common_mwl_free_replay_result(rres.records);
	poweraid_raid_common_mwl_free(ctx);
	memset(g_buf, 0, g_buf_size);
}

/* TR-3.1: slot 回卷 ≥2 轮后最早 commit 记录被回收且未提交记录保留。
 * 用小 region（4 slots: 0=super, 1..3=3 records）：
 *   轮1: append seq1,2,3 满 → commit 3 → 回卷条件就绪
 *   轮2: append seq4,5,6 满 → commit 6 → 回卷条件就绪
 *   轮3: append seq7,8,9 满 → 不 commit → load_replay 应返回 3 条（7,8,9）
 *        已 commit 的旧记录被覆盖，不返回 */
static void
test_mwl_ring_recycle_2rounds(void)
{
	struct poweraid_raid_common_mwl_ctx *ctx;
	int rc;
	struct { int status; uint64_t seq; } ares;
	struct replay_result rres;
	uint32_t i;
	uint64_t expected_seq = 1;

	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);
	CU_ASSERT(ctx->max_slots == 4);  /* 4 slots = 1 super + 3 records */

	rc = -1;
	poweraid_raid_common_mwl_init(ctx, init_cb, &rc);
	CU_ASSERT(rc == 0);

	/* 轮 1: append seq 1,2,3 满 */
	for (i = 0; i < 3; i++) {
		ares.status = -1;
		poweraid_raid_common_mwl_append_record(ctx, g_ch, i, 1, 0, 0, 0,
						       append_cb, &ares);
		CU_ASSERT(ares.status == 0);
		CU_ASSERT(ares.seq == expected_seq);
		expected_seq++;
	}
	/* 满：再 append 应 -ENOSPC（有 inflight） */
	ares.status = 0;
	poweraid_raid_common_mwl_append_record(ctx, g_ch, 99, 1, 0, 0, 0, append_cb, &ares);
	CU_ASSERT(ares.status == -ENOSPC);

	/* commit 3 后 inflight 清空 */
	rc = -1;
	poweraid_raid_common_mwl_commit(ctx, g_ch, 3, commit_cb, &rc);
	CU_ASSERT(rc == 0);

	/* 轮 2: append seq 4,5,6（触发回卷 tail_slot 4→1）*/
	for (i = 0; i < 3; i++) {
		ares.status = -1;
		poweraid_raid_common_mwl_append_record(ctx, g_ch, i, 1, 0, 0, 0,
						       append_cb, &ares);
		CU_ASSERT(ares.status == 0);
		CU_ASSERT(ares.seq == expected_seq);
		expected_seq++;
	}
	/* 回卷后 head_seq 应推进到 commit_seq+1 = 4 */
	CU_ASSERT(ctx->super.head_seq == 4);

	rc = -1;
	poweraid_raid_common_mwl_commit(ctx, g_ch, 6, commit_cb, &rc);
	CU_ASSERT(rc == 0);

	/* 轮 3: append seq 7,8,9 不 commit */
	for (i = 0; i < 3; i++) {
		ares.status = -1;
		poweraid_raid_common_mwl_append_record(ctx, g_ch, i, 1, 0, 0, 0,
						       append_cb, &ares);
		CU_ASSERT(ares.status == 0);
		CU_ASSERT(ares.seq == expected_seq);
		expected_seq++;
	}
	CU_ASSERT(ctx->super.head_seq == 7);

	/* 掉盘 → load_replay */
	poweraid_raid_common_mwl_free(ctx);
	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	memset(&rres, 0, sizeof(rres));
	poweraid_raid_common_mwl_load_replay(ctx, replay_cb, &rres);
	CU_ASSERT(rres.status == 0);
	CU_ASSERT(rres.num_records == 3);
	SPDK_CU_ASSERT_FATAL(rres.records != NULL);

	/* 仅返回未 commit 的 seq 7,8,9；已 commit 的旧记录即使盘上残留也被过滤 */
	CU_ASSERT(rres.records[0].seq == 7);
	CU_ASSERT(rres.records[1].seq == 8);
	CU_ASSERT(rres.records[2].seq == 9);
	CU_ASSERT(ctx->super.commit_seq == 6);

	poweraid_raid_common_mwl_free_replay_result(rres.records);
	poweraid_raid_common_mwl_free(ctx);
	memset(g_buf, 0, g_buf_size);
}

/* TR-3.3: 坏 record CRC 注入：load_replay 跳过坏 record，bad_record_cnt 可查。
 * 由于 bad_record_cnt 不在回调返回，通过结果数量验证：坏的不在结果里。 */
static void
test_mwl_bad_record_crc(void)
{
	struct poweraid_raid_common_mwl_ctx *ctx;
	int rc;
	struct { int status; uint64_t seq; } ares;
	struct replay_result rres;

	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	rc = -1;
	poweraid_raid_common_mwl_init(ctx, init_cb, &rc);
	CU_ASSERT(rc == 0);

	/* append 3 条 (seq 1,2,3) */
	ares.status = -1;
	poweraid_raid_common_mwl_append_record(ctx, g_ch, 0, 8, 0xA1, 0xB1, 0,
					      append_cb, &ares);
	CU_ASSERT(ares.status == 0);
	ares.status = -1;
	poweraid_raid_common_mwl_append_record(ctx, g_ch, 8, 8, 0xA2, 0xB2, 0,
					      append_cb, &ares);
	CU_ASSERT(ares.status == 0);
	ares.status = -1;
	poweraid_raid_common_mwl_append_record(ctx, g_ch, 16, 8, 0xA3, 0xB3, 0,
					      append_cb, &ares);
	CU_ASSERT(ares.status == 0);

	/* 篡改 slot 2 (seq=2) 的 lba 字段，破坏 CRC */
	((struct poweraid_raid_common_mwl_record *)mwl_record_at_slot(2))->lba = 0xDEAD;

	/* 掉盘 → load_replay */
	poweraid_raid_common_mwl_free(ctx);
	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	memset(&rres, 0, sizeof(rres));
	poweraid_raid_common_mwl_load_replay(ctx, replay_cb, &rres);
	CU_ASSERT(rres.status == 0);
	/* 坏的 seq=2 被跳过，仅返回 seq 1 和 3 */
	CU_ASSERT(rres.num_records == 2);
	SPDK_CU_ASSERT_FATAL(rres.records != NULL);
	CU_ASSERT(rres.records[0].seq == 1);
	CU_ASSERT(rres.records[1].seq == 3);

	poweraid_raid_common_mwl_free_replay_result(rres.records);
	poweraid_raid_common_mwl_free(ctx);
	memset(g_buf, 0, g_buf_size);
}

/* TR-3.3: 坏 super CRC：load_replay 视为 fresh disk，返回 clean（无记录） */
static void
test_mwl_bad_super_crc(void)
{
	struct poweraid_raid_common_mwl_ctx *ctx;
	int rc;
	struct { int status; uint64_t seq; } ares;
	struct replay_result rres;

	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	rc = -1;
	poweraid_raid_common_mwl_init(ctx, init_cb, &rc);
	CU_ASSERT(rc == 0);

	ares.status = -1;
	poweraid_raid_common_mwl_append_record(ctx, g_ch, 0, 8, 0xA1, 0xB1, 0,
					      append_cb, &ares);
	CU_ASSERT(ares.status == 0);

	/* 篡改 super 的 version 字段，破坏 CRC */
	((struct poweraid_raid_common_mwl_super *)((uint8_t *)g_buf + g_region_offset))->version = 99;

	/* 掉盘 → load_replay */
	poweraid_raid_common_mwl_free(ctx);
	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	memset(&rres, 0, sizeof(rres));
	poweraid_raid_common_mwl_load_replay(ctx, replay_cb, &rres);
	CU_ASSERT(rres.status == 0);
	CU_ASSERT(rres.records == NULL);
	CU_ASSERT(rres.num_records == 0);

	poweraid_raid_common_mwl_free(ctx);
	memset(g_buf, 0, g_buf_size);
}

/* TR-3.1: get/set acting_lead_slot 辅助 API */
static void
test_mwl_acting_lead_accessors(void)
{
	struct poweraid_raid_common_mwl_ctx *ctx;
	int rc;

	ctx = poweraid_raid_common_mwl_alloc(g_desc, g_ch, g_block_size,
					    g_region_offset, g_region_size);
	SPDK_CU_ASSERT_FATAL(ctx != NULL);

	rc = -1;
	poweraid_raid_common_mwl_init(ctx, init_cb, &rc);
	CU_ASSERT(rc == 0);

	CU_ASSERT(poweraid_raid_common_mwl_get_acting_lead_slot(ctx) == 0);
	CU_ASSERT(poweraid_raid_common_mwl_get_commit_seq(ctx) == 0);

	poweraid_raid_common_mwl_set_acting_lead_slot(ctx, 2);
	CU_ASSERT(poweraid_raid_common_mwl_get_acting_lead_slot(ctx) == 2);
	CU_ASSERT(ctx->super.acting_lead_slot == 2);

	poweraid_raid_common_mwl_free(ctx);
	memset(g_buf, 0, g_buf_size);
}

/* ===== 测试套件 ===== */

int
main(int argc, char **argv)
{
	CU_pSuite suite_default = NULL;
	CU_pSuite suite_small = NULL;
	unsigned int num_failures = 0;

	CU_initialize_registry();

	suite_default = CU_add_suite_with_setup_and_teardown("mwl_default",
			test_setup_default, test_cleanup, NULL, NULL);
	CU_ADD_TEST(suite_default, test_mwl_record_packing_crc);
	CU_ADD_TEST(suite_default, test_mwl_super_packing_crc);
	CU_ADD_TEST(suite_default, test_mwl_alloc_init);
	CU_ADD_TEST(suite_default, test_mwl_append_seq_monotonic);
	CU_ADD_TEST(suite_default, test_mwl_acting_lead_flag);
	CU_ADD_TEST(suite_default, test_mwl_commit_replay);
	CU_ADD_TEST(suite_default, test_mwl_bad_record_crc);
	CU_ADD_TEST(suite_default, test_mwl_bad_super_crc);
	CU_ADD_TEST(suite_default, test_mwl_acting_lead_accessors);

	suite_small = CU_add_suite_with_setup_and_teardown("mwl_small",
			test_setup_small, test_cleanup, NULL, NULL);
	CU_ADD_TEST(suite_small, test_mwl_ring_recycle_2rounds);

	num_failures = spdk_ut_run_tests(argc, argv, NULL);
	CU_cleanup_registry();

	return num_failures;
}
