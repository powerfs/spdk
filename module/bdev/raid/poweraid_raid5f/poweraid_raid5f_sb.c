/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   Superblock v2 实现：
 *   - 内存布局：[256B v1 header][N*64 base_bdevs][256B ext]
 *   - v1 sb->length 仍按 v1 语义 = 256 + N*64（不含 ext），保证 v1 兼容加载
 *   - v2 加载流程：读 256B 头 → 看 version.major → 读 base_bdevs → 如 v2 再读 256B ext
 *   - CRC：v1 crc 覆盖 [0..256+N*64)，ext_crc 覆盖 ext 区 256B（含 ext_crc 字段时先清零再算）
 *
 *   阶段 2：sb_load/sb_write 接入 spdk_bdev_read/spdk_bdev_write_blocks 实际 IO；
 *           sb_load 单盘探测，内部分配 sb_ctx，回调返回，调用方负责 sb_free_loaded。
 *           sb_write 每盘并行写，资源不足用 spdk_bdev_queue_io_wait 排队。
 *
 *   详见 raid5f-enhanced-design.md 第 3.5 节
 */

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/crc32.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/string.h"
#include "spdk/util.h"
#include "spdk/uuid.h"

#include "../bdev_raid.h"
#include "poweraid_raid5f.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_sb);

/* ===== 内部 sb 上下文（opaque 对外；同时供 raid->sb_ctx 与 sb_load loaded_ctx 复用）===== */
struct poweraid_raid5f_sb_ctx {
	/* raw buffer 起始 = v1 superblock 起始；包含 256B 头 + N*64 base_bdevs + 256B ext */
	struct raid_bdev_superblock	*v1;
	/* ext 区指针，紧跟 base_bdevs 之后；v1 兼容加载时为 NULL */
	struct poweraid_raid5f_sb_v2_ext *ext;
	/* spdk_dma_malloc 原始指针（== v1） */
	void				*raw;
	/* raw buffer 总长度：256 + N*64 + 256（v2）或 256 + N*64（v1） */
	uint32_t			raw_size;
	/* 数据块大小（用于 IO 时块对齐） */
	uint32_t			block_size;
	/* base_bdevs 数量 */
	uint8_t				num_base_bdevs;
	/* 是否为 v2（含 ext 区）*/
	uint8_t				is_v2;
	/* ===== sb_load 异步 IO 上下文（仅 sb_load 路径使用）===== */
	struct spdk_bdev_desc		*load_desc;
	struct spdk_io_channel		*load_ch;
	poweraid_raid5f_sb_load_cb	load_cb;
	void				*load_cb_arg;
	/* 当前 buf 容量（多段读时动态扩展） */
	uint32_t			load_buf_size;
};

/* ===== sb_write 异步 IO 上下文 ===== */
struct poweraid_raid5f_sb_write_ctx {
	struct poweraid_raid5f_raid	*raid;
	int				status;
	uint8_t				submitted;
	uint8_t				remaining;
	poweraid_raid5f_sb_write_cb	cb;
	void				*cb_arg;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== 内部辅助 ===== */

static inline uint32_t
sb_v1_length(uint8_t num_base_bdevs)
{
	return POWERAID_RAID5F_SB_V1_LENGTH +
	       num_base_bdevs * sizeof(struct raid_bdev_sb_base_bdev);
}

static inline uint32_t
sb_total_size(uint8_t num_base_bdevs)
{
	return sb_v1_length(num_base_bdevs) + POWERAID_RAID5F_SB_V2_EXT_LENGTH;
}

static inline struct poweraid_raid5f_sb_v2_ext *
sb_ext_at(void *raw, uint8_t num_base_bdevs)
{
	return (struct poweraid_raid5f_sb_v2_ext *)
		((uint8_t *)raw + sb_v1_length(num_base_bdevs));
}

/* 更新 v1 superblock 的 crc 字段（覆盖 raw 起始 sb_v1_length 字节，不含 ext） */
static void
sb_update_v1_crc(struct poweraid_raid5f_sb_ctx *ctx)
{
	ctx->v1->crc = 0;
	ctx->v1->crc = spdk_crc32c_update(ctx->raw, sb_v1_length(ctx->num_base_bdevs), 0);
}

/* 更新 ext 区 crc 字段（覆盖 ext 256B，先清零 ext_crc 再计算） */
static void
sb_update_ext_crc(struct poweraid_raid5f_sb_ctx *ctx)
{
	uint32_t crc;
	ctx->ext->ext_crc = 0;
	crc = spdk_crc32c_update(ctx->ext, POWERAID_RAID5F_SB_V2_EXT_LENGTH, 0);
	ctx->ext->ext_crc = crc;
}

/* 校验 ext crc */
static bool
sb_check_ext_crc(struct poweraid_raid5f_sb_ctx *ctx)
{
	uint32_t prev = ctx->ext->ext_crc;
	uint32_t crc;
	ctx->ext->ext_crc = 0;
	crc = spdk_crc32c_update(ctx->ext, POWERAID_RAID5F_SB_V2_EXT_LENGTH, 0);
	ctx->ext->ext_crc = prev;
	return crc == prev;
}

/* ===== 公共 API：alloc / init / free（raid 级别）===== */

int
poweraid_raid5f_sb_alloc(struct poweraid_raid5f_raid *raid,
			 uint32_t block_size, uint8_t num_base_bdevs)
{
	struct poweraid_raid5f_sb_ctx *ctx;
	uint32_t total;

	if (raid == NULL || block_size == 0 || num_base_bdevs == 0) {
		return -EINVAL;
	}
	if (raid->sb_ctx != NULL) {
		SPDK_ERRLOG("raid %p sb_ctx already allocated\n", raid);
		return -EEXIST;
	}

	total = sb_total_size(num_base_bdevs);
	/* 对齐到 4KiB 边界（NVMe 4K IO 友好） */
	total = SPDK_ALIGN_CEIL(total, 4096);

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		return -ENOMEM;
	}
	ctx->raw = spdk_dma_zmalloc(total, 0x1000, NULL);
	if (!ctx->raw) {
		SPDK_ERRLOG("alloc sb buffer failed (size=%u)\n", total);
		free(ctx);
		return -ENOMEM;
	}
	ctx->raw_size = total;
	ctx->block_size = block_size;
	ctx->num_base_bdevs = num_base_bdevs;
	ctx->is_v2 = 1;  /* 新分配的总是 v2 */
	ctx->v1 = (struct raid_bdev_superblock *)ctx->raw;
	ctx->ext = sb_ext_at(ctx->raw, num_base_bdevs);

	raid->sb_ctx = ctx;
	return 0;
}

void
poweraid_raid5f_sb_init(struct poweraid_raid5f_raid *raid,
			uint32_t level, uint32_t strip_size,
			uint32_t feature_flags)
{
	struct poweraid_raid5f_sb_ctx *ctx;
	uint8_t i;

	if (raid == NULL || raid->sb_ctx == NULL) {
		return;
	}
	ctx = raid->sb_ctx;

	/* === v1 header === */
	memcpy(ctx->v1->signature, RAID_BDEV_SB_SIG, sizeof(ctx->v1->signature));
	ctx->v1->version.major = POWERAID_RAID5F_SB_VERSION_V2_MAJOR;  /* v2 */
	ctx->v1->version.minor = 0;
	spdk_uuid_copy(&ctx->v1->uuid, &raid->uuid);
	snprintf((char *)ctx->v1->name, RAID_BDEV_SB_NAME_SIZE, "%s", raid->name);
	ctx->v1->raid_size = raid->raid_size;
	ctx->v1->block_size = ctx->block_size;
	ctx->v1->level = level;
	ctx->v1->strip_size = strip_size;
	ctx->v1->state = 0;  /* TODO 阶段 2：从 raid->state 同步 */
	ctx->v1->seq_number = 1;
	ctx->v1->num_base_bdevs = ctx->num_base_bdevs;
	ctx->v1->base_bdevs_size = ctx->num_base_bdevs;
	ctx->v1->length = sb_v1_length(ctx->num_base_bdevs);
	for (i = 0; i < ctx->num_base_bdevs; i++) {
		struct raid_bdev_sb_base_bdev *sb_b = &ctx->v1->base_bdevs[i];
		sb_b->slot = i;
		sb_b->state = RAID_SB_BASE_BDEV_CONFIGURED;
		/* uuid/data_offset/data_size 由上层阶段 2 填 */
	}

	/* === ext 区 === */
	memset(ctx->ext, 0, POWERAID_RAID5F_SB_V2_EXT_LENGTH);
	memcpy(ctx->ext->ext_signature, POWERAID_RAID5F_SB_V2_EXT_SIG,
	       sizeof(ctx->ext->ext_signature));
	ctx->ext->ext_major = 1;
	ctx->ext->ext_minor = 0;
	ctx->ext->feature_flags = feature_flags;
	ctx->ext->dif_mode = POWERAID_RAID5F_DIF_NONE;
	ctx->ext->raid_level_ext = (uint8_t)level;
	ctx->ext->synd_cnt = (level == 6) ? 2 : 1;
	ctx->ext->ppl_region_offset = POWERAID_RAID5F_PPL_REGION_OFFSET;
	ctx->ext->ppl_region_size = POWERAID_RAID5F_PPL_REGION_SIZE;
	ctx->ext->ppl_seq = 0;
	ctx->ext->scrub_progress = 0;
	ctx->ext->scrub_last_complete_ts = 0;
	ctx->ext->recon_progress_summary = 0;
	ctx->ext->recon_total_stripes = 0;
	ctx->ext->restripe_progress = 0;
	ctx->ext->restripe_target_stripes = 0;
	ctx->ext->create_ts = (uint64_t)time(NULL);
	ctx->ext->last_modified_ts = ctx->ext->create_ts;

	/* === CRC === */
	sb_update_v1_crc(ctx);
	sb_update_ext_crc(ctx);
}

/* ===== sb_write 异步 IO 实现（参考 bdev_raid_sb.c _raid_bdev_write_superblock）===== */

static void sb_write_one_done(int status, struct poweraid_raid5f_sb_write_ctx *wctx);
static void sb_write_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void sb_write_loop(void *_wctx);

static void
sb_write_one_done(int status, struct poweraid_raid5f_sb_write_ctx *wctx)
{
	if (status != 0) {
		wctx->status = status;
	}

	if (--wctx->remaining == 0) {
		wctx->cb(wctx->status, wctx->cb_arg);
		free(wctx);
	}
}

static void
sb_write_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct poweraid_raid5f_sb_write_ctx *wctx = cb_arg;
	int status = 0;

	if (!success) {
		SPDK_ERRLOG("poweraid_raid5f: sb_write IO failed on bdev %s\n",
			    bdev_io->bdev ? bdev_io->bdev->name : "(unknown)");
		status = -EIO;
	}

	spdk_bdev_free_io(bdev_io);

	sb_write_one_done(status, wctx);
}

static void
sb_write_loop(void *_wctx)
{
	struct poweraid_raid5f_sb_write_ctx *wctx = _wctx;
	struct poweraid_raid5f_raid *raid = wctx->raid;
	struct poweraid_raid5f_sb_ctx *ctx;
	const void *buf;
	uint32_t buf_size;
	uint8_t i;
	int rc;

	if (raid == NULL || raid->sb_ctx == NULL || raid->base_bdevs == NULL) {
		sb_write_one_done(-EINVAL, wctx);
		return;
	}
	ctx = raid->sb_ctx;
	buf = ctx->raw;
	buf_size = ctx->raw_size;

	for (i = wctx->submitted; i < raid->num_base_bdevs; i++) {
		struct poweraid_raid5f_bdev *bdev = raid->base_bdevs[i];
		struct spdk_bdev_desc *desc;
		struct spdk_io_channel *ch;
		struct spdk_bdev *bd;
		uint32_t blocklen, num_blocks;

		if (bdev == NULL || bdev->desc == NULL || bdev->ch == NULL) {
			/* 盘未就绪：跳过，等同完成 */
			assert(wctx->remaining > 1);
			sb_write_one_done(0, wctx);
			wctx->submitted++;
			continue;
		}

		desc = (struct spdk_bdev_desc *)bdev->desc;
		ch = (struct spdk_io_channel *)bdev->ch;
		bd = spdk_bdev_desc_get_bdev(desc);
		blocklen = bd->blocklen;
		num_blocks = buf_size / blocklen;

		rc = spdk_bdev_write_blocks(desc, ch, (void *)buf, 0, num_blocks,
					    sb_write_io_cb, wctx);
		if (rc != 0) {
			if (rc == -ENOMEM) {
				/* 资源不足：排队等待，下次再继续 */
				wctx->wait_entry.bdev = bd;
				wctx->wait_entry.cb_fn = sb_write_loop;
				wctx->wait_entry.cb_arg = wctx;
				spdk_bdev_queue_io_wait(bd, ch, &wctx->wait_entry);
				return;
			}
			SPDK_ERRLOG("poweraid_raid5f: sb_write_blocks rc=%d on bdev %u\n",
				    rc, i);
			assert(wctx->remaining > 1);
			sb_write_one_done(rc, wctx);
		}

		wctx->submitted++;
	}

	/* 全部提交完毕，触发最终完成（remaining 多出的 1 个）*/
	sb_write_one_done(0, wctx);
}

void
poweraid_raid5f_sb_write(struct poweraid_raid5f_raid *raid,
			 poweraid_raid5f_sb_write_cb cb, void *cb_arg)
{
	struct poweraid_raid5f_sb_ctx *ctx;
	struct poweraid_raid5f_sb_write_ctx *wctx;

	if (raid == NULL || raid->sb_ctx == NULL) {
		if (cb) cb(-EINVAL, cb_arg);
		return;
	}
	ctx = raid->sb_ctx;

	wctx = calloc(1, sizeof(*wctx));
	if (!wctx) {
		if (cb) cb(-ENOMEM, cb_arg);
		return;
	}
	wctx->raid = raid;
	wctx->remaining = raid->num_base_bdevs + 1;  /* +1 for final completion */
	wctx->cb = cb;
	wctx->cb_arg = cb_arg;

	/* 更新 seq + CRC，准备 buffer */
	ctx->v1->seq_number++;
	sb_update_v1_crc(ctx);
	if (ctx->is_v2) {
		sb_update_ext_crc(ctx);
	}

	sb_write_loop(wctx);
}

/* ===== sb_load 异步 IO 实现（参考 bdev_raid_sb.c raid_bdev_load_base_bdev_superblock）===== */

static void sb_load_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static int sb_load_parse_and_continue(struct poweraid_raid5f_sb_ctx *ctx);

static void
sb_load_ctx_fail(struct poweraid_raid5f_sb_ctx *ctx, int status)
{
	poweraid_raid5f_sb_load_cb cb = ctx->load_cb;
	void *arg = ctx->load_cb_arg;

	/* 失败：内部释放，回调传 NULL */
	spdk_dma_free(ctx->raw);
	free(ctx);
	if (cb) {
		cb(status, NULL, arg);
	}
}

static void
sb_load_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct poweraid_raid5f_sb_ctx *ctx = cb_arg;
	int rc;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		SPDK_DEBUGLOG(raid5f_sb, "sb_load: read failed\n");
		sb_load_ctx_fail(ctx, -EIO);
		return;
	}

	rc = sb_load_parse_and_continue(ctx);
	if (rc == 0) {
		/* 还需读后续段，已重新提交 IO，等待回调 */
		return;
	}
	if (rc == 1) {
		/* 完整加载成功，调用方接管 ctx */
		if (ctx->load_cb) {
			ctx->load_cb(0, ctx, ctx->load_cb_arg);
		}
		return;
	}
	/* rc < 0：失败 */
	sb_load_ctx_fail(ctx, rc);
}

/* 解析当前 buf 内容；返回 0=需继续读后续段（已提交），1=完成（成功），<0=失败 */
static int
sb_load_parse_and_continue(struct poweraid_raid5f_sb_ctx *ctx)
{
	struct raid_bdev_superblock *sb = ctx->v1;
	uint32_t need_size, first_v1_size, read_offset, read_len;
	void *new_buf;
	int rc;

	/* 1. 校验 signature */
	if (memcmp(sb->signature, RAID_BDEV_SB_SIG, sizeof(sb->signature)) != 0) {
		SPDK_DEBUGLOG(raid5f_sb, "sb_load: signature mismatch (not a RAID disk)\n");
		return -EINVAL;
	}

	/* 2. 校验 v1->length 合法性 */
	if (sb->length < sizeof(struct raid_bdev_superblock) ||
	    sb->length > RAID_BDEV_SB_MAX_LENGTH) {
		SPDK_WARNLOG("sb_load: invalid length %u\n", sb->length);
		return -EINVAL;
	}

	/* 3. 计算总需要大小（v2 时 + ext 256B）*/
	if (sb->version.major == POWERAID_RAID5F_SB_VERSION_V2_MAJOR) {
		need_size = sb->length + POWERAID_RAID5F_SB_V2_EXT_LENGTH;
		ctx->is_v2 = 1;
	} else if (sb->version.major == POWERAID_RAID5F_SB_VERSION_V1_MAJOR) {
		need_size = sb->length;
		ctx->is_v2 = 0;
	} else {
		SPDK_ERRLOG("sb_load: unsupported version major %u\n", sb->version.major);
		return -EINVAL;
	}

	/* 4. 当前 buf 不够 → 续读 */
	if (ctx->load_buf_size < need_size) {
		first_v1_size = ctx->load_buf_size;
		new_buf = spdk_dma_realloc(ctx->raw, need_size,
					   spdk_bdev_get_buf_align(spdk_bdev_desc_get_bdev(ctx->load_desc)),
					   NULL);
		if (!new_buf) {
			return -ENOMEM;
		}
		ctx->raw = new_buf;
		ctx->v1 = (struct raid_bdev_superblock *)ctx->raw;
		ctx->load_buf_size = need_size;

		read_offset = first_v1_size;
		read_len = need_size - first_v1_size;

		rc = spdk_bdev_read(ctx->load_desc, ctx->load_ch,
				    (uint8_t *)ctx->raw + read_offset,
				    read_offset, read_len,
				    sb_load_read_cb, ctx);
		if (rc != 0) {
			SPDK_ERRLOG("sb_load: spdk_bdev_read remainder rc=%d\n", rc);
			return rc;
		}
		return 0;  /* 等回调 */
	}

	/* 5. buf 已完整 → 校验 v1 crc（覆盖整个 v1 区 = sb->length）*/
	{
		uint32_t prev = sb->crc;
		uint32_t crc;
		sb->crc = 0;
		crc = spdk_crc32c_update(ctx->raw, sb->length, 0);
		sb->crc = prev;
		if (crc != prev) {
			SPDK_WARNLOG("sb_load: v1 crc mismatch\n");
			return -EINVAL;
		}
	}

	/* 6. 设置 ctx 元数据 */
	ctx->num_base_bdevs = sb->num_base_bdevs;
	ctx->block_size = sb->block_size;
	ctx->raw_size = need_size;

	if (ctx->is_v2) {
		ctx->ext = (struct poweraid_raid5f_sb_v2_ext *)
			   ((uint8_t *)ctx->raw + sb->length);
		/* 7. 校验 ext_signature */
		if (memcmp(ctx->ext->ext_signature, POWERAID_RAID5F_SB_V2_EXT_SIG,
			   sizeof(ctx->ext->ext_signature)) != 0) {
			SPDK_ERRLOG("sb_load: ext_signature mismatch\n");
			return -EINVAL;
		}
		/* 8. 校验 ext crc */
		if (!sb_check_ext_crc(ctx)) {
			SPDK_WARNLOG("sb_load: ext crc mismatch\n");
			return -EINVAL;
		}
	} else {
		ctx->ext = NULL;
	}

	return 1;  /* 完成 */
}

void
poweraid_raid5f_sb_load(void *bdev_desc, struct spdk_io_channel *ch,
			poweraid_raid5f_sb_load_cb cb, void *cb_arg)
{
	struct spdk_bdev_desc *desc = bdev_desc;
	struct spdk_bdev *bdev;
	struct poweraid_raid5f_sb_ctx *ctx;
	uint32_t first_size;
	int rc;

	if (desc == NULL || cb == NULL) {
		if (cb) cb(-EINVAL, NULL, cb_arg);
		return;
	}

	bdev = spdk_bdev_desc_get_bdev(desc);

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		cb(-ENOMEM, NULL, cb_arg);
		return;
	}

	/* 第一次读：覆盖整个 v1 区（最大 RAID_BDEV_SB_MAX_LENGTH）+ v2 ext(256B)，
	 * 按 blocklen 对齐。不能分段续读非块对齐的字节范围（spdk_bdev_read 要求
	 * offset/nbytes 均块对齐，否则 -EINVAL，会导致既有卷被误判为新卷）。*/
	first_size = spdk_divide_round_up(RAID_BDEV_SB_MAX_LENGTH +
					 POWERAID_RAID5F_SB_V2_EXT_LENGTH,
					 spdk_bdev_get_data_block_size(bdev)) * bdev->blocklen;
	ctx->raw = spdk_dma_malloc(first_size, spdk_bdev_get_buf_align(bdev), NULL);
	if (!ctx->raw) {
		free(ctx);
		cb(-ENOMEM, NULL, cb_arg);
		return;
	}
	ctx->load_buf_size = first_size;
	ctx->v1 = (struct raid_bdev_superblock *)ctx->raw;
	ctx->load_desc = desc;
	ctx->load_ch = ch;
	ctx->load_cb = cb;
	ctx->load_cb_arg = cb_arg;

	rc = spdk_bdev_read(desc, ch, ctx->raw, 0, first_size,
			   sb_load_read_cb, ctx);
	if (rc != 0) {
		SPDK_ERRLOG("sb_load: spdk_bdev_read rc=%d\n", rc);
		spdk_dma_free(ctx->raw);
		free(ctx);
		cb(rc, NULL, cb_arg);
	}
}

/* ===== 释放与查询 ===== */

void
poweraid_raid5f_sb_free(struct poweraid_raid5f_raid *raid)
{
	struct poweraid_raid5f_sb_ctx *ctx;

	if (raid == NULL || raid->sb_ctx == NULL) {
		return;
	}
	ctx = raid->sb_ctx;
	spdk_dma_free(ctx->raw);
	free(ctx);
	raid->sb_ctx = NULL;
}

int
poweraid_raid5f_sb_get_ppl_region(struct poweraid_raid5f_raid *raid,
				   uint64_t *region_offset_bytes,
				   uint64_t *region_size_bytes)
{
	struct poweraid_raid5f_sb_ctx *ctx;

	if (raid == NULL || raid->sb_ctx == NULL) {
		return -ENOENT;
	}
	ctx = raid->sb_ctx;
	if (ctx->ext == NULL) {
		return -ENOENT;
	}
	if (region_offset_bytes) {
		*region_offset_bytes = ctx->ext->ppl_region_offset;
	}
	if (region_size_bytes) {
		*region_size_bytes = ctx->ext->ppl_region_size;
	}
	return 0;
}

void
poweraid_raid5f_sb_free_loaded(struct poweraid_raid5f_sb_ctx *ctx)
{
	if (ctx == NULL) {
		return;
	}
	spdk_dma_free(ctx->raw);
	free(ctx);
}

const void *
poweraid_raid5f_sb_get_write_buffer(struct poweraid_raid5f_raid *raid,
				     uint32_t *out_size)
{
	struct poweraid_raid5f_sb_ctx *ctx;
	if (raid == NULL || raid->sb_ctx == NULL) {
		return NULL;
	}
	ctx = raid->sb_ctx;
	if (out_size) {
		*out_size = ctx->raw_size;
	}
	return ctx->raw;
}

const struct poweraid_raid5f_sb_v2_ext *
poweraid_raid5f_sb_get_ext(struct poweraid_raid5f_raid *raid)
{
	struct poweraid_raid5f_sb_ctx *ctx;
	if (raid == NULL || raid->sb_ctx == NULL) {
		return NULL;
	}
	ctx = raid->sb_ctx;
	return ctx->is_v2 ? ctx->ext : NULL;
}

const struct raid_bdev_superblock *
poweraid_raid5f_sb_loaded_get_v1(struct poweraid_raid5f_sb_ctx *ctx)
{
	return ctx ? ctx->v1 : NULL;
}

const struct poweraid_raid5f_sb_v2_ext *
poweraid_raid5f_sb_loaded_get_ext(struct poweraid_raid5f_sb_ctx *ctx)
{
	return (ctx && ctx->is_v2) ? ctx->ext : NULL;
}
