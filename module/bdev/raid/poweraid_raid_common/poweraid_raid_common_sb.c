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
#include "poweraid_raid_common.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_sb);

/* ===== 内部 sb 上下文（opaque 对外；同时供 raid->sb_ctx 与 sb_load loaded_ctx 复用）===== */
struct poweraid_raid_common_sb_ctx {
	/* raw buffer 起始 = v1 superblock 起始；包含 256B 头 + N*64 base_bdevs + 256B ext */
	struct raid_bdev_superblock	*v1;
	/* ext 区指针，紧跟 base_bdevs 之后；v1 兼容加载时为 NULL */
	struct poweraid_raid_common_sb_v2_ext *ext;
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
	poweraid_raid_common_sb_load_cb	load_cb;
	void				*load_cb_arg;
	/* 当前 buf 容量（多段读时动态扩展） */
	uint32_t			load_buf_size;
};

/* ===== sb_write 异步 IO 上下文 ===== */
struct poweraid_raid_common_sb_write_ctx {
	struct poweraid_raid_common_raid	*raid;
	/* 待写缓冲（v2 组合镜像 或 clear 用零缓冲），自 LBA0 起 buf_size 字节 */
	const void			*buf;
	uint32_t			buf_size;
	int				status;
	uint8_t				submitted;
	uint8_t				remaining;
	/* clear 路径：未就绪成员只跳过不扣减 remaining */
	uint8_t				skip_ready_only;
	/* wctx 自有的 dma 缓冲（clear 零缓冲），完成时释放；常规组合写为 NULL */
	void				*owned_buf;
	poweraid_raid_common_sb_write_cb	cb;
	void				*cb_arg;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

/* ===== 内部辅助 ===== */

static inline uint32_t
sb_v1_length(uint8_t num_base_bdevs)
{
	return POWERAID_RAID_COMMON_SB_V1_LENGTH +
	       num_base_bdevs * sizeof(struct raid_bdev_sb_base_bdev);
}

static inline uint32_t
sb_total_size(uint8_t num_base_bdevs)
{
	return sb_v1_length(num_base_bdevs) + POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH;
}

static inline struct poweraid_raid_common_sb_v2_ext *
sb_ext_at(void *raw, uint8_t num_base_bdevs)
{
	return (struct poweraid_raid_common_sb_v2_ext *)
		((uint8_t *)raw + sb_v1_length(num_base_bdevs));
}

/* 更新 v1 superblock 的 crc 字段（覆盖 raw 起始 sb_v1_length 字节，不含 ext） */
static void
sb_update_v1_crc(struct poweraid_raid_common_sb_ctx *ctx)
{
	ctx->v1->crc = 0;
	ctx->v1->crc = spdk_crc32c_update(ctx->raw, sb_v1_length(ctx->num_base_bdevs), 0);
}

/* 更新 ext 区 crc 字段（覆盖 ext 256B，先清零 ext_crc 再计算） */
static void
sb_update_ext_crc(struct poweraid_raid_common_sb_ctx *ctx)
{
	uint32_t crc;
	ctx->ext->ext_crc = 0;
	crc = spdk_crc32c_update(ctx->ext, POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH, 0);
	ctx->ext->ext_crc = crc;
}

/* 校验 ext crc */
static bool
sb_check_ext_crc(struct poweraid_raid_common_sb_ctx *ctx)
{
	uint32_t prev = ctx->ext->ext_crc;
	uint32_t crc;
	ctx->ext->ext_crc = 0;
	crc = spdk_crc32c_update(ctx->ext, POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH, 0);
	ctx->ext->ext_crc = prev;
	return crc == prev;
}

/* ===== 公共 API：alloc / init / free（raid 级别）===== */

int
poweraid_raid_common_sb_alloc(struct poweraid_raid_common_raid *raid,
			 uint32_t block_size, uint8_t num_base_bdevs)
{
	struct poweraid_raid_common_sb_ctx *ctx;
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

/* 初始化 ext 区（建卷 / 组合器首次组合共用）。 */
static void
sb_ext_init(struct poweraid_raid_common_sb_ctx *ctx, uint32_t level,
	    uint32_t strip_size, uint32_t feature_flags)
{
	(void)strip_size;

	/* === ext 区 === */
	memset(ctx->ext, 0, POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH);
	memcpy(ctx->ext->ext_signature, POWERAID_RAID_COMMON_SB_V2_EXT_SIG,
	       sizeof(ctx->ext->ext_signature));
	ctx->ext->ext_major = 1;
	ctx->ext->ext_minor = 0;
	ctx->ext->feature_flags = feature_flags;
	ctx->ext->dif_mode = POWERAID_RAID_COMMON_DIF_NONE;
	ctx->ext->raid_level_ext = (uint8_t)level;
	if (level == SPDK_BDEV_RAID_LEVEL_RAID6F) {
		ctx->ext->synd_cnt = 2;
	} else if (level == SPDK_BDEV_RAID_LEVEL_RAID1F) {
		/* 镜像卷无 parity syndrome */
		ctx->ext->synd_cnt = 0;
	} else {
		ctx->ext->synd_cnt = 1;
	}
	if (feature_flags & POWERAID_RAID_COMMON_SB_F_PPL) {
		ctx->ext->ppl_region_offset = POWERAID_RAID_COMMON_PPL_REGION_OFFSET;
		ctx->ext->ppl_region_size = POWERAID_RAID_COMMON_PPL_REGION_SIZE;
		ctx->ext->ppl_seq = 0;
	}
	if (feature_flags & POWERAID_RAID_COMMON_SB_F_MWL) {
		ctx->ext->mwl_region_offset = POWERAID_RAID_COMMON_MWL_REGION_OFFSET;
		ctx->ext->mwl_region_size = POWERAID_RAID_COMMON_MWL_REGION_SIZE;
		ctx->ext->mwl_seq = 0;
		ctx->ext->mwl_lead_slot = POWERAID_RAID1F_LEAD_SLOT;
	}
	ctx->ext->scrub_progress = 0;
	ctx->ext->scrub_last_complete_ts = 0;
	ctx->ext->recon_progress_summary = 0;
	ctx->ext->recon_total_stripes = 0;
	ctx->ext->restripe_progress = 0;
	ctx->ext->restripe_target_stripes = 0;
	ctx->ext->create_ts = (uint64_t)time(NULL);
	ctx->ext->last_modified_ts = ctx->ext->create_ts;
}

void
poweraid_raid_common_sb_init(struct poweraid_raid_common_raid *raid,
			uint32_t level, uint32_t strip_size,
			uint32_t feature_flags)
{
	struct poweraid_raid_common_sb_ctx *ctx;
	uint8_t i;

	if (raid == NULL || raid->sb_ctx == NULL) {
		return;
	}
	ctx = raid->sb_ctx;

	/* === v1 header === */
	memcpy(ctx->v1->signature, RAID_BDEV_SB_SIG, sizeof(ctx->v1->signature));
	ctx->v1->version.major = POWERAID_RAID_COMMON_SB_VERSION_V2_MAJOR;  /* v2 */
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

	sb_ext_init(ctx, level, strip_size, feature_flags);

	/* === CRC === */
	sb_update_v1_crc(ctx);
	sb_update_ext_crc(ctx);
}

/* ===== sb_write 异步 IO 实现（参考 bdev_raid_sb.c _raid_bdev_write_superblock）===== */

static void sb_write_one_done(int status, struct poweraid_raid_common_sb_write_ctx *wctx);
static void sb_write_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void sb_write_loop(void *_wctx);

static void
sb_write_one_done(int status, struct poweraid_raid_common_sb_write_ctx *wctx)
{
	if (status != 0) {
		wctx->status = status;
	}

	if (--wctx->remaining == 0) {
		wctx->cb(wctx->status, wctx->cb_arg);
		if (wctx->owned_buf != NULL) {
			spdk_dma_free(wctx->owned_buf);
		}
		free(wctx);
	}
}

static void
sb_write_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct poweraid_raid_common_sb_write_ctx *wctx = cb_arg;
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
	struct poweraid_raid_common_sb_write_ctx *wctx = _wctx;
	struct poweraid_raid_common_raid *raid = wctx->raid;
	const void *buf;
	uint32_t buf_size;
	uint8_t i;
	int rc;

	if (raid == NULL || raid->sb_ctx == NULL || raid->base_bdevs == NULL) {
		sb_write_one_done(-EINVAL, wctx);
		return;
	}
	buf = wctx->buf;
	buf_size = wctx->buf_size;

	for (i = wctx->submitted; i < raid->num_base_bdevs; i++) {
		struct poweraid_raid_common_bdev *bdev = raid->base_bdevs[i];
		struct spdk_bdev_desc *desc;
		struct spdk_io_channel *ch;
		struct spdk_bdev *bd;
		uint32_t blocklen, num_blocks;

		if (bdev == NULL || bdev->desc == NULL || bdev->ch == NULL) {
			/* 盘未就绪：clear 路径只跳过；常规写跳过等同完成。 */
			if (wctx->skip_ready_only) {
				wctx->submitted++;
				continue;
			}
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
poweraid_raid_common_sb_write(struct poweraid_raid_common_raid *raid,
		 poweraid_raid_common_sb_write_cb cb, void *cb_arg)
{
	struct poweraid_raid_common_sb_ctx *ctx;
	struct poweraid_raid_common_sb_write_ctx *wctx;

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
	wctx->buf = ctx->raw;
	wctx->buf_size = ctx->raw_size;
	wctx->remaining = raid->num_base_bdevs + 1;  /* +1 for final completion */
	wctx->cb = cb;
	wctx->cb_arg = cb_arg;

	sb_write_loop(wctx);
}

/* ===== sb_load 异步 IO 实现（参考 bdev_raid_sb.c raid_bdev_load_base_bdev_superblock）===== */

static void sb_load_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static int sb_load_parse_and_continue(struct poweraid_raid_common_sb_ctx *ctx);

static void
sb_load_ctx_fail(struct poweraid_raid_common_sb_ctx *ctx, int status)
{
	poweraid_raid_common_sb_load_cb cb = ctx->load_cb;
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
	struct poweraid_raid_common_sb_ctx *ctx = cb_arg;
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
sb_load_parse_and_continue(struct poweraid_raid_common_sb_ctx *ctx)
{
	struct raid_bdev_superblock *sb = ctx->v1;
	uint32_t need_size, first_v1_size, read_offset, read_len;
	void *new_buf;
	int rc;

	/* 1. 校验 signature。
	 * 无 SPDKRAID 签名 = 空白盘或异族盘（malloc/全新 NVMe 扇区为零），
	 * 返回 -ENODATA 供 FSM 走"新卷"分支；凡签名命中但后续校验失败的，
	 * 一律是本族 sb 损坏/版本不可用，返回 -EILSEQ 拒绝装配。*/
	if (memcmp(sb->signature, RAID_BDEV_SB_SIG, sizeof(sb->signature)) != 0) {
		SPDK_DEBUGLOG(raid5f_sb, "sb_load: signature mismatch (blank/foreign disk)\n");
		return -ENODATA;
	}

	/* 2. 校验 v1->length 合法性 */
	if (sb->length < sizeof(struct raid_bdev_superblock) ||
	    sb->length > RAID_BDEV_SB_MAX_LENGTH) {
		SPDK_WARNLOG("sb_load: invalid length %u\n", sb->length);
		return -EILSEQ;
	}

	/* 3. 计算总需要大小（v2 时 + ext 256B）*/
	if (sb->version.major == POWERAID_RAID_COMMON_SB_VERSION_V2_MAJOR) {
		need_size = sb->length + POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH;
		ctx->is_v2 = 1;
	} else if (sb->version.major == POWERAID_RAID_COMMON_SB_VERSION_V1_MAJOR) {
		need_size = sb->length;
		ctx->is_v2 = 0;
	} else {
		SPDK_ERRLOG("sb_load: unsupported version major %u\n", sb->version.major);
		return -EILSEQ;
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
			return -EILSEQ;
		}
	}

	/* 6. 设置 ctx 元数据 */
	ctx->num_base_bdevs = sb->num_base_bdevs;
	ctx->block_size = sb->block_size;
	ctx->raw_size = need_size;

	if (ctx->is_v2) {
		ctx->ext = (struct poweraid_raid_common_sb_v2_ext *)
			   ((uint8_t *)ctx->raw + sb->length);
		/* 7. 校验 ext_signature */
		if (memcmp(ctx->ext->ext_signature, POWERAID_RAID_COMMON_SB_V2_EXT_SIG,
			   sizeof(ctx->ext->ext_signature)) != 0) {
			SPDK_ERRLOG("sb_load: ext_signature mismatch\n");
			return -EILSEQ;
		}
		/* 8. 校验 ext crc */
		if (!sb_check_ext_crc(ctx)) {
			SPDK_WARNLOG("sb_load: ext crc mismatch\n");
			return -EILSEQ;
		}
	} else {
		ctx->ext = NULL;
	}

	return 1;  /* 完成 */
}

void
poweraid_raid_common_sb_load(void *bdev_desc, struct spdk_io_channel *ch,
			poweraid_raid_common_sb_load_cb cb, void *cb_arg)
{
	struct spdk_bdev_desc *desc = bdev_desc;
	struct spdk_bdev *bdev;
	struct poweraid_raid_common_sb_ctx *ctx;
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
					 POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH,
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
poweraid_raid_common_sb_free(struct poweraid_raid_common_raid *raid)
{
	struct poweraid_raid_common_sb_ctx *ctx;

	if (raid == NULL || raid->sb_ctx == NULL) {
		return;
	}
	ctx = raid->sb_ctx;
	spdk_dma_free(ctx->raw);
	free(ctx);
	raid->sb_ctx = NULL;
}

int
poweraid_raid_common_sb_get_ppl_region(struct poweraid_raid_common_raid *raid,
				   uint64_t *region_offset_bytes,
				   uint64_t *region_size_bytes)
{
	struct poweraid_raid_common_sb_ctx *ctx;

	if (raid == NULL || raid->sb_ctx == NULL) {
		return -ENOENT;
	}
	ctx = raid->sb_ctx;
	if (ctx->ext == NULL ||
	    !(ctx->ext->feature_flags & POWERAID_RAID_COMMON_SB_F_PPL)) {
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

int
poweraid_raid_common_sb_get_mwl_region(struct poweraid_raid_common_raid *raid,
				   uint64_t *region_offset_bytes,
				   uint64_t *region_size_bytes)
{
	struct poweraid_raid_common_sb_ctx *ctx;

	if (raid == NULL || raid->sb_ctx == NULL) {
		return -ENOENT;
	}
	ctx = raid->sb_ctx;
	if (ctx->ext == NULL ||
	    !(ctx->ext->feature_flags & POWERAID_RAID_COMMON_SB_F_MWL)) {
		return -ENOENT;
	}
	if (region_offset_bytes) {
		*region_offset_bytes = ctx->ext->mwl_region_offset;
	}
	if (region_size_bytes) {
		*region_size_bytes = ctx->ext->mwl_region_size;
	}
	return 0;
}

void
poweraid_raid_common_sb_free_loaded(struct poweraid_raid_common_sb_ctx *ctx)
{
	if (ctx == NULL) {
		return;
	}
	spdk_dma_free(ctx->raw);
	free(ctx);
}

const void *
poweraid_raid_common_sb_get_write_buffer(struct poweraid_raid_common_raid *raid,
				     uint32_t *out_size)
{
	struct poweraid_raid_common_sb_ctx *ctx;
	if (raid == NULL || raid->sb_ctx == NULL) {
		return NULL;
	}
	ctx = raid->sb_ctx;
	if (out_size) {
		*out_size = ctx->raw_size;
	}
	return ctx->raw;
}

const struct poweraid_raid_common_sb_v2_ext *
poweraid_raid_common_sb_get_ext(struct poweraid_raid_common_raid *raid)
{
	struct poweraid_raid_common_sb_ctx *ctx;
	if (raid == NULL || raid->sb_ctx == NULL) {
		return NULL;
	}
	ctx = raid->sb_ctx;
	return ctx->is_v2 ? ctx->ext : NULL;
}

const struct raid_bdev_superblock *
poweraid_raid_common_sb_loaded_get_v1(struct poweraid_raid_common_sb_ctx *ctx)
{
	return ctx ? ctx->v1 : NULL;
}

const struct poweraid_raid_common_sb_v2_ext *
poweraid_raid_common_sb_loaded_get_ext(struct poweraid_raid_common_sb_ctx *ctx)
{
	return (ctx && ctx->is_v2) ? ctx->ext : NULL;
}

/* ===== 方向 B：框架 sb 生命周期组合器（issue #11）=====
 *
 * 所有权模型：raid_bdev->sb（框架 dma 缓冲）唯一持有公共段
 * [256B 公共头][N*64B 成员表]，poweraid 仅持有一个 4096B 组合镜像
 * [公共段][256B ext]。每次框架委托写盘前从 raid_bdev->sb 重新组合，
 * 打 major=2、追加 ext、重算两段 CRC；poweraid 不再独立决定写盘时机。
 */

/* 校验 ext 区 CRC。约定与 sb_update_ext_crc 一致：ext_crc 字段视为零后
 * 对完整 256B 连算（与公共段 raid_bdev_sb_update_crc 的口径相同）。
 * 不能跳过 crc 字段分两段拼接——CRC 流中"4 字节零"与"不喂这 4 字节"不等价。 */
static bool
sb_priv_ext_crc_valid(const struct poweraid_raid_common_sb_v2_ext *ext)
{
	static const uint8_t zero_field[sizeof(ext->ext_crc)] = { 0 };
	uint32_t crc;

	crc = spdk_crc32c_update(ext, offsetof(struct poweraid_raid_common_sb_v2_ext,
					       ext_crc), 0);
	crc = spdk_crc32c_update(zero_field, sizeof(ext->ext_crc), crc);
	crc = spdk_crc32c_update((const uint8_t *)ext +
				 offsetof(struct poweraid_raid_common_sb_v2_ext, ext_crc) +
				 sizeof(ext->ext_crc),
				 POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH -
				 offsetof(struct poweraid_raid_common_sb_v2_ext, ext_crc) -
				 sizeof(ext->ext_crc), crc);
	return crc == ext->ext_crc;
}

int
poweraid_raid_common_sb_hook_validate(const void *buf, uint32_t buf_size)
{
	const struct raid_bdev_superblock *sb = buf;
	const struct poweraid_raid_common_sb_v2_ext *ext;
	uint32_t total;

	if (buf == NULL) {
		return -EINVAL;
	}

	if (sb->version.major != POWERAID_RAID_COMMON_SB_VERSION_V2_MAJOR) {
		return -ENODATA;
	}

	/* 本族镜像：公共长度必须与成员数自洽。签名/公共 CRC 已由框架校验。 */
	if (sb->length != sb_v1_length(sb->num_base_bdevs)) {
		SPDK_WARNLOG("poweraid sb: inconsistent common length %u for %u base bdevs\n",
			     sb->length, sb->num_base_bdevs);
		return -EILSEQ;
	}

	total = sb->length + POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH;
	if (buf_size < total) {
		/* ext 尾尚未读全，认领后由框架 -EAGAIN 续读，再次进入本函数。 */
		return 0;
	}

	ext = (const struct poweraid_raid_common_sb_v2_ext *)
	      ((const uint8_t *)buf + sb->length);
	if (memcmp(ext->ext_signature, POWERAID_RAID_COMMON_SB_V2_EXT_SIG,
		   sizeof(ext->ext_signature)) != 0) {
		SPDK_ERRLOG("poweraid sb: ext signature mismatch\n");
		return -EILSEQ;
	}
	if (!sb_priv_ext_crc_valid(ext)) {
		SPDK_WARNLOG("poweraid sb: ext crc mismatch\n");
		return -EILSEQ;
	}

	return 0;
}

uint32_t
poweraid_raid_common_sb_hook_total_size(const struct raid_bdev_superblock *sb)
{
	return sb->length + POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH;
}

/* 每次委托写盘前：用框架最新 raid_bdev->sb 重组镜像公共段。
 * 新卷首次调用时惰性建立镜像（框架 alloc/init 公共 sb 发生在 start() 之后）。 */
static int
sb_priv_compose(struct poweraid_raid_common_raid *raid)
{
	struct raid_bdev *rb = raid->raid_bdev;
	struct raid_bdev_superblock *fsb = rb->sb;
	struct poweraid_raid_common_sb_ctx *ctx;
	uint32_t feature_flags;
	int rc;

	if (fsb == NULL) {
		SPDK_ERRLOG("poweraid sb compose: framework sb missing on raid %s\n",
			    raid->name);
		return -EINVAL;
	}

	if (raid->sb_ctx == NULL) {
		rc = poweraid_raid_common_sb_alloc(raid, rb->bdev.blocklen,
						   rb->num_base_bdevs);
		if (rc != 0) {
			return rc;
		}
		ctx = raid->sb_ctx;
		feature_flags = raid->level == SPDK_BDEV_RAID_LEVEL_RAID1F ?
				POWERAID_RAID_COMMON_SB_F_MWL :
				POWERAID_RAID_COMMON_SB_F_PPL;
		sb_ext_init(ctx, raid->level, raid->strip_size, feature_flags);
	} else {
		ctx = raid->sb_ctx;
	}

	/* 公共段以框架为唯一权威（uuid/成员表/state/data_offset/seq 均在此）。 */
	memcpy(ctx->raw, fsb, fsb->length);
	ctx->num_base_bdevs = fsb->num_base_bdevs;
	ctx->v1->version.major = POWERAID_RAID_COMMON_SB_VERSION_V2_MAJOR;
	ctx->v1->length = fsb->length;
	sb_update_v1_crc(ctx);
	sb_update_ext_crc(ctx);

	return 0;
}

/* C7：data_offset 单轨。
 * 新卷在 shell start() 早期调用：把框架预填的成员 data_offset/data_size
 * 统一改为 PPL/MWL 区口径（[1MiB,5MiB)，4K 盘下 1280 块），随后框架
 * init 公共头会把该值写入成员表，poweraid 不再二次扣除。
 * 重装配卷（rb->sb != NULL）不动：成员表以盘上回放值为权威。
 * superblock=false 的新卷框架不设 1MiB 预留（data_offset=0、data_size=全盘，
 * bdev_raid_configure_base_bdev_cont 仅在 superblock_enabled 时预留），若照旧
 * 跳过，卷数据区将从 LBA0 起与 sb/PPL/MWL 区重叠（RMW 的 PPL 记录写会覆写
 * 用户数据，verify 读到 PPL 签名；1f 还会因 offset=0 被 start 几何校验拒绝），
 * 故 offset=0 的已配置成员同样纳入统一。 */
void
poweraid_raid_common_sb_unify_data_offset(struct raid_bdev *rb)
{
	struct raid_base_bdev_info *base_info;
	uint64_t reserve_blocks;

	if (rb == NULL || rb->sb != NULL) {
		return;
	}

	reserve_blocks = (POWERAID_RAID_COMMON_PPL_REGION_OFFSET +
			  POWERAID_RAID_COMMON_PPL_REGION_SIZE) /
			 spdk_bdev_get_data_block_size(&rb->bdev);

	RAID_FOR_EACH_BASE_BDEV(rb, base_info) {
		uint64_t end_blocks;

		if (base_info->desc == NULL) {
			continue;
		}
		end_blocks = base_info->data_offset + base_info->data_size;
		if (end_blocks <= reserve_blocks) {
			continue;
		}
		base_info->data_size = end_blocks - reserve_blocks;
		base_info->data_offset = reserve_blocks;
	}
}

int
poweraid_raid_common_sb_priv_adopt(struct poweraid_raid_common_raid *raid)
{
	struct raid_bdev *rb;
	struct poweraid_raid_common_sb_ctx *ctx;
	uint32_t total;
	int rc;

	if (raid == NULL || raid->raid_bdev == NULL || raid->raid_bdev->sb == NULL) {
		return -EINVAL;
	}
	rb = raid->raid_bdev;

	rc = poweraid_raid_common_sb_alloc(raid, rb->bdev.blocklen,
					   rb->num_base_bdevs);
	if (rc != 0) {
		return rc;
	}
	ctx = raid->sb_ctx;
	total = rb->sb->length + POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH;
	assert(total <= ctx->raw_size);
	memcpy(ctx->raw, rb->sb, total);
	ctx->num_base_bdevs = rb->sb->num_base_bdevs;

	return 0;
}

/* superblock=false 的新卷：无框架公共段可 adopt，按新卷语义惰性建立私有
 * ctx（仅 ext region 布局供 PPL/MWL fresh init 查询；不落盘）。 */
int
poweraid_raid_common_sb_priv_ensure_fresh(struct poweraid_raid_common_raid *raid)
{
	struct poweraid_raid_common_sb_ctx *ctx;
	uint32_t feature_flags;
	int rc;

	if (raid == NULL || raid->raid_bdev == NULL) {
		return -EINVAL;
	}
	if (raid->sb_ctx != NULL) {
		return 0;
	}

	rc = poweraid_raid_common_sb_alloc(raid, raid->raid_bdev->bdev.blocklen,
					   raid->raid_bdev->num_base_bdevs);
	if (rc != 0) {
		return rc;
	}
	ctx = raid->sb_ctx;
	feature_flags = raid->level == SPDK_BDEV_RAID_LEVEL_RAID1F ?
			POWERAID_RAID_COMMON_SB_F_MWL :
			POWERAID_RAID_COMMON_SB_F_PPL;
	sb_ext_init(ctx, raid->level, raid->strip_size, feature_flags);

	return 0;
}

/* 适配框架 raid_bdev_write_sb_cb ↔ poweraid sb_write_cb。 */
struct sb_priv_trampoline {
	raid_bdev_write_sb_cb		cb;
	void				*cb_ctx;
	struct raid_bdev		*rb;
};

static void
sb_priv_trampoline(int status, void *cb_arg)
{
	struct sb_priv_trampoline *t = cb_arg;
	struct poweraid_raid_common_raid *raid = t->rb->module_private;

	/* 派发 FSM 注册的一次性持久化回调（如新卷 ONLINE 等待的落盘事件）。 */
	if (raid != NULL && raid->sb_persist_done_cb != NULL) {
		poweraid_raid_common_sb_write_cb fsm_cb = raid->sb_persist_done_cb;
		void *fsm_arg = raid->sb_persist_done_arg;

		raid->sb_persist_done_cb = NULL;
		raid->sb_persist_done_arg = NULL;
		fsm_cb(status, fsm_arg);
	}

	t->cb(status, t->rb, t->cb_ctx);
	free(t);
}

void
poweraid_raid_common_sb_hook_write(struct raid_bdev *rb, raid_bdev_write_sb_cb cb,
				  void *cb_ctx)
{
	struct poweraid_raid_common_raid *raid;
	struct sb_priv_trampoline *t;
	int rc;

	raid = rb->module_private;
	rc = sb_priv_compose(raid);
	if (rc != 0) {
		cb(rc, rb, cb_ctx);
		return;
	}

	t = calloc(1, sizeof(*t));
	if (t == NULL) {
		cb(-ENOMEM, rb, cb_ctx);
		return;
	}
	t->cb = cb;
	t->cb_ctx = cb_ctx;
	t->rb = rb;

	poweraid_raid_common_sb_write(raid, sb_priv_trampoline, t);
}

void
poweraid_raid_common_sb_hook_clear(struct raid_bdev *rb, raid_bdev_write_sb_cb cb,
				  void *cb_ctx)
{
	struct poweraid_raid_common_raid *raid;
	struct poweraid_raid_common_sb_write_ctx *wctx;
	struct poweraid_raid_common_bdev *bdev;
	void *zero_buf;
	uint8_t i, ready = 0;

	if (cb == NULL) {
		return;
	}

	raid = rb->module_private;
	/* 早失败回滚（configure 从未执行、本族未落过盘）：无盘可擦，直接成功，
	 * 绝不擦除属于其他阵列的外来 sb（如异 uuid create -EEXIST 回滚）。 */
	if (raid == NULL || raid->sb_ctx == NULL) {
		cb(0, rb, cb_ctx);
		return;
	}

	for (i = 0; i < raid->num_base_bdevs; i++) {
		bdev = raid->base_bdevs[i];
		if (bdev != NULL && bdev->desc != NULL && bdev->ch != NULL) {
			ready++;
		}
	}
	if (ready == 0) {
		cb(0, rb, cb_ctx);
		return;
	}

	zero_buf = spdk_dma_zmalloc(4096, 0x1000, NULL);
	if (zero_buf == NULL) {
		cb(-ENOMEM, rb, cb_ctx);
		return;
	}

	wctx = calloc(1, sizeof(*wctx));
	if (wctx == NULL) {
		spdk_dma_free(zero_buf);
		cb(-ENOMEM, rb, cb_ctx);
		return;
	}
	wctx->raid = raid;
	wctx->buf = zero_buf;
	wctx->buf_size = 4096;
	wctx->remaining = ready + 1;
	wctx->cb = sb_priv_trampoline;
	wctx->owned_buf = zero_buf;
	/* clear 只提交就绪成员；未就绪成员不扣减 remaining。 */
	wctx->skip_ready_only = 1;
	{
		struct sb_priv_trampoline *t = calloc(1, sizeof(*t));
		if (t == NULL) {
			spdk_dma_free(zero_buf);
			free(wctx);
			cb(-ENOMEM, rb, cb_ctx);
			return;
		}
		t->cb = cb;
		t->cb_ctx = cb_ctx;
		t->rb = rb;
		wctx->cb_arg = t;
	}

	sb_write_loop(wctx);
}
