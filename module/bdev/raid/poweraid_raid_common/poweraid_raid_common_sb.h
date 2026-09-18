/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   Superblock v2 扩展：向后兼容 v1（256B 固定），v2 总长 512B（v1 256B + 256B ext）
 *   详见 raid5f-enhanced-design.md 第 3.5 节
 */

#ifndef POWERAID_RAID_COMMON_SB_H
#define POWERAID_RAID_COMMON_SB_H

#include "spdk/stdinc.h"
#include "spdk/uuid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* v1 superblock 固定长度，参考 bdev_raid.h 的 raid_bdev_superblock */
#define POWERAID_RAID_COMMON_SB_V1_LENGTH    256
/* v2 扩展区长度，追加在 v1 末尾（base_bdevs[] 数组之外）*/
#define POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH  256
/* v2 总长度 = v1 + ext */
#define POWERAID_RAID_COMMON_SB_V2_LENGTH    (POWERAID_RAID_COMMON_SB_V1_LENGTH + \
					 POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH)

/* 版本号编码：v1.major=1, v1.minor=0；v2.major=2, v2.minor=0 */
#define POWERAID_RAID_COMMON_SB_VERSION_V1_MAJOR  1
#define POWERAID_RAID_COMMON_SB_VERSION_V2_MAJOR  2

/* PPL 区固定布局（每盘；字节单位）：
 *   LBA0 起 1 块为 sb；[1MiB, 5MiB) 为 PPL 区（4MiB，1024 个 4K slot）；
 *   数据区从 5MiB 起（4K 块下 = 1280 块，strip 64K/16 块对齐）。*/
#define POWERAID_RAID_COMMON_PPL_REGION_OFFSET  (1024ULL * 1024)
#define POWERAID_RAID_COMMON_PPL_REGION_SIZE    (4ULL * 1024 * 1024)

/* MWL（Mirror Write Log，raid1f 意图日志）区固定布局（每盘；字节单位）：
 *   与 PPL 同构：[1MiB, 5MiB) 4MiB；数据区从 5MiB 起（4K 块下 = 1280 块）。*/
#define POWERAID_RAID_COMMON_MWL_REGION_OFFSET  (1024ULL * 1024)
#define POWERAID_RAID_COMMON_MWL_REGION_SIZE    (4ULL * 1024 * 1024)

/* lead 副本的固定槽位（MWL 定序写与 replay 扇出的权威源）。
 * lead 缺失时由最低在场槽位担任 acting lead（标记在 MWL record/super）。 */
#define POWERAID_RAID1F_LEAD_SLOT              0

/* feature_flags：标识 v2 启用的特性 */
#define POWERAID_RAID_COMMON_SB_F_PPL         (1u << 0)
#define POWERAID_RAID_COMMON_SB_F_DIF         (1u << 1)
#define POWERAID_RAID_COMMON_SB_F_SCRUB       (1u << 2)
#define POWERAID_RAID_COMMON_SB_F_RAID6       (1u << 3)
#define POWERAID_RAID_COMMON_SB_F_SPARE_POOL  (1u << 4)
#define POWERAID_RAID_COMMON_SB_F_RESTRIPE    (1u << 5)
/* raid1f：MWL 意图日志（record 仅 hash 无载荷）*/
#define POWERAID_RAID_COMMON_SB_F_MWL         (1u << 6)
/* 预留升级位：MWL record 携带数据载荷的未来格式 */
#define POWERAID_RAID_COMMON_SB_F_MWL_PAYLOAD (1u << 7)

/* dif_mode：DIF/DIX 工作模式 */
enum poweraid_raid_common_dif_mode {
	POWERAID_RAID_COMMON_DIF_NONE = 0,
	POWERAID_RAID_COMMON_DIF_GENERATED,
	POWERAID_RAID_COMMON_DIF_PASSTHROUGH,
};

/**
 * v2 扩展区结构（追加在 v1 superblock 之后）。
 *
 * v1 superblock 末尾是 base_bdevs[]（变长）。为了向后兼容：
 * - v1 加载时：sb->length = 256 + N * 64（v1 不读取 ext）
 * - v2 加载时：sb->length = 512 + N * 64（ext 紧跟 v1 头部 256B 之后）
 *
 * 注意：v1 中 base_bdevs[] 紧跟 v1 末尾，v2 把 base_bdevs[] 移到 ext 之后。
 * 加载策略：
 *   1. 先读 v1 256B，判断 version.major
 *   2. v1：用 v1 路径，base_bdevs[] 紧跟 256B
 *   3. v2：再读 256B ext，base_bdevs[] 紧跟 512B 之后
 */
struct poweraid_raid_common_sb_v2_ext {
	/* v2 魔数，校验 ext 区完整性（避免误读 v1 数据）*/
#define POWERAID_RAID_COMMON_SB_V2_EXT_SIG "PWRAIDEX"
	uint8_t ext_signature[8];
	/* v2 扩展版本号（独立于 v1，用于 ext 内字段演进）*/
	uint16_t ext_major;
	uint16_t ext_minor;
	/* 扩展区 CRC32C（仅 ext 区，不含 v1）*/
	uint32_t ext_crc;
	/* 启用的特性 bitmap（POWERAID_RAID_COMMON_SB_F_*）*/
	uint32_t feature_flags;
	/* DIF/DIX 模式 */
	uint8_t dif_mode;
	/* RAID 扩展级别：5=RAID5, 6=RAID6, 7=RAID-NM */
	uint8_t raid_level_ext;
	/* synd_cnt（RAID6+ 的 syndrome 数，1=单 parity, 2=P+Q, 3=三校验）*/
	uint8_t synd_cnt;
	uint8_t reserved8[5];

	/* PPL（Partial Parity Log）区位置（每盘的相对偏移）*/
	uint64_t ppl_region_offset;
	/* PPL 区大小（字节，每盘）*/
	uint64_t ppl_region_size;
	/* PPL 当前 seq（启动恢复时用）*/
	uint64_t ppl_seq;

	/* MWL（Mirror Write Log，raid1f）区位置（每盘的相对偏移）*/
	uint64_t mwl_region_offset;
	/* MWL 区大小（字节，每盘）*/
	uint64_t mwl_region_size;
	/* MWL 当前已 commit seq（启动恢复时用）*/
	uint64_t mwl_seq;
	/* lead 槽位（永久 slot0；acting lead 标记在 MWL record/super，
	 * 正常情况下本字段恒为 POWERAID_RAID1F_LEAD_SLOT）*/
	uint64_t mwl_lead_slot;

	/* Scrub 进度（stripe_id）*/
	uint64_t scrub_progress;
	/* 上次完整 scrub 完成的时间戳（unix 秒）*/
	uint64_t scrub_last_complete_ts;

	/* 每盘 rebuild 进度（动态数组，长度 = num_base_bdevs）
	 * 注：v2 ext 不直接存动态数组，存的是一个指针偏移到 base_bdevs[] 之后；
	 * 实际生产实现可改为：扫描每盘 sb_base_bdev 中的 reserved[23] 字段存进度。
	 * 此处预留汇总字段：*/
	uint64_t recon_progress_summary;  /* 已重建总 stripe 数 */
	uint64_t recon_total_stripes;     /* 待重建总 stripe 数 */

	/* restripe 进度 */
	uint64_t restripe_progress;
	uint64_t restripe_target_stripes;

	/* 创建/最后修改时间戳（unix 秒）*/
	uint64_t create_ts;
	uint64_t last_modified_ts;

	/* 预留（ext 固定 256B；MWL 切走 32B 后剩 104）*/
	uint8_t reserved[104];
};
SPDK_STATIC_ASSERT(sizeof(struct poweraid_raid_common_sb_v2_ext) ==
		   POWERAID_RAID_COMMON_SB_V2_EXT_LENGTH,
		   "incorrect v2 ext size");

/* API */
struct poweraid_raid_common_raid;

/* sb_load 内部分配的加载上下文（opaque），调用方需用 sb_free_loaded 释放。
 * 与 raid->sb_ctx（由 sb_alloc 分配）独立——sb_load 是单盘探测，不污染 raid 状态。 */
struct poweraid_raid_common_sb_ctx;

/**
 * 分配 v2 superblock（v1 区 + ext 区 + base_bdevs 数组）。
 * 返回 0 成功，负数失败。
 */
int poweraid_raid_common_sb_alloc(struct poweraid_raid_common_raid *raid,
			     uint32_t block_size, uint8_t num_base_bdevs);

/**
 * 初始化 superblock 字段（创建新卷时）。
 */
void poweraid_raid_common_sb_init(struct poweraid_raid_common_raid *raid,
			     uint32_t level, uint32_t strip_size,
			     uint32_t feature_flags);

/**
 * 写入 superblock 到 raid 的所有 base_bdevs（异步，每盘并行写）。
 * 每盘用 base_bdevs[i]->desc + base_bdevs[i]->ch 调用 spdk_bdev_write_blocks。
 * 资源不足时用 spdk_bdev_queue_io_wait 排队。
 */
typedef void (*poweraid_raid_common_sb_write_cb)(int status, void *cb_arg);
void poweraid_raid_common_sb_write(struct poweraid_raid_common_raid *raid,
			     poweraid_raid_common_sb_write_cb cb, void *cb_arg);

/**
 * 加载单盘 superblock（异步，单盘探测，不写 raid->sb_ctx）。
 * 内部分配 sb_ctx，cb 返回指针；调用方负责用 sb_free_loaded 释放。
 * 自动判断 v1/v2，v1 兼容加载。
 *   - status==0 且 loaded_ctx!=NULL：成功加载（v1 时 ext 为 NULL）
 *   - status!=0：读失败或 crc 错（loaded_ctx 为 NULL）
 */
typedef void (*poweraid_raid_common_sb_load_cb)(int status,
		struct poweraid_raid_common_sb_ctx *loaded_ctx,
		void *cb_arg);
void poweraid_raid_common_sb_load(void *bdev_desc, struct spdk_io_channel *ch,
			    poweraid_raid_common_sb_load_cb cb, void *cb_arg);

/**
 * 释放 raid 的 sb_ctx（由 sb_alloc 分配，挂在 raid->sb_ctx）。
 */
void poweraid_raid_common_sb_free(struct poweraid_raid_common_raid *raid);

/**
 * 释放 sb_load 返回的独立 sb_ctx（不依赖 raid）。
 */
void poweraid_raid_common_sb_free_loaded(struct poweraid_raid_common_sb_ctx *ctx);

/**
 * 取 raid->sb_ctx v2 ext 中的 PPL 区布局（字节单位）。
 * 仅当 sb_alloc+sb_init（建卷）后可用；无 ext 时返回 -ENOENT。
 */
int poweraid_raid_common_sb_get_ppl_region(struct poweraid_raid_common_raid *raid,
				      uint64_t *region_offset_bytes,
				      uint64_t *region_size_bytes);

/**
 * 取 raid->sb_ctx v2 ext 中的 MWL 区布局（字节单位）。
 * 仅当 sb_alloc+sb_init（建卷）后可用；无 ext 时返回 -ENOENT。
 */
int poweraid_raid_common_sb_get_mwl_region(struct poweraid_raid_common_raid *raid,
				      uint64_t *region_offset_bytes,
				      uint64_t *region_size_bytes);

/**
 * 获取已准备好的 raw buffer（含 v1 + base_bdevs + ext），供上层
 * 调用 spdk_bdev_write_blocks 使用。out_size 输出 buffer 长度。
 * 返回 NULL 表示 raid 未分配 sb。
 */
const void *poweraid_raid_common_sb_get_write_buffer(struct poweraid_raid_common_raid *raid,
		uint32_t *out_size);

/**
 * 获取 v2 ext 指针，用于上层在加载完成后查询 PPL/scrub/feature_flags 等。
 * 返回 NULL 表示 raid 未分配 sb 或仅 v1 兼容加载（无 ext）。
 */
const struct poweraid_raid_common_sb_v2_ext *poweraid_raid_common_sb_get_ext(
	struct poweraid_raid_common_raid *raid);

/**
 * 查询 sb_load 返回的 loaded_ctx 的 v1 superblock 指针（含 signature/uuid/level/strip_size）。
 * 用于上层判断盘是否属于某 raid。
 */
const struct raid_bdev_superblock *poweraid_raid_common_sb_loaded_get_v1(
	struct poweraid_raid_common_sb_ctx *ctx);

/**
 * 查询 sb_load 返回的 loaded_ctx 的 v2 ext 指针。
 * 返回 NULL 表示该盘仅 v1 兼容加载（无 ext 区）。
 */
const struct poweraid_raid_common_sb_v2_ext *poweraid_raid_common_sb_loaded_get_ext(
	struct poweraid_raid_common_sb_ctx *ctx);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID_COMMON_SB_H */
