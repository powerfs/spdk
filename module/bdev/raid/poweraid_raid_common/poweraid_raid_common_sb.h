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

#include "../bdev_raid.h"

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
 * v2 扩展区结构（追加在框架公共 superblock 成员表之后）。
 *
 * v2 盘上布局（写盘 4096B）：
 *   [256B 框架公共头][N × 64B 成员表 base_bdevs[]][256B 本 ext 区]
 * - 公共 sb->length = 256 + N * 64（不含 ext）
 * - ext 偏移 = 256 + N * 64（3 盘 = 448）
 * - 公共 crc 覆盖 sb->length；ext_crc 仅覆盖 ext 256B（ext_crc 字段
 *   本身先清零再对完整 256B 连算，不可跳过该字段分段拼接）
 *
 * 加载策略（框架钩子）：
 *   1. 先读公共头 + 成员表（首次按 RAID_BDEV_SB_MAX_LENGTH 读取）
 *   2. major != 2 由 sb_validate_disk 返回 -ENODATA（非本族，可 fresh）
 *   3. major == 2 且 buf 不足 total（sb->length + 256）时返回 0 触发续读
 *   4. ext 签名/CRC 校验失败返回 -EILSEQ（本族损坏，禁止覆写）
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

/* ===== 方向 B：框架 sb 生命周期组合器（issue #11）===== */

/**
 * 重装配分支：module->start() 发现框架已从盘上回放 raid_bdev->sb
 * （含保留的 v2 ext 尾）时调用，建立 poweraid 组合镜像，供 PPL/MWL 等
 * ext 几何查询与后续委托写使用。返回 0 成功，负数失败。
 */
int poweraid_raid_common_sb_priv_adopt(struct poweraid_raid_common_raid *raid);

/**
 * superblock=false 的新卷：框架不持久化 sb（configure 直接 cont、不走委托写，
 * rb->sb 恒空），私有 ctx 无入口建立。按新卷语义惰性建 ctx 并填 ext 默认
 * region 布局，仅供 PPL/MWL fresh init 查询几何；不落盘，重启不重装配。
 * 已有 ctx 时为空操作。返回 0 成功，负数失败。
 */
int poweraid_raid_common_sb_priv_ensure_fresh(struct poweraid_raid_common_raid *raid);

/**
 * C7：data_offset 单轨。新卷 shell start() 早期调用，把框架预填的 1MiB
 * 成员保留统一为 5MiB（PPL/MWL 区）口径；重装配卷（rb->sb != NULL）为空操作。
 */
void poweraid_raid_common_sb_unify_data_offset(struct raid_bdev *rb);

/**
 * 擦除单个在线 bdev 盘头的 RAID 超级块（盘退役 / 移除后重新利用）。
 *
 * 仅处理当前不属于任何阵列的盘：以 write 独占方式打开，在线成员已被
 * claim 会直接失败。盘头无本族 sb 签名时幂等成功、不写入。擦除完成
 * （无论是否实际写零）后异步回调一次。
 *
 * 返回 0 表示异步流程已受理（cb 最终必被调用）；负数表示同步拒绝
 * （cb 不会被调用）。
 */
typedef void (*poweraid_raid_common_clear_disk_cb)(int status, void *cb_arg);
int poweraid_raid_common_clear_disk_sb(const char *bdev_name,
				       poweraid_raid_common_clear_disk_cb cb,
				       void *cb_arg);

/*
 * struct raid_bdev_module 的 sb 钩子实现，由 5f/6f/1f 薄壳注册。
 * 签名/契约与 bdev_raid.h 中对应字段一致。
 */
struct raid_bdev;
int poweraid_raid_common_sb_hook_validate(const void *buf, uint32_t buf_size);
uint32_t poweraid_raid_common_sb_hook_total_size(const struct raid_bdev_superblock *sb);
void poweraid_raid_common_sb_hook_write(struct raid_bdev *raid_bdev,
				       raid_bdev_write_sb_cb cb, void *cb_ctx);
void poweraid_raid_common_sb_hook_clear(struct raid_bdev *raid_bdev,
				       raid_bdev_write_sb_cb cb, void *cb_ctx);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID_COMMON_SB_H */
