/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   BDEV 层 FSM handler（25 个事件，对应 poweraid_raid_common_sm.h 中
 *   enum poweraid_raid_common_bdev_event 全部真实事件）
 *
 *   阶段 1：全部为 stub，仅 SPDK_DEBUGLOG 打印 + 立即返回。
 *   后续阶段逐个填充实际逻辑（参考 XISRC xnr_dev_* 函数族）。
 *
 *   详见 raid5f-enhanced-design.md 第 3.10.6 节
 */

#include "spdk/stdinc.h"
#include "spdk/log.h"

#include "poweraid_raid_common.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_sm_bdev);

/* 同 RAID 层 stub 模板，但参数对象为 poweraid_raid_common_bdev */
#define DEFINE_BDEV_HANDLER(name)                                        \
void                                                                     \
poweraid_raid_common_sm_bdev_##name(struct poweraid_raid_common_bdev *bdev,        \
			       enum poweraid_raid_common_bdev_event event)        \
{                                                                        \
	SPDK_DEBUGLOG(raid5f_sm_bdev, "%s: bdev=%p state=0x%" PRIx64    \
		       " event=%u\n", #name, bdev,                       \
		       bdev ? bdev->state : 0ULL, (uint32_t)event);      \
	(void)event;                                                    \
	/* TODO 阶段 2+：实现实际状态转换 */                               \
}

/* ===== 生命周期 stub（8 个；open 见下方实现）===== */
DEFINE_BDEV_HANDLER(start)
DEFINE_BDEV_HANDLER(dev_start)
DEFINE_BDEV_HANDLER(try_open)
DEFINE_BDEV_HANDLER(close)
DEFINE_BDEV_HANDLER(close_finish)
DEFINE_BDEV_HANDLER(callback)
DEFINE_BDEV_HANDLER(finish)
DEFINE_BDEV_HANDLER(destroy)

/* ===== 元数据读取/校验 stub（6 个；try_read_md/validate_md 见下方实现）===== */
DEFINE_BDEV_HANDLER(read_md)
DEFINE_BDEV_HANDLER(test_md_empty)
DEFINE_BDEV_HANDLER(dev_in_raid)
DEFINE_BDEV_HANDLER(dev_not_in_raid)
DEFINE_BDEV_HANDLER(validate_empty_md)
DEFINE_BDEV_HANDLER(compare_md)

/* ===== 元数据写入/合并（4 个）===== */
DEFINE_BDEV_HANDLER(merge_md)
DEFINE_BDEV_HANDLER(write_md)
DEFINE_BDEV_HANDLER(flush_md_all)
DEFINE_BDEV_HANDLER(wait_flush_md)

/* ===== IO/在线状态 stub（3 个；set_online 见下方实现）===== */
DEFINE_BDEV_HANDLER(zero_md)
DEFINE_BDEV_HANDLER(wait_ch_op)
DEFINE_BDEV_HANDLER(check_dev_ready)

#undef DEFINE_BDEV_HANDLER

/* ===== 阶段 1.3 关键 handler 实现（BDEV 层 4 个）=====
 * 方向 B（issue #11）：单盘 sb 探测由框架 examine 完成；poweraid 不再自行
 * sb_load。OPEN 直接进入 VALIDATE_MD，校验对象是框架回放的 raid_bdev->sb。
 */

/* OPEN：base bdev 已由框架打开（desc 已就位），置 BDEV_ST_OPEN，触发 VALIDATE_MD。
 * 参考：raid_base_bdev_info.desc 由 raid_bdev 框架在 raid_bdev_add_base_bdev 分配。
 */
void
poweraid_raid_common_sm_bdev_open(struct poweraid_raid_common_bdev *bdev,
			     enum poweraid_raid_common_bdev_event event)
{
	SPDK_DEBUGLOG(raid5f_sm_bdev, "open: bdev=%p desc=%p event=%u\n", bdev,
		      bdev ? bdev->desc : NULL, (uint32_t)event);
	(void)event;

	if (bdev == NULL) {
		return;
	}

	poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_OPEN);

	/* 为 PPL / recovery 生命周期准备 bdev IO channel（与 FSM 同线程，
	 * 在 unregister_done 中释放）。spdk_bdev_read/write 不接受 NULL channel。*/
	if (bdev->desc != NULL && bdev->ch == NULL) {
		bdev->ch = spdk_bdev_get_io_channel(
			(struct spdk_bdev_desc *)bdev->desc);
		if (bdev->ch == NULL) {
			SPDK_ERRLOG("open: get_io_channel failed bdev=%p\n", bdev);
			poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_FAULTED);
			return;
		}
	}

	/* sb 探测由框架 examine 完成（raid_bdev->sb 已在 start() 前就位或为 NULL）。 */
	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_BDEV, bdev,
				   POWERAID_BDEV_EV_VALIDATE_MD);
}

/* TRY_READ_MD：方向 B 后不再有独立单盘探测；事件保留，直接转发 VALIDATE_MD。 */
void
poweraid_raid_common_sm_bdev_try_read_md(struct poweraid_raid_common_bdev *bdev,
				    enum poweraid_raid_common_bdev_event event)
{
	(void)event;

	if (bdev == NULL) {
		return;
	}

	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_BDEV, bdev,
				   POWERAID_BDEV_EV_VALIDATE_MD);
}

/* VALIDATE_MD：以框架回放的 raid_bdev->sb 为唯一权威做身份/成员一致性校验。
 *   - raid_bdev->sb == NULL：新卷，成员直接上线；
 *   - 否则逐盘校验：卷级 uuid/name/level/strip/成员数一致；成员槽 uuid 与盘匹配；
 *     成员状态合法；v2 ext 存在且几何合法，并按 feature_flags 分配 PPL/MWL ctx。
 * 任一不符：置 FAULTED 拒绝装配（绝不当空白盘覆写）。
 */
void
poweraid_raid_common_sm_bdev_validate_md(struct poweraid_raid_common_bdev *bdev,
				    enum poweraid_raid_common_bdev_event event)
{
	struct poweraid_raid_common_raid *raid;
	struct raid_bdev *rb;
	const struct raid_bdev_superblock *sb;
	const struct raid_bdev_sb_base_bdev *sbm = NULL;
	const struct poweraid_raid_common_sb_v2_ext *ext;
	uint8_t i;

	SPDK_DEBUGLOG(raid5f_sm_bdev, "validate_md: bdev=%p event=%u\n", bdev,
		      (uint32_t)event);
	(void)event;

	if (bdev == NULL || bdev->raid == NULL) {
		return;
	}
	raid = bdev->raid;
	rb = raid->raid_bdev;

	if (rb == NULL || rb->sb == NULL) {
		/* 新卷：盘上无本族 sb，直接上线（公共 sb 由框架首次 configure 写入）。 */
		bdev->md_present = false;
		poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_MD_VALID);
		poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_BDEV, bdev,
					   POWERAID_BDEV_EV_SET_ONLINE);
		return;
	}

	sb = rb->sb;

	/* 卷级身份一致性 */
	if (sb->level != raid->level ||
	    sb->strip_size != raid->strip_size ||
	    sb->num_base_bdevs != raid->num_base_bdevs ||
	    spdk_uuid_compare(&sb->uuid, &raid->uuid) != 0 ||
	    strncmp((const char *)sb->name, raid->name, RAID_BDEV_SB_NAME_SIZE) != 0) {
		SPDK_ERRLOG("validate_md: sb identity mismatch bdev=%p slot=%u\n",
			    bdev, bdev->slot);
		poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_FAULTED);
		return;
	}

	/* 成员槽：按本盘 uuid 在成员表中定位（槽位也应一致），并校验状态。 */
	for (i = 0; i < sb->base_bdevs_size; i++) {
		if (spdk_uuid_compare(&sb->base_bdevs[i].uuid, &bdev->uuid) == 0) {
			sbm = &sb->base_bdevs[i];
			break;
		}
	}
	if (sbm == NULL || sbm->slot != bdev->slot ||
	    (sbm->state != RAID_SB_BASE_BDEV_CONFIGURED &&
	     sbm->state != RAID_SB_BASE_BDEV_FAILED)) {
		SPDK_ERRLOG("validate_md: member uuid/state mismatch bdev=%p slot=%u\n",
			    bdev, bdev->slot);
		poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_FAULTED);
		return;
	}

	ext = poweraid_raid_common_sb_get_ext(raid);
	if (ext == NULL) {
		SPDK_ERRLOG("validate_md: v2 ext missing on reassembled raid %s\n",
			    raid->name);
		poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_FAULTED);
		return;
	}

	/* MWL 卷（raid1f）：ext 必须携带合法日志区几何，否则拒绝装配而不是
	 * 带着 0/残缺几何上线（MWL 层随后无法定位 ring）。*/
	if ((ext->feature_flags & POWERAID_RAID_COMMON_SB_F_MWL) &&
	    (ext->mwl_region_offset == 0 || ext->mwl_region_size == 0 ||
	     ext->mwl_region_offset % sb->block_size != 0 ||
	     ext->mwl_region_size % sb->block_size != 0)) {
		SPDK_ERRLOG("validate_md: bad MWL geometry off=%"PRIu64" size=%"PRIu64
			    " bdev=%p\n", ext->mwl_region_offset, ext->mwl_region_size, bdev);
		poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_FAULTED);
		return;
	}

	/* 若启用 PPL 特性，在此为该盘分配 PPL 上下文（仅内存，不做 IO）。
	 * 实际 replay/recovery 由 RAID ONLINE 在所有盘就绪后触发。*/
	if ((ext->feature_flags & POWERAID_RAID_COMMON_SB_F_PPL) &&
	    bdev->ppl_ctx == NULL && bdev->desc != NULL &&
	    sb->block_size != 0 && ext->ppl_region_size != 0) {
		bdev->ppl_ctx = poweraid_raid_common_ppl_alloc(
			bdev->desc, (struct spdk_io_channel *)bdev->ch,
			sb->block_size,
			ext->ppl_region_offset, ext->ppl_region_size);
		if (bdev->ppl_ctx == NULL) {
			SPDK_WARNLOG("validate_md: ppl_alloc failed bdev=%p (PPL disabled)\n", bdev);
		} else {
			SPDK_DEBUGLOG(raid5f_sm_bdev, "validate_md: ppl_ctx=%p bdev=%p "
				      "off=%"PRIu64" size=%"PRIu64"\n",
				      bdev->ppl_ctx, bdev, ext->ppl_region_offset,
				      ext->ppl_region_size);
		}
	}

	bdev->md_present = true;
	poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_MD_VALID);
	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_BDEV, bdev,
				   POWERAID_BDEV_EV_SET_ONLINE);
}

/* SET_ONLINE：置 BDEV_ST_ONLINE，检查所有 base bdev 是否均 online，
 * 若是则触发 RAID EV_ONLINE。
 * 参考：XISRC xnr_dev_online + raid5f 全盘就绪后框架注册 bdev。
 */
void
poweraid_raid_common_sm_bdev_set_online(struct poweraid_raid_common_bdev *bdev,
				   enum poweraid_raid_common_bdev_event event)
{
	struct poweraid_raid_common_raid *raid;
	uint8_t i;
	bool all_online = true;

	SPDK_DEBUGLOG(raid5f_sm_bdev, "set_online: bdev=%p event=%u\n", bdev,
		      (uint32_t)event);
	(void)event;

	if (bdev == NULL) {
		return;
	}

	poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_ONLINE);

	raid = bdev->raid;
	if (raid == NULL) {
		return;
	}

	/* 检查所有 base bdev 是否都已 online */
	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] == NULL) {
			all_online = false;
			break;
		}
		if (!poweraid_raid_state_test(&raid->base_bdevs[i]->state,
					       POWERAID_BDEV_ST_ONLINE)) {
			all_online = false;
			break;
		}
	}

	if (all_online) {
		poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_RAID, raid,
					   POWERAID_RAID_EV_ONLINE);
	}
}

/* ===== 全局 BDEV FSM 分派表 ===== */
poweraid_raid_common_bdev_handler_t
poweraid_raid_common_bdev_fsm[POWERAID_BDEV_EV_COUNT] = {
	[POWERAID_BDEV_EV_START]              = poweraid_raid_common_sm_bdev_start,
	[POWERAID_BDEV_EV_DEV_START]          = poweraid_raid_common_sm_bdev_dev_start,
	[POWERAID_BDEV_EV_OPEN]              = poweraid_raid_common_sm_bdev_open,
	[POWERAID_BDEV_EV_TRY_OPEN]           = poweraid_raid_common_sm_bdev_try_open,
	[POWERAID_BDEV_EV_CLOSE]             = poweraid_raid_common_sm_bdev_close,
	[POWERAID_BDEV_EV_CLOSE_FINISH]      = poweraid_raid_common_sm_bdev_close_finish,
	[POWERAID_BDEV_EV_CALLBACK]          = poweraid_raid_common_sm_bdev_callback,
	[POWERAID_BDEV_EV_FINISH]            = poweraid_raid_common_sm_bdev_finish,
	[POWERAID_BDEV_EV_DESTROY]           = poweraid_raid_common_sm_bdev_destroy,
	[POWERAID_BDEV_EV_TRY_READ_MD]       = poweraid_raid_common_sm_bdev_try_read_md,
	[POWERAID_BDEV_EV_READ_MD]           = poweraid_raid_common_sm_bdev_read_md,
	[POWERAID_BDEV_EV_TEST_MD_EMPTY]     = poweraid_raid_common_sm_bdev_test_md_empty,
	[POWERAID_BDEV_EV_DEV_IN_RAID]       = poweraid_raid_common_sm_bdev_dev_in_raid,
	[POWERAID_BDEV_EV_DEV_NOT_IN_RAID]   = poweraid_raid_common_sm_bdev_dev_not_in_raid,
	[POWERAID_BDEV_EV_VALIDATE_MD]       = poweraid_raid_common_sm_bdev_validate_md,
	[POWERAID_BDEV_EV_VALIDATE_EMPTY_MD] = poweraid_raid_common_sm_bdev_validate_empty_md,
	[POWERAID_BDEV_EV_COMPARE_MD]         = poweraid_raid_common_sm_bdev_compare_md,
	[POWERAID_BDEV_EV_MERGE_MD]          = poweraid_raid_common_sm_bdev_merge_md,
	[POWERAID_BDEV_EV_WRITE_MD]          = poweraid_raid_common_sm_bdev_write_md,
	[POWERAID_BDEV_EV_FLUSH_MD_ALL]      = poweraid_raid_common_sm_bdev_flush_md_all,
	[POWERAID_BDEV_EV_WAIT_FLUSH_MD]     = poweraid_raid_common_sm_bdev_wait_flush_md,
	[POWERAID_BDEV_EV_ZERO_MD]           = poweraid_raid_common_sm_bdev_zero_md,
	[POWERAID_BDEV_EV_WAIT_CH_OP]        = poweraid_raid_common_sm_bdev_wait_ch_op,
	[POWERAID_BDEV_EV_CHECK_DEV_READY]   = poweraid_raid_common_sm_bdev_check_dev_ready,
	[POWERAID_BDEV_EV_SET_ONLINE]        = poweraid_raid_common_sm_bdev_set_online,
};

SPDK_STATIC_ASSERT(SPDK_COUNTOF(poweraid_raid_common_bdev_fsm) ==
		   POWERAID_BDEV_EV_COUNT,
		   "bdev fsm table size mismatch with enum");
