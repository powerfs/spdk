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
 * 借鉴 XISRC xnr_dev_* 函数族 + raid_base_bdev_info.desc 框架协作。
 */

/* TRY_READ_MD 的 sb_load 回调：根据加载结果触发 VALIDATE_MD 或 SET_ONLINE。
 * 阶段 2：sb_load 实际 IO 异步完成。
 *   - status==0 && loaded_ctx!=NULL：盘上有 RAID sb，存到 bdev->loaded_sb_ctx
 *     供 VALIDATE_MD 校验 ext_signature + ext_crc。
 *   - status!=0（-EINVAL 等）：盘上无 sb（新卷）或读失败 → 直接上线（新卷）。
 *   loaded_ctx 仅在 status==0 时由调用方接管所有权（否则 sb_load 内部已 free）。
 */
static void
bdev_try_read_md_sb_load_cb(int status,
			     struct poweraid_raid_common_sb_ctx *loaded_ctx,
			     void *cb_arg)
{
	struct poweraid_raid_common_bdev *bdev = cb_arg;

	if (bdev == NULL) {
		/* 异常：cb_arg 无 bdev，释放 ctx 后返回 */
		if (loaded_ctx) {
			poweraid_raid_common_sb_free_loaded(loaded_ctx);
		}
		return;
	}

	SPDK_DEBUGLOG(raid5f_sm_bdev, "sb_load cb: bdev=%p status=%d loaded_ctx=%p\n",
		      bdev, status, loaded_ctx);

	if (status == 0 && loaded_ctx != NULL) {
		/* 盘上有 RAID sb → 持有 ctx，进入 VALIDATE_MD 校验 ext_signature */
		bdev->md_present = true;
		bdev->loaded_sb_ctx = loaded_ctx;
		poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_BDEV, bdev,
					   POWERAID_BDEV_EV_VALIDATE_MD);
	} else {
		/* 无 MD（新卷）或加载失败 → 直接上线 */
		assert(bdev->loaded_sb_ctx == NULL);
		poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_BDEV, bdev,
					   POWERAID_BDEV_EV_SET_ONLINE);
	}
}

/* OPEN：base bdev 已由框架打开（desc 已就位），置 BDEV_ST_OPEN，触发 TRY_READ_MD。
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

	/* 为 sb_load / PPL / recovery 生命周期准备 bdev IO channel（与 FSM 同线程，
	 * 在 unregister_done 中释放）。spdk_bdev_read/write 不接受 NULL channel。*/
	if (bdev->desc != NULL && bdev->ch == NULL) {
		bdev->ch = spdk_bdev_get_io_channel(
			spdk_bdev_desc_get_bdev((struct spdk_bdev_desc *)bdev->desc));
		if (bdev->ch == NULL) {
			SPDK_ERRLOG("open: get_io_channel failed bdev=%p\n", bdev);
			poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_FAULTED);
			return;
		}
	}

	/* 触发元数据加载 */
	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_BDEV, bdev,
				   POWERAID_BDEV_EV_TRY_READ_MD);
}

/* TRY_READ_MD：触发 superblock 异步加载（阶段 2 接入 spdk_bdev_read）。
 * 单盘探测，不污染 raid->sb_ctx；loaded_ctx 由回调持有到 VALIDATE_MD 完成。
 * 注：bdev->ch 由 OPEN 在子任务 D 设置（NULL 时 sb_load 仍可读，spdk_bdev_read 用全局 ch）。
 */
void
poweraid_raid_common_sm_bdev_try_read_md(struct poweraid_raid_common_bdev *bdev,
				    enum poweraid_raid_common_bdev_event event)
{
	SPDK_DEBUGLOG(raid5f_sm_bdev, "try_read_md: bdev=%p desc=%p ch=%p event=%u\n",
		      bdev, bdev ? bdev->desc : NULL,
		      bdev ? bdev->ch : NULL, (uint32_t)event);
	(void)event;

	if (bdev == NULL) {
		return;
	}

	poweraid_raid_common_sb_load(bdev->desc, (struct spdk_io_channel *)bdev->ch,
				bdev_try_read_md_sb_load_cb, bdev);
}

/* VALIDATE_MD：校验 ext_signature（v1 crc + ext_crc 已由 sb_load 内部校验通过）。
 * 从 bdev->loaded_sb_ctx 查 ext（不读 raid->sb_ctx，避免单盘探测污染 raid 级状态）。
 * 参考：XISRC xnr_dev_verify_md 检查 magic + ext_signature。
 */
void
poweraid_raid_common_sm_bdev_validate_md(struct poweraid_raid_common_bdev *bdev,
				    enum poweraid_raid_common_bdev_event event)
{
	const struct poweraid_raid_common_sb_v2_ext *ext;
	const struct raid_bdev_superblock *v1;

	SPDK_DEBUGLOG(raid5f_sm_bdev, "validate_md: bdev=%p event=%u\n", bdev,
		      (uint32_t)event);
	(void)event;

	if (bdev == NULL) {
		return;
	}

	v1 = poweraid_raid_common_sb_loaded_get_v1(bdev->loaded_sb_ctx);
	if (v1 == NULL) {
		SPDK_ERRLOG("validate_md: no loaded_ctx bdev=%p\n", bdev);
		poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_FAULTED);
		return;
	}

	ext = poweraid_raid_common_sb_loaded_get_ext(bdev->loaded_sb_ctx);
	if (ext == NULL) {
		/* v1 兼容加载（无 ext 区）：跳过 ext 校验，直接上线 */
		SPDK_DEBUGLOG(raid5f_sm_bdev, "validate_md: v1 disk (no ext) bdev=%p\n", bdev);
		poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_MD_VALID);
		poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_BDEV, bdev,
					   POWERAID_BDEV_EV_SET_ONLINE);
		return;
	}

	if (memcmp(ext->ext_signature, POWERAID_RAID_COMMON_SB_V2_EXT_SIG,
		   sizeof(ext->ext_signature)) != 0) {
		SPDK_ERRLOG("validate_md: ext_signature mismatch bdev=%p\n", bdev);
		poweraid_raid_common_sb_free_loaded(bdev->loaded_sb_ctx);
		bdev->loaded_sb_ctx = NULL;
		poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_FAULTED);
		return;
	}

	/* 阶段 2 子任务 C：若启用 PPL 特性，在此为该盘分配 PPL 上下文（仅内存，
	 * 不做 IO）。实际的 ppl_load_replay / recovery_run 由 RAID ONLINE（子任务 D）
	 * 在所有盘就绪 + read_fn 就绪后触发。bdev->ch 由 OPEN 阶段（子任务 D）设置。*/
	if ((ext->feature_flags & POWERAID_RAID_COMMON_SB_F_PPL) &&
	    bdev->ppl_ctx == NULL && bdev->desc != NULL &&
	    v1->block_size != 0 && ext->ppl_region_size != 0) {
		bdev->ppl_ctx = poweraid_raid_common_ppl_alloc(
			bdev->desc, (struct spdk_io_channel *)bdev->ch,
			v1->block_size,
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

	poweraid_raid_state_set(&bdev->state, POWERAID_BDEV_ST_MD_VALID);
	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_BDEV, bdev,
				   POWERAID_BDEV_EV_SET_ONLINE);
}

/* SET_ONLINE：置 BDEV_ST_ONLINE，检查所有 base bdev 是否均 online，
 * 若是则触发 RAID EV_ONLINE。释放 bdev->loaded_sb_ctx（校验已用完）。
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

	/* 释放 loaded_sb_ctx（VALIDATE_MD 已用完） */
	if (bdev->loaded_sb_ctx != NULL) {
		poweraid_raid_common_sb_free_loaded(bdev->loaded_sb_ctx);
		bdev->loaded_sb_ctx = NULL;
	}

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
