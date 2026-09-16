/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   RAID 层 FSM handler（38 个事件，对应 poweraid_raid_common_sm.h 中
 *   enum poweraid_raid_common_raid_event 全部真实事件）
 *
 *   阶段 1：全部为 stub，仅 SPDK_DEBUGLOG 打印 + 立即返回。
 *   后续阶段逐个填充实际逻辑（参考 XISRC rdx_stt[STATE][EVENT] 二维分派）。
 *
 *   详见 raid5f-enhanced-design.md 第 3.10.6 节
 */

#include "spdk/stdinc.h"
#include "spdk/log.h"

#include "poweraid_raid_common.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_sm_raid);

/*
 * 统一 stub 模板：打印 raid 指针与当前状态位图，便于阶段 2 起逐个替换。
 * 注意 event 参数被显式 (void) 处理，避免 -Wunused-parameter 告警。
 */
#define DEFINE_RAID_HANDLER(name)                                        \
void                                                                     \
poweraid_raid_common_sm_raid_##name(struct poweraid_raid_common_raid *raid,        \
			       enum poweraid_raid_common_raid_event event)        \
{                                                                        \
	SPDK_DEBUGLOG(raid5f_sm_raid, "%s: raid=%p state=0x%" PRIx64     \
		       " event=%u\n", #name, raid,                         \
		       raid ? raid->state : 0ULL, (uint32_t)event);        \
	(void)event;                                                    \
	/* TODO 阶段 2+：实现实际状态转换 */                               \
}

/* ===== 生命周期 stub（6 个；create_dsc/open_bdevs/online/offline 见下方实现）===== */
DEFINE_RAID_HANDLER(create_dev)
DEFINE_RAID_HANDLER(destroy_dev)
DEFINE_RAID_HANDLER(create_old_dev)
DEFINE_RAID_HANDLER(create_old_dsc)
DEFINE_RAID_HANDLER(destroy_complete)
DEFINE_RAID_HANDLER(finish)

/* ===== 元数据（5 个）===== */
DEFINE_RAID_HANDLER(check_md)
DEFINE_RAID_HANDLER(apply_params)
DEFINE_RAID_HANDLER(save_config)
DEFINE_RAID_HANDLER(update_config)
DEFINE_RAID_HANDLER(update_config2)

/* ===== 盘管理（7 个）===== */
DEFINE_RAID_HANDLER(disk_open)
DEFINE_RAID_HANDLER(disk_verify)
DEFINE_RAID_HANDLER(disk_add)
DEFINE_RAID_HANDLER(disk_add_new)
DEFINE_RAID_HANDLER(disk_add_cancel)
DEFINE_RAID_HANDLER(disk_remove)
DEFINE_RAID_HANDLER(disk_online)

/* ===== 后台服务（7 个）===== */
DEFINE_RAID_HANDLER(start_init)
DEFINE_RAID_HANDLER(start_recon)
DEFINE_RAID_HANDLER(start_scrub)
DEFINE_RAID_HANDLER(start_restripe)
DEFINE_RAID_HANDLER(wait_services_stop)
DEFINE_RAID_HANDLER(wait_io_end)
DEFINE_RAID_HANDLER(wait_flush_md)

/* ===== 配置（6 个）===== */
DEFINE_RAID_HANDLER(write_lock)
DEFINE_RAID_HANDLER(add_raid)
DEFINE_RAID_HANDLER(remove_raid)
DEFINE_RAID_HANDLER(remove_raid_config)
DEFINE_RAID_HANDLER(mgmt_callback)
DEFINE_RAID_HANDLER(mgmt_get_callback)

/* ===== 注册（3 个）===== */
DEFINE_RAID_HANDLER(register_bdev)
DEFINE_RAID_HANDLER(iodev_unregister)
DEFINE_RAID_HANDLER(unregister_bdev)

#undef DEFINE_RAID_HANDLER

/* ===== 阶段 1.3 关键 handler 实现（RAID 层 4 个）=====
 * 借鉴 XISRC rdx_stt[STATE][EVENT] 二维分派 + raid5f_start/stop 框架协作。
 * 状态转换 + 调用 sb_ctx API；异步操作通过回调触发下一个 event。
 */

/* CREATE_DSC：创建描述符，分配并初始化 superblock v2 上下文。
 * 流程：sb_alloc → sb_init → 置 CONFIG_DIRTY → 触发 OPEN_BDEVS。
 * 参考：XISRC xnr_create_dsc + raid5f_start 的 r5f_info 分配（L1061-1095）。
 */
void
poweraid_raid_common_sm_raid_create_dsc(struct poweraid_raid_common_raid *raid,
				   enum poweraid_raid_common_raid_event event)
{
	int rc;

	SPDK_DEBUGLOG(raid5f_sm_raid, "create_dsc: raid=%p state=0x%" PRIx64
		      " event=%u\n", raid, raid ? raid->state : 0ULL, (uint32_t)event);
	(void)event;

	if (raid == NULL) {
		SPDK_ERRLOG("create_dsc: NULL raid\n");
		return;
	}

	/* 分配 superblock v2 上下文（v1 + base_bdevs + ext 区）*/
	rc = poweraid_raid_common_sb_alloc(raid, raid->block_size, raid->num_base_bdevs);
	if (rc != 0) {
		SPDK_ERRLOG("create_dsc: sb_alloc failed rc=%d (raid=%p)\n", rc, raid);
		return;
	}

	/* 初始化 superblock 字段（v2 版本号、ext_signature、CRC 双区）。
	 * 阶段 2 默认开启 PPL（write hole 根治）；后续阶段按 raid 配置增减。*/
	poweraid_raid_common_sb_init(raid, raid->level, raid->strip_size,
				POWERAID_RAID_COMMON_SB_F_PPL);

	/* 标记配置待持久化（阶段 2 由 EV_SAVE_CONFIG 落盘）*/
	poweraid_raid_state_set(&raid->state, POWERAID_RAID_ST_CONFIG_DIRTY);

	/* 触发下一事件：打开所有 base bdev */
	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_RAID, raid,
				   POWERAID_RAID_EV_OPEN_BDEVS);
}

/* OPEN_BDEVS：对每个 base_bdev 触发 BDEV FSM EV_OPEN。
 * 每个 bdev 在 SET_ONLINE 后回报；最后一盘 SET_ONLINE 时检测全部 online，
 * 触发 RAID EV_ONLINE。
 * 参考：XISRC xnr_open_bdevs 遍历 + raid5f RAID_FOR_EACH_BASE_BDEV（L1068）。
 */
void
poweraid_raid_common_sm_raid_open_bdevs(struct poweraid_raid_common_raid *raid,
				   enum poweraid_raid_common_raid_event event)
{
	uint8_t i;

	SPDK_DEBUGLOG(raid5f_sm_raid, "open_bdevs: raid=%p num_base_bdevs=%u"
		      " event=%u\n", raid, raid ? raid->num_base_bdevs : 0,
		      (uint32_t)event);
	(void)event;

	if (raid == NULL || raid->base_bdevs == NULL) {
		SPDK_ERRLOG("open_bdevs: NULL raid or base_bdevs\n");
		return;
	}

	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] == NULL) {
			SPDK_ERRLOG("open_bdevs: base_bdev[%u] NULL\n", i);
			continue;
		}
		poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_BDEV,
					   raid->base_bdevs[i],
					   POWERAID_BDEV_EV_OPEN);
	}
}

/* ===== parity fixup：对 data==old/data==new 的未提交 stripe，用当前数据重算
 * 并重写 parity（写+flush）。INCONSISTENT 仅告警（阶段 2 不做数据重构）。===== */
struct parity_fixup {
	struct poweraid_raid_common_raid		*raid;
	struct poweraid_raid_common_recovery_result	*results;
	uint32_t				num;
	uint32_t				idx;
	uint32_t				chunk;
	uint32_t				data_chunks;
	void					*pbuf;
	size_t					strip_bytes;
};

static void parity_fixup_next(struct parity_fixup *op);
static void parity_fixup_read_cb(int status, const void *buf, size_t len, void *cb_arg);
static void parity_fixup_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void parity_fixup_flush_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);

static void
parity_fixup_finish(struct parity_fixup *op)
{
	struct poweraid_raid_common_raid *raid = op->raid;

	SPDK_NOTICELOG("online: parity fixup done, %u uncommitted stripes processed\n",
		       op->num);
	poweraid_raid_common_recovery_free_result(op->results);
	free(op);
	/* recovery + fixup 全部落盘后数据面才允许 IO */
	poweraid_raid_state_clear(&raid->state, POWERAID_RAID_ST_RESTORING);
}

/* stop 与 fixup 并发时安全终止（通道可能已销毁，不能再提交 bdev IO）*/
static void
parity_fixup_abort(struct parity_fixup *op, const char *why)
{
	struct poweraid_raid_common_raid *raid = op->raid;

	SPDK_WARNLOG("online: parity fixup aborted (%s) at %u/%u\n",
		     why, op->idx, op->num);
	if (op->pbuf != NULL) {
		spdk_dma_free(op->pbuf);
	}
	poweraid_raid_common_recovery_free_result(op->results);
	free(op);
	poweraid_raid_state_clear(&raid->state, POWERAID_RAID_ST_RESTORING);
}

static void
parity_fixup_write_or_next(struct parity_fixup *op)
{
	struct poweraid_raid_common_bdev *bdev;
	struct spdk_bdev_desc *desc;
	struct spdk_io_channel *ch;
	uint64_t stripe = op->results[op->idx].stripe_id;
	uint8_t p_idx;
	uint64_t offset;
	int rc;

	if (poweraid_raid_state_test(&op->raid->state, POWERAID_RAID_ST_OFFLINE)) {
		parity_fixup_abort(op, "stop before parity write");
		return;
	}
	p_idx = op->data_chunks - (stripe % op->raid->num_base_bdevs);
	bdev = op->raid->base_bdevs[p_idx];
	desc = (struct spdk_bdev_desc *)bdev->desc;
	ch = (struct spdk_io_channel *)bdev->ch;
	offset = op->raid->data_offset_blocks +
		 stripe * op->raid->strip_size;

	rc = spdk_bdev_write_blocks(desc, ch, op->pbuf, offset,
				    op->raid->strip_size,
				    parity_fixup_write_cb, op);
	if (rc == -ENOMEM) {
		/* 简化处理：阶段 2 启动期资源紧张直接报错跳过，不排队 */
		SPDK_ERRLOG("fixup: parity write ENOMEM stripe=%"PRIu64"\n", stripe);
		spdk_dma_free(op->pbuf);
		op->pbuf = NULL;
		op->idx++;
		parity_fixup_next(op);
	} else if (rc != 0) {
		SPDK_ERRLOG("fixup: parity write rc=%d stripe=%"PRIu64"\n", rc, stripe);
		spdk_dma_free(op->pbuf);
		op->pbuf = NULL;
		op->idx++;
		parity_fixup_next(op);
	}
}

static void
parity_fixup_read_cb(int status, const void *buf, size_t len, void *cb_arg)
{
	struct parity_fixup *op = cb_arg;
	uint8_t *dst = op->pbuf;
	const uint8_t *src = buf;
	size_t i;

	if (poweraid_raid_state_test(&op->raid->state, POWERAID_RAID_ST_OFFLINE)) {
		parity_fixup_abort(op, "stop during fixup reads");
		return;
	}
	if (status != 0 || buf == NULL || len != op->strip_bytes) {
		SPDK_ERRLOG("fixup: read chunk %u failed (%d) stripe=%"PRIu64"\n",
			    op->chunk, status, op->results[op->idx].stripe_id);
		spdk_dma_free(op->pbuf);
		op->pbuf = NULL;
		op->idx++;
		parity_fixup_next(op);
		return;
	}

	for (i = 0; i < len; i++) {
		dst[i] ^= src[i];
	}
	op->chunk++;
	if (op->chunk < op->data_chunks) {
		poweraid_raid_common_recovery_read_strip(op->raid,
			op->results[op->idx].stripe_id, op->chunk,
			op->raid->strip_size, parity_fixup_read_cb, op);
		return;
	}
	parity_fixup_write_or_next(op);
}

static void
parity_fixup_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct parity_fixup *op = cb_arg;
	struct poweraid_raid_common_bdev *bdev;
	uint64_t stripe = op->results[op->idx].stripe_id;
	uint8_t p_idx;
	uint64_t offset;
	int rc;

	spdk_bdev_free_io(bdev_io);
	if (poweraid_raid_state_test(&op->raid->state, POWERAID_RAID_ST_OFFLINE)) {
		parity_fixup_abort(op, "stop after parity write");
		return;
	}
	if (!success) {
		SPDK_ERRLOG("fixup: parity write failed stripe=%"PRIu64"\n", stripe);
		spdk_dma_free(op->pbuf);
		op->pbuf = NULL;
		op->idx++;
		parity_fixup_next(op);
		return;
	}

	p_idx = op->data_chunks - (stripe % op->raid->num_base_bdevs);
	bdev = op->raid->base_bdevs[p_idx];
	offset = op->raid->data_offset_blocks +
		 stripe * op->raid->strip_size;
	rc = spdk_bdev_flush_blocks((struct spdk_bdev_desc *)bdev->desc,
				    (struct spdk_io_channel *)bdev->ch,
				    offset, op->raid->strip_size,
				    parity_fixup_flush_cb, op);
	if (rc != 0) {
		/* -ENOTSUP/不支持 flush 的盘视为持久完成 */
		SPDK_WARNLOG("fixup: flush rc=%d stripe=%"PRIu64" (continue)\n",
			     rc, stripe);
		parity_fixup_flush_cb(NULL, true, op);
	}
}

static void
parity_fixup_flush_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct parity_fixup *op = cb_arg;

	if (bdev_io != NULL) {
		spdk_bdev_free_io(bdev_io);
	}
	if (poweraid_raid_state_test(&op->raid->state, POWERAID_RAID_ST_OFFLINE)) {
		parity_fixup_abort(op, "stop after fixup flush");
		return;
	}
	if (!success) {
		SPDK_ERRLOG("fixup: flush failed stripe=%"PRIu64"\n",
			    op->results[op->idx].stripe_id);
	}
	spdk_dma_free(op->pbuf);
	op->pbuf = NULL;
	op->idx++;
	parity_fixup_next(op);
}

static void
parity_fixup_next(struct parity_fixup *op)
{
	if (poweraid_raid_state_test(&op->raid->state, POWERAID_RAID_ST_OFFLINE)) {
		parity_fixup_abort(op, "stop");
		return;
	}
	while (op->idx < op->num) {
		enum poweraid_raid_common_recovery_action act = op->results[op->idx].action;
		if (act != POWERAID_RECOVERY_ACT_NONE &&
		    act != POWERAID_RECOVERY_ACT_REWRITE_PARITY) {
			op->idx++;  /* INCONSISTENT：不处理 */
			continue;
		}
		op->chunk = 0;
		op->pbuf = spdk_dma_malloc(op->strip_bytes, 0x1000, NULL);
		if (op->pbuf == NULL) {
			SPDK_ERRLOG("fixup: alloc pbuf failed\n");
			op->idx++;
			continue;
		}
		memset(op->pbuf, 0, op->strip_bytes);
		poweraid_raid_common_recovery_read_strip(op->raid,
			op->results[op->idx].stripe_id, 0,
			op->raid->strip_size, parity_fixup_read_cb, op);
		return;
	}
	parity_fixup_finish(op);
}

/* recovery_run 完成回调（阶段 2）：
 *   - 打印三分支判定结果；
 *   - 对 NONE / REWRITE_PARITY 的 stripe 异步重算重写 parity（flush 落盘）；
 *   - INCONSISTENT 仅告警（部分写，阶段 2 无法重构数据，留 scrub/阶段 3+）。*/
static void
poweraid_raid_common_online_recovery_done(int status,
		const struct poweraid_raid_common_recovery_result *results,
		uint32_t num_results, void *cb_arg)
{
	struct poweraid_raid_common_raid *raid = cb_arg;
	struct parity_fixup *op;
	uint32_t i;

	if (status != 0) {
		SPDK_ERRLOG("online: recovery failed (%d) raid=%p\n", status, raid);
		poweraid_raid_state_clear(&raid->state, POWERAID_RAID_ST_RESTORING);
		return;
	}
	if (num_results == 0) {
		SPDK_NOTICELOG("poweraid_raid_common_ppl: replay clean (no uncommitted) raid=%p\n", raid);
		poweraid_raid_state_clear(&raid->state, POWERAID_RAID_ST_RESTORING);
		return;
	}
	SPDK_NOTICELOG("poweraid_raid_common_ppl: replay found %u uncommitted records raid=%p\n",
		       num_results, raid);
	for (i = 0; i < num_results; i++) {
		const char *act = "NONE";
		switch (results[i].action) {
		case POWERAID_RECOVERY_ACT_REWRITE_PARITY: act = "REWRITE_PARITY"; break;
		case POWERAID_RECOVERY_ACT_INCONSISTENT:  act = "INCONSISTENT"; break;
		default: break;
		}
		SPDK_NOTICELOG("recovery: rec seq=%" PRIu64 " stripe=%" PRIu64 " → %s\n",
			       results[i].seq, results[i].stripe_id, act);
	}

	op = calloc(1, sizeof(*op));
	if (op == NULL) {
		poweraid_raid_common_recovery_free_result((struct poweraid_raid_common_recovery_result *)results);
		return;
	}
	op->raid = raid;
	op->results = (struct poweraid_raid_common_recovery_result *)results;
	op->num = num_results;
	op->data_chunks = raid->num_base_bdevs - 1;
	op->strip_bytes = (size_t)raid->strip_size * raid->block_size;
	parity_fixup_next(op);
}

/* 实际置 ONLINE 并触发 recovery（新卷初始化完成 / 既有卷直接进入）。*/
static void
raid_enter_online(struct poweraid_raid_common_raid *raid)
{
	uint8_t i;

	poweraid_raid_state_clear(&raid->state, POWERAID_RAID_ST_OFFLINE);

	/* 恢复（含 parity fixup）完成前数据面必须保持关闭：先置 RESTORING 再置 ONLINE，
	 * submit 门控要求 ONLINE && !RESTORING，RESTORING 在 recovery 回调/fixup
	 * 结束时才清除。*/
	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL && raid->base_bdevs[i]->ppl_ctx != NULL) {
			poweraid_raid_state_set(&raid->state, POWERAID_RAID_ST_RESTORING);
			break;
		}
	}
	poweraid_raid_state_set(&raid->state, POWERAID_RAID_ST_ONLINE);

	/* 触发 PPL 恢复：扫描首个带 ppl_ctx 的盘 */
	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL && raid->base_bdevs[i]->ppl_ctx != NULL) {
			SPDK_NOTICELOG("online: kick recovery on bdev[%u] ppl_ctx=%p\n",
				       i, raid->base_bdevs[i]->ppl_ctx);
			poweraid_raid_common_recovery_run(raid->base_bdevs[i]->ppl_ctx, raid,
						     poweraid_raid_common_recovery_read_strip,
						     poweraid_raid_common_online_recovery_done, raid);
			break;
		}
	}
}

/* 新卷 PPL 初始化：逐盘 alloc ppl_ctx + ppl_init（写 PPL super + flush）。*/
struct fresh_ppl_op {
	struct poweraid_raid_common_raid	*raid;
	uint8_t				idx;
	int				status;
	uint64_t			region_offset;
	uint64_t			region_size;
};

static void fresh_ppl_init_cb(int status, void *cb_arg);

static void
fresh_ppl_step(struct fresh_ppl_op *op)
{
	struct poweraid_raid_common_bdev *bdev;
	struct poweraid_raid_common_ppl_ctx *ppl_ctx;

	while (op->idx < op->raid->num_base_bdevs) {
		bdev = op->raid->base_bdevs[op->idx];
		if (bdev == NULL || bdev->desc == NULL || bdev->ch == NULL) {
			SPDK_ERRLOG("fresh_init: bdev[%u] not ready\n", op->idx);
			op->status = -EINVAL;
			op->idx++;
			continue;
		}
		ppl_ctx = poweraid_raid_common_ppl_alloc(bdev->desc,
					(struct spdk_io_channel *)bdev->ch,
					op->raid->block_size,
					op->region_offset, op->region_size);
		if (ppl_ctx == NULL) {
			SPDK_ERRLOG("fresh_init: ppl_alloc failed bdev[%u]\n", op->idx);
			op->status = -ENOMEM;
			op->idx++;
			continue;
		}
		bdev->ppl_ctx = ppl_ctx;
		poweraid_raid_common_ppl_init(ppl_ctx, fresh_ppl_init_cb, op);
		return;  /* 等回调 */
	}

	if (op->status != 0) {
		SPDK_ERRLOG("fresh_init: PPL init failed (%d)\n", op->status);
	}
	SPDK_NOTICELOG("fresh_init: all %u PPL regions initialized\n",
		       op->raid->num_base_bdevs);
	raid_enter_online(op->raid);
	free(op);
}

static void
fresh_ppl_init_cb(int status, void *cb_arg)
{
	struct fresh_ppl_op *op = cb_arg;

	if (status != 0) {
		SPDK_ERRLOG("fresh_init: ppl_init bdev[%u] failed (%d)\n",
			    op->idx, status);
		op->status = status;
	}
	op->idx++;
	fresh_ppl_step(op);
}

/* sb_write 完成回调：开始逐盘 PPL 初始化。*/
static void
fresh_sb_write_cb(int status, void *cb_arg)
{
	struct poweraid_raid_common_raid *raid = cb_arg;
	struct fresh_ppl_op *op;

	if (status != 0) {
		SPDK_ERRLOG("fresh_init: sb_write failed (%d) raid=%s\n",
			    status, raid->name);
		/* 仍继续上线（无 PPL 保护降级），避免卷悬挂 */
	}
	op = calloc(1, sizeof(*op));
	if (op == NULL) {
		raid_enter_online(raid);
		return;
	}
	op->raid = raid;
	if (poweraid_raid_common_sb_get_ppl_region(raid, &op->region_offset,
					      &op->region_size) != 0) {
		SPDK_ERRLOG("fresh_init: no PPL region in sb ext\n");
		free(op);
		raid_enter_online(raid);
		return;
	}
	fresh_ppl_step(op);
}

/* ONLINE：所有 base bdev 就绪后的汇聚点。
 * - 全部盘无 sb（新卷）：sb_write 落盘 → 逐盘 ppl_init → 置 ONLINE → recovery。
 * - 既有卷（VALIDATE_MD 已按 ext 分配 ppl_ctx）：直接置 ONLINE → recovery。
 */
void
poweraid_raid_common_sm_raid_online(struct poweraid_raid_common_raid *raid,
			       enum poweraid_raid_common_raid_event event)
{
	uint8_t i;
	bool all_fresh;

	SPDK_DEBUGLOG(raid5f_sm_raid, "online: raid=%p old_state=0x%" PRIx64
		      " event=%u\n", raid, raid ? raid->state : 0ULL,
		      (uint32_t)event);
	(void)event;

	if (raid == NULL) {
		return;
	}

	all_fresh = true;
	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL &&
		    raid->base_bdevs[i]->md_present) {
			all_fresh = false;
			break;
		}
	}

	if (all_fresh) {
		SPDK_NOTICELOG("online: fresh volume, writing sb + init PPL raid=%s\n",
			       raid->name);
		poweraid_raid_common_sb_write(raid, fresh_sb_write_cb, raid);
		return;  /* raid_enter_online 由异步回调触发 */
	}

	raid_enter_online(raid);
}

/* OFFLINE：清 RAID_ST_ONLINE，置 OFFLINE。卷停止接受 IO。
 * 后续由 EV_DESTROY_COMPLETE 走销毁路径。
 * 参考：XISRC xnr_offline 清位 + raid5f_stop 返回 false 异步（L1119）。
 */
void
poweraid_raid_common_sm_raid_offline(struct poweraid_raid_common_raid *raid,
				enum poweraid_raid_common_raid_event event)
{
	SPDK_DEBUGLOG(raid5f_sm_raid, "offline: raid=%p old_state=0x%" PRIx64
		      " event=%u\n", raid, raid ? raid->state : 0ULL,
		      (uint32_t)event);
	(void)event;

	if (raid == NULL) {
		return;
	}

	poweraid_raid_state_clear(&raid->state, POWERAID_RAID_ST_ONLINE);
	poweraid_raid_state_set(&raid->state, POWERAID_RAID_ST_OFFLINE);
}

/* ===== 全局 RAID FSM 分派表（dispatch.c 引用）=====
 * 索引 = enum poweraid_raid_common_raid_event，值 = handler 函数指针。
 * 槽位与 enum 顺序严格对齐（designated initializer 已保证）。
 */
poweraid_raid_common_raid_handler_t
poweraid_raid_common_raid_fsm[POWERAID_RAID_EV_COUNT_REAL] = {
	[POWERAID_RAID_EV_CREATE_DEV]         = poweraid_raid_common_sm_raid_create_dev,
	[POWERAID_RAID_EV_DESTROY_DEV]        = poweraid_raid_common_sm_raid_destroy_dev,
	[POWERAID_RAID_EV_CREATE_OLD_DEV]     = poweraid_raid_common_sm_raid_create_old_dev,
	[POWERAID_RAID_EV_OPEN_BDEVS]         = poweraid_raid_common_sm_raid_open_bdevs,
	[POWERAID_RAID_EV_CREATE_DSC]         = poweraid_raid_common_sm_raid_create_dsc,
	[POWERAID_RAID_EV_CREATE_OLD_DSC]     = poweraid_raid_common_sm_raid_create_old_dsc,
	[POWERAID_RAID_EV_ONLINE]             = poweraid_raid_common_sm_raid_online,
	[POWERAID_RAID_EV_OFFLINE]             = poweraid_raid_common_sm_raid_offline,
	[POWERAID_RAID_EV_DESTROY_COMPLETE]    = poweraid_raid_common_sm_raid_destroy_complete,
	[POWERAID_RAID_EV_FINISH]             = poweraid_raid_common_sm_raid_finish,
	[POWERAID_RAID_EV_CHECK_MD]           = poweraid_raid_common_sm_raid_check_md,
	[POWERAID_RAID_EV_APPLY_PARAMS]       = poweraid_raid_common_sm_raid_apply_params,
	[POWERAID_RAID_EV_SAVE_CONFIG]        = poweraid_raid_common_sm_raid_save_config,
	[POWERAID_RAID_EV_UPDATE_CONFIG]      = poweraid_raid_common_sm_raid_update_config,
	[POWERAID_RAID_EV_UPDATE_CONFIG2]     = poweraid_raid_common_sm_raid_update_config2,
	[POWERAID_RAID_EV_DISK_OPEN]          = poweraid_raid_common_sm_raid_disk_open,
	[POWERAID_RAID_EV_DISK_VERIFY]        = poweraid_raid_common_sm_raid_disk_verify,
	[POWERAID_RAID_EV_DISK_ADD]           = poweraid_raid_common_sm_raid_disk_add,
	[POWERAID_RAID_EV_DISK_ADD_NEW]       = poweraid_raid_common_sm_raid_disk_add_new,
	[POWERAID_RAID_EV_DISK_ADD_CANCEL]    = poweraid_raid_common_sm_raid_disk_add_cancel,
	[POWERAID_RAID_EV_DISK_REMOVE]        = poweraid_raid_common_sm_raid_disk_remove,
	[POWERAID_RAID_EV_DISK_ONLINE]        = poweraid_raid_common_sm_raid_disk_online,
	[POWERAID_RAID_EV_START_INIT]         = poweraid_raid_common_sm_raid_start_init,
	[POWERAID_RAID_EV_START_RECON]        = poweraid_raid_common_sm_raid_start_recon,
	[POWERAID_RAID_EV_START_SCRUB]        = poweraid_raid_common_sm_raid_start_scrub,
	[POWERAID_RAID_EV_START_RESTRIPE]     = poweraid_raid_common_sm_raid_start_restripe,
	[POWERAID_RAID_EV_WAIT_SERVICES_STOP] = poweraid_raid_common_sm_raid_wait_services_stop,
	[POWERAID_RAID_EV_WAIT_IO_END]        = poweraid_raid_common_sm_raid_wait_io_end,
	[POWERAID_RAID_EV_WAIT_FLUSH_MD]      = poweraid_raid_common_sm_raid_wait_flush_md,
	[POWERAID_RAID_EV_WRITE_LOCK]         = poweraid_raid_common_sm_raid_write_lock,
	[POWERAID_RAID_EV_ADD_RAID]           = poweraid_raid_common_sm_raid_add_raid,
	[POWERAID_RAID_EV_REMOVE_RAID]        = poweraid_raid_common_sm_raid_remove_raid,
	[POWERAID_RAID_EV_REMOVE_RAID_CONFIG] = poweraid_raid_common_sm_raid_remove_raid_config,
	[POWERAID_RAID_EV_MGMT_CALLBACK]      = poweraid_raid_common_sm_raid_mgmt_callback,
	[POWERAID_RAID_EV_MGMT_GET_CALLBACK]  = poweraid_raid_common_sm_raid_mgmt_get_callback,
	[POWERAID_RAID_EV_REGISTER_BDEV]      = poweraid_raid_common_sm_raid_register_bdev,
	[POWERAID_RAID_EV_IODEV_UNREGISTER]   = poweraid_raid_common_sm_raid_iodev_unregister,
	[POWERAID_RAID_EV_UNREGISTER_BDEV]    = poweraid_raid_common_sm_raid_unregister_bdev,
};

SPDK_STATIC_ASSERT(SPDK_COUNTOF(poweraid_raid_common_raid_fsm) ==
		   POWERAID_RAID_EV_COUNT_REAL,
		   "raid fsm table size mismatch with enum");

/* ===== 缓冲池实现（data_buf / parity_buf 预分配）=====
 *
 * 优化点：原 full-stripe 写路径每 IO 调用 spdk_dma_malloc/free 分配
 * data_buf（整 stripe）和 parity_buf（单 strip），频繁分配释放导致：
 *   1. DMA allocator 锁竞争（spdk_dma_malloc 内部有 mpool 锁）
 *   2. 缓存行/TLB 抖动
 *   3. 延迟不稳定
 *
 * 方案：per-channel 预分配 MAX_STRIPES 个 data_buf 和 parity_buf，
 *       池空时 fallback 到 spdk_dma_malloc，池满时 put 直接 free。
 *       与 stripe_request 池深度对齐，正常负载下零 malloc。
 *       per-channel 单线程访问，无需锁。
 */

int
poweraid_raid_common_buf_pool_init(struct poweraid_raid_common_io_channel *ch,
				   struct poweraid_raid_common_raid *raid)
{
	uint32_t data_chunks = raid->num_base_bdevs - 1;
	uint32_t strip_bytes = raid->strip_size * raid->block_size;
	struct poweraid_raid_common_buf *entry;
	int i;

	ch->data_buf_size = strip_bytes * data_chunks;
	ch->parity_buf_size = strip_bytes;
	ch->n_data_bufs = 0;
	ch->n_parity_bufs = 0;

	TAILQ_INIT(&ch->free_data_bufs);
	TAILQ_INIT(&ch->free_parity_bufs);

	for (i = 0; i < POWERAID_RAID_COMMON_MAX_STRIPES; i++) {
		entry = calloc(1, sizeof(*entry));
		if (entry == NULL) {
			goto err;
		}
		entry->buf = spdk_dma_malloc(ch->data_buf_size, 0, NULL);
		if (entry->buf == NULL) {
			free(entry);
			goto err;
		}
		TAILQ_INSERT_HEAD(&ch->free_data_bufs, entry, link);
		ch->n_data_bufs++;
	}

	for (i = 0; i < POWERAID_RAID_COMMON_MAX_STRIPES; i++) {
		entry = calloc(1, sizeof(*entry));
		if (entry == NULL) {
			goto err;
		}
		entry->buf = spdk_dma_malloc(ch->parity_buf_size, 0, NULL);
		if (entry->buf == NULL) {
			free(entry);
			goto err;
		}
		TAILQ_INSERT_HEAD(&ch->free_parity_bufs, entry, link);
		ch->n_parity_bufs++;
	}

	SPDK_DEBUGLOG(raid5f_sm_raid, "buf_pool_init: data_buf=%"PRIu32
		      "B x%d parity_buf=%"PRIu32"B x%d\n",
		      ch->data_buf_size, ch->n_data_bufs,
		      ch->parity_buf_size, ch->n_parity_bufs);
	return 0;

err:
	poweraid_raid_common_buf_pool_destroy(ch);
	return -ENOMEM;
}

void
poweraid_raid_common_buf_pool_destroy(struct poweraid_raid_common_io_channel *ch)
{
	struct poweraid_raid_common_buf *entry;

	while ((entry = TAILQ_FIRST(&ch->free_data_bufs))) {
		TAILQ_REMOVE(&ch->free_data_bufs, entry, link);
		spdk_dma_free(entry->buf);
		free(entry);
	}
	ch->n_data_bufs = 0;

	while ((entry = TAILQ_FIRST(&ch->free_parity_bufs))) {
		TAILQ_REMOVE(&ch->free_parity_bufs, entry, link);
		spdk_dma_free(entry->buf);
		free(entry);
	}
	ch->n_parity_bufs = 0;
}

void *
poweraid_raid_common_get_data_buf(struct poweraid_raid_common_io_channel *ch)
{
	struct poweraid_raid_common_buf *entry;
	void *buf;

	entry = TAILQ_FIRST(&ch->free_data_bufs);
	if (entry != NULL) {
		TAILQ_REMOVE(&ch->free_data_bufs, entry, link);
		ch->n_data_bufs--;
		buf = entry->buf;
		free(entry);
		return buf;
	}
	/* 池空：fallback malloc（burst 场景）*/
	buf = spdk_dma_malloc(ch->data_buf_size, 0, NULL);
	return buf;
}

void *
poweraid_raid_common_get_parity_buf(struct poweraid_raid_common_io_channel *ch)
{
	struct poweraid_raid_common_buf *entry;
	void *buf;

	entry = TAILQ_FIRST(&ch->free_parity_bufs);
	if (entry != NULL) {
		TAILQ_REMOVE(&ch->free_parity_bufs, entry, link);
		ch->n_parity_bufs--;
		buf = entry->buf;
		free(entry);
		return buf;
	}
	buf = spdk_dma_malloc(ch->parity_buf_size, 0, NULL);
	return buf;
}

void
poweraid_raid_common_put_data_buf(struct poweraid_raid_common_io_channel *ch, void *buf)
{
	struct poweraid_raid_common_buf *entry;

	if (buf == NULL) {
		return;
	}
	if (ch->n_data_bufs < POWERAID_RAID_COMMON_MAX_STRIPES) {
		entry = calloc(1, sizeof(*entry));
		if (entry != NULL) {
			entry->buf = buf;
			TAILQ_INSERT_HEAD(&ch->free_data_bufs, entry, link);
			ch->n_data_bufs++;
			return;
		}
	}
	spdk_dma_free(buf);
}

void
poweraid_raid_common_put_parity_buf(struct poweraid_raid_common_io_channel *ch, void *buf)
{
	struct poweraid_raid_common_buf *entry;

	if (buf == NULL) {
		return;
	}
	if (ch->n_parity_bufs < POWERAID_RAID_COMMON_MAX_STRIPES) {
		entry = calloc(1, sizeof(*entry));
		if (entry != NULL) {
			entry->buf = buf;
			TAILQ_INSERT_HEAD(&ch->free_parity_bufs, entry, link);
			ch->n_parity_bufs++;
			return;
		}
	}
	spdk_dma_free(buf);
}
