/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   poweraid_raid6f 模块入口：RAID6 (P+Q 双校验) 注册到 SPDK bdev_raid 框架
 *
 *   Stage 4 RAID6：data_chunks = N-2，双校验 P (XOR) + Q (GF8 RS)。
 *   布局：left-symmetric (mdadm-compatible)，见 poweraid_raid_common_get_parity_idx。
 */

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/log.h"
#include "spdk/string.h"
#include "spdk/util.h"
#include "spdk/thread.h"
#include "spdk/accel.h"

#include "../bdev_raid.h"
#include "poweraid_raid6f.h"
#include "../poweraid_raid_common/poweraid_raid_common_rmw.h"
#include "../poweraid_raid_common/poweraid_raid_common_merge.h"
#include "../poweraid_raid_common/poweraid_raid_common_gf8.h"
#include "../poweraid_raid_common/poweraid_raid_common_rebuild.h"
#include "poweraid_raid6f_recovery.h"

SPDK_LOG_REGISTER_COMPONENT(poweraid_raid6f)

/* ===== stripe_request 池化管理 ===== */

static struct poweraid_raid_common_req *
poweraid_raid6f_stripe_request_alloc(struct poweraid_raid_common_io_channel *ch,
				     enum poweraid_raid_common_stripe_type type)
{
	struct poweraid_raid_common_req *req;

	req = calloc(1, sizeof(*req));
	if (!req) {
		return NULL;
	}
	req->type = type;
	req->ch = ch;
	return req;
}

static void
poweraid_raid6f_stripe_request_free(struct poweraid_raid_common_req *req)
{
	if (!req) {
		return;
	}
	free(req);
}

/* 前向声明 */
static void
poweraid_raid6f_read_strip(struct poweraid_raid_common_raid *raid,
			   uint64_t stripe_id, uint32_t chunk_idx,
			   uint32_t chunk_len_blocks,
			   void (*cb)(int status, const void *buf, size_t len, void *cb_arg),
			   void *cb_arg);

/* ===== per-thread IO channel ===== */

static int
poweraid_raid6f_ioch_create(void *io_device, void *ctx_buf)
{
	struct poweraid_raid_common_raid *raid = io_device;
	struct poweraid_raid_common_io_channel *ch = ctx_buf;
	struct poweraid_raid_common_req *req;
	int i;

	TAILQ_INIT(&ch->free_write_stripe_requests);
	TAILQ_INIT(&ch->free_reconstruct_stripe_requests);
	TAILQ_INIT(&ch->xor_retry_queue);

	for (i = 0; i < POWERAID_RAID_COMMON_MAX_STRIPES; i++) {
		req = poweraid_raid6f_stripe_request_alloc(ch,
				POWERAID_RAID_COMMON_STRIPE_REQ_WRITE);
		if (!req) {
			goto err;
		}
		TAILQ_INSERT_HEAD(&ch->free_write_stripe_requests, req, link);
	}

	for (i = 0; i < POWERAID_RAID_COMMON_MAX_STRIPES; i++) {
		req = poweraid_raid6f_stripe_request_alloc(ch,
				POWERAID_RAID_COMMON_STRIPE_REQ_RECONSTRUCT);
		if (!req) {
			goto err;
		}
		TAILQ_INSERT_HEAD(&ch->free_reconstruct_stripe_requests, req, link);
	}

	ch->accel_ch = spdk_accel_get_io_channel();
	if (!ch->accel_ch) {
		SPDK_ERRLOG("poweraid_raid6f: failed to get accel io channel\n");
		goto err;
	}

	if (poweraid_raid_common_buf_pool_init(ch, raid) != 0) {
		SPDK_ERRLOG("poweraid_raid6f: buf_pool_init failed\n");
		spdk_put_io_channel(ch->accel_ch);
		ch->accel_ch = NULL;
		goto err;
	}

	if (poweraid_raid_common_merge_init(&ch->merge_ctx, raid, ch) != 0) {
		SPDK_ERRLOG("poweraid_raid6f: merge_init failed\n");
		spdk_put_io_channel(ch->accel_ch);
		ch->accel_ch = NULL;
		goto err;
	}

	SPDK_DEBUGLOG(poweraid_raid6f, "ioch_create: raid=%p ch=%p\n", raid, ch);
	return 0;

err:
	SPDK_ERRLOG("poweraid_raid6f: ioch_create failed\n");
	poweraid_raid_common_buf_pool_destroy(ch);
	while ((req = TAILQ_FIRST(&ch->free_write_stripe_requests))) {
		TAILQ_REMOVE(&ch->free_write_stripe_requests, req, link);
		poweraid_raid6f_stripe_request_free(req);
	}
	while ((req = TAILQ_FIRST(&ch->free_reconstruct_stripe_requests))) {
		TAILQ_REMOVE(&ch->free_reconstruct_stripe_requests, req, link);
		poweraid_raid6f_stripe_request_free(req);
	}
	return -ENOMEM;
}

static void
poweraid_raid6f_ioch_destroy(void *io_device, void *ctx_buf)
{
	struct poweraid_raid_common_io_channel *ch = ctx_buf;
	struct poweraid_raid_common_req *req;

	assert(TAILQ_EMPTY(&ch->xor_retry_queue));

	poweraid_raid_common_merge_destroy(&ch->merge_ctx);

	while ((req = TAILQ_FIRST(&ch->free_write_stripe_requests))) {
		TAILQ_REMOVE(&ch->free_write_stripe_requests, req, link);
		poweraid_raid6f_stripe_request_free(req);
	}

	while ((req = TAILQ_FIRST(&ch->free_reconstruct_stripe_requests))) {
		TAILQ_REMOVE(&ch->free_reconstruct_stripe_requests, req, link);
		poweraid_raid6f_stripe_request_free(req);
	}

	poweraid_raid_common_buf_pool_destroy(ch);

	if (ch->accel_ch) {
		spdk_put_io_channel(ch->accel_ch);
		ch->accel_ch = NULL;
	}

	SPDK_DEBUGLOG(poweraid_raid6f, "ioch_destroy: ch=%p\n", ch);
}

/* ===== 模块生命周期 ===== */

int
poweraid_raid6f_start(struct raid_bdev *raid_bdev)
{
	struct poweraid_raid_common_raid *raid;
	struct raid_base_bdev_info *base_info;
	uint64_t min_blockcnt = UINT64_MAX;
	uint64_t base_data_size, total_stripes, stripe_blocks;
	uint32_t data_chunks;
	uint8_t i;

	SPDK_NOTICELOG("poweraid_raid6f: start raid=%s num_base_bdevs=%u\n",
		       raid_bdev->bdev.name, raid_bdev->num_base_bdevs);

	/* 初始化 GF8 引擎（生成 exp/log 表 + 选择 SIMD 路径），供 Q 校验计算 */
	poweraid_raid_common_gf8_init();

	if (raid_bdev->num_base_bdevs < 4) {
		SPDK_ERRLOG("poweraid_raid6f: need >= 4 base bdevs (got %u)\n",
			    raid_bdev->num_base_bdevs);
		return -EINVAL;
	}

	raid = calloc(1, sizeof(*raid));
	if (!raid) {
		SPDK_ERRLOG("poweraid_raid6f: alloc raid failed\n");
		return -ENOMEM;
	}
	raid->raid_bdev = raid_bdev;
	raid->level = (uint32_t)raid_bdev->level;
	raid->strip_size = raid_bdev->strip_size;
	raid->block_size = raid_bdev->bdev.blocklen;
	raid->num_base_bdevs = raid_bdev->num_base_bdevs;
	raid->num_parity = 2;
	raid->delay_us = MERGE_DELAY_US_DEFAULT;
	/* 注册 6f 模块差异回调（CALC = P+Q，READ_STRIP = RAID6 布局）*/
	raid->ops.calc_parity = poweraid_raid6f_calc_parity;
	raid->ops.read_strip = poweraid_raid6f_read_strip;
	raid->ops.recover_missing = poweraid_raid6f_recover_strip;
	spdk_uuid_copy(&raid->uuid, &raid_bdev->bdev.uuid);
	snprintf(raid->name, sizeof(raid->name), "%s", raid_bdev->bdev.name);

	raid->base_bdevs = calloc(raid->num_base_bdevs, sizeof(*raid->base_bdevs));
	if (!raid->base_bdevs) {
		SPDK_ERRLOG("poweraid_raid6f: alloc base_bdevs failed\n");
		free(raid);
		return -ENOMEM;
	}

	i = 0;
	RAID_FOR_EACH_BASE_BDEV(raid_bdev, base_info) {
		struct poweraid_raid_common_bdev *bdev = calloc(1, sizeof(*bdev));
		if (!bdev) {
			SPDK_ERRLOG("poweraid_raid6f: alloc bdev[%u] failed\n", i);
			while (i > 0) {
				free(raid->base_bdevs[--i]);
			}
			free(raid->base_bdevs);
			free(raid);
			return -ENOMEM;
		}
		bdev->slot = i;
		bdev->raid = raid;
		bdev->desc = base_info->desc;
		bdev->ch = NULL;
		spdk_uuid_copy(&bdev->uuid, &base_info->uuid);
		raid->base_bdevs[i] = bdev;
		if (base_info->desc) {
			uint64_t bs = base_info->data_size ? base_info->data_size :
				    spdk_bdev_desc_get_bdev(base_info->desc)->blockcnt;
			min_blockcnt = spdk_min(min_blockcnt, bs);
		}
		i++;
	}

	/* RAID6: data_chunks = N - 2 */
	data_chunks = raid->num_base_bdevs - 2;

	{
		uint64_t reserve = (POWERAID_RAID_COMMON_PPL_REGION_OFFSET +
				    POWERAID_RAID_COMMON_PPL_REGION_SIZE) /
				   raid->block_size;
		if (reserve % raid->strip_size != 0) {
			SPDK_ERRLOG("poweraid_raid6f: PPL reserve %"PRIu64
				    " not strip-aligned (strip=%u)\n",
				    reserve, raid->strip_size);
			return -EINVAL;
		}
		raid->data_offset_blocks = reserve;
	}
	if (min_blockcnt <= raid->data_offset_blocks) {
		SPDK_ERRLOG("poweraid_raid6f: base bdev too small (%"PRIu64
			    " <= reserve %"PRIu64")\n",
			    min_blockcnt, raid->data_offset_blocks);
		return -EINVAL;
	}
	base_data_size = ((min_blockcnt - raid->data_offset_blocks) /
			  raid->strip_size) * raid->strip_size;
	total_stripes = base_data_size / raid->strip_size;
	stripe_blocks = raid->strip_size * data_chunks;

	raid_bdev->bdev.blockcnt = stripe_blocks * total_stripes;
	raid_bdev->bdev.optimal_io_boundary = raid->strip_size;
	raid_bdev->bdev.split_on_optimal_io_boundary = true;
	raid_bdev->bdev.write_unit_size = raid->strip_size;
	raid_bdev->bdev.split_on_write_unit = true;

	raid->raid_size = raid_bdev->bdev.blockcnt;
	raid_bdev->module_private = raid;

	spdk_io_device_register(raid, poweraid_raid6f_ioch_create,
				poweraid_raid6f_ioch_destroy,
				sizeof(struct poweraid_raid_common_io_channel), NULL);

	SPDK_NOTICELOG("poweraid_raid6f: raid=%p stripe_blocks=%"PRIu64
		       " total_stripes=%"PRIu64" blockcnt=%"PRIu64"\n",
		       raid, stripe_blocks, total_stripes, raid_bdev->bdev.blockcnt);

	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_RAID, raid,
				   POWERAID_RAID_EV_CREATE_DSC);
	return 0;
}

static void
poweraid_raid6f_io_device_unregister_done(void *io_device)
{
	struct poweraid_raid_common_raid *raid = io_device;
	struct raid_bdev *raid_bdev = raid->raid_bdev;
	uint8_t i;

	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL) {
			if (raid->base_bdevs[i]->ppl_ctx != NULL) {
				poweraid_raid_common_ppl_free(raid->base_bdevs[i]->ppl_ctx);
			}
			if (raid->base_bdevs[i]->ch != NULL) {
				spdk_put_io_channel(
					(struct spdk_io_channel *)raid->base_bdevs[i]->ch);
			}
			free(raid->base_bdevs[i]);
		}
	}
	free(raid->base_bdevs);

	if (raid->sb_ctx != NULL) {
		poweraid_raid_common_sb_free(raid);
	}

	SPDK_NOTICELOG("poweraid_raid6f: io_device unregistered, raid=%p\n", raid);
	free(raid);

	raid_bdev_module_stop_done(raid_bdev);
}

bool
poweraid_raid6f_stop(struct raid_bdev *raid_bdev)
{
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	uint8_t i;

	SPDK_NOTICELOG("poweraid_raid6f: stop raid=%s\n", raid_bdev->bdev.name);

	if (raid == NULL) {
		return true;
	}

	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_RAID, raid,
				   POWERAID_RAID_EV_OFFLINE);

	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (raid->base_bdevs[i] != NULL && raid->base_bdevs[i]->ppl_ctx != NULL) {
			poweraid_raid_common_ppl_free(raid->base_bdevs[i]->ppl_ctx);
			raid->base_bdevs[i]->ppl_ctx = NULL;
		}
	}

	spdk_io_device_unregister(raid, poweraid_raid6f_io_device_unregister_done);
	return false;
}

/* ===== read 完成回调（含直读失败→降级重建重试一次）===== */

/* read 路径用 module_private 兼作降级重建重试计数（0/1/2，最多容忍 2 盘故障）。
 * merge 写路径也复用该字段，READ 分支入口会先清 0。*/

static void
poweraid_raid6f_read_complete_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct raid_bdev_io *raid_io = cb_arg;
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	uint32_t data_chunks = raid->num_base_bdevs - raid->num_parity;
	uint32_t stripe_blocks = raid->strip_size * data_chunks;
	uint64_t stripe_index = raid_io->offset_blocks / stripe_blocks;
	uint64_t stripe_offset = raid_io->offset_blocks % stripe_blocks;
	uint8_t chunk_data_idx = stripe_offset >> raid_bdev->strip_size_shift;
	uint8_t p_idx, q_idx, phys;
	uintptr_t retries;
	int rc;

	spdk_bdev_free_io(bdev_io);

	if (spdk_likely(success)) {
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_SUCCESS);
		return;
	}

	SPDK_WARNLOG("poweraid_raid6f: direct read failed, mark slot faulted"
		     " and retry degraded\n");

	/* 直读 IO error：将目标盘标记 FAULTED 后走降级重建。
	 * module_private 兼作降级重试计数（最多 2 盘故障 = 2 次重建重试）。*/
	retries = (uintptr_t)raid_io->module_private;
	if (retries >= raid->num_parity) {
		SPDK_ERRLOG("poweraid_raid6f: degraded retries exhausted (%lu),"
			    " fail read\n", (unsigned long)retries);
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	poweraid_raid_common_get_parity_idx(raid, stripe_index, &p_idx, &q_idx);
	phys = poweraid_raid_common_data_to_phys(chunk_data_idx, p_idx, q_idx,
						raid->num_base_bdevs);
	poweraid_raid6f_mark_slot_faulted(raid, raid_io->raid_ch, phys);

	raid_io->module_private = (void *)(retries + 1);
	rc = poweraid_raid6f_submit_read_request(raid_io);
	if (rc != 0) {
		raid_bdev_io_complete(raid_io, rc == -ENOMEM ?
				      SPDK_BDEV_IO_STATUS_NOMEM :
				      SPDK_BDEV_IO_STATUS_FAILED);
	}
}

/* recovery / parity-fixup 用：读某 stripe 的某个 data chunk（整 strip）。
 * chunk_idx 为 data 序号（0..N-3），内部用 data_to_phys 映射到物理盘（跳过 P/Q）。*/
struct recovery_read_ctx {
	void	(*cb)(int status, const void *buf, size_t len, void *cb_arg);
	void	*cb_arg;
	void	*buf;
	size_t	len;
};

static void
poweraid_raid6f_strip_read_io_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct recovery_read_ctx *rctx = cb_arg;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		spdk_dma_free(rctx->buf);
		rctx->cb(-EIO, NULL, 0, rctx->cb_arg);
		free(rctx);
		return;
	}
	rctx->cb(0, rctx->buf, rctx->len, rctx->cb_arg);
	spdk_dma_free(rctx->buf);
	free(rctx);
}

static void
poweraid_raid6f_read_strip(struct poweraid_raid_common_raid *raid,
			   uint64_t stripe_id, uint32_t chunk_idx,
			   uint32_t chunk_len_blocks,
			   void (*cb)(int status, const void *buf, size_t len, void *cb_arg),
			   void *cb_arg)
{
	struct poweraid_raid_common_bdev *bdev;
	struct recovery_read_ctx *rctx;
	uint8_t p_idx, q_idx;
	uint8_t phys;
	uint64_t offset;
	int rc;

	poweraid_raid_common_get_parity_idx(raid, stripe_id, &p_idx, &q_idx);
	phys = poweraid_raid_common_data_to_phys(chunk_idx, p_idx, q_idx, raid->num_base_bdevs);

	if (phys >= raid->num_base_bdevs) {
		cb(-EINVAL, NULL, 0, cb_arg);
		return;
	}
	bdev = raid->base_bdevs[phys];
	if (bdev == NULL || bdev->desc == NULL || bdev->ch == NULL) {
		cb(-ENODEV, NULL, 0, cb_arg);
		return;
	}

	rctx = calloc(1, sizeof(*rctx));
	if (rctx == NULL) {
		cb(-ENOMEM, NULL, 0, cb_arg);
		return;
	}
	rctx->cb = cb;
	rctx->cb_arg = cb_arg;
	rctx->len = (size_t)chunk_len_blocks * raid->block_size;
	rctx->buf = spdk_dma_malloc(rctx->len, 0x1000, NULL);
	if (rctx->buf == NULL) {
		free(rctx);
		cb(-ENOMEM, NULL, 0, cb_arg);
		return;
	}

	offset = raid->data_offset_blocks + stripe_id * raid->strip_size;
	rc = spdk_bdev_read_blocks((struct spdk_bdev_desc *)bdev->desc,
				   (struct spdk_io_channel *)bdev->ch,
				   rctx->buf, offset, chunk_len_blocks,
				   poweraid_raid6f_strip_read_io_cb, rctx);
	if (rc != 0) {
		spdk_dma_free(rctx->buf);
		free(rctx);
		cb(rc, NULL, 0, cb_arg);
	}
}

/* layout-aware read：盘健在直接读目标 chunk；目标盘故障走降级重建。*/
int
poweraid_raid6f_submit_read_request(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	uint32_t data_chunks = raid->num_base_bdevs - 2;
	uint32_t stripe_blocks = raid->strip_size * data_chunks;
	uint64_t stripe_index = raid_io->offset_blocks / stripe_blocks;
	uint64_t stripe_offset = raid_io->offset_blocks % stripe_blocks;
	uint8_t chunk_data_idx = stripe_offset >> raid_bdev->strip_size_shift;
	uint8_t p_idx, q_idx;
	uint8_t chunk_idx;
	struct raid_base_bdev_info *base_info;
	struct spdk_io_channel *base_ch;
	uint64_t chunk_offset = stripe_offset - (chunk_data_idx << raid_bdev->strip_size_shift);
	uint64_t base_offset;
	struct spdk_bdev_ext_io_opts io_opts = {0};

	poweraid_raid_common_get_parity_idx(raid, stripe_index, &p_idx, &q_idx);
	chunk_idx = poweraid_raid_common_data_to_phys(chunk_data_idx, p_idx, q_idx, raid->num_base_bdevs);
	base_info = &raid_bdev->base_bdev_info[chunk_idx];
	base_offset = raid->data_offset_blocks +
		      (stripe_index << raid_bdev->strip_size_shift) + chunk_offset;

	/* 目标盘故障（热拔/通道释放/IO error 标记）→ 从 P/Q 降级重建 */
	if (poweraid_raid6f_slot_faulted(raid, raid_io->raid_ch, chunk_idx)) {
		return poweraid_raid6f_submit_degraded_read(raid_io, stripe_index,
							    p_idx, q_idx,
							    chunk_idx, chunk_data_idx,
							    chunk_offset);
	}

	io_opts.size = sizeof(io_opts);
	io_opts.memory_domain = raid_io->memory_domain;
	io_opts.memory_domain_ctx = raid_io->memory_domain_ctx;
	io_opts.metadata = raid_io->md_buf;

	base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, chunk_idx);

	return raid_bdev_readv_blocks_ext(base_info, base_ch,
					 raid_io->iovs, raid_io->iovcnt,
					 base_offset, raid_io->num_blocks,
					 poweraid_raid6f_read_complete_cb, raid_io,
					 &io_opts);
}

void
poweraid_raid6f_submit_rw_request(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	struct poweraid_raid_common_io_channel *ch;
	struct poweraid_raid_common_req *req;
	uint32_t data_chunks = raid->num_base_bdevs - 2;
	uint32_t stripe_blocks = raid->strip_size * data_chunks;
	uint64_t stripe_index, stripe_offset;
	uint32_t strip_size_bytes;
	uint32_t blocklen = raid->block_size;
	uint32_t i;
	int rc;

	if (poweraid_raid_state_test((uint64_t *)&raid->state,
				     POWERAID_RAID_ST_RESTORING) ||
	    !poweraid_raid_state_test((uint64_t *)&raid->state,
				      POWERAID_RAID_ST_ONLINE)) {
		SPDK_WARNLOG("poweraid_raid6f: IO before online/restoring, rejected\n");
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_NOMEM);
		return;
	}

	switch (raid_io->type) {
	case SPDK_BDEV_IO_TYPE_READ:
		assert(raid_io->num_blocks <= raid->strip_size);
		/* module_private 在 read 路径是降级重试计数，入口清 0，避免
		 * bdev_io ctx 复用残留（merge 写路径也用该字段）*/
		raid_io->module_private = NULL;
		rc = poweraid_raid6f_submit_read_request(raid_io);
		if (rc != 0) {
			raid_bdev_io_complete(raid_io, rc == -ENOMEM ?
					      SPDK_BDEV_IO_STATUS_NOMEM :
					      SPDK_BDEV_IO_STATUS_FAILED);
		}
		return;

	case SPDK_BDEV_IO_TYPE_WRITE:
		break;

	default:
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	stripe_index = raid_io->offset_blocks / stripe_blocks;
	stripe_offset = raid_io->offset_blocks % stripe_blocks;
	if (stripe_offset != 0 || raid_io->num_blocks != stripe_blocks) {
		int merge_rc = poweraid_raid_common_merge_submit(raid_io);
		if (merge_rc != 0) {
			raid_bdev_io_complete(raid_io, merge_rc == -ENOMEM ?
					      SPDK_BDEV_IO_STATUS_NOMEM :
					      SPDK_BDEV_IO_STATUS_FAILED);
		}
		return;
	}

	/* 重建窗口门控：未越过窗口的全 stripe 写必须延迟到该 stripe 重构完成 */
	{
		struct raid_bdev_io_channel *eff_ch;
		enum poweraid_raid_common_gate gate;

		gate = poweraid_raid_common_rebuild_gate_classify(raid,
				raid_io->raid_ch, stripe_index, &eff_ch);
		if (gate == POWERAID_RAID_COMMON_GATE_WAIT) {
			raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_NOMEM);
			return;
		}
		ch = raid_bdev_channel_get_module_ctx(raid_io->raid_ch);
		if (ch == NULL) {
			SPDK_ERRLOG("poweraid_raid6f: no module channel\n");
			raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
			return;
		}

		req = TAILQ_FIRST(&ch->free_write_stripe_requests);
		if (req == NULL) {
			SPDK_ERRLOG("poweraid_raid6f: no free stripe_request\n");
			raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_NOMEM);
			return;
		}
		TAILQ_REMOVE(&ch->free_write_stripe_requests, req, link);

		req->type = POWERAID_RAID_COMMON_STRIPE_REQ_WRITE;
		req->raid_io = raid_io;
		req->eff_raid_ch = eff_ch;
		req->stripe_index = stripe_index;
		req->raid = raid;
		req->io = raid_io;
	}

	strip_size_bytes = raid->strip_size * blocklen;

	/* P 缓冲 */
	req->parity_buf_alloc = poweraid_raid_common_get_parity_buf(ch);
	if (req->parity_buf_alloc == NULL) {
		SPDK_ERRLOG("poweraid_raid6f: alloc parity_buf failed\n");
		goto err_free_req;
	}
	req->parity_buf = req->parity_buf_alloc;

	/* Q 缓冲 */
	req->q_buf_alloc = poweraid_raid_common_get_q_buf(ch);
	if (req->q_buf_alloc == NULL) {
		SPDK_ERRLOG("poweraid_raid6f: alloc q_buf failed\n");
		goto err_free_parity;
	}
	req->q_buf = req->q_buf_alloc;

	/* 全 stripe 数据缓冲 */
	req->data_buf = poweraid_raid_common_get_data_buf(ch);
	if (req->data_buf == NULL) {
		SPDK_ERRLOG("poweraid_raid6f: alloc data_buf failed\n");
		goto err_free_q;
	}

	rc = (int)spdk_iovcpy(raid_io->iovs, raid_io->iovcnt,
		 &(struct iovec){ .iov_base = req->data_buf,
				  .iov_len = stripe_blocks * blocklen }, 1);
	if (rc != (int)(stripe_blocks * blocklen)) {
		SPDK_ERRLOG("poweraid_raid6f: iovcpy short copied=%d expect=%"PRIu64"\n",
			    rc, (uint64_t)stripe_blocks * blocklen);
		goto err_free_data;
	}

	req->src_bufs = calloc(data_chunks, sizeof(void *));
	if (req->src_bufs == NULL) {
		SPDK_ERRLOG("poweraid_raid6f: alloc src_bufs failed\n");
		goto err_free_data;
	}
	for (i = 0; i < data_chunks; i++) {
		req->src_bufs[i] = (char *)req->data_buf + i * strip_size_bytes;
	}
	req->n_src = data_chunks;
	req->xor_len = strip_size_bytes;

	SPDK_DEBUGLOG(poweraid_raid6f, "submit_write: req=%p stripe=%"PRIu64
		      " parity_buf=%p q_buf=%p data_buf=%p\n",
		      req, stripe_index, req->parity_buf, req->q_buf, req->data_buf);

	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_REQ, req,
				   POWERAID_REQ_EV_ASSIGN);
	return;

err_free_data:
	poweraid_raid_common_put_data_buf(ch, req->data_buf);
	req->data_buf = NULL;
err_free_q:
	poweraid_raid_common_put_q_buf(ch, req->q_buf_alloc);
	req->q_buf_alloc = NULL;
	req->q_buf = NULL;
err_free_parity:
	poweraid_raid_common_put_parity_buf(ch, req->parity_buf_alloc);
	req->parity_buf_alloc = NULL;
	req->parity_buf = NULL;
err_free_req:
	TAILQ_INSERT_HEAD(&ch->free_write_stripe_requests, req, link);
	raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_NOMEM);
}

struct spdk_io_channel *
poweraid_raid6f_get_io_channel(struct raid_bdev *raid_bdev)
{
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;

	return spdk_get_io_channel(raid);
}

int
poweraid_raid6f_submit_process_request(struct raid_bdev_process_request *process_req,
				       struct raid_bdev_io_channel *raid_ch)
{
	return poweraid_raid_common_submit_process_request(process_req, raid_ch);
}

/* ===== FLUSH / UNMAP ===== */
static void
poweraid_raid6f_submit_null_payload_request(struct raid_bdev_io *raid_io)
{
	switch (raid_io->type) {
	case SPDK_BDEV_IO_TYPE_FLUSH:
		poweraid_raid_common_merge_flush_all(raid_io);
		break;

	case SPDK_BDEV_IO_TYPE_UNMAP:
		SPDK_WARNLOG("poweraid_raid6f: UNMAP not yet supported\n");
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		break;

	default:
		SPDK_ERRLOG("poweraid_raid6f: invalid null payload io type %u\n",
			    raid_io->type);
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		break;
	}
}

/* ===== 模块注册 ===== */
struct raid_bdev_module g_poweraid_raid6f_module = {
	.level = SPDK_BDEV_RAID_LEVEL_RAID6F,
	.base_bdevs_min = 4,
	.base_bdevs_constraint = {CONSTRAINT_MAX_BASE_BDEVS_REMOVED, 2},
	.start = poweraid_raid6f_start,
	.stop = poweraid_raid6f_stop,
	.submit_rw_request = poweraid_raid6f_submit_rw_request,
	.get_io_channel = poweraid_raid6f_get_io_channel,
	.submit_process_request = poweraid_raid6f_submit_process_request,
	.submit_null_payload_request = poweraid_raid6f_submit_null_payload_request,
	.base_bdev_removed = poweraid_raid_common_hook_base_bdev_removed,
	.base_bdev_rebuild_starting = poweraid_raid_common_hook_rebuild_starting,
	.process_complete = poweraid_raid_common_hook_process_complete,
	.process_window_advanced = poweraid_raid_common_hook_window_advanced,
};
RAID_MODULE_REGISTER(&g_poweraid_raid6f_module)
