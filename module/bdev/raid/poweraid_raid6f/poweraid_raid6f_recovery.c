/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   RAID6F 降级读 + 重建 strip 恢复实现，详见 poweraid_raid6f_recovery.h。
 *   Stage 4 步骤 7：本文件由 poweraid_raid6f.c 抽出，降级读行为保持不变，
 *   新增 poweraid_raid6f_recover_strip 供公共重建引擎调用。
 */

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/string.h"
#include "spdk/util.h"

#include "../bdev_raid.h"
#include "../poweraid_raid_common/poweraid_raid_common.h"
#include "../poweraid_raid_common/poweraid_raid_common_gf8.h"
#include "poweraid_raid6f_recovery.h"

/* log component 注册在 poweraid_raid6f.c（同一链接单元只能注册一次）*/

/* ===== 降级读（degraded read）：1~2 盘故障时从 P/Q 重建目标 strip ===== */

#define POWERAID_RAID6F_DEG_METHOD_P_XOR	0
#define POWERAID_RAID6F_DEG_METHOD_Q_SINGLE	1
#define POWERAID_RAID6F_DEG_METHOD_DUAL		2

struct poweraid_raid6f_deg_ctx {
	struct raid_bdev_io		*raid_io;
	struct poweraid_raid_common_raid *raid;
	struct raid_bdev_io_channel	*raid_ch;
	void				**bufs;		/* 按 phys 索引，未参与为 NULL */
	uint8_t			n;		/* num_base_bdevs */
	uint8_t			p_idx;
	uint8_t			q_idx;
	uint8_t			target_data;
	uint8_t			missing[2];	/* 故障 data 序号（升序），DUAL 用 */
	uint8_t			method;
	uint32_t		strip_bytes;
	uint32_t		data_off_bytes;	/* 目标数据在重建 strip 内偏移 */
	uint32_t		data_len_bytes;	/* 请求长度 */
	uint32_t		remaining;
	int			status;
};

/* 每个底层 strip 读一个子上下文（回调需知道 phys）*/
struct poweraid_raid6f_deg_subio {
	struct poweraid_raid6f_deg_ctx	*dctx;
	uint8_t				phys;
	struct spdk_bdev_io_wait_entry	wait_entry;
};

bool
poweraid_raid6f_slot_faulted(struct poweraid_raid_common_raid *raid,
			     struct raid_bdev_io_channel *raid_ch, uint8_t phys)
{
	struct raid_base_bdev_info *base_info = &raid->raid_bdev->base_bdev_info[phys];

	if (base_info->desc == NULL) {
		return true;
	}
	if (raid_bdev_channel_get_base_channel(raid_ch, phys) == NULL) {
		return true;
	}
	if (raid->base_bdevs[phys] != NULL &&
	    poweraid_raid_state_test(&raid->base_bdevs[phys]->state,
				     POWERAID_BDEV_ST_FAULTED)) {
		return true;
	}
	return false;
}

/* 惰性刷新卷降级状态位（步骤 7 rebuild 将复用）*/
void
poweraid_raid6f_refresh_degraded_state(struct poweraid_raid_common_raid *raid,
				       struct raid_bdev_io_channel *raid_ch)
{
	uint8_t i, n_fault = 0;

	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (poweraid_raid6f_slot_faulted(raid, raid_ch, i)) {
			n_fault++;
			if (raid->base_bdevs[i] != NULL) {
				poweraid_raid_state_set(&raid->base_bdevs[i]->state,
							POWERAID_BDEV_ST_FAULTED);
			}
		}
	}

	if (n_fault >= 2) {
		poweraid_raid_state_set(&raid->state,
					POWERAID_RAID_ST_DEGRADED |
					POWERAID_RAID_ST_DEGRADED2);
	} else if (n_fault == 1) {
		poweraid_raid_state_set(&raid->state, POWERAID_RAID_ST_DEGRADED);
	}
}

/* 将某个物理槽标记为故障（读 IO error / 热拔事件），并刷新卷降级状态位。
 * 步骤 7 rebuild 将复用该入口做全局故障通告。*/
void
poweraid_raid6f_mark_slot_faulted(struct poweraid_raid_common_raid *raid,
				  struct raid_bdev_io_channel *raid_ch, uint8_t phys)
{
	if (phys >= raid->num_base_bdevs) {
		return;
	}
	if (raid->base_bdevs[phys] != NULL) {
		poweraid_raid_state_set(&raid->base_bdevs[phys]->state,
					POWERAID_BDEV_ST_FAULTED);
	}
	poweraid_raid6f_refresh_degraded_state(raid, raid_ch);
}

static void
poweraid_raid6f_deg_finish(struct poweraid_raid6f_deg_ctx *dctx)
{
	struct raid_bdev_io *raid_io = dctx->raid_io;
	struct poweraid_raid_common_raid *raid = dctx->raid;
	uint32_t data_chunks = raid->num_base_bdevs - raid->num_parity;
	uint32_t len = dctx->strip_bytes;
	const uint8_t *exp = poweraid_raid_common_gf8_exp;
	void *out = NULL, *out2 = NULL;
	uint8_t i;
	int rc = dctx->status;

	if (rc == 0) {
		out = spdk_dma_malloc(len, 0x1000, NULL);
		if (out == NULL) {
			rc = -ENOMEM;
		}
	}

	if (rc == 0 && dctx->method == POWERAID_RAID6F_DEG_METHOD_P_XOR) {
		/* D_t = P XOR 其他存活 data */
		memcpy(out, dctx->bufs[dctx->p_idx], len);
		for (i = 0; i < data_chunks; i++) {
			uint8_t phys;
			if (i == dctx->target_data) {
				continue;
			}
			phys = poweraid_raid_common_data_to_phys(i, dctx->p_idx,
				      dctx->q_idx, dctx->n);
			if (dctx->bufs[phys] != NULL) {
				/* 乘 1 = XOR，走 SIMD 路径 */
				poweraid_raid_common_gf8_mul_const_xor(dctx->bufs[phys],
						       1, out, len);
			}
		}
	}

	if (rc == 0 && dctx->method == POWERAID_RAID6F_DEG_METHOD_Q_SINGLE) {
		/* D_t = α^-t × (Q XOR Σ α^i D_i) */
		uint8_t inv_t = exp[255 - dctx->target_data];

		memcpy(out, dctx->bufs[dctx->q_idx], len);
		for (i = 0; i < data_chunks; i++) {
			uint8_t phys;
			if (i == dctx->target_data) {
				continue;
			}
			phys = poweraid_raid_common_data_to_phys(i, dctx->p_idx,
				      dctx->q_idx, dctx->n);
			if (dctx->bufs[phys] != NULL) {
				poweraid_raid_common_gf8_mul_const_xor(dctx->bufs[phys],
						       exp[i], out, len);
			}
		}
		poweraid_raid_common_gf8_mul_const(out, inv_t, out, len);
	}

	if (rc == 0 && dctx->method == POWERAID_RAID6F_DEG_METHOD_DUAL) {
		void **surviving;

		surviving = calloc(data_chunks, sizeof(*surviving));
		out2 = spdk_dma_malloc(len, 0x1000, NULL);
		if (surviving == NULL || out2 == NULL) {
			rc = -ENOMEM;
		} else {
			for (i = 0; i < data_chunks; i++) {
				uint8_t phys;
				if (i == dctx->missing[0] || i == dctx->missing[1]) {
					continue;
				}
				phys = poweraid_raid_common_data_to_phys(i, dctx->p_idx,
					      dctx->q_idx, dctx->n);
				surviving[i] = dctx->bufs[phys];
			}
			rc = poweraid_raid_common_gf8_decode_2(data_chunks,
					(const void * const *)surviving,
					dctx->bufs[dctx->p_idx],
					dctx->bufs[dctx->q_idx],
					dctx->missing, out, out2, len);
			if (rc != 0) {
				SPDK_ERRLOG("poweraid_raid6f: gf8 decode_2 failed rc=%d"
					    " missing={%u,%u} stripe data chunks=%u\n",
					    rc, dctx->missing[0], dctx->missing[1],
					    data_chunks);
			}
			/* out=missing[0] 恢复值, out2=missing[1] 恢复值 */
			if (rc == 0 && dctx->target_data == dctx->missing[1]) {
				memcpy(out, out2, len);
			}
		}
		free(surviving);
	}

	if (rc == 0) {
		struct iovec src = {
			.iov_base = (uint8_t *)out + dctx->data_off_bytes,
			.iov_len  = dctx->data_len_bytes,
		};
		ssize_t copied = spdk_iovcpy(&src, 1, raid_io->iovs, raid_io->iovcnt);

		if (copied != (ssize_t)dctx->data_len_bytes) {
			SPDK_ERRLOG("poweraid_raid6f: degraded copy short copied=%zd expect=%u\n",
				    copied, dctx->data_len_bytes);
			rc = -EIO;
		}
	}

	for (i = 0; i < dctx->n; i++) {
		if (dctx->bufs[i] != NULL) {
			spdk_dma_free(dctx->bufs[i]);
		}
	}
	free(dctx->bufs);
	if (out != NULL) {
		spdk_dma_free(out);
	}
	if (out2 != NULL) {
		spdk_dma_free(out2);
	}
	free(dctx);

	/* 重建过程中又有存活盘失败（已在 deg_read_cb 标记）：在 RAID6 的
	 * 2 盘故障预算内，用新的故障集合整体重建重试一次。*/
	if (rc == -EIO || rc == -ENODEV) {
		uintptr_t retries = (uintptr_t)raid_io->module_private;

		if (retries < raid->num_parity) {
			int rrc;

			raid_io->module_private = (void *)(retries + 1);
			rrc = poweraid_raid6f_submit_read_request(raid_io);
			if (rrc == 0) {
				return;
			}
			if (rrc == -ENOMEM) {
				raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_NOMEM);
				return;
			}
			/* 故障盘超过 2 等：落到 FAIL 完成 */
		}
	}

	raid_bdev_io_complete(raid_io, rc == 0 ? SPDK_BDEV_IO_STATUS_SUCCESS :
			      rc == -ENOMEM ? SPDK_BDEV_IO_STATUS_NOMEM :
			      SPDK_BDEV_IO_STATUS_FAILED);
}

static void
poweraid_raid6f_deg_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct poweraid_raid6f_deg_subio *subio = cb_arg;
	struct poweraid_raid6f_deg_ctx *dctx = subio->dctx;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		SPDK_WARNLOG("poweraid_raid6f: degraded strip read failed phys=%u, "
			     "mark faulted and escalate\n", subio->phys);
		spdk_dma_free(dctx->bufs[subio->phys]);
		dctx->bufs[subio->phys] = NULL;
		/* 存活盘子读也失败：升级为该槽故障，finish 时整体重建重试一次 */
		poweraid_raid6f_mark_slot_faulted(dctx->raid, dctx->raid_ch,
						  subio->phys);
		dctx->status = -EIO;
	}
	free(subio);

	assert(dctx->remaining > 0);
	if (--dctx->remaining == 0) {
		poweraid_raid6f_deg_finish(dctx);
	}
}

/* 发起对一个 phys 位置整 strip 的读；同步提交失败立即计账。*/
static int
poweraid_raid6f_deg_issue_strip(struct poweraid_raid6f_deg_ctx *dctx, uint8_t phys,
				uint64_t stripe_index)
{
	struct poweraid_raid_common_raid *raid = dctx->raid;
	struct raid_base_bdev_info *base_info = &raid->raid_bdev->base_bdev_info[phys];
	struct spdk_io_channel *base_ch;
	struct poweraid_raid6f_deg_subio *subio;
	uint64_t offset;
	int rc;

	base_ch = raid_bdev_channel_get_base_channel(dctx->raid_ch, phys);
	if (base_ch == NULL || base_info->desc == NULL) {
		return -ENODEV;
	}

	dctx->bufs[phys] = spdk_dma_malloc(dctx->strip_bytes, 0x1000, NULL);
	if (dctx->bufs[phys] == NULL) {
		return -ENOMEM;
	}

	subio = calloc(1, sizeof(*subio));
	if (subio == NULL) {
		spdk_dma_free(dctx->bufs[phys]);
		dctx->bufs[phys] = NULL;
		return -ENOMEM;
	}
	subio->dctx = dctx;
	subio->phys = phys;

	offset = raid->data_offset_blocks + stripe_index * raid->strip_size;
	rc = spdk_bdev_read_blocks(base_info->desc, base_ch, dctx->bufs[phys],
				  offset, raid->strip_size,
				  poweraid_raid6f_deg_read_cb, subio);
	if (rc != 0) {
		SPDK_ERRLOG("poweraid_raid6f: degraded strip read submit failed"
			    " phys=%u stripe=%"PRIu64" rc=%d\n",
			    phys, stripe_index, rc);
		spdk_dma_free(dctx->bufs[phys]);
		dctx->bufs[phys] = NULL;
		free(subio);
	}
	return rc;
}

/* target 所在盘已确认故障，按 stripe 故障角色调度重建读。*/
int
poweraid_raid6f_submit_degraded_read(struct raid_bdev_io *raid_io,
				     uint64_t stripe_index,
				     uint8_t p_idx, uint8_t q_idx,
				     uint8_t target_phys, uint8_t target_data,
				     uint64_t chunk_offset_blocks)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;
	uint32_t data_chunks = raid->num_base_bdevs - raid->num_parity;
	struct poweraid_raid6f_deg_ctx *dctx;
	uint8_t i, n_failed = 0;
	uint8_t n_missing = 0;
	bool p_dead = false;
	uint8_t d;
	int rc;

	poweraid_raid6f_refresh_degraded_state(raid, raid_io->raid_ch);

	/* 第一遍：统计故障盘数（超过 2 直接拒绝；框架本应已 deconfigure）*/
	for (i = 0; i < raid->num_base_bdevs; i++) {
		if (poweraid_raid6f_slot_faulted(raid, raid_io->raid_ch, i)) {
			n_failed++;
		}
	}

	if (n_failed > raid->num_parity) {
		SPDK_ERRLOG("poweraid_raid6f: %u failed bdevs > parity %u, cannot read\n",
			    n_failed, raid->num_parity);
		return -EIO;
	}

	dctx = calloc(1, sizeof(*dctx));
	if (dctx == NULL) {
		return -ENOMEM;
	}
	dctx->raid_io = raid_io;
	dctx->raid = raid;
	dctx->raid_ch = raid_io->raid_ch;
	dctx->n = raid->num_base_bdevs;
	dctx->p_idx = p_idx;
	dctx->q_idx = q_idx;
	dctx->target_data = target_data;
	dctx->strip_bytes = raid->strip_size * raid->block_size;
	dctx->data_off_bytes = chunk_offset_blocks * raid->block_size;
	dctx->data_len_bytes = raid_io->num_blocks * raid->block_size;

	dctx->bufs = calloc(dctx->n, sizeof(*dctx->bufs));
	if (dctx->bufs == NULL) {
		free(dctx);
		return -ENOMEM;
	}

	/* 第二遍：按 stripe 角色分类故障盘（P / Q / data 序号）*/
	for (i = 0; i < raid->num_base_bdevs; i++) {
		uint8_t dd;

		if (!poweraid_raid6f_slot_faulted(raid, raid_io->raid_ch, i)) {
			continue;
		}
		if (i == p_idx) {
			p_dead = true;
		} else if (i == q_idx) {
			/* Q 故障不影响方法选择标记（P 法可用时优先 P 法）*/
		} else {
			dd = poweraid_raid_common_phys_to_data(i, p_idx, q_idx);
			if (n_missing < 2) {
				dctx->missing[n_missing++] = dd;
			}
		}
	}

	/* 选择重建方法 */
	if (n_missing == 2) {
		dctx->method = POWERAID_RAID6F_DEG_METHOD_DUAL;
		/* gf8_decode_2 要求 missing[0] < missing[1] */
		if (dctx->missing[0] > dctx->missing[1]) {
			uint8_t tmp = dctx->missing[0];
			dctx->missing[0] = dctx->missing[1];
			dctx->missing[1] = tmp;
		}
	} else if (!p_dead) {
		dctx->method = POWERAID_RAID6F_DEG_METHOD_P_XOR;
	} else {
		dctx->method = POWERAID_RAID6F_DEG_METHOD_Q_SINGLE;
	}

	SPDK_DEBUGLOG(poweraid_raid6f,
		      "degraded read stripe=%"PRIu64
		      " target=data%u/phys%u failed=%u method=%s\n",
		      stripe_index, target_data, target_phys, n_failed,
		      dctx->method == POWERAID_RAID6F_DEG_METHOD_P_XOR ? "P-xor" :
		      dctx->method == POWERAID_RAID6F_DEG_METHOD_Q_SINGLE ? "Q-single" :
		      "dual-decode");

	/* 下发需要的整 strip 读 */
	if (dctx->method == POWERAID_RAID6F_DEG_METHOD_DUAL) {
		rc = poweraid_raid6f_deg_issue_strip(dctx, p_idx, stripe_index);
		if (rc == 0) {
			dctx->remaining++;
		} else {
			dctx->status = rc;
	}
		rc = poweraid_raid6f_deg_issue_strip(dctx, q_idx, stripe_index);
		if (rc == 0) {
			dctx->remaining++;
		} else {
			dctx->status = rc;
	}
	} else if (dctx->method == POWERAID_RAID6F_DEG_METHOD_P_XOR) {
		rc = poweraid_raid6f_deg_issue_strip(dctx, p_idx, stripe_index);
		if (rc == 0) {
			dctx->remaining++;
		} else {
			dctx->status = rc;
	}
	} else {
		rc = poweraid_raid6f_deg_issue_strip(dctx, q_idx, stripe_index);
		if (rc == 0) {
			dctx->remaining++;
		} else {
			dctx->status = rc;
	}
	}

	for (d = 0; d < data_chunks; d++) {
		uint8_t phys;
		bool need;

		if (dctx->method == POWERAID_RAID6F_DEG_METHOD_DUAL) {
			need = (d != dctx->missing[0] && d != dctx->missing[1]);
		} else {
			need = (d != target_data);
		}
		if (!need) {
			continue;
		}
		phys = poweraid_raid_common_data_to_phys(d, p_idx, q_idx,
						      dctx->n);
		rc = poweraid_raid6f_deg_issue_strip(dctx, phys, stripe_index);
		if (rc == 0) {
			dctx->remaining++;
		} else {
			dctx->status = rc;
	}
	}

	if (dctx->remaining == 0) {
		poweraid_raid6f_deg_finish(dctx);
	}
	return 0;
}

/* ===== 重建引擎：整条 strip 恢复（目标槽可为 data/P/Q，允许另一盘同故障）===== */

/* 恢复方法（data 目标复用 DEG_METHOD_*；P/Q 目标新增）*/
#define RECOVER_METHOD_P_XOR_ALL	3	/* 目标=P：XOR 全部 data */
#define RECOVER_METHOD_Q_SUM		4	/* 目标=Q：Σ α^i D_i */
#define RECOVER_METHOD_P_WITH_D		5	/* 目标=P 且另一 data d 故障 */
#define RECOVER_METHOD_Q_WITH_D		6	/* 目标=Q 且另一 data d 故障 */

struct poweraid_raid6f_recover_ctx {
	struct poweraid_raid_common_raid		*raid;
	struct raid_bdev_io_channel		*raid_ch;
	uint64_t				stripe_index;
	uint8_t					n;
	uint8_t					p_idx;
	uint8_t					q_idx;
	uint8_t					target_phys;
	uint8_t					target_data;	/* 目标为 data 时有效 */
	uint8_t					missing[2];	/* 故障 data 序号（升序）*/
	uint8_t					n_missing_data;
	bool					miss_p;
	bool					miss_q;
	uint8_t					method;
	uint32_t				strip_bytes;
	void					*out_buf;
	void					*tmp_buf;	/* DUAL/Q|P_WITH_D 用 */
	void					**bufs;		/* phys 索引的读缓冲 */
	uint32_t				remaining;
	int					status;
	void					(*cb)(int status, void *cb_arg);
	void					*cb_arg;
};

struct recover_subio {
	struct poweraid_raid6f_recover_ctx	*rctx;
	uint8_t					phys;
	struct spdk_bdev_io_wait_entry		wait_entry;
};

static void poweraid_raid6f_recover_finish(void *arg);

static void
recover_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct recover_subio *subio = cb_arg;
	struct poweraid_raid6f_recover_ctx *rctx = subio->rctx;

	spdk_bdev_free_io(bdev_io);
	if (!success) {
		SPDK_ERRLOG("poweraid_raid6f: rebuild strip read failed phys=%u "
			    "stripe=%"PRIu64"\n", subio->phys, rctx->stripe_index);
		rctx->status = -EIO;
	}
	free(subio);

	assert(rctx->remaining > 0);
	if (--rctx->remaining == 0) {
		poweraid_raid6f_recover_finish(rctx);
	}
}

static void
recover_read_wait_cb(void *arg);

/* 发起一个整 strip 读；ENOMEM 走 bdev io wait 自动重试。返回 0 已计 remaining（或排队），
 * 负值表示该条不可用（调用方计 status）。*/
static int
recover_issue_strip(struct poweraid_raid6f_recover_ctx *rctx, uint8_t phys)
{
	struct poweraid_raid_common_raid *raid = rctx->raid;
	struct raid_base_bdev_info *base_info = &raid->raid_bdev->base_bdev_info[phys];
	struct spdk_io_channel *base_ch;
	struct recover_subio *subio;
	uint64_t offset;
	int rc;

	base_ch = raid_bdev_channel_get_base_channel(rctx->raid_ch, phys);
	if (base_ch == NULL || base_info->desc == NULL) {
		return -ENODEV;
	}

	rctx->bufs[phys] = spdk_dma_malloc(rctx->strip_bytes, 0x1000, NULL);
	if (rctx->bufs[phys] == NULL) {
		return -ENOMEM;
	}

	subio = calloc(1, sizeof(*subio));
	if (subio == NULL) {
		spdk_dma_free(rctx->bufs[phys]);
		rctx->bufs[phys] = NULL;
		return -ENOMEM;
	}
	subio->rctx = rctx;
	subio->phys = phys;

	offset = raid->data_offset_blocks + rctx->stripe_index * raid->strip_size;
	rc = spdk_bdev_read_blocks(base_info->desc, base_ch, rctx->bufs[phys],
				  offset, raid->strip_size,
				  recover_read_cb, subio);
	if (rc == -ENOMEM) {
		subio->wait_entry.bdev = spdk_bdev_desc_get_bdev(base_info->desc);
		subio->wait_entry.cb_fn = (spdk_bdev_io_wait_cb)recover_read_wait_cb;
		subio->wait_entry.cb_arg = subio;
		spdk_bdev_queue_io_wait(subio->wait_entry.bdev, base_ch,
					&subio->wait_entry);
		return 0;
	}
	if (rc != 0) {
		SPDK_ERRLOG("poweraid_raid6f: rebuild read submit phys=%u rc=%d\n",
			    phys, rc);
		spdk_dma_free(rctx->bufs[phys]);
		rctx->bufs[phys] = NULL;
		free(subio);
	}
	return rc;
}

static void
recover_read_wait_cb(void *arg)
{
	struct recover_subio *subio = arg;
	struct poweraid_raid6f_recover_ctx *rctx = subio->rctx;
	struct poweraid_raid_common_raid *raid = rctx->raid;
	struct raid_base_bdev_info *base_info =
		&raid->raid_bdev->base_bdev_info[subio->phys];
	struct spdk_io_channel *base_ch;
	int rc;

	base_ch = raid_bdev_channel_get_base_channel(rctx->raid_ch, subio->phys);
	rc = spdk_bdev_read_blocks(base_info->desc, base_ch, rctx->bufs[subio->phys],
				   raid->data_offset_blocks +
				   rctx->stripe_index * raid->strip_size,
				   raid->strip_size,
				   recover_read_cb, subio);
	if (rc == -ENOMEM) {
		subio->wait_entry.bdev = spdk_bdev_desc_get_bdev(base_info->desc);
		subio->wait_entry.cb_fn = (spdk_bdev_io_wait_cb)recover_read_wait_cb;
		subio->wait_entry.cb_arg = subio;
		spdk_bdev_queue_io_wait(subio->wait_entry.bdev, base_ch,
					&subio->wait_entry);
		return;
	}
	if (rc != 0) {
		spdk_dma_free(rctx->bufs[subio->phys]);
		rctx->bufs[subio->phys] = NULL;
		free(subio);
		rctx->status = rc;
		assert(rctx->remaining > 0);
		if (--rctx->remaining == 0) {
			poweraid_raid6f_recover_finish(rctx);
		}
	}
}

static void
poweraid_raid6f_recover_finish(void *arg)
{
	struct poweraid_raid6f_recover_ctx *rctx = arg;
	struct poweraid_raid_common_raid *raid = rctx->raid;
	uint32_t data_chunks = raid->num_base_bdevs - raid->num_parity;
	uint32_t len = rctx->strip_bytes;
	const uint8_t *exp = poweraid_raid_common_gf8_exp;
	uint8_t i;
	int rc = rctx->status;
	void (*cb)(int, void *) = rctx->cb;
	void *cb_arg = rctx->cb_arg;

	/* 计算（rc==0 时 bufs 必然齐）*/
	if (rc == 0) {
		switch (rctx->method) {
		case POWERAID_RAID6F_DEG_METHOD_P_XOR:
			/* 目标 data t：D_t = P XOR 其他存活 data */
			memcpy(rctx->out_buf, rctx->bufs[rctx->p_idx], len);
			for (i = 0; i < data_chunks; i++) {
				uint8_t phys;
				if (i == rctx->target_data) {
					continue;
				}
				phys = poweraid_raid_common_data_to_phys(i,
						rctx->p_idx, rctx->q_idx, rctx->n);
				if (rctx->bufs[phys] != NULL) {
					poweraid_raid_common_gf8_mul_const_xor(
						rctx->bufs[phys], 1,
						rctx->out_buf, len);
				}
			}
			break;

		case POWERAID_RAID6F_DEG_METHOD_Q_SINGLE: {
			uint8_t inv_t = exp[255 - rctx->target_data];

			memcpy(rctx->out_buf, rctx->bufs[rctx->q_idx], len);
			for (i = 0; i < data_chunks; i++) {
				uint8_t phys;
				if (i == rctx->target_data) {
					continue;
				}
				phys = poweraid_raid_common_data_to_phys(i,
						rctx->p_idx, rctx->q_idx, rctx->n);
				if (rctx->bufs[phys] != NULL) {
					poweraid_raid_common_gf8_mul_const_xor(
						rctx->bufs[phys], exp[i],
						rctx->out_buf, len);
				}
			}
			poweraid_raid_common_gf8_mul_const(rctx->out_buf, inv_t,
							    rctx->out_buf, len);
			break;
		}

		case POWERAID_RAID6F_DEG_METHOD_DUAL: {
			void **surviving = calloc(data_chunks, sizeof(*surviving));

			if (surviving == NULL) {
				rc = -ENOMEM;
				break;
			}
			for (i = 0; i < data_chunks; i++) {
				uint8_t phys;
				if (i == rctx->missing[0] || i == rctx->missing[1]) {
					continue;
				}
				phys = poweraid_raid_common_data_to_phys(i,
						rctx->p_idx, rctx->q_idx, rctx->n);
				surviving[i] = rctx->bufs[phys];
			}
			/* missing 升序；目标排第二时把 out2 换为 out_buf */
			if (rctx->target_data == rctx->missing[0]) {
				rc = poweraid_raid_common_gf8_decode_2(
						data_chunks,
						(const void * const *)surviving,
						rctx->bufs[rctx->p_idx],
						rctx->bufs[rctx->q_idx],
						rctx->missing,
						rctx->out_buf, rctx->tmp_buf, len);
			} else {
				rc = poweraid_raid_common_gf8_decode_2(
						data_chunks,
						(const void * const *)surviving,
						rctx->bufs[rctx->p_idx],
						rctx->bufs[rctx->q_idx],
						rctx->missing,
						rctx->tmp_buf, rctx->out_buf, len);
			}
			free(surviving);
			if (rc != 0) {
				SPDK_ERRLOG("poweraid_raid6f: rebuild dual decode "
					    "failed rc=%d missing={%u,%u}\n",
					    rc, rctx->missing[0], rctx->missing[1]);
			}
			break;
		}

		case RECOVER_METHOD_P_XOR_ALL:
			memset(rctx->out_buf, 0, len);
			for (i = 0; i < data_chunks; i++) {
				uint8_t phys = poweraid_raid_common_data_to_phys(i,
						rctx->p_idx, rctx->q_idx, rctx->n);
				poweraid_raid_common_gf8_mul_const_xor(
					rctx->bufs[phys], 1, rctx->out_buf, len);
			}
			break;

		case RECOVER_METHOD_Q_SUM:
			memset(rctx->out_buf, 0, len);
			for (i = 0; i < data_chunks; i++) {
				uint8_t phys = poweraid_raid_common_data_to_phys(i,
						rctx->p_idx, rctx->q_idx, rctx->n);
				poweraid_raid_common_gf8_mul_const_xor(
					rctx->bufs[phys], exp[i], rctx->out_buf, len);
			}
			break;

		case RECOVER_METHOD_P_WITH_D: {
			/* 另一 data d 故障：先用 Q 单缺法恢复 d 到 tmp，再 XOR 全部 data */
			uint8_t d = rctx->missing[0];
			uint8_t dphys;

			memcpy(rctx->tmp_buf, rctx->bufs[rctx->q_idx], len);
			for (i = 0; i < data_chunks; i++) {
				uint8_t phys;
				if (i == d) {
					continue;
				}
				phys = poweraid_raid_common_data_to_phys(i,
						rctx->p_idx, rctx->q_idx, rctx->n);
				poweraid_raid_common_gf8_mul_const_xor(
					rctx->bufs[phys], exp[i], rctx->tmp_buf, len);
			}
			poweraid_raid_common_gf8_mul_const(rctx->tmp_buf,
				poweraid_raid_common_gf8_exp[255 - d],
				rctx->tmp_buf, len);

			memcpy(rctx->out_buf, rctx->tmp_buf, len);
			for (i = 0; i < data_chunks; i++) {
				if (i == d) {
					continue;
				}
				dphys = poweraid_raid_common_data_to_phys(i,
						rctx->p_idx, rctx->q_idx, rctx->n);
				poweraid_raid_common_gf8_mul_const_xor(
					rctx->bufs[dphys], 1, rctx->out_buf, len);
			}
			break;
		}

		case RECOVER_METHOD_Q_WITH_D: {
			/* 另一 data d 故障：先用 P 恢复 d 到 tmp，再做加权 Σ α^i D_i */
			uint8_t d = rctx->missing[0];

			memcpy(rctx->tmp_buf, rctx->bufs[rctx->p_idx], len);
			for (i = 0; i < data_chunks; i++) {
				uint8_t phys;
				if (i == d) {
					continue;
				}
				phys = poweraid_raid_common_data_to_phys(i,
						rctx->p_idx, rctx->q_idx, rctx->n);
				poweraid_raid_common_gf8_mul_const_xor(
					rctx->bufs[phys], 1, rctx->tmp_buf, len);
			}

			memset(rctx->out_buf, 0, len);
			for (i = 0; i < data_chunks; i++) {
				void *src;
				uint8_t phys = poweraid_raid_common_data_to_phys(i,
						rctx->p_idx, rctx->q_idx, rctx->n);
				if (i == d) {
					src = rctx->tmp_buf;
				} else {
					src = rctx->bufs[phys];
				}
				poweraid_raid_common_gf8_mul_const_xor(
					src, exp[i], rctx->out_buf, len);
			}
			break;
		}

		default:
			SPDK_ERRLOG("poweraid_raid6f: recover method %u unknown\n",
				    rctx->method);
			rc = -EINVAL;
			break;
		}
	}

	for (i = 0; i < rctx->n; i++) {
		if (rctx->bufs[i] != NULL) {
			spdk_dma_free(rctx->bufs[i]);
		}
	}
	free(rctx->bufs);
	if (rctx->tmp_buf != NULL) {
		spdk_dma_free(rctx->tmp_buf);
	}
	free(rctx);

	cb(rc, cb_arg);
}

int
poweraid_raid6f_recover_strip(struct poweraid_raid_common_raid *raid,
			      struct raid_bdev_io_channel *raid_ch,
			      uint64_t stripe_index, uint8_t target_phys,
			      void *out_buf, uint32_t strip_len,
			      void (*cb)(int status, void *cb_arg),
			      void *cb_arg)
{
	struct poweraid_raid6f_recover_ctx *rctx;
	struct raid_base_bdev_info *base_info;
	uint8_t i, n_fault = 0;
	uint8_t d;
	bool need_tmp;
	int rc;

	if (target_phys >= raid->num_base_bdevs ||
	    strip_len != raid->strip_size * raid->block_size) {
		return -EINVAL;
	}

	rctx = calloc(1, sizeof(*rctx));
	if (rctx == NULL) {
		return -ENOMEM;
	}
	rctx->raid = raid;
	rctx->raid_ch = raid_ch;
	rctx->stripe_index = stripe_index;
	rctx->n = raid->num_base_bdevs;
	rctx->target_phys = target_phys;
	rctx->strip_bytes = strip_len;
	rctx->out_buf = out_buf;
	rctx->cb = cb;
	rctx->cb_arg = cb_arg;

	poweraid_raid_common_get_parity_idx(raid, stripe_index,
					    &rctx->p_idx, &rctx->q_idx);

	rctx->bufs = calloc(rctx->n, sizeof(*rctx->bufs));
	if (rctx->bufs == NULL) {
		free(rctx);
		return -ENOMEM;
	}

	/* 故障集合：重建目标 + 其他已移除（desc==NULL）的槽位。
	 * 重建期间目标 cbdev 带 FAULTED 位但 desc 已打开（新盘空白），不可读。*/
	for (i = 0; i < rctx->n; i++) {
		base_info = &raid->raid_bdev->base_bdev_info[i];
		bool fault = (i == target_phys) || (base_info->desc == NULL);

		if (!fault) {
			continue;
		}
		n_fault++;
		if (i == rctx->p_idx) {
			rctx->miss_p = true;
		} else if (i == rctx->q_idx) {
			rctx->miss_q = true;
		} else if (rctx->n_missing_data < 2) {
			rctx->missing[rctx->n_missing_data++] =
				poweraid_raid_common_phys_to_data(i,
					rctx->p_idx, rctx->q_idx);
		}
	}
	if (n_fault > raid->num_parity) {
		SPDK_ERRLOG("poweraid_raid6f: rebuild stripe=%"PRIu64
			    " %u faults > 2\n", stripe_index, n_fault);
		free(rctx->bufs);
		free(rctx);
		return -EIO;
	}

	/* 角色与方法选择 */
	if (target_phys == rctx->p_idx) {
		rctx->method = (rctx->n_missing_data == 1) ?
			       RECOVER_METHOD_P_WITH_D : RECOVER_METHOD_P_XOR_ALL;
	} else if (target_phys == rctx->q_idx) {
		rctx->method = (rctx->n_missing_data == 1) ?
			       RECOVER_METHOD_Q_WITH_D : RECOVER_METHOD_Q_SUM;
	} else {
		rctx->target_data = poweraid_raid_common_phys_to_data(target_phys,
					rctx->p_idx, rctx->q_idx);
		if (rctx->n_missing_data == 2) {
			rctx->method = POWERAID_RAID6F_DEG_METHOD_DUAL;
			if (rctx->missing[0] > rctx->missing[1]) {
				uint8_t t = rctx->missing[0];
				rctx->missing[0] = rctx->missing[1];
				rctx->missing[1] = t;
			}
		} else if (!rctx->miss_p) {
			rctx->method = POWERAID_RAID6F_DEG_METHOD_P_XOR;
		} else {
			rctx->method = POWERAID_RAID6F_DEG_METHOD_Q_SINGLE;
		}
	}

	need_tmp = (rctx->method == POWERAID_RAID6F_DEG_METHOD_DUAL ||
		    rctx->method == RECOVER_METHOD_P_WITH_D ||
		    rctx->method == RECOVER_METHOD_Q_WITH_D);
	if (need_tmp) {
		rctx->tmp_buf = spdk_dma_malloc(strip_len, 0x1000, NULL);
		if (rctx->tmp_buf == NULL) {
			free(rctx->bufs);
			free(rctx);
			return -ENOMEM;
		}
	}

	SPDK_DEBUGLOG(poweraid_raid6f,
		      "rebuild stripe=%"PRIu64" target=phys%u faults=%u method=%u\n",
		      stripe_index, target_phys, n_fault, rctx->method);

	/* 下发整 strip 读 */
	if (target_phys != rctx->p_idx &&
	    (rctx->method == POWERAID_RAID6F_DEG_METHOD_P_XOR ||
	     rctx->method == POWERAID_RAID6F_DEG_METHOD_DUAL ||
	     rctx->method == RECOVER_METHOD_Q_WITH_D)) {
		rc = recover_issue_strip(rctx, rctx->p_idx);
		if (rc == 0) {
			rctx->remaining++;
		} else {
			rctx->status = rc;
		}
	}
	if (target_phys != rctx->q_idx &&
	    (rctx->method == POWERAID_RAID6F_DEG_METHOD_Q_SINGLE ||
	     rctx->method == POWERAID_RAID6F_DEG_METHOD_DUAL ||
	     rctx->method == RECOVER_METHOD_P_WITH_D)) {
		rc = recover_issue_strip(rctx, rctx->q_idx);
		if (rc == 0) {
			rctx->remaining++;
		} else {
			rctx->status = rc;
		}
	}

	{
		uint32_t data_chunks = raid->num_base_bdevs - raid->num_parity;

		for (d = 0; d < data_chunks; d++) {
			uint8_t phys;
			bool need;

			if (rctx->method == POWERAID_RAID6F_DEG_METHOD_DUAL) {
				need = (d != rctx->missing[0] && d != rctx->missing[1]);
			} else if (rctx->method == POWERAID_RAID6F_DEG_METHOD_P_XOR ||
				   rctx->method == POWERAID_RAID6F_DEG_METHOD_Q_SINGLE) {
				need = (d != rctx->target_data);
			} else if (rctx->method == RECOVER_METHOD_P_WITH_D ||
				   rctx->method == RECOVER_METHOD_Q_WITH_D) {
				need = (d != rctx->missing[0]);
			} else {
				/* P_XOR_ALL / Q_SUM：全部 data */
				need = true;
			}
			if (!need) {
				continue;
			}
			phys = poweraid_raid_common_data_to_phys(d, rctx->p_idx,
					rctx->q_idx, rctx->n);
			rc = recover_issue_strip(rctx, phys);
			if (rc == 0) {
				rctx->remaining++;
			} else {
				rctx->status = rc;
			}
		}
	}

	if (rctx->remaining == 0) {
		poweraid_raid6f_recover_finish(rctx);
	}
	return 0;
}
