/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   poweraid_raid6f CALC 实现：P (XOR) + Q (GF8 Reed-Solomon 编码)
 *
 *   注册到 raid->ops.calc_parity，由 common 层 REQ CALC handler 调用。
 *   - P: 优先 spdk_accel_submit_xor 异步；失败 fallback 同步 XOR。
 *   - Q: poweraid_raid_common_gf8_encode 同步计算（SIMD 加速）。
 *   完成后调用 cb(req, status)。
 */

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/log.h"
#include "spdk/accel.h"

#include "../poweraid_raid_common/poweraid_raid_common.h"
#include "../poweraid_raid_common/poweraid_raid_common_gf8.h"
#include "poweraid_raid6f.h"

SPDK_LOG_REGISTER_COMPONENT(poweraid_raid6f_calc)

/* accel XOR 完成回调：Q 已同步算完，此处通知用户 */
static void
poweraid_raid6f_xor_accel_cb(void *cb_arg, int status)
{
	struct poweraid_raid_common_req *req = cb_arg;
	poweraid_raid_calc_cb_t user_cb = req->xor.cb;

	if (status != 0) {
		SPDK_WARNLOG("poweraid_raid6f: accel xor failed (%d), sync fallback\n",
			     status);
		poweraid_raid_common_req_xor_sync(req);
	}
	req->xor.cb = NULL;
	if (user_cb != NULL) {
		user_cb(req, 0);
	}
}

/* 同步计算 Q = Σ α^i × D_i（GF8 encode）。Q 缓冲为 req->q_buf。*/
static int
poweraid_raid6f_calc_q(struct poweraid_raid_common_req *req)
{
	int rc;

	if (req->q_buf == NULL || req->src_bufs == NULL || req->n_src < 2) {
		SPDK_ERRLOG("poweraid_raid6f: calc_q bad args (q_buf=%p n_src=%u)\n",
			    req->q_buf, req->n_src);
		return -EINVAL;
	}

	rc = poweraid_raid_common_gf8_encode(req->n_src,
					     (const void * const *)req->src_bufs,
					     req->q_buf, req->xor_len);
	if (rc != 0) {
		SPDK_ERRLOG("poweraid_raid6f: gf8_encode failed rc=%d\n", rc);
	}
	return rc;
}

void
poweraid_raid6f_calc_parity(struct poweraid_raid_common_req *req,
			    poweraid_raid_calc_cb_t cb)
{
	struct spdk_io_channel *accel_ch;
	int rc;

	req->xor.cb = cb;

	/* Q: 同步 GF8 编码（SIMD 加速，耗时与 strip 大小线性相关）*/
	rc = poweraid_raid6f_calc_q(req);
	if (rc != 0) {
		req->xor.cb = NULL;
		cb(req, rc);
		return;
	}

	/* P: accel 异步 XOR */
	accel_ch = req->ch ? req->ch->accel_ch : NULL;
	if (accel_ch == NULL) {
		SPDK_WARNLOG("poweraid_raid6f: no accel_ch, sync xor\n");
		poweraid_raid_common_req_xor_sync(req);
		req->xor.cb = NULL;
		cb(req, 0);
		return;
	}

	rc = spdk_accel_submit_xor(accel_ch, req->parity_buf, req->src_bufs,
				   req->n_src, req->xor_len,
				   poweraid_raid6f_xor_accel_cb, req);
	if (rc == 0) {
		return;  /* 异步完成，等 accel cb */
	}

	SPDK_WARNLOG("poweraid_raid6f: submit_xor rc=%d, sync fallback\n", rc);
	poweraid_raid_common_req_xor_sync(req);
	req->xor.cb = NULL;
	cb(req, 0);
}
