/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   GF(2^8) 软件参考实现（Stage 4 RAID6 Q 校验）
 *
 *   多项式 0x11d，生成元 α = 2。与 Linux mdadm / ISA-L RAID6 兼容。
 *
 *   本文件提供:
 *     1. log/antilog 表（运行时生成，避免手写错误）
 *     2. 标量乘法函数（正确性基准 + fallback）
 *     3. encode / decode_2 入口（分派到 SIMD 或标量）
 *
 *   SIMD 路径在 poweraid_raid5f_gf8_x86.c / _neon.c 中实现，
 *   通过函数指针注册。无 SIMD 时 fallback 到本文件标量实现。
 */

#include "spdk/stdinc.h"
#include "spdk/log.h"

#include "poweraid_raid5f_gf8.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_gf8);

/* ===== GF(2^8) 对数表 / 反对数表 =====
 *
 * 多项式 0x11d，生成元 α = 2。运行时生成（gf8_init 调用）。
 * exp[i] = α^(i mod 255), i = 0..511（扩展到 512 避免 mod 255 开销）
 * log[x] = log_α(x), x = 1..255; log[0] = 0（约定）
 */

#define GF8_POLY 0x11d
#define GF8_GEN  2  /* α = 2, primitive element */

uint8_t poweraid_raid5f_gf8_log[256];
uint8_t poweraid_raid5f_gf8_exp[512];

/* ===== 表生成 + 验证 ===== */

static void
gf8_generate_tables(void)
{
	uint8_t val = 1;
	int i;

	/* 生成 exp[0..254] 和 log[1..255] */
	for (i = 0; i < 255; i++) {
		poweraid_raid5f_gf8_exp[i] = val;
		poweraid_raid5f_gf8_log[val] = (uint8_t)i;
		/* val = val × α = val × 2 mod poly(0x11d) */
		val = (val << 1) ^ ((val & 0x80) ? GF8_POLY : 0);
	}
	/* exp[255] = α^255 = α^0 = 1 */
	poweraid_raid5f_gf8_exp[255] = 1;

	/* 扩展 exp[256..511] = exp[0..255]（避免运行时 mod 255）*/
	for (i = 256; i < 512; i++) {
		poweraid_raid5f_gf8_exp[i] = poweraid_raid5f_gf8_exp[i - 255];
	}

	/* log[0] 约定为 0（实际未定义，调用方需检查零元）*/
	poweraid_raid5f_gf8_log[0] = 0;
}

static int
gf8_verify_tables(void)
{
	uint8_t val = 1;
	int i;

	for (i = 0; i < 255; i++) {
		if (poweraid_raid5f_gf8_exp[i] != val) {
			SPDK_ERRLOG("GF8 exp table mismatch at i=%d: got 0x%02x expected 0x%02x\n",
				    i, poweraid_raid5f_gf8_exp[i], val);
			return -EINVAL;
		}
		if (poweraid_raid5f_gf8_log[val] != (uint8_t)i) {
			SPDK_ERRLOG("GF8 log table mismatch at val=0x%02x: got %d expected %d\n",
				    val, poweraid_raid5f_gf8_log[val], i);
			return -EINVAL;
		}
		val = (val << 1) ^ ((val & 0x80) ? GF8_POLY : 0);
	}

	if (poweraid_raid5f_gf8_exp[255] != 1) {
		SPDK_ERRLOG("GF8 exp[255]=0x%02x, expected 0x01\n", poweraid_raid5f_gf8_exp[255]);
		return -EINVAL;
	}

	/* 验证 exp[255+i] == exp[i] for i=0..255
	 * （因为 α^(255+i) = α^i，扩展表从 255 开始重复）*/
	for (i = 0; i < 256; i++) {
		if (poweraid_raid5f_gf8_exp[255 + i] != poweraid_raid5f_gf8_exp[i]) {
			SPDK_ERRLOG("GF8 exp[255+%d]=0x%02x != exp[%d]=0x%02x\n",
				    i, poweraid_raid5f_gf8_exp[255 + i],
				    i, poweraid_raid5f_gf8_exp[i]);
			return -EINVAL;
		}
	}

	return 0;
}

/* ===== SIMD 函数指针（由 gf8_x86.c 或 gf8_neon.c 注册）===== */

static poweraid_raid5f_gf8_mul_const_fn     g_mul_const     = NULL;
static poweraid_raid5f_gf8_mul_const_xor_fn g_mul_const_xor = NULL;
static const char *g_impl_name = "scalar";

/* ===== 软件标量实现 ===== */

static void
gf8_mul_const_scalar(const uint8_t *src, uint8_t constant, uint8_t *dst, uint64_t len)
{
	uint64_t i;
	const uint8_t *log = poweraid_raid5f_gf8_log;
	const uint8_t *exp = poweraid_raid5f_gf8_exp;

	if (constant == 0) {
		memset(dst, 0, len);
		return;
	}
	if (constant == 1) {
		if (dst != src) {
			memcpy(dst, src, len);
		}
		return;
	}

	/* 常量查找表（每次调用构建一次，避免对每个元素查 log）*/
	uint8_t mul_lut[256];
	uint8_t lc = log[constant];
	for (i = 0; i < 256; i++) {
		mul_lut[i] = (i == 0) ? 0 : exp[lc + log[i]];
	}
	for (i = 0; i < len; i++) {
		dst[i] = mul_lut[src[i]];
	}
}

static void
gf8_mul_const_xor_scalar(const uint8_t *src, uint8_t constant, uint8_t *dst, uint64_t len)
{
	uint64_t i;
	const uint8_t *log = poweraid_raid5f_gf8_log;
	const uint8_t *exp = poweraid_raid5f_gf8_exp;

	if (constant == 0) {
		return;
	}
	if (constant == 1) {
		for (i = 0; i < len; i++) {
			dst[i] ^= src[i];
		}
		return;
	}

	uint8_t mul_lut[256];
	uint8_t lc = log[constant];
	for (i = 0; i < 256; i++) {
		mul_lut[i] = (i == 0) ? 0 : exp[lc + log[i]];
	}
	for (i = 0; i < len; i++) {
		dst[i] ^= mul_lut[src[i]];
	}
}

/* ===== 公共 API 实现 ===== */

int
poweraid_raid5f_gf8_init(void)
{
	int rc;

	gf8_generate_tables();

	rc = gf8_verify_tables();
	if (rc != 0) {
		SPDK_ERRLOG("GF8 table verification failed: %d\n", rc);
		return rc;
	}

	g_mul_const     = gf8_mul_const_scalar;
	g_mul_const_xor = gf8_mul_const_xor_scalar;
	g_impl_name     = "scalar";

	SPDK_NOTICELOG("GF8 engine initialized (scalar fallback)\n");

	/* 让 SIMD 实现覆盖函数指针（如果有）*/
	poweraid_raid5f_gf8_try_select_simd(&g_mul_const, &g_mul_const_xor, &g_impl_name);

	SPDK_NOTICELOG("GF8 engine using %s path\n", g_impl_name);
	return 0;
}

void
poweraid_raid5f_gf8_cleanup(void)
{
	g_mul_const     = NULL;
	g_mul_const_xor = NULL;
	g_impl_name     = "uninitialized";
}

const char *
poweraid_raid5f_gf8_impl_name(void)
{
	return g_impl_name;
}

int
poweraid_raid5f_gf8_encode(uint32_t data_chunks,
			   const void * const *data_bufs,
			   void *q_buf,
			   uint64_t len)
{
	uint32_t i;
	const uint8_t *exp = poweraid_raid5f_gf8_exp;

	if (data_chunks < 2 || data_bufs == NULL || q_buf == NULL || len == 0) {
		return -EINVAL;
	}

	/* Q = Σ α^i × D_i, i = 0..N-1
	 * 第一块 (i=0): α^0 = 1 → Q = D_0
	 * 后续块: Q ^= α^i × D_i
	 */
	memcpy(q_buf, data_bufs[0], len);

	for (i = 1; i < data_chunks; i++) {
		uint8_t coeff = exp[i];  /* α^i */
		g_mul_const_xor(data_bufs[i], coeff, q_buf, len);
	}

	return 0;
}

int
poweraid_raid5f_gf8_decode_2(uint32_t data_chunks,
			     const void * const *surviving_bufs,
			     const void *p_buf, const void *q_buf,
			     const uint8_t missing_idx[2],
			     void *out_buf0, void *out_buf1,
			     uint64_t len)
{
	uint32_t i;
	uint8_t m0 = missing_idx[0], m1 = missing_idx[1];
	const uint8_t *exp = poweraid_raid5f_gf8_exp;
	const uint8_t *log = poweraid_raid5f_gf8_log;
	uint8_t *a_buf, *b_buf, *tmp;
	uint8_t g_m0, g_m1, det, inv_det;

	if (data_chunks < 3 || surviving_bufs == NULL || p_buf == NULL ||
	    q_buf == NULL || out_buf0 == NULL || out_buf1 == NULL || len == 0) {
		return -EINVAL;
	}
	if (m0 >= data_chunks || m1 >= data_chunks || m0 >= m1) {
		return -EINVAL;
	}

	/* 分配 3 个临时缓冲: a, b, tmp */
	a_buf = malloc(len);
	b_buf = malloc(len);
	tmp = malloc(len);
	if (a_buf == NULL || b_buf == NULL || tmp == NULL) {
		free(a_buf); free(b_buf); free(tmp);
		return -ENOMEM;
	}

	/* a = P XOR XOR(surviving D) = D_{m0} XOR D_{m1}
	 * （因为 P = XOR(all D)，减去幸存的 = 缺失的 XOR）*/
	memcpy(a_buf, p_buf, len);
	for (i = 0; i < data_chunks; i++) {
		if (i == m0 || i == m1) {
			continue;
		}
		g_mul_const_xor(surviving_bufs[i], 1, a_buf, len);
	}

	/* b = Q XOR Σ(surviving α^i × D_i) = α^{m0} × D_{m0} XOR α^{m1} × D_{m1} */
	memcpy(b_buf, q_buf, len);
	for (i = 0; i < data_chunks; i++) {
		if (i == m0 || i == m1) {
			continue;
		}
		g_mul_const_xor(surviving_bufs[i], exp[i], b_buf, len);
	}

	/* 解 2×2 方程组:
	 * | 1       1      | | D_{m0} |   | a |
	 * | α^{m0}  α^{m1} | | D_{m1} | = | b |
	 *
	 * det = α^{m0} XOR α^{m1}
	 * D_{m0} = (α^{m1} × a XOR b) × inv(det)
	 * D_{m1} = (α^{m0} × a XOR b) × inv(det)
	 */
	g_m0 = exp[m0];
	g_m1 = exp[m1];
	det = g_m0 ^ g_m1;
	if (det == 0) {
		SPDK_ERRLOG("GF8 decode_2: determinant is 0 (m0=%u m1=%u)\n", m0, m1);
		free(a_buf); free(b_buf); free(tmp);
		return -EINVAL;
	}
	inv_det = exp[255 - log[det]];

	/* D_{m0} = (α^{m1} × a XOR b) × inv_det */
	g_mul_const(a_buf, g_m1, tmp, len);
	for (i = 0; i < len; i++) {
		tmp[i] ^= b_buf[i];
	}
	g_mul_const(tmp, inv_det, out_buf0, len);

	/* D_{m1} = (α^{m0} × a XOR b) × inv_det */
	g_mul_const(a_buf, g_m0, tmp, len);
	for (i = 0; i < len; i++) {
		tmp[i] ^= b_buf[i];
	}
	g_mul_const(tmp, inv_det, out_buf1, len);

	free(a_buf);
	free(b_buf);
	free(tmp);
	return 0;
}
