/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   GF(2^8) ARM NEON SIMD 实现（Stage 4 RAID6 Q 校验）
 *
 *   核心技术: split nibble lookup table + vqtbl1q_u8 (NEON tbl 指令)
 *     - 与 x86 pshufb 等价的 16 路并行查表
 *     - hi_lut[16] / lo_lut[16] 同 x86 版本
 *     - vqtbl1q_u8 替代 _mm_shuffle_epi8
 *
 *   Non-temporal store: stnp（STNP, 立数 Non-temporal Pair Store）
 *   绕过 cache 直接写内存。符合 project memory 硬约束。
 *
 *   注意: 本文件仅在 ARM 平台编译。x86 平台使用 gf8_x86.c。
 *   BF3 DPU 部署（阶段 7）时验证此路径。
 */

#include "spdk/stdinc.h"
#include "spdk/log.h"

#include "poweraid_raid5f_gf8.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_gf8_neon);

#ifdef __aarch64__
#include <arm_neon.h>

/* ===== CPU 特性检测（ARM64: NEON 总是可用）===== */

/* ARM64 (AArch64) NEON 总是可用，无需运行时检测。
 * BF3 (ARM Cortex-X2 / Neoverse N2) 还支持 SVE，但 NEON 足够。*/

/* ===== 构建常量乘法查找表（与 x86 版本相同算法）===== */

static void
build_mul_lut_neon(uint8_t c, uint8_t hi_lut[16], uint8_t lo_lut[16])
{
	const uint8_t *exp = poweraid_raid5f_gf8_exp;
	const uint8_t *log = poweraid_raid5f_gf8_log;
	uint8_t lc;
	int i;

	if (c == 0) {
		memset(hi_lut, 0, 16);
		memset(lo_lut, 0, 16);
		return;
	}

	lc = log[c];
	for (i = 0; i < 16; i++) {
		lo_lut[i] = (i == 0) ? 0 : exp[lc + log[i]];
		uint8_t v = i << 4;
		hi_lut[i] = (v == 0) ? 0 : exp[lc + log[v]];
	}
}

/* ===== NEON mul_const: dst[i] = c × src[i] ===== */

static void
gf8_mul_const_neon(const uint8_t *src, uint8_t c, uint8_t *dst, uint64_t len)
{
	uint8_t hi_lut[16], lo_lut[16];

	if (c == 0) {
		memset(dst, 0, len);
		return;
	}
	if (c == 1) {
		if (dst != src) {
			memcpy(dst, src, len);
		}
		return;
	}

	build_mul_lut_neon(c, hi_lut, lo_lut);

	uint8x16_t v_hi_lut = vld1q_u8(hi_lut);
	uint8x16_t v_lo_lut = vld1q_u8(lo_lut);
	uint8x16_t v_lo_mask = vdupq_n_u8(0x0f);
	uint64_t i;

	for (i = 0; i + 16 <= len; i += 16) {
		uint8x16_t v = vld1q_u8(src + i);
		uint8x16_t lo = vandq_u8(v, v_lo_mask);
		uint8x16_t hi = vshrq_n_u8(v, 4);
		hi = vandq_u8(hi, v_lo_mask);
		uint8x16_t r_lo = vqtbl1q_u8(v_lo_lut, lo);
		uint8x16_t r_hi = vqtbl1q_u8(v_hi_lut, hi);
		uint8x16_t result = veorq_u8(r_hi, r_lo);
		vst1q_u8(dst + i, result);
	}

	for (; i < len; i++) {
		uint8_t byte = src[i];
		dst[i] = hi_lut[byte >> 4] ^ lo_lut[byte & 0x0f];
	}
}

/* ===== NEON mul_const_xor: dst[i] ^= c × src[i]（encode 用）=====
 *
 * Non-temporal store: 用 stnp（STNP q0, q1, [addr]）绕过 cache。
 * 注意: stnp 要求 32 字节对齐。不对齐时 fallback 到常规 store。
 */
static void
gf8_mul_const_xor_neon(const uint8_t *src, uint8_t c, uint8_t *dst, uint64_t len)
{
	uint8_t hi_lut[16], lo_lut[16];

	if (c == 0) {
		return;
	}
	if (c == 1) {
		uint64_t i;
		for (i = 0; i + 16 <= len; i += 16) {
			uint8x16_t v = vld1q_u8(src + i);
			uint8x16_t d = vld1q_u8(dst + i);
			d = veorq_u8(d, v);
			vst1q_u8(dst + i, d);
		}
		for (; i < len; i++) {
			dst[i] ^= src[i];
		}
		return;
	}

	build_mul_lut_neon(c, hi_lut, lo_lut);

	uint8x16_t v_hi_lut = vld1q_u8(hi_lut);
	uint8x16_t v_lo_lut = vld1q_u8(lo_lut);
	uint8x16_t v_lo_mask = vdupq_n_u8(0x0f);
	uint64_t i;

	for (i = 0; i + 32 <= len; i += 32) {
		uint8x16_t v0 = vld1q_u8(src + i);
		uint8x16_t v1 = vld1q_u8(src + i + 16);

		uint8x16_t lo0 = vandq_u8(v0, v_lo_mask);
		uint8x16_t hi0 = vandq_u8(vshrq_n_u8(v0, 4), v_lo_mask);
		uint8x16_t r0 = veorq_u8(vqtbl1q_u8(v_hi_lut, hi0),
					 vqtbl1q_u8(v_lo_lut, lo0));

		uint8x16_t lo1 = vandq_u8(v1, v_lo_mask);
		uint8x16_t hi1 = vandq_u8(vshrq_n_u8(v1, 4), v_lo_mask);
		uint8x16_t r1 = veorq_u8(vqtbl1q_u8(v_hi_lut, hi1),
					 vqtbl1q_u8(v_lo_lut, lo1));

		/* Non-temporal store: stnp 绕过 cache
		 * 需要先读 dst、XOR、再用 stnp 写 */
		uint8x16_t d0 = vld1q_u8(dst + i);
		uint8x16_t d1 = vld1q_u8(dst + i + 16);
		d0 = veorq_u8(d0, r0);
		d1 = veorq_u8(d1, r1);

		/* stnp 要求指针 32 字节对齐；不对齐时用 vst1q */
		if (((uintptr_t)(dst + i) & 31) == 0) {
			__builtin_nontemporal_store(d0, (uint8x16_t *)(dst + i));
			__builtin_nontemporal_store(d1, (uint8x16_t *)(dst + i + 16));
		} else {
			vst1q_u8(dst + i, d0);
			vst1q_u8(dst + i + 16, d1);
		}
	}
	__sync_synchronize();

	for (; i + 16 <= len; i += 16) {
		uint8x16_t v = vld1q_u8(src + i);
		uint8x16_t lo = vandq_u8(v, v_lo_mask);
		uint8x16_t hi = vandq_u8(vshrq_n_u8(v, 4), v_lo_mask);
		uint8x16_t r = veorq_u8(vqtbl1q_u8(v_hi_lut, hi),
					vqtbl1q_u8(v_lo_lut, lo));
		uint8x16_t d = vld1q_u8(dst + i);
		d = veorq_u8(d, r);
		vst1q_u8(dst + i, d);
	}

	for (; i < len; i++) {
		uint8_t byte = src[i];
		dst[i] ^= hi_lut[byte >> 4] ^ lo_lut[byte & 0x0f];
	}
}

/* ===== SIMD 路径选择入口 ===== */

void
poweraid_raid5f_gf8_try_select_simd(
	poweraid_raid5f_gf8_mul_const_fn     *p_mul_const,
	poweraid_raid5f_gf8_mul_const_xor_fn *p_mul_const_xor,
	const char                          **p_impl_name)
{
	/* ARM64 NEON 总是可用 */
	*p_mul_const     = gf8_mul_const_neon;
	*p_mul_const_xor = gf8_mul_const_xor_neon;
	*p_impl_name     = "neon";
	SPDK_NOTICELOG("GF8: selected ARM NEON path (vqtbl1q + stnp)\n");
}

#else /* !__aarch64__ */

/* 非 ARM 平台: 不覆盖函数指针，保持标量或 x86 路径 */
void
poweraid_raid5f_gf8_try_select_simd(
	poweraid_raid5f_gf8_mul_const_fn     *p_mul_const,
	poweraid_raid5f_gf8_mul_const_xor_fn *p_mul_const_xor,
	const char                          **p_impl_name)
{
	/* x86 平台由 gf8_x86.c 处理；其他平台保持标量 */
	(void)p_mul_const;
	(void)p_mul_const_xor;
	(void)p_impl_name;
}

#endif /* __aarch64__ */
