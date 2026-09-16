/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   GF(2^8) x86 SSE/AVX2 SIMD 实现（Stage 4 RAID6 Q 校验）
 *
 *   核心技术: split nibble lookup table + pshufb (_mm_shuffle_epi8)
 *     - 将 GF8 乘以常量 c 分解为 4-bit nibble 查表
 *     - hi_lut[16]: c × (i << 4), i = 0..15
 *     - lo_lut[16]: c × i,          i = 0..15
 *     - pshufb 对 16 字节并行查表
 *     - 结果 = pshufb(hi_lut, hi_nibbles) XOR pshufb(lo_lut, lo_nibbles)
 *
 *   Non-temporal store: encode 路径的 q_buf 写入用 _mm_stream_si128，
 *   绕过 cache 直接写内存（Q 缓冲写后不立即读，避免污染 cache）。
 *   符合 project memory 硬约束: SIMD 必须含 non-temporal store 优化。
 *
 *   详见 raid5f-enhanced-design.md 第 3.3 节（阶段 4）。
 */

#include "spdk/stdinc.h"

#ifdef __x86_64__
#include <immintrin.h>  /* SSE/AVX2 intrinsics */
#endif

#include "spdk/log.h"

#include "poweraid_raid_common_gf8.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_gf8_x86);

#ifdef __x86_64__

/* ===== CPU 特性检测 ===== */

static bool
cpu_supports_ssse3(void)
{
	unsigned int eax, ebx, ecx, edx;

	__asm__ __volatile__("cpuid"
			     : "=a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx)
			     : "a"(1));
	return (ecx & (1u << 9)) != 0;  /* SSSE3 bit */
}

static bool
cpu_supports_avx2(void)
{
	unsigned int eax, ebx, ecx, edx;

	__asm__ __volatile__("cpuid"
			     : "=a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx)
			     : "a"(7), "c"(0));
	return (ebx & (1u << 5)) != 0;  /* AVX2 bit */
}

/* ===== 构建常量乘法查找表 =====
 *
 * 为常量 c 构建 2 个 16 字节表（hi + lo），供 pshufb 使用。
 * pshufb 要求表为 16 字节，索引 0..15。
 * 高 nibble 表的 mask: 清低 4 位，留高 4 位 → pshufb 直接用低 4 位索引。
 */

static void
build_mul_lut(uint8_t c, uint8_t hi_lut[16], uint8_t lo_lut[16])
{
	const uint8_t *exp = poweraid_raid_common_gf8_exp;
	const uint8_t *log = poweraid_raid_common_gf8_log;
	uint8_t lc;
	int i;

	if (c == 0) {
		memset(hi_lut, 0, 16);
		memset(lo_lut, 0, 16);
		return;
	}

	lc = log[c];
	for (i = 0; i < 16; i++) {
		/* lo_lut[i] = c × i */
		lo_lut[i] = (i == 0) ? 0 : exp[lc + log[i]];
		/* hi_lut[i] = c × (i << 4) */
		uint8_t v = i << 4;
		hi_lut[i] = (v == 0) ? 0 : exp[lc + log[v]];
	}
}

/* nibble mask: 输入字节的高 4 位右移到低 4 位（用作 hi_lut 索引）*/
/* 注意: __m128i 全局变量不能用 _mm_set1_epi8 初始化（非常量表达式）。
 * 在各函数内用局部变量 _mm_set1_epi8(0x0f) 代替。*/

/* ===== SSE mul_const: dst[i] = c × src[i] ===== */

static void
gf8_mul_const_ssse3(const uint8_t *src, uint8_t c, uint8_t *dst, uint64_t len)
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

	build_mul_lut(c, hi_lut, lo_lut);

	__m128i v_hi_lut = _mm_loadu_si128((const __m128i *)hi_lut);
	__m128i v_lo_lut = _mm_loadu_si128((const __m128i *)lo_lut);
	__m128i v_lo_mask = _mm_set1_epi8(0x0f);
	uint64_t i;

	/* 16 字节一组处理 */
	for (i = 0; i + 16 <= len; i += 16) {
		__m128i v = _mm_loadu_si128((const __m128i *)(src + i));
		__m128i lo = _mm_and_si128(v, v_lo_mask);
		__m128i hi = _mm_srli_epi16(v, 4);
		hi = _mm_and_si128(hi, v_lo_mask);
		__m128i r_lo = _mm_shuffle_epi8(v_lo_lut, lo);
		__m128i r_hi = _mm_shuffle_epi8(v_hi_lut, hi);
		__m128i result = _mm_xor_si128(r_hi, r_lo);
		_mm_storeu_si128((__m128i *)(dst + i), result);
	}

	/* 尾部标量处理 */
	for (; i < len; i++) {
		uint8_t byte = src[i];
		uint8_t lo = byte & 0x0f;
		uint8_t hi = byte >> 4;
		dst[i] = hi_lut[hi] ^ lo_lut[lo];
	}
}

/* ===== SSE mul_const_xor: dst[i] ^= c × src[i]（encode 用）=====
 *
 * Non-temporal store 优化: q_buf 是只写不读的目标缓冲，
 * 用 _mm_stream_si128 绕过 cache，避免 cache 污染。
 * 最后一次 stream 后需 _mm_sfence 保证可见性。
 */
static void
gf8_mul_const_xor_ssse3(const uint8_t *src, uint8_t c, uint8_t *dst, uint64_t len)
{
	uint8_t hi_lut[16], lo_lut[16];

	if (c == 0) {
		return; /* dst 不变 */
	}
	if (c == 1) {
		/* 纯 XOR，不走 non-temporal（因为 dst 既有读又有写）*/
		uint64_t i;
		for (i = 0; i + 16 <= len; i += 16) {
			__m128i v = _mm_loadu_si128((const __m128i *)(src + i));
			__m128i d = _mm_loadu_si128((const __m128i *)(dst + i));
			d = _mm_xor_si128(d, v);
			_mm_storeu_si128((__m128i *)(dst + i), d);
		}
		for (; i < len; i++) {
			dst[i] ^= src[i];
		}
		return;
	}

	build_mul_lut(c, hi_lut, lo_lut);

	__m128i v_hi_lut = _mm_loadu_si128((const __m128i *)hi_lut);
	__m128i v_lo_lut = _mm_loadu_si128((const __m128i *)lo_lut);
	__m128i v_lo_mask = _mm_set1_epi8(0x0f);
	uint64_t i;

	for (i = 0; i + 16 <= len; i += 16) {
		__m128i v = _mm_loadu_si128((const __m128i *)(src + i));
		__m128i lo = _mm_and_si128(v, v_lo_mask);
		__m128i hi = _mm_srli_epi16(v, 4);
		hi = _mm_and_si128(hi, v_lo_mask);
		__m128i r_lo = _mm_shuffle_epi8(v_lo_lut, lo);
		__m128i r_hi = _mm_shuffle_epi8(v_hi_lut, hi);
		__m128i product = _mm_xor_si128(r_hi, r_lo);

		/* 读取当前 dst 值，XOR 乘积，用 non-temporal store 写回 */
		__m128i d = _mm_loadu_si128((const __m128i *)(dst + i));
		d = _mm_xor_si128(d, product);
		_mm_stream_si128((__m128i *)(dst + i), d);
	}
	_mm_sfence();

	for (; i < len; i++) {
		uint8_t byte = src[i];
		uint8_t lo = byte & 0x0f;
		uint8_t hi = byte >> 4;
		dst[i] ^= hi_lut[hi] ^ lo_lut[lo];
	}
}

/* ===== AVX2 版本（32 字节一组，吞吐翻倍）===== */

static void
gf8_mul_const_avx2(const uint8_t *src, uint8_t c, uint8_t *dst, uint64_t len)
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

	build_mul_lut(c, hi_lut, lo_lut);

	/* AVX2: 32 字节一组，用两个 __m128i 拼成 __m256i */
	__m128i v_hi_lut_128 = _mm_loadu_si128((const __m128i *)hi_lut);
	__m128i v_lo_lut_128 = _mm_loadu_si128((const __m128i *)lo_lut);
	__m256i v_hi_lut = _mm256_broadcastsi128_si256(v_hi_lut_128);
	__m256i v_lo_lut = _mm256_broadcastsi128_si256(v_lo_lut_128);
	__m256i v_lo_mask = _mm256_set1_epi8(0x0f);
	uint64_t i;

	for (i = 0; i + 32 <= len; i += 32) {
		__m256i v = _mm256_loadu_si256((const __m256i *)(src + i));
		__m256i lo = _mm256_and_si256(v, v_lo_mask);
		__m256i hi = _mm256_srli_epi16(v, 4);
		hi = _mm256_and_si256(hi, v_lo_mask);
		__m256i r_lo = _mm256_shuffle_epi8(v_lo_lut, lo);
		__m256i r_hi = _mm256_shuffle_epi8(v_hi_lut, hi);
		__m256i result = _mm256_xor_si256(r_hi, r_lo);
		_mm256_storeu_si256((__m256i *)(dst + i), result);
	}

	/* 尾部 16 字节组 */
	if (i + 16 <= len) {
		__m128i v = _mm_loadu_si128((const __m128i *)(src + i));
		__m128i lo = _mm_and_si128(v, _mm_set1_epi8(0x0f));
		__m128i hi = _mm_and_si128(_mm_srli_epi16(v, 4), _mm_set1_epi8(0x0f));
		__m128i r = _mm_xor_si128(_mm_shuffle_epi8(v_hi_lut_128, hi),
					  _mm_shuffle_epi8(v_lo_lut_128, lo));
		_mm_storeu_si128((__m128i *)(dst + i), r);
		i += 16;
	}

	for (; i < len; i++) {
		uint8_t byte = src[i];
		dst[i] = hi_lut[byte >> 4] ^ lo_lut[byte & 0x0f];
	}
}

static void
gf8_mul_const_xor_avx2(const uint8_t *src, uint8_t c, uint8_t *dst, uint64_t len)
{
	uint8_t hi_lut[16], lo_lut[16];

	if (c == 0) {
		return;
	}
	if (c == 1) {
		uint64_t i;
		for (i = 0; i + 32 <= len; i += 32) {
			__m256i v = _mm256_loadu_si256((const __m256i *)(src + i));
			__m256i d = _mm256_loadu_si256((const __m256i *)(dst + i));
			d = _mm256_xor_si256(d, v);
			_mm256_storeu_si256((__m256i *)(dst + i), d);
		}
		for (; i + 16 <= len; i += 16) {
			__m128i v = _mm_loadu_si128((const __m128i *)(src + i));
			__m128i d = _mm_loadu_si128((const __m128i *)(dst + i));
			d = _mm_xor_si128(d, v);
			_mm_storeu_si128((__m128i *)(dst + i), d);
		}
		for (; i < len; i++) {
			dst[i] ^= src[i];
		}
		return;
	}

	build_mul_lut(c, hi_lut, lo_lut);

	__m128i v_hi_lut_128 = _mm_loadu_si128((const __m128i *)hi_lut);
	__m128i v_lo_lut_128 = _mm_loadu_si128((const __m128i *)lo_lut);
	__m256i v_hi_lut = _mm256_broadcastsi128_si256(v_hi_lut_128);
	__m256i v_lo_lut = _mm256_broadcastsi128_si256(v_lo_lut_128);
	__m256i v_lo_mask = _mm256_set1_epi8(0x0f);
	uint64_t i;

	for (i = 0; i + 32 <= len; i += 32) {
		__m256i v = _mm256_loadu_si256((const __m256i *)(src + i));
		__m256i lo = _mm256_and_si256(v, v_lo_mask);
		__m256i hi = _mm256_srli_epi16(v, 4);
		hi = _mm256_and_si256(hi, v_lo_mask);
		__m256i r_lo = _mm256_shuffle_epi8(v_lo_lut, lo);
		__m256i r_hi = _mm256_shuffle_epi8(v_hi_lut, hi);
		__m256i product = _mm256_xor_si256(r_hi, r_lo);

		/* Non-temporal store: 256-bit stream 需要拆成两个 128-bit stream
		 * （_mm256_stream_si256 对齐要求 32 字节，且不能保证所有 CPU 支持）。
		 * 用 _mm_stream_si128 拆分写，确保 non-temporal 语义。*/
		__m256i d = _mm256_loadu_si256((const __m256i *)(dst + i));
		d = _mm256_xor_si256(d, product);
		__m128i lo128 = _mm256_extracti128_si256(d, 0);
		__m128i hi128 = _mm256_extracti128_si256(d, 1);
		_mm_stream_si128((__m128i *)(dst + i), lo128);
		_mm_stream_si128((__m128i *)(dst + i + 16), hi128);
	}
	_mm_sfence();

	/* 尾部 16 字节组 */
	if (i + 16 <= len) {
		__m128i v = _mm_loadu_si128((const __m128i *)(src + i));
		__m128i lo = _mm_and_si128(v, _mm_set1_epi8(0x0f));
		__m128i hi = _mm_and_si128(_mm_srli_epi16(v, 4), _mm_set1_epi8(0x0f));
		__m128i r = _mm_xor_si128(_mm_shuffle_epi8(v_hi_lut_128, hi),
					  _mm_shuffle_epi8(v_lo_lut_128, lo));
		__m128i d = _mm_loadu_si128((const __m128i *)(dst + i));
		d = _mm_xor_si128(d, r);
		_mm_stream_si128((__m128i *)(dst + i), d);
		i += 16;
	}
	_mm_sfence();

	for (; i < len; i++) {
		uint8_t byte = src[i];
		dst[i] ^= hi_lut[byte >> 4] ^ lo_lut[byte & 0x0f];
	}
}

/* ===== SIMD 路径选择入口 ===== */

void
poweraid_raid_common_gf8_try_select_simd(
	poweraid_raid_common_gf8_mul_const_fn     *p_mul_const,
	poweraid_raid_common_gf8_mul_const_xor_fn *p_mul_const_xor,
	const char                          **p_impl_name)
{
	if (cpu_supports_avx2()) {
		*p_mul_const     = gf8_mul_const_avx2;
		*p_mul_const_xor = gf8_mul_const_xor_avx2;
		*p_impl_name     = "avx2";
		SPDK_NOTICELOG("GF8: selected AVX2 path (pshufb + stream_si128)\n");
	} else if (cpu_supports_ssse3()) {
		*p_mul_const     = gf8_mul_const_ssse3;
		*p_mul_const_xor = gf8_mul_const_xor_ssse3;
		*p_impl_name     = "ssse3";
		SPDK_NOTICELOG("GF8: selected SSSE3 path (pshufb + stream_si128)\n");
	} else {
		SPDK_NOTICELOG("GF8: no x86 SIMD available, keeping scalar path\n");
	}
}

#else /* !__x86_64__ */

/* 非 x86 平台: 不覆盖函数指针，保持标量 fallback */
void
poweraid_raid_common_gf8_try_select_simd(
	poweraid_raid_common_gf8_mul_const_fn     *p_mul_const,
	poweraid_raid_common_gf8_mul_const_xor_fn *p_mul_const_xor,
	const char                          **p_impl_name)
{
	SPDK_NOTICELOG("GF8: x86 SIMD not available on this platform, using scalar\n");
	(void)p_mul_const;
	(void)p_mul_const_xor;
	(void)p_impl_name;
}

#endif /* __x86_64__ */
