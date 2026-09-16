/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   GF(2^8) Reed-Solomon 编码引擎（Stage 4 RAID6 Q 校验）
 *
 *   多项式: 0x11d（与 Linux mdadm RAID6 兼容）
 *   生成元: α = 2（primitive element）
 *
 *   Q 校验: Q = Σ (α^i × D_i), i = 0..N-1（N = data_chunks）
 *   双盘恢复: 解 GF(2^8) 2×2 方程组（P + Q）
 *
 *   实现分层:
 *     - poweraid_raid5f_gf8.c:     软件参考实现（查表，正确性基准）
 *     - poweraid_raid5f_gf8_x86.c: x86 SSE/AVX2（pshufb 16 路并行 + non-temporal store）
 *     - poweraid_raid5f_gf8_neon.c: ARM NEON（vqtbl1q 16 路并行 + stnp non-temporal）
 *
 *   运行时由 gf8_init() 通过 CPUID 选最快路径，函数指针分发。
 *
 *   详见 raid5f-enhanced-design.md 第 3.3 节（阶段 4）。
 */

#ifndef POWERAID_RAID5F_GF8_H
#define POWERAID_RAID5F_GF8_H

#include "spdk/stdinc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 公共 API ===== */

/* 编码完成回调 */
typedef void (*poweraid_gf8_completion_cb)(void *cb_arg, int status);

/* 模块初始化：构建 log/antilog 表，CPUID 选最优路径。
 * 线程安全，仅需调用一次（模块加载时）。*/
int  poweraid_raid5f_gf8_init(void);

/* 模块清理 */
void poweraid_raid5f_gf8_cleanup(void);

/* 同步编码：Q = Σ (α^i × D_i), i = 0..data_chunks-1
 *
 * 计算 data_chunks 个数据缓冲的 GF8 编码，结果写入 q_buf。
 * 每个缓冲长度 = len 字节。
 *
 * 参数:
 *   data_chunks:  数据 chunk 数（≥2）
 *   data_bufs:    data_chunks 个数据缓冲指针数组
 *   q_buf:        输出 Q 缓冲（调用方分配，len 字节）
 *   len:          每个 chunk 的字节长度（必须 ≥1，无对齐要求）
 *
 * 返回: 0 成功, 负数错误码 */
int poweraid_raid5f_gf8_encode(uint32_t data_chunks,
			       const void * const *data_bufs,
			       void *q_buf,
			       uint64_t len);

/* 同步双盘解码：从 P + Q 恢复 2 个缺失的 data chunk
 *
 * 给定 data_chunks 个 data 位置（其中 2 个缺失），以及 P 和 Q，
 * 求解 2 个缺失的 data。
 *
 * 参数:
 *   data_chunks:    总 data chunk 数（≥3）
 *   surviving_bufs: data_chunks 个指针，缺失位置传 NULL，
 *                   幸存位置传数据缓冲指针
 *   p_buf:          P 缓冲（XOR of all data）
 *   q_buf:          Q 缓冲（GF8 encode of all data）
 *   missing_idx:    2 个缺失 data 的序号，missing_idx[0] < missing_idx[1]
 *   out_buf0:       输出 missing_idx[0] 的恢复数据
 *   out_buf1:       输出 missing_idx[1] 的恢复数据
 *   len:            每个 chunk 的字节长度
 *
 * 返回: 0 成功, 负数错误码 */
int poweraid_raid5f_gf8_decode_2(uint32_t data_chunks,
				 const void * const *surviving_bufs,
				 const void *p_buf, const void *q_buf,
				 const uint8_t missing_idx[2],
				 void *out_buf0, void *out_buf1,
				 uint64_t len);

/* ===== 内部: 标量表（供 x86/neon 实现共享）=====
 *
 * 表在 gf8_init() 时运行时生成（多项式 0x11d, 生成元 α=2），
 * 避免手写表数据的错误风险。init 前内容未定义。
 */

/* GF(2^8) 对数表（log_α[x], x=1..255; log[0] = 0 约定）*/
extern uint8_t poweraid_raid5f_gf8_log[256];

/* GF(2^8) 反对数表（α^x, x=0..254; 扩展到 512 以避免 mod 255）*/
extern uint8_t poweraid_raid5f_gf8_exp[512];

/* ===== 内部: 标量运算（供软件参考实现和测试用）===== */

static inline uint8_t
poweraid_raid5f_gf8_mul_scalar(uint8_t a, uint8_t b)
{
	if (a == 0 || b == 0) {
		return 0;
	}
	return poweraid_raid5f_gf8_exp[poweraid_raid5f_gf8_log[a] + poweraid_raid5f_gf8_log[b]];
}

static inline uint8_t
poweraid_raid5f_gf8_div_scalar(uint8_t a, uint8_t b)
{
	if (a == 0) {
		return 0;
	}
	/* b != 0 前提（调用方保证）*/
	return poweraid_raid5f_gf8_exp[(uint32_t)(poweraid_raid5f_gf8_log[a] + 255 - poweraid_raid5f_gf8_log[b]) % 255];
}

static inline uint8_t
poweraid_raid5f_gf8_inv_scalar(uint8_t a)
{
	/* a != 0 前提 */
	return poweraid_raid5f_gf8_exp[255 - poweraid_raid5f_gf8_log[a]];
}

/* ===== 内部: 乘法函数指针类型（SIMD 路径注册用）=====
 *
 * gf8_mul_const: dst[i] = constant × src[i]（逐字节 GF8 乘以常量）
 *   - 每个缓冲 len 字节
 *   - dst 可 == src（原地乘法）
 */
typedef void (*poweraid_raid5f_gf8_mul_const_fn)(const uint8_t *src, uint8_t constant,
		uint8_t *dst, uint64_t len);

/* gf8_mul_const_xor: dst[i] ^= constant × src[i]（累加模式，用于 encode）*/
typedef void (*poweraid_raid5f_gf8_mul_const_xor_fn)(const uint8_t *src, uint8_t constant,
		uint8_t *dst, uint64_t len);

/* 获取当前激活的 SIMD 路径名（调试/日志用）*/
const char *poweraid_raid5f_gf8_impl_name(void);

/* ===== 内部: SIMD 路径选择（由 gf8_init 调用）=====
 *
 * SIMD 实现文件（gf8_x86.c / gf8_neon.c）提供此函数，
 * 检测 CPU 特性，若支持则覆盖传入的函数指针。
 * 若不支持则保持不变（fallback 到标量）。
 *
 * 参数均为 in/out:
 *   p_mul_const:     乘法函数指针
 *   p_mul_const_xor: 乘法+累加函数指针
 *   p_impl_name:     实现名（调试用）
 */
void poweraid_raid5f_gf8_try_select_simd(
	poweraid_raid5f_gf8_mul_const_fn     *p_mul_const,
	poweraid_raid5f_gf8_mul_const_xor_fn *p_mul_const_xor,
	const char                          **p_impl_name);

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID5F_GF8_H */
