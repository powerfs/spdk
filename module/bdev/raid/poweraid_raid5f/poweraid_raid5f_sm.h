/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   增强 RAID5F 模块：三层 FSM（RAID / BDEV / REQ）+ 位图状态字
 *   借鉴 XIRaid 反编译架构（leeraid/lee_raid_split/lee_raid_01_xnr_business.c）
 *   详见 raid5f-enhanced-design.md 第 3.10 节
 */

#ifndef POWERAID_RAID5F_SM_H
#define POWERAID_RAID5F_SM_H

#include "spdk/stdinc.h"
#include "spdk/queue.h"
#include "spdk/thread.h"
#include "spdk/uuid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 状态位图（__int64 位图，多状态并存，原子操作）=====
 * 对比：raid5f 现有 enum spdk_bdev_raid_state 为单值，无法表达
 *       "ONLINE | RECON | SCRUBBING" 等组合状态。
 * 借鉴：XIRaid _InterlockedAnd64(state, ~0x40LL) 模式
 */
#define POWERAID_RAID_ST_ONLINE          (1ULL << 0)  /* 卷在线，可接受 IO */
#define POWERAID_RAID_ST_DEGRADED        (1ULL << 1)  /* 1 盘故障，仍可服务 */
#define POWERAID_RAID_ST_DEGRADED2       (1ULL << 2)  /* 2 盘故障（RAID6 only）*/
#define POWERAID_RAID_ST_RECON           (1ULL << 3)  /* 重建中 */
#define POWERAID_RAID_ST_SCRUBBING       (1ULL << 4)  /* 巡检中 */
#define POWERAID_RAID_ST_RESTORING       (1ULL << 5)  /* 元数据恢复中 */
#define POWERAID_RAID_ST_RESTRIPING      (1ULL << 6)  /* 在线扩容中 */
#define POWERAID_RAID_ST_INIT            (1ULL << 7)  /* 首次初始化（scrub）中 */
#define POWERAID_RAID_ST_OFFLINE        (1ULL << 8)  /* 卷离线 */
#define POWERAID_RAID_ST_CONFIG_DIRTY    (1ULL << 9)  /* 配置待持久化 */
#define POWERAID_RAID_ST_MD_DIRTY        (1ULL << 10) /* 元数据待刷盘 */
#define POWERAID_RAID_ST_PPL_DIRTY       (1ULL << 11) /* PPL 待刷盘 */

/* BDEV 状态位图（每盘一份）*/
#define POWERAID_BDEV_ST_PRESENT          (1ULL << 0)  /* 盘存在 */
#define POWERAID_BDEV_ST_OPEN             (1ULL << 1)  /* 已 open */
#define POWERAID_BDEV_ST_ONLINE          (1ULL << 2)  /* 健康 */
#define POWERAID_BDEV_ST_FAULTED         (1ULL << 3)  /* 故障 */
#define POWERAID_BDEV_ST_RECON           (1ULL << 4)  /* 重建中 */
#define POWERAID_BDEV_ST_MD_VALID        (1ULL << 5)  /* 元数据已校验 */
#define POWERAID_BDEV_ST_MD_DIRTY        (1ULL << 6)  /* 元数据待刷盘 */
#define POWERAID_BDEV_ST_SPARE           (1ULL << 7)  /* 热备盘 */

/* REQ 状态位图（每 IO 一份，uint32_t 节省内存）*/
#define POWERAID_REQ_ST_ASSIGNED         (1u << 0)
#define POWERAID_REQ_ST_READ0            (1u << 1)  /* 读旧 data（RMW）*/
#define POWERAID_REQ_ST_READ1            (1u << 2)  /* 读旧 parity（RMW）*/
#define POWERAID_REQ_ST_CALC              (1u << 3)  /* XOR/RS 计算中 */
#define POWERAID_REQ_ST_WRITE            (1u << 4)  /* 写新 data */
#define POWERAID_REQ_ST_WRITE1           (1u << 5)  /* 写新 parity */
#define POWERAID_REQ_ST_WAIT_MD          (1u << 6)  /* 等 PPL commit */
#define POWERAID_REQ_ST_IO_COMPLETE      (1u << 7)
#define POWERAID_REQ_ST_FAILED           (1u << 8)
#define POWERAID_REQ_ST_DESTROYING       (1u << 9)

/* ===== FSM 层 ===== */
enum poweraid_raid5f_fsm_layer {
	POWERAID_FSM_LAYER_RAID = 0,
	POWERAID_FSM_LAYER_BDEV,
	POWERAID_FSM_LAYER_REQ,
	POWERAID_FSM_LAYER_COUNT,
};

/* ===== RAID FSM 事件（44 个，对应 44 个 handler）===== */
enum poweraid_raid5f_raid_event {
	/* 生命周期 */
	POWERAID_RAID_EV_CREATE_DEV = 0,
	POWERAID_RAID_EV_DESTROY_DEV,
	POWERAID_RAID_EV_CREATE_OLD_DEV,
	POWERAID_RAID_EV_OPEN_BDEVS,
	POWERAID_RAID_EV_CREATE_DSC,
	POWERAID_RAID_EV_CREATE_OLD_DSC,
	POWERAID_RAID_EV_ONLINE,
	POWERAID_RAID_EV_OFFLINE,
	POWERAID_RAID_EV_DESTROY_COMPLETE,
	POWERAID_RAID_EV_FINISH,
	/* 元数据 */
	POWERAID_RAID_EV_CHECK_MD,
	POWERAID_RAID_EV_APPLY_PARAMS,
	POWERAID_RAID_EV_SAVE_CONFIG,
	POWERAID_RAID_EV_UPDATE_CONFIG,
	POWERAID_RAID_EV_UPDATE_CONFIG2,
	/* 盘管理 */
	POWERAID_RAID_EV_DISK_OPEN,
	POWERAID_RAID_EV_DISK_VERIFY,
	POWERAID_RAID_EV_DISK_ADD,
	POWERAID_RAID_EV_DISK_ADD_NEW,
	POWERAID_RAID_EV_DISK_ADD_CANCEL,
	POWERAID_RAID_EV_DISK_REMOVE,
	POWERAID_RAID_EV_DISK_ONLINE,
	/* 后台服务 */
	POWERAID_RAID_EV_START_INIT,
	POWERAID_RAID_EV_START_RECON,
	POWERAID_RAID_EV_START_SCRUB,
	POWERAID_RAID_EV_START_RESTRIPE,
	POWERAID_RAID_EV_WAIT_SERVICES_STOP,
	POWERAID_RAID_EV_WAIT_IO_END,
	POWERAID_RAID_EV_WAIT_FLUSH_MD,
	/* 配置 */
	POWERAID_RAID_EV_WRITE_LOCK,
	POWERAID_RAID_EV_ADD_RAID,
	POWERAID_RAID_EV_REMOVE_RAID,
	POWERAID_RAID_EV_REMOVE_RAID_CONFIG,
	POWERAID_RAID_EV_MGMT_CALLBACK,
	POWERAID_RAID_EV_MGMT_GET_CALLBACK,
	/* 注册 */
	POWERAID_RAID_EV_REGISTER_BDEV,
	POWERAID_RAID_EV_IODEV_UNREGISTER,
	POWERAID_RAID_EV_UNREGISTER_BDEV,
	/* 扩展（保留至 44 个）*/
	POWERAID_RAID_EV_COUNT,
};

#define POWERAID_RAID_EV_COUNT_REAL (POWERAID_RAID_EV_COUNT)
SPDK_STATIC_ASSERT(POWERAID_RAID_EV_COUNT <= 44, "RAID events exceed 44");

/* ===== BDEV FSM 事件（24 个）===== */
enum poweraid_raid5f_bdev_event {
	POWERAID_BDEV_EV_START = 0,
	POWERAID_BDEV_EV_DEV_START,
	POWERAID_BDEV_EV_OPEN,
	POWERAID_BDEV_EV_TRY_OPEN,
	POWERAID_BDEV_EV_CLOSE,
	POWERAID_BDEV_EV_CLOSE_FINISH,
	POWERAID_BDEV_EV_CALLBACK,
	POWERAID_BDEV_EV_FINISH,
	POWERAID_BDEV_EV_DESTROY,
	POWERAID_BDEV_EV_TRY_READ_MD,
	POWERAID_BDEV_EV_READ_MD,
	POWERAID_BDEV_EV_TEST_MD_EMPTY,
	POWERAID_BDEV_EV_DEV_IN_RAID,
	POWERAID_BDEV_EV_DEV_NOT_IN_RAID,
	POWERAID_BDEV_EV_VALIDATE_MD,
	POWERAID_BDEV_EV_VALIDATE_EMPTY_MD,
	POWERAID_BDEV_EV_COMPARE_MD,
	POWERAID_BDEV_EV_MERGE_MD,
	POWERAID_BDEV_EV_WRITE_MD,
	POWERAID_BDEV_EV_FLUSH_MD_ALL,
	POWERAID_BDEV_EV_WAIT_FLUSH_MD,
	POWERAID_BDEV_EV_ZERO_MD,
	POWERAID_BDEV_EV_WAIT_CH_OP,
	POWERAID_BDEV_EV_CHECK_DEV_READY,
	POWERAID_BDEV_EV_SET_ONLINE,
	POWERAID_BDEV_EV_COUNT,
};

SPDK_STATIC_ASSERT(POWERAID_BDEV_EV_COUNT <= 28, "BDEV events exceed 28");

/* ===== REQ FSM 事件（18 个）===== */
enum poweraid_raid5f_req_event {
	POWERAID_REQ_EV_ASSIGN = 0,
	POWERAID_REQ_EV_REASSIGN,
	POWERAID_REQ_EV_COMPLETE_SERVICE_REQ,
	POWERAID_REQ_EV_IO_COMPLETE,
	POWERAID_REQ_EV_WAIT_MD,
	POWERAID_REQ_EV_DESTROY,
	POWERAID_REQ_EV_READ_RESTRIPE,
	POWERAID_REQ_EV_READ_FULL,
	POWERAID_REQ_EV_READ_ALL_FULL,
	POWERAID_REQ_EV_READ0,
	POWERAID_REQ_EV_READ1,
	POWERAID_REQ_EV_READ_RECON1,
	POWERAID_REQ_EV_WRITE_FULL,
	POWERAID_REQ_EV_WRITE_PARITY,
	POWERAID_REQ_EV_WRITE_ALL_FULL,
	POWERAID_REQ_EV_WRITE_RECON,
	POWERAID_REQ_EV_WRITE,
	POWERAID_REQ_EV_WRITE_ALL,
	POWERAID_REQ_EV_WRITE1,
	POWERAID_REQ_EV_CALC,
	POWERAID_REQ_EV_COUNT,
};

SPDK_STATIC_ASSERT(POWERAID_REQ_EV_COUNT <= 24, "REQ events exceed 24");

/* ===== FSM 对象前向声明 ===== */
struct poweraid_raid5f_raid;
struct poweraid_raid5f_bdev;
struct poweraid_raid5f_req;

/* ===== FSM handler 函数原型 ===== */
typedef void (*poweraid_raid5f_raid_handler_t)(struct poweraid_raid5f_raid *raid,
		enum poweraid_raid5f_raid_event event);
typedef void (*poweraid_raid5f_bdev_handler_t)(struct poweraid_raid5f_bdev *bdev,
		enum poweraid_raid5f_bdev_event event);
typedef void (*poweraid_raid5f_req_handler_t)(struct poweraid_raid5f_req *req,
		enum poweraid_raid5f_req_event event);

/* ===== FSM 对象（最小骨架）===== */
struct poweraid_raid5f_raid {
	uint64_t state;  /* POWERAID_RAID_ST_* 位图 */
	struct spdk_uuid uuid;
	char name[64];
	uint32_t level;
	uint32_t strip_size;
	uint32_t block_size;
	uint64_t raid_size;
	/* 数据区起始块（跳过 LBA0 sb 与 PPL 区，按 strip 对齐）*/
	uint64_t data_offset_blocks;
	uint8_t num_base_bdevs;
	struct poweraid_raid5f_bdev **base_bdevs;  /* num_base_bdevs 个 */
	/* superblock v2 上下文，由 poweraid_raid5f_sb.c 内部管理，
	 * 对外 opaque；NULL 表示尚未分配。 */
	void *sb_ctx;
	/* 回指 SPDK raid_bdev 框架对象，start() 建立桥接，供 IO 路径访问 base_bdev_info。 */
	struct raid_bdev *raid_bdev;
	TAILQ_ENTRY(poweraid_raid5f_raid) link;
};

struct poweraid_raid5f_bdev {
	uint64_t state;  /* POWERAID_BDEV_ST_* 位图 */
	struct spdk_uuid uuid;
	uint8_t slot;
	void *desc;  /* spdk_bdev_desc */
	struct poweraid_raid5f_raid *raid;
	void *ch;               /* struct spdk_io_channel*；OPEN 阶段填充，sb_write/sb_load 用 */
	void *loaded_sb_ctx;    /* struct poweraid_raid5f_sb_ctx*；sb_load 回调持有至 VALIDATE_MD 完成，SET_ONLINE 释放 */
	void *ppl_ctx;          /* struct poweraid_raid5f_ppl_ctx*；VALIDATE_MD/建卷初始化分配，OFFLINE 释放 */
	bool md_present;        /* sb_load 是否在盘上读到有效 sb（区分新卷/既有卷）*/
};

/* stripe_request 类型（req 同时充当 stripe_request，复用池化管理）*/
enum poweraid_raid5f_stripe_type {
	POWERAID_RAID5F_STRIPE_REQ_WRITE = 0,
	POWERAID_RAID5F_STRIPE_REQ_RECONSTRUCT,
};

struct poweraid_raid5f_io_channel;

struct poweraid_raid5f_req {
	uint32_t state;  /* POWERAID_REQ_ST_* 位图 */
	void *io;  /* spdk_bdev_io */
	struct poweraid_raid5f_raid *raid;
	/* stripe_request 字段（D-4 引入池化管理；D-5 填充实际 IO 数据）*/
	enum poweraid_raid5f_stripe_type type;
	struct poweraid_raid5f_io_channel *ch;  /* 所属 io channel 回指 */
	struct raid_bdev_io *raid_io;           /* 关联的 raid_bdev_io */
	uint64_t stripe_index;                  /* stripe 序号 */
	/* XOR 计算参数（CALC 用），由写路径填充；src_bufs 不由 req 拥有。
	 * parity_buf 为目标 parity 缓冲；src_bufs 为数据 chunk 缓冲指针数组。
	 * 全部非 NULL 时 CALC 走 spdk_accel_submit_xor；任一为 NULL 则跳过实际计算。*/
	void				*parity_buf;
	void				**src_bufs;
	uint32_t			n_src;
	uint64_t			xor_len;
	/* XOR 异步状态（参考 raid5f stripe_request.xor）*/
	struct {
		size_t remaining;
		int    status;
		void  (*cb)(struct poweraid_raid5f_req *req, int status);
	} xor;
	/* 写路径状态（D-5 PPL 5 步 barrier）*/
	uint64_t ppl_seq;                /* PPL append 回传的 seq */
	uint32_t base_bdev_io_remaining; /* 未完成 base bdev IO 计数 */
	int      base_bdev_io_status;    /* 聚合状态（0 成功，<0 失败）*/
	void    *data_buf;               /* owned 全 stripe 数据缓冲（spdk_dma_malloc）*/
	void    *parity_buf_alloc;       /* owned parity 缓冲（区别于 xor 期借用 parity_buf）*/
	TAILQ_ENTRY(poweraid_raid5f_req) link;  /* free 池 / xor_retry_queue 链接 */
};

/* per-thread IO channel：参考 raid5f_io_channel（L120-136）。
 * D-4 基础设施：stripe_request 池 + accel_ch + xor_retry_queue。
 * D-5 在此基础上扩展 PPL 5 步 barrier 状态。*/
#define POWERAID_RAID5F_MAX_STRIPES 32

struct poweraid_raid5f_io_channel {
	/* 空闲 stripe_request 池（write/reconstruct 分离，参考 raid5f）*/
	TAILQ_HEAD(, poweraid_raid5f_req) free_write_stripe_requests;
	TAILQ_HEAD(, poweraid_raid5f_req) free_reconstruct_stripe_requests;

	/* accel framework channel（spdk_accel_submit_xor 用）*/
	struct spdk_io_channel *accel_ch;

	/* accel_ch 资源不足时重试队列 */
	TAILQ_HEAD(, poweraid_raid5f_req) xor_retry_queue;
};

/* ===== 通用分派入口 ===== */
void poweraid_raid5f_sm_process(enum poweraid_raid5f_fsm_layer layer,
			       void *obj, uint32_t event);

/* ===== 全局分派表（由 sm_raid.c / sm_bdev.c / sm_req.c 提供）=====
 * 数组长度 == 对应 *_EV_COUNT_REAL / *_EV_COUNT。
 * 索引 == enum event 值，值 == handler 函数指针。
 * sm_dispatch.c 仅引用，不再重复持有 static 表，避免与各层 .c 文件产生重复声明。
 */
extern poweraid_raid5f_raid_handler_t poweraid_raid5f_raid_fsm[];
extern poweraid_raid5f_bdev_handler_t poweraid_raid5f_bdev_fsm[];
extern poweraid_raid5f_req_handler_t poweraid_raid5f_req_fsm[];

/* ===== 状态位原子操作 ===== */
static inline bool
poweraid_raid_state_test(uint64_t *state, uint64_t bits)
{
	return (__atomic_load_n(state, __ATOMIC_ACQUIRE) & bits) == bits;
}

static inline bool
poweraid_raid_state_test_any(uint64_t *state, uint64_t bits)
{
	return (__atomic_load_n(state, __ATOMIC_ACQUIRE) & bits) != 0;
}

static inline void
poweraid_raid_state_set(uint64_t *state, uint64_t bits)
{
	__atomic_fetch_or(state, bits, __ATOMIC_ACQ_REL);
}

static inline void
poweraid_raid_state_clear(uint64_t *state, uint64_t bits)
{
	__atomic_fetch_and(state, ~bits, __ATOMIC_ACQ_REL);
}

/* ===== 复合判断 ===== */
static inline bool
poweraid_raid_can_accept_io(const struct poweraid_raid5f_raid *raid)
{
	uint64_t s = __atomic_load_n((uint64_t *)&raid->state, __ATOMIC_ACQUIRE);
	return (s & POWERAID_RAID_ST_ONLINE) && !(s & POWERAID_RAID_ST_OFFLINE);
}

static inline bool
poweraid_raid_is_degraded(const struct poweraid_raid5f_raid *raid)
{
	uint64_t s = __atomic_load_n((uint64_t *)&raid->state, __ATOMIC_ACQUIRE);
	return (s & POWERAID_RAID_ST_DEGRADED) || (s & POWERAID_RAID_ST_DEGRADED2);
}

static inline bool
poweraid_raid_is_recon_in_progress(const struct poweraid_raid5f_raid *raid)
{
	return poweraid_raid_state_test_any((uint64_t *)&raid->state,
					   POWERAID_RAID_ST_RECON);
}

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID5F_SM_H */
