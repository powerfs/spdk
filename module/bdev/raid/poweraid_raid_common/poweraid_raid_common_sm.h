/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   增强 RAID5F 模块：三层 FSM（RAID / BDEV / REQ）+ 位图状态字
 *   借鉴 XIRaid 反编译架构（leeraid/lee_raid_split/lee_raid_01_xnr_business.c）
 *   详见 raid5f-enhanced-design.md 第 3.10 节
 */

#ifndef POWERAID_RAID_COMMON_SM_H
#define POWERAID_RAID_COMMON_SM_H

#include "spdk/stdinc.h"
#include "spdk/queue.h"
#include "spdk/thread.h"
#include "spdk/uuid.h"

#include "poweraid_raid_common_merge.h"

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
enum poweraid_raid_common_fsm_layer {
	POWERAID_FSM_LAYER_RAID = 0,
	POWERAID_FSM_LAYER_BDEV,
	POWERAID_FSM_LAYER_REQ,
	POWERAID_FSM_LAYER_COUNT,
};

/* ===== RAID FSM 事件（44 个，对应 44 个 handler）===== */
enum poweraid_raid_common_raid_event {
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
enum poweraid_raid_common_bdev_event {
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
enum poweraid_raid_common_req_event {
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
struct poweraid_raid_common_raid;
struct poweraid_raid_common_bdev;
struct poweraid_raid_common_req;

/* ===== FSM handler 函数原型 ===== */
typedef void (*poweraid_raid_common_raid_handler_t)(struct poweraid_raid_common_raid *raid,
		enum poweraid_raid_common_raid_event event);
typedef void (*poweraid_raid_common_bdev_handler_t)(struct poweraid_raid_common_bdev *bdev,
		enum poweraid_raid_common_bdev_event event);
typedef void (*poweraid_raid_common_req_handler_t)(struct poweraid_raid_common_req *req,
		enum poweraid_raid_common_req_event event);

/* ===== 模块差异回调（5f / 6f 注册不同实现）=====
 *
 * 阶段 4 步骤 4：CALC 分流点。common 层的 REQ CALC handler 调用
 * raid->ops.calc_parity(req, cb) 计算校验值，完成后通过 cb(req, status)
 * 通知 common 层继续状态机。5f 注册 XOR（P only），6f 注册 P+Q 并行。
 *
 * calc_parity 约定：
 *   - req 已填充 src_bufs / parity_buf / n_src / xor_len
 *   - 完成后必须调用 cb(req, status)，status=0 成功，<0 失败
 *   - 可异步（accel）或同步内联
 */
typedef void (*poweraid_raid_calc_cb_t)(struct poweraid_raid_common_req *req, int status);

struct poweraid_raid_common_ops {
	void (*calc_parity)(struct poweraid_raid_common_req *req,
			    poweraid_raid_calc_cb_t cb);
	/* 按模块布局读指定 stripe 的某个 data chunk（整 strip）。
	 * 5f/6f 各自注册，供 recovery / parity_fixup 使用。
	 * data_cb 签名与 poweraid_raid_common_recovery_read_data_cb 一致。*/
	void (*read_strip)(struct poweraid_raid_common_raid *raid,
			   uint64_t stripe_id, uint32_t chunk_idx,
			   uint32_t chunk_len_blocks,
			   void (*data_cb)(int status, const void *buf, size_t len, void *cb_arg),
			   void *cb_arg);

	/* 重建引擎：重构 stripe 中 target_phys 槽位的整条 strip 到 out_buf
	 * （DMA、长度 strip_size*block_size）。RAID6 允许另一成员盘同时故障。
	 * 异步 cb(status)；返回 0 已接受，<0 立即失败。*/
	int (*recover_missing)(struct poweraid_raid_common_raid *raid,
			       struct raid_bdev_io_channel *raid_ch,
			       uint64_t stripe_index, uint8_t target_phys,
			       void *out_buf, uint32_t strip_len,
			       void (*cb)(int status, void *cb_arg),
			       void *cb_arg);
};

/* ===== FSM 对象（最小骨架）===== */
struct poweraid_raid_common_raid {
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
	uint8_t num_parity;          /* 1=RAID5, 2=RAID6 */
	struct poweraid_raid_common_bdev **base_bdevs;  /* num_base_bdevs 个 */
	/* superblock v2 上下文，由 poweraid_raid_common_sb.c 内部管理，
	 * 对外 opaque；NULL 表示尚未分配。 */
	void *sb_ctx;
	/* 合并层延迟（Stage 3c 可配置；0 = merge OFF，submit 即 flush）。
	 * start() 置默认值 MERGE_DELAY_US_DEFAULT，RPC bdev_poweraid_raid_common_set_merge_delay
	 * 运行时修改，ioch_create 时复制到各 channel 的 merge_ctx。 */
	uint64_t delay_us;
	/* 回指 SPDK raid_bdev 框架对象，start() 建立桥接，供 IO 路径访问 base_bdev_info。 */
	struct raid_bdev *raid_bdev;
	/* 模块差异回调（5f=XOR, 6f=P+Q），由模块 start() 注册 */
	struct poweraid_raid_common_ops ops;
	TAILQ_ENTRY(poweraid_raid_common_raid) link;
};

struct poweraid_raid_common_bdev {
	uint64_t state;  /* POWERAID_BDEV_ST_* 位图 */
	struct spdk_uuid uuid;
	uint8_t slot;
	void *desc;  /* spdk_bdev_desc */
	struct poweraid_raid_common_raid *raid;
	void *ch;               /* struct spdk_io_channel*；OPEN 阶段填充，sb_write/sb_load 用 */
	void *loaded_sb_ctx;    /* struct poweraid_raid_common_sb_ctx*；sb_load 回调持有至 VALIDATE_MD 完成，SET_ONLINE 释放 */
	void *ppl_ctx;          /* struct poweraid_raid_common_ppl_ctx*；VALIDATE_MD/建卷初始化分配，OFFLINE 释放 */
	void *mwl_ctx;          /* struct poweraid_raid_common_mwl_ctx*；raid1f VALIDATE_MD/建卷初始化分配，OFFLINE 释放 */
	bool md_present;        /* sb_load 是否在盘上读到有效 sb（区分新卷/既有卷）*/
	bool format_done;       /* 换盘：新盘 sb+PPL 异步格式化完成（重建完成状态收敛条件）*/
};

/* stripe_request 类型（req 同时充当 stripe_request，复用池化管理）*/
enum poweraid_raid_common_stripe_type {
	POWERAID_RAID_COMMON_STRIPE_REQ_WRITE = 0,
	POWERAID_RAID_COMMON_STRIPE_REQ_RECONSTRUCT,
};

struct poweraid_raid_common_io_channel;

struct poweraid_raid_common_req {
	uint32_t state;  /* POWERAID_REQ_ST_* 位图 */
	void *io;  /* spdk_bdev_io */
	struct poweraid_raid_common_raid *raid;
	/* stripe_request 字段（D-4 引入池化管理；D-5 填充实际 IO 数据）*/
	enum poweraid_raid_common_stripe_type type;
	struct poweraid_raid_common_io_channel *ch;  /* 所属 io channel 回指 */
	struct raid_bdev_io *raid_io;           /* 关联的 raid_bdev_io */
	/* 成员盘 channel 查询使用的 raid channel。重建窗口已越过本 stripe 时
	 * 指向框架 shadow channel（目标槽位路由到 target_ch）；否则==raid_io->raid_ch。
	 * 由提交点（shell 直写 / merge 全 stripe）在门控分类后填充。*/
	struct raid_bdev_io_channel *eff_raid_ch;
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
		void  (*cb)(struct poweraid_raid_common_req *req, int status);
	} xor;
	/* 写路径状态（D-5 PPL 5 步 barrier）*/
	uint64_t ppl_seq;                /* PPL append 回传的 seq */
	uint32_t base_bdev_io_remaining; /* 未完成 base bdev IO 计数 */
	int      base_bdev_io_status;    /* 聚合状态（0 成功，<0 失败）*/
	void    *data_buf;               /* owned 全 stripe 数据缓冲（spdk_dma_malloc）*/
	void    *parity_buf_alloc;       /* owned P parity 缓冲（区别于 xor 期借用 parity_buf）*/
	void    *q_buf;                  /* Q parity 目标缓冲（RAID6 用，RAID5 为 NULL）*/
	void    *q_buf_alloc;            /* owned Q parity 缓冲（RAID6 用）*/
	TAILQ_ENTRY(poweraid_raid_common_req) link;  /* free 池 / xor_retry_queue 链接 */
};

/* per-thread IO channel：参考 raid5f_io_channel（L120-136）。
 * D-4 基础设施：stripe_request 池 + accel_ch + xor_retry_queue。
 * D-5 在此基础上扩展 PPL 5 步 barrier 状态。
 * 3c：MAX_STRIPES 32→64（1M 写 qd=8 拆分为 32 并发 stripe，2× 余量；
 *     耗尽时 merge 层延迟重试，不再 fail IO）。*/
#define POWERAID_RAID_COMMON_MAX_STRIPES 64

/* 缓冲池条目：data_buf / parity_buf 预分配池，避免 per-IO spdk_dma_malloc/free。
 * 池深度 == MAX_STRIPES（与 stripe_request 池对齐），池空时 fallback malloc，
 * 池满时 put 直接 free。per-channel 无锁访问。*/
struct poweraid_raid_common_buf {
	void *buf;
	TAILQ_ENTRY(poweraid_raid_common_buf) link;
};

struct poweraid_raid_common_io_channel {
	/* 空闲 stripe_request 池（write/reconstruct 分离，参考 raid5f）*/
	TAILQ_HEAD(, poweraid_raid_common_req) free_write_stripe_requests;
	TAILQ_HEAD(, poweraid_raid_common_req) free_reconstruct_stripe_requests;

	/* accel framework channel（spdk_accel_submit_xor 用）*/
	struct spdk_io_channel *accel_ch;

	/* accel_ch 资源不足时重试队列 */
	TAILQ_HEAD(, poweraid_raid_common_req) xor_retry_queue;

	/* data_buf / parity_buf 预分配池（性能优化：避免 per-IO malloc）*/
	TAILQ_HEAD(, poweraid_raid_common_buf) free_data_bufs;
	TAILQ_HEAD(, poweraid_raid_common_buf) free_parity_bufs;
	TAILQ_HEAD(, poweraid_raid_common_buf) free_q_bufs;   /* RAID6 Q 缓冲池 */
	uint32_t data_buf_size;    /* 完整 stripe 字节数 = strip_size * (n-1) * block_size */
	uint32_t parity_buf_size;  /* strip 字节数 = strip_size * block_size */
	uint32_t q_buf_size;       /* Q 缓冲字节数 = strip_size * block_size（RAID6）*/
	uint32_t n_data_bufs;      /* 当前池中 data_buf 数量 */
	uint32_t n_parity_bufs;    /* 当前池中 parity_buf 数量 */
	uint32_t n_q_bufs;         /* 当前池中 q_buf 数量 */

	/* 合并层上下文（阶段 3b）*/
	struct merge_ctx merge_ctx;
};

/* ===== 通用分派入口 ===== */
void poweraid_raid_common_sm_process(enum poweraid_raid_common_fsm_layer layer,
			       void *obj, uint32_t event);

/* ===== 全局分派表（由 sm_raid.c / sm_bdev.c / sm_req.c 提供）=====
 * 数组长度 == 对应 *_EV_COUNT_REAL / *_EV_COUNT。
 * 索引 == enum event 值，值 == handler 函数指针。
 * sm_dispatch.c 仅引用，不再重复持有 static 表，避免与各层 .c 文件产生重复声明。
 */
extern poweraid_raid_common_raid_handler_t poweraid_raid_common_raid_fsm[];
extern poweraid_raid_common_bdev_handler_t poweraid_raid_common_bdev_fsm[];
extern poweraid_raid_common_req_handler_t poweraid_raid_common_req_fsm[];

/* ===== 缓冲池 API（data_buf / parity_buf 预分配，per-channel 无锁）===== */
void *poweraid_raid_common_get_data_buf(struct poweraid_raid_common_io_channel *ch);
void *poweraid_raid_common_get_parity_buf(struct poweraid_raid_common_io_channel *ch);
void *poweraid_raid_common_get_q_buf(struct poweraid_raid_common_io_channel *ch);
void  poweraid_raid_common_put_data_buf(struct poweraid_raid_common_io_channel *ch, void *buf);
void  poweraid_raid_common_put_parity_buf(struct poweraid_raid_common_io_channel *ch, void *buf);
void  poweraid_raid_common_put_q_buf(struct poweraid_raid_common_io_channel *ch, void *buf);
int   poweraid_raid_common_buf_pool_init(struct poweraid_raid_common_io_channel *ch,
				    struct poweraid_raid_common_raid *raid);
void  poweraid_raid_common_buf_pool_destroy(struct poweraid_raid_common_io_channel *ch);

/* 同步 XOR fallback（供 5f/6f calc_parity 实现调用）*/
void poweraid_raid_common_req_xor_sync(struct poweraid_raid_common_req *req);

/* ===== 布局 helper（left-symmetric，5f/6f 通用）=====
 * get_parity_idx: 计算 stripe 的 P/Q 物理位置。
 *   RAID5(num_parity=1): q_idx 设为 num_base_bdevs（无效值）。
 *   RAID6(num_parity=2): q_idx 有效。
 * data_to_phys: data chunk index → physical bdev index。
 * phys_to_data: physical bdev index → data chunk index（非 parity 位置）。
 */
void poweraid_raid_common_get_parity_idx(struct poweraid_raid_common_raid *raid,
					  uint64_t stripe_index,
					  uint8_t *p_idx, uint8_t *q_idx);
uint8_t poweraid_raid_common_data_to_phys(uint8_t data_idx, uint8_t p_idx,
					   uint8_t q_idx, uint8_t num_base_bdevs);
uint8_t poweraid_raid_common_phys_to_data(uint8_t phys, uint8_t p_idx, uint8_t q_idx);

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
poweraid_raid_can_accept_io(const struct poweraid_raid_common_raid *raid)
{
	uint64_t s = __atomic_load_n((uint64_t *)&raid->state, __ATOMIC_ACQUIRE);
	return (s & POWERAID_RAID_ST_ONLINE) && !(s & POWERAID_RAID_ST_OFFLINE);
}

static inline bool
poweraid_raid_is_degraded(const struct poweraid_raid_common_raid *raid)
{
	uint64_t s = __atomic_load_n((uint64_t *)&raid->state, __ATOMIC_ACQUIRE);
	return (s & POWERAID_RAID_ST_DEGRADED) || (s & POWERAID_RAID_ST_DEGRADED2);
}

static inline bool
poweraid_raid_is_recon_in_progress(const struct poweraid_raid_common_raid *raid)
{
	return poweraid_raid_state_test_any((uint64_t *)&raid->state,
					   POWERAID_RAID_ST_RECON);
}

#ifdef __cplusplus
}
#endif

#endif /* POWERAID_RAID_COMMON_SM_H */
