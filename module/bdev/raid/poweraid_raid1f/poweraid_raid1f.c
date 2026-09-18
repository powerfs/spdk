/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   poweraid RAID1F 模块：N 副本镜像（N>=2），骨架版
 *
 *   Task 1：级别注册 + 基础镜像读写（等价上游 raid1）+ 通用 process 重建占位。
 *   后续任务：sb v2 MWL ext -> MWL 日志层 -> 写路径定序 -> replay 恢复 ->
 *             降级/lead 反向重建。
 */

#include "spdk/stdinc.h"
#include "spdk/bdev_module.h"
#include "spdk/log.h"

#include "../bdev_raid.h"
#include "poweraid_raid1f.h"
#include "../poweraid_raid_common/poweraid_raid_common_sm.h"

/* ===== IO channel：每副本读计数器（读均衡用，与 raid1 相同）===== */
struct poweraid_raid1f_io_channel {
	uint64_t read_blocks_outstanding[0];
};

static void
raid1f_channel_inc_read_counters(struct raid_bdev_io_channel *raid_ch, uint8_t idx,
				 uint64_t num_blocks)
{
	struct poweraid_raid1f_io_channel *ch = raid_bdev_channel_get_module_ctx(raid_ch);

	assert(ch->read_blocks_outstanding[idx] <= UINT64_MAX - num_blocks);
	ch->read_blocks_outstanding[idx] += num_blocks;
}

static void
raid1f_channel_dec_read_counters(struct raid_bdev_io_channel *raid_ch, uint8_t idx,
				 uint64_t num_blocks)
{
	struct poweraid_raid1f_io_channel *ch = raid_bdev_channel_get_module_ctx(raid_ch);

	assert(ch->read_blocks_outstanding[idx] >= num_blocks);
	ch->read_blocks_outstanding[idx] -= num_blocks;
}

static void
raid1f_init_ext_io_opts(struct spdk_bdev_ext_io_opts *opts, struct raid_bdev_io *raid_io)
{
	memset(opts, 0, sizeof(*opts));
	opts->size = sizeof(*opts);
	opts->memory_domain = raid_io->memory_domain;
	opts->memory_domain_ctx = raid_io->memory_domain_ctx;
	opts->metadata = raid_io->md_buf;
}

/* ===== 写完成（基础镜像路径：每副本独立完成汇聚）===== */
static void
raid1f_write_bdev_io_completion(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct raid_bdev_io *raid_io = cb_arg;

	if (!success) {
		struct raid_base_bdev_info *base_info;

		base_info = raid_bdev_channel_get_base_info(raid_io->raid_ch, bdev_io->bdev);
		if (base_info) {
			raid_bdev_fail_base_bdev(base_info);
		}
	}

	spdk_bdev_free_io(bdev_io);

	raid_bdev_io_complete_part(raid_io, 1, success ?
				   SPDK_BDEV_IO_STATUS_SUCCESS :
				   SPDK_BDEV_IO_STATUS_FAILED);
}

/* ===== 读路径：均衡 + 失败切换 + 回写纠正（移植自 raid1.c）===== */
static struct raid_base_bdev_info *
raid1f_get_read_io_base_bdev(struct raid_bdev_io *raid_io)
{
	assert(raid_io->type == SPDK_BDEV_IO_TYPE_READ);
	return &raid_io->raid_bdev->base_bdev_info[raid_io->base_bdev_io_submitted];
}

static void
raid1f_correct_read_error_completion(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct raid_bdev_io *raid_io = cb_arg;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		struct raid_base_bdev_info *base_info;

		base_info = raid1f_get_read_io_base_bdev(raid_io);
		/* 回写纠正失败：摘除该副本，但读 IO 仍以成功完成 */
		raid_bdev_fail_base_bdev(base_info);
	}

	raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_SUCCESS);
}

/* TODO(MWL Task4): 回写纠正必须走 MWL 定序写路径（intent -> lead -> 其余），
 * 骨架阶段直接写原副本，与上游 raid1 行为一致。 */
static void
raid1f_correct_read_error(void *_raid_io)
{
	struct raid_bdev_io *raid_io = _raid_io;
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct spdk_bdev_ext_io_opts io_opts;
	struct raid_base_bdev_info *base_info;
	struct spdk_io_channel *base_ch;
	uint8_t i;
	int ret;

	i = raid_io->base_bdev_io_submitted;
	base_info = &raid_bdev->base_bdev_info[i];
	base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, i);
	assert(base_ch != NULL);

	raid1f_init_ext_io_opts(&io_opts, raid_io);
	ret = raid_bdev_writev_blocks_ext(base_info, base_ch, raid_io->iovs, raid_io->iovcnt,
					  raid_io->offset_blocks, raid_io->num_blocks,
					  raid1f_correct_read_error_completion, raid_io,
					  &io_opts);
	if (spdk_unlikely(ret != 0)) {
		if (ret == -ENOMEM) {
			raid_bdev_queue_io_wait(raid_io,
				spdk_bdev_desc_get_bdev(base_info->desc),
				base_ch, raid1f_correct_read_error);
		} else {
			raid_bdev_fail_base_bdev(base_info);
			raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_SUCCESS);
		}
	}
}

static void raid1f_read_other_base_bdev(void *_raid_io);

static void
raid1f_read_other_completion(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct raid_bdev_io *raid_io = cb_arg;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		assert(raid_io->base_bdev_io_remaining > 0);
		raid_io->base_bdev_io_remaining--;
		raid1f_read_other_base_bdev(raid_io);
		return;
	}

	raid1f_correct_read_error(raid_io);
}

static void
raid1f_read_other_base_bdev(void *_raid_io)
{
	struct raid_bdev_io *raid_io = _raid_io;
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct spdk_bdev_ext_io_opts io_opts;
	struct raid_base_bdev_info *base_info;
	struct spdk_io_channel *base_ch;
	uint8_t i;
	int ret;

	for (i = raid_bdev->num_base_bdevs - raid_io->base_bdev_io_remaining;
	     i < raid_bdev->num_base_bdevs; i++) {
		base_info = &raid_bdev->base_bdev_info[i];
		base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, i);

		if (base_ch == NULL || i == raid_io->base_bdev_io_submitted) {
			raid_io->base_bdev_io_remaining--;
			continue;
		}

		raid1f_init_ext_io_opts(&io_opts, raid_io);
		ret = raid_bdev_readv_blocks_ext(base_info, base_ch, raid_io->iovs,
						 raid_io->iovcnt,
						 raid_io->offset_blocks,
						 raid_io->num_blocks,
						 raid1f_read_other_completion, raid_io,
						 &io_opts);
		if (spdk_unlikely(ret != 0)) {
			if (ret == -ENOMEM) {
				raid_bdev_queue_io_wait(raid_io,
					spdk_bdev_desc_get_bdev(base_info->desc),
					base_ch, raid1f_read_other_base_bdev);
			} else {
				break;
			}
		}
		return;
	}

	base_info = raid1f_get_read_io_base_bdev(raid_io);
	raid_bdev_fail_base_bdev(base_info);

	raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
}

static void
raid1f_read_bdev_io_completion(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct raid_bdev_io *raid_io = cb_arg;

	spdk_bdev_free_io(bdev_io);

	raid1f_channel_dec_read_counters(raid_io->raid_ch,
					 raid_io->base_bdev_io_submitted,
					 raid_io->num_blocks);

	if (!success) {
		raid_io->base_bdev_io_remaining = raid_io->raid_bdev->num_base_bdevs;
		raid1f_read_other_base_bdev(raid_io);
		return;
	}

	raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_SUCCESS);
}

static void raid1f_submit_rw_request(struct raid_bdev_io *raid_io);

static void
_raid1f_submit_rw_request(void *_raid_io)
{
	struct raid_bdev_io *raid_io = _raid_io;

	raid1f_submit_rw_request(raid_io);
}

static uint8_t
raid1f_channel_next_read_base_bdev(struct raid_bdev *raid_bdev,
				   struct raid_bdev_io_channel *raid_ch)
{
	struct poweraid_raid1f_io_channel *ch = raid_bdev_channel_get_module_ctx(raid_ch);
	uint64_t min_outstanding = UINT64_MAX;
	uint8_t idx, min_idx = UINT8_MAX;

	for (idx = 0; idx < raid_bdev->num_base_bdevs; idx++) {
		if (raid_bdev_channel_get_base_channel(raid_ch, idx) == NULL) {
			continue;
		}
		if (ch->read_blocks_outstanding[idx] < min_outstanding) {
			min_outstanding = ch->read_blocks_outstanding[idx];
			min_idx = idx;
		}
	}

	return min_idx;
}

static int
raid1f_submit_read_request(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct raid_bdev_io_channel *raid_ch = raid_io->raid_ch;
	struct spdk_bdev_ext_io_opts io_opts;
	struct raid_base_bdev_info *base_info;
	struct spdk_io_channel *base_ch;
	uint8_t idx;
	int ret;

	idx = raid1f_channel_next_read_base_bdev(raid_bdev, raid_ch);
	if (spdk_unlikely(idx == UINT8_MAX)) {
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
		return 0;
	}

	base_info = &raid_bdev->base_bdev_info[idx];
	base_ch = raid_bdev_channel_get_base_channel(raid_ch, idx);

	raid1f_init_ext_io_opts(&io_opts, raid_io);
	ret = raid_bdev_readv_blocks_ext(base_info, base_ch, raid_io->iovs,
					 raid_io->iovcnt,
					 raid_io->offset_blocks, raid_io->num_blocks,
					 raid1f_read_bdev_io_completion, raid_io,
					 &io_opts);

	if (spdk_likely(ret == 0)) {
		raid1f_channel_inc_read_counters(raid_ch, idx, raid_io->num_blocks);
		raid_io->base_bdev_io_submitted = idx;
	} else if (spdk_unlikely(ret == -ENOMEM)) {
		raid_bdev_queue_io_wait(raid_io, spdk_bdev_desc_get_bdev(base_info->desc),
					base_ch, _raid1f_submit_rw_request);
		return 0;
	}

	return ret;
}

/* TODO(MWL Task4): 替换为 intent FUA -> lead 数据 -> 其余数据 -> commit 的
 * 定序写 FSM。骨架阶段所有在场副本并行写。 */
static int
raid1f_submit_write_request(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct spdk_bdev_ext_io_opts io_opts;
	struct raid_base_bdev_info *base_info;
	struct spdk_io_channel *base_ch;
	uint8_t idx;
	uint64_t base_bdev_io_not_submitted;
	int ret = 0;

	if (raid_io->base_bdev_io_submitted == 0) {
		raid_io->base_bdev_io_remaining = raid_bdev->num_base_bdevs;
		raid_bdev_io_set_default_status(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
	}

	raid1f_init_ext_io_opts(&io_opts, raid_io);
	for (idx = raid_io->base_bdev_io_submitted;
	     idx < raid_bdev->num_base_bdevs; idx++) {
		base_info = &raid_bdev->base_bdev_info[idx];
		base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, idx);

		if (base_ch == NULL) {
			/* 跳过缺槽（降级）：该副本计失败 */
			raid_io->base_bdev_io_submitted++;
			raid_bdev_io_complete_part(raid_io, 1, SPDK_BDEV_IO_STATUS_FAILED);
			continue;
		}

		ret = raid_bdev_writev_blocks_ext(base_info, base_ch, raid_io->iovs,
						  raid_io->iovcnt,
						  raid_io->offset_blocks,
						  raid_io->num_blocks,
						  raid1f_write_bdev_io_completion,
						  raid_io, &io_opts);
		if (spdk_unlikely(ret != 0)) {
			if (spdk_unlikely(ret == -ENOMEM)) {
				raid_bdev_queue_io_wait(raid_io,
					spdk_bdev_desc_get_bdev(base_info->desc),
					base_ch, _raid1f_submit_rw_request);
				return 0;
			}

			base_bdev_io_not_submitted = raid_bdev->num_base_bdevs -
						     raid_io->base_bdev_io_submitted;
			raid_bdev_io_complete_part(raid_io, base_bdev_io_not_submitted,
						   SPDK_BDEV_IO_STATUS_FAILED);
			return 0;
		}

		raid_io->base_bdev_io_submitted++;
	}

	if (raid_io->base_bdev_io_submitted == 0) {
		ret = -ENODEV;
	}

	return ret;
}

static void
raid1f_submit_rw_request(struct raid_bdev_io *raid_io)
{
	struct poweraid_raid_common_raid *raid = raid_io->raid_bdev->module_private;
	int ret;

	/* FSM ONLINE 前（建卷 sb 落盘/重启校验中）或 Task 5 起 MWL replay
	 * RESTORING 期间，返回 NOMEM 由 bdev 层排队重试（与 5f 同策略）。*/
	if (raid == NULL ||
	    poweraid_raid_state_test(&raid->state, POWERAID_RAID_ST_RESTORING) ||
	    !poweraid_raid_state_test(&raid->state, POWERAID_RAID_ST_ONLINE)) {
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_NOMEM);
		return;
	}

	switch (raid_io->type) {
	case SPDK_BDEV_IO_TYPE_READ:
		ret = raid1f_submit_read_request(raid_io);
		break;
	case SPDK_BDEV_IO_TYPE_WRITE:
		ret = raid1f_submit_write_request(raid_io);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (spdk_unlikely(ret != 0)) {
		raid_bdev_io_complete(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
	}
}

/* ===== unmap / flush ===== */
static void raid1f_submit_null_payload_request(struct raid_bdev_io *raid_io);

static void
_raid1f_submit_null_payload_request(void *_raid_io)
{
	struct raid_bdev_io *raid_io = _raid_io;

	raid1f_submit_null_payload_request(raid_io);
}

static inline void
raid1f_null_payload_request_io_completion(struct spdk_bdev_io *bdev_io, bool success,
		void *cb_arg)
{
	raid1f_write_bdev_io_completion(bdev_io, success, cb_arg);
}

static void
raid1f_submit_null_payload_request(struct raid_bdev_io *raid_io)
{
	struct raid_bdev *raid_bdev = raid_io->raid_bdev;
	struct spdk_bdev_ext_io_opts io_opts;
	struct raid_base_bdev_info *base_info;
	struct spdk_io_channel *base_ch;
	uint8_t idx;
	uint64_t base_bdev_io_not_submitted;
	int ret = 0;

	if (raid_io->base_bdev_io_submitted == 0) {
		raid_io->base_bdev_io_remaining = raid_bdev->num_base_bdevs;
		raid_bdev_io_set_default_status(raid_io, SPDK_BDEV_IO_STATUS_FAILED);
	}

	raid1f_init_ext_io_opts(&io_opts, raid_io);
	for (idx = raid_io->base_bdev_io_submitted;
	     idx < raid_bdev->num_base_bdevs; idx++) {
		base_info = &raid_bdev->base_bdev_info[idx];
		base_ch = raid_bdev_channel_get_base_channel(raid_io->raid_ch, idx);

		if (base_ch == NULL) {
			raid_io->base_bdev_io_submitted++;
			raid_bdev_io_complete_part(raid_io, 1, SPDK_BDEV_IO_STATUS_FAILED);
			continue;
		}

		switch (raid_io->type) {
		case SPDK_BDEV_IO_TYPE_UNMAP:
			ret = raid_bdev_unmap_blocks(base_info, base_ch,
						     raid_io->offset_blocks,
						     raid_io->num_blocks,
						     raid1f_null_payload_request_io_completion,
						     raid_io);
			break;

		case SPDK_BDEV_IO_TYPE_FLUSH:
			ret = raid_bdev_flush_blocks(base_info, base_ch,
						     raid_io->offset_blocks,
						     raid_io->num_blocks,
						     raid1f_null_payload_request_io_completion,
						     raid_io);
			break;

		default:
			SPDK_ERRLOG("submit request, invalid io type with null payload %u\n",
				    raid_io->type);
			assert(false);
			ret = -EIO;
		}

		if (spdk_unlikely(ret != 0)) {
			if (spdk_unlikely(ret == -ENOMEM)) {
				raid_bdev_queue_io_wait(raid_io,
					spdk_bdev_desc_get_bdev(base_info->desc),
					base_ch, _raid1f_submit_null_payload_request);
				return;
			}

			base_bdev_io_not_submitted = raid_bdev->num_base_bdevs -
						     raid_io->base_bdev_io_submitted;
			raid_bdev_io_complete_part(raid_io, base_bdev_io_not_submitted,
						   SPDK_BDEV_IO_STATUS_FAILED);
			return;
		}

		raid_io->base_bdev_io_submitted++;
	}
}

/* ===== io device 生命周期 ===== */
static void
raid1f_ioch_destroy(void *io_device, void *ctx_buf)
{
}

static int
raid1f_ioch_create(void *io_device, void *ctx_buf)
{
	struct poweraid_raid1f_io_channel *ch = ctx_buf;
	struct poweraid_raid_common_raid *raid = io_device;

	memset(ch->read_blocks_outstanding, 0,
	       raid->num_base_bdevs * sizeof(ch->read_blocks_outstanding[0]));
	return 0;
}

/* 分配模块私有上下文（复用 poweraid common raid 骨架，FSM/RPC 与 5f/6f 同构）。
 * 槽位状态初始为 0，由 common FSM（OPEN→TRY_READ_MD→VALIDATE_MD→ONLINE）驱动；
 * Task 2 起 sb v2(F_MWL) 由 FSM 落盘/校验，MWL 日志层 Task 3 接入。 */
static struct poweraid_raid_common_raid *
raid1f_common_ctx_alloc(struct raid_bdev *raid_bdev)
{
	struct poweraid_raid_common_raid *raid;
	struct raid_base_bdev_info *base_info;
	uint8_t i = 0;

	raid = calloc(1, sizeof(*raid));
	if (raid == NULL) {
		return NULL;
	}

	raid->raid_bdev = raid_bdev;
	raid->level = (uint32_t)raid_bdev->level;
	raid->strip_size = raid_bdev->strip_size;
	raid->block_size = raid_bdev->bdev.blocklen;
	raid->num_base_bdevs = raid_bdev->num_base_bdevs;
	raid->num_parity = 0;  /* 镜像卷无 parity；get_info 按 level 判级 */
	spdk_uuid_copy(&raid->uuid, &raid_bdev->bdev.uuid);
	snprintf(raid->name, sizeof(raid->name), "%s", raid_bdev->bdev.name);

	raid->base_bdevs = calloc(raid->num_base_bdevs, sizeof(*raid->base_bdevs));
	if (raid->base_bdevs == NULL) {
		free(raid);
		return NULL;
	}

	RAID_FOR_EACH_BASE_BDEV(raid_bdev, base_info) {
		struct poweraid_raid_common_bdev *cb;

		cb = calloc(1, sizeof(*cb));
		if (cb == NULL) {
			while (i > 0) {
				free(raid->base_bdevs[--i]);
			}
			free(raid->base_bdevs);
			free(raid);
			return NULL;
		}
		cb->slot = i;
		cb->raid = raid;
		cb->desc = base_info->desc;
		spdk_uuid_copy(&cb->uuid, &base_info->uuid);
		raid->base_bdevs[i] = cb;
		i++;
	}

	return raid;
}

static void
raid1f_common_ctx_free(struct poweraid_raid_common_raid *raid)
{
	uint8_t i;

	if (raid == NULL) {
		return;
	}
	if (raid->base_bdevs != NULL) {
		for (i = 0; i < raid->num_base_bdevs; i++) {
			if (raid->base_bdevs[i] != NULL &&
			    raid->base_bdevs[i]->ch != NULL) {
				spdk_put_io_channel(
					(struct spdk_io_channel *)raid->base_bdevs[i]->ch);
			}
			free(raid->base_bdevs[i]);
		}
		free(raid->base_bdevs);
	}
	if (raid->sb_ctx != NULL) {
		poweraid_raid_common_sb_free(raid);
	}
	free(raid);
}

static void
raid1f_io_device_unregister_done(void *io_device)
{
	raid_bdev_module_stop_done(((struct poweraid_raid_common_raid *)io_device)->raid_bdev);
	raid1f_common_ctx_free(io_device);
}

static int
poweraid_raid1f_start(struct raid_bdev *raid_bdev)
{
	struct poweraid_raid_common_raid *raid;
	struct raid_base_bdev_info *base_info;
	uint64_t min_blockcnt = UINT64_MAX;
	uint64_t reserve;
	int rc;

	SPDK_NOTICELOG("raid1f: start raid=%s num_base_bdevs=%u\n",
		       raid_bdev->bdev.name, raid_bdev->num_base_bdevs);

	if (raid_bdev->num_base_bdevs < 2) {
		SPDK_ERRLOG("raid1f: need >= 2 base bdevs (got %u)\n",
			    raid_bdev->num_base_bdevs);
		return -EINVAL;
	}

	raid = raid1f_common_ctx_alloc(raid_bdev);
	if (raid == NULL) {
		SPDK_ERRLOG("raid1f: alloc context failed\n");
		return -ENOMEM;
	}

	/* 元数据保留区：LBA0 sb + [1MiB,5MiB) MWL 区，数据区从 5MiB 起
	 * （4K 块下 = 1280 块），与 5f/6f 的 PPL 几何同构。superblock=false
	 * 路径下框架不管理 data_offset，由模块在 base_info 上设置，
	 * raid_bdev_{read,write}v_blocks_ext 会自动叠加该偏移。*/
	reserve = (POWERAID_RAID_COMMON_MWL_REGION_OFFSET +
		   POWERAID_RAID_COMMON_MWL_REGION_SIZE) / raid->block_size;
	raid->data_offset_blocks = reserve;

	RAID_FOR_EACH_BASE_BDEV(raid_bdev, base_info) {
		struct spdk_bdev *base_bdev = spdk_bdev_desc_get_bdev(base_info->desc);
		uint64_t data_size;

		if (base_bdev->blockcnt <= reserve) {
			SPDK_ERRLOG("raid1f: base bdev %s too small (%"PRIu64
				    " <= reserve %"PRIu64")\n",
				    base_bdev->name, base_bdev->blockcnt, reserve);
			rc = -EINVAL;
			goto err_free;
		}
		data_size = base_bdev->blockcnt - reserve;
		base_info->data_offset = reserve;
		base_info->data_size = data_size;
		min_blockcnt = spdk_min(min_blockcnt, data_size);
	}

	raid->raid_size = min_blockcnt;
	raid_bdev->bdev.blockcnt = min_blockcnt;
	raid_bdev->module_private = raid;

	spdk_io_device_register(raid, raid1f_ioch_create, raid1f_ioch_destroy,
				sizeof(struct poweraid_raid1f_io_channel) +
				raid_bdev->num_base_bdevs * sizeof(uint64_t),
				NULL);

	/* 触发 common FSM：CREATE_DSC → sb_alloc/init(F_MWL) → OPEN_BDEVS →
	 * 读盘校验/新卷写 sb → ONLINE（Task 5 起 ONLINE 前插入 MWL replay）。 */
	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_RAID, raid,
				   POWERAID_RAID_EV_CREATE_DSC);
	return 0;

err_free:
	raid1f_common_ctx_free(raid);
	return rc;
}

static bool
poweraid_raid1f_stop(struct raid_bdev *raid_bdev)
{
	struct poweraid_raid_common_raid *raid = raid_bdev->module_private;

	if (raid == NULL) {
		return true;  /* 无 FSM 对象，同步完成 */
	}

	/* FSM 置 OFFLINE（停止接受 IO）；stop_done 在 io device 注销回调中发出 */
	poweraid_raid_common_sm_process(POWERAID_FSM_LAYER_RAID, raid,
				   POWERAID_RAID_EV_OFFLINE);

	raid_bdev->module_private = NULL;
	spdk_io_device_unregister(raid, raid1f_io_device_unregister_done);

	return false;
}

static struct spdk_io_channel *
poweraid_raid1f_get_io_channel(struct raid_bdev *raid_bdev)
{
	return spdk_get_io_channel(raid_bdev->module_private);
}

/* ===== 通用 process 重建（镜像拷贝：读健康副本 -> 写目标成员）。
 * Task 7 在此之上接入 acting lead 反向重建与 poweraid hooks。 */
static void
raid1f_process_write_completed(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct raid_bdev_process_request *process_req = cb_arg;

	spdk_bdev_free_io(bdev_io);
	raid_bdev_process_request_complete(process_req, success ? 0 : -EIO);
}

static void raid1f_process_submit_write(struct raid_bdev_process_request *process_req);

static void
_raid1f_process_submit_write(void *ctx)
{
	struct raid_bdev_process_request *process_req = ctx;

	raid1f_process_submit_write(process_req);
}

static void
raid1f_process_submit_write(struct raid_bdev_process_request *process_req)
{
	struct raid_bdev_io *raid_io = &process_req->raid_io;
	struct spdk_bdev_ext_io_opts io_opts;
	int ret;

	raid1f_init_ext_io_opts(&io_opts, raid_io);
	ret = raid_bdev_writev_blocks_ext(process_req->target, process_req->target_ch,
					  raid_io->iovs, raid_io->iovcnt,
					  raid_io->offset_blocks, raid_io->num_blocks,
					  raid1f_process_write_completed, process_req,
					  &io_opts);
	if (spdk_unlikely(ret != 0)) {
		if (ret == -ENOMEM) {
			raid_bdev_queue_io_wait(raid_io,
				spdk_bdev_desc_get_bdev(process_req->target->desc),
				process_req->target_ch, _raid1f_process_submit_write);
		} else {
			raid_bdev_process_request_complete(process_req, ret);
		}
	}
}

static void
raid1f_process_read_completed(struct raid_bdev_io *raid_io,
			      enum spdk_bdev_io_status status)
{
	struct raid_bdev_process_request *process_req = SPDK_CONTAINEROF(raid_io,
			struct raid_bdev_process_request, raid_io);

	if (status != SPDK_BDEV_IO_STATUS_SUCCESS) {
		raid_bdev_process_request_complete(process_req, -EIO);
		return;
	}

	raid1f_process_submit_write(process_req);
}

static int
poweraid_raid1f_submit_process_request(struct raid_bdev_process_request *process_req,
				       struct raid_bdev_io_channel *raid_ch)
{
	struct raid_bdev_io *raid_io = &process_req->raid_io;
	int ret;

	raid_bdev_io_init(raid_io, raid_ch, SPDK_BDEV_IO_TYPE_READ,
			  process_req->offset_blocks, process_req->num_blocks,
			  &process_req->iov, 1, process_req->md_buf, NULL, NULL);
	raid_io->completion_cb = raid1f_process_read_completed;

	ret = raid1f_submit_read_request(raid_io);
	if (spdk_likely(ret == 0)) {
		return process_req->num_blocks;
	} else if (ret < 0) {
		return ret;
	} else {
		return -EINVAL;
	}
}

static bool
poweraid_raid1f_resize(struct raid_bdev *raid_bdev)
{
	int rc;
	uint64_t min_blockcnt = UINT64_MAX;
	struct raid_base_bdev_info *base_info;

	RAID_FOR_EACH_BASE_BDEV(raid_bdev, base_info) {
		struct spdk_bdev *base_bdev;

		if (base_info->desc == NULL) {
			continue;
		}
		base_bdev = spdk_bdev_desc_get_bdev(base_info->desc);
		min_blockcnt = spdk_min(min_blockcnt,
					base_bdev->blockcnt - base_info->data_offset);
	}

	if (min_blockcnt == raid_bdev->bdev.blockcnt) {
		return false;
	}

	rc = spdk_bdev_notify_blockcnt_change(&raid_bdev->bdev, min_blockcnt);
	if (rc != 0) {
		SPDK_ERRLOG("Failed to notify blockcount change\n");
		return false;
	}

	RAID_FOR_EACH_BASE_BDEV(raid_bdev, base_info) {
		base_info->data_size = min_blockcnt;
	}
	return true;
}

static struct raid_bdev_module g_poweraid_raid1f_module = {
	.level = SPDK_BDEV_RAID_LEVEL_RAID1F,
	.base_bdevs_min = 2,
	.base_bdevs_constraint = {CONSTRAINT_MIN_BASE_BDEVS_OPERATIONAL, 1},
	.memory_domains_supported = true,
	.start = poweraid_raid1f_start,
	.stop = poweraid_raid1f_stop,
	.submit_rw_request = raid1f_submit_rw_request,
	.submit_null_payload_request = raid1f_submit_null_payload_request,
	.get_io_channel = poweraid_raid1f_get_io_channel,
	.submit_process_request = poweraid_raid1f_submit_process_request,
	.resize = poweraid_raid1f_resize,
};
RAID_MODULE_REGISTER(&g_poweraid_raid1f_module)

SPDK_LOG_REGISTER_COMPONENT(poweraid_raid1f)
