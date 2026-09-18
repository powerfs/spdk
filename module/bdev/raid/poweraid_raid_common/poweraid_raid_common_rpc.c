/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   poweraid_raid5f JSON-RPC 方法（Stage 3c）
 *
 *   bdev_poweraid_raid5f_set_merge_delay：设置合并层延迟。
 *     delay_us=0 表示 merge OFF（submit 即 flush，纯 RMW 基线）。
 *     修改 raid->delay_us 并遍历既有 io channel 更新 merge_ctx。
 *     JSON conf 场景（fio 内嵌 SPDK）：在 bdev_raid_create 之后调用，
 *     此时 channel 尚未创建，merge_init 从 raid->delay_us 读取。
 *
 *   bdev_poweraid_raid5f_get_merge_delay：查询当前延迟配置。
 */

#include "spdk/stdinc.h"
#include "spdk/rpc.h"
#include "spdk/log.h"
#include "spdk/string.h"
#include "spdk/util.h"
#include "spdk/thread.h"

#include "../bdev_raid.h"
#include "spdk/bdev.h"
#include "poweraid_raid_common.h"
#include "poweraid_raid_common_merge.h"
#include "poweraid_raid_common_sm.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_rpc);

/* ===== set_merge_delay ===== */

struct rpc_set_merge_delay {
	char		*name;
	uint64_t	delay_us;
	struct spdk_jsonrpc_request *request;
};

static const struct spdk_json_object_decoder rpc_bdev_poweraid_raid5f_set_merge_delay_decoders_manual[] = {
	{"name", offsetof(struct rpc_set_merge_delay, name), spdk_json_decode_string},
	{"delay_us", offsetof(struct rpc_set_merge_delay, delay_us), spdk_json_decode_uint64},
};

static void
rpc_free_set_merge_delay(struct rpc_set_merge_delay *req)
{
	free(req->name);
	free(req);
}

static void
set_merge_delay_channel(struct spdk_io_channel_iter *i)
{
	struct rpc_set_merge_delay *req = spdk_io_channel_iter_get_ctx(i);
	struct poweraid_raid_common_io_channel *ch =
		spdk_io_channel_get_ctx(spdk_io_channel_iter_get_channel(i));

	ch->merge_ctx.delay_us = req->delay_us;
	spdk_for_each_channel_continue(i, 0);
}

static void
set_merge_delay_done(struct spdk_io_channel_iter *i, int status)
{
	struct rpc_set_merge_delay *req = spdk_io_channel_iter_get_ctx(i);

	if (status != 0) {
		spdk_jsonrpc_send_error_response_fmt(req->request, status,
						     "Failed to set merge delay on some channels");
	} else {
		SPDK_NOTICELOG("merge delay set: raid=%s delay_us=%"PRIu64"\n",
			       req->name, req->delay_us);
		spdk_jsonrpc_send_bool_response(req->request, true);
	}
	rpc_free_set_merge_delay(req);
}

static void
rpc_bdev_poweraid_raid5f_set_merge_delay(struct spdk_jsonrpc_request *request,
					 const struct spdk_json_val *params)
{
	struct rpc_set_merge_delay *req;
	struct raid_bdev *raid_bdev;
	struct poweraid_raid_common_raid *raid;

	req = calloc(1, sizeof(*req));
	if (req == NULL) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, spdk_strerror(ENOMEM));
		return;
	}

	if (spdk_json_decode_object(params, rpc_bdev_poweraid_raid5f_set_merge_delay_decoders_manual,
				    SPDK_COUNTOF(rpc_bdev_poweraid_raid5f_set_merge_delay_decoders_manual), req)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		rpc_free_set_merge_delay(req);
		return;
	}
	req->request = request;

	raid_bdev = raid_bdev_find_by_name(req->name);
	if (raid_bdev == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV,
						     "RAID bdev %s not found", req->name);
		rpc_free_set_merge_delay(req);
		return;
	}
	raid = raid_bdev->module_private;
	if (raid == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV,
						     "RAID bdev %s is not a poweraid_raid5f device",
						     req->name);
		rpc_free_set_merge_delay(req);
		return;
	}

	/* 先写 raid 级配置（后续新建 channel 从此读取），再遍历既有 channel */
	raid->delay_us = req->delay_us;
	spdk_for_each_channel(raid, set_merge_delay_channel, req, set_merge_delay_done);
}
SPDK_RPC_REGISTER("bdev_poweraid_raid5f_set_merge_delay",
		  rpc_bdev_poweraid_raid5f_set_merge_delay, SPDK_RPC_RUNTIME)

/* ===== get_merge_delay ===== */

struct rpc_get_merge_delay {
	char	*name;
};

static const struct spdk_json_object_decoder rpc_bdev_poweraid_raid5f_get_merge_delay_decoders_manual[] = {
	{"name", offsetof(struct rpc_get_merge_delay, name), spdk_json_decode_string},
};

static void
rpc_bdev_poweraid_raid5f_get_merge_delay(struct spdk_jsonrpc_request *request,
					 const struct spdk_json_val *params)
{
	struct rpc_get_merge_delay req = {};
	struct raid_bdev *raid_bdev;
	struct poweraid_raid_common_raid *raid;
	struct spdk_json_write_ctx *w;

	if (spdk_json_decode_object(params, rpc_bdev_poweraid_raid5f_get_merge_delay_decoders_manual,
				    SPDK_COUNTOF(rpc_bdev_poweraid_raid5f_get_merge_delay_decoders_manual), &req)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		return;
	}

	raid_bdev = raid_bdev_find_by_name(req.name);
	if (raid_bdev == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV,
						     "RAID bdev %s not found", req.name);
		free(req.name);
		return;
	}
	raid = raid_bdev->module_private;
	free(req.name);
	if (raid == NULL) {
		spdk_jsonrpc_send_error_response(request, -ENODEV,
						 "not a poweraid_raid5f device");
		return;
	}

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "name", raid->name);
	spdk_json_write_named_uint64(w, "delay_us", raid->delay_us);
	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(request, w);
}
SPDK_RPC_REGISTER("bdev_poweraid_raid5f_get_merge_delay",
		  rpc_bdev_poweraid_raid5f_get_merge_delay, SPDK_RPC_RUNTIME)

/* ===== replace_base_bdev（Stage 4 步骤 7）=====
 *
 * 在指定空槽位加入替换成员盘（要求 raid 处于 ONLINE/DEGRADED 且无活动重建）。
 * 框架对无有效 superblock 的新盘自动触发 rebuild；本方法仅完成"指定槽位加入"。
 */

struct rpc_replace_base_bdev {
	char		*name;		/* raid bdev 名 */
	uint32_t	slot;
	char		*bdev;		/* 替换用 base bdev 名 */
	struct spdk_jsonrpc_request *request;
};

static const struct spdk_json_object_decoder rpc_replace_base_bdev_decoders[] = {
	{"name", offsetof(struct rpc_replace_base_bdev, name), spdk_json_decode_string},
	{"slot", offsetof(struct rpc_replace_base_bdev, slot), spdk_json_decode_uint32},
	{"bdev", offsetof(struct rpc_replace_base_bdev, bdev), spdk_json_decode_string},
};

static void
replace_base_bdev_action_cb(void *ctx, int rc)
{
	struct rpc_replace_base_bdev *req = ctx;

	if (rc != 0) {
		spdk_jsonrpc_send_error_response_fmt(req->request, rc,
						     "Failed to add bdev %s at slot %u: %s",
						     req->bdev, req->slot, spdk_strerror(-rc));
	} else {
		SPDK_NOTICELOG("replace: raid=%s slot=%u bdev=%s added, rebuild triggered\n",
			       req->name, req->slot, req->bdev);
		spdk_jsonrpc_send_bool_response(req->request, true);
	}
	free(req->name);
	free(req->bdev);
	free(req);
}

static void
rpc_bdev_poweraid_raid_replace_base_bdev(struct spdk_jsonrpc_request *request,
		const struct spdk_json_val *params)
{
	struct rpc_replace_base_bdev *req;
	struct raid_bdev *raid_bdev;
	struct poweraid_raid_common_raid *raid;
	struct raid_base_bdev_info *base_info;
	struct spdk_bdev *new_bdev, *ref_bdev;
	struct spdk_bdev_desc *ref_desc;
	uint8_t i;
	int rc;

	req = calloc(1, sizeof(*req));
	if (req == NULL) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, spdk_strerror(ENOMEM));
		return;
	}
	if (spdk_json_decode_object(params, rpc_replace_base_bdev_decoders,
				    SPDK_COUNTOF(rpc_replace_base_bdev_decoders), req)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		free(req);
		return;
	}
	req->request = request;

	raid_bdev = raid_bdev_find_by_name(req->name);
	if (raid_bdev == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV,
						     "RAID bdev %s not found", req->name);
		goto out;
	}
	raid = raid_bdev->module_private;
	if (raid == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -EINVAL,
						     "%s is not a poweraid raid bdev", req->name);
		goto out;
	}
	if (req->slot >= raid_bdev->num_base_bdevs) {
		spdk_jsonrpc_send_error_response_fmt(request, -EINVAL,
						     "slot %u out of range (num=%u)",
						     req->slot, raid_bdev->num_base_bdevs);
		goto out;
	}
	if (raid_bdev->state != SPDK_BDEV_RAID_STATE_ONLINE) {
		spdk_jsonrpc_send_error_response_fmt(request, -EBUSY,
						     "raid %s is not online", req->name);
		goto out;
	}
	if (raid_bdev->process != NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -EBUSY,
						     "raid %s has a rebuild in progress", req->name);
		goto out;
	}
	base_info = &raid_bdev->base_bdev_info[req->slot];
	if (base_info->desc != NULL || base_info->name != NULL ||
	    !spdk_uuid_is_null(&base_info->uuid)) {
		spdk_jsonrpc_send_error_response_fmt(request, -EINVAL,
						     "slot %u of raid %s is not empty",
						     req->slot, req->name);
		goto out;
	}

	new_bdev = spdk_bdev_get_by_name(req->bdev);
	if (new_bdev == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV,
						     "bdev %s not found", req->bdev);
		goto out;
	}

	/* 容量不得小于任一在用成员盘 */
	for (i = 0; i < raid_bdev->num_base_bdevs; i++) {
		if (raid_bdev->base_bdev_info[i].desc == NULL) {
			continue;
		}
		ref_desc = raid_bdev->base_bdev_info[i].desc;
		ref_bdev = spdk_bdev_desc_get_bdev(ref_desc);
		if (new_bdev->blockcnt < ref_bdev->blockcnt) {
			spdk_jsonrpc_send_error_response_fmt(request, -EINVAL,
				"bdev %s too small: %"PRIu64" blocks < member %"PRIu64,
				req->bdev, new_bdev->blockcnt, ref_bdev->blockcnt);
			goto out;
		}
	}

	rc = raid_bdev_add_base_bdev_at_slot(raid_bdev, (uint8_t)req->slot,
					     req->bdev,
					     replace_base_bdev_action_cb, req);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response_fmt(request, rc,
						     "add_base_bdev_at_slot failed: %s",
						     spdk_strerror(-rc));
		free(req->name);
		free(req->bdev);
		free(req);
	}
	return;

out:
	free(req->name);
	free(req->bdev);
	free(req);
}
SPDK_RPC_REGISTER("bdev_poweraid_raid_replace_base_bdev",
		  rpc_bdev_poweraid_raid_replace_base_bdev, SPDK_RPC_RUNTIME)

/* ===== get_info（状态/重建标志/每槽位视图）===== */

struct rpc_poweraid_raid_get_info {
	char	*name;
};

static const struct spdk_json_object_decoder rpc_poweraid_raid_get_info_decoders[] = {
	{"name", offsetof(struct rpc_poweraid_raid_get_info, name), spdk_json_decode_string},
};

static void
rpc_bdev_poweraid_raid_get_info(struct spdk_jsonrpc_request *request,
				const struct spdk_json_val *params)
{
	struct rpc_poweraid_raid_get_info req = {};
	struct raid_bdev *raid_bdev;
	struct poweraid_raid_common_raid *raid;
	struct spdk_json_write_ctx *w;
	uint8_t i;

	if (spdk_json_decode_object(params, rpc_poweraid_raid_get_info_decoders,
				    SPDK_COUNTOF(rpc_poweraid_raid_get_info_decoders), &req)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		return;
	}

	raid_bdev = raid_bdev_find_by_name(req.name);
	if (raid_bdev == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV,
						     "RAID bdev %s not found", req.name);
		free(req.name);
		return;
	}
	raid = raid_bdev->module_private;
	if (raid == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -EINVAL,
						     "%s is not a poweraid raid bdev", req.name);
		free(req.name);
		return;
	}

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "name", raid->name);
	spdk_json_write_named_uint32(w, "num_base_bdevs", raid_bdev->num_base_bdevs);
	/* raid1f 为镜像卷（num_parity=0），不能按 num_parity 判级；5f/6f 维持原映射。
	 * Task 8 将以字符串级别（raid_level）替代数字编码。 */
	if (raid->level == SPDK_BDEV_RAID_LEVEL_RAID1F) {
		spdk_json_write_named_uint32(w, "level", 1);
	} else {
		spdk_json_write_named_uint32(w, "level", raid->num_parity == 2 ? 6 : 5);
	}
	spdk_json_write_named_uint64(w, "state", raid->state);
	spdk_json_write_named_bool(w, "online",
		poweraid_raid_state_test(&raid->state, POWERAID_RAID_ST_ONLINE));
	spdk_json_write_named_bool(w, "degraded",
		poweraid_raid_state_test(&raid->state, POWERAID_RAID_ST_DEGRADED));
	spdk_json_write_named_bool(w, "degraded2",
		poweraid_raid_state_test(&raid->state, POWERAID_RAID_ST_DEGRADED2));
	spdk_json_write_named_bool(w, "reconstructing",
		poweraid_raid_state_test(&raid->state, POWERAID_RAID_ST_RECON));
	spdk_json_write_named_bool(w, "process_active", raid_bdev->process != NULL);

	spdk_json_write_named_array_begin(w, "base_bdevs");
	for (i = 0; i < raid_bdev->num_base_bdevs; i++) {
		struct raid_base_bdev_info *info = &raid_bdev->base_bdev_info[i];
		struct poweraid_raid_common_bdev *cb = raid->base_bdevs[i];

		spdk_json_write_object_begin(w);
		spdk_json_write_named_uint32(w, "slot", i);
		spdk_json_write_named_string(w, "bdev", info->name != NULL ? info->name : "");
		spdk_json_write_named_bool(w, "present", info->desc != NULL);
		spdk_json_write_named_bool(w, "process_target", info->is_process_target);
		spdk_json_write_named_uint64(w, "state", cb != NULL ? cb->state : 0);
		spdk_json_write_named_bool(w, "faulted",
			cb != NULL && poweraid_raid_state_test(&cb->state,
							      POWERAID_BDEV_ST_FAULTED));
		spdk_json_write_named_bool(w, "member_online",
			cb != NULL && poweraid_raid_state_test(&cb->state,
							      POWERAID_BDEV_ST_ONLINE));
		spdk_json_write_object_end(w);
	}
	spdk_json_write_array_end(w);

	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(request, w);

	free(req.name);
}
SPDK_RPC_REGISTER("bdev_poweraid_raid_get_info",
		  rpc_bdev_poweraid_raid_get_info, SPDK_RPC_RUNTIME)
