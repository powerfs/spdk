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
#include "poweraid_raid5f.h"
#include "poweraid_raid5f_merge.h"

SPDK_LOG_REGISTER_COMPONENT(raid5f_rpc);

/* ===== set_merge_delay ===== */

struct rpc_set_merge_delay {
	char		*name;
	uint64_t	delay_us;
	struct spdk_jsonrpc_request *request;
};

static const struct spdk_json_object_decoder rpc_set_merge_delay_decoders[] = {
	{"name", offsetof(struct rpc_set_merge_delay, name), spdk_json_decode_string},
	{"delay_us", offsetof(struct rpc_set_merge_delay, delay_us), spdk_json_decode_uint64, true},
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
	struct poweraid_raid5f_io_channel *ch =
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
rpc_poweraid_raid5f_set_merge_delay(struct spdk_jsonrpc_request *request,
				    const struct spdk_json_val *params)
{
	struct rpc_set_merge_delay *req;
	struct raid_bdev *raid_bdev;
	struct poweraid_raid5f_raid *raid;

	req = calloc(1, sizeof(*req));
	if (req == NULL) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, spdk_strerror(ENOMEM));
		return;
	}

	if (spdk_json_decode_object(params, rpc_set_merge_delay_decoders,
				    SPDK_COUNTOF(rpc_set_merge_delay_decoders), req)) {
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
		  rpc_poweraid_raid5f_set_merge_delay, SPDK_RPC_RUNTIME)

/* ===== get_merge_delay ===== */

struct rpc_get_merge_delay {
	char	*name;
};

static const struct spdk_json_object_decoder rpc_get_merge_delay_decoders[] = {
	{"name", offsetof(struct rpc_get_merge_delay, name), spdk_json_decode_string},
};

static void
rpc_poweraid_raid5f_get_merge_delay(struct spdk_jsonrpc_request *request,
				    const struct spdk_json_val *params)
{
	struct rpc_get_merge_delay req = {};
	struct raid_bdev *raid_bdev;
	struct poweraid_raid5f_raid *raid;
	struct spdk_json_write_ctx *w;

	if (spdk_json_decode_object(params, rpc_get_merge_delay_decoders,
				    SPDK_COUNTOF(rpc_get_merge_delay_decoders), &req)) {
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
		  rpc_poweraid_raid5f_get_merge_delay, SPDK_RPC_RUNTIME)
