/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (c) 2026 poweraid. All rights reserved.
 *
 *   三层 FSM 通用分派入口：poweraid_raid5f_sm_process()
 *
 *   分派表本体在各自的 .c 文件中定义（poweraid_raid5f_sm_raid.c /
 *   poweraid_raid5f_sm_bdev.c / poweraid_raid5f_sm_req.c），
 *   本文件仅做边界检查 + 函数指针调用，便于后续单文件增强。
 *
 *   详见 raid5f-enhanced-design.md 第 3.10.6 节
 */

#include "spdk/stdinc.h"
#include "spdk/log.h"

#include "poweraid_raid5f.h"

SPDK_LOG_REGISTER_COMPONENT(bdev_poweraid_raid5f);

void
poweraid_raid5f_sm_process(enum poweraid_raid5f_fsm_layer layer, void *obj,
			  uint32_t event)
{
	SPDK_DEBUGLOG(bdev_poweraid_raid5f,
		      "layer=%d obj=%p event=%u\n", layer, obj, event);

	switch (layer) {
	case POWERAID_FSM_LAYER_RAID:
		if (event < POWERAID_RAID_EV_COUNT_REAL &&
		    poweraid_raid5f_raid_fsm[event] != NULL) {
			poweraid_raid5f_raid_fsm[event](
				(struct poweraid_raid5f_raid *)obj,
				(enum poweraid_raid5f_raid_event)event);
		} else {
			SPDK_ERRLOG("RAID FSM: no handler for event %u (obj=%p)\n",
				    event, obj);
		}
		break;
	case POWERAID_FSM_LAYER_BDEV:
		if (event < POWERAID_BDEV_EV_COUNT &&
		    poweraid_raid5f_bdev_fsm[event] != NULL) {
			poweraid_raid5f_bdev_fsm[event](
				(struct poweraid_raid5f_bdev *)obj,
				(enum poweraid_raid5f_bdev_event)event);
		} else {
			SPDK_ERRLOG("BDEV FSM: no handler for event %u (obj=%p)\n",
				    event, obj);
		}
		break;
	case POWERAID_FSM_LAYER_REQ:
		if (event < POWERAID_REQ_EV_COUNT &&
		    poweraid_raid5f_req_fsm[event] != NULL) {
			poweraid_raid5f_req_fsm[event](
				(struct poweraid_raid5f_req *)obj,
				(enum poweraid_raid5f_req_event)event);
		} else {
			SPDK_ERRLOG("REQ FSM: no handler for event %u (obj=%p)\n",
				    event, obj);
		}
		break;
	default:
		SPDK_ERRLOG("unknown FSM layer %d\n", layer);
		break;
	}
}
