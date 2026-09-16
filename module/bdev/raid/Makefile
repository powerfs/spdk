#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2015 Intel Corporation.
#  All rights reserved.
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

SO_VER := 8
SO_MINOR := 0

CFLAGS += -I$(SPDK_ROOT_DIR)/lib/bdev/
C_SRCS = bdev_raid.c bdev_raid_rpc.c bdev_raid_sb.c raid0.c raid1.c concat.c

ifeq ($(CONFIG_RAID5F),y)
C_SRCS += raid5f.c
endif

ifeq ($(CONFIG_POWERAID_RAID5F),y)
C_SRCS += poweraid_raid5f/poweraid_raid5f.c \
	          poweraid_raid5f/poweraid_raid5f_sm_dispatch.c \
	          poweraid_raid5f/poweraid_raid5f_sm_raid.c \
	          poweraid_raid5f/poweraid_raid5f_sm_bdev.c \
	          poweraid_raid5f/poweraid_raid5f_sm_req.c \
	          poweraid_raid5f/poweraid_raid5f_sb.c \
	          poweraid_raid5f/poweraid_raid5f_ppl.c \
	          poweraid_raid5f/poweraid_raid5f_recovery.c \
	          poweraid_raid5f/poweraid_raid5f_rmw.c
endif

LIBNAME = bdev_raid

SPDK_MAP_FILE = $(SPDK_ROOT_DIR)/mk/spdk_blank.map

include $(SPDK_ROOT_DIR)/mk/spdk.lib.mk
