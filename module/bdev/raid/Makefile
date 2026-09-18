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
# 公共层（5f/6f 共用）
C_SRCS += poweraid_raid_common/poweraid_raid_common_sm_dispatch.c \
	          poweraid_raid_common/poweraid_raid_common_sm_raid.c \
	          poweraid_raid_common/poweraid_raid_common_sm_bdev.c \
	          poweraid_raid_common/poweraid_raid_common_sm_req.c \
	          poweraid_raid_common/poweraid_raid_common_sb.c \
		  poweraid_raid_common/poweraid_raid_common_ppl.c \
		  poweraid_raid_common/poweraid_raid_common_mwl.c \
		  poweraid_raid_common/poweraid_raid_common_recovery.c \
	          poweraid_raid_common/poweraid_raid_common_rmw.c \
	          poweraid_raid_common/poweraid_raid_common_merge.c \
	          poweraid_raid_common/poweraid_raid_common_rebuild.c \
	          poweraid_raid_common/poweraid_raid_common_rpc.c \
	          poweraid_raid_common/poweraid_raid_common_gf8.c
# GF8 SIMD 路径: 按架构选择
ifeq ($(shell uname -m),x86_64)
C_SRCS += poweraid_raid_common/poweraid_raid_common_gf8_x86.c
else ifeq ($(shell uname -m),aarch64)
C_SRCS += poweraid_raid_common/poweraid_raid_common_gf8_neon.c
endif
# 5f 薄壳
C_SRCS += poweraid_raid5f/poweraid_raid5f.c
# 1f 薄壳（镜像 + MWL 写意图日志，复用 common 层 sb/sm 骨架）
C_SRCS += poweraid_raid1f/poweraid_raid1f.c
# 6f 薄壳 (RAID6 P+Q)
C_SRCS += poweraid_raid6f/poweraid_raid6f.c \
          poweraid_raid6f/poweraid_raid6f_calc.c \
          poweraid_raid6f/poweraid_raid6f_recovery.c
endif

LIBNAME = bdev_raid

SPDK_MAP_FILE = $(SPDK_ROOT_DIR)/mk/spdk_blank.map

include $(SPDK_ROOT_DIR)/mk/spdk.lib.mk
