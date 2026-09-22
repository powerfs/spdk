#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2026 poweraid.
#  All rights reserved.
#
#  poweraid_raid6f 增强 RAID6F 模块（独立子目录，用于单元测试静态库构建）
#  Stage 4 RAID6 (P+Q)：本目录含 6f 薄壳 + CALC，公共层在 ../poweraid_raid_common/

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../../../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

SO_VER := 8
SO_MINOR := 0

CFLAGS += -I$(SPDK_ROOT_DIR)/lib/bdev/
CFLAGS += -I$(SPDK_ROOT_DIR)/module/bdev/raid/
CFLAGS += -I$(SPDK_ROOT_DIR)/module/bdev/raid/poweraid_raid_common/

COMMON_DIR := ../poweraid_raid_common

# 公共层（5f/6f 共用）
C_SRCS = $(COMMON_DIR)/poweraid_raid_common_sm_dispatch.c \
	 $(COMMON_DIR)/poweraid_raid_common_sm_raid.c \
	 $(COMMON_DIR)/poweraid_raid_common_sm_bdev.c \
	 $(COMMON_DIR)/poweraid_raid_common_sm_req.c \
	 $(COMMON_DIR)/poweraid_raid_common_sb.c \
	 $(COMMON_DIR)/poweraid_raid_common_ppl.c \
	 $(COMMON_DIR)/poweraid_raid_common_recovery.c \
	 $(COMMON_DIR)/poweraid_raid_common_rmw.c \
	 $(COMMON_DIR)/poweraid_raid_common_merge.c \
	 $(COMMON_DIR)/poweraid_raid_common_rebuild.c \
	 $(COMMON_DIR)/poweraid_raid_common_rpc.c \
	 $(COMMON_DIR)/poweraid_raid_common_gf8.c

# GF8 SIMD 路径: 按架构选择
ifeq ($(shell uname -m),x86_64)
C_SRCS += $(COMMON_DIR)/poweraid_raid_common_gf8_x86.c
else ifeq ($(shell uname -m),aarch64)
C_SRCS += $(COMMON_DIR)/poweraid_raid_common_gf8_neon.c
endif

# 6f 薄壳
C_SRCS += poweraid_raid6f.c
C_SRCS += poweraid_raid6f_calc.c
C_SRCS += poweraid_raid6f_recovery.c

LIBNAME = poweraid_raid6f

SPDK_MAP_FILE = $(SPDK_ROOT_DIR)/mk/spdk_blank.map

include $(SPDK_ROOT_DIR)/mk/spdk.lib.mk
