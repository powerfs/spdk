#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2026 poweraid.
#  All rights reserved.
#
#  poweraid_raid5f 增强 RAID5F 模块（独立子目录）

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../../../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

SO_VER := 8
SO_MINOR := 0

CFLAGS += -I$(SPDK_ROOT_DIR)/lib/bdev/
CFLAGS += -I$(SPDK_ROOT_DIR)/module/bdev/raid/

C_SRCS = poweraid_raid5f.c \
	 poweraid_raid5f_sm_dispatch.c \
	 poweraid_raid5f_sm_raid.c \
	 poweraid_raid5f_sm_bdev.c \
	 poweraid_raid5f_sm_req.c \
	 poweraid_raid5f_sb.c \
	 poweraid_raid5f_ppl.c \
	 poweraid_raid5f_recovery.c \
	 poweraid_raid5f_rmw.c \
	 poweraid_raid5f_merge.c \
	 poweraid_raid5f_rpc.c \
	 poweraid_raid5f_gf8.c

# GF8 SIMD 路径: 按架构选择（避免符号冲突）
ifeq ($(shell uname -m),x86_64)
C_SRCS += poweraid_raid5f_gf8_x86.c
else ifeq ($(shell uname -m),aarch64)
C_SRCS += poweraid_raid5f_gf8_neon.c
endif

LIBNAME = poweraid_raid5f

SPDK_MAP_FILE = $(SPDK_ROOT_DIR)/mk/spdk_blank.map

include $(SPDK_ROOT_DIR)/mk/spdk.lib.mk
