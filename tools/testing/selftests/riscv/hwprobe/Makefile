# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2021 ARM Limited
# Originally tools/testing/arm64/abi/Makefile

CFLAGS += -I$(top_srcdir)/tools/include
CFLAGS += -I$(top_srcdir)/arch/riscv/include

TEST_GEN_PROGS := hwprobe cbo rva23

include ../../lib.mk

$(OUTPUT)/hwprobe: hwprobe.c sys_hwprobe.S
	$(CC) -static -o$@ $(CFLAGS) $(LDFLAGS) $^

$(OUTPUT)/cbo: cbo.c sys_hwprobe.S
	$(CC) -static -o$@ $(CFLAGS) $(LDFLAGS) $^

$(OUTPUT)/rva23: rva23.c sys_hwprobe.S
	$(CC) -static -o$@ $(CFLAGS) $(LDFLAGS) $^
