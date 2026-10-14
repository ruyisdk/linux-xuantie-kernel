# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2023 VRULL

CFLAGS += -I$(top_srcdir)/tools/include
CFLAGS += -I$(top_srcdir)/arch/riscv/include

TEST_GEN_PROGS := xuantie_dtso

include ../../lib.mk

$(OUTPUT)/xuantie_dtso: xuantie_dtso.c ../hwprobe/sys_hwprobe.S
	$(CC) -static -o$@ $(CFLAGS) $(LDFLAGS) $^
