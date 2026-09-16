# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2023 VRULL

CFLAGS += -I$(top_srcdir)/tools/include
CFLAGS += -I$(top_srcdir)/arch/riscv/include

TEST_GEN_PROGS := dtso

include ../../lib.mk

$(OUTPUT)/dtso: dtso.c ../hwprobe/sys_hwprobe.S
	$(CC) -static -o$@ $(CFLAGS) $(LDFLAGS) $^
