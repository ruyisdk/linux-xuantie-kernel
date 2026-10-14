# SPDX-License-Identifier: GPL-2.0
# Originally tools/testing/arm64/Makefile

include ../lib.mk
# When ARCH not overridden for crosscompiling, lookup machine
ARCH ?= $(shell uname -m 2>/dev/null || echo not)

ifneq (,$(filter $(ARCH),riscv))
	ifeq ($(shell $(CC) -dM -E - < /dev/null 2>/dev/null | grep -c "__riscv_xlen 64"),1)
		RISCV_SUBTARGETS ?= xuantie_dtso hwprobe vector mm pm string_zbb cfi3
	else
		RISCV_SUBTARGETS ?= xuantie_dtso hwprobe vector mm pm string_zbb
	endif
else
	RISCV_SUBTARGETS :=
endif

CFLAGS := -Wall -O2 -g

# A proper top_srcdir is needed by KSFT(lib.mk)
top_srcdir = $(realpath ../../../../)

# Additional include paths needed by kselftest.h and local headers
CFLAGS += -I$(top_srcdir)/tools/testing/selftests/

CFLAGS += $(KHDR_INCLUDES)

export CFLAGS
export top_srcdir

all:
	@for DIR in $(RISCV_SUBTARGETS); do				\
		BUILD_TARGET=$(OUTPUT)/$$DIR;			\
		mkdir -p $$BUILD_TARGET;			\
		$(MAKE) OUTPUT=$$BUILD_TARGET -C $$DIR $@;		\
	done

install: all
	@for DIR in $(RISCV_SUBTARGETS); do				\
		BUILD_TARGET=$(OUTPUT)/$$DIR;			\
		$(MAKE) OUTPUT=$$BUILD_TARGET -C $$DIR $@;		\
	done

run_tests: all
	@for DIR in $(RISCV_SUBTARGETS); do				\
		BUILD_TARGET=$(OUTPUT)/$$DIR;			\
		$(MAKE) OUTPUT=$$BUILD_TARGET -C $$DIR $@;		\
	done

# Avoid any output on non riscv on emit_tests
emit_tests:
	@for DIR in $(RISCV_SUBTARGETS); do				\
		BUILD_TARGET=$(OUTPUT)/$$DIR;			\
		$(MAKE) OUTPUT=$$BUILD_TARGET -C $$DIR $@;		\
	done

clean:
	@for DIR in $(RISCV_SUBTARGETS); do				\
		BUILD_TARGET=$(OUTPUT)/$$DIR;			\
		$(MAKE) OUTPUT=$$BUILD_TARGET -C $$DIR $@;		\
	done

.PHONY: all clean install run_tests emit_tests
