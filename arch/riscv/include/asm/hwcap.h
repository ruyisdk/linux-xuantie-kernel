/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copied from arch/arm64/include/asm/hwcap.h
 *
 * Copyright (C) 2012 ARM Ltd.
 * Copyright (C) 2017 SiFive
 */
#ifndef _ASM_RISCV_HWCAP_H
#define _ASM_RISCV_HWCAP_H

#include <asm/alternative-macros.h>
#include <asm/errno.h>
#include <linux/bits.h>
#include <uapi/asm/hwcap.h>

#define RISCV_ISA_EXT_a		('a' - 'a')
#define RISCV_ISA_EXT_b		('b' - 'a')
#define RISCV_ISA_EXT_c		('c' - 'a')
#define RISCV_ISA_EXT_d		('d' - 'a')
#define RISCV_ISA_EXT_f		('f' - 'a')
#define RISCV_ISA_EXT_h		('h' - 'a')
#define RISCV_ISA_EXT_i		('i' - 'a')
#define RISCV_ISA_EXT_j		('j' - 'a')
#define RISCV_ISA_EXT_k		('k' - 'a')
#define RISCV_ISA_EXT_m		('m' - 'a')
#define RISCV_ISA_EXT_p		('p' - 'a')
#define RISCV_ISA_EXT_q		('q' - 'a')
#define RISCV_ISA_EXT_s		('s' - 'a')
#define RISCV_ISA_EXT_u		('u' - 'a')
#define RISCV_ISA_EXT_v		('v' - 'a')

/*
 * These macros represent the logical IDs of each multi-letter RISC-V ISA
 * extension and are used in the ISA bitmap. The logical IDs start from
 * RISCV_ISA_EXT_BASE, which allows the 0-25 range to be reserved for single
 * letter extensions. The maximum, RISCV_ISA_EXT_MAX, is defined in order
 * to allocate the bitmap and may be increased when necessary.
 *
 * New extensions should just be added to the bottom, rather than added
 * alphabetically, in order to avoid unnecessary shuffling.
 */
#define RISCV_ISA_EXT_BASE		26

#define RISCV_ISA_EXT_SSCOFPMF		26
#define RISCV_ISA_EXT_SSTC		27
#define RISCV_ISA_EXT_SVINVAL		28
#define RISCV_ISA_EXT_SVPBMT		29
#define RISCV_ISA_EXT_ZBB		30
#define RISCV_ISA_EXT_ZICBOM		31
#define RISCV_ISA_EXT_ZIHINTPAUSE	32
#define RISCV_ISA_EXT_SVNAPOT		33
#define RISCV_ISA_EXT_ZICBOZ		34
#define RISCV_ISA_EXT_SMAIA		35
#define RISCV_ISA_EXT_SSAIA		36
#define RISCV_ISA_EXT_ZBA		37
#define RISCV_ISA_EXT_ZBS		38
#define RISCV_ISA_EXT_ZICNTR		39
#define RISCV_ISA_EXT_ZICSR		40
#define RISCV_ISA_EXT_ZIFENCEI		41
#define RISCV_ISA_EXT_ZIHPM		42
#define RISCV_ISA_EXT_SMSTATEEN		43
#define RISCV_ISA_EXT_ZICOND		44
#define RISCV_ISA_EXT_ZBC		45
#define RISCV_ISA_EXT_ZBKB		46
#define RISCV_ISA_EXT_ZBKC		47
#define RISCV_ISA_EXT_ZBKX		48
#define RISCV_ISA_EXT_ZKND		49
#define RISCV_ISA_EXT_ZKNE		50
#define RISCV_ISA_EXT_ZKNH		51
#define RISCV_ISA_EXT_ZKR		52
#define RISCV_ISA_EXT_ZKSED		53
#define RISCV_ISA_EXT_ZKSH		54
#define RISCV_ISA_EXT_ZKT		55
#define RISCV_ISA_EXT_ZVBB		56
#define RISCV_ISA_EXT_ZVBC		57
#define RISCV_ISA_EXT_ZVKB		58
#define RISCV_ISA_EXT_ZVKG		59
#define RISCV_ISA_EXT_ZVKNED		60
#define RISCV_ISA_EXT_ZVKNHA		61
#define RISCV_ISA_EXT_ZVKNHB		62
#define RISCV_ISA_EXT_ZVKSED		63
#define RISCV_ISA_EXT_ZVKSH		64
#define RISCV_ISA_EXT_ZVKT		65
#define RISCV_ISA_EXT_ZFH		66
#define RISCV_ISA_EXT_ZFHMIN		67
#define RISCV_ISA_EXT_ZIHINTNTL		68
#define RISCV_ISA_EXT_ZVFH		69
#define RISCV_ISA_EXT_ZVFHMIN		70
#define RISCV_ISA_EXT_ZFA		71
#define RISCV_ISA_EXT_ZTSO		72
#define RISCV_ISA_EXT_ZACAS		73
#define RISCV_ISA_EXT_ZVE32X		74
#define RISCV_ISA_EXT_ZVE32F		75
#define RISCV_ISA_EXT_ZVE64X		76
#define RISCV_ISA_EXT_ZVE64F		77
#define RISCV_ISA_EXT_ZVE64D		78
#define RISCV_ISA_EXT_ZIMOP		79
#define RISCV_ISA_EXT_ZCA		80
#define RISCV_ISA_EXT_ZCB		81
#define RISCV_ISA_EXT_ZCD		82
#define RISCV_ISA_EXT_ZCF		83
#define RISCV_ISA_EXT_ZCMOP		84
#define RISCV_ISA_EXT_ZAWRS		85
#define RISCV_ISA_EXT_ZICCRSE		88

#define RISCV_ISA_EXT_MAX		128

#ifdef CONFIG_RISCV_M_MODE
#define RISCV_ISA_EXT_SxAIA		RISCV_ISA_EXT_SMAIA
#else
#define RISCV_ISA_EXT_SxAIA		RISCV_ISA_EXT_SSAIA
#endif

#ifndef __ASSEMBLY__

#include <linux/jump_label.h>
#include <asm/cpufeature.h>

unsigned long riscv_get_elf_hwcap(void);

#if 0

struct riscv_isa_ext_data {
	const unsigned int id;
	const char *name;
	const char *property;
};

extern const struct riscv_isa_ext_data riscv_isa_ext[];
extern const size_t riscv_isa_ext_count;
extern bool riscv_isa_fallback;

unsigned long riscv_isa_extension_base(const unsigned long *isa_bitmap);

#define riscv_isa_extension_mask(ext) BIT_MASK(RISCV_ISA_EXT_##ext)

bool __riscv_isa_extension_available(const unsigned long *isa_bitmap, int bit);
#define riscv_isa_extension_available(isa_bitmap, ext)	\
	__riscv_isa_extension_available(isa_bitmap, RISCV_ISA_EXT_##ext)

static __always_inline bool
riscv_has_extension_likely(const unsigned long ext)
{
	compiletime_assert(ext < RISCV_ISA_EXT_MAX,
			   "ext must be < RISCV_ISA_EXT_MAX");

	if (IS_ENABLED(CONFIG_RISCV_ALTERNATIVE)) {
		asm goto(
		ALTERNATIVE("j	%l[l_no]", "nop", 0, %[ext], 1)
		:
		: [ext] "i" (ext)
		:
		: l_no);
	} else {
		if (!__riscv_isa_extension_available(NULL, ext))
			goto l_no;
	}

	return true;
l_no:
	return false;
}

static __always_inline bool
riscv_has_extension_unlikely(const unsigned long ext)
{
	compiletime_assert(ext < RISCV_ISA_EXT_MAX,
			   "ext must be < RISCV_ISA_EXT_MAX");

	if (IS_ENABLED(CONFIG_RISCV_ALTERNATIVE)) {
		asm goto(
		ALTERNATIVE("nop", "j	%l[l_yes]", 0, %[ext], 1)
		:
		: [ext] "i" (ext)
		:
		: l_yes);
	} else {
		if (__riscv_isa_extension_available(NULL, ext))
			goto l_yes;
	}

	return false;
l_yes:
	return true;
}


static __always_inline bool riscv_cpu_has_extension_likely(int cpu, const unsigned long ext)
{
	if (IS_ENABLED(CONFIG_RISCV_ALTERNATIVE) && riscv_has_extension_likely(ext))
		return true;

	return __riscv_isa_extension_available(hart_isa[cpu].isa, ext);
}

static __always_inline bool riscv_cpu_has_extension_unlikely(int cpu, const unsigned long ext)
{
	if (IS_ENABLED(CONFIG_RISCV_ALTERNATIVE) && riscv_has_extension_unlikely(ext))
		return true;

	return __riscv_isa_extension_available(hart_isa[cpu].isa, ext);
}
#endif
#endif

#endif /* _ASM_RISCV_HWCAP_H */
