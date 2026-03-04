/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_RISCV_VENDOR_EXTENSIONS_THEAD_H
#define _ASM_RISCV_VENDOR_EXTENSIONS_THEAD_H

#include <asm/vendor_extensions.h>

#include <linux/types.h>

/*
 * Extension keys must be strictly less than RISCV_ISA_VENDOR_EXT_MAX.
 */
#define RISCV_ISA_VENDOR_EXT_XTHEADVECTOR		0
#define RISCV_ISA_VENDOR_EXT_XTHEADVDOT			1
#define RISCV_ISA_VENDOR_EXT_XTHEADVARITH		2
#define RISCV_ISA_VENDOR_EXT_XTHEADVCRYPTO		3
#define RISCV_ISA_VENDOR_EXT_XTHEADVCODER		4
#define RISCV_ISA_VENDOR_EXT_XTHEADVFOFP8MIN	5
#define RISCV_ISA_VENDOR_EXT_XTHEADVFOE8M0MIN	6
#define RISCV_ISA_VENDOR_EXT_XTHEADVFOFP4MIN	7
#define RISCV_ISA_VENDOR_EXT_XTHEADCBOP			8
#define RISCV_ISA_VENDOR_EXT_XTHEADAIOE			9
#define RISCV_ISA_VENDOR_EXT_XTHEADCRC			10

extern struct riscv_isa_vendor_ext_data_list riscv_isa_vendor_ext_list_thead;

#endif
