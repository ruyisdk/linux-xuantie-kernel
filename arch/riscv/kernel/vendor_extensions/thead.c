// SPDX-License-Identifier: GPL-2.0-only

#include <asm/cpufeature.h>
#include <asm/vendor_extensions.h>
#include <asm/vendor_extensions/thead.h>

#include <linux/array_size.h>
#include <linux/cpumask.h>
#include <linux/types.h>

/* All T-Head vendor extensions supported in Linux */
static const struct riscv_isa_ext_data riscv_isa_vendor_ext_thead[] = {
	__RISCV_ISA_EXT_DATA(xtheadvector, RISCV_ISA_VENDOR_EXT_XTHEADVECTOR),
#ifdef CONFIG_RISCV_ISA_XUANTIE_SSDTSO
	__RISCV_ISA_EXT_DATA(xtheadssdtso, RISCV_ISA_VENDOR_EXT_XUANTIE_SSDTSO),
#endif
	__RISCV_ISA_EXT_DATA(xtheadvdot, RISCV_ISA_VENDOR_EXT_XUANTIE_VDOT),
	__RISCV_ISA_EXT_DATA(xtheadvarith, RISCV_ISA_VENDOR_EXT_XUANTIE_VARITH),
	__RISCV_ISA_EXT_DATA(xtheadvcrypto, RISCV_ISA_VENDOR_EXT_XUANTIE_VCRYPTO),
	__RISCV_ISA_EXT_DATA(xtheadvcoder, RISCV_ISA_VENDOR_EXT_XUANTIE_VCODER),
	__RISCV_ISA_EXT_DATA(xtheadvfofp8min, RISCV_ISA_VENDOR_EXT_XUANTIE_VFOFP8MIN),
	__RISCV_ISA_EXT_DATA(xtheadvfoe8m0min, RISCV_ISA_VENDOR_EXT_XUANTIE_VFOE8M0MIN),
	__RISCV_ISA_EXT_DATA(xtheadvfofp4min, RISCV_ISA_VENDOR_EXT_XUANTIE_VFOFP4MIN),
	__RISCV_ISA_EXT_DATA(xtheadcbop, RISCV_ISA_VENDOR_EXT_XUANTIE_CBOP),
	__RISCV_ISA_EXT_DATA(xtheadaioe, RISCV_ISA_VENDOR_EXT_XUANTIE_AIOE),
	__RISCV_ISA_EXT_DATA(xtheadcrc, RISCV_ISA_VENDOR_EXT_XUANTIE_CRC),
	__RISCV_ISA_EXT_DATA(xtheadmatrix, RISCV_ISA_VENDOR_EXT_XTHEADMATRIX),
	__RISCV_ISA_EXT_DATA(xtheadfxcr, RISCV_ISA_VENDOR_EXT_XTHEADFXCR),
	__RISCV_ISA_EXT_DATA(xtheadutnmode, RISCV_ISA_VENDOR_EXT_XTHEADUTNMODE),
};

struct riscv_isa_vendor_ext_data_list riscv_isa_vendor_ext_list_thead = {
	.ext_data_count = ARRAY_SIZE(riscv_isa_vendor_ext_thead),
	.ext_data = riscv_isa_vendor_ext_thead,
};
