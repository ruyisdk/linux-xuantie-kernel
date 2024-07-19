// SPDX-License-Identifier: GPL-2.0-only

#include <asm/cpufeature.h>
#include <asm/vendor_extensions.h>
#include <asm/vendor_extensions/andes.h>

#include <linux/types.h>
#include <linux/kernel.h>

/* All Andes vendor extensions supported in Linux */
const struct riscv_isa_ext_data riscv_isa_vendor_ext_andes[] = {
	{
		.name = "xandespmu",
		.property = "xandespmu",
		.id = RISCV_ISA_VENDOR_EXT_XANDESPMU
	},
};

struct riscv_isa_vendor_ext_data_list riscv_isa_vendor_ext_list_andes = {
	.ext_data_count = ARRAY_SIZE(riscv_isa_vendor_ext_andes),
	.ext_data = riscv_isa_vendor_ext_andes,
};
