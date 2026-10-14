// SPDX-License-Identifier: GPL-2.0-only

#include <asm/vendor_extensions/thead.h>
#include <asm/vendor_extensions/thead_hwprobe.h>
#include <asm/vendor_extensions/vendor_hwprobe.h>

#include <linux/cpumask.h>
#include <linux/types.h>

#include <uapi/asm/hwprobe.h>
#include <uapi/asm/vendor/thead.h>

void hwprobe_isa_vendor_ext_thead_0(struct riscv_hwprobe *pair, const struct cpumask *cpus)
{
	VENDOR_EXTENSION_SUPPORTED(pair, cpus,
				   riscv_isa_vendor_ext_list_thead.per_hart_isa_bitmap, {
		VENDOR_EXT_KEY(XTHEADVECTOR);
		VENDOR_EXT_KEY(XUANTIE_SSDTSO);
		VENDOR_EXT_KEY(XUANTIE_VDOT);
		VENDOR_EXT_KEY(XUANTIE_VARITH);
		VENDOR_EXT_KEY(XUANTIE_VCRYPTO);
		VENDOR_EXT_KEY(XUANTIE_VCODER);
		VENDOR_EXT_KEY(XUANTIE_VFOFP8MIN);
		VENDOR_EXT_KEY(XUANTIE_VFOE8M0MIN);
		VENDOR_EXT_KEY(XUANTIE_VFOFP4MIN);
		VENDOR_EXT_KEY(XUANTIE_CBOP);
		VENDOR_EXT_KEY(XUANTIE_AIOE);
		VENDOR_EXT_KEY(XUANTIE_CRC);
	});
}
