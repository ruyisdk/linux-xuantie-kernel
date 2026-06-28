/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Public registration API for the RISC-V Capacity and Bandwidth QoS
 * Register Interface (CBQRI) core. Discovery layers (device tree
 * platform drivers) call riscv_cbqri_register_cc_dt() to hand a capacity
 * controller descriptor to the core, which owns all subsequent state.
 */
#ifndef _LINUX_RISCV_CBQRI_H
#define _LINUX_RISCV_CBQRI_H

#include <linux/types.h>

struct cpumask;

enum cbqri_controller_type {
	CBQRI_CONTROLLER_TYPE_CAPACITY,
};

/**
 * struct cbqri_controller_info - registration descriptor
 * @addr:        MMIO base address of the controller's register interface
 * @size:        size of the MMIO region
 * @type:        controller type (capacity)
 * @rcid_count:  number of supported RCIDs
 * @cache_id:    cache id used as the resctrl domain id
 */
struct cbqri_controller_info {
	phys_addr_t			addr;
	phys_addr_t			size;
	enum cbqri_controller_type	type;
	u32				rcid_count;
	u32				cache_id;
};

#if IS_ENABLED(CONFIG_RISCV_CBQRI)
int riscv_cbqri_register_cc_dt(const struct cbqri_controller_info *info,
			       u32 cache_level, const struct cpumask *cpu_mask);
#else
static inline int
riscv_cbqri_register_cc_dt(const struct cbqri_controller_info *info,
			   u32 cache_level, const struct cpumask *cpu_mask)
{
	return -ENODEV;
}
#endif

#endif /* _LINUX_RISCV_CBQRI_H */
