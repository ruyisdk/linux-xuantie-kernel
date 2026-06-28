/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _DRIVERS_RESCTRL_CBQRI_INTERNAL_H
#define _DRIVERS_RESCTRL_CBQRI_INTERNAL_H

#include <linux/bitfield.h>
#include <linux/riscv_cbqri.h>
#include <linux/cpumask.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/types.h>

/*
 * FIELD_MODIFY() - modify a bitfield element (compat shim for kernels < 6.18).
 * Equivalent to: *_reg_p = (*_reg_p & ~_mask) | FIELD_PREP(_mask, _val)
 */
#ifndef FIELD_MODIFY
#define FIELD_MODIFY(_mask, _reg_p, _val)				\
	do {								\
		*(_reg_p) &= ~(_mask);					\
		*(_reg_p) |= FIELD_PREP(_mask, _val);			\
	} while (0)
#endif

/* Capacity Controller (CC) MMIO register offsets. */
#define CBQRI_CC_CAPABILITIES_OFF 0
#define CBQRI_CC_ALLOC_CTL_OFF   24
#define CBQRI_CC_BLOCK_MASK_OFF  32

/*
 * Per CBQRI 3.5 the block-mask width BMW is the smallest multiple of 64 bits
 * that holds NCBLKS, so cc_cunits sits at 32 + BMW/8 bytes. This constant is
 * valid only while NCBLKS <= 64. cbqri_probe_cc() rejects ncblks > 32 before
 * it reads cc_capabilities.CUNITS, forcing BMW to 64 bits, one 8-byte block
 * mask, and cc_cunits at 0x28. Raising that cap above 64 would require
 * computing 32 + roundup(ncblks, 64) / 8 instead of a constant.
 */
#define CBQRI_CC_CUNITS_OFF      40

/*
 * Highest base register offset (cc_block_mask at 0x20) plus its 8-byte width.
 * cbqri_probe_controller() rejects smaller mappings. cc_cunits at 0x28 is
 * optional and only required when cc_capabilities.CUNITS is set, which
 * cbqri_probe_cc() checks against the mapping size separately.
 */
#define CBQRI_CTRL_MIN_REG_SPAN  0x28u

#define CBQRI_CC_CAPABILITIES_VER_MINOR_MASK  GENMASK_ULL(3, 0)
#define CBQRI_CC_CAPABILITIES_VER_MAJOR_MASK  GENMASK_ULL(7, 4)
#define CBQRI_CC_CAPABILITIES_NCBLKS_MASK     GENMASK_ULL(23, 8)
#define CBQRI_CC_CAPABILITIES_CUNITS_MASK     BIT_ULL(25)

/*
 * CC control registers are 64-bit, but the CBQRI spec only guarantees
 * single-copy atomicity for naturally aligned 4-byte accesses. They are read
 * as two 32-bit halves (cbqri_readq) reconstructed into a u64 for field
 * extraction, and written via the low 32-bit half, so the driver does not
 * depend on native 64-bit MMIO. Keep every field mask GENMASK_ULL so
 * FIELD_MODIFY() or ~mask on the reconstructed u64 never zero-extends a
 * 32-bit mask and clobbers STATUS/BUSY/WPRI in bits 63:32.
 */
#define CBQRI_CONTROL_REGISTERS_OP_MASK      GENMASK_ULL(4, 0)
#define CBQRI_CONTROL_REGISTERS_AT_MASK      GENMASK_ULL(7, 5)
/* AT field values (CBQRI Table 1): data vs code half for CDP */
#define CBQRI_CONTROL_REGISTERS_AT_DATA      0
#define CBQRI_CONTROL_REGISTERS_AT_CODE      1
#define CBQRI_CONTROL_REGISTERS_RCID_MASK    GENMASK_ULL(19, 8)
#define CBQRI_CONTROL_REGISTERS_STATUS_MASK  GENMASK_ULL(38, 32)
#define CBQRI_CONTROL_REGISTERS_BUSY_MASK    GENMASK_ULL(39, 39)

#define CBQRI_CC_ALLOC_CTL_OP_CONFIG_LIMIT 1
#define CBQRI_CC_ALLOC_CTL_OP_READ_LIMIT   2
#define CBQRI_CC_ALLOC_CTL_STATUS_SUCCESS  1

/* Capacity Controller hardware capabilities */
struct riscv_cbqri_capacity_caps {
	u16 ncblks;
	bool supports_alloc_at_code;
	/* cc_capabilities.CUNITS: controller enforces a capacity-unit limit */
	bool cunits;
};

/**
 * struct cbqri_cc_config - desired capacity allocation state for one rcid
 * @cbm:         capacity block mask
 * @at:          AT half the @cbm applies to (CBQRI_CONTROL_REGISTERS_AT_DATA
 *               or CBQRI_CONTROL_REGISTERS_AT_CODE)
 * @cdp_enabled: when false and the controller supports AT, mirror @cbm
 *               into the other AT half so both stay in sync
 */
struct cbqri_cc_config {
	u64  cbm;
	u32  at;
	bool cdp_enabled;
};

struct cbqri_controller {
	void __iomem *base;
	/*
	 * Serializes the write-then-poll-busy MMIO sequences on this
	 * controller. Each CBQRI op may busy-wait up to 1 ms on slow
	 * firmware, so use a sleeping mutex to keep preemption enabled.
	 * All resctrl-arch entry points run in process context.
	 */
	struct mutex lock;

	struct riscv_cbqri_capacity_caps cc;

	bool alloc_capable;

	phys_addr_t addr;
	phys_addr_t size;
	enum cbqri_controller_type type;
	u32 rcid_count;

	struct list_head list;

	struct cache_controller {
		u32 cache_level;
		struct cpumask cpu_mask;
		/* Cache id used as the resctrl domain id */
		u32 cache_id;
	} cache;
};

extern struct list_head cbqri_controllers;
extern struct mutex cbqri_controllers_lock;

void cbqri_controller_destroy(struct cbqri_controller *ctrl);

int cbqri_apply_cache_config(struct cbqri_controller *ctrl, u32 closid,
			     const struct cbqri_cc_config *cfg);

int cbqri_read_cache_config(struct cbqri_controller *ctrl, u32 closid,
			    u32 at, u32 *cbm_out);

#endif /* _DRIVERS_RESCTRL_CBQRI_INTERNAL_H */
