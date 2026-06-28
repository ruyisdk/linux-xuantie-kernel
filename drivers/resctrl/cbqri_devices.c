// SPDX-License-Identifier: GPL-2.0-only

#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

#include <linux/bitfield.h>
#include <linux/riscv_cbqri.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <asm/csr.h>

#include "cbqri_internal.h"

LIST_HEAD(cbqri_controllers);

/*
 * Serializes cbqri_controllers mutations against a concurrent insert under
 * asynchronous driver probing, and against the boot-time walk in the resctrl
 * glue. Runtime cpuhp walks happen after registration has settled.
 */
DEFINE_MUTEX(cbqri_controllers_lock);

/*
 * CBQRI registers are 64-bit, but the spec only guarantees single-copy
 * atomicity for naturally aligned 4-byte accesses. Read the two halves and
 * reconstruct, so the driver does not rely on native 64-bit MMIO.
 */
static u64 cbqri_readq(void __iomem *addr)
{
	return (u64)readl(addr) | ((u64)readl(addr + 4) << 32);
}

/* Set capacity block mask (cc_block_mask) */
static void cbqri_set_cbm(struct cbqri_controller *ctrl, u64 cbm)
{
	/*
	 * cbqri_probe_cc() rejects ncblks > 32, so the mask fits the low word.
	 * Per CBQRI 3.5 the cc_block_mask bits above NCBLKS are read-only zero,
	 * so the upper word needs no write.
	 */
	writel(lower_32_bits(cbm), ctrl->base + CBQRI_CC_BLOCK_MASK_OFF);
}

/*
 * Clear cc_cunits so a CONFIG_LIMIT on a CUNITS-capable controller imposes no
 * capacity-unit limit. resctrl models only the block mask. On controllers
 * with cc_capabilities.CUNITS == 0 the register is read-only zero, so there is
 * nothing to do.
 */
static void cbqri_clear_cunits(struct cbqri_controller *ctrl)
{
	if (ctrl->cc.cunits) {
		writel(0, ctrl->base + CBQRI_CC_CUNITS_OFF);
		writel(0, ctrl->base + CBQRI_CC_CUNITS_OFF + 4);
	}
}

static int cbqri_wait_busy_flag(struct cbqri_controller *ctrl, int reg_offset,
				u64 *regp)
{
	u64 reg;
	int ret;

	/*
	 * Sleeping poll: caller holds ctrl->lock as a sleeping mutex, so
	 * 10us/1ms is safe under PREEMPT_RT.
	 */
	ret = read_poll_timeout(cbqri_readq, reg,
				!FIELD_GET(CBQRI_CONTROL_REGISTERS_BUSY_MASK, reg),
				10, 1000, false, ctrl->base + reg_offset);
	if (ret)
		return ret;
	if (regp)
		*regp = reg;
	return 0;
}

/*
 * Perform capacity allocation control operation on capacity controller.
 * Caller must hold ctrl->lock.
 */
static int cbqri_cc_alloc_op(struct cbqri_controller *ctrl, int operation,
			     int rcid, u32 at)
{
	int reg_offset = CBQRI_CC_ALLOC_CTL_OFF;
	int status;
	u64 reg;

	lockdep_assert_held(&ctrl->lock);

	if (cbqri_wait_busy_flag(ctrl, reg_offset, &reg) < 0) {
		pr_err_ratelimited("BUSY timeout before starting operation\n");
		return -EIO;
	}
	FIELD_MODIFY(CBQRI_CONTROL_REGISTERS_OP_MASK, &reg, operation);
	FIELD_MODIFY(CBQRI_CONTROL_REGISTERS_RCID_MASK, &reg, rcid);

	/*
	 * CBQRI Table 1: AT 0=Data, 1=Code. Program AT on controllers
	 * that report supports_alloc_at_code. On controllers that don't,
	 * AT is reserved-zero and the op acts on both halves.
	 */
	reg &= ~CBQRI_CONTROL_REGISTERS_AT_MASK;
	if (ctrl->cc.supports_alloc_at_code)
		reg |= FIELD_PREP(CBQRI_CONTROL_REGISTERS_AT_MASK, at);

	writel(lower_32_bits(reg), ctrl->base + reg_offset);

	if (cbqri_wait_busy_flag(ctrl, reg_offset, &reg) < 0) {
		pr_err_ratelimited("BUSY timeout during operation\n");
		return -EIO;
	}

	status = FIELD_GET(CBQRI_CONTROL_REGISTERS_STATUS_MASK, reg);
	if (status != CBQRI_CC_ALLOC_CTL_STATUS_SUCCESS) {
		pr_err_ratelimited("operation %d failed: status=%d\n", operation, status);
		return -EIO;
	}

	return 0;
}

/*
 * Apply a capacity block mask and verify via CONFIG_LIMIT + READ_LIMIT.
 *
 * AT-capable controllers with CDP off need a second CONFIG_LIMIT on the
 * other AT half (the spec encodes AT only as 0=Data / 1=Code, there is
 * no "both halves" value). CDP-on issues separate per-type writes from
 * resctrl, so a single CONFIG_LIMIT per call is correct.
 */
int cbqri_apply_cache_config(struct cbqri_controller *ctrl, u32 closid,
			     const struct cbqri_cc_config *cfg)
{
	bool need_at_mirror;
	u64 saved_cbm = 0;
	int err = 0;
	u64 reg;

	mutex_lock(&ctrl->lock);

	need_at_mirror = ctrl->cc.supports_alloc_at_code && !cfg->cdp_enabled;

	/*
	 * Capture the cfg->at half CBM before any write so a partial
	 * AT-mirror failure can revert and keep the two halves consistent.
	 * Pre-clear cc_block_mask so a silent firmware no-op (status
	 * SUCCESS but staging not updated) shows as a zero readback
	 * rather than carrying stale data from a prior op.
	 */
	if (need_at_mirror) {
		cbqri_set_cbm(ctrl, 0);
		err = cbqri_cc_alloc_op(ctrl, CBQRI_CC_ALLOC_CTL_OP_READ_LIMIT,
					closid, cfg->at);
		if (err < 0)
			goto out;
		saved_cbm = cbqri_readq(ctrl->base + CBQRI_CC_BLOCK_MASK_OFF);
	}

	/* Set capacity block mask (cc_block_mask) */
	cbqri_set_cbm(ctrl, cfg->cbm);
	cbqri_clear_cunits(ctrl);

	/* Capacity config limit operation for the AT half implied by cfg->at */
	err = cbqri_cc_alloc_op(ctrl, CBQRI_CC_ALLOC_CTL_OP_CONFIG_LIMIT,
				closid, cfg->at);
	if (err < 0)
		goto out;

	/*
	 * CDP-off mirror: on AT-capable controllers, also program the
	 * other AT half with the same mask so the two halves stay in sync.
	 */
	if (need_at_mirror) {
		u32 other = (cfg->at == CBQRI_CONTROL_REGISTERS_AT_CODE) ?
			    CBQRI_CONTROL_REGISTERS_AT_DATA :
			    CBQRI_CONTROL_REGISTERS_AT_CODE;

		cbqri_set_cbm(ctrl, cfg->cbm);
		cbqri_clear_cunits(ctrl);
		err = cbqri_cc_alloc_op(ctrl,
					CBQRI_CC_ALLOC_CTL_OP_CONFIG_LIMIT,
					closid, other);
		if (err < 0) {
			int rerr;

			/*
			 * Best-effort revert of the cfg->at half so the two
			 * halves stay in sync. A schemata read sees only one
			 * half, so silent divergence would otherwise report
			 * the new value as if the write had succeeded.
			 */
			cbqri_set_cbm(ctrl, saved_cbm);
			cbqri_clear_cunits(ctrl);
			rerr = cbqri_cc_alloc_op(ctrl,
						 CBQRI_CC_ALLOC_CTL_OP_CONFIG_LIMIT,
						 closid, cfg->at);
			if (rerr < 0)
				pr_err_ratelimited("AT-mirror revert failed (err=%d), AT halves diverged\n",
						   rerr);
			goto out;
		}
	}

	/* Clear cc_block_mask before read limit to verify op works */
	cbqri_set_cbm(ctrl, 0);

	/* Perform a capacity read limit operation to verify blockmask */
	err = cbqri_cc_alloc_op(ctrl, CBQRI_CC_ALLOC_CTL_OP_READ_LIMIT,
				closid, cfg->at);
	if (err < 0)
		goto out;

	/*
	 * Read capacity blockmask and narrow to u32 to match resctrl's CBM
	 * width. cbqri_probe_cc() rejects ncblks > 32 so the upper bits are
	 * reserved zero.
	 */
	reg = cbqri_readq(ctrl->base + CBQRI_CC_BLOCK_MASK_OFF);
	if (lower_32_bits(reg) != cfg->cbm) {
		pr_err_ratelimited("CBM verify mismatch (reg=%llx != cbm=%llx)\n",
				   reg, cfg->cbm);
		err = -EIO;
	}

out:
	mutex_unlock(&ctrl->lock);
	return err;
}

/*
 * Read the configured CBM for closid on the at half via READ_LIMIT.
 * Pre-clears cc_block_mask before the op so a silent firmware no-op
 * (status SUCCESS but staging not updated) is detectable in cbm_out.
 */
int cbqri_read_cache_config(struct cbqri_controller *ctrl, u32 closid,
			    u32 at, u32 *cbm_out)
{
	int err;

	mutex_lock(&ctrl->lock);
	cbqri_set_cbm(ctrl, 0);
	err = cbqri_cc_alloc_op(ctrl, CBQRI_CC_ALLOC_CTL_OP_READ_LIMIT, closid, at);
	if (err == 0) {
		/*
		 * resctrl exposes the CBM as a u32 and cbqri_probe_cc() rejects
		 * ncblks > 32, so the mask fits the low 32-bit word of the
		 * cc_block_mask register.
		 */
		*cbm_out = readl(ctrl->base + CBQRI_CC_BLOCK_MASK_OFF);
	}
	mutex_unlock(&ctrl->lock);
	return err;
}

static int cbqri_probe_feature(struct cbqri_controller *ctrl, int reg_offset,
			       int operation, int *status, bool *access_type_supported)
{
	const u64 active_mask = CBQRI_CONTROL_REGISTERS_OP_MASK |
				CBQRI_CONTROL_REGISTERS_AT_MASK |
				CBQRI_CONTROL_REGISTERS_RCID_MASK;
	u64 reg, saved_reg;
	int at;

	/*
	 * Default the output to false so the status==0 (feature not
	 * implemented) path returns a deterministic value to the caller
	 * rather than leaving an uninitialized bool.
	 */
	*access_type_supported = false;

	/* Keep the initial register value to preserve the WPRI fields */
	reg = cbqri_readq(ctrl->base + reg_offset);
	saved_reg = reg;

	/* Drain any in-flight firmware op before issuing our own write. */
	if (cbqri_wait_busy_flag(ctrl, reg_offset, &saved_reg) < 0) {
		pr_err("BUSY timeout before probe operation\n");
		return -EIO;
	}

	/*
	 * Execute the requested operation with all active fields
	 * (OP/AT/RCID) zeroed except OP itself. Every bit not in
	 * active_mask is WPRI and gets carried over from saved_reg.
	 */
	reg = (saved_reg & ~active_mask) |
	      FIELD_PREP(CBQRI_CONTROL_REGISTERS_OP_MASK, operation);
	writel(lower_32_bits(reg), ctrl->base + reg_offset);
	if (cbqri_wait_busy_flag(ctrl, reg_offset, &reg) < 0) {
		pr_err_ratelimited("BUSY timeout during operation\n");
		return -EIO;
	}

	/* Get the operation status */
	*status = FIELD_GET(CBQRI_CONTROL_REGISTERS_STATUS_MASK, reg);

	/*
	 * Check for the AT support if the register is implemented
	 * (if not, the status value will remain 0)
	 */
	if (*status != 0) {
		/*
		 * Re-issue operation with AT=CODE so the controller
		 * latches AT=CODE on supported hardware (or resets it to 0
		 * on hardware that doesn't). OP must be a defined CBQRI op
		 * here. OP=0 is a no-op and would silently disable CDP.
		 */
		reg = (saved_reg & ~active_mask) |
		      FIELD_PREP(CBQRI_CONTROL_REGISTERS_OP_MASK, operation) |
		      FIELD_PREP(CBQRI_CONTROL_REGISTERS_AT_MASK,
				 CBQRI_CONTROL_REGISTERS_AT_CODE);
		writel(lower_32_bits(reg), ctrl->base + reg_offset);
		if (cbqri_wait_busy_flag(ctrl, reg_offset, &reg) < 0) {
			pr_err("BUSY timeout setting AT field\n");
			return -EIO;
		}

		/*
		 * If the AT field value has been reset to zero,
		 * then the AT support is not present
		 */
		at = FIELD_GET(CBQRI_CONTROL_REGISTERS_AT_MASK, reg);
		if (at == CBQRI_CONTROL_REGISTERS_AT_CODE)
			*access_type_supported = true;
	}

	/*
	 * Restore the original register value.
	 * Clear OP to avoid re-triggering the probe op.
	 */
	saved_reg &= ~CBQRI_CONTROL_REGISTERS_OP_MASK;
	writel(lower_32_bits(saved_reg), ctrl->base + reg_offset);
	if (cbqri_wait_busy_flag(ctrl, reg_offset, NULL) < 0) {
		pr_err("BUSY timeout restoring register value\n");
		return -EIO;
	}

	return 0;
}

static int cbqri_probe_cc(struct cbqri_controller *ctrl)
{
	int err, status;
	int ver_major, ver_minor;
	u64 reg;

	reg = cbqri_readq(ctrl->base + CBQRI_CC_CAPABILITIES_OFF);
	if (reg == 0)
		return -ENODEV;

	ver_minor = FIELD_GET(CBQRI_CC_CAPABILITIES_VER_MINOR_MASK, reg);
	ver_major = FIELD_GET(CBQRI_CC_CAPABILITIES_VER_MAJOR_MASK, reg);
	ctrl->cc.ncblks = FIELD_GET(CBQRI_CC_CAPABILITIES_NCBLKS_MASK, reg);

	pr_info("version=%d.%d ncblks=%d cache_level=%d\n",
		 ver_major, ver_minor,
		 ctrl->cc.ncblks, ctrl->cache.cache_level);

	/*
	 * NCBLKS == 0 would divide-by-zero in the schemata math while
	 * ctrl->lock is held.
	 */
	if (!ctrl->cc.ncblks) {
		pr_warn("CC at %pa has 0 capacity blocks, skipping\n",
			&ctrl->addr);
		return -ENODEV;
	}

	if (ctrl->cc.ncblks > 32) {
		pr_warn("CC at %pa has ncblks=%u > 32 (resctrl CBM is u32), skipping\n",
			&ctrl->addr, ctrl->cc.ncblks);
		return -ENODEV;
	}

	/*
	 * On a CUNITS-capable controller, CONFIG_LIMIT also consumes cc_cunits,
	 * whose reset value is unspecified. The driver clears it to 0 (no limit)
	 * on every CONFIG_LIMIT, so cc_cunits at 0x28 must be within the mapping.
	 */
	ctrl->cc.cunits = FIELD_GET(CBQRI_CC_CAPABILITIES_CUNITS_MASK, reg);
	if (ctrl->cc.cunits && ctrl->size < CBQRI_CC_CUNITS_OFF + 8) {
		pr_warn("CC at %pa supports CUNITS but maps only %pa, skipping\n",
			&ctrl->addr, &ctrl->size);
		return -ENODEV;
	}

	/* Probe allocation features */
	err = cbqri_probe_feature(ctrl, CBQRI_CC_ALLOC_CTL_OFF,
				  CBQRI_CC_ALLOC_CTL_OP_READ_LIMIT,
				  &status, &ctrl->cc.supports_alloc_at_code);
	if (err)
		return err;

	if (status == CBQRI_CC_ALLOC_CTL_STATUS_SUCCESS)
		ctrl->alloc_capable = true;

	return 0;
}

static int cbqri_probe_controller(struct cbqri_controller *ctrl)
{
	int err;

	pr_info("controller info: type=%d addr=%pa size=%pa max-rcid=%u\n",
		 ctrl->type, &ctrl->addr, &ctrl->size, ctrl->rcid_count);

	if (!ctrl->addr) {
		pr_warn("controller has invalid addr=0x0, skipping\n");
		return -EINVAL;
	}

	if (ctrl->size < CBQRI_CTRL_MIN_REG_SPAN) {
		pr_warn("controller at %pa: size %pa < minimum 0x%x, skipping\n",
			&ctrl->addr, &ctrl->size, CBQRI_CTRL_MIN_REG_SPAN);
		return -EINVAL;
	}

	if (!request_mem_region(ctrl->addr, ctrl->size, "cbqri_controller")) {
		pr_err("request_mem_region failed for %pa\n", &ctrl->addr);
		return -EBUSY;
	}

	ctrl->base = ioremap(ctrl->addr, ctrl->size);
	if (!ctrl->base) {
		pr_err("ioremap failed for %pa\n", &ctrl->addr);
		err = -ENOMEM;
		goto err_release;
	}

	switch (ctrl->type) {
	case CBQRI_CONTROLLER_TYPE_CAPACITY:
		err = cbqri_probe_cc(ctrl);
		break;
	default:
		pr_err("unknown controller type %d\n", ctrl->type);
		err = -ENODEV;
		break;
	}

	if (err)
		goto err_iounmap;

	return 0;

err_iounmap:
	iounmap(ctrl->base);
	ctrl->base = NULL;
err_release:
	release_mem_region(ctrl->addr, ctrl->size);
	return err;
}

void cbqri_controller_destroy(struct cbqri_controller *ctrl)
{
	/*
	 * cbqri_probe_controller() clears ctrl->base on its error paths and
	 * releases the mem region itself, so reach into both only when
	 * destroy is rolling back a successful probe.
	 */
	if (ctrl->base) {
		iounmap(ctrl->base);
		release_mem_region(ctrl->addr, ctrl->size);
	}
	kfree(ctrl);
}

/**
 * riscv_cbqri_register_cc_dt() - register a DT-described capacity controller
 * @info:        registration descriptor. info->cache_id is used as the
 *               resctrl domain id. info->type must be CAPACITY.
 * @cache_level: cache level (2 or 3) the controller backs, mapped to the
 *               resctrl L2/L3 resource by the resctrl glue.
 * @cpu_mask:    CPUs that share this cache.
 *
 * The cache topology is supplied directly by the caller. A device-tree
 * platform driver that already knows which CPUs share the cache and at what
 * level passes that in. There is no firmware table to resolve it from.
 *
 * Return: 0 on success, or a negative errno on failure.
 */
int riscv_cbqri_register_cc_dt(const struct cbqri_controller_info *info,
			       u32 cache_level, const struct cpumask *cpu_mask)
{
	struct cbqri_controller *ctrl;
	int err;

	if (!info->addr) {
		pr_warn("skipping controller with invalid addr=0x0\n");
		return -EINVAL;
	}

	if (info->type != CBQRI_CONTROLLER_TYPE_CAPACITY) {
		pr_warn("register_cc_dt called with non-capacity type %u\n",
			info->type);
		return -EINVAL;
	}

	if (!cpu_mask || cpumask_empty(cpu_mask)) {
		pr_warn("register_cc_dt called with empty cpu_mask\n");
		return -EINVAL;
	}

	ctrl = kzalloc(sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	mutex_init(&ctrl->lock);

	ctrl->addr = info->addr;
	ctrl->size = info->size;
	ctrl->type = info->type;
	ctrl->rcid_count = info->rcid_count;

	/*
	 * SRMCFG encodes RCID in 12 bits. Reject an out-of-range count rather
	 * than silently truncating in every FIELD_PREP(SRMCFG_RCID_MASK, closid)
	 * on the schedule-in fast path.
	 */
	if (ctrl->rcid_count > FIELD_MAX(SRMCFG_RCID_MASK) + 1) {
		pr_warn("CC at %pa has RCID count %u beyond the 12-bit SRMCFG field, skipping\n",
			&ctrl->addr, ctrl->rcid_count);
		cbqri_controller_destroy(ctrl);
		return -EINVAL;
	}

	ctrl->cache.cache_id = info->cache_id;
	ctrl->cache.cache_level = cache_level;
	cpumask_copy(&ctrl->cache.cpu_mask, cpu_mask);

	err = cbqri_probe_controller(ctrl);
	if (err) {
		cbqri_controller_destroy(ctrl);
		return err;
	}

	/*
	 * Allocation capability comes from the capabilities register probed
	 * above, not from device tree. rcid_count only bounds the RCID range,
	 * so a controller the hardware reports as alloc-capable but described
	 * with no RCID count cannot be driven. Reject that inconsistency. A
	 * monitoring-only controller (not alloc_capable) needs no RCID count.
	 */
	if (ctrl->alloc_capable && !ctrl->rcid_count) {
		pr_warn("CC at %pa is alloc-capable but has no RCID count, skipping\n",
			&ctrl->addr);
		cbqri_controller_destroy(ctrl);
		return -EINVAL;
	}

	mutex_lock(&cbqri_controllers_lock);
	list_add_tail(&ctrl->list, &cbqri_controllers);
	mutex_unlock(&cbqri_controllers_lock);
	return 0;
}
