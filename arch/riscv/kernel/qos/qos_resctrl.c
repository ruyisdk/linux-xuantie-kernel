// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2022 Rivos Inc.

#define pr_fmt(fmt) "qos: resctrl: " fmt

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/riscv_qos.h>
#include <linux/resctrl.h>
#include <linux/types.h>
#include <asm/csr.h>
#include <asm/qos.h>
#include "internal.h"

#define MAX_CONTROLLERS 6
static struct cbqri_resctrl_res cbqri_resctrl_resources[RDT_NUM_RESOURCES];
/*
 * The size of the controllers array reflects that number of CBQRI controllers
 * (3x capacity, 3x bandwidth) in the hypothetical SoC that Qemu implements for
 * the CBQRI proof-of-concept.
 *
 * The proper solution will involve dynamically allocating cbqri_controller
 * structures and adding them to a list of controllers. This avoids having to
 * make assumptions about the system.
 */
static struct cbqri_controller controllers[MAX_CONTROLLERS];
static int found_controllers;

static bool exposed_alloc_capable;
static bool exposed_mon_capable;
/* CDP (code data prioritization) on x86 is AT (access type) on RISC-V */
static bool exposed_cdp_l2_capable;
static bool exposed_cdp_l3_capable;
static bool is_cdp_l2_enabled;
static bool is_cdp_l3_enabled;

/* used by resctrl_arch_system_num_rmid_idx() */
static u32 max_rmid;

LIST_HEAD(cbqri_controllers);

static int qos_wait_busy_flag(struct cbqri_controller *ctrl, int reg_offset);

bool resctrl_arch_alloc_capable(void)
{
	return exposed_alloc_capable;
}

bool resctrl_arch_mon_capable(void)
{
	return exposed_mon_capable;
}

bool resctrl_arch_is_mbm_local_enabled(void)
{
	/*
	 * Seems to be the bandwidth between a L2 cache and L3 cache
	 * according to Intel resctrl implementation. There is no meaning
	 * in supporting such stat when we use resctrl MON_DATA feature
	 * to support CBQRI bandwidth monitoring.
	 */
	return false;
}

bool resctrl_arch_is_mbm_total_enabled(void)
{
	return true;
}

bool resctrl_arch_get_cdp_enabled(enum resctrl_res_level rid)
{
	switch (rid) {
	case RDT_RESOURCE_L2:
		return is_cdp_l2_enabled;

	case RDT_RESOURCE_L3:
		return is_cdp_l3_enabled;

	default:
		return false;
	}
}

int resctrl_arch_set_cdp_enabled(enum resctrl_res_level rid, bool enable)
{
	switch (rid) {
	case RDT_RESOURCE_L2:
		if (!exposed_cdp_l2_capable)
			return -ENODEV;
		is_cdp_l2_enabled = enable;
		break;

	case RDT_RESOURCE_L3:
		if (!exposed_cdp_l3_capable)
			return -ENODEV;
		is_cdp_l3_enabled = enable;
		break;

	default:
		return -ENODEV;
	}

	return 0;
}

bool resctrl_arch_is_llc_occupancy_enabled(void)
{
	/*
	 * There is no occupancy in CBQRI bandwidth controller (resctrl MON_DATA
	 * was the only available way to implement RISC-V memory monitoring).
	 */
	return false;
}

/*
 * Note about terminology between x86 (Intel RDT/AMD QoS) and RISC-V:
 *   CLOSID on x86 is RCID on RISC-V
 *     RMID on x86 is MCID on RISC-V
 */
u32 resctrl_arch_get_num_closid(struct rdt_resource *res)
{
	struct cbqri_resctrl_res *hw_res;

	hw_res = container_of(res, struct cbqri_resctrl_res, resctrl_res);

	return hw_res->max_rcid;
}

u32 resctrl_arch_system_num_rmid_idx(void)
{
	return max_rmid;
}

u32 resctrl_arch_rmid_idx_encode(u32 closid, u32 rmid)
{
	return rmid;
}

void resctrl_arch_rmid_idx_decode(u32 idx, u32 *closid, u32 *rmid)
{
	*closid = ((u32)~0); /* refer to X86_RESCTRL_BAD_CLOSID */
	*rmid = idx;
}

/* RISC-V resctrl interface does not maintain a default srmcfg value for a given CPU */
void resctrl_arch_set_cpu_default_closid_rmid(int cpu, u32 closid, u32 rmid) { }

void resctrl_sched_in(struct task_struct *tsk)
{
	qos_sched_in(tsk);
}

void resctrl_arch_sync_cpu_defaults(void *info)
{
	resctrl_sched_in(current);
}

void resctrl_arch_set_closid_rmid(struct task_struct *tsk, u32 closid, u32 rmid)
{
	u32 srmcfg;

	WARN_ON_ONCE((closid & SRMCFG_RCID_MASK) != closid);
	WARN_ON_ONCE((rmid & SRMCFG_MCID_MASK) != rmid);

	srmcfg = rmid << SRMCFG_MCID_SHIFT;
	srmcfg |= closid;
	WRITE_ONCE(tsk->thread.srmcfg, srmcfg);
}

bool resctrl_arch_match_closid(struct task_struct *tsk, u32 closid)
{
	u32 srmcfg;
	bool match;

	srmcfg = READ_ONCE(tsk->thread.srmcfg);
	match = (srmcfg & SRMCFG_RCID_MASK) == closid;
	return match;
}

bool resctrl_arch_match_rmid(struct task_struct *tsk, u32 closid, u32 rmid)
{
	u32 tsk_rmid;

	tsk_rmid = READ_ONCE(tsk->thread.srmcfg);
	tsk_rmid >>= SRMCFG_MCID_SHIFT;
	tsk_rmid &= SRMCFG_MCID_MASK;

	return tsk_rmid == rmid;
}

struct rdt_resource *resctrl_arch_get_resource(enum resctrl_res_level l)
{
	if (l >= RDT_NUM_RESOURCES)
		return NULL;

	return &cbqri_resctrl_resources[l].resctrl_res;
}

struct rdt_domain *resctrl_arch_find_domain(struct rdt_resource *r, int id)
{
	struct rdt_domain *domain;
	struct list_head *l;

	list_for_each(l, &r->domains) {
		domain = list_entry(l, struct rdt_domain, list);
		if (id == domain->id)
			return domain;
	}

	return NULL;
}

void resctrl_arch_reset_resources(void)
{
	/* not implemented for the RISC-V resctrl implementation */
}

void *resctrl_arch_mon_ctx_alloc(struct rdt_resource *r, int evtid)
{
	/* RISC-V can always read an rmid, nothing needs allocating */
	return NULL;
}

void resctrl_arch_mon_ctx_free(struct rdt_resource *r, int evtid, void *ctx)
{
	/* not implemented for the RISC-V resctrl interface */
}

bool resctrl_arch_is_evt_configurable(enum resctrl_event_id evt)
{
	return false;
}

int resctrl_arch_rmid_read(struct rdt_resource  *r, struct rdt_domain *d,
			   u32 closid, u32 rmid, enum resctrl_event_id eventid,
			   u64 *val, void *ignored)
{
	/*
	 * The current Qemu implementation of CBQRI capacity and bandwidth
	 * controllers do not emulate the utilization of resources over
	 * time. Therefore, Qemu currently sets the invalid bit in
	 * cc_mon_ctr_val and bc_mon_ctr_val, and there is no meaningful
	 * value other than 0 to return for reading an RMID (e.g. MCID in
	 * CBQRI terminology)
	 */

	return 0;
}

void resctrl_arch_reset_rmid(struct rdt_resource *r, struct rdt_domain *d,
			     u32 closid, u32 rmid, enum resctrl_event_id eventid)
{
	/* not implemented for the RISC-V resctrl interface */
}

void resctrl_arch_mon_event_config_read(void *info)
{
	/* not implemented for the RISC-V resctrl interface */
}

void resctrl_arch_mon_event_config_write(void *info)
{
	/* not implemented for the RISC-V resctrl interface */
}

void resctrl_arch_reset_rmid_all(struct rdt_resource *r, struct rdt_domain *d)
{
	/* not implemented for the RISC-V resctrl implementation */
}

/* Set capacity block mask (cc_block_mask) */
static void cbqri_set_cbm(struct cbqri_controller *ctrl, u64 cbm)
{
		int reg_offset;
		u64 reg;

		reg_offset = CBQRI_CC_BLOCK_MASK_OFF;
		reg = ioread64(ctrl->base + reg_offset);

		reg = cbm;
		iowrite64(reg, ctrl->base + reg_offset);

		reg = ioread64(ctrl->base + reg_offset);
}

/* Set the Rbwb (reserved bandwidth blocks) field in bc_bw_alloc */
static void cbqri_set_rbwb(struct cbqri_controller *ctrl, u64 rbwb)
{
		int reg_offset;
		u64 reg;

		reg_offset = CBQRI_BC_BW_ALLOC_OFF;
		reg = ioread64(ctrl->base + reg_offset);
		reg |= rbwb & CBQRI_CONTROL_REGISTERS_RBWB_MASK;
		iowrite64(reg, ctrl->base + reg_offset);
}

/* Get the Rbwb (reserved bandwidth blocks) field in bc_bw_alloc */
static u64 cbqri_get_rbwb(struct cbqri_controller *ctrl)
{
		int reg_offset;
		u64 reg;

		reg_offset = CBQRI_BC_BW_ALLOC_OFF;
		reg = ioread64(ctrl->base + reg_offset);
		reg &= CBQRI_CONTROL_REGISTERS_RBWB_MASK;

		return reg;
}

static int qos_wait_busy_flag(struct cbqri_controller *ctrl, int reg_offset)
{
	unsigned long timeout = jiffies + (HZ / 10); /* Timeout after 100ms */
	u64 reg;
	int busy;

	while (time_before(jiffies, timeout)) {
		reg = ioread64(ctrl->base + reg_offset);
		busy = (reg >> CBQRI_CONTROL_REGISTERS_BUSY_SHIFT) &
			CBQRI_CONTROL_REGISTERS_BUSY_MASK;
		if (!busy)
			return 0;
	}

	pr_warn("%s(): busy timeout", __func__);
	return -EIO;
}

/* Perform operation on capacity controller */
static int do_capacity_alloc_op(struct cbqri_controller *ctrl, int operation,
				enum resctrl_conf_type type, int rcid)
{
	int reg_offset = CBQRI_CC_ALLOC_CTL_OFF;
	int status;
	u64 reg;

	reg = ioread64(ctrl->base + reg_offset);
	reg &= ~(CBQRI_CONTROL_REGISTERS_OP_MASK << CBQRI_CONTROL_REGISTERS_OP_SHIFT);
	reg |= (operation & CBQRI_CONTROL_REGISTERS_OP_MASK) <<
		CBQRI_CONTROL_REGISTERS_OP_SHIFT;
	reg &= ~(CBQRI_CONTROL_REGISTERS_RCID_MASK <<
		 CBQRI_CONTROL_REGISTERS_RCID_SHIFT);
	reg |= (rcid & CBQRI_CONTROL_REGISTERS_RCID_MASK) <<
		CBQRI_CONTROL_REGISTERS_RCID_SHIFT;

	/* CBQRI capacity AT is only supported on caches for now */
	if (ctrl->ctrl_info->type == CBQRI_CONTROLLER_TYPE_CAPACITY &&
	    ((ctrl->ctrl_info->cache.cache_level == 2 && is_cdp_l2_enabled) ||
	    (ctrl->ctrl_info->cache.cache_level == 3 && is_cdp_l3_enabled))) {
		reg &= ~(CBQRI_CONTROL_REGISTERS_AT_MASK <<
			 CBQRI_CONTROL_REGISTERS_AT_SHIFT);
		switch (type) {
		case CDP_CODE:
			reg |= (CBQRI_CONTROL_REGISTERS_AT_CODE &
				CBQRI_CONTROL_REGISTERS_AT_MASK) <<
				CBQRI_CONTROL_REGISTERS_AT_SHIFT;
			break;
		case CDP_DATA:
		default:
			reg |= (CBQRI_CONTROL_REGISTERS_AT_DATA &
				CBQRI_CONTROL_REGISTERS_AT_MASK) <<
				CBQRI_CONTROL_REGISTERS_AT_SHIFT;
			break;
		}
	}

	iowrite64(reg, ctrl->base + reg_offset);

	if (qos_wait_busy_flag(ctrl, reg_offset) < 0) {
		pr_err("%s(): BUSY timeout when executing the operation", __func__);
		return -EIO;
	}

	reg = ioread64(ctrl->base + reg_offset);
	status = (reg >> CBQRI_CONTROL_REGISTERS_STATUS_SHIFT) &
		  CBQRI_CONTROL_REGISTERS_STATUS_MASK;
	if (status != 1) {
		pr_err("%s(): operation %d failed: status=%d", __func__, operation, status);
		return -EIO;
	}

	return 0;
}

/* Perform operation on bandwidth controller */
static int do_bandwidth_alloc_op(struct cbqri_controller *ctrl, int operation, int rcid)
{
	int reg_offset = CBQRI_BC_ALLOC_CTL_OFF;
	int status;
	u64 reg;

	reg = ioread64(ctrl->base + reg_offset);
	reg &= ~(CBQRI_CONTROL_REGISTERS_OP_MASK << CBQRI_CONTROL_REGISTERS_OP_SHIFT);
	reg |=  (operation & CBQRI_CONTROL_REGISTERS_OP_MASK) <<
		 CBQRI_CONTROL_REGISTERS_OP_SHIFT;
	reg &= ~(CBQRI_CONTROL_REGISTERS_RCID_MASK << CBQRI_CONTROL_REGISTERS_RCID_SHIFT);
	reg |=  (rcid & CBQRI_CONTROL_REGISTERS_RCID_MASK) <<
		 CBQRI_CONTROL_REGISTERS_RCID_SHIFT;
	iowrite64(reg, ctrl->base + reg_offset);

	if (qos_wait_busy_flag(ctrl, reg_offset) < 0) {
		pr_err("%s(): BUSY timeout when executing the operation", __func__);
		return -EIO;
	}

	reg = ioread64(ctrl->base + reg_offset);
	status = (reg >> CBQRI_CONTROL_REGISTERS_STATUS_SHIFT) &
		  CBQRI_CONTROL_REGISTERS_STATUS_MASK;
	if (status != 1) {
		pr_err("%s(): operation %d failed with status = %d",
		       __func__, operation, status);
		return -EIO;
	}

	return 0;
}

static int cbqri_apply_bw_config(struct cbqri_resctrl_dom *hw_dom, u32 closid,
				 enum resctrl_conf_type type, struct cbqri_config *cfg)
{
	struct cbqri_controller *ctrl = hw_dom->hw_ctrl;
	int ret = 0;
	u64 rbwb;

	if (cfg->cbm != hw_dom->ctrl_val[closid]) {
		/* Store the new rbwb in the ctrl_val array for this closid in this domain */
		hw_dom->ctrl_val[closid] = cfg->rbwb;

		/* set reserved bandwidth blocks */
		cbqri_set_rbwb(ctrl, cfg->rbwb);
		/* get reserved bandwidth blocks */
		rbwb = cbqri_get_rbwb(ctrl);

		/* Capacity config limit operation for RCID (closid) */
		ret = do_capacity_alloc_op(ctrl, CBQRI_CC_ALLOC_CTL_OP_CONFIG_LIMIT, type, closid);
		if (ret < 0) {
			pr_err("%s(): operation failed: ret = %d", __func__, ret);
			return ret;
		}

		/* Clear rbwb before read limit to verify op works*/
		cbqri_set_rbwb(ctrl, 0);

		/* Bandwidth allocation read limit operation for RCID (closid) to verify */
		ret = do_capacity_alloc_op(ctrl, CBQRI_CC_ALLOC_CTL_OP_READ_LIMIT, type, closid);
		if (ret < 0) {
			pr_err("%s(): operation failed: ret = %d", __func__, ret);
			return ret;
		}

		/* Read capacity block mask for RCID (closid) to verify */
		rbwb = cbqri_get_rbwb(ctrl);
	}

	return ret;
}

static int cbqri_apply_cache_config(struct cbqri_resctrl_dom *hw_dom, u32 closid,
				    enum resctrl_conf_type type, struct cbqri_config *cfg)
{
	struct cbqri_controller *ctrl = hw_dom->hw_ctrl;
	int reg_offset, err = 0;
	u64 reg;

	if (cfg->cbm != hw_dom->ctrl_val[closid]) {
		/* store the new cbm in the ctrl_val array for this closid in this domain */
		hw_dom->ctrl_val[closid] = cfg->cbm;

		/* set capacity block mask (cc_block_mask) */
		cbqri_set_cbm(ctrl, cfg->cbm);

		/* Capacity config limit operation for RCID (closid) */
		err = do_capacity_alloc_op(ctrl, CBQRI_CC_ALLOC_CTL_OP_CONFIG_LIMIT, type, closid);
		if (err < 0) {
			pr_err("%s(): operation failed: err = %d", __func__, err);
			return err;
		}

		/* clear cc_block_mask before read limit to verify op works*/
		cbqri_set_cbm(ctrl, 0);

		/* Capacity read limit operation for RCID (closid) to verify */
		err = do_capacity_alloc_op(ctrl, CBQRI_CC_ALLOC_CTL_OP_READ_LIMIT, type, closid);
		if (err < 0) {
			pr_err("%s(): operation failed: err = %d", __func__, err);
			return err;
		}

		/* Read capacity block mask for RCID (closid) to verify */
		reg_offset = CBQRI_CC_BLOCK_MASK_OFF;
		reg = ioread64(ctrl->base + reg_offset);
	}

	return err;
}

int resctrl_arch_update_one(struct rdt_resource *r, struct rdt_domain *d,
			    u32 closid, enum resctrl_conf_type t, u32 cfg_val)
{
	struct cbqri_resctrl_res *res;
	struct cbqri_resctrl_dom *dom;
	struct cbqri_config cfg = {0};
	int err = 0;

	res = container_of(r, struct cbqri_resctrl_res, resctrl_res);
	dom = container_of(d, struct cbqri_resctrl_dom, resctrl_dom);

	if (!r->alloc_capable)
		return -EINVAL;

	switch (r->rid) {
	case RDT_RESOURCE_L2:
	case RDT_RESOURCE_L3:
		cfg.cbm = cfg_val;
		err = cbqri_apply_cache_config(dom, closid, t, &cfg);
		break;
	case RDT_RESOURCE_MBA:
		cfg.rbwb = cfg_val;
		err = cbqri_apply_bw_config(dom, closid, t, &cfg);
		break;
	default:
		return -EINVAL;
	}

	return err;
}

int resctrl_arch_update_domains(struct rdt_resource *r, u32 closid)
{
	struct resctrl_staged_config *cfg;
	enum resctrl_conf_type t;
	struct rdt_domain *d;
	int err = 0;

	list_for_each_entry(d, &r->domains, list) {
		for (t = 0; t < CDP_NUM_TYPES; t++) {
			cfg = &d->staged_config[t];
			if (!cfg->have_new_ctrl)
				continue;
			err = resctrl_arch_update_one(r, d, closid, t, cfg->new_ctrl);
			if (err) {
				pr_debug("%s(): return err=%d", __func__, err);
				return err;
			}
		}
	}
	return err;
}

u32 resctrl_arch_get_config(struct rdt_resource *r, struct rdt_domain *d,
			    u32 closid, enum resctrl_conf_type type)
{
	struct cbqri_resctrl_res *hw_res;
	struct cbqri_resctrl_dom *hw_dom;
	struct cbqri_controller *ctrl;
	int reg_offset;
	int err;
	u64 reg;

	hw_res = container_of(r, struct cbqri_resctrl_res, resctrl_res);
	hw_dom = container_of(d, struct cbqri_resctrl_dom, resctrl_dom);

	ctrl = hw_dom->hw_ctrl;

	if (!r->alloc_capable)
		return -EINVAL;

	switch (r->rid) {
	case RDT_RESOURCE_L2:
	case RDT_RESOURCE_L3:

		/* Clear cc_block_mask before read limit operation */
		cbqri_set_cbm(ctrl, 0);

		/* Capacity read limit operation for RCID (closid) */
		err = do_capacity_alloc_op(ctrl, CBQRI_CC_ALLOC_CTL_OP_READ_LIMIT, type, closid);
		if (err < 0) {
			pr_err("%s(): operation failed: err = %d", __func__, err);
			return -EIO;
		}

		/* Read capacity block mask for RCID (closid) */
		reg_offset = CBQRI_CC_BLOCK_MASK_OFF;
		reg = ioread64(ctrl->base + reg_offset);

		/* Update the config value for the closid in this domain */
		hw_dom->ctrl_val[closid] = reg;

		return hw_dom->ctrl_val[closid];

	case RDT_RESOURCE_MBA:

		/* Capacity read limit operation for RCID (closid) */
		err = do_bandwidth_alloc_op(ctrl, CBQRI_CC_ALLOC_CTL_OP_READ_LIMIT, closid);
		if (err < 0) {
			pr_err("%s(): operation failed: err = %d", __func__, err);
			return -EIO;
		}

		hw_dom->ctrl_val[closid] = cbqri_get_rbwb(ctrl);

		return hw_dom->ctrl_val[closid];

	default:
		return -EINVAL;
	}
}

static int qos_discover_controller_feature(struct cbqri_controller *ctrl,
					   int reg_offset,
					   int operation,
					   int *status,
					   bool *access_type_supported)
{
	u64 reg, saved_reg;
	int at;

	/* Keep the initial register value to preserve the WPRI fields */
	reg = ioread64(ctrl->base + reg_offset);
	saved_reg = reg;

	/* Execute the requested operation to find if the register is implemented */
	reg &= ~(CBQRI_CONTROL_REGISTERS_OP_MASK << CBQRI_CONTROL_REGISTERS_OP_SHIFT);
	reg |= (operation & CBQRI_CONTROL_REGISTERS_OP_MASK) << CBQRI_CONTROL_REGISTERS_OP_SHIFT;
	iowrite64(reg, ctrl->base + reg_offset);
	if (qos_wait_busy_flag(ctrl, reg_offset) < 0) {
		pr_err("%s(): BUSY timeout when executing the operation", __func__);
		return -EIO;
	}

	/* Get the operation status */
	reg = ioread64(ctrl->base + reg_offset);
	*status = (reg >> CBQRI_CONTROL_REGISTERS_STATUS_SHIFT) &
		   CBQRI_CONTROL_REGISTERS_STATUS_MASK;

	/*
	 * Check for the AT support if the register is implemented
	 * (if not, the status value will remain 0)
	 */
	if (*status != 0) {
		/* Set the AT field to a valid value */
		reg = saved_reg;
		reg &= ~(CBQRI_CONTROL_REGISTERS_AT_MASK << CBQRI_CONTROL_REGISTERS_AT_SHIFT);
		reg |= CBQRI_CONTROL_REGISTERS_AT_CODE << CBQRI_CONTROL_REGISTERS_AT_SHIFT;
		iowrite64(reg, ctrl->base + reg_offset);
		if (qos_wait_busy_flag(ctrl, reg_offset) < 0) {
			pr_err("%s(): BUSY timeout when setting AT field", __func__);
			return -EIO;
		}

		/*
		 * If the AT field value has been reset to zero,
		 * then the AT support is not present
		 */
		reg = ioread64(ctrl->base + reg_offset);
		at = (reg >> CBQRI_CONTROL_REGISTERS_AT_SHIFT) & CBQRI_CONTROL_REGISTERS_AT_MASK;
		if (at == CBQRI_CONTROL_REGISTERS_AT_CODE)
			*access_type_supported = true;
		else
			*access_type_supported = false;
	}

	/* Restore the original register value */
	iowrite64(saved_reg, ctrl->base + reg_offset);
	if (qos_wait_busy_flag(ctrl, reg_offset) < 0) {
		pr_err("%s(): BUSY timeout when restoring the original register value", __func__);
		return -EIO;
	}

	return 0;
}

/*
 * Note: for the purposes of the CBQRI proof-of-concept, debug logging
 * has been left in this function that discovers the properties of CBQRI
 * capable controllers in the system. pr_debug calls would be removed
 * before submitting non-RFC patches.
 */
static int qos_resctrl_discover_controller(struct cbqri_controller_info *ctrl_info,
					   struct cbqri_controller *ctrl)
{
	int err = 0, status;
	u64 reg;

	pr_debug("controller info: type=%d addr=0x%lx size=%lu max-rcid=%u max-mcid=%u",
		 ctrl_info->type, ctrl_info->addr, ctrl_info->size,
		 ctrl_info->rcid_count, ctrl_info->mcid_count);

	/* max_rmid is used by resctrl_arch_system_num_rmid_idx() */
	max_rmid = ctrl_info->mcid_count;

	ctrl->ctrl_info = ctrl_info;

	/* Try to access the memory-mapped CBQRI registers */
	if (!request_mem_region(ctrl_info->addr, ctrl_info->size, "cbqri_controller")) {
		pr_debug("%s(): return %d", __func__, err);
		return err;
	}
	ctrl->base = ioremap(ctrl_info->addr, ctrl_info->size);
	if (!ctrl->base) {
		pr_debug("%s(): goto err_release_mem_region", __func__);
		goto err_release_mem_region;
	}

	ctrl->alloc_capable = false;
	ctrl->mon_capable = false;

	/* Discover capacity allocation and monitoring features */
	if (ctrl_info->type == CBQRI_CONTROLLER_TYPE_CAPACITY) {
		pr_debug("probe capacity controller");

		/* Make sure the register is implemented */
		reg = ioread64(ctrl->base + CBQRI_CC_CAPABILITIES_OFF);
		if (reg == 0) {
			err = -ENODEV;
			goto err_iounmap;
		}

		ctrl->ver_minor = reg & CBQRI_CC_CAPABILITIES_VER_MINOR_MASK;
		ctrl->ver_major = reg & CBQRI_CC_CAPABILITIES_VER_MAJOR_MASK;

		ctrl->cc.supports_alloc_op_flush_rcid = (reg >> CBQRI_CC_CAPABILITIES_FRCID_SHIFT)
			& CBQRI_CC_CAPABILITIES_FRCID_MASK;

		ctrl->cc.ncblks = (reg >> CBQRI_CC_CAPABILITIES_NCBLKS_SHIFT) &
				   CBQRI_CC_CAPABILITIES_NCBLKS_MASK;

		/* Calculate size of capacity block in bytes */
		ctrl->cc.blk_size = ctrl_info->cache.cache_size / ctrl->cc.ncblks;
		ctrl->cc.cache_level = ctrl_info->cache.cache_level;

		pr_debug("version=%d.%d ncblks=%d blk_size=%d cache_level=%d",
			 ctrl->ver_major, ctrl->ver_minor,
			 ctrl->cc.ncblks, ctrl->cc.blk_size, ctrl->cc.cache_level);

		/* Discover monitoring features */
		err = qos_discover_controller_feature(ctrl, CBQRI_CC_MON_CTL_OFF,
						      CBQRI_CC_MON_CTL_OP_READ_COUNTER,
						      &status, &ctrl->cc.supports_mon_at_code);
		if (err) {
			pr_err("%s() failed to discover cc_mon_ctl feature", __func__);
			goto err_iounmap;
		}

		if (status == CBQRI_CC_MON_CTL_STATUS_SUCCESS) {
			pr_debug("cc_mon_ctl is supported");
			ctrl->cc.supports_mon_op_config_event = true;
			ctrl->cc.supports_mon_op_read_counter = true;
			ctrl->mon_capable = true;
		} else {
			pr_debug("cc_mon_ctl is NOT supported");
			ctrl->cc.supports_mon_op_config_event = false;
			ctrl->cc.supports_mon_op_read_counter = false;
			ctrl->mon_capable = false;
		}
		/*
		 * AT data is "always" supported as it has the same value
		 * than when AT field is not supported.
		 */
		ctrl->cc.supports_mon_at_data = true;
		pr_debug("supports_mon_at_data: %d, supports_mon_at_code: %d",
			 ctrl->cc.supports_mon_at_data, ctrl->cc.supports_mon_at_code);

		/* Discover allocation features */
		err = qos_discover_controller_feature(ctrl, CBQRI_CC_ALLOC_CTL_OFF,
						      CBQRI_CC_ALLOC_CTL_OP_READ_LIMIT,
						      &status, &ctrl->cc.supports_alloc_at_code);
		if (err) {
			pr_err("%s() failed to discover cc_alloc_ctl feature", __func__);
			goto err_iounmap;
		}

		if (status == CBQRI_CC_ALLOC_CTL_STATUS_SUCCESS) {
			pr_debug("cc_alloc_ctl is supported");
			ctrl->cc.supports_alloc_op_config_limit = true;
			ctrl->cc.supports_alloc_op_read_limit = true;
			ctrl->alloc_capable = true;
			exposed_alloc_capable = true;
		} else {
			pr_debug("cc_alloc_ctl is NOT supported");
			ctrl->cc.supports_alloc_op_config_limit = false;
			ctrl->cc.supports_alloc_op_read_limit = false;
			ctrl->alloc_capable = false;
		}
		/*
		 * AT data is "always" supported as it has the same value
		 * than when AT field is not supported
		 */
		ctrl->cc.supports_alloc_at_data = true;
		pr_debug("supports_alloc_at_data: %d, supports_alloc_at_code: %d",
			 ctrl->cc.supports_alloc_at_data,
			 ctrl->cc.supports_alloc_at_code);
	} else if (ctrl_info->type == CBQRI_CONTROLLER_TYPE_BANDWIDTH) {
		pr_debug("probe bandwidth controller");

		/* Make sure the register is implemented */
		reg = ioread64(ctrl->base + CBQRI_BC_CAPABILITIES_OFF);
		if (reg == 0) {
			err = -ENODEV;
			goto err_iounmap;
		}

		ctrl->ver_minor = reg & CBQRI_BC_CAPABILITIES_VER_MINOR_MASK;
		ctrl->ver_major = reg & CBQRI_BC_CAPABILITIES_VER_MAJOR_MASK;

		ctrl->bc.nbwblks = (reg >> CBQRI_BC_CAPABILITIES_NBWBLKS_SHIFT) &
				    CBQRI_BC_CAPABILITIES_NBWBLKS_MASK;
		ctrl->bc.mrbwb = (reg >> CBQRI_BC_CAPABILITIES_MRBWB_SHIFT) &
				  CBQRI_BC_CAPABILITIES_MRBWB_MASK;

		pr_debug("version=%d.%d nbwblks=%d mrbwb=%d",
			 ctrl->ver_major, ctrl->ver_minor,
			 ctrl->bc.nbwblks, ctrl->bc.mrbwb);

		// Discover monitoring features
		err = qos_discover_controller_feature(ctrl, CBQRI_BC_MON_CTL_OFF,
						      CBQRI_BC_MON_CTL_OP_READ_COUNTER,
						      &status, &ctrl->bc.supports_mon_at_code);
		if (err) {
			pr_err("%s() failed to discover bc_mon_ctl feature", __func__);
			goto err_iounmap;
		}

		if (status == CBQRI_BC_MON_CTL_STATUS_SUCCESS) {
			pr_debug("bc_mon_ctl is supported");
			ctrl->bc.supports_mon_op_config_event = true;
			ctrl->bc.supports_mon_op_read_counter = true;
			ctrl->mon_capable = true;
			exposed_mon_capable = true;
		} else {
			pr_debug("bc_mon_ctl is NOT supported");
			ctrl->bc.supports_mon_op_config_event = false;
			ctrl->bc.supports_mon_op_read_counter = false;
			ctrl->mon_capable = false;
		}
		// AT data is "always" supported as it has the same value
		// than when AT field is not supported
		ctrl->bc.supports_mon_at_data = true;
		pr_debug("supports_mon_at_data: %d, supports_mon_at_code: %d",
			 ctrl->bc.supports_mon_at_data, ctrl->bc.supports_mon_at_code);

		// Discover allocation features
		err = qos_discover_controller_feature(ctrl, CBQRI_BC_ALLOC_CTL_OFF,
						      CBQRI_BC_ALLOC_CTL_OP_READ_LIMIT,
						      &status, &ctrl->bc.supports_alloc_at_code);
		if (err) {
			pr_err("%s() failed to discover bc_alloc_ctl feature", __func__);
			goto err_iounmap;
		}

		if (status == CBQRI_BC_ALLOC_CTL_STATUS_SUCCESS) {
			pr_debug("bc_alloc_ctl is supported");
			ctrl->bc.supports_alloc_op_config_limit = true;
			ctrl->bc.supports_alloc_op_read_limit = true;
			ctrl->alloc_capable = true;
			exposed_alloc_capable = true;
		} else {
			pr_debug("bc_alloc_ctl is NOT supported");
			ctrl->bc.supports_alloc_op_config_limit = false;
			ctrl->bc.supports_alloc_op_read_limit = false;
			ctrl->alloc_capable = false;
		}

		/*
		 * AT data is "always" supported as it has the same value
		 * than when AT field is not supported
		 */
		ctrl->bc.supports_alloc_at_data = true;
		pr_debug("supports_alloc_at_data: %d, supports_alloc_at_code: %d",
			 ctrl->bc.supports_alloc_at_data, ctrl->bc.supports_alloc_at_code);
	} else {
		pr_err("controller type is UNKNOWN");
		err = -ENODEV;
		goto err_release_mem_region;
	}

	return 0;

err_iounmap:
	pr_err("%s(): err_iounmap", __func__);
	iounmap(ctrl->base);

err_release_mem_region:
	pr_err("%s(): err_release_mem_region", __func__);
	release_mem_region(ctrl_info->addr, ctrl_info->size);

	return err;
}

static struct rdt_domain *qos_new_domain(struct cbqri_controller *ctrl)
{
	struct cbqri_resctrl_dom *hw_dom;
	struct rdt_domain *domain;

	hw_dom = kzalloc(sizeof(*hw_dom), GFP_KERNEL);
	if (!hw_dom)
		return NULL;

	/* associate this cbqri_controller with the domain */
	hw_dom->hw_ctrl = ctrl;

	/* the rdt_domain struct from inside the cbqri_resctrl_dom struct */
	domain = &hw_dom->resctrl_dom;

	INIT_LIST_HEAD(&domain->list);

	return domain;
}

static int qos_bw_blocks_to_percentage(struct cbqri_controller *ctrl, int blocks)
{
	int percentage;

	if (ctrl->ctrl_info->type != CBQRI_CONTROLLER_TYPE_BANDWIDTH)
		return -1;

	blocks *= 100;
	percentage = blocks / ctrl->bc.nbwblks;
	if (blocks % ctrl->bc.nbwblks)
		percentage++;

	return percentage;
}

static int domain_setup_ctrlval(struct rdt_resource *r, struct rdt_domain *d)
{
	struct cbqri_resctrl_res *hw_res;
	struct cbqri_resctrl_dom *hw_dom;
	u64 *dc;
	int i;

	hw_res = container_of(r, struct cbqri_resctrl_res, resctrl_res);
	hw_dom = container_of(d, struct cbqri_resctrl_dom, resctrl_dom);

	dc = kmalloc_array(hw_res->max_rcid, sizeof(*hw_dom->ctrl_val),
			   GFP_KERNEL);
	if (!dc)
		return -ENOMEM;

	hw_dom->ctrl_val = dc;

	for (i = 0; i < hw_res->max_rcid; i++, dc++)
		*dc = r->default_ctrl;

	/*
	 * The Qemu implementation for the CBQRI proof-of-concept has
	 * known reset values for all registers. A proper solution would
	 * be to perform a CONFIG_LIMIT operation to set the default for
	 * each RCID
	 */
	return 0;
}

static int qos_resctrl_add_controller_domain(struct cbqri_controller *ctrl, int *id)
{
	struct rdt_domain *domain, *mon_domain = NULL;
	struct cbqri_resctrl_res *cbqri_res = NULL;
	struct cbqri_resctrl_dom *hw_dom_to_free = NULL;
	struct rdt_resource *res = NULL;
	int internal_id = *id;
	int err;

	domain = qos_new_domain(ctrl);
	if (!domain)
		return -ENOSPC;

	if (ctrl->ctrl_info->type == CBQRI_CONTROLLER_TYPE_CAPACITY) {
		cpumask_copy(&domain->cpu_mask, &ctrl->ctrl_info->cache.cpu_mask);

		if (ctrl->ctrl_info->cache.cache_level == 2) {
			cbqri_res = &cbqri_resctrl_resources[RDT_RESOURCE_L2];
			cbqri_res->max_rcid = ctrl->ctrl_info->rcid_count;
			cbqri_res->max_mcid = ctrl->ctrl_info->mcid_count;
			res = &cbqri_res->resctrl_res;
			res->num_rmid = ctrl->ctrl_info->mcid_count;
			res->fflags = RFTYPE_RES_CACHE;
			res->rid = RDT_RESOURCE_L2;
			res->name = "L2";
			res->alloc_capable = ctrl->alloc_capable;
			res->mon_capable = ctrl->mon_capable;
			res->format_str = "%d=%0*x";
			res->cache_level = 2;
			res->data_width = 3;
			res->cache.arch_has_sparse_bitmasks = false;
			res->cache.arch_has_per_cpu_cfg = false;
			res->cache.shareable_bits = 0x000;
			res->cache.min_cbm_bits = 1;
			res->cache.cbm_len = ctrl->cc.ncblks;
			res->default_ctrl = BIT_MASK(ctrl->cc.ncblks) - 1;
		} else if (ctrl->ctrl_info->cache.cache_level == 3) {
			cbqri_res = &cbqri_resctrl_resources[RDT_RESOURCE_L3];
			cbqri_res->max_rcid = ctrl->ctrl_info->rcid_count;
			cbqri_res->max_mcid = ctrl->ctrl_info->mcid_count;
			res = &cbqri_res->resctrl_res;
			res->num_rmid = ctrl->ctrl_info->mcid_count;
			res->fflags = RFTYPE_RES_CACHE;
			res->rid = RDT_RESOURCE_L3;
			res->name = "L3";
			res->cache_level = 3;
			res->alloc_capable = ctrl->alloc_capable;
			res->mon_capable = ctrl->mon_capable;
			res->format_str = "%d=%0*x";
			res->data_width = 4;
			res->cache.arch_has_sparse_bitmasks = false;
			res->cache.arch_has_per_cpu_cfg = false;
			res->cache.shareable_bits = 0x000;
			res->cache.min_cbm_bits = 1;
			res->cache.cbm_len = ctrl->cc.ncblks;
			res->default_ctrl = BIT_MASK(ctrl->cc.ncblks) - 1;
		} else {
			pr_err("%s(): unknown cache level %d", __func__,
			       ctrl->ctrl_info->cache.cache_level);
			err = -ENODEV;
			goto err_free_domain;
		}
	} else if (ctrl->ctrl_info->type == CBQRI_CONTROLLER_TYPE_BANDWIDTH) {
		if (ctrl->alloc_capable) {
			cbqri_res = &cbqri_resctrl_resources[RDT_RESOURCE_MBA];
			cbqri_res->max_rcid = ctrl->ctrl_info->rcid_count;
			cbqri_res->max_mcid = ctrl->ctrl_info->mcid_count;
			res = &cbqri_res->resctrl_res;
			res->num_rmid = ctrl->ctrl_info->mcid_count;
			res->fflags = RFTYPE_RES_MB;
			res->rid = RDT_RESOURCE_MBA;
			res->name = "MB";
			res->alloc_capable = ctrl->alloc_capable;
			/*
			 * MBA resource is only for allocation and
			 * monitoring can only be done with L3 resource
			 */
			res->mon_capable = false;
			res->format_str = "%d=%*u";
			res->data_width = 4;
			/*
			 * MBA schemata expects percentage so convert
			 * maximum reserved bw blocks to percentage
			 */
			res->default_ctrl = qos_bw_blocks_to_percentage(ctrl, ctrl->bc.mrbwb);
			/*
			 * Use values from 0 to MBA_MAX instead of power of two values,
			 * see Intel System Programming Guide manual section 18.19.7.2
			 */
			res->membw.delay_linear = true;
			res->membw.arch_needs_linear = true;
			/* Disable non-linear values support */
			res->membw.throttle_mode = THREAD_THROTTLE_UNDEFINED;
			// The minimum percentage allowed by the CBQRI spec
			res->membw.min_bw = 1;
			res->membw.bw_gran = 1;
		}

		/*
		 * Monitoring is not possible with Intel MBA resource,
		 * so add a L3 domain if monitoring is supported by the CBQRI
		 * bandwidth controller.
		 */
		if (ctrl->mon_capable) {
			struct cbqri_resctrl_res *mon_cbqri_res;

			mon_cbqri_res = &cbqri_resctrl_resources[RDT_RESOURCE_L3];
			mon_domain = qos_new_domain(ctrl);
			if (!domain)
				goto err_free_mon_domain;

			/*
			 * For CBQRI, any cpu (technically a hart in RISC-V terms)
			 * can access the memory-mapped registers of any CBQRI
			 * controller in the system. Thus it does not matter for
			 * RISC-V which cpu runs the resctrl code.
			 */
			cpumask_setall(&domain->cpu_mask);

			err = domain_setup_ctrlval(&mon_cbqri_res->resctrl_res, mon_domain);
			if (err)
				goto err_free_mon_domain;

			mon_domain->id = internal_id;
			internal_id++;
			list_add_tail(&mon_domain->list, &mon_cbqri_res->resctrl_res.domains);
			err = resctrl_online_domain(res, mon_domain);
			if (err) {
				pr_debug("%s(): BW monitoring domain online failed", __func__);
				goto err_free_mon_domain;
			}
		}
	} else {
		pr_err("%s(): unknown resource %d", __func__, ctrl->ctrl_info->type);
		err = -ENODEV;
		goto err_free_domain;
	}

	domain->id = internal_id;
	err = domain_setup_ctrlval(res, domain);
	if (err)
		goto err_free_mon_domain;

	if (cbqri_res) {
		list_add_tail(&domain->list, &cbqri_res->resctrl_res.domains);
		*id = internal_id;
		err = resctrl_online_domain(res, domain);
		if (err) {
			pr_debug("%s(): failed to online cbqri_res domain", __func__);
			goto err_free_domain;
		}
	}

	return 0;

err_free_mon_domain:
	if (!mon_domain) {
		/*
		 * mon_domain is a struct rdt_domain which is a member of
		 * struct cbqri_resctrl_dom. That cbqri_resctrl_dom was
		 * allocated in qos_new_domain() and must be freed.
		 */
		hw_dom_to_free = container_of(mon_domain, struct cbqri_resctrl_dom, resctrl_dom);
		kfree(hw_dom_to_free);
	}

err_free_domain:
	/* similar to err_free_mon_domain but the ptr is 'domain' instead */
	hw_dom_to_free = container_of(domain, struct cbqri_resctrl_dom, resctrl_dom);
	kfree(hw_dom_to_free);

	return err;
}

static int qos_resctrl_setup_resources(void)
{
	struct rdt_domain *domain, *domain_temp;
	struct cbqri_controller_info *ctrl_info;
	struct cbqri_controller *ctrl;
	struct cbqri_resctrl_res *res;
	int err = 0, i, id = 0;

	list_for_each_entry(ctrl_info, &cbqri_controllers, list) {
		err = qos_resctrl_discover_controller(ctrl_info, &controllers[found_controllers]);
		if (err) {
			pr_err("%s(): qos_resctrl_discover_controller failed (%d)", __func__, err);
			goto err_unmap_controllers;
		}

		found_controllers++;
		if (found_controllers > MAX_CONTROLLERS) {
			pr_warn("%s(): increase MAX_CONTROLLERS value", __func__);
			break;
		}
	}

	if (!found_controllers) {
		pr_debug("%s(): qos_resctrl_discover_controller failed (no found_controllers)", __func__);
		return -ENODEV;
	}

	for (i = 0; i < RDT_NUM_RESOURCES; i++) {
		res = &cbqri_resctrl_resources[i];
		INIT_LIST_HEAD(&res->resctrl_res.domains);
		INIT_LIST_HEAD(&res->resctrl_res.evt_list);
		res->resctrl_res.rid = i;
	}

	for (i = 0; i < found_controllers; i++) {
		ctrl = &controllers[i];
		err = qos_resctrl_add_controller_domain(ctrl, &id);
		if (err) {
			pr_err("%s(): failed to add controller domain (%d)", __func__, err);
			goto err_free_controllers_list;
		}
		id++;

		/*
		 * CDP (code data prioritization) on x86 is similar to
		 * the AT (access type) field in CBQRI. CDP only supports
		 * caches so this must be a CBQRI capacity controller.
		 */
		if (ctrl->ctrl_info->type == CBQRI_CONTROLLER_TYPE_CAPACITY &&
		    ctrl->cc.supports_alloc_at_code &&
		    ctrl->cc.supports_alloc_at_data) {
			if (ctrl->ctrl_info->cache.cache_level == 2)
				exposed_cdp_l2_capable = true;
			else
				exposed_cdp_l3_capable = true;
		}
	}

	pr_debug("exposed_alloc_capable = %d", exposed_alloc_capable);
	pr_debug("exposed_mon_capable = %d", exposed_mon_capable);
	pr_debug("exposed_cdp_l2_capable = %d", exposed_cdp_l2_capable);
	pr_debug("exposed_cdp_l3_capable = %d", exposed_cdp_l3_capable);
	return 0;

err_free_controllers_list:
	for (i = 0; i < RDT_NUM_RESOURCES; i++) {
		res = &cbqri_resctrl_resources[i];
		list_for_each_entry_safe(domain, domain_temp, &res->resctrl_res.domains, list) {
			kfree(domain);
		}
	}

err_unmap_controllers:
	for (i = 0; i < found_controllers; i++) {
		iounmap(controllers[i].base);
		release_mem_region(controllers[i].ctrl_info->addr, controllers[i].ctrl_info->size);
	}

	return err;
}

int qos_resctrl_setup(void)
{
	s32 err;

	err = qos_resctrl_setup_resources();
	if (err) {
		pr_debug("%s(): failed with error %d\n", __func__, err);
		return err;
	}

	err = resctrl_init();

	return err;
}

int qos_resctrl_online_cpu(unsigned int cpu)
{
	resctrl_online_cpu(cpu);
	return 0;
}

int qos_resctrl_offline_cpu(unsigned int cpu)
{
	resctrl_offline_cpu(cpu);
	return 0;
}
