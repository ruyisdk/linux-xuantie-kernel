// SPDX-License-Identifier: GPL-2.0-only

#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

#include <linux/bitfield.h>
#include <linux/cacheinfo.h>
#include <linux/riscv_cbqri.h>
#include <linux/cpu.h>
#include <linux/cpufeature.h>
#include <linux/cpuhotplug.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/resctrl.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <asm/csr.h>
#include <asm/qos.h>

#include "cbqri_internal.h"

struct cbqri_resctrl_res {
	struct cbqri_controller *ctrl;
	struct rdt_resource     resctrl_res;
	bool                    cdp_enabled;
};

struct cbqri_resctrl_dom {
	struct rdt_ctrl_domain  resctrl_ctrl_dom;
	struct cbqri_controller *hw_ctrl;
};

static struct cbqri_resctrl_res cbqri_resctrl_resources[RDT_NUM_RESOURCES];

static bool exposed_alloc_capable;

/* Protects ctrl_domain list mutations across CPU hotplug. */
static DEFINE_MUTEX(cbqri_domain_list_lock);

static struct rdt_ctrl_domain *
cbqri_find_ctrl_domain(struct list_head *h, int id)
{
	struct rdt_domain_hdr *hdr = resctrl_find_domain(h, id, NULL);

	return hdr ? container_of(hdr, struct rdt_ctrl_domain, hdr) : NULL;
}

/* Map a hardware cache level to its resctrl resource id, or -ENODEV. */
static int cbqri_cache_level_to_rid(u32 cache_level)
{
	switch (cache_level) {
	case 2:
		return RDT_RESOURCE_L2;
	case 3:
		return RDT_RESOURCE_L3;
	default:
		return -ENODEV;
	}
}

static int cbqri_apply_cache_config_dom(struct cbqri_resctrl_dom *hw_dom,
					struct rdt_resource *r,
					u32 closid, enum resctrl_conf_type t,
					u64 cbm)
{
	struct cbqri_resctrl_res *hw_res =
		container_of(r, struct cbqri_resctrl_res, resctrl_res);
	struct cbqri_cc_config cfg = {
		.cbm = cbm,
		.at = (t == CDP_CODE) ? CBQRI_CONTROL_REGISTERS_AT_CODE :
					CBQRI_CONTROL_REGISTERS_AT_DATA,
		.cdp_enabled = hw_res->cdp_enabled,
	};

	return cbqri_apply_cache_config(hw_dom->hw_ctrl, closid, &cfg);
}

bool resctrl_arch_alloc_capable(void)
{
	return exposed_alloc_capable;
}

bool resctrl_arch_mon_capable(void)
{
	return false;
}

bool resctrl_arch_get_cdp_enabled(enum resctrl_res_level rid)
{
	if (rid != RDT_RESOURCE_L2 && rid != RDT_RESOURCE_L3)
		return false;
	return cbqri_resctrl_resources[rid].cdp_enabled;
}

int resctrl_arch_set_cdp_enabled(enum resctrl_res_level rid, bool enable)
{
	struct cbqri_resctrl_res *cbqri_res;

	if (rid != RDT_RESOURCE_L2 && rid != RDT_RESOURCE_L3)
		return -ENODEV;

	cbqri_res = &cbqri_resctrl_resources[rid];
	if (!cbqri_res->resctrl_res.cdp_capable)
		return -ENODEV;

	cbqri_res->cdp_enabled = enable;
	return 0;
}

struct rdt_resource *resctrl_arch_get_resource(enum resctrl_res_level l)
{
	if (l >= RDT_NUM_RESOURCES)
		return NULL;

	return &cbqri_resctrl_resources[l].resctrl_res;
}

/*
 * fs/resctrl unconditionally references the symbols below before checking
 * mon_capable. They are stubs for features CBQRI does not yet support.
 */
bool resctrl_arch_is_evt_configurable(enum resctrl_event_id evt)
{
	return false;
}

void *resctrl_arch_mon_ctx_alloc(struct rdt_resource *r,
				 enum resctrl_event_id evtid)
{
	return NULL;
}

void resctrl_arch_mon_ctx_free(struct rdt_resource *r,
			       enum resctrl_event_id evtid, void *arch_mon_ctx)
{
}

void resctrl_arch_mon_event_config_read(void *info)
{
}

void resctrl_arch_mon_event_config_write(void *info)
{
}

void resctrl_arch_reset_rmid_all(struct rdt_resource *r, struct rdt_mon_domain *d)
{
}

void resctrl_arch_reset_rmid(struct rdt_resource *r, struct rdt_mon_domain *d,
			     u32 closid, u32 rmid, enum resctrl_event_id eventid)
{
}

int resctrl_arch_rmid_read(struct rdt_resource *r, struct rdt_mon_domain *d,
			   u32 closid, u32 rmid, enum resctrl_event_id eventid,
			   u64 *val, void *arch_mon_ctx)
{
	return -ENODATA;
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

	if (!hw_res->ctrl)
		return 0;

	return hw_res->ctrl->rcid_count;
}

u32 resctrl_arch_system_num_rmid_idx(void)
{
	return 1;
}

u32 resctrl_arch_rmid_idx_encode(u32 closid, u32 rmid)
{
	return rmid;
}

void resctrl_arch_rmid_idx_decode(u32 idx, u32 *closid, u32 *rmid)
{
	*closid = RISCV_RESCTRL_EMPTY_CLOSID;
	*rmid = idx;
}

void resctrl_arch_set_cpu_default_closid_rmid(int cpu, u32 closid, u32 rmid)
{
	u32 srmcfg = FIELD_PREP(SRMCFG_RCID_MASK, closid) |
		     FIELD_PREP(SRMCFG_MCID_MASK, rmid);

	WRITE_ONCE(per_cpu(cpu_srmcfg_default, cpu), srmcfg);
}

void resctrl_arch_sched_in(struct task_struct *tsk)
{
	__switch_to_srmcfg(tsk);
}

void resctrl_arch_set_closid_rmid(struct task_struct *tsk, u32 closid, u32 rmid)
{
	u32 srmcfg = FIELD_PREP(SRMCFG_RCID_MASK, closid) |
		     FIELD_PREP(SRMCFG_MCID_MASK, rmid);

	WRITE_ONCE(tsk->thread.srmcfg, srmcfg);
}

void resctrl_arch_sync_cpu_closid_rmid(void *info)
{
	struct resctrl_cpu_defaults *r = info;

	lockdep_assert_preemption_disabled();

	if (r) {
		resctrl_arch_set_cpu_default_closid_rmid(smp_processor_id(),
							 r->closid, r->rmid);
	}

	resctrl_arch_sched_in(current);
}

bool resctrl_arch_match_closid(struct task_struct *tsk, u32 closid)
{
	return FIELD_GET(SRMCFG_RCID_MASK, READ_ONCE(tsk->thread.srmcfg)) == closid;
}

bool resctrl_arch_match_rmid(struct task_struct *tsk, u32 closid, u32 rmid)
{
	return FIELD_GET(SRMCFG_MCID_MASK, READ_ONCE(tsk->thread.srmcfg)) == rmid;
}

int resctrl_arch_update_one(struct rdt_resource *r, struct rdt_ctrl_domain *d,
			    u32 closid, enum resctrl_conf_type t, u32 cfg_val)
{
	struct cbqri_resctrl_dom *dom;

	dom = container_of(d, struct cbqri_resctrl_dom, resctrl_ctrl_dom);

	if (!r->alloc_capable)
		return -EINVAL;

	switch (r->rid) {
	case RDT_RESOURCE_L2:
	case RDT_RESOURCE_L3:
		return cbqri_apply_cache_config_dom(dom, r, closid, t, cfg_val);
	default:
		return -EINVAL;
	}
}

int resctrl_arch_update_domains(struct rdt_resource *r, u32 closid)
{
	struct resctrl_staged_config *cfg;
	enum resctrl_conf_type t;
	struct rdt_ctrl_domain *d;
	int err = 0;

	/* Walking r->ctrl_domains, ensure it can't race with cpuhp */
	lockdep_assert_cpus_held();

	list_for_each_entry(d, &r->ctrl_domains, hdr.list) {
		for (t = 0; t < CDP_NUM_TYPES; t++) {
			cfg = &d->staged_config[t];
			if (!cfg->have_new_ctrl)
				continue;
			err = resctrl_arch_update_one(r, d, closid, t, cfg->new_ctrl);
			if (err)
				return err;
		}
	}
	return err;
}

u32 resctrl_arch_get_config(struct rdt_resource *r, struct rdt_ctrl_domain *d,
			    u32 closid, enum resctrl_conf_type type)
{
	struct cbqri_resctrl_dom *hw_dom;
	struct cbqri_controller *ctrl;
	u32 at;
	u32 val;
	int err;

	hw_dom = container_of(d, struct cbqri_resctrl_dom, resctrl_ctrl_dom);
	ctrl = hw_dom->hw_ctrl;
	val = resctrl_get_default_ctrl(r);

	if (!r->alloc_capable)
		return val;

	switch (r->rid) {
	case RDT_RESOURCE_L2:
	case RDT_RESOURCE_L3:
		at = (type == CDP_CODE) ? CBQRI_CONTROL_REGISTERS_AT_CODE :
					  CBQRI_CONTROL_REGISTERS_AT_DATA;
		err = cbqri_read_cache_config(ctrl, closid, at, &val);
		if (err < 0)
			val = resctrl_get_default_ctrl(r);
		break;
	default:
		break;
	}

	return val;
}

void resctrl_arch_reset_all_ctrls(struct rdt_resource *r)
{
	struct cbqri_resctrl_res *hw_res;
	struct rdt_ctrl_domain *d;
	enum resctrl_conf_type t;
	u32 default_ctrl;
	int i;

	lockdep_assert_cpus_held();

	hw_res = container_of(r, struct cbqri_resctrl_res, resctrl_res);
	default_ctrl = resctrl_get_default_ctrl(r);

	if (!hw_res->ctrl)
		return;

	list_for_each_entry(d, &r->ctrl_domains, hdr.list) {
		for (i = 0; i < hw_res->ctrl->rcid_count; i++) {
			for (t = 0; t < CDP_NUM_TYPES; t++) {
				int rerr;

				rerr = resctrl_arch_update_one(r, d, i, t, default_ctrl);
				if (rerr)
					pr_err_ratelimited("rid=%d reset RCID %u type %u failed (%d)\n",
							   r->rid, i, t, rerr);
			}
		}
	}
}

static struct rdt_ctrl_domain *cbqri_new_domain(struct cbqri_controller *ctrl)
{
	struct cbqri_resctrl_dom *hw_dom;
	struct rdt_ctrl_domain *domain;

	hw_dom = kzalloc(sizeof(*hw_dom), GFP_KERNEL);
	if (!hw_dom)
		return NULL;

	hw_dom->hw_ctrl = ctrl;
	domain = &hw_dom->resctrl_ctrl_dom;

	INIT_LIST_HEAD(&domain->hdr.list);

	return domain;
}

static int cbqri_init_domain_ctrlval(struct rdt_resource *r, struct rdt_ctrl_domain *d)
{
	struct cbqri_resctrl_res *hw_res;
	enum resctrl_conf_type t;
	int err = 0;
	int i;

	hw_res = container_of(r, struct cbqri_resctrl_res, resctrl_res);

	for (i = 0; i < hw_res->ctrl->rcid_count; i++) {
		/*
		 * Seed both DATA and CODE staged slots so a later mount
		 * with -o cdp does not see stale CODE values.
		 * On non-AT controllers cbqri_cc_alloc_op() masks AT to 0
		 * so all three iterations land on the same hardware state.
		 * The redundant writes are harmless.
		 */
		for (t = 0; t < CDP_NUM_TYPES; t++) {
			err = resctrl_arch_update_one(r, d, i, t,
						      resctrl_get_default_ctrl(r));
			if (err)
				return err;
		}
	}
	return 0;
}

/*
 * Walk cbqri_controllers and pick one capacity controller (CC) per cache
 * level (L2/L3) to back the corresponding RDT_RESOURCE_L*. When more than
 * one CC sits at the same level (e.g. one per socket), they must agree on
 * rcid_count / ncblks / alloc_capable. A mismatch is fatal because resctrl
 * exposes a single set of caps per rid. The first matching controller wins.
 */
static int cbqri_resctrl_pick_caches(void)
{
	struct cbqri_controller *ctrl;
	int ret = 0;

	mutex_lock(&cbqri_controllers_lock);

	list_for_each_entry(ctrl, &cbqri_controllers, list) {
		struct cbqri_resctrl_res *cbqri_res;
		int rid;

		if (ctrl->type != CBQRI_CONTROLLER_TYPE_CAPACITY)
			continue;
		if (!ctrl->alloc_capable)
			continue;

		rid = cbqri_cache_level_to_rid(ctrl->cache.cache_level);
		if (rid < 0) {
			pr_info("skipping controller at unsupported cache level %u\n",
				ctrl->cache.cache_level);
			continue;
		}

		cbqri_res = &cbqri_resctrl_resources[rid];
		if (cbqri_res->ctrl) {
			/*
			 * CCs at the same cache level must agree on every cap
			 * resctrl exposes globally. Reject mismatches at pick
			 * time so the inconsistency is visible at boot.
			 */
			if (cbqri_res->ctrl->rcid_count != ctrl->rcid_count ||
			    cbqri_res->ctrl->cc.ncblks != ctrl->cc.ncblks ||
			    cbqri_res->ctrl->cc.supports_alloc_at_code !=
				    ctrl->cc.supports_alloc_at_code ||
			    cbqri_res->ctrl->alloc_capable != ctrl->alloc_capable) {
				pr_err("L%d controllers have mismatched capabilities\n",
				       ctrl->cache.cache_level);
				ret = -EINVAL;
				break;
			}
			continue;
		}

		cbqri_res->ctrl = ctrl;
	}

	mutex_unlock(&cbqri_controllers_lock);
	return ret;
}

/*
 * Fill the rdt_resource fields for one picked rid. An rid with no picked
 * controller is left untouched so it stays out of resctrl_arch_get_resource().
 */
static void cbqri_resctrl_control_init(struct cbqri_resctrl_res *cbqri_res)
{
	struct cbqri_controller *ctrl = cbqri_res->ctrl;
	struct rdt_resource *res = &cbqri_res->resctrl_res;

	if (!ctrl)
		return;

	switch (res->rid) {
	case RDT_RESOURCE_L2:
	case RDT_RESOURCE_L3:
		res->name = (res->rid == RDT_RESOURCE_L2) ? "L2" : "L3";
		res->schema_fmt = RESCTRL_SCHEMA_BITMAP;
		res->ctrl_scope = (res->rid == RDT_RESOURCE_L2) ?
				    RESCTRL_L2_CACHE : RESCTRL_L3_CACHE;
		res->cache.cbm_len = ctrl->cc.ncblks;
		res->default_ctrl = GENMASK(ctrl->cc.ncblks - 1, 0);
		res->cache.shareable_bits = 0;
		res->cache.min_cbm_bits = 1;
		res->cache.arch_has_sparse_bitmasks = false;
		res->cdp_capable = ctrl->cc.supports_alloc_at_code;
		res->alloc_capable = ctrl->alloc_capable;
		INIT_LIST_HEAD(&res->ctrl_domains);
		INIT_LIST_HEAD(&res->mon_domains);
		break;
	default:
		break;
	}
}

static void cbqri_resctrl_accumulate_caps(void)
{
	int rid;

	for (rid = 0; rid < RDT_NUM_RESOURCES; rid++) {
		struct cbqri_resctrl_res *hw_res = &cbqri_resctrl_resources[rid];

		if (!hw_res->ctrl)
			continue;
		if (hw_res->ctrl->alloc_capable)
			exposed_alloc_capable = true;
	}
}

/*
 * Create, list-insert, and online a fresh ctrl_domain backing ctrl on
 * resource res, seeded with cpu and identified by dom_id. Caller must
 * hold cbqri_domain_list_lock and must have already verified that no
 * existing ctrl_domain on res carries this id.
 */
static struct rdt_ctrl_domain *cbqri_create_ctrl_domain(struct cbqri_controller *ctrl,
							struct rdt_resource *res,
							unsigned int cpu, int dom_id)
{
	struct rdt_ctrl_domain *domain;
	struct list_head *pos = NULL;
	int err;

	domain = cbqri_new_domain(ctrl);
	if (!domain)
		return ERR_PTR(-ENOMEM);

	cpumask_set_cpu(cpu, &domain->hdr.cpu_mask);
	domain->hdr.id = dom_id;
	domain->hdr.type = RESCTRL_CTRL_DOMAIN;

	err = cbqri_init_domain_ctrlval(res, domain);
	if (err) {
		kfree(container_of(domain, struct cbqri_resctrl_dom,
				   resctrl_ctrl_dom));
		return ERR_PTR(err);
	}

	/* Insert sorted by id so user-visible ordering is deterministic. */
	resctrl_find_domain(&res->ctrl_domains, dom_id, &pos);
	list_add_tail(&domain->hdr.list, pos);

	resctrl_online_ctrl_domain(res, domain);

	return domain;
}

static int cbqri_attach_cpu_to_cap_ctrl(struct cbqri_controller *ctrl,
					unsigned int cpu)
{
	struct cbqri_resctrl_res *hw_res;
	struct rdt_ctrl_domain *domain;
	struct rdt_resource *res;
	int dom_id;
	int rid;

	rid = cbqri_cache_level_to_rid(ctrl->cache.cache_level);
	if (rid < 0)
		return 0;
	hw_res = &cbqri_resctrl_resources[rid];

	if (!hw_res->ctrl)
		return 0;

	res = &hw_res->resctrl_res;
	dom_id = ctrl->cache.cache_id;

	domain = cbqri_find_ctrl_domain(&res->ctrl_domains, dom_id);
	if (domain) {
		cpumask_set_cpu(cpu, &domain->hdr.cpu_mask);
		return 0;
	}

	domain = cbqri_create_ctrl_domain(ctrl, res, cpu, dom_id);
	if (IS_ERR(domain))
		return PTR_ERR(domain);

	return 0;
}

static void cbqri_detach_cpu_from_ctrl_domains(struct rdt_resource *res,
					       unsigned int cpu)
{
	struct rdt_ctrl_domain *domain, *tmp;

	list_for_each_entry_safe(domain, tmp, &res->ctrl_domains, hdr.list) {
		if (!cpumask_test_cpu(cpu, &domain->hdr.cpu_mask))
			continue;
		cpumask_clear_cpu(cpu, &domain->hdr.cpu_mask);
		if (cpumask_empty(&domain->hdr.cpu_mask)) {
			resctrl_offline_ctrl_domain(res, domain);
			list_del(&domain->hdr.list);
			kfree(container_of(domain, struct cbqri_resctrl_dom,
					   resctrl_ctrl_dom));
		}
	}
}

/*
 * Remove a CPU from every domain it was attached to. The per-resource
 * detach helpers act only when the CPU is set in a domain's mask, so this
 * is idempotent and undoes a partial online attach as well as a full
 * offline. Caller holds cbqri_domain_list_lock.
 */
static void cbqri_detach_cpu_from_all_ctrls(unsigned int cpu)
{
	int rid;

	lockdep_assert_held(&cbqri_domain_list_lock);

	for (rid = 0; rid < RDT_NUM_RESOURCES; rid++) {
		struct cbqri_resctrl_res *hw_res = &cbqri_resctrl_resources[rid];

		if (!hw_res->ctrl)
			continue;
		cbqri_detach_cpu_from_ctrl_domains(&hw_res->resctrl_res, cpu);
	}
}

/*
 * Attach a CPU to every controller that claims it. On failure, detach the
 * CPU from everything attached so far: the cpuhp core does not run this
 * state's offline teardown when its startup fails, so a partial attach
 * would otherwise leak into the domain cpu_masks. Caller holds
 * cbqri_domain_list_lock.
 */
static int cbqri_attach_cpu_to_all_ctrls(unsigned int cpu)
{
	struct cbqri_controller *ctrl;
	int err = 0;

	lockdep_assert_held(&cbqri_domain_list_lock);

	/*
	 * Hold cbqri_controllers_lock across the walk so a controller
	 * registered after boot cannot corrupt it. The register path takes
	 * it as a leaf and never cbqri_domain_list_lock, so this nesting
	 * cannot invert.
	 */
	mutex_lock(&cbqri_controllers_lock);
	list_for_each_entry(ctrl, &cbqri_controllers, list) {
		if (ctrl->type != CBQRI_CONTROLLER_TYPE_CAPACITY)
			continue;
		if (!cpumask_test_cpu(cpu, &ctrl->cache.cpu_mask))
			continue;
		if (!ctrl->alloc_capable)
			continue;

		err = cbqri_attach_cpu_to_cap_ctrl(ctrl, cpu);
		if (err) {
			cbqri_detach_cpu_from_all_ctrls(cpu);
			break;
		}
	}
	mutex_unlock(&cbqri_controllers_lock);

	return err;
}

static bool cbqri_resctrl_inited;

static void cbqri_resctrl_teardown(void)
{
	int rid;

	if (!cbqri_resctrl_inited)
		return;

	resctrl_exit();

	for (rid = 0; rid < RDT_NUM_RESOURCES; rid++) {
		struct cbqri_resctrl_res *hw_res = &cbqri_resctrl_resources[rid];

		hw_res->ctrl = NULL;
		hw_res->cdp_enabled = false;
	}
	exposed_alloc_capable = false;
	cbqri_resctrl_inited = false;
}

static int cbqri_resctrl_setup(void)
{
	int rid;
	int err;

	for (rid = 0; rid < RDT_NUM_RESOURCES; rid++)
		cbqri_resctrl_resources[rid].resctrl_res.rid = rid;

	err = cbqri_resctrl_pick_caches();
	if (err)
		return err;

	for (rid = 0; rid < RDT_NUM_RESOURCES; rid++)
		cbqri_resctrl_control_init(&cbqri_resctrl_resources[rid]);

	cbqri_resctrl_accumulate_caps();

	if (!exposed_alloc_capable) {
		pr_debug("no resctrl-capable CBQRI controllers found\n");
		return -ENODEV;
	}

	err = resctrl_init();
	if (err)
		return err;

	cbqri_resctrl_inited = true;
	return 0;
}

static int cbqri_resctrl_online_cpu(unsigned int cpu)
{
	int err;

	mutex_lock(&cbqri_domain_list_lock);
	err = cbqri_attach_cpu_to_all_ctrls(cpu);
	mutex_unlock(&cbqri_domain_list_lock);
	if (err)
		return err;

	/*
	 * Seed the per-CPU default RCID/MCID to the reserved (0, 0) pair and
	 * notify the resctrl core so it tracks this CPU in the default group.
	 */
	resctrl_arch_set_cpu_default_closid_rmid(cpu, 0, 0);
	resctrl_online_cpu(cpu);
	return 0;
}

static int cbqri_resctrl_offline_cpu(unsigned int cpu)
{
	resctrl_offline_cpu(cpu);

	mutex_lock(&cbqri_domain_list_lock);
	cbqri_detach_cpu_from_all_ctrls(cpu);
	mutex_unlock(&cbqri_domain_list_lock);
	return 0;
}

static int __init cbqri_arch_late_init(void)
{
	int err;

	if (!riscv_isa_extension_available(NULL, SSQOSID))
		return -ENODEV;

	err = cbqri_resctrl_setup();
	if (err)
		return err;

	err = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "cbqri:online",
				cbqri_resctrl_online_cpu,
				cbqri_resctrl_offline_cpu);
	if (err < 0) {
		cbqri_resctrl_teardown();
		return err;
	}

	return 0;
}
late_initcall(cbqri_arch_late_init);
