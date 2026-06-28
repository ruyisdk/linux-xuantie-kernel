/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _RESCTRL_H
#define _RESCTRL_H

#include <linux/cacheinfo.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/pid.h>
#include <linux/resctrl_types.h>
#include <linux/wait.h>

#ifdef CONFIG_ARCH_HAS_CPU_RESCTRL
#include <asm/resctrl.h>
#endif

/* CLOSID, RMID value used by the default control group */
#define RESCTRL_RESERVED_CLOSID		0
#define RESCTRL_RESERVED_RMID		0

#define RESCTRL_PICK_ANY_CPU		-1

#ifdef CONFIG_PROC_CPU_RESCTRL

int proc_resctrl_show(struct seq_file *m,
		      struct pid_namespace *ns,
		      struct pid *pid,
		      struct task_struct *tsk);

#endif

/* max value for struct rdt_*_domain's mbps_val */
#define MBA_MAX_MBPS   U32_MAX

/* Walk all possible resources, with variants for only controls or monitors. */
#define for_each_rdt_resource(_r)						\
	for ((_r) = resctrl_arch_get_resource(0);				\
	     (_r) && (_r)->rid < RDT_NUM_RESOURCES;				\
	     (_r) = resctrl_arch_get_resource((_r)->rid + 1))

#define for_each_capable_rdt_resource(r)				      \
	for_each_rdt_resource((r))					      \
		if ((r)->alloc_capable || (r)->mon_capable)

#define for_each_alloc_capable_rdt_resource(r)				      \
	for_each_rdt_resource((r))					      \
		if ((r)->alloc_capable)

#define for_each_mon_capable_rdt_resource(r)				      \
	for_each_rdt_resource((r))					      \
		if ((r)->mon_capable)

enum resctrl_res_level {
	RDT_RESOURCE_L3,
	RDT_RESOURCE_L2,
	RDT_RESOURCE_MBA,
	RDT_RESOURCE_SMBA,

	/* Must be the last */
	RDT_NUM_RESOURCES,
};

/**
 * enum resctrl_conf_type - The type of configuration.
 * @CDP_NONE:	No prioritisation, both code and data are controlled or monitored.
 * @CDP_CODE:	Configuration applies to instruction fetches.
 * @CDP_DATA:	Configuration applies to reads and writes.
 */
enum resctrl_conf_type {
	CDP_NONE,
	CDP_CODE,
	CDP_DATA,
};

#define CDP_NUM_TYPES	(CDP_DATA + 1)

/*
 * Event IDs are defined in resctrl_types.h.  Convert to bit positions used
 * by arch helpers (and by mon_evt event_id).
 */

/**
 * enum resctrl_scope - Scope of a resource domain
 *
 * @RESCTRL_L2_CACHE:	Domain scope is the L2 cache.
 * @RESCTRL_L3_CACHE:	Domain scope is the L3 cache.
 * @RESCTRL_L3_NODE:	Domain scope is L3-cache aligned NUMA node
 *			(Sub-NUMA-Cluster monitor scope).
 */
enum resctrl_scope {
	RESCTRL_L2_CACHE = 2,
	RESCTRL_L3_CACHE = 3,
	RESCTRL_L3_NODE,
};

/**
 * enum resctrl_schema_fmt - Format of a "schemata" file value entry
 * @RESCTRL_SCHEMA_BITMAP: A bitmap of the cache portions.
 * @RESCTRL_SCHEMA_RANGE:  A bandwidth percentage (or other numeric range).
 */
enum resctrl_schema_fmt {
	RESCTRL_SCHEMA_BITMAP,
	RESCTRL_SCHEMA_RANGE,
};

/**
 * enum resctrl_domain_type - The type of a resctrl domain
 * @RESCTRL_CTRL_DOMAIN: Control domain.
 * @RESCTRL_MON_DOMAIN:  Monitor domain.
 */
enum resctrl_domain_type {
	RESCTRL_CTRL_DOMAIN,
	RESCTRL_MON_DOMAIN,
};

/**
 * struct resctrl_staged_config - parsed configuration to be applied
 * @new_ctrl:		new ctrl value to be loaded
 * @have_new_ctrl:	whether the user provided new_ctrl is valid
 */
struct resctrl_staged_config {
	u32			new_ctrl;
	bool			have_new_ctrl;
};

/**
 * struct pseudo_lock_region - pseudo-lock region information
 * @s:			Resctrl schema for the resource to which this
 *			pseudo-locked region belongs
 * @d:			Control domain to which this pseudo-locked region
 *			belongs
 * @cbm:		bitmask of the pseudo-locked region
 * @lock_thread_wq:	waitqueue used to wait on the pseudo-locking thread
 *			completion
 * @thread_done:	variable used by waitqueue to test if pseudo-locking
 *			thread completed
 * @cpu:		core associated with the cache on which the setup code
 *			will be run
 * @line_size:		size of the cache lines
 * @size:		size of pseudo-locked region in bytes
 * @kmem:		the kernel memory associated with pseudo-locked region
 * @minor:		minor number of character device associated with this
 *			region
 * @debugfs_dir:	pointer to this region's directory in the debugfs
 *			filesystem
 * @pm_reqs:		Power management QoS requests related to this region
 */
struct pseudo_lock_region {
	struct resctrl_schema		*s;
	struct rdt_ctrl_domain		*d;
	u32				cbm;
	wait_queue_head_t		lock_thread_wq;
	int				thread_done;
	int				cpu;
	unsigned int			line_size;
	unsigned int			size;
	void				*kmem;
	unsigned int			minor;
	struct dentry			*debugfs_dir;
	struct list_head		pm_reqs;
};

/**
 * struct rdt_domain_hdr - common header for a resctrl ctrl or mon domain
 * @list:	all instances of this resource (member of either
 *		rdt_resource->ctrl_domains or rdt_resource->mon_domains)
 * @id:		unique id for this instance (cache id, etc.)
 * @type:	control or monitor domain
 * @cpu_mask:	which CPUs share this resource
 */
struct rdt_domain_hdr {
	struct list_head		list;
	int				id;
	enum resctrl_domain_type	type;
	struct cpumask			cpu_mask;
};

/**
 * struct rdt_ctrl_domain - group of CPUs sharing a resctrl control resource
 * @hdr:		common fields shared with mon domains
 * @plr:		pseudo-locked region (if any) associated with domain
 * @staged_config:	parsed configuration to be applied
 * @mbps_val:		When mba_sc is enabled, this holds the array of user
 *			specified control values for mba_sc in MBps, indexed
 *			by closid
 */
struct rdt_ctrl_domain {
	struct rdt_domain_hdr		hdr;
	struct pseudo_lock_region	*plr;
	struct resctrl_staged_config	staged_config[CDP_NUM_TYPES];
	u32				*mbps_val;
};

/**
 * struct rdt_mon_domain - group of CPUs sharing a resctrl monitor resource
 * @hdr:		common fields shared with ctrl domains
 * @ci:			cacheinfo for the cache that defines this monitor
 *			domain
 * @rmid_busy_llc:	bitmap of which limbo RMIDs are above threshold
 * @mbm_total:		saved state for MBM total bandwidth
 * @mbm_local:		saved state for MBM local bandwidth
 * @mbm_over:		worker to periodically read MBM h/w counters
 * @cqm_limbo:		worker to periodically read CQM h/w counters
 * @mbm_work_cpu:	worker CPU for MBM h/w counters
 * @cqm_work_cpu:	worker CPU for CQM h/w counters
 */
struct rdt_mon_domain {
	struct rdt_domain_hdr		hdr;
	struct cacheinfo		*ci;
	unsigned long			*rmid_busy_llc;
	struct mbm_state		*mbm_total;
	struct mbm_state		*mbm_local;
	struct delayed_work		mbm_over;
	struct delayed_work		cqm_limbo;
	int				mbm_work_cpu;
	int				cqm_work_cpu;
};

/*
 * Backwards-compatible alias used by drivers that haven't been ported to the
 * split control/monitor domain layout.  New code should use rdt_ctrl_domain
 * or rdt_mon_domain explicitly.
 */
#define rdt_domain rdt_ctrl_domain

/* Forward declaration; defined in fs/resctrl/internal.h. */
struct mbm_state;

/**
 * struct resctrl_cache - Cache allocation related data
 * @cbm_len:		Length of the cache bit mask
 * @min_cbm_bits:	Minimum number of consecutive bits to be set.
 *			The value 0 means the architecture can support
 *			zero CBM.
 * @shareable_bits:	Bitmask of shareable resource with other
 *			executing entities
 * @arch_has_sparse_bitmasks:	True if a bitmap like f00f is valid.
 * @arch_has_per_cpu_cfg:	True if QOS_CFG register for this cache
 *				level has CPU scope.
 */
struct resctrl_cache {
	unsigned int	cbm_len;
	unsigned int	min_cbm_bits;
	unsigned int	shareable_bits;
	bool		arch_has_sparse_bitmasks;
	bool		arch_has_per_cpu_cfg;
};

/**
 * enum membw_throttle_mode - System's memory bandwidth throttling mode
 * @THREAD_THROTTLE_UNDEFINED:	Not relevant to the system
 * @THREAD_THROTTLE_MAX:	Memory bandwidth is throttled at the core
 *				always using smallest bandwidth percentage
 *				assigned to threads, aka "max throttling"
 * @THREAD_THROTTLE_PER_THREAD:	Memory bandwidth is throttled at the thread
 */
enum membw_throttle_mode {
	THREAD_THROTTLE_UNDEFINED = 0,
	THREAD_THROTTLE_MAX,
	THREAD_THROTTLE_PER_THREAD,
};

/**
 * struct resctrl_membw - Memory bandwidth allocation related data
 * @min_bw:		Minimum memory bandwidth percentage user can request
 * @max_bw:		Maximum memory bandwidth percentage user can request
 * @bw_gran:		Granularity at which the memory bandwidth is allocated
 * @delay_linear:	True if memory B/W delay is in linear scale
 * @arch_needs_linear:	True if we can't configure non-linear resources
 * @throttle_mode:	Bandwidth throttling mode when threads request
 *			different memory bandwidths
 * @mba_sc:		True if MBA software controller(mba_sc) is enabled
 * @default_to_min:	Set when default value should clamp to min_bw rather
 *			than max_bw.
 * @mb_map:		Mapping of memory B/W percentage to memory B/W delay
 */
struct resctrl_membw {
	u32				min_bw;
	u32				max_bw;
	u32				bw_gran;
	u32				delay_linear;
	bool				arch_needs_linear;
	enum membw_throttle_mode	throttle_mode;
	bool				mba_sc;
	bool				default_to_min;
	u32				*mb_map;
};

struct rdt_parse_data;
struct resctrl_schema;

/**
 * struct resctrl_cpu_defaults - per-CPU CLOSID/RMID defaults
 * @closid: default CLOSID to apply on this CPU.
 * @rmid:   default RMID to apply on this CPU.
 *
 * Passed to resctrl_arch_sync_cpu_closid_rmid() to switch the running CPU's
 * resctrl context.
 */
struct resctrl_cpu_defaults {
	u32	closid;
	u32	rmid;
};

/**
 * struct rdt_resource - attributes of a resctrl resource
 * @rid:		The index of the resource
 * @alloc_capable:	Is allocation available on this machine
 * @mon_capable:	Is monitor feature available on this machine
 * @num_rmid:		Number of RMIDs available
 * @schema_fmt:		The format of the "schemata" file value entries.
 * @ctrl_scope:		Scope of the control domains for this resource.
 * @mon_scope:		Scope of the monitor domains for this resource.
 * @cache:		Cache allocation related data
 * @membw:		If the component has bandwidth controls, their properties.
 * @ctrl_domains:	All control domains for this resource
 * @mon_domains:	All monitor domains for this resource
 * @name:		Name to use in "schemata" file.
 * @default_ctrl:	Specifies default cache cbm or memory B/W percent.
 * @evt_list:		List of monitoring events
 * @cdp_capable:	Is the CDP feature available on this resource
 */
struct rdt_resource {
	int				rid;
	bool				alloc_capable;
	bool				mon_capable;
	int				num_rmid;
	enum resctrl_schema_fmt		schema_fmt;
	enum resctrl_scope		ctrl_scope;
	enum resctrl_scope		mon_scope;
	struct resctrl_cache		cache;
	struct resctrl_membw		membw;
	struct list_head		ctrl_domains;
	struct list_head		mon_domains;
	char				*name;
	u32				default_ctrl;
	struct list_head		evt_list;
	bool				cdp_capable;
	u32				mbm_cfg_mask;
};

/**
 * struct resctrl_schema - configuration abilities of a resource presented to
 *			   user-space
 * @list:	Member of resctrl_schema_all.
 * @name:	The name to use in the "schemata" file.
 * @fmt_str:	The format used to print each entry of the "schemata" file.
 * @conf_type:	Whether this schema is specific to code/data.
 * @res:	The resource structure exported by the architecture to describe
 *		the hardware that is configured by this schema.
 * @num_closid:	The number of closid that can be used with this schema. When
 *		features like CDP are enabled, this will be lower than the
 *		hardware supports for the resource.
 */
struct resctrl_schema {
	struct list_head		list;
	char				name[8];
	const char			*fmt_str;
	enum resctrl_conf_type		conf_type;
	struct rdt_resource		*res;
	u32				num_closid;
};

/**
 * struct resctrl_mon_config_info - Information to read/write MBM event config
 * @r:		The resource being configured.
 * @d:		The domain being configured.
 * @evtid:	The event id being configured.
 * @mon_config:	The current/new configuration value for the event.
 */
struct resctrl_mon_config_info {
	struct rdt_resource	*r;
	struct rdt_mon_domain	*d;
	u32			evtid;
	u32			mon_config;
};

/**
 * resctrl_get_default_ctrl() - Get the resource's default control value.
 * @r: The resource.
 */
static inline u32 resctrl_get_default_ctrl(struct rdt_resource *r)
{
	return r->default_ctrl;
}

/**
 * get_cpu_cacheinfo_level() - Return cacheinfo for the cache at @scope on @cpu.
 * @cpu:	CPU whose caches to inspect.
 * @scope:	Cache level (RESCTRL_L2_CACHE or RESCTRL_L3_CACHE) to find.
 */
static inline struct cacheinfo *get_cpu_cacheinfo_level(int cpu,
							enum resctrl_scope scope)
{
	struct cpu_cacheinfo *ci = get_cpu_cacheinfo(cpu);
	int i;

	if (!ci || !ci->info_list)
		return NULL;

	for (i = 0; i < ci->num_leaves; i++) {
		if (ci->info_list[i].level == (unsigned int)scope)
			return &ci->info_list[i];
	}

	return NULL;
}

/* The number of closid supported by this resource regardless of CDP */
u32 resctrl_arch_get_num_closid(struct rdt_resource *r);

/* Total number of (CLOSID, RMID) index slots. */
u32 resctrl_arch_system_num_rmid_idx(void);

int resctrl_arch_update_domains(struct rdt_resource *r, u32 closid);

struct rdt_domain_hdr *resctrl_find_domain(struct list_head *h, int id,
					   struct list_head **pos);

bool resctrl_arch_is_evt_configurable(enum resctrl_event_id evt);

/**
 * resctrl_arch_get_resource() - Get an arch-private rdt_resource for a level.
 * @l:	Resource level.
 *
 * Returns the rdt_resource describing the hardware for @l, or a dummy resource
 * for which alloc_capable and mon_capable are both false.  Never returns NULL
 * for a valid @l so callers can dereference the returned pointer.
 */
struct rdt_resource *resctrl_arch_get_resource(enum resctrl_res_level l);

bool resctrl_arch_alloc_capable(void);
bool resctrl_arch_mon_capable(void);

void resctrl_arch_enable_alloc(void);
void resctrl_arch_disable_alloc(void);
void resctrl_arch_enable_mon(void);
void resctrl_arch_disable_mon(void);

/* Monitor context allocation/free (used by MPAM-like backends; may stub). */
void *resctrl_arch_mon_ctx_alloc(struct rdt_resource *r, enum resctrl_event_id evtid);
void resctrl_arch_mon_ctx_free(struct rdt_resource *r, enum resctrl_event_id evtid, void *ctx);

/* Reset all control values for the given resource to defaults. */
void resctrl_arch_reset_all_ctrls(struct rdt_resource *r);

/*
 * Whether specific monitoring features are enabled.  Architectures provide
 * these (usually as inline stubs returning false) via <asm/resctrl.h>.
 */

/**
 * resctrl_arch_mon_event_config_write() - Write the config for an event.
 * @config_info: struct resctrl_mon_config_info describing the resource, domain
 *		 and event.
 */
void resctrl_arch_mon_event_config_write(void *config_info);

/**
 * resctrl_arch_mon_event_config_read() - Read the config for an event.
 * @config_info: struct resctrl_mon_config_info describing the resource, domain
 *		 and event.
 */
void resctrl_arch_mon_event_config_read(void *config_info);

/* For use by arch code to remap resctrl's smaller CDP CLOSID range */
static inline u32 resctrl_get_config_index(u32 closid,
					   enum resctrl_conf_type type)
{
	switch (type) {
	default:
	case CDP_NONE:
		return closid;
	case CDP_CODE:
		return closid * 2 + 1;
	case CDP_DATA:
		return closid * 2;
	}
}

bool resctrl_arch_get_cdp_enabled(enum resctrl_res_level l);
int resctrl_arch_set_cdp_enabled(enum resctrl_res_level l, bool enable);

/*
 * Update the ctrl_val and apply this config right now.
 * Must be called on one of the domain's CPUs.
 */
int resctrl_arch_update_one(struct rdt_resource *r, struct rdt_ctrl_domain *d,
			    u32 closid, enum resctrl_conf_type t, u32 cfg_val);

u32 resctrl_arch_get_config(struct rdt_resource *r, struct rdt_ctrl_domain *d,
			    u32 closid, enum resctrl_conf_type type);

#ifdef CONFIG_RESCTRL_FS
int resctrl_online_ctrl_domain(struct rdt_resource *r, struct rdt_ctrl_domain *d);
int resctrl_online_mon_domain(struct rdt_resource *r, struct rdt_mon_domain *d);
void resctrl_offline_ctrl_domain(struct rdt_resource *r, struct rdt_ctrl_domain *d);
void resctrl_offline_mon_domain(struct rdt_resource *r, struct rdt_mon_domain *d);
void resctrl_online_cpu(unsigned int cpu);
void resctrl_offline_cpu(unsigned int cpu);
int resctrl_init(void);
void resctrl_exit(void);
#else
static inline int resctrl_online_ctrl_domain(struct rdt_resource *r,
					     struct rdt_ctrl_domain *d) { return 0; }
static inline int resctrl_online_mon_domain(struct rdt_resource *r,
					    struct rdt_mon_domain *d) { return 0; }
static inline void resctrl_offline_ctrl_domain(struct rdt_resource *r,
					       struct rdt_ctrl_domain *d) { }
static inline void resctrl_offline_mon_domain(struct rdt_resource *r,
					      struct rdt_mon_domain *d) { }
static inline void resctrl_online_cpu(unsigned int cpu) { }
static inline void resctrl_offline_cpu(unsigned int cpu) { }
static inline int resctrl_init(void) { return 0; }
static inline void resctrl_exit(void) { }
#endif

/**
 * resctrl_arch_rmid_read() - Read the eventid counter corresponding to
 *			      (closid, rmid) for this resource and domain.
 * @r:			resource that the counter should be read from.
 * @d:			domain that the counter should be read from.
 * @closid:		closid of the counter to read.
 * @rmid:		rmid of the counter to read.
 * @eventid:		eventid to read, e.g. L3 occupancy.
 * @val:		result of the counter read in bytes.
 * @arch_mon_ctx:	architecture-private context returned by
 *			resctrl_arch_mon_ctx_alloc().
 *
 * Call from process context on a CPU that belongs to domain @d.
 *
 * Return:
 * 0 on success, or -EIO, -EINVAL etc on error.
 */
int resctrl_arch_rmid_read(struct rdt_resource *r, struct rdt_mon_domain *d,
			   u32 closid, u32 rmid,
			   enum resctrl_event_id eventid, u64 *val,
			   void *arch_mon_ctx);

/**
 * resctrl_arch_reset_rmid() - Reset any private state associated with rmid
 *			       and eventid.
 */
void resctrl_arch_reset_rmid(struct rdt_resource *r, struct rdt_mon_domain *d,
			     u32 closid, u32 rmid, enum resctrl_event_id eventid);

/**
 * resctrl_arch_reset_rmid_all() - Reset all private state associated with
 *				   all rmids and eventids.
 */
void resctrl_arch_reset_rmid_all(struct rdt_resource *r, struct rdt_mon_domain *d);

extern unsigned int resctrl_rmid_realloc_threshold;
extern unsigned int resctrl_rmid_realloc_limit;

int rdtgroup_init(void);
void rdtgroup_exit(void);

u64 resctrl_arch_get_prefetch_disable_bits(void);
int resctrl_arch_pseudo_lock_fn(void *_plr);
int resctrl_arch_measure_cycles_lat_fn(void *_plr);
int resctrl_arch_measure_l2_residency(void *_plr);
int resctrl_arch_measure_l3_residency(void *_plr);

#endif /* _RESCTRL_H */
