// SPDX-License-Identifier: GPL-2.0

#define XUANTIE_LINK_PMU_PDEV_NAME "xuantie_link_pmu"
#define pr_fmt(fmt) XUANTIE_LINK_PMU_PDEV_NAME ": " fmt

#include <linux/bitmap.h>
#include <linux/cpu_pm.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#define XUANTIE_LINK_DEV_TYPE_XL100			100
#define XUANTIE_LINK_DEV_TYPE_XL200			200
#define XUANTIE_LINK_DEV_TYPE_XL300			300
#define XUANTIE_LINK_PMU_VENDOR_ID			0x5b7

#define XUANTIE_LINK_PMU_NUM_COUNTERS		6
#define XUANTIE_LINK_PMU_COUNTER_MASK		GENMASK_ULL(63, 0)

#define HPCP_EVENT_OF BIT_ULL(63)

/* HPCP reg */
#define XUANTIE_LINK_PMU_L3IMP				0x048
#define XUANTIE_LINK_PMU_HPCPCOUNT0			0x500
#define XUANTIE_LINK_PMU_HPCPCOUNT1			0x508
#define XUANTIE_LINK_PMU_HPCPCOUNT2			0x510
#define XUANTIE_LINK_PMU_HPCPCOUNT3			0x518
#define XUANTIE_LINK_PMU_HPCPCOUNT4			0x520
#define XUANTIE_LINK_PMU_HPCPCOUNT5			0x528
#define XUANTIE_LINK_PMU_HPCPEVT0CR			0x540
#define XUANTIE_LINK_PMU_HPCPEVT1CR			0x548
#define XUANTIE_LINK_PMU_HPCPEVT2CR			0x550
#define XUANTIE_LINK_PMU_HPCPEVT3CR			0x558
#define XUANTIE_LINK_PMU_HPCPEVT4CR			0x560
#define XUANTIE_LINK_PMU_HPCPEVT5CR			0x568
#define XUANTIE_LINK_PMU_HPCPCNTOF			0x5c0
#define XUANTIE_LINK_PMU_HPCPMAUTHCR		0x5c8
#define XUANTIE_LINK_PMU_HPCPHAUTHCR		0x5d0 // XL300
#define XUANTIE_LINK_PMU_HPCPSAUTHCR		0x5d8
#define XUANTIE_LINK_PMU_HPCPINHIBIT		0x5e0
#define XUANTIE_LINK_PMU_HPCPINTPEND		0x5e8
#define XUANTIE_LINK_PMU_VENDOR_N_IMP_ID	0x600 // XL300

/* HPCP event */
#define NO_EVENT		0x000
#define ALL_L3_DA		0x001
#define ALL_L3_DM		0x002
#define ALL_L3_IA		0x003
#define ALL_L3_IM		0x004
#define ALL_L3_RVLD		0x005
#define ALL_L3_RSTALL	0x006
#define ALL_L3_WVLD		0x007 // XL300
#define ALL_L3_WSTALL	0x008 // XL300
#define C0_L3_DA		0x041
#define C0_L3_DM		0x042
#define C0_L3_IA		0x043
#define C0_L3_IM		0x044
#define C0_L3_RVLD		0x045
#define C0_L3_RSTALL	0x046
#define C0_L3_WVLD		0x047 // XL300
#define C0_L3_WSTALL	0x048 // XL300
/* CN=0..7 */
#define L3_DA(CN)		(C0_L3_DA + 0x040 * CN)
#define L3_DM(CN)		(C0_L3_DM + 0x040 * CN)
#define L3_IA(CN)		(C0_L3_IA + 0x040 * CN)
#define L3_IM(CN)		(C0_L3_IM + 0x040 * CN)
#define L3_RVLD(CN)		(C0_L3_RVLD + 0x040 * CN)
#define L3_RSTALL(CN)	(C0_L3_RSTALL + 0x040 * CN)
#define L3_WVLD(CN)		(C0_L3_WVLD + 0x040 * CN)
#define L3_WSTALL(CN)	(C0_L3_WSTALL + 0x040 * CN)

#define to_xuantie_link_pmu(p) (container_of(p, struct xuantie_link_pmu, pmu))

#define XUANTIE_LINK_FORMAT_ATTR(_name, _config)                          \
	(&((struct dev_ext_attribute[]){ {                                \
		.attr = __ATTR(_name, 0444,                               \
			       xuantie_link_pmu_sysfs_format_show, NULL), \
		.var = (void *)_config,                                   \
	} })[0].attr.attr)

#define XUANTIE_LINK_EVENT_ATTR(_name, _id) \
	PMU_EVENT_ATTR_ID(_name, xuantie_link_pmu_sysfs_event_show, _id)

static u32 xuantie_link_pmu_cpuhp_state;
static bool debug_enable;

struct xuantie_link_devtype_data {
	u32 quirks;
};

struct xuantie_link_hw_events {
	struct perf_event *events[XUANTIE_LINK_PMU_NUM_COUNTERS];
	DECLARE_BITMAP(used_mask, XUANTIE_LINK_PMU_NUM_COUNTERS);
};

struct xuantie_link_pmu {
	struct pmu pmu;
	struct xuantie_link_hw_events __percpu *hw_events;
	struct hlist_node node;
	struct notifier_block pm_nb;
	void __iomem *pmu_base;
	cpumask_t cpumask;
	u64 identifier;
	const struct xuantie_link_devtype_data *devtype_data;
	u32 irq;
};

static ssize_t debug_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", debug_enable);
}
static ssize_t debug_enable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int ret = kstrtobool(buf, &debug_enable);

	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_RW(debug_enable);
static struct attribute *xuantie_link_pmu_debug_attrs[] = {
	&dev_attr_debug_enable.attr,
	NULL
};
static const struct attribute_group xuantie_link_pmu_debug_attr_group = {
	.attrs = xuantie_link_pmu_debug_attrs,
};

static ssize_t xuantie_link_pmu_sysfs_format_show(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	struct dev_ext_attribute *eattr =
		container_of(attr, struct dev_ext_attribute, attr);

	return sysfs_emit(buf, "%s\n", (char *)eattr->var);
}
static struct attribute *xuantie_link_pmu_format_attrs[] = {
	XUANTIE_LINK_FORMAT_ATTR(event, "config:0-31"),
	NULL
};
static const struct attribute_group xuantie_link_pmu_format_attr_group = {
	.name = "format",
	.attrs = xuantie_link_pmu_format_attrs,
};

static ssize_t xuantie_link_pmu_sysfs_event_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct perf_pmu_events_attr *eattr =
		container_of(attr, struct perf_pmu_events_attr, attr);

	return sysfs_emit(buf, "event=0x%02llx\n", eattr->id);
}

static struct attribute *xuantie_link_pmu_event_attrs[] = {
	XUANTIE_LINK_EVENT_ATTR(all_l3_da, ALL_L3_DA),
	XUANTIE_LINK_EVENT_ATTR(all_l3_dm, ALL_L3_DM),
	XUANTIE_LINK_EVENT_ATTR(all_l3_ia, ALL_L3_IA),
	XUANTIE_LINK_EVENT_ATTR(all_l3_im, ALL_L3_IM),
	XUANTIE_LINK_EVENT_ATTR(all_l3_rvld, ALL_L3_RVLD),
	XUANTIE_LINK_EVENT_ATTR(all_l3_rstall, ALL_L3_RSTALL),
	XUANTIE_LINK_EVENT_ATTR(all_l3_wvld, ALL_L3_WVLD),
	XUANTIE_LINK_EVENT_ATTR(all_l3_wstall, ALL_L3_WSTALL),

	XUANTIE_LINK_EVENT_ATTR(c0_l3_da, L3_DA(0)),
	XUANTIE_LINK_EVENT_ATTR(c0_l3_dm, L3_DM(0)),
	XUANTIE_LINK_EVENT_ATTR(c0_l3_ia, L3_IA(0)),
	XUANTIE_LINK_EVENT_ATTR(c0_l3_im, L3_IM(0)),
	XUANTIE_LINK_EVENT_ATTR(c0_l3_rvld, L3_RVLD(0)),
	XUANTIE_LINK_EVENT_ATTR(c0_l3_rstall, L3_RSTALL(0)),
	XUANTIE_LINK_EVENT_ATTR(c0_l3_wvld, L3_WVLD(0)),
	XUANTIE_LINK_EVENT_ATTR(c0_l3_wstall, L3_WSTALL(0)),

	XUANTIE_LINK_EVENT_ATTR(c1_l3_da, L3_DA(1)),
	XUANTIE_LINK_EVENT_ATTR(c1_l3_dm, L3_DM(1)),
	XUANTIE_LINK_EVENT_ATTR(c1_l3_ia, L3_IA(1)),
	XUANTIE_LINK_EVENT_ATTR(c1_l3_im, L3_IM(1)),
	XUANTIE_LINK_EVENT_ATTR(c1_l3_rvld, L3_RVLD(1)),
	XUANTIE_LINK_EVENT_ATTR(c1_l3_rstall, L3_RSTALL(1)),
	XUANTIE_LINK_EVENT_ATTR(c1_l3_wvld, L3_WVLD(1)),
	XUANTIE_LINK_EVENT_ATTR(c1_l3_wstall, L3_WSTALL(1)),

	XUANTIE_LINK_EVENT_ATTR(c2_l3_da, L3_DA(2)),
	XUANTIE_LINK_EVENT_ATTR(c2_l3_dm, L3_DM(2)),
	XUANTIE_LINK_EVENT_ATTR(c2_l3_ia, L3_IA(2)),
	XUANTIE_LINK_EVENT_ATTR(c2_l3_im, L3_IM(2)),
	XUANTIE_LINK_EVENT_ATTR(c2_l3_rvld, L3_RVLD(2)),
	XUANTIE_LINK_EVENT_ATTR(c2_l3_rstall, L3_RSTALL(2)),
	XUANTIE_LINK_EVENT_ATTR(c2_l3_wvld, L3_WVLD(2)),
	XUANTIE_LINK_EVENT_ATTR(c2_l3_wstall, L3_WSTALL(2)),

	XUANTIE_LINK_EVENT_ATTR(c3_l3_da, L3_DA(3)),
	XUANTIE_LINK_EVENT_ATTR(c3_l3_dm, L3_DM(3)),
	XUANTIE_LINK_EVENT_ATTR(c3_l3_ia, L3_IA(3)),
	XUANTIE_LINK_EVENT_ATTR(c3_l3_im, L3_IM(3)),
	XUANTIE_LINK_EVENT_ATTR(c3_l3_rvld, L3_RVLD(3)),
	XUANTIE_LINK_EVENT_ATTR(c3_l3_rstall, L3_RSTALL(3)),
	XUANTIE_LINK_EVENT_ATTR(c3_l3_wvld, L3_WVLD(3)),
	XUANTIE_LINK_EVENT_ATTR(c3_l3_wstall, L3_WSTALL(3)),

	XUANTIE_LINK_EVENT_ATTR(c4_l3_da, L3_DA(4)),
	XUANTIE_LINK_EVENT_ATTR(c4_l3_dm, L3_DM(4)),
	XUANTIE_LINK_EVENT_ATTR(c4_l3_ia, L3_IA(4)),
	XUANTIE_LINK_EVENT_ATTR(c4_l3_im, L3_IM(4)),
	XUANTIE_LINK_EVENT_ATTR(c4_l3_rvld, L3_RVLD(4)),
	XUANTIE_LINK_EVENT_ATTR(c4_l3_rstall, L3_RSTALL(4)),
	XUANTIE_LINK_EVENT_ATTR(c4_l3_wvld, L3_WVLD(4)),
	XUANTIE_LINK_EVENT_ATTR(c4_l3_wstall, L3_WSTALL(4)),

	XUANTIE_LINK_EVENT_ATTR(c5_l3_da, L3_DA(5)),
	XUANTIE_LINK_EVENT_ATTR(c5_l3_dm, L3_DM(5)),
	XUANTIE_LINK_EVENT_ATTR(c5_l3_ia, L3_IA(5)),
	XUANTIE_LINK_EVENT_ATTR(c5_l3_im, L3_IM(5)),
	XUANTIE_LINK_EVENT_ATTR(c5_l3_rvld, L3_RVLD(5)),
	XUANTIE_LINK_EVENT_ATTR(c5_l3_rstall, L3_RSTALL(5)),
	XUANTIE_LINK_EVENT_ATTR(c5_l3_wvld, L3_WVLD(5)),
	XUANTIE_LINK_EVENT_ATTR(c5_l3_wstall, L3_WSTALL(5)),

	XUANTIE_LINK_EVENT_ATTR(c6_l3_da, L3_DA(6)),
	XUANTIE_LINK_EVENT_ATTR(c6_l3_dm, L3_DM(6)),
	XUANTIE_LINK_EVENT_ATTR(c6_l3_ia, L3_IA(6)),
	XUANTIE_LINK_EVENT_ATTR(c6_l3_im, L3_IM(6)),
	XUANTIE_LINK_EVENT_ATTR(c6_l3_rvld, L3_RVLD(6)),
	XUANTIE_LINK_EVENT_ATTR(c6_l3_rstall, L3_RSTALL(6)),
	XUANTIE_LINK_EVENT_ATTR(c6_l3_wvld, L3_WVLD(6)),
	XUANTIE_LINK_EVENT_ATTR(c6_l3_wstall, L3_WSTALL(6)),

	XUANTIE_LINK_EVENT_ATTR(c7_l3_da, L3_DA(7)),
	XUANTIE_LINK_EVENT_ATTR(c7_l3_dm, L3_DM(7)),
	XUANTIE_LINK_EVENT_ATTR(c7_l3_ia, L3_IA(7)),
	XUANTIE_LINK_EVENT_ATTR(c7_l3_im, L3_IM(7)),
	XUANTIE_LINK_EVENT_ATTR(c7_l3_rvld, L3_RVLD(7)),
	XUANTIE_LINK_EVENT_ATTR(c7_l3_rstall, L3_RSTALL(7)),
	XUANTIE_LINK_EVENT_ATTR(c7_l3_wvld, L3_WVLD(7)),
	XUANTIE_LINK_EVENT_ATTR(c7_l3_wstall, L3_WSTALL(7)),

	NULL
};

static const struct attribute_group xuantie_link_pmu_events_attr_group = {
	.name = "events",
	.attrs = xuantie_link_pmu_event_attrs,
};

static ssize_t cpumask_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		to_xuantie_link_pmu(dev_get_drvdata(dev));

	return cpumap_print_to_pagebuf(true, buf, &xuantie_link_pmu->cpumask);
}
static DEVICE_ATTR_RO(cpumask);
static struct attribute *xuantie_link_pmu_cpumask_attrs[] = {
	&dev_attr_cpumask.attr,
	NULL
};
static const struct attribute_group xuantie_link_pmu_cpumask_attr_group = {
	.attrs = xuantie_link_pmu_cpumask_attrs,
};

static ssize_t identifier_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		to_xuantie_link_pmu(dev_get_drvdata(dev));

	return sysfs_emit(buf, "VENDOR_ID=0x%x\nIMP_ID=0x%x\n",
			  lower_32_bits(xuantie_link_pmu->identifier),
			  upper_32_bits(xuantie_link_pmu->identifier));
}
static DEVICE_ATTR_RO(identifier);
static struct attribute *xuantie_link_pmu_identifier_attrs[] = {
	&dev_attr_identifier.attr,
	NULL
};
static const struct attribute_group xuantie_link_pmu_identifier_attr_group = {
	.attrs = xuantie_link_pmu_identifier_attrs,
};

static const struct attribute_group *xuantie_link_pmu_attr_groups[] = {
	&xuantie_link_pmu_format_attr_group,
	&xuantie_link_pmu_events_attr_group,
	&xuantie_link_pmu_cpumask_attr_group,
	&xuantie_link_pmu_identifier_attr_group,
	&xuantie_link_pmu_debug_attr_group,
	NULL
};

static void
xuantie_link_pmu_set_event_period(struct perf_event *event,
				  struct xuantie_link_pmu *xuantie_link_pmu)
{
	struct hw_perf_event *hwc = &event->hw;
	int idx = event->hw.idx;
	u64 val;

	/*
	 * Program counter to half of it's max count to handle
	 * cases of extreme interrupt latency.
	 */
	val = XUANTIE_LINK_PMU_COUNTER_MASK >> 1;
	local64_set(&hwc->prev_count, val);
	writeq(val, xuantie_link_pmu->pmu_base + XUANTIE_LINK_PMU_HPCPCOUNT0 +
			    idx * 8);
	if (debug_enable)
		pr_info("%s: writeq idx=%d reg=%x val=%llx\n", __func__, idx,
			XUANTIE_LINK_PMU_HPCPCOUNT0 + idx * 8, val);
}

static void
xuantie_link_pmu_counter_start(struct perf_event *event,
			       struct xuantie_link_pmu *xuantie_link_pmu)
{
	int idx = event->hw.idx;
	int event_id = event->hw.config;
	u64 val = (~HPCP_EVENT_OF) & event_id;

	writeq(val, xuantie_link_pmu->pmu_base + XUANTIE_LINK_PMU_HPCPEVT0CR +
			    idx * 8);
	if (debug_enable)
		pr_info("%s: writeq idx=%d event_id=%x reg=%x val=%llx\n",
			__func__, idx, event_id,
			XUANTIE_LINK_PMU_HPCPEVT0CR + idx * 8, val);
}

static void
xuantie_link_pmu_counter_stop(struct perf_event *event,
			      struct xuantie_link_pmu *xuantie_link_pmu)
{
	int idx = event->hw.idx;
	u64 val = HPCP_EVENT_OF + NO_EVENT;

	writeq(val, xuantie_link_pmu->pmu_base + XUANTIE_LINK_PMU_HPCPEVT0CR +
			    idx * 8);
	if (debug_enable)
		pr_info("%s: writeq idx=%d reg=%x val=%llx\n", __func__, idx,
			XUANTIE_LINK_PMU_HPCPEVT0CR + idx * 8, val);
}

static int xuantie_link_pmu_event_init(struct perf_event *event)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		to_xuantie_link_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;

	/*
	 * Sampling is not supported, as counters are shared
	 * by all CPU.
	 */
	if (hwc->sample_period) {
		pr_err("Sampling not supported\n");
		return -EOPNOTSUPP;
	}

	/*
	 * Per-task and attach to a task are not supported,
	 * as uncore events are not specific to any CPU.
	 */
	if (event->cpu < 0 || event->attach_state & PERF_ATTACH_TASK) {
		pr_err("Per-task mode not supported\n");
		return -EOPNOTSUPP;
	}

	hwc->idx = -1;
	hwc->config = event->attr.config;
	event->cpu = cpumask_first(&xuantie_link_pmu->cpumask);

	return 0;
}

static void xuantie_link_pmu_update(struct perf_event *event)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		to_xuantie_link_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;
	u64 prev_raw_count, new_raw_count;
	u64 oldval;
	u64 delta;

	do {
		prev_raw_count = local64_read(&hwc->prev_count);
		new_raw_count = readq(xuantie_link_pmu->pmu_base +
				      XUANTIE_LINK_PMU_HPCPCOUNT0 + idx * 8);
		if (debug_enable)
			pr_info("%s: readq idx=%d reg=%x prev_raw_count=%llx new_raw_count=%llx\n",
				__func__, idx,
				XUANTIE_LINK_PMU_HPCPCOUNT0 + idx * 8,
				prev_raw_count, new_raw_count);
		oldval = local64_cmpxchg(&hwc->prev_count, prev_raw_count, new_raw_count);
	} while (oldval != prev_raw_count);

	delta = new_raw_count - prev_raw_count;
	local64_add(delta, &event->count);
	local64_sub(delta, &hwc->period_left);
	if (debug_enable)
		pr_info("%s: count=%ld count=%lx\n", __func__,
			local64_read(&event->count),
			local64_read(&event->count));
}

static void xuantie_link_pmu_start(struct perf_event *event, int flags)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		to_xuantie_link_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;

	if (WARN_ON_ONCE(!(hwc->state & PERF_HES_STOPPED)))
		return;

	if (flags & PERF_EF_RELOAD)
		WARN_ON_ONCE(!(event->hw.state & PERF_HES_UPTODATE));

	hwc->state = 0;

	xuantie_link_pmu_set_event_period(event, xuantie_link_pmu);
	xuantie_link_pmu_counter_start(event, xuantie_link_pmu);

	perf_event_update_userpage(event);
}

static void xuantie_link_pmu_stop(struct perf_event *event, int flags)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		to_xuantie_link_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;

	if (hwc->state & PERF_HES_STOPPED)
		return;

	xuantie_link_pmu_counter_stop(event, xuantie_link_pmu);
	xuantie_link_pmu_update(event);
	hwc->state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
}

static int xuantie_link_pmu_add(struct perf_event *event, int flags)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		to_xuantie_link_pmu(event->pmu);
	struct xuantie_link_hw_events *hw_events =
		this_cpu_ptr(xuantie_link_pmu->hw_events);
	struct hw_perf_event *hwc = &event->hw;
	unsigned long *used_mask = hw_events->used_mask;
	u32 n_events = XUANTIE_LINK_PMU_NUM_COUNTERS;
	int idx;

	idx = find_first_zero_bit(used_mask, n_events);
	/* All counter are in use */
	if (idx < 0)
		return idx;

	set_bit(idx, used_mask);

	if (debug_enable)
		pr_info("%s: idx=%d used_mask=%lx flags=%d\n", __func__, idx,
			*used_mask, flags);

	hwc->idx = idx;
	hw_events->events[idx] = event;
	hwc->state = PERF_HES_UPTODATE | PERF_HES_STOPPED;

	if (flags & PERF_EF_START)
		xuantie_link_pmu_start(event, PERF_EF_RELOAD);

	perf_event_update_userpage(event);

	return 0;
}

static void xuantie_link_pmu_del(struct perf_event *event, int flags)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		to_xuantie_link_pmu(event->pmu);
	struct xuantie_link_hw_events *hw_events =
		this_cpu_ptr(xuantie_link_pmu->hw_events);
	struct hw_perf_event *hwc = &event->hw;

	if (debug_enable)
		pr_info("%s: idx=%d flags=%d\n", __func__, hwc->idx, flags);

	xuantie_link_pmu_stop(event, PERF_EF_UPDATE);
	hw_events->events[hwc->idx] = NULL;
	clear_bit(hwc->idx, hw_events->used_mask);
	hwc->idx = -1;
	perf_event_update_userpage(event);
}

static irqreturn_t xuantie_link_pmu_handle_irq(int irq_num, void *data)
{
	struct xuantie_link_pmu *xuantie_link_pmu = data;
	struct xuantie_link_hw_events *hw_events =
		this_cpu_ptr(xuantie_link_pmu->hw_events);
	bool handled = false;
	int idx;
	u64 overflow_status =
		readq(xuantie_link_pmu->pmu_base + XUANTIE_LINK_PMU_HPCPCNTOF);
	u64 int_pend =
		readq(xuantie_link_pmu->pmu_base + XUANTIE_LINK_PMU_HPCPINTPEND);

	if (debug_enable)
		pr_info("%s:%d overflow_status=%llx int_pend=%llx\n",
			__func__, __LINE__, overflow_status, int_pend);

	for (idx = 0; idx < XUANTIE_LINK_PMU_NUM_COUNTERS; idx++) {
		struct perf_event *event = hw_events->events[idx];

		if (!event)
			continue;

		if (!(overflow_status & BIT_ULL(idx)))
			continue;

		xuantie_link_pmu_counter_stop(event, xuantie_link_pmu);
		xuantie_link_pmu_update(event);
		xuantie_link_pmu_set_event_period(event, xuantie_link_pmu);
		xuantie_link_pmu_counter_start(event, xuantie_link_pmu);

		handled = true;
	}

	if (handled)
		writeq(0, xuantie_link_pmu->pmu_base + XUANTIE_LINK_PMU_HPCPINTPEND);

	return IRQ_RETVAL(handled);
}

static int xuantie_link_pmu_pm_notify(struct notifier_block *b,
				      unsigned long cmd, void *v)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		container_of(b, struct xuantie_link_pmu, pm_nb);
	struct xuantie_link_hw_events *hw_events =
		this_cpu_ptr(xuantie_link_pmu->hw_events);
	int enabled = bitmap_weight(hw_events->used_mask,
				    XUANTIE_LINK_PMU_NUM_COUNTERS);
	struct perf_event *event;
	int idx;

	if (!enabled)
		return NOTIFY_OK;

	for (idx = 0; idx < XUANTIE_LINK_PMU_NUM_COUNTERS; idx++) {
		event = hw_events->events[idx];
		if (!event)
			continue;

		switch (cmd) {
		case CPU_PM_ENTER:
			/* Stop and update the counter */
			xuantie_link_pmu_stop(event, PERF_EF_UPDATE);
			break;
		case CPU_PM_EXIT:
		case CPU_PM_ENTER_FAILED:
			/* Restore and enable the counter */
			xuantie_link_pmu_start(event, PERF_EF_RELOAD);
			break;
		default:
			break;
		}
	}

	return NOTIFY_OK;
}

static int
xuantie_link_pmu_pm_register(struct xuantie_link_pmu *xuantie_link_pmu)
{
	if (!IS_ENABLED(CONFIG_CPU_PM))
		return 0;

	xuantie_link_pmu->pm_nb.notifier_call = xuantie_link_pmu_pm_notify;
	return cpu_pm_register_notifier(&xuantie_link_pmu->pm_nb);
}

static void
xuantie_link_pmu_pm_unregister(struct xuantie_link_pmu *xuantie_link_pmu)
{
	if (!IS_ENABLED(CONFIG_CPU_PM))
		return;

	cpu_pm_unregister_notifier(&xuantie_link_pmu->pm_nb);
}

static int xuantie_link_pmu_device_probe(struct platform_device *pdev)
{
	struct xuantie_link_pmu *xuantie_link_pmu;
	struct xuantie_link_hw_events *hw_events;
	struct resource *res;
	int cpuid, i, irq, ret;

	xuantie_link_pmu =
		devm_kzalloc(&pdev->dev, sizeof(*xuantie_link_pmu), GFP_KERNEL);
	if (!xuantie_link_pmu) {
		dev_err(&pdev->dev, "Failed to allocate device data\n");
		return -ENOMEM;
	}

	xuantie_link_pmu->pmu_base =
		devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(xuantie_link_pmu->pmu_base))
		return PTR_ERR(xuantie_link_pmu->pmu_base);

	xuantie_link_pmu->hw_events =
		alloc_percpu_gfp(struct xuantie_link_hw_events, GFP_KERNEL);
	if (!xuantie_link_pmu->hw_events) {
		dev_err(&pdev->dev, "Failed to allocate per-cpu PMU data\n");
		return -ENOMEM;
	}

	for_each_possible_cpu(cpuid) {
		hw_events = per_cpu_ptr(xuantie_link_pmu->hw_events, cpuid);
		for (i = 0; i < XUANTIE_LINK_PMU_NUM_COUNTERS; i++)
			hw_events->events[i] = NULL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, xuantie_link_pmu_handle_irq, 0,
			       XUANTIE_LINK_PMU_PDEV_NAME, xuantie_link_pmu);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ\n");
		return -EINVAL;
	}

	xuantie_link_pmu->irq = irq;

	xuantie_link_pmu->devtype_data = of_device_get_match_data(&pdev->dev);

	ret = cpuhp_state_add_instance(xuantie_link_pmu_cpuhp_state,
				       &xuantie_link_pmu->node);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register hotplug\n");
		return ret;
	}

	ret = xuantie_link_pmu_pm_register(xuantie_link_pmu);
	if (ret) {
		cpuhp_state_remove_instance(xuantie_link_pmu_cpuhp_state,
					    &xuantie_link_pmu->node);
		return ret;
	}

	if (xuantie_link_pmu->devtype_data->quirks == XUANTIE_LINK_DEV_TYPE_XL300)
		xuantie_link_pmu->identifier =
			readq(xuantie_link_pmu->pmu_base + XUANTIE_LINK_PMU_VENDOR_N_IMP_ID);
	else
		xuantie_link_pmu->identifier =
			readq(xuantie_link_pmu->pmu_base + XUANTIE_LINK_PMU_L3IMP) << 32 |
			XUANTIE_LINK_PMU_VENDOR_ID;

	xuantie_link_pmu->pmu = (struct pmu){
		.task_ctx_nr = perf_invalid_context,
		.event_init = xuantie_link_pmu_event_init,
		.read = xuantie_link_pmu_update,
		.start = xuantie_link_pmu_start,
		.stop = xuantie_link_pmu_stop,
		.add = xuantie_link_pmu_add,
		.del = xuantie_link_pmu_del,
		.attr_groups = xuantie_link_pmu_attr_groups,
		.capabilities = PERF_PMU_CAP_NO_EXCLUDE,
	};

	ret = perf_pmu_register(&xuantie_link_pmu->pmu,
				XUANTIE_LINK_PMU_PDEV_NAME, -1);
	if (ret) {
		xuantie_link_pmu_pm_unregister(xuantie_link_pmu);
		cpuhp_state_remove_instance(xuantie_link_pmu_cpuhp_state,
					    &xuantie_link_pmu->node);
	}

	debug_enable = false;
	pr_info("pmu_base=%pK irq=%d identifier=%llx devtype_data.quirks=%d\n",
		xuantie_link_pmu->pmu_base, xuantie_link_pmu->irq,
		xuantie_link_pmu->identifier,
		xuantie_link_pmu->devtype_data->quirks);

	return ret;
}

static int xuantie_link_pmu_device_remove(struct platform_device *pdev)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		to_xuantie_link_pmu(platform_get_drvdata(pdev));

	perf_pmu_unregister(&xuantie_link_pmu->pmu);
	xuantie_link_pmu_pm_unregister(xuantie_link_pmu);
	cpuhp_state_remove_instance(xuantie_link_pmu_cpuhp_state,
				    &xuantie_link_pmu->node);

	return 0;
}

static const struct xuantie_link_devtype_data xl100_devtype_data = {
	.quirks = XUANTIE_LINK_DEV_TYPE_XL100,
};

static const struct xuantie_link_devtype_data xl200_devtype_data = {
	.quirks = XUANTIE_LINK_DEV_TYPE_XL200,
};

static const struct xuantie_link_devtype_data xl300_devtype_data = {
	.quirks = XUANTIE_LINK_DEV_TYPE_XL300,
};

static const struct of_device_id xuantie_link_pmu_of_device_ids[] = {
	{ .compatible = "xuantie,xl100-pmu", .data = &xl100_devtype_data, },
	{ .compatible = "xuantie,xl200-pmu", .data = &xl200_devtype_data, },
	{ .compatible = "xuantie,xl300-pmu", .data = &xl300_devtype_data, },
	{ },
};
MODULE_DEVICE_TABLE(of, xuantie_link_pmu_of_device_ids);

static struct platform_driver xuantie_link_pmu_driver = {
	.driver = {
		.name = XUANTIE_LINK_PMU_PDEV_NAME,
		.of_match_table = xuantie_link_pmu_of_device_ids,
		.suppress_bind_attrs = true,
	},
	.probe  = xuantie_link_pmu_device_probe,
	.remove = xuantie_link_pmu_device_remove,
};

static int xuantie_link_pmu_online_cpu(unsigned int cpu,
				       struct hlist_node *node)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		hlist_entry_safe(node, struct xuantie_link_pmu, node);

	if (cpumask_empty(&xuantie_link_pmu->cpumask))
		cpumask_set_cpu(cpu, &xuantie_link_pmu->cpumask);

	WARN_ON(irq_set_affinity(xuantie_link_pmu->irq, cpumask_of(cpu)));

	return 0;
}

static int xuantie_link_pmu_offline_cpu(unsigned int cpu,
					struct hlist_node *node)
{
	struct xuantie_link_pmu *xuantie_link_pmu =
		hlist_entry_safe(node, struct xuantie_link_pmu, node);
	unsigned int target;

	if (!cpumask_test_and_clear_cpu(cpu, &xuantie_link_pmu->cpumask))
		return 0;

	target = cpumask_any_but(cpu_online_mask, cpu);
	if (target >= nr_cpu_ids)
		return 0;

	perf_pmu_migrate_context(&xuantie_link_pmu->pmu, cpu, target);

	cpumask_set_cpu(target, &xuantie_link_pmu->cpumask);
	WARN_ON(irq_set_affinity(xuantie_link_pmu->irq, cpumask_of(target)));

	return 0;
}

static int __init xuantie_link_pmu_init(void)
{
	int ret;

	ret = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN,
				      "soc/xuantie/xuantie_link_pmu:online",
				      xuantie_link_pmu_online_cpu,
				      xuantie_link_pmu_offline_cpu);
	if (ret < 0)
		return ret;

	xuantie_link_pmu_cpuhp_state = ret;

	return platform_driver_register(&xuantie_link_pmu_driver);
}

device_initcall(xuantie_link_pmu_init);

MODULE_DESCRIPTION("PMU driver for Xuantie Link");
MODULE_AUTHOR("Chen Pei <cp0613@linux.alibaba.com>");
MODULE_LICENSE("GPL");
