// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform driver for a RISC-V CBQRI capacity controller that backs a CPU
 * cache. The controller is described in device tree by the generic
 * "riscv,cbqri-capacity-controller" compatible together with a phandle to the
 * cache node it governs. The driver hands it to the CBQRI core, which probes
 * the capabilities register and exposes a controller that supports allocation
 * as the resctrl cache allocation resource for that cache.
 */

#define pr_fmt(fmt) "cbqri-capacity: " fmt

#include <linux/cacheinfo.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/ioport.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/resctrl.h>
#include <linux/riscv_cbqri.h>
#include <linux/types.h>

static int cbqri_capacity_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cbqri_controller_info info = {};
	struct device_node *cache_np;
	cpumask_var_t cpu_mask;
	struct resource *res;
	u32 rcid_count, cache_level;
	int cache_id, cpu, ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	ret = of_property_read_u32(dev->of_node, "riscv,cbqri-rcid", &rcid_count);
	if (ret) {
		dev_err(dev, "missing riscv,cbqri-rcid\n");
		return ret;
	}

	cache_np = of_parse_phandle(dev->of_node, "riscv,cbqri-cache", 0);
	if (!cache_np) {
		dev_err(dev, "missing riscv,cbqri-cache phandle\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(cache_np, "cache-level", &cache_level);
	if (ret) {
		dev_err(dev, "%pOF: missing cache-level\n", cache_np);
		goto out_put;
	}

	if (!zalloc_cpumask_var(&cpu_mask, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto out_put;
	}

	/*
	 * Associate the controller with its cache instance via
	 * cacheinfo. The matching cache provides the cache id and the
	 * set of harts that share the cache.
	 */
	cache_id = -1;
	cpus_read_lock();
	for_each_online_cpu(cpu) {
		struct cacheinfo *ci = get_cpu_cacheinfo_level(cpu, cache_level);

		if (ci && ci->fw_token == cache_np) {
			cache_id = ci->id;
			cpumask_copy(cpu_mask, &ci->shared_cpu_map);
			break;
		}
	}
	cpus_read_unlock();

	if (cache_id < 0) {
		dev_err(dev, "%pOF: no online hart reports an L%u cache for this node\n",
			cache_np, cache_level);
		ret = -ENODEV;
		goto out_free;
	}

	info.type = CBQRI_CONTROLLER_TYPE_CAPACITY;
	info.addr = res->start;
	info.size = resource_size(res);
	info.rcid_count = rcid_count;
	info.cache_id = cache_id;

	ret = riscv_cbqri_register_cc_dt(&info, cache_level, cpu_mask);
	if (ret) {
		dev_err(dev, "failed to register capacity controller: %d\n", ret);
		goto out_free;
	}

	dev_info(dev, "registered L%u capacity controller at %pa (cache_id=%d, rcid=%u)\n",
		 cache_level, &info.addr, cache_id, rcid_count);

out_free:
	free_cpumask_var(cpu_mask);
out_put:
	of_node_put(cache_np);
	return ret;
}

static const struct of_device_id cbqri_capacity_of_match[] = {
	{ .compatible = "riscv,cbqri-capacity-controller" },
	{}
};
MODULE_DEVICE_TABLE(of, cbqri_capacity_of_match);

static struct platform_driver cbqri_capacity_driver = {
	.probe	= cbqri_capacity_probe,
	.driver = {
		.name		= "cbqri-capacity",
		.of_match_table	= cbqri_capacity_of_match,
		/*
		 * The controller is registered permanently into the
		 * CBQRI core for the life of the system. Block unbind
		 * so userspace cannot leave a dangling controller.
		 */
		.suppress_bind_attrs = true,
	},
};

/*
 * Register at device_initcall so probe runs before the CBQRI core's
 * late_initcall which walks the cbqri_controllers list.
 */
builtin_platform_driver(cbqri_capacity_driver);
