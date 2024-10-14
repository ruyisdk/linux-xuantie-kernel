// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Kendryte K230 temperature sensor support driver
 *
 * Copyright (C) 2024, Canaan Bright Sight Co., Ltd
 */

#include <linux/clk.h>
#include <linux/cpu_cooling.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/types.h>

#define TS_CONFIG 0x00
#define TS_DATA 0x04
#define TS_POWERDOWN 0x22
#define TS_POWERON 0x23

struct canaan_thermal_data {
	struct thermal_zone_device *tz;
	void __iomem *base;
};

static int canaan_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct canaan_thermal_data *data = tz->devdata;
	u32 val = 0;

	iowrite32(TS_POWERDOWN, data->base + TS_CONFIG);
	iowrite32(TS_POWERON, data->base + TS_CONFIG);
	msleep(20);

	while (1) {
		val = ioread32(data->base + TS_DATA);
		// msleep(2600);

		if (val >> 12) {
			*temp = val;
			break;
		}
	}

	return 0;
}

static struct thermal_zone_device_ops canaan_tz_ops = {
	.get_temp = canaan_get_temp,
};

static const struct of_device_id of_canaan_thermal_match[] = {
	{ .compatible = "canaan,k230-tsensor" },
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, of_canaan_thermal_match);

static int canaan_thermal_probe(struct platform_device *pdev)
{
	struct canaan_thermal_data *data;
	struct resource *res;
	int ret;

	dev_vdbg(&pdev->dev, "[TS]: %s %d\n", __func__, __LINE__);

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	platform_set_drvdata(pdev, data);

	data->tz = thermal_tripless_zone_device_register(
		"canaan_thermal_zone", data, &canaan_tz_ops, NULL);

	if (IS_ERR(data->tz)) {
		ret = PTR_ERR(data->tz);
		dev_err(&pdev->dev,
			"failed to register thermal zone device %d\n", ret);
		return ret;
	}

	iowrite32(TS_POWERDOWN, data->base + TS_CONFIG);
	iowrite32(TS_POWERON, data->base + TS_CONFIG);
	msleep(20);

	dev_vdbg(&pdev->dev, "[TS]: %s %d\n", __func__, __LINE__);

	return 0;
}

static int canaan_thermal_remove(struct platform_device *pdev)
{
	struct canaan_thermal_data *data = platform_get_drvdata(pdev);

	thermal_zone_device_unregister(data->tz);

	return 0;
}

static struct platform_driver canaan_thermal = {
	.driver = {
		.name = "canaan_thermal",
		.of_match_table = of_canaan_thermal_match,
	},
	.probe = canaan_thermal_probe,
	.remove = canaan_thermal_remove,
};
module_platform_driver(canaan_thermal);

MODULE_DESCRIPTION("Thermal driver for canaan k230 Soc");
MODULE_LICENSE("GPL");
