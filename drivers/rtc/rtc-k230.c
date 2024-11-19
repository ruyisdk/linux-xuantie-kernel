// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

/* K230 RTC register offsets and bits */
#define RTC_DATE		0x00
#define  DATE_DAY_OFFSET	0
#define  DATE_MONTH_OFFSET	8
#define  DATE_YEAR_L_OFFSET	16
#define  DATE_YEAR_LEAP_OFFSET	23
#define  DATE_YEAR_H_OFFSET	24
#define RTC_TIME		0x04
#define  TIME_SECOND_OFFSET	0
#define  TIME_MINUTE_OFFSET	8
#define  TIME_HOUR_OFFSET	16
#define  TIME_WEEK_OFFSET	24
#define RTC_ALARM_DATE		0x08
#define RTC_ALARM_TIME		0x0c
#define RTC_COUNT		0x10
#define  COUNT_CURR_OFFSET	0
#define  COUNT_SUM_OFFSET	16
#define RTC_CTRL		0x14
#define  CTRL_WR_EN		BIT(0)
#define  CTRL_RD_EN		BIT(1)
#define  CTRL_TICK_IRQ_EN	BIT(8)
#define  CTRL_TICK_IRQ_EN	BIT(8)
#define  CTRL_TICK_IRQ_SEL	GENMASK(9, 12)
#define  CTRL_ALARM_IRQ_EN	BIT(16)
#define  CTRL_ALARM_IRQ_CLR	BIT(17)
#define  CTRL_SECOND_CMP	BIT(24)
#define  CTRL_MINUTE_CMP	BIT(25)
#define  CTRL_HOUR_CMP		BIT(26)
#define  CTRL_WEEK_CMP		BIT(27)
#define  CTRL_DAY_CMP		BIT(28)
#define  CTRL_MONTH_CMP		BIT(29)
#define  CTRL_YEAR_CMP		BIT(30)

#define rtc_readl(dev, reg)		readl((dev)->base + (reg))
#define rtc_writel(dev, reg, val)	writel((val), (dev)->base + (reg))

struct k230_rtc {
	struct rtc_device *rtc;
	void __iomem *base;
};

static int k230_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct k230_rtc *rtc = dev_get_drvdata(dev);
	u32 value;
	int year, century;

	year = tm->tm_year + 1900;
	if (year < 1 || year > 12800) {
		dev_err(dev, "rtc only supports year in range %u - %u\n",
			1, 12800);
		return -EINVAL;
	}

	value = rtc_readl(rtc, RTC_CTRL);
	value |= CTRL_WR_EN;
	rtc_writel(rtc, RTC_CTRL, value);

	century = year / 100;
	year = year % 100;
	if (year == 0) {
		year = 100;
		century = century - 1;
	}
	value = (century << DATE_YEAR_H_OFFSET) |
		(year << DATE_YEAR_L_OFFSET) |
		((tm->tm_mon + 1) << DATE_MONTH_OFFSET) |
		((tm->tm_mday) << DATE_DAY_OFFSET);
	rtc_writel(rtc, RTC_DATE, value);
	value = ((tm->tm_wday) << TIME_WEEK_OFFSET) |
		((tm->tm_hour) << TIME_HOUR_OFFSET) |
		((tm->tm_min) << TIME_MINUTE_OFFSET) |
		((tm->tm_sec) << TIME_SECOND_OFFSET);
	rtc_writel(rtc, RTC_TIME, value);

	rtc_writel(rtc, RTC_COUNT, (32767UL << COUNT_SUM_OFFSET));

	value = rtc_readl(rtc, RTC_CTRL);
	value &= ~CTRL_WR_EN;
	rtc_writel(rtc, RTC_CTRL, value);

	return 0;
}

static int k230_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct k230_rtc *rtc = dev_get_drvdata(dev);
	u32 value, date_value, date_value_tmp, time_value;
	int year, century;

	do {
		date_value = rtc_readl(rtc, RTC_DATE);
		time_value = rtc_readl(rtc, RTC_TIME);
		date_value_tmp = rtc_readl(rtc, RTC_DATE);
	} while (date_value != date_value_tmp);

	tm->tm_sec  = (time_value >> TIME_SECOND_OFFSET) & 0x3f;
	tm->tm_min  = (time_value >> TIME_MINUTE_OFFSET) & 0x3f;
	tm->tm_hour = (time_value >> TIME_HOUR_OFFSET) & 0x1f;
	tm->tm_wday = (time_value >> TIME_WEEK_OFFSET) & 0x7;
	tm->tm_mon  = ((date_value >> DATE_MONTH_OFFSET) & 0xf) - 1;
	tm->tm_mday = (date_value >> DATE_DAY_OFFSET) & 0xf;
	century = (date_value >> DATE_YEAR_H_OFFSET) & 0x7f;
	year = (date_value >> DATE_YEAR_L_OFFSET) & 0x7f;
	tm->tm_year = century * 100 + year;
	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
	tm->tm_year -= 1900;

	return 0;
}

static int k230_rtc_init(struct k230_rtc *rtc)
{
	rtc_writel(rtc, RTC_CTRL, CTRL_RD_EN);

	return 0;
}

static const struct rtc_class_ops k230_rtc_ops = {
	.read_time		= k230_rtc_read_time,
	.set_time		= k230_rtc_set_time,
};

static int k230_rtc_probe(struct platform_device *pdev)
{
	struct k230_rtc *rtc;

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rtc->base))
		return PTR_ERR(rtc->base);

	platform_set_drvdata(pdev, rtc);

	k230_rtc_init(rtc);

	rtc->rtc = devm_rtc_device_register(&pdev->dev, "k230-rtc",
					    &k230_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		dev_err(&pdev->dev, "can't register rtc device\n");
		return PTR_ERR(rtc->rtc);
	}

	return 0;
}

static void k230_rtc_remove(struct platform_device *pdev)
{
	struct k230_rtc *rtc = platform_get_drvdata(pdev);

	rtc_writel(rtc, RTC_CTRL, 0);
}

static const struct of_device_id k230_rtc_match[] = {
	{ .compatible = "canaan,k230-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, k230_rtc_match);

static struct platform_driver k230_rtc_driver = {
	.probe	= k230_rtc_probe,
	.remove_new = k230_rtc_remove,
	.driver	= {
		.name = "k230-rtc",
		.of_match_table	= k230_rtc_match,
	},
};
module_platform_driver(k230_rtc_driver);

MODULE_AUTHOR("Canaan SDK Team");
MODULE_DESCRIPTION("Canaan Kendyte K230 chip ADC Driver");
MODULE_LICENSE("GPL");
