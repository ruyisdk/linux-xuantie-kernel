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
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define ADC_MAX_CHANNEL     6
#define ADC_MAX_DMA_CHN     3

struct k230_adc_regs {
	uint32_t trim_reg;
	uint32_t cfg_reg;
	uint32_t mode_reg;
	uint32_t thsd_reg;
	uint32_t dma_intr_reg;
	uint32_t data_reg[ADC_MAX_CHANNEL];
	uint32_t data_dma[ADC_MAX_DMA_CHN];
};

struct k230_adc_priv {
	void __iomem *base;
	struct clk *clk;
	struct mutex lock;
};

#define K230_ADC_CHANNEL(n)						\
	{								\
		.channel		= n,				\
		.datasheet_name		= "channel"#n,			\
		.type			= IIO_VOLTAGE,			\
		.indexed		= 1,				\
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),	\
	}

static const struct iio_chan_spec k230_adc_channels[] = {
	K230_ADC_CHANNEL(0),
	K230_ADC_CHANNEL(1),
	K230_ADC_CHANNEL(2),
	K230_ADC_CHANNEL(3),
	K230_ADC_CHANNEL(4),
	K230_ADC_CHANNEL(5),
};

static int k230_adc_init(struct k230_adc_priv *dev)
{
	struct k230_adc_regs *regs = dev->base;
	uint32_t data;

	data = readl(&regs->trim_reg);
	data &= ~(0x1);
	writel(data, &regs->trim_reg);
	data |= (0x1) | (0x1 << 20);
	writel(data, &regs->trim_reg);
	fsleep(1000);
	data &= ~(0x1 << 20);
	writel(data, &regs->trim_reg);

	writel(0x0, &regs->mode_reg);
	mutex_init(&dev->lock);

	return 0;
}

static int k230_adc_read(struct k230_adc_priv *dev, int channel)
{
	struct k230_adc_regs *regs = dev->base;
	int ret = -ETIMEDOUT, timeout;

	mutex_lock(&dev->lock);

	writel(channel | 0x10, &regs->cfg_reg);
	for (timeout = 0; timeout < 1000; timeout++) {
		if ((readl(&regs->cfg_reg) & 0x10100) == 0x10000) {
			ret = readl(&regs->data_reg[channel]);
			break;
		}
		fsleep(1);
	}

	mutex_unlock(&dev->lock);

	return ret;
}

static int k230_adc_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	struct k230_adc_priv *priv = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type != IIO_VOLTAGE)
			return -EINVAL;

		*val = k230_adc_read(priv, chan->channel);
		if (*val < 0)
			return *val;

		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static const struct iio_info k230_adc_info = {
	.read_raw = k230_adc_read_raw,
};

static int k230_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct k230_adc_priv *priv;
	struct device *dev = &pdev->dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev) {
		dev_err(dev, "Failed allocating iio device\n");
		return -ENOMEM;
	}

	priv = iio_priv(indio_dev);
	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "Failed getting clock\n");
		return ret;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(dev, "Could not prepare or enable clock.\n");
		return ret;
	}

	k230_adc_init(priv);

	indio_dev->name = dev_name(dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &k230_adc_info;
	indio_dev->channels = k230_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(k230_adc_channels);

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		dev_err(dev, "could not register k230-adc");

	return ret;
}

static const struct of_device_id k230_adc_match[] = {
	{ .compatible = "canaan,k230-adc", },
	{ },
};
MODULE_DEVICE_TABLE(of, k230_adc_match);

static struct platform_driver k230_adc_driver = {
	.driver	= {
		.name = "k230-adc",
		.of_match_table	= k230_adc_match,
	},
	.probe	= k230_adc_probe,
};
module_platform_driver(k230_adc_driver);

MODULE_AUTHOR("Canaan SDK Team");
MODULE_DESCRIPTION("Canaan Kendyte K230 chip ADC Driver");
MODULE_LICENSE("GPL");
