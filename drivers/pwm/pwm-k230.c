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
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/bitfield.h>

/* Register offsets */
#define PWM_PWMCFG		0x0
#define PWM_PWMCOUNT		0x8
#define PWM_PWMS		0x10
#define PWM_PWMCMP(i)		(0x20 + 4 * (i))

/* PWMCFG fields */
#define PWM_PWMCFG_SCALE	GENMASK(3, 0)
#define PWM_PWMCFG_STICKY	BIT(8)
#define PWM_PWMCFG_ZERO_CMP	BIT(9)
#define PWM_PWMCFG_DEGLITCH	BIT(10)
#define PWM_PWMCFG_EN_ALWAYS	BIT(12)
#define PWM_PWMCFG_EN_ONCE	BIT(13)
#define PWM_PWMCFG_CENTER	BIT(16)
#define PWM_PWMCFG_GANG		BIT(24)
#define PWM_PWMCFG_IP		BIT(28)

#define PWM_CMPWIDTH		16

struct k230_pwm_chip {
	struct pwm_chip	chip;
	void __iomem *regs;
	struct clk *clk;
	struct mutex lock;
	u8 user_count;
	u8 scale;
	u64 period;
	u64 clk_freq;
};

static inline struct k230_pwm_chip *k230_pwm_from_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct k230_pwm_chip, chip);
}

static int k230_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct k230_pwm_chip *priv = k230_pwm_from_chip(chip);

	mutex_lock(&priv->lock);
	priv->user_count++;
	mutex_unlock(&priv->lock);

	return 0;
}

static void k230_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct k230_pwm_chip *priv = k230_pwm_from_chip(chip);

	mutex_lock(&priv->lock);
	priv->user_count--;
	if (priv->user_count == 0)
		writel(0, priv->regs + PWM_PWMCFG);
	mutex_unlock(&priv->lock);
}

static void k230_pwm_init(struct k230_pwm_chip *priv)
{
	u64 val;

	mutex_init(&priv->lock);
	priv->clk_freq = clk_get_rate(priv->clk);

	val = readl(priv->regs + PWM_PWMCFG);
	if ((val & PWM_PWMCFG_EN_ALWAYS) == 0) {
		writel(1, priv->regs + PWM_PWMCMP(0));
		writel(0, priv->regs + PWM_PWMCMP(1));
		writel(0, priv->regs + PWM_PWMCMP(2));
		writel(0, priv->regs + PWM_PWMCMP(3));
	}
}

static void k230_pwm_update_period(struct k230_pwm_chip *priv)
{
	u64 val, scale, freq;

	freq = priv->clk_freq;
	val = div64_ul(priv->period * freq, NSEC_PER_SEC) - 1;
	scale = clamp(ilog2(val) - PWM_CMPWIDTH, 0, 0xf);
	val = val >> scale;
	if (val == 0)
		val = 1;
	writel(val, priv->regs + PWM_PWMCMP(0));

	val = PWM_PWMCFG_EN_ALWAYS | PWM_PWMCFG_ZERO_CMP |
	      FIELD_PREP(PWM_PWMCFG_SCALE, scale);
	writel(val, priv->regs + PWM_PWMCFG);
	priv->scale = scale;
}

static int k230_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			      struct pwm_state *state)
{
	struct k230_pwm_chip *priv = k230_pwm_from_chip(chip);
	u64 duty, period, freq, scale;

	scale = priv->scale;
	freq = priv->clk_freq;
	duty = readl(priv->regs + PWM_PWMCMP(pwm->hwpwm + 1));
	period = readl(priv->regs + PWM_PWMCMP(0)) + 1;
	if (duty > period)
		duty = period;

	*state = pwm->state;
	state->polarity = PWM_POLARITY_INVERSED;
	state->period = (period << scale) * NSEC_PER_SEC / freq;
	state->duty_cycle = (duty << scale) * NSEC_PER_SEC / freq;

	return 0;
}

static int k230_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			  const struct pwm_state *state)
{
	struct k230_pwm_chip *priv = k230_pwm_from_chip(chip);
	u64 val, duty, period, freq;

	if (state->polarity != PWM_POLARITY_INVERSED)
		return -EINVAL;

	period = state->period;
	duty = state->duty_cycle;
	if (!state->enabled)
		duty = 0;

	mutex_lock(&priv->lock);
	if (period != priv->period) {
		if (priv->user_count != 1 && priv->period) {
			mutex_unlock(&priv->lock);
			return -EBUSY;
		}
		priv->period = period;
		k230_pwm_update_period(priv);
	}
	mutex_unlock(&priv->lock);

	freq = priv->clk_freq;
	val = div64_ul(duty * freq, NSEC_PER_SEC);
	val = val >> priv->scale;
	val = min(val, (1U << PWM_CMPWIDTH) - 1);
	writel(val, priv->regs + PWM_PWMCMP(pwm->hwpwm + 1));

	return 0;
}

static const struct pwm_ops k230_pwm_ops = {
	.request = k230_pwm_request,
	.free = k230_pwm_free,
	.get_state = k230_pwm_get_state,
	.apply = k230_pwm_apply,
	.owner = THIS_MODULE,
};

static int k230_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct k230_pwm_chip *priv;
	struct pwm_chip *chip;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	chip = &priv->chip;
	chip->dev = dev;
	chip->ops = &k230_pwm_ops;
	chip->npwm = 3;

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	priv->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	k230_pwm_init(priv);

	ret = devm_pwmchip_add(dev, chip);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct of_device_id k230_pwm_of_match[] = {
	{ .compatible = "canaan,k230-pwm" },
	{},
};
MODULE_DEVICE_TABLE(of, k230_pwm_of_match);

static struct platform_driver k230_pwm_driver = {
	.probe = k230_pwm_probe,
	.driver = {
		.name = "k230-pwm",
		.of_match_table = k230_pwm_of_match,
	},
};
module_platform_driver(k230_pwm_driver);

MODULE_AUTHOR("Canaan SDK Team");
MODULE_DESCRIPTION("Canaan Kendyte K230 chip PWM Driver");
MODULE_LICENSE("GPL");
