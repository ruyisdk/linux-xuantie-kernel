// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022, Canaan Bright Sight Co., Ltd
 * Copyright (c) 2011 Jamie Iles
 *
 * All enquiries to support@picochip.com
 * based on gpio-dwapb.c
 */
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "gpiolib.h"

#define GPIO_SWPORTA_DR		0x00
#define GPIO_SWPORTA_DDR	0x04
#define GPIO_SWPORTB_DR		0x0c
#define GPIO_SWPORTB_DDR	0x10
#define GPIO_SWPORTC_DR		0x18
#define GPIO_SWPORTC_DDR	0x1c
#define GPIO_SWPORTD_DR		0x24
#define GPIO_SWPORTD_DDR	0x28
#define GPIO_INTEN		0x30
#define GPIO_INTMASK		0x34
#define GPIO_INTTYPE_LEVEL	0x38
#define GPIO_INT_POLARITY	0x3c
#define GPIO_INTSTATUS		0x40
#define GPIO_PORTA_DEBOUNCE	0x48
#define GPIO_PORTA_EOI		0x4c
#define GPIO_EXT_PORTA		0x50
#define GPIO_EXT_PORTB		0x54
#define GPIO_EXT_PORTC		0x58
#define GPIO_EXT_PORTD		0x5c

#define K230_DRIVER_NAME	"gpio-k230"
#define K230_MAX_PORTS		4
#define K230_MAX_GPIOS		32

#define GPIO_EXT_PORT_STRIDE	0x04 /* register stride 32 bits */
#define GPIO_SWPORT_DR_STRIDE	0x0c /* register stride 3*32 bits */
#define GPIO_SWPORT_DDR_STRIDE	0x0c /* register stride 3*32 bits */

#define GPIO_REG_OFFSET_V1	0
#define GPIO_REG_OFFSET_V2	1
#define GPIO_REG_OFFSET_MASK	BIT(0)

#define GPIO_INTMASK_V2		0x44
#define GPIO_INTTYPE_LEVEL_V2	0x34
#define GPIO_INT_POLARITY_V2	0x38
#define GPIO_INTSTATUS_V2	0x3c
#define GPIO_PORTA_EOI_V2	0x40

#define K230_NR_CLOCKS		2

struct k230_gpio;

struct k230_port_property {
	struct fwnode_handle *fwnode;
	unsigned int idx;
	unsigned int ngpio;
	unsigned int gpio_base;
	int irq[K230_MAX_GPIOS];
};

struct k230_platform_data {
	struct k230_port_property *properties;
	unsigned int nports;
};

#ifdef CONFIG_PM_SLEEP
/* Store GPIO context across system-wide suspend/resume transitions */
struct k230_context {
	u32 data;
	u32 dir;
	u32 ext;
	u32 int_en;
	u32 int_mask;
	u32 int_type;
	u32 int_pol;
	u32 int_deb;
	u32 wake_en;
};
#endif

struct k230_gpio_port_irqchip {
	unsigned int		nr_irqs;
	unsigned int		irq[K230_MAX_GPIOS];
};

struct k230_gpio_port {
	struct gpio_chip	gc;
	struct k230_gpio_port_irqchip *pirq;
	struct k230_gpio	*gpio;
#ifdef CONFIG_PM_SLEEP
	struct k230_context	*ctx;
#endif
	unsigned int		idx;
};
#define to_k230_gpio(_gc) \
	(container_of(_gc, struct k230_gpio_port, gc)->gpio)

struct k230_gpio {
	struct	device		*dev;
	void __iomem		*regs;
	struct k230_gpio_port	*ports;
	unsigned int		nr_ports;
	unsigned int		flags;
	struct reset_control	*rst;
	struct clk_bulk_data	clks[K230_NR_CLOCKS];
};

static inline u32 gpio_reg_v2_convert(unsigned int offset)
{
	switch (offset) {
	case GPIO_INTMASK:
		return GPIO_INTMASK_V2;
	case GPIO_INTTYPE_LEVEL:
		return GPIO_INTTYPE_LEVEL_V2;
	case GPIO_INT_POLARITY:
		return GPIO_INT_POLARITY_V2;
	case GPIO_INTSTATUS:
		return GPIO_INTSTATUS_V2;
	case GPIO_PORTA_EOI:
		return GPIO_PORTA_EOI_V2;
	}

	return offset;
}

static inline u32 gpio_reg_convert(struct k230_gpio *gpio, unsigned int offset)
{
	if ((gpio->flags & GPIO_REG_OFFSET_MASK) == GPIO_REG_OFFSET_V2)
		return gpio_reg_v2_convert(offset);

	return offset;
}

static inline u32 k230_read(struct k230_gpio *gpio, unsigned int offset)
{
	struct gpio_chip *gc	= &gpio->ports[0].gc;
	void __iomem *reg_base	= gpio->regs;

	return gc->read_reg(reg_base + gpio_reg_convert(gpio, offset));
}

static inline void k230_write(struct k230_gpio *gpio, unsigned int offset,
			       u32 val)
{
	struct gpio_chip *gc	= &gpio->ports[0].gc;
	void __iomem *reg_base	= gpio->regs;

	gc->write_reg(reg_base + gpio_reg_convert(gpio, offset), val);
}

static struct k230_gpio_port *k230_offs_to_port(struct k230_gpio *gpio, unsigned int offs)
{
	struct k230_gpio_port *port;
	int i;

	for (i = 0; i < gpio->nr_ports; i++) {
		port = &gpio->ports[i];
		if (port->idx == offs / K230_MAX_GPIOS)
			return port;
	}

	return NULL;
}

static void k230_toggle_trigger(struct k230_gpio *gpio, unsigned int offs)
{
	struct k230_gpio_port *port = k230_offs_to_port(gpio, offs);
	struct gpio_chip *gc;
	u32 pol;
	int val;

	if (!port)
		return;
	gc = &port->gc;

	pol = k230_read(gpio, GPIO_INT_POLARITY);
	/* Just read the current value right out of the data register */
	val = gc->get(gc, offs % K230_MAX_GPIOS);
	if (val)
		pol &= ~BIT(offs);
	else
		pol |= BIT(offs);

	k230_write(gpio, GPIO_INT_POLARITY, pol);
}

static u32 k230_do_irq(struct k230_gpio *gpio)
{
	struct gpio_chip *gc = &gpio->ports[0].gc;
	unsigned long irq_status;
	irq_hw_number_t hwirq;

	irq_status = k230_read(gpio, GPIO_INTSTATUS);
	for_each_set_bit(hwirq, &irq_status, K230_MAX_GPIOS) {
		int gpio_irq = irq_find_mapping(gc->irq.domain, hwirq);
		u32 irq_type = irq_get_trigger_type(gpio_irq);

		generic_handle_irq(gpio_irq);

		if ((irq_type & IRQ_TYPE_SENSE_MASK) == IRQ_TYPE_EDGE_BOTH)
			k230_toggle_trigger(gpio, hwirq);
	}

	return irq_status;
}

static void k230_irq_handler(struct irq_desc *desc)
{
	struct k230_gpio *gpio = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);
	k230_do_irq(gpio);
	chained_irq_exit(chip, desc);
}

static void k230_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct k230_gpio *gpio = to_k230_gpio(gc);
	u32 val = BIT(irqd_to_hwirq(d));
	unsigned long flags;

	raw_spin_lock_irqsave(&gc->bgpio_lock, flags);
	k230_write(gpio, GPIO_PORTA_EOI, val);
	raw_spin_unlock_irqrestore(&gc->bgpio_lock, flags);
}

static void k230_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct k230_gpio *gpio = to_k230_gpio(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&gc->bgpio_lock, flags);
	val = k230_read(gpio, GPIO_INTMASK) | BIT(hwirq);
	k230_write(gpio, GPIO_INTMASK, val);
	raw_spin_unlock_irqrestore(&gc->bgpio_lock, flags);

	gpiochip_disable_irq(gc, hwirq);
}

static void k230_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct k230_gpio *gpio = to_k230_gpio(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	unsigned long flags;
	u32 val;

	gpiochip_enable_irq(gc, hwirq);

	raw_spin_lock_irqsave(&gc->bgpio_lock, flags);
	val = k230_read(gpio, GPIO_INTMASK) & ~BIT(hwirq);
	k230_write(gpio, GPIO_INTMASK, val);
	raw_spin_unlock_irqrestore(&gc->bgpio_lock, flags);
}

static void k230_irq_enable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct k230_gpio *gpio = to_k230_gpio(gc);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&gc->bgpio_lock, flags);
	val = k230_read(gpio, GPIO_INTEN);
	val |= BIT(irqd_to_hwirq(d));
	k230_write(gpio, GPIO_INTEN, val);
	raw_spin_unlock_irqrestore(&gc->bgpio_lock, flags);
}

static void k230_irq_disable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct k230_gpio *gpio = to_k230_gpio(gc);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&gc->bgpio_lock, flags);
	val = k230_read(gpio, GPIO_INTEN);
	val &= ~BIT(irqd_to_hwirq(d));
	k230_write(gpio, GPIO_INTEN, val);
	raw_spin_unlock_irqrestore(&gc->bgpio_lock, flags);
}

static int k230_irq_set_type(struct irq_data *d, u32 type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct k230_gpio *gpio = to_k230_gpio(gc);
	irq_hw_number_t bit = irqd_to_hwirq(d);
	unsigned long level, polarity, flags;

	raw_spin_lock_irqsave(&gc->bgpio_lock, flags);
	level = k230_read(gpio, GPIO_INTTYPE_LEVEL);
	polarity = k230_read(gpio, GPIO_INT_POLARITY);

	switch (type) {
	case IRQ_TYPE_EDGE_BOTH:
		level |= BIT(bit);
		k230_toggle_trigger(gpio, bit);
		break;
	case IRQ_TYPE_EDGE_RISING:
		level |= BIT(bit);
		polarity |= BIT(bit);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		level |= BIT(bit);
		polarity &= ~BIT(bit);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		level &= ~BIT(bit);
		polarity |= BIT(bit);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		level &= ~BIT(bit);
		polarity &= ~BIT(bit);
		break;
	}

	if (type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(d, handle_level_irq);
	else if (type & IRQ_TYPE_EDGE_BOTH)
		irq_set_handler_locked(d, handle_edge_irq);

	k230_write(gpio, GPIO_INTTYPE_LEVEL, level);
	if (type != IRQ_TYPE_EDGE_BOTH)
		k230_write(gpio, GPIO_INT_POLARITY, polarity);
	raw_spin_unlock_irqrestore(&gc->bgpio_lock, flags);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int k230_irq_set_wake(struct irq_data *d, unsigned int enable)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct k230_gpio *gpio = to_k230_gpio(gc);
	struct k230_context *ctx = gpio->ports[0].ctx;
	irq_hw_number_t bit = irqd_to_hwirq(d);

	if (enable)
		ctx->wake_en |= BIT(bit);
	else
		ctx->wake_en &= ~BIT(bit);

	return 0;
}
#else
#define k230_irq_set_wake	NULL
#endif

static const struct irq_chip k230_irq_chip = {
	.name		= K230_DRIVER_NAME,
	.irq_ack	= k230_irq_ack,
	.irq_mask	= k230_irq_mask,
	.irq_unmask	= k230_irq_unmask,
	.irq_set_type	= k230_irq_set_type,
	.irq_enable	= k230_irq_enable,
	.irq_disable	= k230_irq_disable,
	.irq_set_wake	= k230_irq_set_wake,
	.flags		= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int k230_gpio_set_debounce(struct gpio_chip *gc,
				   unsigned int offset, unsigned int debounce)
{
	struct k230_gpio_port *port = gpiochip_get_data(gc);
	struct k230_gpio *gpio = port->gpio;
	unsigned long flags, val_deb;
	unsigned long mask = BIT(offset);

	raw_spin_lock_irqsave(&gc->bgpio_lock, flags);

	val_deb = k230_read(gpio, GPIO_PORTA_DEBOUNCE);
	if (debounce)
		val_deb |= mask;
	else
		val_deb &= ~mask;
	k230_write(gpio, GPIO_PORTA_DEBOUNCE, val_deb);

	raw_spin_unlock_irqrestore(&gc->bgpio_lock, flags);

	return 0;
}

static int k230_gpio_set_config_default(struct gpio_chip *gc,
				   unsigned int offset, unsigned long config)
{
	return 0;
}

static int k230_gpio_set_config(struct gpio_chip *gc, unsigned int offset,
				 unsigned long config)
{
	if (pinconf_to_config_param(config) == PIN_CONFIG_INPUT_DEBOUNCE) {
		u32 debounce = pinconf_to_config_argument(config);

		return k230_gpio_set_debounce(gc, offset, debounce);
	}

	return k230_gpio_set_config_default(gc, offset, config);
}

static int k230_convert_irqs(struct k230_gpio_port_irqchip *pirq,
			      struct k230_port_property *pp)
{
	int i;

	/* Group all available IRQs into an array of parental IRQs. */
	for (i = 0; i < pp->ngpio; ++i) {
		if (!pp->irq[i])
			continue;

		pirq->irq[pirq->nr_irqs++] = pp->irq[i];
	}

	return pirq->nr_irqs ? 0 : -ENOENT;
}

static void k230_configure_irqs(struct k230_gpio *gpio,
				 struct k230_gpio_port *port,
				 struct k230_port_property *pp)
{
	struct k230_gpio_port_irqchip *pirq;
	struct gpio_chip *gc = &port->gc;
	struct gpio_irq_chip *girq;

	pirq = devm_kzalloc(gpio->dev, sizeof(*pirq), GFP_KERNEL);
	if (!pirq)
		return;

	if (k230_convert_irqs(pirq, pp)) {
		dev_warn(gpio->dev, "no IRQ for port%d\n", pp->idx);
		goto err_kfree_pirq;
	}

	girq = &gc->irq;
	girq->handler = handle_bad_irq;
	girq->default_type = IRQ_TYPE_NONE;

	port->pirq = pirq;

	girq->num_parents = pirq->nr_irqs;
	girq->parents = pirq->irq;
	girq->parent_handler_data = gpio;
	girq->parent_handler = k230_irq_handler;

	gpio_irq_chip_set_chip(girq, &k230_irq_chip);

	return;

err_kfree_pirq:
	devm_kfree(gpio->dev, pirq);
}

static int k230_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	return 0;
}

static void k230_gpio_free(struct gpio_chip *gc, unsigned int offset)
{
}

static int k230_gpio_add_port(struct k230_gpio *gpio,
			       struct k230_port_property *pp,
			       unsigned int offs)
{
	struct k230_gpio_port *port;
	void __iomem *dat, *set, *dirout;
	int err;

	port = &gpio->ports[offs];
	port->gpio = gpio;
	port->idx = pp->idx;

#ifdef CONFIG_PM_SLEEP
	port->ctx = devm_kzalloc(gpio->dev, sizeof(*port->ctx), GFP_KERNEL);
	if (!port->ctx)
		return -ENOMEM;
#endif

	dat = gpio->regs + GPIO_EXT_PORTA + pp->idx * GPIO_EXT_PORT_STRIDE;
	set = gpio->regs + GPIO_SWPORTA_DR + pp->idx * GPIO_SWPORT_DR_STRIDE;
	dirout = gpio->regs + GPIO_SWPORTA_DDR + pp->idx * GPIO_SWPORT_DDR_STRIDE;

	/* This registers 32 GPIO lines per port */
	err = bgpio_init(&port->gc, gpio->dev, 4, dat, set, NULL, dirout,
			 NULL, 0);
	if (err) {
		dev_err(gpio->dev, "failed to init gpio chip for port%d\n",
			port->idx);
		return err;
	}

	port->gc.fwnode = pp->fwnode;
	port->gc.ngpio = pp->ngpio;
	port->gc.base = pp->gpio_base;
	// port->gc.request = gpiochip_generic_request;
	// port->gc.free = gpiochip_generic_free;
	port->gc.request = k230_gpio_request;
	port->gc.free = k230_gpio_free;

	/* Only port A support debounce */
	if (pp->idx == 0)
		port->gc.set_config = k230_gpio_set_config;
	else
		port->gc.set_config = k230_gpio_set_config_default;

	/* Only port A can provide interrupts in all configurations of the IP */
	if (pp->idx == 0)
		k230_configure_irqs(gpio, port, pp);

	err = devm_gpiochip_add_data(gpio->dev, &port->gc, port);
	if (err) {
		dev_err(gpio->dev, "failed to register gpiochip for port%d\n",
			port->idx);
		return err;
	}

	return 0;
}

static void k230_get_irq(struct device *dev, struct fwnode_handle *fwnode,
			  struct k230_port_property *pp)
{
	int irq, j;

	for (j = 0; j < pp->ngpio; j++) {
		irq = fwnode_irq_get(fwnode, j);
		if (irq > 0)
			pp->irq[j] = irq;
	}
}

static struct k230_platform_data *k230_gpio_get_pdata(struct device *dev)
{
	struct fwnode_handle *fwnode;
	struct k230_platform_data *pdata;
	struct k230_port_property *pp;
	int nports;
	int i;

	nports = device_get_child_node_count(dev);
	if (nports == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->properties = devm_kcalloc(dev, nports, sizeof(*pp), GFP_KERNEL);
	if (!pdata->properties)
		return ERR_PTR(-ENOMEM);

	pdata->nports = nports;

	i = 0;
	device_for_each_child_node(dev, fwnode)  {
		pp = &pdata->properties[i++];
		pp->fwnode = fwnode;

		if (fwnode_property_read_u32(fwnode, "reg", &pp->idx) ||
		    pp->idx >= K230_MAX_PORTS) {
			dev_err(dev,
				"missing/invalid port index for port%d\n", i);
			fwnode_handle_put(fwnode);
			return ERR_PTR(-EINVAL);
		}

		if (fwnode_property_read_u32(fwnode, "ngpios", &pp->ngpio) &&
		    fwnode_property_read_u32(fwnode, "snps,nr-gpios", &pp->ngpio)) {
			dev_info(dev,
				 "failed to get number of gpios for port%d\n",
				 i);
			pp->ngpio = K230_MAX_GPIOS;
		}

		pp->gpio_base	= -1;

		/* For internal use only, new platforms mustn't exercise this */
		if (is_software_node(fwnode))
			fwnode_property_read_u32(fwnode, "gpio-base", &pp->gpio_base);

		/*
		 * Only port A can provide interrupts in all configurations of
		 * the IP.
		 */
		if (pp->idx == 0)
			k230_get_irq(dev, fwnode, pp);
	}

	return pdata;
}

static void k230_assert_reset(void *data)
{
	struct k230_gpio *gpio = data;

	reset_control_assert(gpio->rst);
}

static int k230_get_reset(struct k230_gpio *gpio)
{
	int err;

	gpio->rst = devm_reset_control_get_optional_shared(gpio->dev, NULL);
	if (IS_ERR(gpio->rst))
		return dev_err_probe(gpio->dev, PTR_ERR(gpio->rst),
				     "Cannot get reset descriptor\n");

	err = reset_control_deassert(gpio->rst);
	if (err) {
		dev_err(gpio->dev, "Cannot deassert reset lane\n");
		return err;
	}

	return devm_add_action_or_reset(gpio->dev, k230_assert_reset, gpio);
}

static void k230_disable_clks(void *data)
{
	struct k230_gpio *gpio = data;

	clk_bulk_disable_unprepare(K230_NR_CLOCKS, gpio->clks);
}

static int k230_get_clks(struct k230_gpio *gpio)
{
	int err;

	/* Optional bus and debounce clocks */
	gpio->clks[0].id = "bus";
	gpio->clks[1].id = "db";
	err = devm_clk_bulk_get_optional(gpio->dev, K230_NR_CLOCKS,
					 gpio->clks);
	if (err)
		return dev_err_probe(gpio->dev, err,
				     "Cannot get APB/Debounce clocks\n");

	err = clk_bulk_prepare_enable(K230_NR_CLOCKS, gpio->clks);
	if (err) {
		dev_err(gpio->dev, "Cannot enable APB/Debounce clocks\n");
		return err;
	}

	return devm_add_action_or_reset(gpio->dev, k230_disable_clks, gpio);
}

static const struct of_device_id k230_of_match[] = {
	{ .compatible = "canaan,k230-apb-gpio", .data = (void *)0 },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, k230_of_match);

static int k230_gpio_probe(struct platform_device *pdev)
{
	unsigned int i;
	struct k230_gpio *gpio;
	int err;
	struct k230_platform_data *pdata;
	struct device *dev = &pdev->dev;

	pdata = k230_gpio_get_pdata(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->dev = &pdev->dev;
	gpio->nr_ports = pdata->nports;

	err = k230_get_reset(gpio);
	if (err)
		return err;

	gpio->ports = devm_kcalloc(&pdev->dev, gpio->nr_ports,
				   sizeof(*gpio->ports), GFP_KERNEL);
	if (!gpio->ports)
		return -ENOMEM;

	gpio->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpio->regs))
		return PTR_ERR(gpio->regs);

	err = k230_get_clks(gpio);
	if (err)
		return err;

	gpio->flags = (uintptr_t)device_get_match_data(dev);

	for (i = 0; i < gpio->nr_ports; i++) {
		err = k230_gpio_add_port(gpio, &pdata->properties[i], i);
		if (err)
			return err;
	}

	platform_set_drvdata(pdev, gpio);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int k230_gpio_suspend(struct device *dev)
{
	struct k230_gpio *gpio = dev_get_drvdata(dev);
	struct gpio_chip *gc	= &gpio->ports[0].gc;
	unsigned long flags;
	int i;

	raw_spin_lock_irqsave(&gc->bgpio_lock, flags);
	for (i = 0; i < gpio->nr_ports; i++) {
		unsigned int offset;
		unsigned int idx = gpio->ports[i].idx;
		struct k230_context *ctx = gpio->ports[i].ctx;

		offset = GPIO_SWPORTA_DDR + idx * GPIO_SWPORT_DDR_STRIDE;
		ctx->dir = k230_read(gpio, offset);

		offset = GPIO_SWPORTA_DR + idx * GPIO_SWPORT_DR_STRIDE;
		ctx->data = k230_read(gpio, offset);

		offset = GPIO_EXT_PORTA + idx * GPIO_EXT_PORT_STRIDE;
		ctx->ext = k230_read(gpio, offset);

		/* Only port A can provide interrupts */
		if (idx == 0) {
			ctx->int_mask	= k230_read(gpio, GPIO_INTMASK);
			ctx->int_en	= k230_read(gpio, GPIO_INTEN);
			ctx->int_pol	= k230_read(gpio, GPIO_INT_POLARITY);
			ctx->int_type	= k230_read(gpio, GPIO_INTTYPE_LEVEL);
			ctx->int_deb	= k230_read(gpio, GPIO_PORTA_DEBOUNCE);

			/* Mask out interrupts */
			k230_write(gpio, GPIO_INTMASK, ~ctx->wake_en);
		}
	}
	raw_spin_unlock_irqrestore(&gc->bgpio_lock, flags);

	clk_bulk_disable_unprepare(K230_NR_CLOCKS, gpio->clks);

	return 0;
}

static int k230_gpio_resume(struct device *dev)
{
	struct k230_gpio *gpio = dev_get_drvdata(dev);
	struct gpio_chip *gc	= &gpio->ports[0].gc;
	unsigned long flags;
	int i, err;

	err = clk_bulk_prepare_enable(K230_NR_CLOCKS, gpio->clks);
	if (err) {
		dev_err(gpio->dev, "Cannot reenable APB/Debounce clocks\n");
		return err;
	}

	raw_spin_lock_irqsave(&gc->bgpio_lock, flags);
	for (i = 0; i < gpio->nr_ports; i++) {
		unsigned int offset;
		unsigned int idx = gpio->ports[i].idx;
		struct k230_context *ctx = gpio->ports[i].ctx;

		offset = GPIO_SWPORTA_DR + idx * GPIO_SWPORT_DR_STRIDE;
		k230_write(gpio, offset, ctx->data);

		offset = GPIO_SWPORTA_DDR + idx * GPIO_SWPORT_DDR_STRIDE;
		k230_write(gpio, offset, ctx->dir);

		offset = GPIO_EXT_PORTA + idx * GPIO_EXT_PORT_STRIDE;
		k230_write(gpio, offset, ctx->ext);

		/* Only port A can provide interrupts */
		if (idx == 0) {
			k230_write(gpio, GPIO_INTTYPE_LEVEL, ctx->int_type);
			k230_write(gpio, GPIO_INT_POLARITY, ctx->int_pol);
			k230_write(gpio, GPIO_PORTA_DEBOUNCE, ctx->int_deb);
			k230_write(gpio, GPIO_INTEN, ctx->int_en);
			k230_write(gpio, GPIO_INTMASK, ctx->int_mask);

			/* Clear out spurious interrupts */
			k230_write(gpio, GPIO_PORTA_EOI, 0xffffffff);
		}
	}
	raw_spin_unlock_irqrestore(&gc->bgpio_lock, flags);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(k230_gpio_pm_ops, k230_gpio_suspend,
			 k230_gpio_resume);

static struct platform_driver k230_gpio_driver = {
	.driver		= {
		.name	= K230_DRIVER_NAME,
		.pm	= &k230_gpio_pm_ops,
		.of_match_table = k230_of_match,
	},
	.probe		= k230_gpio_probe,
};

module_platform_driver(k230_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Canaan K230 GPIO driver");
MODULE_ALIAS("platform:" K230_DRIVER_NAME);
