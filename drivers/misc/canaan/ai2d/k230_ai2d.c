// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/irqflags.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/cacheinfo.h>
#include <linux/sizes.h>
#include <asm/csr.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/poll.h>

struct ai2d_plat {
	struct resource *res;
	void __iomem *regs;
	char *ai2d_reg;
	int major;
	int minor;
	int irq;
	struct class *class;
	struct device *device;
	struct cdev cdev;
};

static struct ai2d_plat *plat;
static unsigned int ai2d_int_flag;
static int ai2d_fasync_flag;
DECLARE_WAIT_QUEUE_HEAD(ai2d_waitq);
struct fasync_struct *ai2d_fasync;

static DEFINE_MUTEX(ai2d_mutex);

static void ai2d_interrupt_clear(void)
{
	iowrite32(0x1, (plat->ai2d_reg + 0xa0));
	iowrite32(0x0, (plat->ai2d_reg + 0xa4));
	iowrite32(0x0, (plat->ai2d_reg + 0xa8));
	iowrite32(0x0, (plat->ai2d_reg + 0xac));
}

static irqreturn_t ai2d_irq(int irq, void *dev_id)
{
	ai2d_interrupt_clear();
	ai2d_int_flag = 1;
	wake_up_interruptible(&ai2d_waitq);

	if (ai2d_fasync_flag)
		kill_fasync(&ai2d_fasync, SIGIO, POLL_IN);
	return IRQ_HANDLED;
}

static int ai2d_drv_fasync(int fd, struct file *file, int on)
{
	int err;

	err = fasync_helper(fd, file, on, &ai2d_fasync);
	if (err < 0)
		return err;

	ai2d_fasync_flag = on;

	return 0;
}

static unsigned int ai2d_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;

	poll_wait(file, &ai2d_waitq, wait);
	if (ai2d_int_flag) {
		ret |= POLLIN;
		ai2d_int_flag = 0;
	}

	return ret;
}

static int ai2d_open(struct inode *inode, struct file *filp)
{
	ai2d_int_flag = 0;
	return 0;
}

static int ai2d_release(struct inode *inode, struct file *filp)
{
	ai2d_int_flag = 0;
	return 0;
}

const struct file_operations ai2d_fops = {
	.owner = THIS_MODULE,
	.open = ai2d_open,
	.release = ai2d_release,
	.poll = ai2d_poll,
	.fasync = ai2d_drv_fasync,
};

static int ai2d_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err = 0;
	dev_t dev = 0;
	int devno;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("%s: get resource failed\n", __func__);
		err = -ENODEV;
		goto error;
	}

	plat = kzalloc(sizeof(struct ai2d_plat), GFP_KERNEL);
	if (!plat) {
		err = -ENOMEM;
		goto error;
	}

	plat->res = res;

	plat->irq = platform_get_irq(pdev, 0);
	if (plat->irq < 0) {
		pr_err("Gnne get irq err\n");
		err = -ENODEV;
		goto cleanup_kmem;
	}

	err = request_irq(plat->irq, ai2d_irq, 0, "t0", NULL);
	if (err) {
		pr_err("Gnne request irq err %d\n", err);
		goto cleanup_kmem;
	}

	plat->regs = ioremap(res->start, resource_size(res));
	if (!plat->regs) {
		pr_err("could not remap register memory\n");
		err = -ENOMEM;
		goto cleanup_irq;
	}
	plat->ai2d_reg = plat->regs;

	plat->major = 0;
	plat->minor = 0;

	err = alloc_chrdev_region(&dev, 0, 1, "k230-ai2d");
	if (err) {
		pr_err("k230-ai2d: can't get major %d\n",
			plat->major);
		goto cleanup_ioremap;
	}
	plat->major = MAJOR(dev);

	plat->class = class_create("k230_ai2d_class");
	if (IS_ERR(plat->class)) {
		err = PTR_ERR(plat->class);
		goto cleanup_ioremap;
	}

	devno = MKDEV(plat->major, plat->minor);

	cdev_init(&plat->cdev, &ai2d_fops);
	plat->cdev.owner = THIS_MODULE;
	err = cdev_add(&plat->cdev, devno, 1);
	if (err) {
		pr_err("Error %d adding ai2d device number %d\n", err, plat->minor);
		goto cleanup_class;
	}

	plat->device =
		device_create(plat->class, NULL, devno, NULL, "k230-ai2d");
	if (IS_ERR(plat->device)) {
		pr_err("device not created\n");
		err = PTR_ERR(plat->device);
		goto cleanup_cdev;
	}

	return 0;

cleanup_cdev:
	cdev_del(&plat->cdev);
cleanup_class:
	class_destroy(plat->class);
cleanup_ioremap:
	iounmap(plat->regs);
cleanup_irq:
	free_irq(plat->irq, NULL);
cleanup_kmem:
	kfree(plat);
error:
	return err;
}

static int ai2d_remove(struct platform_device *pdev)
{
	dev_t dev = MKDEV(plat->major, plat->minor);

	device_destroy(plat->class, dev);
	cdev_del(&plat->cdev);
	class_destroy(plat->class);
	iounmap(plat->regs);
	free_irq(plat->irq, NULL);
	kfree(plat);

	return 0;
}

static const struct of_device_id k230_ai2d_ids[] = { { .compatible =
							       "k230-ai2d" },
						     {} };

static struct platform_driver k230_ai2d_driver = {
	.probe          = ai2d_probe,
	.remove         = ai2d_remove,
	.driver         = {
		.name           = "k230-ai2d",
		.of_match_table = of_match_ptr(k230_ai2d_ids),
	},
};

int ai2d_module_init(void)
{
	int ret;

	ret = platform_driver_register(&k230_ai2d_driver);

	return ret;
}

void ai2d_module_deinit(void)
{
	platform_driver_unregister(&k230_ai2d_driver);
}

module_init(ai2d_module_init);
module_exit(ai2d_module_deinit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for k230 ai2d");
