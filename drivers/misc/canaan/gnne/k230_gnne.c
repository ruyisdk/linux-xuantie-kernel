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
#include <linux/dma-mapping.h>

#define gnne_writeq(v, addr)                       \
	{                                         \
		(*(uint64_t *)((size_t)(addr))) = (v); \
	}

struct gnne_plat {
	struct resource *res;
	void __iomem *regs;
	char *gnne_reg;
	int major;
	int minor;
	int irq;
	struct class *class;
	struct device *device;
	struct cdev cdev;
};

static struct gnne_plat *plat;
static unsigned int gnne_int_flag;
static int gnne_fasync_flag;
DECLARE_WAIT_QUEUE_HEAD(gnne_waitq);
struct fasync_struct *gnne_fasync;

static DEFINE_MUTEX(gnne_mutex);

static void gnne_interrupt_clear(void)
{
	gnne_writeq(0x400000004, (plat->gnne_reg + 0x128));
}

static irqreturn_t gnne_irq(int irq, void *dev_id)
{
	gnne_interrupt_clear();
	gnne_int_flag = 1;
	wake_up_interruptible(&gnne_waitq);

	if (gnne_fasync_flag)
		kill_fasync(&gnne_fasync, SIGIO, POLL_IN);
	return IRQ_HANDLED;
}

static int gnne_drv_fasync(int fd, struct file *file, int on)
{
	int err;

	err = fasync_helper(fd, file, on, &gnne_fasync);
	if (err < 0)
		return err;

	gnne_fasync_flag = on;

	return 0;
}

static unsigned int gnne_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;

	poll_wait(file, &gnne_waitq, wait);
	if (gnne_int_flag) {
		ret |= POLLIN;
		gnne_int_flag = 0;
	}

	return ret;
}

static int gnne_open(struct inode *inode, struct file *filp)
{
	void __iomem *sysctl_reg;

	gnne_int_flag = 0;

	sysctl_reg = ioremap(0x91103028, 4);
	if (!sysctl_reg) {
		pr_err("can't remap gnne sysctl 0x%08X\n", 0x91103028);
		return -1;
	}
	iowrite32(0x00030002, sysctl_reg);
	iounmap(sysctl_reg);

	sysctl_reg = ioremap(0x91100008, 4);
	if (!sysctl_reg) {
		pr_err("can't remap gnne sysctl 0x%08X\n", 0x91100008);
		return -1;
	}
	iowrite32(0x80000405, sysctl_reg);
	iowrite32(0x80000405, sysctl_reg);
	iounmap(sysctl_reg);

	return 0;
}

static int gnne_release(struct inode *inode, struct file *filp)
{
	void __iomem *sysctl_reg;

	gnne_int_flag = 0;

	sysctl_reg = ioremap(0x91103028, 4);
	if (!sysctl_reg) {
		pr_err("can't remap gnne sysctl 0x%08X\n", 0x91103028);
		return -1;
	}
	iowrite32(0x00030001, sysctl_reg);
	iounmap(sysctl_reg);

	sysctl_reg = ioremap(0x91100008, 4);
	if (!sysctl_reg) {
		pr_err("can't remap gnne sysctl 0x%08X\n", 0x91100008);
		return -1;
	}
	iowrite32(0x00000404, sysctl_reg);
	iounmap(sysctl_reg);

	return 0;
}

const struct file_operations gnne_fops = {
	.owner = THIS_MODULE,
	.open = gnne_open,
	.release = gnne_release,
	.poll = gnne_poll,
	.fasync = gnne_drv_fasync,
};

static int gnne_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err = 0;
	dev_t dev = 0;

	int devno;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("%s: get resource failed", __func__);
		err = -ENODEV;
		goto error;
	}

	plat = kzalloc(sizeof(struct gnne_plat), GFP_KERNEL);
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

	err = request_irq(plat->irq, gnne_irq, 0, "t0", NULL);
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
	plat->gnne_reg = plat->regs;

	plat->major = 0;
	plat->minor = 0;

	err = alloc_chrdev_region(&dev, 0, 1, "k230-gnne");
	if (err) {
		pr_err("k230-gnne: can't get major %d\n",
			plat->major);
		goto cleanup_ioremap;
	}
	plat->major = MAJOR(dev);

	plat->class = class_create("k230_gnne_class");
	if (IS_ERR(plat->class)) {
		err = PTR_ERR(plat->class);
		goto cleanup_ioremap;
	}

	devno = MKDEV(plat->major, plat->minor);

	cdev_init(&plat->cdev, &gnne_fops);
	plat->cdev.owner = THIS_MODULE;
	err = cdev_add(&plat->cdev, devno, 1);
	if (err) {
		pr_err("Error %d adding gnne device number %d\n", err, plat->minor);
		goto cleanup_class;
	}

	plat->device =
		device_create(plat->class, NULL, devno, NULL, "k230-gnne");
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

static int gnne_remove(struct platform_device *pdev)
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

static const struct of_device_id k230_gnne_ids[] = { { .compatible =
								   "k230-gnne" },
							 {} };

static struct platform_driver k230_gnne_driver = {
	.probe          = gnne_probe,
	.remove         = gnne_remove,
	.driver         = {
		.name           = "k230-gnne",
		.of_match_table = of_match_ptr(k230_gnne_ids),
	},
};

int gnne_module_init(void)
{
	int ret;

	ret = platform_driver_register(&k230_gnne_driver);
	return ret;
}

void gnne_module_deinit(void)
{
	platform_driver_unregister(&k230_gnne_driver);
}

module_init(gnne_module_init);
module_exit(gnne_module_deinit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for k230 gnne");
