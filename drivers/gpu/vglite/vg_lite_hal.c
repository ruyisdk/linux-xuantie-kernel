// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Copyright (C) 2024, Canaan Bright Sight Co., Ltd
 * Copyright (c) 2014 - 2020 Vivante Corporation
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 */

#include "linux/device/bus.h"
#include "linux/dma-direction.h"
#include "linux/err.h"
#include "linux/kern_levels.h"
#include "linux/module.h"
#include "linux/power_supply.h"
#include "linux/printk.h"
#include "linux/scatterlist.h"
#include "linux/slab.h"
#include "linux/types.h"
#include "vg_lite_platform.h"
#include "vg_lite_kernel.h"
#include "vg_lite_hal.h"
#include "vg_lite_ioctl.h"
#include "vg_lite_hw.h"
#include "vg_lite_type.h"
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pagemap.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/mm_types.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>

MODULE_LICENSE("Dual MIT/GPL");
#ifdef MODULE_IMPORT_NS
MODULE_IMPORT_NS(DMA_BUF);
#endif

static int vg_lite_init(struct platform_device *pdev);
static int vg_lite_exit(struct platform_device *pdev);

#define VM_FLAGS (VM_IO | VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP)

#define current_mm_mmap_sem current->mm->mmap_lock

#define GET_PAGE_COUNT(size, p_size) (((size) + (p_size)-1) / p_size)

/* Struct definitions. */
struct dma_node {
	struct list_head list;
	dma_addr_t dma_addr;
	void *virt_addr;
	unsigned long size;
	struct vg_lite_kernel_map_memory map;
};

enum um_desc_type {
	UM_PAGE_MAP,
	UM_PFN_MAP,
};

struct mapped_memory {
	struct list_head list;
	u32 flags;

	union {
		struct {
			/* parse user dma_buf fd */
			/* Descriptor of a dma_buf imported. */
			struct dma_buf *dmabuf;
			struct sg_table *sgt;
			struct dma_buf_attachment *attachment;

			int npages;
			int pid;
			struct list_head list;
		} dmabuf_desc;
	};
};

struct vg_lite_device {
	void *gpu; /* Register memory base */
	struct page *pages;
	unsigned int order;
	void *virtual;
	u32 physical;
	u32 size;
	int irq_enabled;

	u32 int_flags;

	wait_queue_head_t int_queue;
	void *device;
	int registered;
	int major;
	struct class *class;
	struct device *dev;
	struct clk *clk;
	struct reset_control *reset;
	struct list_head dma_list_head;
	struct list_head mapped_list_head;
	int created;
};

struct client_data {
	struct vg_lite_device *device;
	struct vm_area_struct *vm;
	void *contiguous_mapped;
};

/* Data and objects declarations. */
static int cached = 1;

static struct vg_lite_device *device;
static struct client_data *private_data;

void vg_lite_hal_delay(u32 milliseconds)
{
	/* Delay the requested amount. */
	msleep(milliseconds);
}

void vg_lite_hal_barrier(void)
{
	// Make sure command buffer and pixels fresh
	smp_mb();
	if (cached)
		flush_cache_all();
}

void vg_lite_hal_initialize(void){};
void vg_lite_hal_deinitialize(void){};

void vg_lite_hal_open(void)
{
	// Power-on
	pm_runtime_get_sync(device->dev);
	// Reset
	if (!IS_ERR(device->reset))
		reset_control_reset(device->reset);
}

void vg_lite_hal_close(void)
{
	// Power-off
	pm_runtime_put_sync(device->dev);
}

#define VG_LITE_PAD(number, align_bytes) \
	((number) +                      \
	 (((align_bytes) - (number) % (align_bytes)) % (align_bytes)))

enum vg_lite_error vg_lite_hal_allocate_contiguous(unsigned long size,
						void **logical, void **klogical,
						u32 *physical, void **node)
{
	struct dma_node *n = kmalloc(sizeof(struct dma_node), GFP_KERNEL);

	if (!n)
		return VG_LITE_OUT_OF_MEMORY;
#define MANUAL_ALIGN 1
	/* FIXME: Align phy address to 64 bytes, manualy align trig tainted. */
#if MANUAL_ALIGN
	size = VG_LITE_ALIGN(size, 64);
#endif
	n->size = size;
	n->virt_addr = dma_alloc_coherent(device->dev, size, &n->dma_addr,
					  GFP_KERNEL | GFP_DMA);
	if (!n->virt_addr) {
		kfree(n);
		return VG_LITE_OUT_OF_MEMORY;
	}
	// map
	n->map.bytes = n->size;
	n->map.physical = n->dma_addr;
	if (vg_lite_hal_map_memory(&n->map) != VG_LITE_SUCCESS) {
		dma_free_coherent(device->dev, n->size, n->virt_addr,
				  n->dma_addr);
		kfree(n);
		return VG_LITE_OUT_OF_RESOURCES;
	}
	list_add(&n->list, &device->dma_list_head);
#if MANUAL_ALIGN
	*klogical = (u8 *)VG_LITE_PAD((size_t)n->virt_addr, 64);
	*logical = (u8 *)VG_LITE_PAD((size_t)n->map.logical, 64);
	*physical = VG_LITE_PAD(n->map.physical, 64);
#else
	*logical = n->map.logical;
	*physical = n->map.physical;
#endif
	*node = n;
	return VG_LITE_SUCCESS;
}

void vg_lite_hal_free_contiguous(void *memory_handle)
{
	struct dma_node *n = memory_handle;
	struct vg_lite_kernel_unmap_memory unmap = { .bytes = n->map.bytes,
						.logical = n->map.logical };
	vg_lite_hal_unmap_memory(&unmap);
	dma_free_coherent(device->dev, n->size, n->virt_addr, n->dma_addr);
	list_del(&n->list);
	kfree(n);
}

void vg_lite_hal_free_os_heap(void)
{
	struct dma_node *dn;
	struct dma_node *dn2;
	struct mapped_memory *mapped, *_mapped;

	list_for_each_entry_safe(dn, dn2, &device->dma_list_head, list) {
		vg_lite_hal_free_contiguous(dn);
	}
	list_for_each_entry_safe(mapped, _mapped, &device->mapped_list_head, list) {
		vg_lite_hal_unmap(mapped);
	}
}

u32 vg_lite_hal_peek(u32 address)
{
	/* Read data from the GPU register. */
	return readl(device->gpu + address);
}

void vg_lite_hal_poke(u32 address, u32 data)
{
	/* Write data to the GPU register. */
	writel(data, device->gpu + (u64)address);
}

enum vg_lite_error vg_lite_hal_query_mem(struct vg_lite_kernel_mem *mem)
{
	// FIXME: not impliment
	if (device) {
		mem->bytes = 0;
		return VG_LITE_SUCCESS;
	}
	mem->bytes = 0;
	return VG_LITE_NO_CONTEXT;
}

enum vg_lite_error vg_lite_hal_map_memory(struct vg_lite_kernel_map_memory *node)
{
	enum vg_lite_error error = VG_LITE_SUCCESS;
	void *logical = NULL;
	u64 physical = node->physical;
	u32 offset = physical & (PAGE_SIZE - 1);
	u64 bytes = node->bytes + offset;
	u32 num_pages, pfn = 0;
	struct vg_lite_kernel_unmap_memory unmap_node;
	struct vm_area_struct *vma;

	logical = (void *)vm_mmap(NULL, 0L, bytes, PROT_READ | PROT_WRITE,
				  MAP_SHARED | MAP_NORESERVE, 0);

	if (!logical) {
		node->logical = NULL;
		return VG_LITE_OUT_OF_MEMORY;
	}

	down_write(&current_mm_mmap_sem);

	vma = find_vma(current->mm, (unsigned long)logical);

	if (!vma)
		return VG_LITE_OUT_OF_RESOURCES;

	pfn = (physical >> PAGE_SHIFT);
	num_pages = GET_PAGE_COUNT(bytes, PAGE_SIZE);

	/* Make this mapping cached / non-cached. */
	if (!cached)
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, pfn, num_pages << PAGE_SHIFT,
			    vma->vm_page_prot) < 0)
		error = VG_LITE_OUT_OF_MEMORY;

	node->logical = (void *)((uint8_t *)logical + offset);

	up_write(&current_mm_mmap_sem);

	if (error) {
		unmap_node.bytes = node->bytes;
		unmap_node.logical = node->logical;
		vg_lite_hal_unmap_memory(&unmap_node);
	}

	return error;
}

enum vg_lite_error vg_lite_hal_unmap_memory(struct vg_lite_kernel_unmap_memory *node)
{
	enum vg_lite_error error = VG_LITE_SUCCESS;
	void *_logical;
	u32 bytes;
	u32 offset = (u64)node->logical & (PAGE_SIZE - 1);

	if (unlikely(!current->mm))
		return error;

	_logical = (void *)((uint8_t *)node->logical - offset);
	bytes = GET_PAGE_COUNT(node->bytes + offset, PAGE_SIZE) * PAGE_SIZE;

	if (vm_munmap((unsigned long)_logical, bytes) < 0) {
		error = VG_LITE_INVALID_ARGUMENT;
		dev_warn(device->dev, "vm_munmap error\n");
	}

	return error;
}

int vg_lite_hal_wait_interrupt(u32 timeout, u32 mask, u32 *value)
{
	// FIXME: struct timeval tv;
	unsigned long jiffies;
	unsigned long result;
#define IGNORE_INTERRUPT 1
#if IGNORE_INTERRUPT
	unsigned int int_flag;
	u32 idle = 0x7fffffff;
#endif

	/* Convert timeval to jiffies. */
	if (timeout == VG_LITE_INFINITE) {
		/* Set 1 second timeout. */
		jiffies = msecs_to_jiffies(1000);
	} else {
		/* Convert timeout in ms to timeval. */
		jiffies = msecs_to_jiffies(timeout);
	}

	/* Wait for interrupt, ignoring timeout. */
	do {
		result = wait_event_interruptible_timeout(
			device->int_queue, device->int_flags & mask, jiffies);
#if IGNORE_INTERRUPT
		/* Wait until GPU is idle */
		int_flag = vg_lite_hal_peek(0x10);
		idle = vg_lite_hal_peek(0x04);
		if (int_flag) {
			result = int_flag;
			dev_vdbg(device->dev,
				"vg_lite: waiting... idle: 0x%08X, int: 0x%08X, FE: 0x%08X 0x%08X 0x%08X\n",
				idle, int_flag, vg_lite_hal_peek(0x500),
				vg_lite_hal_peek(0x504), vg_lite_hal_peek(0x508)
			);
		}
#endif
	} while ((timeout == VG_LITE_INFINITE && result == 0) || (idle != 0x7fffffff));

	/* Report the event(s) got. */
	if (value)
		*value = device->int_flags & mask;

	device->int_flags = 0;
	return (result != 0);
}

enum vg_lite_error vg_lite_hal_operation_cache(void *handle,
					    enum vg_lite_cache_op cache_op)
{
	return VG_LITE_SUCCESS;
}

enum vg_lite_error vg_lite_hal_memory_export(int32_t *fd)
{
	// TODO
	return VG_LITE_NOT_SUPPORT;
}

void *vg_lite_hal_map(uint32_t flags, uint32_t bytes, void *logical,
		      uint32_t physical, int32_t dma_buf_fd, uint32_t *gpu)
{
	struct mapped_memory *mapped;

	mapped = kmalloc(sizeof(struct mapped_memory), GFP_KERNEL);
	if (mapped == NULL)
		return NULL;
	memset(mapped, 0, sizeof(struct mapped_memory));
	mapped->flags = flags;

	if (flags == VG_LITE_HAL_MAP_DMABUF) {
		struct scatterlist *sg;
		unsigned int i;

		mapped->dmabuf_desc.dmabuf = dma_buf_get(dma_buf_fd);
		if (IS_ERR(mapped->dmabuf_desc.dmabuf))
			goto error;
		mapped->dmabuf_desc.attachment =
			dma_buf_attach(mapped->dmabuf_desc.dmabuf, device->dev);
		if (IS_ERR(mapped->dmabuf_desc.attachment)) {
			dma_buf_put(mapped->dmabuf_desc.dmabuf);
			goto error;
		}
		mapped->dmabuf_desc.sgt = dma_buf_map_attachment(
			mapped->dmabuf_desc.attachment, DMA_BIDIRECTIONAL);
		if (IS_ERR(mapped->dmabuf_desc.sgt)) {
			dma_buf_detach(mapped->dmabuf_desc.dmabuf,
				       mapped->dmabuf_desc.attachment);
			dma_buf_put(mapped->dmabuf_desc.dmabuf);
			goto error;
		}
		for_each_sg(mapped->dmabuf_desc.sgt->sgl, sg,
			     mapped->dmabuf_desc.sgt->orig_nents, i) {
			*gpu = sg_dma_address(sg);
		}
	} else {
		dev_err(device->dev, "this map type not support!\n");
		return NULL;
	}

	list_add(&mapped->list, &device->mapped_list_head);
	return mapped;

error:
	kfree(mapped);
	return NULL;
}

void vg_lite_hal_unmap(void *handle)
{
	struct mapped_memory *mapped = handle;

	if (mapped->flags == VG_LITE_HAL_MAP_DMABUF) {
		dma_buf_unmap_attachment(mapped->dmabuf_desc.attachment,
					 mapped->dmabuf_desc.sgt,
					 DMA_BIDIRECTIONAL);

		dma_buf_detach(mapped->dmabuf_desc.dmabuf,
			       mapped->dmabuf_desc.attachment);

		dma_buf_put(mapped->dmabuf_desc.dmabuf);
	} else {
		dev_err(device->dev, "this map type not support!\n");
	}

	list_del(&mapped->list);
	kfree(mapped);
}

int drv_open(struct inode *inode, struct file *file)
{
	struct client_data *data;

	vg_lite_hal_open();
	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(device->dev, "kmalloc error\n");
		return -1;
	}
	data->device = device;
	data->contiguous_mapped = NULL;
	file->private_data = data;
	return 0;
}

// FIXME
int drv_release(struct inode *inode, struct file *file)
{
	struct client_data *data = (struct client_data *)file->private_data;

	vg_lite_hal_free_os_heap();
	kfree(data);
	file->private_data = NULL;
	vg_lite_hal_close();

	return 0;
}

#ifdef HAVE_UNLOCKED_IOCTL
long drv_ioctl(struct file *file, unsigned int ioctl_code, unsigned long arg)
#else
static const char * const vg_lite_command_string[] = {
	"VG_LITE_INITIALIZE",
	"VG_LITE_TERMINATE",
	"VG_LITE_ALLOCATE",
	"VG_LITE_FREE",
	"VG_LITE_SUBMIT",
	"VG_LITE_WAIT",
	"VG_LITE_RESET",
	"VG_LITE_DEBUG",
	"VG_LITE_MAP",
	"VG_LITE_UNMAP",
	"VG_LITE_CHECK",
	"VG_LITE_QUERY_MEM",
	"VG_LITE_FLEXA_DISABLE",
	"VG_LITE_FLEXA_ENABLE",
	"VG_LITE_FLEXA_STOP_FRAME",
	"VG_LITE_FLEXA_SET_BACKGROUND_ADDRESS",
	"VG_LITE_MAP_MEMORY",
	"VG_LITE_UNMAP_MEMORY",
	"VG_LITE_BUFFER_FROM_DMA_BUF"

};

long drv_ioctl(/*struct inode *inode, */ struct file *file,
	       unsigned int ioctl_code, unsigned long arg)
#endif
{
	struct ioctl_data arguments;
	void *data;

#ifndef HAVE_UNLOCKED_IOCTL
	/* inode will be not used */
	//(void)inode;
#endif
	private_data = (struct client_data *)file->private_data;
	if (!private_data)
		return -1;

	if (ioctl_code != VG_LITE_IOCTL)
		return -1;

	// FIXME: if ((void *)!arg)
	if (!arg)
		return -1;

	if (copy_from_user(&arguments, (void *)arg, sizeof(arguments)) != 0)
		return -1;

	data = kmalloc(arguments.bytes, GFP_KERNEL);
	if (!data)
		return -1;

	if (copy_from_user(data, arguments.buffer, arguments.bytes) != 0)
		goto error;

	if (arguments.command < sizeof(vg_lite_command_string) / 8) {
		dev_vdbg(device->dev, "ioctl %s\n",
			 vg_lite_command_string[arguments.command]);
	} else {
		dev_err(device->dev, "ioctl unknown command\n");
	}
	arguments.error = vg_lite_kernel(arguments.command, data);

	if (copy_to_user(arguments.buffer, data, arguments.bytes) != 0)
		goto error;

	kfree(data);

	if (copy_to_user((void *)arg, &arguments, sizeof(arguments)) != 0)
		return -1;

	return 0;

error:
	kfree(data);
	return -1;
}

ssize_t drv_read(struct file *file, char *buffer, size_t length, loff_t *offset)
{
	struct client_data *private = (struct client_data *)file->private_data;

	if (length != 4)
		return 0;

	if (copy_to_user((void __user *)buffer,
			 (const void *)&private->device->size,
			 sizeof(private->device->size)) != 0)
		return 0;

	memcpy(buffer, &private->device->size, 4);
	return 4;
}

int drv_mmap(struct file *file, struct vm_area_struct *vm)
{
	unsigned long size;
	struct client_data *private = (struct client_data *)file->private_data;

	return 0;

	if (!cached)
		vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);

	vm->vm_pgoff = 0;

	size = vm->vm_end - vm->vm_start;
	if (size > private->device->size)
		size = private->device->size;

	if (remap_pfn_range(vm, vm->vm_start,
			    private->device->physical >> PAGE_SHIFT, size,
			    vm->vm_page_prot) < 0) {
		dev_err(device->dev, "remap_pfn_range error\n");
		return -1;
	}

	private->vm = vm;
	private->contiguous_mapped = (void *)vm->vm_start;

	dev_info(device->dev, "mapped %scached contiguous memory to %p\n",
		 cached ? "" : "non-", private->contiguous_mapped);

	return 0;
}

static const struct file_operations file_operations = {
	.owner = THIS_MODULE,
	.open = drv_open,
	.release = drv_release,
	.read = drv_read,
	//#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = drv_ioctl,
	//#endif
	//#ifdef HAVE_COMPAT_IOCTL
	.compat_ioctl = drv_ioctl,
	//#endif
	.mmap = drv_mmap,
};

static int vg_lite_exit(struct platform_device *pdev)
{
	/* Check for valid device. */
	if (device) {
		if (device->gpu) {
			/* Unmap the GPU registers. */
			iounmap(device->gpu);
			device->gpu = NULL;
		}

		if (device->pages)
			/* Free the contiguous memory. */
			__free_pages(device->pages, device->order);

		if (device->irq_enabled)
			/* Free the IRQ. */
			free_irq(platform_get_irq(pdev, 0) /*GPU_IRQ*/, device);

		vg_lite_hal_free_os_heap();

		if (device->created)
			/* Destroy the device. */
			device_destroy(device->class, MKDEV(device->major, 0));

		if (device->class)
			/* Destroy the class. */
			class_destroy(device->class);

		if (device->registered)
			/* Unregister the device. */
			unregister_chrdev(device->major, "vg_lite");

		/* Free up the device structure. */
		kfree(device);
	}
	put_device(&pdev->dev);
	return 0;
}

static irqreturn_t irq_handler(int irq, void *context)
{
	struct vg_lite_device *device = context;

	/* Read interrupt status. */
	u32 flags = vg_lite_hal_peek(VG_LITE_INTR_STATUS);

	if (flags) {
		/* Combine with current interrupt flags. */
		device->int_flags |= flags;

		/* Wake up any waiters. */
		wake_up_interruptible(&device->int_queue);

		/* We handled the IRQ. */
		return IRQ_HANDLED;
	}

	/* Not our IRQ. */
	return IRQ_NONE;
}

static int vg_lite_init(struct platform_device *pdev)
{
	struct resource *mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	int irq_line = platform_get_irq(pdev, 0);
	int error = 0;
	struct device *dev;

	/* Create device structure. */
	device = kmalloc(sizeof(*device), GFP_KERNEL);
	memset(device, 0, sizeof(struct vg_lite_device));
	get_device(&pdev->dev);
	device->dev = &pdev->dev;
	device->clk = devm_clk_get(&pdev->dev, "vglite");
	device->reset = devm_reset_control_get(&pdev->dev, NULL);
	pm_runtime_enable(device->dev);

	/* Map the GPU registers. */
	device->gpu = ioremap(mem->start, resource_size(mem));
	if (!device->gpu) {
		dev_err(device->dev, "ioremap error\n");
		kfree(device);
		return -1;
	}

	/* Initialize the wait queue. */
	init_waitqueue_head(&device->int_queue);

	/* Install IRQ. */
	if (irq_line < 0) {
		dev_err(device->dev, "platform_get_irq error\n");
		vg_lite_exit(pdev);
		return -1;
	}
	error = request_irq(irq_line /*GPU_IRQ*/, irq_handler, 0, "vg_lite_irq",
			    device);
	if (error) {
		dev_err(device->dev, "request_irq error\n");
		vg_lite_exit(pdev);
		return -1;
	}
	device->irq_enabled = 1;

	/* Register device. */
	device->major = register_chrdev(0, "vg_lite", &file_operations);
	if (device->major < 0) {
		dev_err(device->dev, "register_chrdev error\n");
		vg_lite_exit(pdev);
		return -1;
	}
	device->registered = 1;

	/* Create the graphics class. */
	device->class = class_create("vg_lite_class");
	if (!device->class) {
		dev_err(device->dev, "class_create error\n");
		vg_lite_exit(pdev);
		return -1;
	}

	/* Create the device. */
	dev = device_create(device->class, NULL, MKDEV(device->major, 0), NULL,
			    "vg_lite");
	if (!dev) {
		dev_err(device->dev, "device_create error\n");
		vg_lite_exit(pdev);
		return -1;
	}
	device->created = 1;

	INIT_LIST_HEAD(&device->dma_list_head);
	INIT_LIST_HEAD(&device->mapped_list_head);
	if (dma_set_mask_and_coherent(device->dev, DMA_BIT_MASK(32))) {
		dev_err(device->dev, "dma_set_mask_and_coherent error\n");
		vg_lite_exit(pdev);
		return -1;
	}

	/* Success. */
	return 0;
}

static const struct of_device_id gc8000ul_of_match[] = {
	{ .compatible = "verisilicon,gc8000ul" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, gc8000ul_of_match);

module_param(cached, int, 0600);

static struct platform_driver gc8000ul_platform_driver = {
	.driver = { .name = "gc8000ul", .of_match_table = gc8000ul_of_match },
	.probe = vg_lite_init,
	.remove = vg_lite_exit,
};
module_platform_driver(gc8000ul_platform_driver);
