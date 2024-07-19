// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>

#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/capability.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/backing-dev.h>
#include <linux/shmem_fs.h>
#include <linux/splice.h>
#include <linux/pfn.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/uio.h>
#include <linux/uaccess.h>
#include <linux/security.h>

static struct device *mmz_dev;
static DEFINE_MUTEX(mmz_mutex);

#define MMZ_ALLOC_MEM _IOWR('g', 1, unsigned long)
#define MMZ_FREE_MEM _IOWR('g', 2, unsigned long)
struct mmz_info {
	void *user_virt_addr;
	void *kernel_virt_addr;
	unsigned long mmz_phys;
	unsigned long length;
};

static long mmz_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct mmz_info t_mem_info;

	switch (cmd) {
	case MMZ_ALLOC_MEM:
		if (copy_from_user(&t_mem_info, (void *)arg,
				   sizeof(struct mmz_info)))
			return -EFAULT;
		t_mem_info.kernel_virt_addr = dma_alloc_coherent(
			mmz_dev, t_mem_info.length,
			(dma_addr_t *)(&(t_mem_info.mmz_phys)),
			GFP_KERNEL | GFP_DMA);
		if (copy_to_user((void *)arg, &t_mem_info, sizeof(struct mmz_info)))
			return -EFAULT;
		break;
	case MMZ_FREE_MEM:
		if (copy_from_user(&t_mem_info, (void *)arg,
				   sizeof(struct mmz_info)))
			return -EFAULT;
		dma_free_coherent(mmz_dev, t_mem_info.length,
				  t_mem_info.kernel_virt_addr,
				  t_mem_info.mmz_phys);
		break;
	default:
		pr_err("%s: Unknown ioctl: 0x%.8X\n", __func__, cmd);
		return -EINVAL;
	}
	return 0;
}

static long mmz_unlocked_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	long ret = 0;

	mutex_lock(&mmz_mutex);
	ret = mmz_ioctl(file, cmd, arg);
	mutex_unlock(&mmz_mutex);

	return ret;
}

#ifndef ARCH_HAS_VALID_PHYS_ADDR_RANGE
static inline int valid_phys_addr_range(phys_addr_t addr, size_t count)
{
	return addr + count <= __pa(high_memory);
}

static inline int valid_mmap_phys_addr_range(unsigned long pfn, size_t size)
{
	return 1;
}
#endif

#ifndef CONFIG_MMU
static unsigned long
get_unmapped_area_mem(struct file *file, unsigned long addr, unsigned long len,
		      unsigned long pgoff, unsigned long flags)
{
	if (!valid_mmap_phys_addr_range(pgoff, len))
		return (unsigned long)-EINVAL;
	return pgoff << PAGE_SHIFT;
}

/* permit direct mmap, for read, write or exec */
static unsigned int memory_mmap_capabilities(struct file *file)
{
	return NOMMU_MAP_DIRECT | NOMMU_MAP_READ | NOMMU_MAP_WRITE |
	       NOMMU_MAP_EXEC;
}

static unsigned int zero_mmap_capabilities(struct file *file)
{
	return NOMMU_MAP_COPY;
}

/* can't do an in-place private mapping if there's no MMU */
static inline int private_mapping_ok(struct vm_area_struct *vma)
{
	return is_nommu_shared_mapping(vma->vm_flags);
}
#else

static inline int private_mapping_ok(struct vm_area_struct *vma)
{
	return 1;
}
#endif

#ifdef CONFIG_STRICT_DEVMEM
static inline int page_is_allowed(unsigned long pfn)
{
	return devmem_is_allowed(pfn);
}
static inline int range_is_allowed(unsigned long pfn, unsigned long size)
{
	u64 from = ((u64)pfn) << PAGE_SHIFT;
	u64 to = from + size;
	u64 cursor = from;

	while (cursor < to) {
		if (!devmem_is_allowed(pfn))
			return 0;
		cursor += PAGE_SIZE;
		pfn++;
	}
	return 1;
}
#else
static inline int page_is_allowed(unsigned long pfn)
{
	return 1;
}
static inline int range_is_allowed(unsigned long pfn, unsigned long size)
{
	return 1;
}
#endif

int __weak phys_mem_access_prot_allowed(struct file *file, unsigned long pfn,
					unsigned long size, pgprot_t *vma_prot)
{
	return 1;
}

#ifdef pgprot_noncached
static int uncached_access(struct file *file, phys_addr_t addr)
{
#if defined(CONFIG_IA64)
	/*
	 * On ia64, we ignore O_DSYNC because we cannot tolerate memory
	 * attribute aliases.
	 */
	return !(efi_mem_attributes(addr) & EFI_MEMORY_WB);
#else
	/*
	 * Accessing memory above the top the kernel knows about or through a
	 * file pointer
	 * that was marked O_DSYNC will be done non-cached.
	 */
	if (file->f_flags & O_DSYNC)
		return 1;
	return addr >= __pa(high_memory);
#endif
}
#endif

static pgprot_t phys_mem_access_prot(struct file *file, unsigned long pfn,
				     unsigned long size, pgprot_t vma_prot)
{
#ifdef pgprot_noncached
	phys_addr_t offset = pfn << PAGE_SHIFT;

	if (uncached_access(file, offset))
		return pgprot_noncached(vma_prot);
#endif
	return vma_prot;
}

static const struct vm_operations_struct mmap_mem_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};

static int mmz_map(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;
	phys_addr_t offset = (phys_addr_t)vma->vm_pgoff << PAGE_SHIFT;

	/* Does it even fit in phys_addr_t? */
	if (offset >> PAGE_SHIFT != vma->vm_pgoff)
		return -EINVAL;

	/* It's illegal to wrap around the end of the physical address space. */
	if (offset + (phys_addr_t)size - 1 < offset)
		return -EINVAL;

	if (!valid_mmap_phys_addr_range(vma->vm_pgoff, size))
		return -EINVAL;

	if (!private_mapping_ok(vma))
		return -EINVAL;

	if (!range_is_allowed(vma->vm_pgoff, size))
		return -EPERM;

	if (!phys_mem_access_prot_allowed(file, vma->vm_pgoff, size,
					  &vma->vm_page_prot))
		return -EINVAL;

	if (offset < 0x80000000)
		vma->vm_page_prot.pgprot &= ~(1ULL << 63);
	else {
		vma->vm_page_prot = phys_mem_access_prot(
			file, vma->vm_pgoff, size, vma->vm_page_prot);
	}

	vma->vm_ops = &mmap_mem_ops;

	/* Remap-pfn-range will mark the range VM_IO */
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

static int mmap_mem(struct file *file, struct vm_area_struct *vma)
{
	int ret = 0;

	mutex_lock(&mmz_mutex);
	mmz_map(file, vma);
	mutex_unlock(&mmz_mutex);

	return ret;
}

static const struct file_operations mmz_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mmz_unlocked_ioctl,
	.mmap = mmap_mem,
};

static struct miscdevice mmz_misc = {
	.name = "mmz",
	.fops = &mmz_fops,
};

static int __init mmz_init(void)
{
	int ret = 0;

	ret = misc_register(&mmz_misc);
	if (unlikely(ret)) {
		pr_err("failed to register mmz test misc device!\n");
		return ret;
	}
	mmz_dev = mmz_misc.this_device;
	mmz_dev->coherent_dma_mask = ~0;
	_dev_info(mmz_dev, "registered.\n");

	return ret;
}
module_init(mmz_init);

static void __exit mmz_exit(void)
{
	misc_deregister(&mmz_misc);
}
module_exit(mmz_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for alloc continuous memory for user");
