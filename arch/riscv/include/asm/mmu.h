/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 Regents of the University of California
 */


#ifndef _ASM_RISCV_MMU_H
#define _ASM_RISCV_MMU_H

#ifndef __ASSEMBLY__

typedef struct {
#ifndef CONFIG_MMU
	unsigned long	end_brk;
#else
	atomic_long_t id;
#endif
	void *vdso;
#ifdef CONFIG_SMP
	/* A local icache flush is needed before user execution can resume. */
	cpumask_t icache_stale_mask;
#endif
#ifdef CONFIG_BINFMT_ELF_FDPIC
	unsigned long exec_fdpic_loadmap;
	unsigned long interp_fdpic_loadmap;
#endif
#ifdef CONFIG_RISCV_ISA_POINTER_MASKING
	unsigned long flags;
	u8 pmlen;
#endif
#ifdef CONFIG_DYNAMIC_MEMORY_CONSISTENCY_MODEL
	unsigned long memory_consistency_model;
#endif
} mm_context_t;

/* Lock the pointer masking mode because this mm is multithreaded */
#define MM_CONTEXT_LOCK_PMLEN	0

void __init create_pgd_mapping(pgd_t *pgdp, uintptr_t va, phys_addr_t pa,
			       phys_addr_t sz, pgprot_t prot);
#endif /* __ASSEMBLY__ */

#endif /* _ASM_RISCV_MMU_H */
