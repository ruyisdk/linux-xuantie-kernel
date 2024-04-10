/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 Regents of the University of California
 */

#ifndef _ASM_RISCV_PROCESSOR_H
#define _ASM_RISCV_PROCESSOR_H

#include <linux/const.h>
#include <linux/cache.h>
#include <linux/prctl.h>

#include <vdso/processor.h>

#include <asm/ptrace.h>
#include <asm/hwcap.h>
#include <asm/hw_breakpoint.h>

/*
 * addr is a hint to the maximum userspace address that mmap should provide, so
 * this macro needs to return the largest address space available so that
 * mmap_end < addr, being mmap_end the top of that address space.
 * See Documentation/arch/riscv/vm-layout.rst for more details.
 */
#define arch_get_mmap_end(addr, len, flags)			\
({								\
	unsigned long mmap_end;					\
	typeof(addr) _addr = (addr);				\
	if ((_addr) == 0 || is_compat_task() ||			\
	    ((_addr + len) > BIT(VA_BITS - 1)))			\
		mmap_end = STACK_TOP_MAX;			\
	else							\
		mmap_end = (_addr + len);			\
	mmap_end;						\
})

#define arch_get_mmap_base(addr, base)				\
({								\
	unsigned long mmap_base;				\
	typeof(addr) _addr = (addr);				\
	typeof(base) _base = (base);				\
	unsigned long rnd_gap = DEFAULT_MAP_WINDOW - (_base);	\
	if ((_addr) == 0 || is_compat_task() || 		\
	    ((_addr + len) > BIT(VA_BITS - 1)))			\
		mmap_base = (_base);				\
	else							\
		mmap_base = (_addr + len) - rnd_gap;		\
	mmap_base;						\
})

#ifdef CONFIG_64BIT
#define DEFAULT_MAP_WINDOW	(UL(1) << (MMAP_VA_BITS - 1))
#define STACK_TOP_MAX		TASK_SIZE_64
#else
#define DEFAULT_MAP_WINDOW	TASK_SIZE
#define STACK_TOP_MAX		TASK_SIZE
#endif
#define STACK_ALIGN		16

#define STACK_TOP		DEFAULT_MAP_WINDOW

#ifdef CONFIG_MMU
#define user_max_virt_addr() arch_get_mmap_end(ULONG_MAX, 0, 0)
#else
#define user_max_virt_addr() 0
#endif /* CONFIG_MMU */

/*
 * This decides where the kernel will search for a free chunk of vm
 * space during mmap's.
 */
#ifdef CONFIG_64BIT
#define TASK_UNMAPPED_BASE	PAGE_ALIGN((UL(1) << MMAP_MIN_VA_BITS) / 3)
#else
#define TASK_UNMAPPED_BASE	PAGE_ALIGN(TASK_SIZE / 3)
#endif

#ifndef __ASSEMBLY__

struct task_struct;
struct perf_event;
struct pt_regs;

/*
 * We use a flag to track in-kernel Vector context. Currently the flag has the
 * following meaning:
 *
 *  - bit 0: indicates whether the in-kernel Vector context is active. The
 *    activation of this state disables the preemption.
 */
#define RISCV_KERNEL_MODE_V	0x1

/* CPU-specific state of a task */
struct thread_struct {
	/* Callee-saved registers */
	unsigned long ra;
	unsigned long sp;	/* Kernel mode stack */
	xlen_t     s[12];	/* s[0]: frame pointer */
	struct __riscv_d_ext_state fstate;
	unsigned long bad_cause;
	u32 riscv_v_flags;
	u32 vstate_ctrl;
	struct __riscv_v_ext_state vstate;
	struct __riscv_m_ext_state mstate;
#ifdef CONFIG_XUANTIE_CSR_EXT
	u32 fxcr;
	u32 utnmode;
#endif
#ifdef CONFIG_HAVE_HW_BREAKPOINT
	struct perf_event *ptrace_bps[HW_BP_NUM_MAX];
	struct arch_hw_breakpoint hbp_break[HW_BP_NUM_MAX];
	struct arch_hw_breakpoint hbp_watch[HW_BP_NUM_MAX];
#endif
#ifdef CONFIG_DYNAMIC_MEMORY_CONSISTENCY_MODEL
	unsigned long memory_consistency_model;
#endif
#ifdef CONFIG_RISCV_ISA_SSQOSID
	u32 sqoscfg;
#endif
} __attribute__((__aligned__(sizeof(xlen_t))));

/* Whitelist the fstate from the task_struct for hardened usercopy */
static inline void arch_thread_struct_whitelist(unsigned long *offset,
						unsigned long *size)
{
	*offset = offsetof(struct thread_struct, fstate);
	*size = sizeof_field(struct thread_struct, fstate);
}

#define INIT_THREAD {					\
	.sp = sizeof(init_stack) + (long)&init_stack,	\
}

#define task_pt_regs(tsk)						\
	((struct pt_regs *)(task_stack_page(tsk) + THREAD_SIZE		\
			    - ALIGN(sizeof(struct pt_regs), STACK_ALIGN)))

#define KSTK_EIP(tsk)		(ulong)(task_pt_regs(tsk)->epc)
#define KSTK_ESP(tsk)		(ulong)(task_pt_regs(tsk)->sp)


/* Do necessary setup to start up a newly executed thread. */
extern void start_thread(struct pt_regs *regs,
			unsigned long pc, unsigned long sp);

extern unsigned long __get_wchan(struct task_struct *p);


static inline void wait_for_interrupt(void)
{
	__asm__ __volatile__ ("wfi");
}

struct device_node;
int riscv_of_processor_hartid(struct device_node *node, unsigned long *hartid);
int riscv_early_of_processor_hartid(struct device_node *node, unsigned long *hartid);
int riscv_of_parent_hartid(struct device_node *node, unsigned long *hartid);

extern void riscv_fill_hwcap(void);
extern int arch_dup_task_struct(struct task_struct *dst, struct task_struct *src);

extern unsigned long signal_minsigstksz __ro_after_init;

#ifdef CONFIG_RISCV_ISA_V
/* Userspace interface for PR_RISCV_V_{SET,GET}_VS prctl()s: */
#define RISCV_V_SET_CONTROL(arg)	riscv_v_vstate_ctrl_set_current(arg)
#define RISCV_V_GET_CONTROL()		riscv_v_vstate_ctrl_get_current()
extern long riscv_v_vstate_ctrl_set_current(unsigned long arg);
extern long riscv_v_vstate_ctrl_get_current(void);
#endif /* CONFIG_RISCV_ISA_V */

#ifdef CONFIG_RISCV_ISA_POINTER_MASKING
/* PR_{SET,GET}_TAGGED_ADDR_CTRL prctl */
long set_tagged_addr_ctrl(struct task_struct *task, unsigned long arg);
long get_tagged_addr_ctrl(struct task_struct *task);
#define SET_TAGGED_ADDR_CTRL(arg)	set_tagged_addr_ctrl(current, arg)
#define GET_TAGGED_ADDR_CTRL()		get_tagged_addr_ctrl(current)
#endif

#ifdef CONFIG_RISCV_ISA_SSDTSO
extern int dtso_set_memory_consistency_model(unsigned long arg);
extern int dtso_get_memory_consistency_model(void);
#define SET_MEMORY_CONSISTENCY_MODEL(arg)	dtso_set_memory_consistency_model(arg)
#define GET_MEMORY_CONSISTENCY_MODEL()		dtso_get_memory_consistency_model()
#endif /* CONIG_RISCV_ISA_SSDTSO */

#endif /* __ASSEMBLY__ */

#endif /* _ASM_RISCV_PROCESSOR_H */
