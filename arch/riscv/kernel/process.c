// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2009 Sunplus Core Technology Co., Ltd.
 *  Chen Liqin <liqin.chen@sunplusct.com>
 *  Lennox Wu <lennox.wu@sunplusct.com>
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 */

#include <linux/bitfield.h>
#include <linux/cpu.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/sched/task_stack.h>
#include <linux/tick.h>
#include <linux/ptrace.h>
#include <linux/uaccess.h>

#include <asm/unistd.h>
#include <asm/processor.h>
#include <asm/csr.h>
#include <asm/stacktrace.h>
#include <asm/string.h>
#include <asm/switch_to.h>
#include <asm/thread_info.h>
#include <asm/cpuidle.h>
#include <asm/vector.h>
#include <asm/usercfi.h>

#if defined(CONFIG_STACKPROTECTOR) && !defined(CONFIG_STACKPROTECTOR_PER_TASK)
#include <linux/stackprotector.h>
unsigned long __stack_chk_guard __read_mostly;
EXPORT_SYMBOL(__stack_chk_guard);
#endif

extern asmlinkage void ret_from_fork(void);

void arch_cpu_idle(void)
{
	cpu_do_idle();
}

void __show_regs(struct pt_regs *regs)
{
	show_regs_print_info(KERN_DEFAULT);

	if (!user_mode(regs)) {
		pr_cont("epc : %pS\n", (void *)(ulong)regs->epc);
		pr_cont(" ra : %pS\n", (void *)(ulong)regs->ra);
	}

	pr_cont("epc : " REG_FMT " ra : " REG_FMT " sp : " REG_FMT "\n",
		regs->epc, regs->ra, regs->sp);
	pr_cont(" gp : " REG_FMT " tp : " REG_FMT " t0 : " REG_FMT "\n",
		regs->gp, regs->tp, regs->t0);
	pr_cont(" t1 : " REG_FMT " t2 : " REG_FMT " s0 : " REG_FMT "\n",
		regs->t1, regs->t2, regs->s0);
	pr_cont(" s1 : " REG_FMT " a0 : " REG_FMT " a1 : " REG_FMT "\n",
		regs->s1, regs->a0, regs->a1);
	pr_cont(" a2 : " REG_FMT " a3 : " REG_FMT " a4 : " REG_FMT "\n",
		regs->a2, regs->a3, regs->a4);
	pr_cont(" a5 : " REG_FMT " a6 : " REG_FMT " a7 : " REG_FMT "\n",
		regs->a5, regs->a6, regs->a7);
	pr_cont(" s2 : " REG_FMT " s3 : " REG_FMT " s4 : " REG_FMT "\n",
		regs->s2, regs->s3, regs->s4);
	pr_cont(" s5 : " REG_FMT " s6 : " REG_FMT " s7 : " REG_FMT "\n",
		regs->s5, regs->s6, regs->s7);
	pr_cont(" s8 : " REG_FMT " s9 : " REG_FMT " s10: " REG_FMT "\n",
		regs->s8, regs->s9, regs->s10);
	pr_cont(" s11: " REG_FMT " t3 : " REG_FMT " t4 : " REG_FMT "\n",
		regs->s11, regs->t3, regs->t4);
	pr_cont(" t5 : " REG_FMT " t6 : " REG_FMT " ssp : " REG_FMT "\n",
		regs->t5, regs->t6, (xlen_t)get_active_shstk(current));

	pr_cont("status: " REG_FMT " badaddr: " REG_FMT " cause: " REG_FMT "\n",
		regs->status, regs->badaddr, regs->cause);
}
void show_regs(struct pt_regs *regs)
{
	__show_regs(regs);
	if (!user_mode(regs))
		dump_backtrace(regs, NULL, KERN_DEFAULT);
}

#ifdef CONFIG_COMPAT
static bool compat_mode_supported __read_mostly;

bool compat_elf_check_arch(Elf32_Ehdr *hdr)
{
	return (compat_mode_supported || (hdr->e_flags & EF_RISCV_64ILP32)) &&
	       hdr->e_machine == EM_RISCV &&
	       hdr->e_ident[EI_CLASS] == ELFCLASS32;
}

static int __init compat_mode_detect(void)
{
	unsigned long tmp = csr_read(CSR_STATUS);

	csr_write(CSR_STATUS, (tmp & ~SR_UXL) | SR_UXL_32);
	compat_mode_supported =
			(csr_read(CSR_STATUS) & SR_UXL) == SR_UXL_32;

	csr_write(CSR_STATUS, tmp);

	pr_info("riscv: ELF compat mode %s",
			compat_mode_supported ? "supported" : "unsupported");

	return 0;
}
early_initcall(compat_mode_detect);
#endif

void start_thread(struct pt_regs *regs, unsigned long pc,
	unsigned long sp)
{
	regs->status = SR_PIE;
	if (has_fpu()) {
		regs->status |= SR_FS_INITIAL;
		/*
		 * Restore the initial value to the FP register
		 * before starting the user program.
		 */
		fstate_restore(current, regs);
	}
	regs->epc = pc;
	regs->sp = sp;

	/*
	 * clear shadow stack state on exec.
	 * libc will set it later via prctl.
	 */
	set_shstk_status(current, false);
	set_shstk_base(current, 0, 0);
	set_active_shstk(current, 0);
	/*
	 * disable indirect branch tracking on exec.
	 * libc will enable it later via prctl.
	 */
	set_indir_lp_status(current, false);

#ifndef CONFIG_ARCH_RV32I
	regs->status &= ~SR_UXL;

	if (test_thread_flag(TIF_32BIT) && !test_thread_flag(TIF_64ILP32))
		regs->status |= SR_UXL_32;
	else
		regs->status |= SR_UXL_64;
#endif
}

void flush_thread(void)
{
#ifdef CONFIG_FPU
	/*
	 * Reset FPU state and context
	 *	frm: round to nearest, ties to even (IEEE default)
	 *	fflags: accrued exceptions cleared
	 */
	fstate_off(current, task_pt_regs(current));
	memset(&current->thread.fstate, 0, sizeof(current->thread.fstate));
#endif
#ifdef CONFIG_RISCV_ISA_V
	/* Reset vector state */
	riscv_v_vstate_ctrl_init(current);
	riscv_v_vstate_off(task_pt_regs(current));
	kfree(current->thread.vstate.datap);
	memset(&current->thread.vstate, 0, sizeof(struct __riscv_v_ext_state));
#endif
#ifdef CONFIG_RISCV_ISA_POINTER_MASKING
	if (riscv_has_extension_unlikely(RISCV_ISA_EXT_SUPM)) {
		envcfg_update_bits(current, ENVCFG_PMM, ENVCFG_PMM_PMLEN_0);
		current->thread_info.pmlen = 0;
	}
#endif
}

void arch_release_task_struct(struct task_struct *tsk)
{
	/* Free the vector context of datap. */
	if (has_vector())
		kfree(tsk->thread.vstate.datap);
}

int arch_dup_task_struct(struct task_struct *dst, struct task_struct *src)
{
	fstate_save(src, task_pt_regs(src));
	*dst = *src;
	/* clear entire V context, including datap for a new task */
	memset(&dst->thread.vstate, 0, sizeof(struct __riscv_v_ext_state));

	return 0;
}

void exit_thread(struct task_struct *tsk)
{
	if (IS_ENABLED(CONFIG_RISCV_USER_CFI))
		shstk_release(tsk);
}

int copy_thread(struct task_struct *p, const struct kernel_clone_args *args)
{
	unsigned long clone_flags = args->flags;
	unsigned long usp = args->stack;
	unsigned long tls = args->tls;
	struct pt_regs *childregs = task_pt_regs(p);

#ifdef CONFIG_RISCV_ISA_POINTER_MASKING
	/* Ensure all threads in this mm have the same pointer masking mode. */
	if (p->mm && (clone_flags & CLONE_VM))
		set_bit(MM_CONTEXT_LOCK_PMLEN, &p->mm->context.flags);
#endif

	memset(&p->thread.s, 0, sizeof(p->thread.s));

	/* p->thread holds context to be restored by __switch_to() */
	if (unlikely(args->fn)) {
		/* Kernel thread */
		memset(childregs, 0, sizeof(struct pt_regs));
		/* Supervisor/Machine, irqs on: */
		childregs->status = SR_PP | SR_PIE;

		p->thread.s[0] = (unsigned long)args->fn;
		p->thread.s[1] = (unsigned long)args->fn_arg;
	} else {
		*childregs = *(current_pt_regs());
		/* Turn off status.VS */
		riscv_v_vstate_off(childregs);
		if (usp) /* User fork */
			childregs->sp = usp;
		if (clone_flags & CLONE_SETTLS)
			childregs->tp = tls;
		childregs->a0 = 0; /* Return value of fork() */
		p->thread.s[0] = 0;
	}
	p->thread.riscv_v_flags = 0;
	p->thread.ra = (unsigned long)ret_from_fork;
	p->thread.sp = (unsigned long)childregs; /* kernel sp */
	return 0;
}

bool force_task_size_32 = false;
EXPORT_SYMBOL(force_task_size_32);

static int __init force_task_2gb(char *p)
{
	force_task_size_32 = true;
	pr_info("Force u64lp64 task size into 2GB.\n");
	return 0;
}
early_param("force_task_2gb", force_task_2gb);

#ifdef CONFIG_RISCV_ISA_POINTER_MASKING
static bool have_user_pmlen_7;
static bool have_user_pmlen_16;

/*
 * Control the relaxed ABI allowing tagged user addresses into the kernel.
 */
static unsigned int tagged_addr_disabled;

long set_tagged_addr_ctrl(struct task_struct *task, unsigned long arg)
{
	unsigned long valid_mask = PR_PMLEN_MASK | PR_TAGGED_ADDR_ENABLE;
	struct thread_info *ti = task_thread_info(task);
	struct mm_struct *mm = task->mm;
	unsigned long pmm;
	u8 pmlen;

	if (test_ti_thread_flag(ti, TIF_32BIT))
		return -EINVAL;

	if (arg & ~valid_mask)
		return -EINVAL;

	pmlen = FIELD_GET(PR_PMLEN_MASK, arg);
	if (pmlen > 16) {
		return -EINVAL;
	} else if (pmlen > 7) {
		if (have_user_pmlen_16)
			pmlen = 16;
		else
			return -EINVAL;
	} else if (pmlen > 0) {
		/*
		 * Prefer the smallest PMLEN that satisfies the user's request,
		 * in case choosing a larger PMLEN has a performance impact.
		 */
		if (have_user_pmlen_7)
			pmlen = 7;
		else if (have_user_pmlen_16)
			pmlen = 16;
		else
			return -EINVAL;
	}

	/*
	 * Do not allow the enabling of the tagged address ABI if globally
	 * disabled via sysctl abi.tagged_addr_disabled, if pointer masking
	 * is disabled for userspace.
	 */
	if (arg & PR_TAGGED_ADDR_ENABLE && (tagged_addr_disabled || !pmlen))
		return -EINVAL;

	if (pmlen == 7)
		pmm = ENVCFG_PMM_PMLEN_7;
	else if (pmlen == 16)
		pmm = ENVCFG_PMM_PMLEN_16;
	else
		pmm = ENVCFG_PMM_PMLEN_0;

	if (!(arg & PR_TAGGED_ADDR_ENABLE))
		pmlen = 0;

	if (mmap_write_lock_killable(mm))
		return -EINTR;

	if (test_bit(MM_CONTEXT_LOCK_PMLEN, &mm->context.flags) && mm->context.pmlen != pmlen) {
		mmap_write_unlock(mm);
		return -EBUSY;
	}

	envcfg_update_bits(task, ENVCFG_PMM, pmm);
	task->mm->context.pmlen = pmlen;
	task->thread_info.pmlen = pmlen;

	mmap_write_unlock(mm);

	return 0;
}

long get_tagged_addr_ctrl(struct task_struct *task)
{
	struct thread_info *ti = task_thread_info(task);
	long ret = 0;

	if (test_ti_thread_flag(ti, TIF_32BIT))
		return -EINVAL;

	if (task->thread_info.pmlen)
		ret = PR_TAGGED_ADDR_ENABLE;

	/*
	 * The task's pmlen is only set if the tagged address ABI is enabled,
	 * so the effective PMLEN must be extracted from envcfg.PMM.
	 */
	switch (task->thread_info.envcfg & ENVCFG_PMM) {
	case ENVCFG_PMM_PMLEN_7:
		ret |= FIELD_PREP(PR_PMLEN_MASK, 7);
		break;
	case ENVCFG_PMM_PMLEN_16:
		ret |= FIELD_PREP(PR_PMLEN_MASK, 16);
		break;
	}

	return ret;
}

static bool try_to_set_pmm(unsigned long value)
{
	csr_set(CSR_ENVCFG, value);
	return (csr_read_clear(CSR_ENVCFG, ENVCFG_PMM) & ENVCFG_PMM) == value;
}

/*
 * Global sysctl to disable the tagged user addresses support. This control
 * only prevents the tagged address ABI enabling via prctl() and does not
 * disable it for tasks that already opted in to the relaxed ABI.
 */

static struct ctl_table tagged_addr_sysctl_table[] = {
	{
		.procname	= "tagged_addr_disabled",
		.mode		= 0644,
		.data		= &tagged_addr_disabled,
		.maxlen		= sizeof(int),
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= SYSCTL_ZERO,
		.extra2		= SYSCTL_ONE,
	},
};

static int __init tagged_addr_init(void)
{
	if (!riscv_has_extension_unlikely(RISCV_ISA_EXT_SUPM))
		return 0;

	/*
	 * envcfg.PMM is a WARL field. Detect which values are supported.
	 * Assume the supported PMLEN values are the same on all harts.
	 */
	csr_clear(CSR_ENVCFG, ENVCFG_PMM);
	have_user_pmlen_7 = try_to_set_pmm(ENVCFG_PMM_PMLEN_7);
	have_user_pmlen_16 = try_to_set_pmm(ENVCFG_PMM_PMLEN_16);

	if (!register_sysctl("abi", tagged_addr_sysctl_table))
		return -EINVAL;

	return 0;
}
core_initcall(tagged_addr_init);
#endif	/* CONFIG_RISCV_ISA_POINTER_MASKING */
