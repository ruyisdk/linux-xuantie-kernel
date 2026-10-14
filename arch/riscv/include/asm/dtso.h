/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2023 Christoph Muellner <christoph.muellner@vrull.eu>
 */

#ifndef __ASM_RISCV_DTSO_H
#define __ASM_RISCV_DTSO_H

#define RISCV_MEMORY_CONSISTENCY_MODEL_WMO     0
#define RISCV_MEMORY_CONSISTENCY_MODEL_TSO     1

#ifdef CONFIG_RISCV_ISA_XUANTIE_SSDTSO

#include <linux/sched/task_stack.h>
#include <asm/cpufeature.h>
#include <asm/csr.h>
#include <asm/vendor_extensions.h>
#include <asm/vendor_extensions/thead.h>
#include <asm/vendorid_list.h>

static __always_inline bool has_dtso(void)
{
	return riscv_isa_vendor_extension_available(THEAD_VENDOR_ID,
						    XUANTIE_SSDTSO);
}

static __always_inline bool has_ztso(void)
{
	return riscv_has_extension_unlikely(RISCV_ISA_EXT_ZTSO);
}

static inline bool dtso_is_enabled(void)
{
	if (has_dtso())
		return csr_read(CSR_ENVCFG) & ENVCFG_DTSO;
	return 0;
}

static inline void dtso_disable(void)
{
	if (has_dtso() && !has_ztso())
		csr_clear(CSR_ENVCFG, ENVCFG_DTSO);
}

static inline void dtso_enable(void)
{
	if (has_dtso() && !has_ztso())
		csr_set(CSR_ENVCFG, ENVCFG_DTSO);
}

static inline unsigned long get_memory_consistency_model(
		struct task_struct *task)
{
	return task->thread.memory_consistency_model;
}

static inline void set_memory_consitency_model(struct task_struct *task,
		unsigned long model)
{
	task->thread.memory_consistency_model = model;
}

static inline void dtso_sched_in(struct task_struct *next)
{
	unsigned long next_model = get_memory_consistency_model(next);

	if (next_model == RISCV_MEMORY_CONSISTENCY_MODEL_TSO) {
		next->thread_info.envcfg |= ENVCFG_DTSO;
		dtso_enable();
	} else {
		next->thread_info.envcfg &= ~ENVCFG_DTSO;
		dtso_disable();
	}
}

#else /* ! CONFIG_RISCV_ISA_XUANTIE_SSDTSO */

static __always_inline bool has_dtso(void) { return false; }
static __always_inline bool dtso_is_enabled(void) { return false; }
#define dtso_disable() do { } while (0)
#define dtso_enable() do { } while (0)
#define dtso_sched_in(next) do { } while (0)

#endif /* CONFIG_RISCV_ISA_XUANTIE_SSDTSO */

#endif /* ! __ASM_RISCV_DTSO_H */
