/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_XUANTIE_CSR_EXT_H
#define _ASM_XUANTIE_CSR_EXT_H

#ifdef CONFIG_XUANTIE_CSR_EXT

#include <asm/csr.h>
#include <asm/hwcap.h>
#include <linux/sched.h>

static __always_inline bool has_xuantie_csr_ext(void)
{
	return riscv_has_extension_unlikely(RISCV_ISA_EXT_XTHEADFXCR) |
		riscv_has_extension_unlikely(RISCV_ISA_EXT_XTHEADUTNMODE);
}

static inline void __switch_to_xuantie_csr_ext(struct task_struct *prev,
				      struct task_struct *next)
{
	if (riscv_has_extension_likely(RISCV_ISA_EXT_XTHEADFXCR)) {
		csr_set(CSR_STATUS, SR_FS);
		prev->thread.fxcr = csr_read(CSR_FXCR);
		csr_write(CSR_FXCR, next->thread.fxcr);
		csr_clear(CSR_STATUS, SR_FS);
	}

	if (riscv_has_extension_likely(RISCV_ISA_EXT_XTHEADUTNMODE)) {
		prev->thread.utnmode = csr_read(CSR_UTNMODE);
		csr_write(CSR_UTNMODE, next->thread.utnmode);
	}
}
#else

static __always_inline bool has_xuantie_csr_ext(void) { return false; }
#define __switch_to_xuantie_csr_ext(__prev, __next)	do {} while (0)

#endif /* CONFIG_XUANTIE_CSR_EXT */
#endif /* _ASM_XUANTIE_CSR_EXT_H */
