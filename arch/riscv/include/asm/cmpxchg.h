/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2014 Regents of the University of California
 */

#ifndef _ASM_RISCV_CMPXCHG_H
#define _ASM_RISCV_CMPXCHG_H

#include <linux/bug.h>
#include <linux/mmdebug.h>

#include <asm/alternative-macros.h>
#include <asm/fence.h>
#include <asm/hwcap.h>
#include <asm/insn-def.h>

#define __xchg_relaxed(ptr, new, size)					\
({									\
	__typeof__(ptr) __ptr = (ptr);					\
	__typeof__(new) __new = (new);					\
	__typeof__(*(ptr)) __ret;					\
	switch (size) {							\
	case 4:								\
		__asm__ __volatile__ (					\
			"	amoswap.w %0, %2, %1\n"			\
			: "=r" (__ret), "+A" (*__ptr)			\
			: "r" (__new)					\
			: "memory");					\
		break;							\
	case 8:								\
		__asm__ __volatile__ (					\
			"	amoswap.d %0, %2, %1\n"			\
			: "=r" (__ret), "+A" (*__ptr)			\
			: "r" (__new)					\
			: "memory");					\
		break;							\
	default:							\
		BUILD_BUG();						\
	}								\
	__ret;								\
})

#define arch_xchg_relaxed(ptr, x)					\
({									\
	__typeof__(*(ptr)) _x_ = (x);					\
	(__typeof__(*(ptr))) __xchg_relaxed((ptr),			\
					    _x_, sizeof(*(ptr)));	\
})

#define __xchg_acquire(ptr, new, size)					\
({									\
	__typeof__(ptr) __ptr = (ptr);					\
	__typeof__(new) __new = (new);					\
	__typeof__(*(ptr)) __ret;					\
	switch (size) {							\
	case 4:								\
		__asm__ __volatile__ (					\
			"	amoswap.w %0, %2, %1\n"			\
			RISCV_ACQUIRE_BARRIER				\
			: "=r" (__ret), "+A" (*__ptr)			\
			: "r" (__new)					\
			: "memory");					\
		break;							\
	case 8:								\
		__asm__ __volatile__ (					\
			"	amoswap.d %0, %2, %1\n"			\
			RISCV_ACQUIRE_BARRIER				\
			: "=r" (__ret), "+A" (*__ptr)			\
			: "r" (__new)					\
			: "memory");					\
		break;							\
	default:							\
		BUILD_BUG();						\
	}								\
	__ret;								\
})

#define arch_xchg_acquire(ptr, x)					\
({									\
	__typeof__(*(ptr)) _x_ = (x);					\
	(__typeof__(*(ptr))) __xchg_acquire((ptr),			\
					    _x_, sizeof(*(ptr)));	\
})

#define __xchg_release(ptr, new, size)					\
({									\
	__typeof__(ptr) __ptr = (ptr);					\
	__typeof__(new) __new = (new);					\
	__typeof__(*(ptr)) __ret;					\
	switch (size) {							\
	case 4:								\
		__asm__ __volatile__ (					\
			RISCV_RELEASE_BARRIER				\
			"	amoswap.w %0, %2, %1\n"			\
			: "=r" (__ret), "+A" (*__ptr)			\
			: "r" (__new)					\
			: "memory");					\
		break;							\
	case 8:								\
		__asm__ __volatile__ (					\
			RISCV_RELEASE_BARRIER				\
			"	amoswap.d %0, %2, %1\n"			\
			: "=r" (__ret), "+A" (*__ptr)			\
			: "r" (__new)					\
			: "memory");					\
		break;							\
	default:							\
		BUILD_BUG();						\
	}								\
	__ret;								\
})

#define arch_xchg_release(ptr, x)					\
({									\
	__typeof__(*(ptr)) _x_ = (x);					\
	(__typeof__(*(ptr))) __xchg_release((ptr),			\
					    _x_, sizeof(*(ptr)));	\
})

#define __arch_xchg(ptr, new, size)					\
({									\
	__typeof__(ptr) __ptr = (ptr);					\
	__typeof__(new) __new = (new);					\
	__typeof__(*(ptr)) __ret;					\
	switch (size) {							\
	case 4:								\
		__asm__ __volatile__ (					\
			"	amoswap.w.aqrl %0, %2, %1\n"		\
			: "=r" (__ret), "+A" (*__ptr)			\
			: "r" (__new)					\
			: "memory");					\
		break;							\
	case 8:								\
		__asm__ __volatile__ (					\
			"	amoswap.d.aqrl %0, %2, %1\n"		\
			: "=r" (__ret), "+A" (*__ptr)			\
			: "r" (__new)					\
			: "memory");					\
		break;							\
	default:							\
		BUILD_BUG();						\
	}								\
	__ret;								\
})

#define arch_xchg(ptr, x)						\
({									\
	__typeof__(*(ptr)) _x_ = (x);					\
	(__typeof__(*(ptr))) __arch_xchg((ptr), _x_, sizeof(*(ptr)));	\
})

#define xchg32(ptr, x)							\
({									\
	BUILD_BUG_ON(sizeof(*(ptr)) != 4);				\
	arch_xchg((ptr), (x));						\
})

#define xchg64(ptr, x)							\
({									\
	BUILD_BUG_ON(sizeof(*(ptr)) != 8);				\
	arch_xchg((ptr), (x));						\
})

/*
 * Atomic compare and exchange.  Compare OLD with MEM, if identical,
 * store NEW in MEM.  Return the initial value in MEM.  Success is
 * indicated by comparing RETURN with OLD.
 */
#ifdef CONFIG_ARCH_RV64ILP32
#define SEXTW	"\tsext.w  %z3, %z3\n"
#else
#define SEXTW	""
#endif

#define __cmpxchg_relaxed(ptr, old, new, size)				\
({									\
	__typeof__(ptr) __ptr = (ptr);					\
	__typeof__(*(ptr)) __old = (old);				\
	__typeof__(*(ptr)) __new = (new);				\
	__typeof__(*(ptr)) __ret;					\
	register unsigned int __rc;					\
	switch (size) {							\
	case 4:								\
		__asm__ __volatile__ (					\
			SEXTW						\
			"0:	lr.w %0, %2\n"				\
			"	bne  %0, %z3, 1f\n"			\
			"	sc.w %1, %z4, %2\n"			\
			"	bnez %1, 0b\n"				\
			"1:\n"						\
			: "=&r" (__ret), "=&r" (__rc), "+A" (*__ptr)	\
			: "rJ" ((long)__old), "rJ" (__new)		\
			: "memory");					\
		break;							\
	case 8:								\
		__asm__ __volatile__ (					\
			"0:	lr.d %0, %2\n"				\
			"	bne %0, %z3, 1f\n"			\
			"	sc.d %1, %z4, %2\n"			\
			"	bnez %1, 0b\n"				\
			"1:\n"						\
			: "=&r" (__ret), "=&r" (__rc), "+A" (*__ptr)	\
			: "rJ" (__old), "rJ" (__new)			\
			: "memory");					\
		break;							\
	default:							\
		BUILD_BUG();						\
	}								\
	__ret;								\
})

#define arch_cmpxchg_relaxed(ptr, o, n)					\
({									\
	__typeof__(*(ptr)) _o_ = (o);					\
	__typeof__(*(ptr)) _n_ = (n);					\
	(__typeof__(*(ptr))) __cmpxchg_relaxed((ptr),			\
					_o_, _n_, sizeof(*(ptr)));	\
})

#define __cmpxchg_acquire(ptr, old, new, size)				\
({									\
	__typeof__(ptr) __ptr = (ptr);					\
	__typeof__(*(ptr)) __old = (old);				\
	__typeof__(*(ptr)) __new = (new);				\
	__typeof__(*(ptr)) __ret;					\
	register unsigned int __rc;					\
	switch (size) {							\
	case 4:								\
		__asm__ __volatile__ (					\
			SEXTW						\
			"0:	lr.w %0, %2\n"				\
			"	bne  %0, %z3, 1f\n"			\
			"	sc.w %1, %z4, %2\n"			\
			"	bnez %1, 0b\n"				\
			RISCV_ACQUIRE_BARRIER				\
			"1:\n"						\
			: "=&r" (__ret), "=&r" (__rc), "+A" (*__ptr)	\
			: "rJ" ((long)__old), "rJ" (__new)		\
			: "memory");					\
		break;							\
	case 8:								\
		__asm__ __volatile__ (					\
			"0:	lr.d %0, %2\n"				\
			"	bne %0, %z3, 1f\n"			\
			"	sc.d %1, %z4, %2\n"			\
			"	bnez %1, 0b\n"				\
			RISCV_ACQUIRE_BARRIER				\
			"1:\n"						\
			: "=&r" (__ret), "=&r" (__rc), "+A" (*__ptr)	\
			: "rJ" (__old), "rJ" (__new)			\
			: "memory");					\
		break;							\
	default:							\
		BUILD_BUG();						\
	}								\
	__ret;								\
})

#define arch_cmpxchg_acquire(ptr, o, n)					\
({									\
	__typeof__(*(ptr)) _o_ = (o);					\
	__typeof__(*(ptr)) _n_ = (n);					\
	(__typeof__(*(ptr))) __cmpxchg_acquire((ptr),			\
					_o_, _n_, sizeof(*(ptr)));	\
})

#define __cmpxchg_release(ptr, old, new, size)				\
({									\
	__typeof__(ptr) __ptr = (ptr);					\
	__typeof__(*(ptr)) __old = (old);				\
	__typeof__(*(ptr)) __new = (new);				\
	__typeof__(*(ptr)) __ret;					\
	register unsigned int __rc;					\
	switch (size) {							\
	case 4:								\
		__asm__ __volatile__ (					\
			SEXTW						\
			RISCV_RELEASE_BARRIER				\
			"0:	lr.w %0, %2\n"				\
			"	bne  %0, %z3, 1f\n"			\
			"	sc.w %1, %z4, %2\n"			\
			"	bnez %1, 0b\n"				\
			"1:\n"						\
			: "=&r" (__ret), "=&r" (__rc), "+A" (*__ptr)	\
			: "rJ" ((long)__old), "rJ" (__new)		\
			: "memory");					\
		break;							\
	case 8:								\
		__asm__ __volatile__ (					\
			RISCV_RELEASE_BARRIER				\
			"0:	lr.d %0, %2\n"				\
			"	bne %0, %z3, 1f\n"			\
			"	sc.d %1, %z4, %2\n"			\
			"	bnez %1, 0b\n"				\
			"1:\n"						\
			: "=&r" (__ret), "=&r" (__rc), "+A" (*__ptr)	\
			: "rJ" (__old), "rJ" (__new)			\
			: "memory");					\
		break;							\
	default:							\
		BUILD_BUG();						\
	}								\
	__ret;								\
})

#define arch_cmpxchg_release(ptr, o, n)					\
({									\
	__typeof__(*(ptr)) _o_ = (o);					\
	__typeof__(*(ptr)) _n_ = (n);					\
	(__typeof__(*(ptr))) __cmpxchg_release((ptr),			\
					_o_, _n_, sizeof(*(ptr)));	\
})

#define __cmpxchg(ptr, old, new, size)					\
({									\
	__typeof__(ptr) __ptr = (ptr);					\
	__typeof__(*(ptr)) __old = (old);				\
	__typeof__(*(ptr)) __new = (new);				\
	__typeof__(*(ptr)) __ret;					\
	register unsigned int __rc;					\
	switch (size) {							\
	case 4:								\
		__asm__ __volatile__ (					\
			SEXTW						\
			"0:	lr.w %0, %2\n"				\
			"	bne  %0, %z3, 1f\n"			\
			"	sc.w.rl %1, %z4, %2\n"			\
			"	bnez %1, 0b\n"				\
			"	fence rw, rw\n"				\
			"1:\n"						\
			: "=&r" (__ret), "=&r" (__rc), "+A" (*__ptr)	\
			: "rJ" ((long)__old), "rJ" (__new)		\
			: "memory");					\
		break;							\
	case 8:								\
		__asm__ __volatile__ (					\
			"0:	lr.d %0, %2\n"				\
			"	bne %0, %z3, 1f\n"			\
			"	sc.d.rl %1, %z4, %2\n"			\
			"	bnez %1, 0b\n"				\
			"	fence rw, rw\n"				\
			"1:\n"						\
			: "=&r" (__ret), "=&r" (__rc), "+A" (*__ptr)	\
			: "rJ" (__old), "rJ" (__new)			\
			: "memory");					\
		break;							\
	default:							\
		BUILD_BUG();						\
	}								\
	__ret;								\
})

#define arch_cmpxchg(ptr, o, n)						\
({									\
	__typeof__(*(ptr)) _o_ = (o);					\
	__typeof__(*(ptr)) _n_ = (n);					\
	(__typeof__(*(ptr))) __cmpxchg((ptr),				\
				       _o_, _n_, sizeof(*(ptr)));	\
})

#define arch_cmpxchg_local(ptr, o, n)					\
	(__cmpxchg_relaxed((ptr), (o), (n), sizeof(*(ptr))))

#define arch_cmpxchg64(ptr, o, n)					\
({									\
	BUILD_BUG_ON(sizeof(*(ptr)) != 8);				\
	arch_cmpxchg((ptr), (o), (n));					\
})

#define arch_cmpxchg64_local(ptr, o, n)					\
({									\
	BUILD_BUG_ON(sizeof(*(ptr)) != 8);				\
	arch_cmpxchg_relaxed((ptr), (o), (n));				\
})

#ifdef CONFIG_ARCH_RV64ILP32
#define system_has_cmpxchg_double()	1

#define __cmpxchg_double_check(ptr1, ptr2)				\
({									\
	if (sizeof(*(ptr1)) != 4)					\
		BUILD_BUG();						\
	if (sizeof(*(ptr2)) != 4)					\
		BUILD_BUG();						\
	VM_BUG_ON((ulong *)(ptr2) - (ulong *)(ptr1) != 1);		\
	VM_BUG_ON(((ulong)ptr1 & 0x7) != 0);				\
})

#define __cmpxchg_double(old1, old2, new1, new2, ptr)			\
({									\
	__typeof__(ptr) __ptr = (ptr);					\
	register unsigned int __ret;					\
	u64 __old;							\
	u64 __new;							\
	u64 __tmp;							\
	switch (sizeof(*(ptr))) {					\
	case 4:								\
		__old = ((u64)old2 << 32) | (u64)old1;			\
		__new = ((u64)new2 << 32) | (u64)new1;			\
		__asm__ __volatile__ (					\
			"0:	lr.d %0, %2\n"				\
			"	bne %0, %z3, 1f\n"			\
			"	sc.d %1, %z4, %2\n"			\
			"	bnez %1, 0b\n"				\
			"1:\n"						\
			: "=&r" (__tmp), "=&r" (__ret), "+A" (*__ptr)	\
			: "rJ" (__old), "rJ" (__new)			\
			: "memory");					\
		__ret = (__old == __tmp);				\
		break;							\
	default:							\
		BUILD_BUG();						\
	}								\
	__ret;								\
})

#define arch_cmpxchg_double(ptr1, ptr2, o1, o2, n1, n2)			\
({									\
	int __ret;							\
	__cmpxchg_double_check(ptr1, ptr2);				\
	__ret = __cmpxchg_double((ulong)(o1), (ulong)(o2),		\
				 (ulong)(n1), (ulong)(n2),		\
				  ptr1);				\
	__ret;								\
})

#endif

#ifdef CONFIG_RISCV_ISA_ZAWRS
/*
 * Despite wrs.nto being "WRS-with-no-timeout", in the absence of changes to
 * @val we expect it to still terminate within a "reasonable" amount of time
 * for an implementation-specific other reason, a pending, locally-enabled
 * interrupt, or because it has been configured to raise an illegal
 * instruction exception.
 */
static __always_inline void __cmpwait(void *ptr,
				      unsigned long val,
				      int size)
{
	unsigned long tmp;

	asm goto(ALTERNATIVE("j %l[no_zawrs]", "nop",
			     0, RISCV_ISA_EXT_ZAWRS, 1)
		 : : : : no_zawrs);

	switch (size) {
	case 4:
		asm volatile(
		"	lr.w	%0, %1\n"
		"	xor	%0, %0, %2\n"
		"	bnez	%0, 1f\n"
			ZAWRS_WRS_NTO "\n"
		"1:"
		: "=&r" (tmp), "+A" (*(u32 *)ptr)
		: "r" (val));
		break;
#if __riscv_xlen == 64
	case 8:
		asm volatile(
		"	lr.d	%0, %1\n"
		"	xor	%0, %0, %2\n"
		"	bnez	%0, 1f\n"
			ZAWRS_WRS_NTO "\n"
		"1:"
		: "=&r" (tmp), "+A" (*(u64 *)ptr)
		: "r" (val));
		break;
#endif
	default:
		BUILD_BUG();
	}

	return;

no_zawrs:
	asm volatile(RISCV_PAUSE : : : "memory");
}

#define __cmpwait_relaxed(ptr, val) \
	__cmpwait((ptr), (unsigned long)(val), sizeof(*(ptr)))
#endif

#endif /* _ASM_RISCV_CMPXCHG_H */
