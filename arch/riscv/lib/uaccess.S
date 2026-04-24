#include <linux/linkage.h>
#include <asm-generic/export.h>
#include <asm/asm.h>
#include <asm/asm-extable.h>
#include <asm/csr.h>

	.macro fixup op reg addr lbl
100:
	\op \reg, \addr
	_asm_extable	100b, \lbl
	.endm

SYM_FUNC_START(__asm_copy_to_user)

#ifdef CONFIG_RISCV_EFFICIENT_UNALIGNED_ACCESS

	/* Enable access to user memory */
	li t6, SR_SUM
	csrs CSR_STATUS, t6

	/*
	 * Save the terminal address which will be used to compute the number
	 * of bytes copied in case of a fixup exception.
	 */
	add	t5, a0, a2

	/*
	 * Register allocation for code below:
	 * a0 - start of uncopied dst
	 * a1 - start of uncopied src
	 * a2 - size
	 * t0 - end of uncopied dst
	 */
	mv      t0, t5          /* t0 = end of dst */

	/* If length < 8, go to byte copy */
	li      a3, 8
	bltu    a2, a3, .L_copy_by_byte

	/* Handle lengths >= 8 */
	srli    t4, a2, 7       /* t4 = len / 128 */
	bnez    t4, .L_len_ge_128

	/* Check for [64, 128) */
	srli    t4, a2, 6       /* t4 = len / 64 */
	beqz    t4, .L_len_less_64

.L_handle_64_to_127:
	andi    a2, a2, 63      /* remaining after 64-byte block */

	fixup ld a4, 0(a1), 10f
	fixup ld a5, 8(a1), 10f
	fixup ld a6, 16(a1), 10f
	fixup ld a7, 24(a1), 10f
	fixup sd a4, 0(a0), 10f
	fixup sd a5, 8(a0), 10f
	fixup sd a6, 16(a0), 10f
	fixup sd a7, 24(a0), 10f

	fixup ld a4, 32(a1), 10f
	fixup ld a5, 40(a1), 10f
	fixup ld a6, 48(a1), 10f
	fixup ld a7, 56(a1), 10f
	fixup sd a4, 32(a0), 10f
	fixup sd a5, 40(a0), 10f
	fixup sd a6, 48(a0), 10f
	fixup sd a7, 56(a0), 10f
	addi    a1, a1, 64
	addi    a0, a0, 64
	j       .L_len_less_64

.L_len_ge_128:
	andi    a2, a2, 127     /* tail after full 128-byte blocks */
.L_loop_128:
	fixup ld a4, 0(a1), 10f
	fixup ld a5, 8(a1), 10f
	fixup ld a6, 16(a1), 10f
	fixup ld a7, 24(a1), 10f
	fixup sd a4, 0(a0), 10f
	fixup sd a5, 8(a0), 10f
	fixup sd a6, 16(a0), 10f
	fixup sd a7, 24(a0), 10f

	fixup ld a4, 32(a1), 10f
	fixup ld a5, 40(a1), 10f
	fixup ld a6, 48(a1), 10f
	fixup ld a7, 56(a1), 10f
	fixup sd a4, 32(a0), 10f
	fixup sd a5, 40(a0), 10f
	fixup sd a6, 48(a0), 10f
	fixup sd a7, 56(a0), 10f

	addi    a1, a1, 64
	addi    a0, a0, 64

	fixup ld a4, 0(a1), 10f
	fixup ld a5, 8(a1), 10f
	fixup ld a6, 16(a1), 10f
	fixup ld a7, 24(a1), 10f
	fixup sd a4, 0(a0), 10f
	fixup sd a5, 8(a0), 10f
	fixup sd a6, 16(a0), 10f
	fixup sd a7, 24(a0), 10f

	fixup ld a4, 32(a1), 10f
	fixup ld a5, 40(a1), 10f
	fixup ld a6, 48(a1), 10f
	fixup ld a7, 56(a1), 10f
	fixup sd a4, 32(a0), 10f
	fixup sd a5, 40(a0), 10f
	fixup sd a6, 48(a0), 10f
	fixup sd a7, 56(a0), 10f

	addi    a1, a1, 64
	addi    a0, a0, 64

	addi    t4, t4, -1
	bnez    t4, .L_loop_128

.L_len_less_64:
	srli    t4, a2, 4       /* t4 = len / 16 */
	beqz    t4, .L_len_less_16
	andi    a2, a2, 15
.L_loop_16:
	fixup ld a4, 0(a1), 10f
	fixup ld a5, 8(a1), 10f
	fixup sd a4, 0(a0), 10f
	fixup sd a5, 8(a0), 10f
	addi    a1, a1, 16
	addi    a0, a0, 16
	addi    t4, t4, -1
	bnez    t4, .L_loop_16

.L_len_less_16:
	srli    t4, a2, 3       /* t4 = len / 8 */
	beqz    t4, .L_len_less_8
	andi    a2, a2, 7
	fixup ld a4, 0(a1), 10f
	fixup sd a4, 0(a0), 10f
	addi a1, a1, 8
	addi a0, a0, 8


.L_len_less_8:
	srli    t4, a2, 2       /* t4 = len / 4 */
	beqz    t4, .L_copy_by_byte
	andi    a2, a2, 3
	fixup lw a4, 0(a1), 10f
	fixup sw a4, 0(a0), 10f
	addi a1, a1, 4
	addi a0, a0, 4

.L_copy_by_byte:
	beqz    a2, .L_return
.L_byte_loop:
	fixup lb a4, 0(a1), 10f
	fixup sb a4, 0(a0), 10f
	addi a1, a1, 1
	addi a0, a0, 1

	addi    a2, a2, -1
	bnez    a2, .L_byte_loop

.L_return:
	csrc CSR_STATUS, t6
	li   a0, 0
	ret

10:     /* Fixup handler */
	csrc CSR_STATUS, t6
	sub  a0, t5, a0   /* return number of bytes NOT copied */
	ret

#else

	/* Enable access to user memory */
	li t6, SR_SUM
	csrs CSR_STATUS, t6

	/*
	 * Save the terminal address which will be used to compute the number
	 * of bytes copied in case of a fixup exception.
	 */
	add	t5, a0, a2

	/*
	 * Register allocation for code below:
	 * a0 - start of uncopied dst
	 * a1 - start of uncopied src
	 * a2 - size
	 * t0 - end of uncopied dst
	 */
	add	t0, a0, a2

	/*
	 * Use byte copy only if too small.
	 * SZREG holds 4 for RV32 and 8 for RV64
	 */
	li	a3, 9*SZREG /* size must be larger than size in word_copy */
	bltu	a2, a3, .Lbyte_copy_tail

	/*
	 * Copy first bytes until dst is aligned to word boundary.
	 * a0 - start of dst
	 * t1 - start of aligned dst
	 */
	addi	t1, a0, SZREG-1
	andi	t1, t1, ~(SZREG-1)
	/* dst is already aligned, skip */
	beq	a0, t1, .Lskip_align_dst
1:
	/* a5 - one byte for copying data */
	fixup lb      a5, 0(a1), 10f
	addi	a1, a1, 1	/* src */
	fixup sb      a5, 0(a0), 10f
	addi	a0, a0, 1	/* dst */
	bltu	a0, t1, 1b	/* t1 - start of aligned dst */

.Lskip_align_dst:
	/*
	 * Now dst is aligned.
	 * Use shift-copy if src is misaligned.
	 * Use word-copy if both src and dst are aligned because
	 * can not use shift-copy which do not require shifting
	 */
	/* a1 - start of src */
	andi	a3, a1, SZREG-1
	bnez	a3, .Lshift_copy

.Lword_copy:
        /*
	 * Both src and dst are aligned, unrolled word copy
	 *
	 * a0 - start of aligned dst
	 * a1 - start of aligned src
	 * t0 - end of aligned dst
	 */
	addi	t0, t0, -(8*SZREG) /* not to over run */
2:
	fixup REG_L   a4,        0(a1), 10f
	fixup REG_L   a5,    SZREG(a1), 10f
	fixup REG_L   a6,  2*SZREG(a1), 10f
	fixup REG_L   a7,  3*SZREG(a1), 10f
	fixup REG_L   t1,  4*SZREG(a1), 10f
	fixup REG_L   t2,  5*SZREG(a1), 10f
	fixup REG_L   t3,  6*SZREG(a1), 10f
	fixup REG_L   t4,  7*SZREG(a1), 10f
	fixup REG_S   a4,        0(a0), 10f
	fixup REG_S   a5,    SZREG(a0), 10f
	fixup REG_S   a6,  2*SZREG(a0), 10f
	fixup REG_S   a7,  3*SZREG(a0), 10f
	fixup REG_S   t1,  4*SZREG(a0), 10f
	fixup REG_S   t2,  5*SZREG(a0), 10f
	fixup REG_S   t3,  6*SZREG(a0), 10f
	fixup REG_S   t4,  7*SZREG(a0), 10f
	addi	a0, a0, 8*SZREG
	addi	a1, a1, 8*SZREG
	bltu	a0, t0, 2b

	addi	t0, t0, 8*SZREG /* revert to original value */
	j	.Lbyte_copy_tail

.Lshift_copy:

	/*
	 * Word copy with shifting.
	 * For misaligned copy we still perform aligned word copy, but
	 * we need to use the value fetched from the previous iteration and
	 * do some shifts.
	 * This is safe because reading is less than a word size.
	 *
	 * a0 - start of aligned dst
	 * a1 - start of src
	 * a3 - a1 & mask:(SZREG-1)
	 * t0 - end of uncopied dst
	 * t1 - end of aligned dst
	 */
	/* calculating aligned word boundary for dst */
	andi	t1, t0, ~(SZREG-1)
	/* Converting unaligned src to aligned src */
	andi	a1, a1, ~(SZREG-1)

	/*
	 * Calculate shifts
	 * t3 - prev shift
	 * t4 - current shift
	 */
	slli	t3, a3, 3 /* converting bytes in a3 to bits */
	li	a5, SZREG*8
	sub	t4, a5, t3

	/* Load the first word to combine with second word */
	fixup REG_L   a5, 0(a1), 10f

3:
	/* Main shifting copy
	 *
	 * a0 - start of aligned dst
	 * a1 - start of aligned src
	 * t1 - end of aligned dst
	 */

	/* At least one iteration will be executed */
	srl	a4, a5, t3
	fixup REG_L   a5, SZREG(a1), 10f
	addi	a1, a1, SZREG
	sll	a2, a5, t4
	or	a2, a2, a4
	fixup REG_S   a2, 0(a0), 10f
	addi	a0, a0, SZREG
	bltu	a0, t1, 3b

	/* Revert src to original unaligned value  */
	add	a1, a1, a3

.Lbyte_copy_tail:
	/*
	 * Byte copy anything left.
	 *
	 * a0 - start of remaining dst
	 * a1 - start of remaining src
	 * t0 - end of remaining dst
	 */
	bgeu	a0, t0, .Lout_copy_user  /* check if end of copy */
4:
	fixup lb      a5, 0(a1), 10f
	addi	a1, a1, 1	/* src */
	fixup sb      a5, 0(a0), 10f
	addi	a0, a0, 1	/* dst */
	bltu	a0, t0, 4b	/* t0 - end of dst */

.Lout_copy_user:
	/* Disable access to user memory */
	csrc CSR_STATUS, t6
	li	a0, 0
	ret

	/* Exception fixup code */
10:
	/* Disable access to user memory */
	csrc CSR_STATUS, t6
	sub a0, t5, a0
	ret
#endif
SYM_FUNC_END(__asm_copy_to_user)
EXPORT_SYMBOL(__asm_copy_to_user)
SYM_FUNC_ALIAS(__asm_copy_from_user, __asm_copy_to_user)
EXPORT_SYMBOL(__asm_copy_from_user)


SYM_FUNC_START(__clear_user)

	/* Enable access to user memory */
	li t6, SR_SUM
	csrs CSR_STATUS, t6

	add a3, a0, a1
	addi t0, a0, SZREG-1
	andi t1, a3, ~(SZREG-1)
	andi t0, t0, ~(SZREG-1)
	/*
	 * a3: terminal address of target region
	 * t0: lowest doubleword-aligned address in target region
	 * t1: highest doubleword-aligned address in target region
	 */
	bgeu t0, t1, 2f
	bltu a0, t0, 4f
1:
	fixup REG_S, zero, (a0), 11f
	addi a0, a0, SZREG
	bltu a0, t1, 1b
2:
	bltu a0, a3, 5f

3:
	/* Disable access to user memory */
	csrc CSR_STATUS, t6
	li a0, 0
	ret
4: /* Edge case: unalignment */
	fixup sb, zero, (a0), 11f
	addi a0, a0, 1
	bltu a0, t0, 4b
	j 1b
5: /* Edge case: remainder */
	fixup sb, zero, (a0), 11f
	addi a0, a0, 1
	bltu a0, a3, 5b
	j 3b

	/* Exception fixup code */
11:
	/* Disable access to user memory */
	csrc CSR_STATUS, t6
	sub a0, a3, a0
	ret
SYM_FUNC_END(__clear_user)
EXPORT_SYMBOL(__clear_user)
