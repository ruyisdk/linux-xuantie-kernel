// SPDX-License-Identifier: GPL-2.0-only
/*
 * Test program for RISC-V hardware probe interface to validate RVA23 features
 * Copyright (C) 2023 Rivos, Inc
 */

#include "hwprobe.h"
#include "../../kselftest.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

int main(int argc, char **argv)
{
	struct riscv_hwprobe pairs[RISCV_HWPROBE_MAX_KEY + 1];
	unsigned long cpus;
	long out;

	ksft_print_header();
	ksft_set_plan(79);

	/* Initialize CPU set */
	cpus = 1; /* Check first CPU */

	/* Test various hardware probe keys defined in the spec */
	pairs[0].key = RISCV_HWPROBE_KEY_MVENDORID;
	pairs[1].key = RISCV_HWPROBE_KEY_MARCHID;
	pairs[2].key = RISCV_HWPROBE_KEY_MIMPID;
	pairs[3].key = RISCV_HWPROBE_KEY_BASE_BEHAVIOR;
	pairs[4].key = RISCV_HWPROBE_KEY_IMA_EXT_0;
	pairs[5].key = RISCV_HWPROBE_KEY_CPUPERF_0;
	pairs[6].key = RISCV_HWPROBE_KEY_ZICBOZ_BLOCK_SIZE;
	pairs[7].key = RISCV_HWPROBE_KEY_IMA_EXT_1;
	pairs[8].key = RISCV_HWPROBE_KEY_IMA_EXT_2;

	out = riscv_hwprobe(pairs, 9, 1, &cpus, 0);
	if (out != 0)
		ksft_exit_fail_msg("riscv_hwprobe() failed with %ld\n", out);

	/* Test MVENDORID */
	if (pairs[0].key >= 0)
		ksft_test_result_pass("MVENDORID = 0x%lx\n", pairs[0].value);
	else
		ksft_test_result_fail("MVENDORID not available\n");

	/* Test MARCHID */
	if (pairs[1].key >= 0)
		ksft_test_result_pass("MARCHID = 0x%lx\n", pairs[1].value);
	else
		ksft_test_result_fail("MARCHID not available\n");

	/* Test MIMPID */
	if (pairs[2].key >= 0)
		ksft_test_result_pass("MIMPID = 0x%lx\n", pairs[2].value);
	else
		ksft_test_result_fail("MIMPID not available\n");

	/* Test BASE_BEHAVIOR - should always be present for valid implementations */
	if (pairs[3].key == RISCV_HWPROBE_KEY_BASE_BEHAVIOR) {
		if (pairs[3].value & RISCV_HWPROBE_BASE_BEHAVIOR_IMA)
			ksft_test_result_pass("BASE_BEHAVIOR: IMA support detected\n");
		else
			ksft_test_result_fail("BASE_BEHAVIOR: IMA support not detected\n");
	} else
		ksft_test_result_fail("BASE_BEHAVIOR key not recognized\n");

	/* Test IMA Extensions - checking for all RVA23 standard extensions */
	if (pairs[4].key == RISCV_HWPROBE_KEY_IMA_EXT_0) {
		ksft_test_result_pass("IMA_EXT_0 = 0x%016lx\n", pairs[4].value);
		/* Basic extensions */
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_IMA_FD,
			"Extension FD (Double-Precision Floating-Point) %s\n",
			(pairs[4].value & RISCV_HWPROBE_IMA_FD) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_IMA_C,
			"Extension C (Compressed Instructions) %s\n",
			(pairs[4].value & RISCV_HWPROBE_IMA_C) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_IMA_V,
			"Extension V (Vector Computation) %s\n",
			(pairs[4].value & RISCV_HWPROBE_IMA_V) ? "present" : "absent");

		/* Standard Z* extensions */
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZBA,
			"Extension Zba (Address Computation) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZBA) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZBB,
			"Extension Zbb (Basic Bit-Manipulation) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZBB) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZBS,
			"Extension Zbs (Single-Bit Manipulation) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZBS) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZBC,
			"Extension Zbc (Carryless Multiplication) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZBC) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZBKB,
			"Extension Zbkb (Bit Manipulation for Cryptography) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZBKB) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZBKC,
			"Extension Zbkc (Carryless Multiplication for Cryptography) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZBKC) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZBKX,
			"Extension Zbkx (Crossbar Permutation) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZBKX) ? "present" : "absent");

		/* Cryptographic extensions */
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZKND,
			"Extension Zknd (AES Decryption) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZKND) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZKNE,
			"Extension Zkne (AES Encryption) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZKNE) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZKNH,
			"Extension Zknh (SHA2 Hashing) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZKNH) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZKSED,
			"Extension Zksed (SM4 Block Cypher) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZKSED) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZKSH,
			"Extension Zksh (SM3 Hashing) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZKSH) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZKT,
			"Extension Zkt (Data-Independent Execution Latency) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZKT) ? "present" : "absent");

		/* Vector cryptographic extensions */
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVBB,
			"Extension Zvbb (Vector Basic Bit-manipulation) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVBB) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVBC,
			"Extension Zvbc (Vector Carryless Multiplication) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVBC) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVKB,
			"Extension Zvkb (Vector Basic Bit Manipulation for Cryptography) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVKB) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVKG,
			"Extension Zvkg (Vector Cryptography) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVKG) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVKNED,
			"Extension Zvkned (Vector AES Decryption) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVKNED) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVKNHA,
			"Extension ZvknhA (Vector NIST SHA2) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVKNHA) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVKNHB,
			"Extension ZvknhB (Vector NIST SHA2) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVKNHB) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVKSED,
			"Extension Zvksed (Vector SM4) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVKSED) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVKSH,
			"Extension Zvksh (Vector SM3) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVKSH) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVKT,
			"Extension Zvkt (Vector Data-Independent Execution Latency) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVKT) ? "present" : "absent");

		/* Floating-point extensions */
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZFH,
			"Extension Zfh (Half-Precision Floating-Point) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZFH) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZFHMIN,
			"Extension Zfhmin (Minimal Half-Precision Floating-Point) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZFHMIN) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVFH,
			"Extension Zvfh (Vector Half-Precision Floating-Point) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVFH) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVFHMIN,
			"Extension Zvfhmin (Vector minimal half-precision floating-point) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVFHMIN) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZFBFMIN,
			"Extension Zfbfmin (Scalar BF16 converts) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZFBFMIN) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVFBFMIN,
			"Extension Zvfbfmin (Vector BF16 converts) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVFBFMIN) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVFBFWMA,
			"Extension Zvfbfwma (Vector BF16 widening mul-add) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVFBFWMA) ? "present" : "absent");

		/* Hints and conditional extensions */
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZIHINTNTL,
			"Extension Zihintntl (Non-temporal locality hints) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZIHINTNTL) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZIHINTPAUSE,
			"Extension Zihintpause (Pause Hint) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZIHINTPAUSE) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZICOND,
			"Extension Zicond (Integer conditional operations) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZICOND) ? "present" : "absent");

		/* Cache and memory extensions */
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZICBOZ,
			"Extension Zicboz (Cache-Block Zeroing) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZICBOZ) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZICBOM,
			"Extension Zicbom (Cache-Block Management) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZICBOM) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZICBOP,
			"Extension Zicbop (Cache-Block Prefetching) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZICBOP) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZAWRS,
			"Extension Zawrs (Wait-on-reservation-set instructions) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZAWRS) ? "present" : "absent");

		/* Atomic and memory ordering extensions */
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZACAS,
			"Extension Zacas (Atomic Compare-and-Swap (CAS) instructions) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZACAS) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZAAMO,
			"Extension Zaamo (Additional atomic memory operations) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZAAMO) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZALRSC,
			"Extension Zalrsc (Load Reserved/Store Conditional extensions) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZALRSC) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZABHA,
			"Extension Zabha (Byte and Halfword Atomic Memory Operations) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZABHA) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZALASR,
			"Extension Zalasr (Address-based Load-and-Set Register) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZALASR) ? "present" : "absent");

		/* Vector extensions */
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVE32X,
			"Extension Zve32x (Embedded Vector Computation 32-bit integer) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVE32X) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVE32F,
			"Extension Zve32f (Embedded Vector Computation 32-bit integer, 32-bit FP) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVE32F) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVE64X,
			"Extension Zve64x (Embedded Vector Computation 64-bit integer) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVE64X) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVE64F,
			"Extension Zve64f (Embedded Vector Computation 64-bit integer, 32-bit FP) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVE64F) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZVE64D,
			"Extension Zve64d (Embedded Vector Computation 64-bit integer, 64-bit FP) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZVE64D) ? "present" : "absent");

		/* Other extensions */
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZFA,
			"Extension Zfa (Additional floating-Point instructions) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZFA) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZTSO,
			"Extension Ztso (Total Store Ordering) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZTSO) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZIMOP,
			"Extension Zimop (may-be-operations) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZIMOP) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZCA,
			"Extension Zca (Additional compressed instructions) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZCA) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZCB,
			"Extension Zcb (Additional compressed instructions) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZCB) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZCD,
			"Extension Zcd (Compressed double-precision floating-point) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZCD) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZCF,
			"Extension Zcf (Compressed single-precision floating-point) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZCF) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZCMOP,
			"Extension Zcmop (Compressed may-be-operations) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZCMOP) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_SUPM,
			"Extension Supm (Pointer masking) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_SUPM) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZICNTR,
			"Extension Zicntr (Basic Performance Counters) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZICNTR) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZIHPM,
			"Extension Zihpm (Hardware Performance Counters) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZIHPM) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_SSDTSO,
			"Extension Ssdtso (Dynamic Total Store Ordering) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_SSDTSO) ? "present" : "absent");
		ksft_test_result(pairs[4].value & RISCV_HWPROBE_EXT_ZICFILP,
			"Extension Zicfilp (Landing Pads) %s\n",
			(pairs[4].value & RISCV_HWPROBE_EXT_ZICFILP) ? "present" : "absent");
	} else
		ksft_test_result_fail("IMA_EXT_0 not available\n");

	/* Test CPUPERF_0 */
	if (pairs[5].key == RISCV_HWPROBE_KEY_CPUPERF_0) {
		ksft_test_result_pass("CPUPERF_0 = 0x%lx\n", pairs[5].value);
		switch (pairs[5].value & RISCV_HWPROBE_MISALIGNED_MASK) {
		case RISCV_HWPROBE_MISALIGNED_UNKNOWN:
			ksft_test_result_pass("Misaligned access: unknown behavior\n");
			break;
		case RISCV_HWPROBE_MISALIGNED_EMULATED:
			ksft_test_result_pass("Misaligned access: emulated\n");
			break;
		case RISCV_HWPROBE_MISALIGNED_SLOW:
			ksft_test_result_pass("Misaligned access: slow\n");
			break;
		case RISCV_HWPROBE_MISALIGNED_FAST:
			ksft_test_result_pass("Misaligned access: fast\n");
			break;
		case RISCV_HWPROBE_MISALIGNED_UNSUPPORTED:
			ksft_test_result_pass("Misaligned access: unsupported\n");
			break;
		default:
			ksft_test_result_pass("Misaligned access: invalid value\n");
			break;
		}
	} else
		ksft_test_result_fail("CPUPERF_0 not available\n");

	/* Test ZICBOZ block size */
	if (pairs[6].key == RISCV_HWPROBE_KEY_ZICBOZ_BLOCK_SIZE)
		ksft_test_result_pass("ZICBOZ_BLOCK_SIZE = %lu bytes\n", pairs[6].value);
	else
		ksft_test_result_fail("ZICBOZ_BLOCK_SIZE not available\n");

	/* Test IMA Extensions - checking for all RVA23 standard extensions */
	if (pairs[7].key == RISCV_HWPROBE_KEY_IMA_EXT_1) {
		ksft_test_result_pass("IMA_EXT_1 = 0x%016lx\n", pairs[7].value);

		ksft_test_result(pairs[7].value & RISCV_HWPROBE_EXT_ZICFISS,
			"Extension Zicfiss (Shadow Stack) %s\n",
			(pairs[7].value & RISCV_HWPROBE_EXT_ZICFISS) ? "present" : "absent");
	} else
		ksft_test_result_fail("IMA_EXT_1 not available\n");

	/* Test IMA Extensions - checking for all RVA23.1 standard extensions */
	if (pairs[8].key == RISCV_HWPROBE_KEY_IMA_EXT_2) {
		ksft_test_result_pass("IMA_EXT_2 = 0x%016lx\n", pairs[8].value);

		ksft_test_result(pairs[8].value & RISCV_HWPROBE_EXT_SSDBLTRP,
			"Extension Ssdbltrp (Double Trap) %s\n",
			(pairs[8].value & RISCV_HWPROBE_EXT_SSDBLTRP) ? "present" : "absent");
		ksft_test_result(pairs[8].value & RISCV_HWPROBE_EXT_SMCDELEG,
			"Extension Smcdeleg (Counter Delegation) %s\n",
			(pairs[8].value & RISCV_HWPROBE_EXT_SMCDELEG) ? "present" : "absent");
		ksft_test_result(pairs[8].value & RISCV_HWPROBE_EXT_SSCCFG,
			"Extension Ssccfg (Counter Configuration) %s\n",
			(pairs[8].value & RISCV_HWPROBE_EXT_SSCCFG) ? "present" : "absent");
		ksft_test_result(pairs[8].value & RISCV_HWPROBE_EXT_SSCTR,
			"Extension Ssctr (Control Transfer Records) %s\n",
			(pairs[8].value & RISCV_HWPROBE_EXT_SSCTR) ? "present" : "absent");
	} else
		ksft_test_result_fail("IMA_EXT_2 not available\n");

	/* Test with invalid key to ensure proper error handling */
	pairs[0].key = 0x5555;
	out = riscv_hwprobe(pairs, 1, 1, &cpus, 0);
	if (out == 0 && pairs[0].key == -1)
		ksft_test_result_pass("Invalid key properly rejected\n");
	else
		ksft_test_result_fail("Invalid key not handled correctly\n");

	ksft_finished();
	return 0;
}
