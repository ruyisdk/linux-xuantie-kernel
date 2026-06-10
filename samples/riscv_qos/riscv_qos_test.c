// SPDX-License-Identifier: GPL-2.0-only

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <asm/csr.h>

int test_cpu_srmcfg(void)
{
	u32 ret = 0;
	u64 srmcfg_1, srmcfg_2;

	srmcfg_1 = csr_read(CSR_SRMCFG);
	pr_info("%s:%d csr_read(CSR_SRMCFG)=%llx\n",
			__func__, __LINE__, srmcfg_1);

	/*
	 * Tests whether the maximum RCID and MCID are supported
	 */
	csr_write(CSR_SRMCFG, SRMCFG_MASK);
	srmcfg_2 = csr_read(CSR_SRMCFG);
	if (srmcfg_2 == 0) {
		pr_warn("%s:%d csr_write(CSR_SRMCFG, %lx) failed! ret=%llx\n",
			__func__, __LINE__, SRMCFG_MASK, srmcfg_2);
		ret = -1;
	}
	if (srmcfg_2 != SRMCFG_MASK) {
		pr_warn("%s:%d csr_write(CSR_SRMCFG, %lx)=%llx. Maximum RCID and MCID not supported.\n",
			__func__, __LINE__, SRMCFG_MASK, srmcfg_2);
	}

	/*
	 * Test whether srmcfg can restore the initial value
	 */
	csr_write(CSR_SRMCFG, srmcfg_1);
	srmcfg_2 = csr_read(CSR_SRMCFG);
	if (srmcfg_2 != srmcfg_1) {
		pr_err("%s:%d csr_write(CSR_SRMCFG, %llx) failed! ret=%llx\n",
			__func__, __LINE__, srmcfg_1, srmcfg_2);
		ret = -1;
	}

	return ret;
}

static int riscv_qos_test_init(void)
{
	return test_cpu_srmcfg();
}

static void riscv_qos_test_exit(void)
{
}

module_init(riscv_qos_test_init);
module_exit(riscv_qos_test_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chen Pei <cp0613@linux.alibaba.com>");
