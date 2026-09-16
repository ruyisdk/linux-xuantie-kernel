// SPDX-License-Identifier: GPL-2.0-only
/* dtso - used for functional tests of memory consistency model switching
 * at run-time.
 *
 * Copyright (c) 2023 Christoph Muellner <christoph.muellner@vrull.eu>
 */

#include <sys/prctl.h>
#include <unistd.h>
#include <errno.h>

#include "../hwprobe/hwprobe.h"
#include "../../kselftest_harness.h"

#ifndef RISCV_HWPROBE_EXT_SSDTSO
#define RISCV_HWPROBE_EXT_SSDTSO        (1ULL << 5)
#endif

#ifndef PR_SET_MEMORY_CONSISTENCY_MODEL
#define PR_SET_MEMORY_CONSISTENCY_MODEL         77
#endif

#ifndef PR_GET_MEMORY_CONSISTENCY_MODEL
#define PR_GET_MEMORY_CONSISTENCY_MODEL         78
#endif

#ifndef PR_MEMORY_CONSISTENCY_MODEL_RISCV_WMO
#define PR_MEMORY_CONSISTENCY_MODEL_RISCV_WMO  1
#endif

#ifndef PR_MEMORY_CONSISTENCY_MODEL_RISCV_TSO
#define PR_MEMORY_CONSISTENCY_MODEL_RISCV_TSO  2
#endif

/*
 * We have the following cases:
 * 1) DTSO support disabed in the kernel config:
 *    - Ssdtso is not detected
 *    - {G,S}ET_MEMORY_CONSISTENCY_MODEL fails with EINVAL
 * 2) DTSO support enabled and Ssdtso not available:
 *    - Ssdtso is not detected
 *    - {G,S}ET_MEMORY_CONSISTENCY_MODEL works for WMO and fails for TSO with EINVAL:
 * 3) DTSO support enabled and Ssdtso available
 *    - Ssdtso is detected
 *    - {G,S}ET_MEMORY_CONSISTENCY_MODEL works for WMO and TSO
 */

TEST(dtso)
{
	struct riscv_hwprobe pair;
	int ret;
	bool ssdtso_configured;
	bool ssdtso_available;

	ret = prctl(PR_GET_MEMORY_CONSISTENCY_MODEL);
	if (ret < 0) {
		ASSERT_EQ(errno, EINVAL);
		ssdtso_configured = false;
	} else {
		ASSERT_TRUE(ret == PR_MEMORY_CONSISTENCY_MODEL_RISCV_WMO ||
			    ret == PR_MEMORY_CONSISTENCY_MODEL_RISCV_TSO);
		ssdtso_configured = true;
	}
	TH_LOG("ssdtso_configured = %s", (ret < 0) ? "false" : "true");

	pair.key = RISCV_HWPROBE_KEY_IMA_EXT_2;
	ret = riscv_hwprobe(&pair, 1, 0, NULL, 0);
	ASSERT_GE(ret, 0);

	if (pair.key == -1) {
		ksft_test_result_skip("RISCV_HWPROBE_KEY_IMA_EXT_2 not supported\n");
		return;
	}

	ASSERT_EQ(pair.key, RISCV_HWPROBE_KEY_IMA_EXT_2);
	ssdtso_available = !!(pair.value & RISCV_HWPROBE_EXT_SSDTSO);
	TH_LOG("ssdtso_available = %s", ssdtso_available ? "true" : "false");

	if (ssdtso_configured) {
		/* Read out current model. */
		ret = prctl(PR_GET_MEMORY_CONSISTENCY_MODEL);
		ASSERT_TRUE(ret == PR_MEMORY_CONSISTENCY_MODEL_RISCV_WMO ||
			    ret == PR_MEMORY_CONSISTENCY_MODEL_RISCV_TSO);

		TH_LOG("Read out current model = %d('%s')", ret,
			ret == PR_MEMORY_CONSISTENCY_MODEL_RISCV_WMO ?
			"WMO" : "TSO");

		if (ssdtso_available) {
			/* Switch to TSO. */
			ret = prctl(PR_SET_MEMORY_CONSISTENCY_MODEL,
				    PR_MEMORY_CONSISTENCY_MODEL_RISCV_TSO);
			ASSERT_EQ(ret, 0);
			ret = prctl(PR_GET_MEMORY_CONSISTENCY_MODEL);
			ASSERT_TRUE(ret == PR_MEMORY_CONSISTENCY_MODEL_RISCV_TSO);

			TH_LOG("prctl(PR_MEMORY_CONSISTENCY_MODEL_RISCV_TSO, '%s'(%d))",
				ret == PR_MEMORY_CONSISTENCY_MODEL_RISCV_WMO ?
				"WMO" : "TSO", ret);

			/* Try switching back to WMO (must fail). */
			ret = prctl(PR_SET_MEMORY_CONSISTENCY_MODEL,
				    PR_MEMORY_CONSISTENCY_MODEL_RISCV_WMO);
			ASSERT_LT(ret, 0);
			ret = prctl(PR_GET_MEMORY_CONSISTENCY_MODEL);
			ASSERT_TRUE(ret == PR_MEMORY_CONSISTENCY_MODEL_RISCV_TSO);
		} else {
			/* Set the same model, that's currently active. */
			TH_LOG("prctl(PR_SET_MEMORY_CONSISTENCY_MODEL, '%s'(%d))",
				ret == PR_MEMORY_CONSISTENCY_MODEL_RISCV_WMO ?
				"WMO" : "TSO", ret);
			ret = prctl(PR_SET_MEMORY_CONSISTENCY_MODEL, ret);
			ASSERT_EQ(ret, 0);
		}
	} else {
		ASSERT_EQ(ssdtso_available, false);
		ksft_test_result_skip("Ssdtso not configured\n");
	}
}

TEST_HARNESS_MAIN
