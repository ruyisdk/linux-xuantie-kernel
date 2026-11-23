/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2024 Christoph Muellner <christoph.muellner@vrull.eu>
 */

#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/prctl.h>

#include <asm/cpu.h>
#include <asm/dtso.h>

#include <trace/events/ipi.h>

static void dtso_apply_model_on_cpu(void *info)
{
	struct task_struct *task = info;

	if (current->mm == task->mm)
		dtso_sched_in(current);
}

int dtso_set_memory_consistency_model(unsigned long arg)
{
	unsigned long cur_model = get_memory_consistency_model(current);
	unsigned long new_model;

	switch (arg) {
	case PR_MEMORY_CONSISTENCY_MODEL_RISCV_WMO:
		new_model = RISCV_MEMORY_CONSISTENCY_MODEL_WMO;
		break;
	case PR_MEMORY_CONSISTENCY_MODEL_RISCV_TSO:
		new_model = RISCV_MEMORY_CONSISTENCY_MODEL_TSO;
		break;
	default:
		return -EINVAL;
	}

	/* No change requested. */
	if (cur_model == new_model)
		return 0;

	/* Enabling TSO only works if DTSO is available. */
	if (new_model == RISCV_MEMORY_CONSISTENCY_MODEL_TSO && !has_dtso())
		return -EINVAL;

	/* Switching TSO->WMO is not allowed. */
	if (new_model == RISCV_MEMORY_CONSISTENCY_MODEL_WMO)
		return -EINVAL;

	/* Stored in the mm, so threads created during the switch see it too. */
	set_memory_consistency_model(current, new_model);

	/* Apply the new model on every CPU right away. */
	on_each_cpu(dtso_apply_model_on_cpu, current, 1);

	return 0;
}

int dtso_get_memory_consistency_model(void)
{
	unsigned long cur_model = get_memory_consistency_model(current);

	if (cur_model == RISCV_MEMORY_CONSISTENCY_MODEL_TSO)
		return PR_MEMORY_CONSISTENCY_MODEL_RISCV_TSO;

	return PR_MEMORY_CONSISTENCY_MODEL_RISCV_WMO;
}
