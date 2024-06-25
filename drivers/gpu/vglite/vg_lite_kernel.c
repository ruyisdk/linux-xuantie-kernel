// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Copyright (c) 2014 - 2020 Vivante Corporation
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 */

#include "vg_lite_platform.h"
#include "vg_lite_kernel.h"
#include "vg_lite_hal.h"
#include "vg_lite_hw.h"
#include "vg_lite_options.h"
#if defined(__linux__) && !defined(EMULATOR)
#include <linux/sched.h>
/*#include <asm/uaccess.h>*/
#include <linux/uaccess.h>
#include <linux/version.h>
#endif

#define FLEXA_TIMEOUT_STATE BIT(21)
#define FLEXA_HANDSHEKE_FAIL_STATE BIT(22)
#define MIN_TS_SIZE (8 << 10)

static int s_reference;

static enum vg_lite_error do_terminate(struct vg_lite_kernel_terminate *data);

static void soft_reset(void);

static void gpu(int enable)
{
	union vg_lite_hw_clock_control value;
	uint32_t reset_timer = 2;
	const uint32_t reset_timer_limit = 1000;

	if (enable) {
		/* Disable clock gating. */
		value.data = vg_lite_hal_peek(VG_LITE_HW_CLOCK_CONTROL);
		value.control.clock_gate = 0;
		vg_lite_hal_poke(VG_LITE_HW_CLOCK_CONTROL, value.data);
		vg_lite_hal_delay(1);

		/* Set clock speed. */
		value.control.scale = 64;
		value.control.scale_load = 1;
		vg_lite_hal_poke(VG_LITE_HW_CLOCK_CONTROL, value.data);
		vg_lite_hal_delay(1);
		value.control.scale_load = 0;
		vg_lite_hal_poke(VG_LITE_HW_CLOCK_CONTROL, value.data);
		vg_lite_hal_delay(5);

		do {
			/* Perform a soft reset. */
			soft_reset();
			vg_lite_hal_delay(reset_timer);
			/* If reset failed, try again with a longer wait.
			 * Need to check why if dead lopp happens here.
			 */
			reset_timer *= 2;
		} while (!VG_LITE_KERNEL_IS_GPU_IDLE());
	} else {
		while (!VG_LITE_KERNEL_IS_GPU_IDLE() &&
		       (reset_timer <
			reset_timer_limit) // Force shutdown if timeout.
		) {
			vg_lite_hal_delay(reset_timer);
			reset_timer *= 2;
		}

		/* Set idle speed. */
		value.data = vg_lite_hal_peek(VG_LITE_HW_CLOCK_CONTROL);
		value.control.scale = 1;
		value.control.scale_load = 1;
		vg_lite_hal_poke(VG_LITE_HW_CLOCK_CONTROL, value.data);
		vg_lite_hal_delay(1);
		value.control.scale_load = 0;
		vg_lite_hal_poke(VG_LITE_HW_CLOCK_CONTROL, value.data);
		vg_lite_hal_delay(5);

		/* Enable clock gating. */
		value.control.clock_gate = 1;
		vg_lite_hal_poke(VG_LITE_HW_CLOCK_CONTROL, value.data);
		vg_lite_hal_delay(1);
	}
}

/* Initialize some customized modeuls [DDRLess]. */
static enum vg_lite_error init_3rd(struct vg_lite_kernel_initialize *data)
{
	enum vg_lite_error error = VG_LITE_SUCCESS;

	/* TODO: Init the YUV<->RGB converters. Reserved for SOC. */
	// vg_lite_hal_poke(0x00514, data->yuv_pre);
    // vg_lite_hal_poke(0x00518, data->yuv_post);
	return error;
}

static enum vg_lite_error init_vglite(struct vg_lite_kernel_initialize *data)
{
	enum vg_lite_error error = VG_LITE_SUCCESS;
	struct vg_lite_kernel_context *context;
	int i;

#if defined(__linux__) && !defined(EMULATOR)
	struct vg_lite_kernel_context __user *context_usr;
	struct vg_lite_kernel_context mycontext = { 0 };

	// Construct the context.
	context_usr = (struct vg_lite_kernel_context __user *)data->context;
	if (!access_ok(context_usr, sizeof(*context_usr)) ||
	    !access_ok(context_usr, sizeof(*context_usr)))
		/* Out of memory. */
		return VG_LITE_OUT_OF_MEMORY;
	context = &mycontext;
#else
	// Construct the context.
	context = data->context;
	if (context == NULL) {
		/* Out of memory. */
		return VG_LITE_OUT_OF_MEMORY;
	}
#endif

	/* Zero out all pointers. */
	for (i = 0; i < CMDBUF_COUNT; i++) {
		context->command_buffer[i] = NULL;
		context->command_buffer_logical[i] = NULL;
		context->command_buffer_physical[i] = 0;
	}
	context->tess_buffer = NULL;
	context->tessbuf_logical = NULL;
	context->tessbuf_physical = 0;
	/* Increment reference counter. */
	if (s_reference++ == 0) {
		/* Initialize the SOC. */
		vg_lite_hal_initialize();

		/* Enable the GPU. */
		gpu(1);
	}

	/* Fill in hardware capabilities. */
	data->capabilities.data = 0;

	/* Allocate the command buffer. */
	if (data->command_buffer_size) {
		for (i = 0; i < 2; i++) {
			/* Allocate the memory. */
			error = vg_lite_hal_allocate_contiguous(
				data->command_buffer_size,
				&context->command_buffer_logical[i],
				&context->command_buffer_klogical[i],
				&context->command_buffer_physical[i],
				&context->command_buffer[i]);
			if (error != VG_LITE_SUCCESS) {
				/* Free any allocated memory. */
				struct vg_lite_kernel_terminate terminate = {
					context
				};
				do_terminate(&terminate);

				/* Out of memory. */
				return error;
			}

			/* Return command buffer logical pointer and GPU address. */
			data->command_buffer[i] =
				context->command_buffer_logical[i];
			data->command_buffer_gpu[i] =
				context->command_buffer_physical[i];
		}
	}

	/* Allocate the tessellation buffer. */
	if ((data->tess_width > 0) && (data->tess_height > 0)) {
		int width = data->tess_width;
		int height = 0;
		int vg_countbuffer_size = 0, total_size = 0, ts_buffer_size = 0;

		height = VG_LITE_ALIGN(data->tess_height, 16);

#if (CHIPID == 0x355 || CHIPID == 0x255)
		{
			unsigned long stride, buffer_size, l1_size, l2_size;
#if (CHIPID == 0x355)
			data->capabilities.cap.l2_cache = 1;
			width = VG_LITE_ALIGN(width, 128);
#endif
			/* Check if we can used tiled tessellation (128x16). */
			if (((width & 127) == 0) && ((height & 15) == 0))
				data->capabilities.cap.tiled = 0x3;
			else
				data->capabilities.cap.tiled = 0x2;

			/* Compute tessellation buffer size. */
			stride = VG_LITE_ALIGN(width * 8, 64);
			buffer_size = VG_LITE_ALIGN(stride * height, 64);
			/* Each bit in the L1 cache represents 64 bytes of tessellation data. */
			l1_size = VG_LITE_ALIGN(
				VG_LITE_ALIGN(buffer_size / 64, 64) / 8, 64);
#if (CHIPID == 0x355)
			/* Each bit in the L2 cache represents 32 bytes of L1 data. */
			l2_size = VG_LITE_ALIGN(
				VG_LITE_ALIGN(l1_size / 32, 64) / 8, 64);
#else
			l2_size = 0;
#endif
			total_size = buffer_size + l1_size + l2_size;
			ts_buffer_size = buffer_size;
		}
#else /* (CHIPID==0x355 || CHIPID==0x255) */
		{
			/* Check if we can used tiled tessellation (128x16). */
			if (((width & 127) == 0) && ((height & 15) == 0))
				data->capabilities.cap.tiled = 0x3;
			else
				data->capabilities.cap.tiled = 0x2;

			vg_countbuffer_size = height * 3;
			vg_countbuffer_size =
				VG_LITE_ALIGN(vg_countbuffer_size, 64);
			total_size = height * 128;
			if (total_size < MIN_TS_SIZE)
				total_size = MIN_TS_SIZE;
			ts_buffer_size = total_size - vg_countbuffer_size;
		}
#endif /* (CHIPID==0x355 || CHIPID==0x255) */

		/* Allocate the memory. */
		error = vg_lite_hal_allocate_contiguous(
			total_size, &context->tessbuf_logical,
			&context->tessbuf_klogical, &context->tessbuf_physical,
			&context->tess_buffer);
		if (error != VG_LITE_SUCCESS) {
			/* Free any allocated memory. */
			struct vg_lite_kernel_terminate terminate = { context };

			do_terminate(&terminate);
			return error;
		}

		/* Return the tessellation buffer pointers and GPU addresses. */
		data->physical_addr = context->tessbuf_physical;
		data->logical_addr = (uint8_t *)context->tessbuf_logical;
		data->tessbuf_size = ts_buffer_size;
		data->countbuf_size = vg_countbuffer_size;
		data->tess_w_h = width | (height << 16);
	}

	/* Enable all interrupts. */
	vg_lite_hal_poke(VG_LITE_INTR_ENABLE, 0xFFFFFFFF);

#if defined(__linux__) && !defined(EMULATOR)
	if (copy_to_user(context_usr, context,
			 sizeof(struct vg_lite_kernel_context)) != 0) {
		// Free any allocated memory.
		struct vg_lite_kernel_terminate terminate = { context };

		do_terminate(&terminate);

		return VG_LITE_NO_CONTEXT;
	}
#endif
	return error;
}

static enum vg_lite_error do_initialize(struct vg_lite_kernel_initialize *data)
{
	enum vg_lite_error error = VG_LITE_SUCCESS;
	/* Free any allocated memory for the context. */
	do {
		error = init_vglite(data);
		if (error != VG_LITE_SUCCESS)
			break;

		error = init_3rd(data);
		if (error != VG_LITE_SUCCESS)
			break;
	} while (0);

	return error;
}

static enum vg_lite_error terminate_vglite(struct vg_lite_kernel_terminate *data)
{
	struct vg_lite_kernel_context *context = NULL;
#if defined(__linux__) && !defined(EMULATOR)
	struct vg_lite_kernel_context mycontext = { 0 };

	if (copy_from_user(&mycontext, data->context,
			   sizeof(struct vg_lite_kernel_context)) != 0) {
		return VG_LITE_NO_CONTEXT;
	}
	context = &mycontext;
#else
	context = data->context;
#endif

	/* Free any allocated memory for the context. */
	if (context->command_buffer[0]) {
		/* Free the command buffer. */
		vg_lite_hal_free_contiguous(context->command_buffer[0]);
		context->command_buffer[0] = NULL;
	}

	if (context->command_buffer[1]) {
		/* Free the command buffer. */
		vg_lite_hal_free_contiguous(context->command_buffer[1]);
		context->command_buffer[1] = NULL;
	}

	if (context->tess_buffer) {
		/* Free the tessellation buffer. */
		vg_lite_hal_free_contiguous(context->tess_buffer);
		context->tess_buffer = NULL;
	}
	vg_lite_hal_free_os_heap();
	/* Decrement reference counter. */
	if (--s_reference == 0) {
		/* Disable the GPU. */
		gpu(0);

		/* De-initialize the SOC. */
		vg_lite_hal_deinitialize();
	}

#if defined(__linux__) && !defined(EMULATOR)
	if (copy_to_user((struct vg_lite_kernel_context __user *)data->context,
			 &mycontext, sizeof(struct vg_lite_kernel_context)) != 0) {
		return VG_LITE_NO_CONTEXT;
	}
#endif
	return VG_LITE_SUCCESS;
}

static enum vg_lite_error terminate_3rd(struct vg_lite_kernel_terminate *data)
{
	/* TODO: Terminate the converters. */

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_terminate(struct vg_lite_kernel_terminate *data)
{
	terminate_vglite(data);
	terminate_3rd(data);

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_allocate(struct vg_lite_kernel_allocate *data)
{
	enum vg_lite_error error;

	error = vg_lite_hal_allocate_contiguous(data->bytes, &data->memory,
						&data->kmemory,
						&data->memory_gpu,
						&data->memory_handle);
	return error;
}

static enum vg_lite_error do_free(struct vg_lite_kernel_free *data)
{
	vg_lite_hal_free_contiguous(data->memory_handle);

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_submit(struct vg_lite_kernel_submit *data)
{
	uint32_t offset;
	struct vg_lite_kernel_context *context = NULL;
	uint32_t physical =
		data->context->command_buffer_physical[data->command_id];

#if defined(__linux__) && !defined(EMULATOR)
	struct vg_lite_kernel_context mycontext = { 0 };

	if (copy_from_user(&mycontext, data->context,
			   sizeof(struct vg_lite_kernel_context)) != 0) {
		return VG_LITE_NO_CONTEXT;
	}
	context = &mycontext;
	physical = context->command_buffer_physical[data->command_id];
#else
	context = data->context;
	if (context == NULL)
		return VG_LITE_NO_CONTEXT;
#endif
	/* Perform a memory barrier. */
	vg_lite_hal_barrier();

	offset = (uint8_t *)data->commands -
		 (uint8_t *)context->command_buffer_logical[data->command_id];

	/* Write the registers to kick off the command execution (CMDBUF_SIZE). */
	vg_lite_hal_poke(VG_LITE_HW_CMDBUF_ADDRESS, physical + offset);
	vg_lite_hal_poke(VG_LITE_HW_CMDBUF_SIZE, (data->command_size + 7) / 8);

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_wait(struct vg_lite_kernel_wait *data)
{
	/* Wait for interrupt. */
	if (!vg_lite_hal_wait_interrupt(data->timeout_ms, data->event_mask,
					&data->event_got)) {
		/* Timeout. */
		return VG_LITE_TIMEOUT;
	}

#if gcFEATURE_VG_FLEXA
	if (data->event_got & FLEXA_TIMEOUT_STATE)
		return VG_LITE_FLEXA_TIME_OUT;

	if (data->event_got & FLEXA_HANDSHEKE_FAIL_STATE)
		return VG_LITE_FLEXA_HANDSHAKE_FAIL;
#endif

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_reset(void)
{
	/* Disable and enable the GPU. */
	gpu(1);
	vg_lite_hal_poke(VG_LITE_INTR_ENABLE, 0xFFFFFFFF);

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_gpu_close(void)
{
	gpu(0);

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_debug(void)
{
	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_map(struct vg_lite_kernel_map *data)
{
	data->memory_handle = vg_lite_hal_map(data->flags, data->bytes,
					      data->logical, data->physical,
					      data->dma_buf_fd,
					      &data->memory_gpu);
	if (data->memory_handle == NULL)
		return VG_LITE_OUT_OF_RESOURCES;

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_unmap(struct vg_lite_kernel_unmap *data)
{
	vg_lite_hal_unmap(data->memory_handle);

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_peek(struct vg_lite_kernel_info *data)
{
	data->reg = vg_lite_hal_peek(data->addr);

	return VG_LITE_SUCCESS;
}

#if gcFEATURE_VG_FLEXA
static enum vg_lite_error do_flexa_enable(vg_lite_kernel_flexa_info_t *data)
{
	/* reset all flexa states */
	vg_lite_hal_poke(0x03600, 0x0);
	/* set sync mode */
	vg_lite_hal_poke(0x03604, data->segment_address);

	vg_lite_hal_poke(0x03608, data->segment_count);

	vg_lite_hal_poke(0x0360C, data->segment_size);

	vg_lite_hal_poke(0x0520, data->sync_mode);

	vg_lite_hal_poke(0x03610, data->stream_id | data->sbi_mode |
					  data->start_flag | data->stop_flag |
					  data->reset_flag);

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error
do_flexa_set_background_address(vg_lite_kernel_flexa_info_t *data)
{
	vg_lite_hal_poke(0x03604, data->segment_address);

	vg_lite_hal_poke(0x03608, data->segment_count);

	vg_lite_hal_poke(0x0360C, data->segment_size);

	vg_lite_hal_poke(0x03610, data->stream_id | data->sbi_mode |
					  data->start_flag | data->stop_flag |
					  data->reset_flag);

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_flexa_disable(vg_lite_kernel_flexa_info_t *data)
{
	vg_lite_hal_poke(0x0520, data->sync_mode);

	vg_lite_hal_poke(0x03610, data->stream_id | data->sbi_mode);

	/* reset all flexa states */
	vg_lite_hal_poke(0x03600, 0x0);

	return VG_LITE_SUCCESS;
}

static enum vg_lite_error do_flexa_stop_frame(vg_lite_kernel_flexa_info_t *data)
{
	vg_lite_hal_poke(0x03610, data->stream_id | data->sbi_mode |
					  data->start_flag | data->stop_flag |
					  data->reset_flag);

	return VG_LITE_SUCCESS;
}
#endif

static enum vg_lite_error do_query_mem(struct vg_lite_kernel_mem *data)
{
	enum vg_lite_error error = VG_LITE_SUCCESS;

	error = vg_lite_hal_query_mem(data);

	return error;
}

static enum vg_lite_error do_map_memory(struct vg_lite_kernel_map_memory *data)
{
	enum vg_lite_error error = VG_LITE_SUCCESS;

	error = vg_lite_hal_map_memory(data);

	return error;
}

static enum vg_lite_error do_unmap_memory(struct vg_lite_kernel_unmap_memory *data)
{
	enum vg_lite_error error = VG_LITE_SUCCESS;

	error = vg_lite_hal_unmap_memory(data);

	return error;
}

static enum vg_lite_error do_cache(struct vg_lite_kernel_cache *data)
{
	enum vg_lite_error error = VG_LITE_SUCCESS;

	error = vg_lite_hal_operation_cache(data->memory_handle,
					    data->cache_op);

	return error;
}

static enum vg_lite_error do_export_memory(struct vg_lite_kernel_export_memory *data)
{
	enum vg_lite_error error = VG_LITE_SUCCESS;

	error = vg_lite_hal_memory_export(&data->fd);

	return error;
}

static void soft_reset(void)
{
	union vg_lite_hw_clock_control value;

	value.data = vg_lite_hal_peek(VG_LITE_HW_CLOCK_CONTROL);

	/* Perform a soft reset. */
	value.control.isolate = 1;
	vg_lite_hal_poke(VG_LITE_HW_CLOCK_CONTROL, value.data);
	value.control.soft_reset = 1;
	vg_lite_hal_poke(VG_LITE_HW_CLOCK_CONTROL, value.data);
	vg_lite_hal_delay(5);
	value.control.soft_reset = 0;
	vg_lite_hal_poke(VG_LITE_HW_CLOCK_CONTROL, value.data);
	value.control.isolate = 0;
	vg_lite_hal_poke(VG_LITE_HW_CLOCK_CONTROL, value.data);
}

enum vg_lite_error vg_lite_kernel(enum vg_lite_kernel_command command, void *data)
{
	/* Dispatch on command. */
	switch (command) {
	case VG_LITE_INITIALIZE:
		/* Initialize the context. */
		return do_initialize(data);

	case VG_LITE_TERMINATE:
		/* Terminate the context. */
		return do_terminate(data);

	case VG_LITE_ALLOCATE:
		/* Allocate contiguous memory. */
		return do_allocate(data);

	case VG_LITE_FREE:
		/* Free contiguous memory. */
		return do_free(data);

	case VG_LITE_SUBMIT:
		/* Submit a command buffer. */
		return do_submit(data);

	case VG_LITE_WAIT:
		/* Wait for the GPU. */
		return do_wait(data);

	case VG_LITE_RESET:
		/* Reset the GPU. */
		return do_reset();

	case VG_LITE_DEBUG:
		/* Perform debugging features. */
		return do_debug();

	case VG_LITE_MAP:
		/* Map some memory. */
		return do_map(data);

	case VG_LITE_UNMAP:
		/* Unmap some memory. */
		return do_unmap(data);

		/* Get register info. */
	case VG_LITE_CHECK:
		/* Get register value. */
		return do_peek(data);

#if gcFEATURE_VG_FLEXA
	case VG_LITE_FLEXA_DISABLE:
		/* Write register value. */
		return do_flexa_disable(data);

	case VG_LITE_FLEXA_ENABLE:
		/* Write register value. */
		return do_flexa_enable(data);

	case VG_LITE_FLEXA_STOP_FRAME:
		/* Write register value. */
		return do_flexa_stop_frame(data);

	case VG_LITE_FLEXA_SET_BACKGROUND_ADDRESS:
		/* Write register value. */
		return do_flexa_set_background_address(data);
#endif

	case VG_LITE_QUERY_MEM:
		return do_query_mem(data);

	case VG_LITE_MAP_MEMORY:
		/* Map memory to user */
		return do_map_memory(data);

	case VG_LITE_UNMAP_MEMORY:
		/* Unmap memory to user */
		return do_unmap_memory(data);

	case VG_LITE_CLOSE:
		return do_gpu_close();

	case VG_LITE_CACHE:
		return do_cache(data);

	case VG_LITE_EXPORT_MEMORY:
		return do_export_memory(data);

	default:
		break;
	}

	/* Invalid command. */
	return VG_LITE_INVALID_ARGUMENT;
}
