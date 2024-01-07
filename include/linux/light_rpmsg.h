/*
 * Copyright (C) 2023 Alibaba Group Holding Limited.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*
 * @file linux/light_rpmsg.h
 *
 * @brief Global header file for imx RPMSG
 *
 * @ingroup RPMSG
 */
#ifndef __LINUX_LIGHT_RPMSG_H__
#define __LINUX_LIGHT_RPMSG_H__

#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>

/* Category define */
#define LIGHT_RMPSG_LIFECYCLE	1
#define LIGHT_RPMSG_PMIC		2
#define LIGHT_RPMSG_AUDIO		3
#define LIGHT_RPMSG_KEY		4
#define LIGHT_RPMSG_GPIO		5
#define LIGHT_RPMSG_RTC		6
#define LIGHT_RPMSG_SENSOR	7
/* rpmsg version */
#define LIGHT_RMPSG_MAJOR		1
#define LIGHT_RMPSG_MINOR		0

enum light_rpmsg_variants {
        LIGHTA,
        LIGHTB,
        LIGHT_RPMSG,
};

struct light_virdev {
        struct virtio_device vdev;
        unsigned int vring[2];
        struct virtqueue *vq[2];
        int base_vq_id;
        int num_of_vqs;
        struct notifier_block nb;
};

struct light_rpmsg_vproc {
        char *rproc_name;
        struct mutex lock;
        struct clk *mu_clk;
        enum light_rpmsg_variants variant;
        int vdev_nums;
        int first_notify;
#define MAX_VDEV_NUMS   8
        struct light_virdev ivdev[MAX_VDEV_NUMS];
        void __iomem *mu_base;
        struct delayed_work rpmsg_work;
        struct blocking_notifier_head notifier;
#define MAX_NUM 10      /* enlarge it if overflow happen */
        u32 m4_message[MAX_NUM];
        u32 in_idx;
        u32 out_idx;
        u32 core_id;
        spinlock_t mu_lock;
#ifdef CONFIG_PM_SLEEP
        struct semaphore pm_sem;
        int sleep_flag;
#endif
};

struct light_rpmsg_head {
	u8 cate;
	u8 major;
	u8 minor;
	u8 type;
	u8 cmd;
	u8 reserved[5];
} __attribute__ ((packed));

#endif /* __LINUX_LIGHT_RPMSG_H__*/

