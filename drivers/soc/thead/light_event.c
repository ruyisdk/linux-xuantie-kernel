// SPDX-License-Identifier: GPL-2.0-only
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/firmware/thead/ipc.h>
#include <linux/firmware/thead/light_event.h>

/*
 * AON SRAM total size is 0x10000, reserve 0x100 for event.
 * Notice: c902 *.ld also need resize.
 * -------------- 0xff_ffef8000
 * |		|
 * |		|
 * |		|
 * |    c902	|
 * |		|
 * |		|
 * |		|
 * -------------- 0xff_fff07f00
 * |   reserve	|
 * |		|
 * --------------
 */
#define LIGHT_AON_SRAM_LEN	0x10000
#define LIGHT_AON_SRAM_RESERV	(LIGHT_AON_SRAM_LEN - 0x100)
#define LIGHT_EVENT_OFFSET	(LIGHT_AON_SRAM_RESERV + 0x10)
#define LIGHT_EVENT_CHECK	(LIGHT_EVENT_OFFSET + 0x4)

#define LIGHT_EVENT_MAGIC	0x5A5A5A5A

struct light_aon_msg_event_ctrl {
	struct light_aon_rpc_msg_hdr hdr;
	u32 reserve_offset;
	u32 reserved[5];
} __packed __aligned(4);

struct light_event {
	struct device *dev;

	struct light_aon_ipc *ipc_handle;
	struct light_aon_msg_event_ctrl msg;

	struct regmap *aon_iram;
	bool init;
};

struct light_event *light_event;

static void light_event_msg_hdr_fill(struct light_aon_rpc_msg_hdr *hdr, enum light_aon_misc_func func)
{
	hdr->ver = LIGHT_AON_RPC_VERSION;
	hdr->svc = (uint8_t)LIGHT_AON_RPC_SVC_MISC;
	hdr->func = (uint8_t)func;
	hdr->size = LIGHT_AON_RPC_MSG_NUM;
}

static int light_event_aon_reservemem(struct light_event *event)
{
	struct light_aon_ipc *ipc = event->ipc_handle;
	int ret = 0;

	dev_dbg(event->dev, "aon reservemem...\n");

	light_event_msg_hdr_fill(&event->msg.hdr, LIGHT_AON_MISC_FUNC_AON_RESERVE_MEM);
	event->msg.reserve_offset = LIGHT_EVENT_OFFSET;

	ret = light_aon_call_rpc(ipc, &event->msg, true);
	if (ret)
		dev_err(event->dev, "failed to set aon reservemem\n");

	return ret;
}

int light_event_set_rebootmode(enum light_rebootmode_index mode)
{
	int ret;

	if (!light_event || !light_event->init)
		return -EINVAL;

	ret = regmap_write(light_event->aon_iram, LIGHT_EVENT_OFFSET, mode);
	if (ret) {
		dev_err(light_event->dev, "set rebootmode failed,ret:%d\n", ret);
		return ret;
	}

	dev_info(light_event->dev, "set rebootmode:0x%x\n", mode);

	return 0;
}
EXPORT_SYMBOL_GPL(light_event_set_rebootmode);

int light_event_get_rebootmode(enum light_rebootmode_index *mode)
{
	int ret;

	if (!light_event || !light_event->init)
		return -EINVAL;

	ret = regmap_read(light_event->aon_iram, LIGHT_EVENT_OFFSET, mode);
	if (ret) {
		dev_err(light_event->dev, "get rebootmode failed,ret:%d\n", ret);
		return ret;
	}
	dev_dbg(light_event->dev, "%s get rebootmode:0x%x\n", __func__, *mode);

	return 0;
}
EXPORT_SYMBOL_GPL(light_event_get_rebootmode);

static int light_event_check_powerup(void)
{
	enum light_rebootmode_index mode;
	unsigned int val;
	int ret;

	if (!light_event->init)
		return -EINVAL;

	ret = regmap_read(light_event->aon_iram, LIGHT_EVENT_CHECK, &val);
	if (ret) {
		dev_err(light_event->dev, "get magicnum failed,ret:%d\n", ret);
		return ret;
	}
	ret = regmap_read(light_event->aon_iram, LIGHT_EVENT_OFFSET, &mode);
	if (ret) {
		dev_err(light_event->dev, "get rebootmode failed,ret:%d\n", ret);
		return ret;
	}
	dev_info(light_event->dev, "magicnum:0x%x mode:0x%x\n", val, mode);

	/* powerup means SRAM data is randam */
	if (val != LIGHT_EVENT_MAGIC && mode != LIGHT_EVENT_PMIC_ONKEY)
		light_event_set_rebootmode(LIGHT_EVENT_PMIC_POWERUP);

	ret = regmap_write(light_event->aon_iram, LIGHT_EVENT_CHECK, LIGHT_EVENT_MAGIC);
	if (ret) {
		dev_err(light_event->dev, "set magicnum failed,ret:%d\n", ret);
		return ret;
	}

	return 0;
}

static ssize_t rebootmode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	enum light_rebootmode_index mode;

	if (kstrtouint(buf, 0, &mode) < 0)
		return -EINVAL;
	light_event_set_rebootmode(mode);

	return count;
}

static ssize_t
rebootmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	enum light_rebootmode_index mode;

	light_event_get_rebootmode(&mode);

	return sprintf(buf, "0x%x\n", mode);
}
static DEVICE_ATTR_RW(rebootmode);

static struct attribute *event_attrs[] = {
	&dev_attr_rebootmode.attr,
	NULL
};
ATTRIBUTE_GROUPS(event);

static int light_event_open(struct inode *inode, struct file *f)
{
	return 0;
}

static int light_event_release(struct inode *inode, struct file *f)
{
	return 0;
}

static long light_event_ioctl(struct file *f, unsigned int ioctl,
			    unsigned long arg)
{
	return 0;
}

static const struct file_operations light_event_fops = {
	.owner          = THIS_MODULE,
	.release        = light_event_release,
	.open           = light_event_open,
	.unlocked_ioctl = light_event_ioctl,
};

static struct miscdevice light_event_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "light-event",
	.fops = &light_event_fops,
};

static int light_event_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct device_node	*np = dev->of_node;
	struct light_event	*thead;
	int			ret;

	thead = devm_kzalloc(&pdev->dev, sizeof(*thead), GFP_KERNEL);
	if (!thead)
		return -ENOMEM;

	ret = light_aon_get_handle(&(thead->ipc_handle));
	if (ret == -EPROBE_DEFER)
		return ret;

	platform_set_drvdata(pdev, thead);
	thead->dev = &pdev->dev;

	thead->aon_iram = syscon_regmap_lookup_by_phandle(np, "aon-iram-regmap");
	if (IS_ERR(thead->aon_iram))
		return PTR_ERR(thead->aon_iram);

	ret = misc_register(&light_event_misc);
	if (ret < 0)
		return ret;

	ret = light_event_aon_reservemem(thead);
	if (ret) {
		dev_err(dev, "set aon reservemem failed!\n");
		return -EPERM;
	}
	thead->init = true;
	light_event = thead;

	ret = light_event_check_powerup();
	if (ret) {
		dev_err(dev, "check powerup failed!\n");
		light_event = NULL;
		return -EPERM;
	}
	dev_info(dev, "light-event driver init successfully\n");

	return 0;
}

static int light_event_remove(struct platform_device *pdev)
{
	misc_deregister(&light_event_misc);

	return 0;
}

static const struct of_device_id light_event_of_match[] = {
	{ .compatible = "thead,light-event" },
	{ },
};
MODULE_DEVICE_TABLE(of, light_event_of_match);

static struct platform_driver light_event_driver = {
	.probe		= light_event_probe,
	.remove		= light_event_remove,
	.driver		= {
		.name	= "light-event",
		.dev_groups	= event_groups,
		.of_match_table	= light_event_of_match,
	},
};

module_platform_driver(light_event_driver);

MODULE_DESCRIPTION("light-event driver");
MODULE_LICENSE("GPL v2");
