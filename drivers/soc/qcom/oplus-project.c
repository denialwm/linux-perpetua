// SPDX-License-Identifier: GPL-2.0-only
/*
 * The project information OnePlus bootloaders leave in SMEM: the handset's
 * project number, PCB version and RF variant. The modem's storage service
 * hands them to the modem at start, as stock Android's does.
 */

#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/smem.h>
#include <linux/sysfs.h>

#define SMEM_OPLUS_PROJECT	135

/* The start of the bootloader's OCDT record */
struct oplus_ocdt {
	__le32 version;
	/* BCDT */
	__le32 project;
	__le32 dtsi;
	__le32 audio;
	u8 feature[40];
	/* SCDT */
	__le32 boot_mode;
	__le32 rf;
	__le32 pcb;
} __packed;

struct oplus_project {
	u32 project;
	u32 pcb;
	u32 rf;
};

#define OPLUS_PROJECT_ATTR(name)					\
static ssize_t name##_show(struct device *dev,				\
			   struct device_attribute *attr, char *buf)	\
{									\
	struct oplus_project *p = dev_get_drvdata(dev);			\
									\
	return sysfs_emit(buf, "%u\n", p->name);			\
}									\
static DEVICE_ATTR_RO(name)

OPLUS_PROJECT_ATTR(project);
OPLUS_PROJECT_ATTR(pcb);
OPLUS_PROJECT_ATTR(rf);

static struct attribute *oplus_project_attrs[] = {
	&dev_attr_project.attr,
	&dev_attr_pcb.attr,
	&dev_attr_rf.attr,
	NULL
};
ATTRIBUTE_GROUPS(oplus_project);

static int oplus_project_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct oplus_ocdt *ocdt;
	struct oplus_project *p;
	size_t size;

	ocdt = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_OPLUS_PROJECT, &size);
	if (IS_ERR(ocdt))
		return dev_err_probe(dev, PTR_ERR(ocdt), "no project information in SMEM\n");
	if (size < sizeof(*ocdt))
		return dev_err_probe(dev, -EINVAL, "project information is %zu bytes\n", size);

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->project = le32_to_cpu(ocdt->project);
	p->pcb = le32_to_cpu(ocdt->pcb);
	p->rf = le32_to_cpu(ocdt->rf);
	platform_set_drvdata(pdev, p);

	dev_info(dev, "project %u, PCB %u, RF %u\n", p->project, p->pcb, p->rf);

	return 0;
}

static const struct of_device_id oplus_project_of_match[] = {
	{ .compatible = "oneplus,project-info" },
	{}
};
MODULE_DEVICE_TABLE(of, oplus_project_of_match);

static struct platform_driver oplus_project_driver = {
	.probe = oplus_project_probe,
	.driver = {
		.name = "oplus-project",
		.of_match_table = oplus_project_of_match,
		.dev_groups = oplus_project_groups,
	},
};
module_platform_driver(oplus_project_driver);

MODULE_DESCRIPTION("OnePlus project information from SMEM");
MODULE_LICENSE("GPL");
