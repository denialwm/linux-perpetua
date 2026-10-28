// SPDX-License-Identifier: GPL-2.0
/*
 * Dongwoon DW9785 lens focus driver
 *
 * The DW9785 drives both focus and optical image stabilisation of a camera
 * module from its own DSP. Only focus is driven here. Register use follows
 * the Nubia RedMagic 11 Pro (nx809j) stock start-up list and was checked on
 * that phone; structure follows dw9719.c.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define DW9785_REG_CHIP_ID		CCI_REG16(0x7000)
#define DW9785_CHIP_ID			0x9785
#define DW9785_REG_FW_VERSION		CCI_REG16(0x7001)

/* Focus target, 0 to 4095 */
#define DW9785_REG_FOCUS		CCI_REG16(0x7070)
#define DW9785_FOCUS_MAX		4095
#define DW9785_FOCUS_DEFAULT		2048

#define DW9785_REG_DSP_CTRL		CCI_REG16(0xd001)
#define DW9785_REG_LOGIC_RESET		CCI_REG16(0xd002)

/* Stock power-up order; they go down in reverse */
static const char * const dw9785_supply_names[] = {
	"vio",
	"vaf",
	"oisvdd",
};

#define DW9785_NUM_SUPPLIES	ARRAY_SIZE(dw9785_supply_names)

/* The rest of the stock start-up list, after the reset and the DSP start */
static const struct cci_reg_sequence dw9785_init[] = {
	{ CCI_REG16(0xdd03), 0x0002 },
	{ CCI_REG16(0xdd04), 0x0002 },
};

struct dw9785 {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[DW9785_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *focus;
};

#define to_dw9785(_sd)	container_of(_sd, struct dw9785, sd)

static void dw9785_power_off(struct dw9785 *dw9785)
{
	int i;

	for (i = DW9785_NUM_SUPPLIES - 1; i >= 0; i--)
		regulator_disable(dw9785->supplies[i].consumer);
}

static int dw9785_power_on(struct dw9785 *dw9785)
{
	int ret = 0;
	int i;

	for (i = 0; i < DW9785_NUM_SUPPLIES; i++) {
		ret = regulator_enable(dw9785->supplies[i].consumer);
		if (ret)
			goto err_disable;
	}
	fsleep(4000);

	cci_write(dw9785->regmap, DW9785_REG_LOGIC_RESET, 1, &ret);
	fsleep(4000);
	cci_write(dw9785->regmap, DW9785_REG_DSP_CTRL, 1, &ret);
	fsleep(25000);
	cci_multi_reg_write(dw9785->regmap, dw9785_init,
			    ARRAY_SIZE(dw9785_init), &ret);
	fsleep(1000);
	if (ret) {
		dev_err(dw9785->dev, "failed to start: %d\n", ret);
		dw9785_power_off(dw9785);
	}

	return ret;

err_disable:
	while (--i >= 0)
		regulator_disable(dw9785->supplies[i].consumer);
	return ret;
}

static int dw9785_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dw9785 *dw9785 = container_of(ctrl->handler, struct dw9785,
					     ctrls);
	int ret;

	/* Only apply changes to the controls if the device is powered up */
	if (!pm_runtime_get_if_in_use(dw9785->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		ret = cci_write(dw9785->regmap, DW9785_REG_FOCUS, ctrl->val,
				NULL);
		break;
	default:
		ret = -EINVAL;
	}

	pm_runtime_put(dw9785->dev);

	return ret;
}

static const struct v4l2_ctrl_ops dw9785_ctrl_ops = {
	.s_ctrl = dw9785_set_ctrl,
};

static int dw9785_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	dw9785_power_off(to_dw9785(sd));

	return 0;
}

static int dw9785_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct dw9785 *dw9785 = to_dw9785(sd);
	int ret;

	ret = dw9785_power_on(dw9785);
	if (ret)
		return ret;

	ret = cci_write(dw9785->regmap, DW9785_REG_FOCUS, dw9785->focus->val,
			NULL);
	if (ret)
		dw9785_power_off(dw9785);

	return ret;
}

static int dw9785_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int dw9785_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put_autosuspend(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops dw9785_internal_ops = {
	.open = dw9785_open,
	.close = dw9785_close,
};

static const struct v4l2_subdev_ops dw9785_ops = { };

static int dw9785_identify(struct dw9785 *dw9785)
{
	u64 id, version;
	int ret = 0;

	cci_read(dw9785->regmap, DW9785_REG_CHIP_ID, &id, &ret);
	cci_read(dw9785->regmap, DW9785_REG_FW_VERSION, &version, &ret);
	if (ret)
		return dev_err_probe(dw9785->dev, ret, "failed to read the chip ID\n");

	if (id != DW9785_CHIP_ID)
		return dev_err_probe(dw9785->dev, -ENODEV,
				     "unexpected chip ID 0x%04llx\n", id);

	dev_dbg(dw9785->dev, "firmware 0x%04llx\n", version);

	return 0;
}

static int dw9785_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9785 *dw9785;
	int ret;
	int i;

	dw9785 = devm_kzalloc(dev, sizeof(*dw9785), GFP_KERNEL);
	if (!dw9785)
		return -ENOMEM;

	dw9785->dev = dev;

	dw9785->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(dw9785->regmap))
		return PTR_ERR(dw9785->regmap);

	for (i = 0; i < DW9785_NUM_SUPPLIES; i++)
		dw9785->supplies[i].supply = dw9785_supply_names[i];

	ret = devm_regulator_bulk_get(dev, DW9785_NUM_SUPPLIES,
				      dw9785->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	v4l2_i2c_subdev_init(&dw9785->sd, client, &dw9785_ops);
	dw9785->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9785->sd.internal_ops = &dw9785_internal_ops;

	v4l2_ctrl_handler_init(&dw9785->ctrls, 1);
	dw9785->focus = v4l2_ctrl_new_std(&dw9785->ctrls, &dw9785_ctrl_ops,
					  V4L2_CID_FOCUS_ABSOLUTE, 0,
					  DW9785_FOCUS_MAX, 1,
					  DW9785_FOCUS_DEFAULT);
	if (dw9785->ctrls.error) {
		ret = dw9785->ctrls.error;
		goto err_free_ctrls;
	}
	dw9785->sd.ctrl_handler = &dw9785->ctrls;

	ret = media_entity_pads_init(&dw9785->sd.entity, 0, NULL);
	if (ret)
		goto err_free_ctrls;

	dw9785->sd.entity.function = MEDIA_ENT_F_LENS;

	ret = dw9785_resume(dev);
	if (ret)
		goto err_cleanup_media;

	ret = dw9785_identify(dw9785);
	if (ret)
		goto err_power_off;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev(&dw9785->sd);
	if (ret)
		goto err_pm_runtime;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_pm_runtime:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
err_power_off:
	dw9785_power_off(dw9785);
err_cleanup_media:
	media_entity_cleanup(&dw9785->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&dw9785->ctrls);

	return ret;
}

static void dw9785_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9785 *dw9785 = to_dw9785(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&dw9785->ctrls);
	media_entity_cleanup(&dw9785->sd.entity);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		dw9785_power_off(dw9785);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id dw9785_of_match[] = {
	{ .compatible = "dongwoon,dw9785" },
	{ }
};
MODULE_DEVICE_TABLE(of, dw9785_of_match);

static DEFINE_RUNTIME_DEV_PM_OPS(dw9785_pm_ops, dw9785_suspend, dw9785_resume,
				 NULL);

static struct i2c_driver dw9785_i2c_driver = {
	.driver = {
		.name = "dw9785",
		.pm = pm_sleep_ptr(&dw9785_pm_ops),
		.of_match_table = dw9785_of_match,
	},
	.probe = dw9785_probe,
	.remove = dw9785_remove,
};
module_i2c_driver(dw9785_i2c_driver);

MODULE_DESCRIPTION("Dongwoon DW9785 lens focus driver");
MODULE_LICENSE("GPL");
