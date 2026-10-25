// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AWP1921 piezoelectric micropump driver
 *
 * The AWP1921 drives the piezo micropump of the liquid cooling loop in the
 * Nubia RedMagic 11 Pro. It is exposed as a hwmon pwm1 channel and as a
 * thermal cooling device with the off/low/mid/high levels of the vendor
 * driver.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

#define AWP1921_REG_RESET	0x01
#define AWP1921_RESET_MAGIC	0x5283
#define AWP1921_REG_STATUS	0x02
#define AWP1921_STATUS_FAULT	BIT(5)
#define AWP1921_REG_PLAY	0x06
#define AWP1921_PLAY_START	BIT(2)
#define AWP1921_PLAY_STOP	BIT(3)
#define AWP1921_REG_CFG		0x13
#define AWP1921_CFG_DEFAULT	0x401f
#define AWP1921_REG_AMP		0x14
#define AWP1921_AMP_LEVEL	GENMASK(6, 2)
#define AWP1921_AMP_EN		BIT(1)
#define AWP1921_REG_FREQ	0x15
#define AWP1921_FREQ_DEFAULT	4
#define AWP1921_REG_CHIP_ID	0x1e
#define AWP1921_CHIP_ID		0x5081

#define AWP1921_AMP_MAX		27
#define AWP1921_CHECK_INTERVAL	(60 * HZ)

/* off, low, mid, high as used by the vendor driver */
static const u8 awp1921_levels[] = { 0, 18, 20, 24 };

struct awp1921 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	struct delayed_work check_work;
	struct mutex lock;
	unsigned int amp;	/* 0 = off */
	bool running;
};

static int awp1921_start(struct awp1921 *awp)
{
	int ret;

	gpiod_set_value_cansleep(awp->enable_gpio, 1);
	usleep_range(1000, 2000);

	ret = regmap_write(awp->regmap, AWP1921_REG_RESET, AWP1921_RESET_MAGIC);
	if (ret)
		goto err;
	/* The vendor driver waits about this long after the reset */
	msleep(460);

	ret = regmap_write(awp->regmap, AWP1921_REG_CFG, AWP1921_CFG_DEFAULT);
	if (ret)
		goto err;
	ret = regmap_write(awp->regmap, AWP1921_REG_AMP, AWP1921_AMP_EN |
			   FIELD_PREP(AWP1921_AMP_LEVEL, awp->amp));
	if (ret)
		goto err;
	ret = regmap_write(awp->regmap, AWP1921_REG_FREQ, AWP1921_FREQ_DEFAULT);
	if (ret)
		goto err;
	ret = regmap_write(awp->regmap, AWP1921_REG_PLAY, AWP1921_PLAY_START);
	if (ret)
		goto err;

	awp->running = true;
	schedule_delayed_work(&awp->check_work, AWP1921_CHECK_INTERVAL);
	return 0;

err:
	gpiod_set_value_cansleep(awp->enable_gpio, 0);
	dev_err(awp->dev, "failed to start pump: %d\n", ret);
	return ret;
}

static void awp1921_stop(struct awp1921 *awp)
{
	if (!awp->running)
		return;

	regmap_write(awp->regmap, AWP1921_REG_PLAY, AWP1921_PLAY_STOP);
	gpiod_set_value_cansleep(awp->enable_gpio, 0);
	awp->running = false;
}

static int awp1921_set_amp(struct awp1921 *awp, unsigned int amp)
{
	unsigned int old = awp->amp;
	int ret = 0;

	awp->amp = amp;

	if (!amp) {
		cancel_delayed_work(&awp->check_work);
		awp1921_stop(awp);
	} else if (!awp->running) {
		ret = awp1921_start(awp);
	} else if (amp != old) {
		ret = regmap_write(awp->regmap, AWP1921_REG_AMP, AWP1921_AMP_EN |
				   FIELD_PREP(AWP1921_AMP_LEVEL, amp));
	}

	if (ret)
		awp->amp = awp->running ? old : 0;

	return ret;
}

static void awp1921_check_work(struct work_struct *work)
{
	struct awp1921 *awp = container_of(work, struct awp1921,
					   check_work.work);
	unsigned int status;

	mutex_lock(&awp->lock);
	if (!awp->running)
		goto out;

	if (!regmap_read(awp->regmap, AWP1921_REG_STATUS, &status) &&
	    !(status & AWP1921_STATUS_FAULT)) {
		schedule_delayed_work(&awp->check_work, AWP1921_CHECK_INTERVAL);
		goto out;
	}

	dev_warn(awp->dev, "pump fault (status %#x), restarting\n", status);
	awp1921_stop(awp);
	awp1921_start(awp);
out:
	mutex_unlock(&awp->lock);
}

static int awp1921_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	struct awp1921 *awp = dev_get_drvdata(dev);

	if (type != hwmon_pwm || attr != hwmon_pwm_input)
		return -EOPNOTSUPP;

	*val = DIV_ROUND_CLOSEST(awp->amp * 255, AWP1921_AMP_MAX);
	return 0;
}

static int awp1921_write(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long val)
{
	struct awp1921 *awp = dev_get_drvdata(dev);
	int ret;

	if (type != hwmon_pwm || attr != hwmon_pwm_input)
		return -EOPNOTSUPP;
	if (val < 0 || val > 255)
		return -EINVAL;

	mutex_lock(&awp->lock);
	ret = awp1921_set_amp(awp, DIV_ROUND_CLOSEST(val * AWP1921_AMP_MAX, 255));
	mutex_unlock(&awp->lock);

	return ret;
}

static umode_t awp1921_is_visible(const void *data,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	if (type == hwmon_pwm && attr == hwmon_pwm_input)
		return 0644;

	return 0;
}

static const struct hwmon_ops awp1921_hwmon_ops = {
	.is_visible = awp1921_is_visible,
	.read = awp1921_read,
	.write = awp1921_write,
};

static const struct hwmon_channel_info * const awp1921_info[] = {
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT),
	NULL
};

static const struct hwmon_chip_info awp1921_chip_info = {
	.ops = &awp1921_hwmon_ops,
	.info = awp1921_info,
};

static int awp1921_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	*state = ARRAY_SIZE(awp1921_levels) - 1;
	return 0;
}

static int awp1921_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct awp1921 *awp = cdev->devdata;
	unsigned long i;

	for (i = ARRAY_SIZE(awp1921_levels) - 1; i > 0; i--)
		if (awp->amp >= awp1921_levels[i])
			break;

	*state = i;
	return 0;
}

static int awp1921_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state)
{
	struct awp1921 *awp = cdev->devdata;
	int ret;

	if (state >= ARRAY_SIZE(awp1921_levels))
		return -EINVAL;

	mutex_lock(&awp->lock);
	ret = awp1921_set_amp(awp, awp1921_levels[state]);
	mutex_unlock(&awp->lock);

	return ret;
}

static const struct thermal_cooling_device_ops awp1921_cooling_ops = {
	.get_max_state = awp1921_get_max_state,
	.get_cur_state = awp1921_get_cur_state,
	.set_cur_state = awp1921_set_cur_state,
};

static const struct regmap_config awp1921_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0x1f,
};

static void awp1921_cleanup(void *data)
{
	struct awp1921 *awp = data;

	mutex_lock(&awp->lock);
	awp->amp = 0;
	awp1921_stop(awp);
	mutex_unlock(&awp->lock);
	cancel_delayed_work_sync(&awp->check_work);
}

static int awp1921_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct thermal_cooling_device *cdev;
	struct device *hwmon;
	struct awp1921 *awp;
	unsigned int id;
	int ret;

	awp = devm_kzalloc(dev, sizeof(*awp), GFP_KERNEL);
	if (!awp)
		return -ENOMEM;

	awp->dev = dev;
	mutex_init(&awp->lock);
	INIT_DELAYED_WORK(&awp->check_work, awp1921_check_work);
	i2c_set_clientdata(client, awp);

	awp->regmap = devm_regmap_init_i2c(client, &awp1921_regmap_config);
	if (IS_ERR(awp->regmap))
		return PTR_ERR(awp->regmap);

	awp->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(awp->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(awp->enable_gpio),
				     "failed to get enable gpio\n");
	usleep_range(1000, 2000);

	ret = regmap_read(awp->regmap, AWP1921_REG_CHIP_ID, &id);
	gpiod_set_value_cansleep(awp->enable_gpio, 0);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip id\n");
	if (id != AWP1921_CHIP_ID)
		return dev_err_probe(dev, -ENODEV, "unknown chip id %#x\n", id);

	ret = devm_add_action_or_reset(dev, awp1921_cleanup, awp);
	if (ret)
		return ret;

	hwmon = devm_hwmon_device_register_with_info(dev, "awp1921", awp,
						     &awp1921_chip_info, NULL);
	if (IS_ERR(hwmon))
		return dev_err_probe(dev, PTR_ERR(hwmon),
				     "failed to register hwmon device\n");

	if (IS_ENABLED(CONFIG_THERMAL)) {
		cdev = devm_thermal_of_child_cooling_device_register(dev,
								     dev->of_node,
								     "awp1921", awp,
								     &awp1921_cooling_ops);
		if (IS_ERR(cdev))
			return dev_err_probe(dev, PTR_ERR(cdev),
					     "failed to register cooling device\n");
	}

	return 0;
}

static void awp1921_shutdown(struct i2c_client *client)
{
	awp1921_cleanup(i2c_get_clientdata(client));
}

static int awp1921_suspend(struct device *dev)
{
	struct awp1921 *awp = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&awp->check_work);
	mutex_lock(&awp->lock);
	awp1921_stop(awp);
	mutex_unlock(&awp->lock);

	return 0;
}

static int awp1921_resume(struct device *dev)
{
	struct awp1921 *awp = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&awp->lock);
	if (awp->amp)
		ret = awp1921_start(awp);
	mutex_unlock(&awp->lock);

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(awp1921_pm_ops, awp1921_suspend,
				awp1921_resume);

static const struct of_device_id awp1921_of_match[] = {
	{ .compatible = "awinic,awp1921" },
	{ }
};
MODULE_DEVICE_TABLE(of, awp1921_of_match);

static const struct i2c_device_id awp1921_id[] = {
	{ "awp1921" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, awp1921_id);

static struct i2c_driver awp1921_driver = {
	.driver = {
		.name = "awp1921",
		.of_match_table = awp1921_of_match,
		.pm = pm_sleep_ptr(&awp1921_pm_ops),
	},
	.probe = awp1921_probe,
	.shutdown = awp1921_shutdown,
	.id_table = awp1921_id,
};
module_i2c_driver(awp1921_driver);

MODULE_DESCRIPTION("Awinic AWP1921 piezo micropump driver");
MODULE_LICENSE("GPL");
