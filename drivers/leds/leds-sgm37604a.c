// SPDX-License-Identifier: GPL-2.0-only
/*
 * SG Micro SGM37604A four-string white LED driver
 */

#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define SGM37604A_ENABLE		0x10
#define  SGM37604A_DEV_EN		BIT(0)
#define  SGM37604A_LED_EN(n)		BIT(1 + (n))
#define SGM37604A_BRT_CTRL		0x11
#define  SGM37604A_LED_MOD		GENMASK(6, 5)
#define  SGM37604A_RAMP_EN		BIT(4)
#define SGM37604A_BRT_MSB		0x19
#define SGM37604A_BRT_LSB		0x1a
#define  SGM37604A_BRT_LSB_MASK		GENMASK(3, 0)
#define SGM37604A_FAULT			0x1f

#define SGM37604A_STRINGS		4
#define SGM37604A_MAX_BRIGHTNESS	4095

struct sgm37604a {
	struct led_classdev cdev;
	struct regmap *regmap;
};

static int sgm37604a_brightness_set(struct led_classdev *cdev,
				    enum led_brightness brightness)
{
	struct sgm37604a *chip = container_of(cdev, struct sgm37604a, cdev);
	int ret;

	/*
	 * Brightness from the I2C code alone, ramped. The default mode
	 * multiplies it with the PWM input, which the display controller
	 * drives only while its driver runs. Switch on the first change, so
	 * that the backlight keeps the bootloader's level until then.
	 */
	ret = regmap_update_bits(chip->regmap, SGM37604A_BRT_CTRL,
				 SGM37604A_LED_MOD | SGM37604A_RAMP_EN,
				 SGM37604A_RAMP_EN);
	if (ret)
		return ret;

	ret = regmap_write(chip->regmap, SGM37604A_BRT_LSB,
			   brightness & SGM37604A_BRT_LSB_MASK);
	if (ret)
		return ret;

	return regmap_write(chip->regmap, SGM37604A_BRT_MSB, brightness >> 4);
}

static bool sgm37604a_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg == SGM37604A_FAULT;
}

static const struct regmap_config sgm37604a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SGM37604A_FAULT,
	.volatile_reg = sgm37604a_volatile_reg,
	.cache_type = REGCACHE_MAPLE,
};

static int sgm37604a_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct led_init_data init_data = {};
	u32 sources[SGM37604A_STRINGS];
	unsigned int enable = SGM37604A_DEV_EN;
	unsigned int msb, lsb;
	struct sgm37604a *chip;
	int count, i, ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->regmap = devm_regmap_init_i2c(client, &sgm37604a_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "failed to init regmap\n");

	struct fwnode_handle *child __free(fwnode_handle) =
		device_get_next_child_node(dev, NULL);
	if (!child)
		return dev_err_probe(dev, -ENODEV, "no led node\n");

	/* All four strings unless the board names the ones it wires. */
	count = fwnode_property_count_u32(child, "led-sources");
	if (count > SGM37604A_STRINGS)
		return dev_err_probe(dev, -EINVAL, "too many led-sources\n");
	if (count > 0) {
		ret = fwnode_property_read_u32_array(child, "led-sources",
						     sources, count);
		if (ret)
			return ret;
		for (i = 0; i < count; i++) {
			if (sources[i] >= SGM37604A_STRINGS)
				return dev_err_probe(dev, -EINVAL,
						     "invalid led-source %u\n",
						     sources[i]);
			enable |= SGM37604A_LED_EN(sources[i]);
		}
	} else {
		for (i = 0; i < SGM37604A_STRINGS; i++)
			enable |= SGM37604A_LED_EN(i);
	}

	ret = regmap_write(chip->regmap, SGM37604A_ENABLE, enable);
	if (ret)
		return ret;

	/* Start from the brightness the bootloader left. */
	ret = regmap_read(chip->regmap, SGM37604A_BRT_MSB, &msb);
	if (ret)
		return ret;
	ret = regmap_read(chip->regmap, SGM37604A_BRT_LSB, &lsb);
	if (ret)
		return ret;

	chip->cdev.brightness = msb << 4 | (lsb & SGM37604A_BRT_LSB_MASK);
	chip->cdev.max_brightness = SGM37604A_MAX_BRIGHTNESS;
	chip->cdev.brightness_set_blocking = sgm37604a_brightness_set;

	init_data.fwnode = child;
	init_data.devicename = client->name;
	init_data.default_label = ":";

	return devm_led_classdev_register_ext(dev, &chip->cdev, &init_data);
}

static const struct of_device_id sgm37604a_of_match[] = {
	{ .compatible = "sgmicro,sgm37604a" },
	{ }
};
MODULE_DEVICE_TABLE(of, sgm37604a_of_match);

static const struct i2c_device_id sgm37604a_id[] = {
	{ .name = "sgm37604a" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sgm37604a_id);

static struct i2c_driver sgm37604a_driver = {
	.driver = {
		.name = "sgm37604a",
		.of_match_table = sgm37604a_of_match,
	},
	.probe = sgm37604a_probe,
	.id_table = sgm37604a_id,
};
module_i2c_driver(sgm37604a_driver);

MODULE_DESCRIPTION("SG Micro SGM37604A LED driver");
MODULE_LICENSE("GPL");
