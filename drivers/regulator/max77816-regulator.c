// SPDX-License-Identifier: GPL-2.0-only
/*
 * Maxim MAX77816 buck-boost as a fixed 5.0 V supply: the backlight boost of
 * the Samsung Galaxy Tab S7 FE panel.
 *
 * The EN pin switches the regulator. VOUT (register 0x04) resets to 3.30 V
 * (0x23; 3.40 V on the A version) per the datasheet, so after each enable
 * the driver writes 0x78 = 5.00 V, as Samsung's panel code does
 * (ss_boost_control: write 0x04 = 0x78 after boost_en). Nothing else is
 * written. The voltage is fixed; consumers can only switch it on and off.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define MAX77816_VOUT		0x04
#define MAX77816_VOUT_5V0	0x78	/* 2.60 V + 0x78 * 20 mV */

struct max77816 {
	struct i2c_client *client;
	struct gpio_desc *en;
};

static int max77816_enable(struct regulator_dev *rdev)
{
	struct max77816 *chip = rdev_get_drvdata(rdev);
	int ret;

	gpiod_set_value_cansleep(chip->en, 1);
	usleep_range(1000, 1100);	/* Samsung's boost post-on delay */
	ret = i2c_smbus_write_byte_data(chip->client, MAX77816_VOUT, MAX77816_VOUT_5V0);
	if (ret)
		gpiod_set_value_cansleep(chip->en, 0);
	return ret;
}

static int max77816_disable(struct regulator_dev *rdev)
{
	struct max77816 *chip = rdev_get_drvdata(rdev);

	gpiod_set_value_cansleep(chip->en, 0);
	return 0;
}

static int max77816_is_enabled(struct regulator_dev *rdev)
{
	struct max77816 *chip = rdev_get_drvdata(rdev);

	return gpiod_get_value_cansleep(chip->en);
}

static const struct regulator_ops max77816_ops = {
	.enable = max77816_enable,
	.disable = max77816_disable,
	.is_enabled = max77816_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
};

static const struct regulator_desc max77816_desc = {
	.name = "max77816",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &max77816_ops,
	.n_voltages = 1,
	.min_uV = 5000000,
};

static int max77816_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config cfg = { .dev = dev };
	struct regulator_dev *rdev;
	struct max77816 *chip;
	int on, ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;

	/* Keep the state the bootloader left: it runs the backlight of the boot screen. */
	chip->en = devm_gpiod_get(dev, "enable", GPIOD_ASIS);
	if (IS_ERR(chip->en))
		return dev_err_probe(dev, PTR_ERR(chip->en), "failed to get enable-gpios\n");
	on = gpiod_get_value_cansleep(chip->en);
	if (on < 0)
		return on;
	ret = gpiod_direction_output(chip->en, on);
	if (ret)
		return ret;

	/*
	 * The chip answers: a missing or wrong device fails here, before any consumer enables it.
	 */
	ret = i2c_smbus_read_byte_data(client, MAX77816_VOUT);
	if (ret < 0)
		return dev_err_probe(dev, ret, "no answer at %#x\n", client->addr);
	dev_info(dev, "enable %s, VOUT code %#04x\n", on ? "high" : "low", ret);

	cfg.driver_data = chip;
	cfg.of_node = dev->of_node;
	cfg.init_data = of_get_regulator_init_data(dev, dev->of_node, &max77816_desc);
	rdev = devm_regulator_register(dev, &max77816_desc, &cfg);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev), "failed to register regulator\n");

	return 0;
}

static const struct of_device_id max77816_of_match[] = {
	{ .compatible = "maxim,max77816" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77816_of_match);

static struct i2c_driver max77816_driver = {
	.driver = {
		.name = "max77816",
		.of_match_table = max77816_of_match,
	},
	.probe = max77816_probe,
};
module_i2c_driver(max77816_driver);

MODULE_DESCRIPTION("Maxim MAX77816 buck-boost, fixed 5.0 V");
MODULE_LICENSE("GPL");
