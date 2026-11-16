// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI DRV2624 haptic driver, RTP mode with internal trigger.
 *
 * Minimal driver for the Pixel 5a LRA: reset the device, configure the
 * rated voltage/overdrive clamp from the device tree, then stream an 8-bit
 * signed amplitude into RTP_INPUT and trigger with GO. Only the stock
 * tuning properties are read; no waveform RAM is programmed.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define DRV2624_REG_ID		0x00
#define DRV2624_REG_MODE	0x07
#define DRV2624_REG_GO		0x0c
#define DRV2624_REG_RTP_INPUT	0x0e
#define DRV2624_REG_RATED_VOLTAGE	0x1f
#define DRV2624_REG_OD_CLAMP	0x20
#define DRV2624_REG_LOOP_CONTROL	0x23

#define DRV2624_MODE_RTP	0x00

struct drv2624 {
	struct regmap *regmap;
	struct input_dev *input;
	struct delayed_work stop_work;
};

static const struct regmap_config drv2624_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static void drv2624_stop_work(struct work_struct *work)
{
	struct drv2624 *h = container_of(to_delayed_work(work),
					 struct drv2624, stop_work);

	regmap_write(h->regmap, DRV2624_REG_GO, 0);
}

static int drv2624_play(struct input_dev *dev, void *data,
			struct ff_effect *effect)
{
	struct drv2624 *h = input_get_drvdata(dev);
	u8 strong = effect->u.rumble.strong_magnitude >> 8;
	u8 weak = effect->u.rumble.weak_magnitude >> 8;
	u8 magnitude = max(strong, weak);

	cancel_delayed_work_sync(&h->stop_work);

	if (!magnitude) {
		regmap_write(h->regmap, DRV2624_REG_GO, 0);
		return 0;
	}

	regmap_write(h->regmap, DRV2624_REG_MODE, DRV2624_MODE_RTP);
	regmap_write(h->regmap, DRV2624_REG_RTP_INPUT, magnitude);
	regmap_write(h->regmap, DRV2624_REG_GO, 1);

	if (effect->replay.length)
		schedule_delayed_work(&h->stop_work,
				      msecs_to_jiffies(effect->replay.length));
	return 0;
}

static int drv2624_probe(struct i2c_client *client)
{
	struct drv2624 *h;
	struct gpio_desc *reset;
	unsigned int val;
	u8 rated = 0x6d;
	u8 od_clamp = 0xa1;
	int ret;

	h = devm_kzalloc(&client->dev, sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;

	h->regmap = devm_regmap_init_i2c(client, &drv2624_regmap_config);
	if (IS_ERR(h->regmap))
		return PTR_ERR(h->regmap);

	reset = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset))
		return PTR_ERR(reset);
	if (reset) {
		gpiod_set_value_cansleep(reset, 0);
		msleep(10);
		gpiod_set_value_cansleep(reset, 1);
		msleep(10);
	}

	ret = regmap_read(h->regmap, DRV2624_REG_ID, &val);
	if (ret)
		return ret;
	if ((val & 0xf0) != 0)
		dev_warn(&client->dev, "unexpected device id %#x\n", val);

	device_property_read_u8(&client->dev, "ti,rated-voltage", &rated);
	device_property_read_u8(&client->dev, "ti,odclamp-voltage", &od_clamp);
	regmap_write(h->regmap, DRV2624_REG_RATED_VOLTAGE, rated);
	regmap_write(h->regmap, DRV2624_REG_OD_CLAMP, od_clamp);
	regmap_write(h->regmap, DRV2624_REG_LOOP_CONTROL, 0x00);
	regmap_write(h->regmap, DRV2624_REG_MODE, DRV2624_MODE_RTP);

	h->input = devm_input_allocate_device(&client->dev);
	if (!h->input)
		return -ENOMEM;
	h->input->name = "drv2624:haptics";
	h->input->id.bustype = BUS_I2C;
	input_set_drvdata(h->input, h);
	input_set_capability(h->input, EV_FF, FF_RUMBLE);

	INIT_DELAYED_WORK(&h->stop_work, drv2624_stop_work);

	ret = input_ff_create_memless(h->input, NULL, drv2624_play);
	if (ret)
		return ret;

	return input_register_device(h->input);
}

static void drv2624_remove(struct i2c_client *client)
{
	struct drv2624 *h = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&h->stop_work);
	regmap_write(h->regmap, DRV2624_REG_GO, 0);
}

static const struct of_device_id drv2624_of_match[] = {
	{ .compatible = "ti,drv2624" },
	{ }
};
MODULE_DEVICE_TABLE(of, drv2624_of_match);

static const struct i2c_device_id drv2624_i2c_id[] = {
	{ "drv2624", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, drv2624_i2c_id);

static struct i2c_driver drv2624_driver = {
	.driver = {
		.name = "drv2624",
		.of_match_table = drv2624_of_match,
	},
	.probe = drv2624_probe,
	.remove = drv2624_remove,
	.id_table = drv2624_i2c_id,
};
module_i2c_driver(drv2624_driver);

MODULE_DESCRIPTION("TI DRV2624 haptic driver");
MODULE_LICENSE("GPL");
