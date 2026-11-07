// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW22127 RGB LED driver
 *
 * The AW22127 is an LED matrix driver with a small MCU running a program
 * kept in the chip's own flash. Lighting effects are commands to that
 * program: a byte stream fed through the TASK1 mailbox and started with
 * TASK0. Register protocol from Awinic's GPL leds-aw22xxx.c and Joonhoe
 * Kim's leds-aw22127 for the Lenovo TB323FU; command layout from the effect
 * files of the Nubia NX809J (RedMagic 11 Pro) vendor stack.
 *
 * The program drives its LEDs in groups. On the NX809J group 0 is the logo
 * and group 1 the "game mode" light. Each group is one multicolor LED. A
 * command carries the steps of every group; a step is a duration in ms (big
 * endian), the brightness at its start and at its end, and R, G, B. The MCU
 * ramps within a step and loops over each group's steps. While every group
 * is off the chip is held in reset.
 *
 * The driver never writes the chip's flash.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/led-class-multicolor.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

#define AW22127_REG_CHIPID	0x00
#define AW22127_REG_SRST	0x01
#define AW22127_REG_GCR		0x02
#define AW22127_REG_MCUCTR	0x04
#define AW22127_REG_TASK0	0x05
#define AW22127_REG_TASK1	0x06
#define AW22127_REG_INTEN	0x09
#define AW22127_REG_IMAX	0x0b
#define AW22127_REG_PST		0x0c
#define AW22127_REG_PAGE	0xff

#define AW22127_CHIPID		0x27
#define AW22127_SRST_RESET	0x55
#define AW22127_GCR_CHIPEN	BIT(0)
#define AW22127_MCUCTR_MAILBOX	0x07

/* The current code the vendor driver programs (its imax 11) */
#define AW22127_IMAX		0x06

/*
 * "Groups in steps": this header with the group count in its last byte and
 * twelve zero bytes, one line per group with its step count, then each
 * group's index and its steps. The second byte is also the TASK0 value that
 * starts the command.
 */
static const u8 aw22127_cmd_hdr[] = {
	0x02, 0x82, 0x01, 0x01, 0x07, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#define AW22127_HDR_NGROUPS	8
#define AW22127_HDR_START	1
#define AW22127_LINE_LEN	9
#define AW22127_LINE_NSTEPS	2
#define AW22127_STEP_LEN	7
#define AW22127_MAX_GROUPS	2
#define AW22127_MAX_STEPS	16
#define AW22127_CMD_MAX		(sizeof(aw22127_cmd_hdr) + AW22127_MAX_GROUPS * \
				 (AW22127_LINE_LEN + 1 + AW22127_MAX_STEPS * AW22127_STEP_LEN))
#define AW22127_STATIC_MS	500

struct aw22127_step {
	u16 ms;
	u8 from;
	u8 to;
};

struct aw22127;

struct aw22127_group {
	struct aw22127 *aw;
	struct led_classdev_mc mc;
	struct mc_subled subled[3];
	struct aw22127_step steps[AW22127_MAX_STEPS];
	/* zero while the group is off */
	unsigned int nsteps;
};

struct aw22127 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset;
	struct mutex lock;	/* the chip, on, groups, cmd */
	bool on;
	unsigned int ngroups;
	struct aw22127_group groups[AW22127_MAX_GROUPS];
	u8 cmd[AW22127_CMD_MAX];
	size_t cmd_len;
	u8 next[AW22127_CMD_MAX];
};

static const struct regmap_config aw22127_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static void aw22127_power(struct aw22127 *aw, bool on)
{
	/* reset-gpios is active low: deasserted = running */
	gpiod_set_value_cansleep(aw->reset, !on);
	if (on)
		usleep_range(2000, 3000);
}

static int aw22127_play(struct aw22127 *aw, const u8 *cmd, size_t len)
{
	struct regmap *rm = aw->regmap;
	size_t i;
	int ret;

	ret = regmap_write(rm, AW22127_REG_PAGE, 0);
	ret = ret ?: regmap_write(rm, AW22127_REG_GCR, AW22127_GCR_CHIPEN);
	if (ret)
		return ret;
	if (!aw->on)
		usleep_range(2000, 3000);

	ret = regmap_write(rm, AW22127_REG_IMAX, AW22127_IMAX);
	ret = ret ?: regmap_write(rm, AW22127_REG_PST, 0);
	ret = ret ?: regmap_write(rm, AW22127_REG_TASK0, 0x01);
	ret = ret ?: regmap_write(rm, AW22127_REG_MCUCTR, 0x01);
	ret = ret ?: regmap_write(rm, AW22127_REG_INTEN, 0x01);
	ret = ret ?: regmap_write(rm, AW22127_REG_MCUCTR, 0x03);
	for (i = 0; !ret && i < len; i++) {
		if (i)
			ret = regmap_write(rm, AW22127_REG_MCUCTR,
					   AW22127_MCUCTR_MAILBOX);
		ret = ret ?: regmap_write(rm, AW22127_REG_TASK1, cmd[i]);
	}
	ret = ret ?: regmap_write(rm, AW22127_REG_MCUCTR, AW22127_MCUCTR_MAILBOX);
	ret = ret ?: regmap_write(rm, AW22127_REG_TASK0, cmd[AW22127_HDR_START]);

	return ret;
}

static void aw22127_off(struct aw22127 *aw)
{
	struct regmap *rm = aw->regmap;

	if (!aw->on)
		return;

	regmap_write(rm, AW22127_REG_PAGE, 0);
	regmap_write(rm, AW22127_REG_TASK0, 0);
	regmap_write(rm, AW22127_REG_MCUCTR, 0x01);
	regmap_write(rm, AW22127_REG_GCR, 0);
	regmap_write(rm, AW22127_REG_SRST, AW22127_SRST_RESET);
	aw22127_power(aw, false);
	aw->on = false;
	aw->cmd_len = 0;
}

/* Build the command for every group's current state into aw->next. */
static size_t aw22127_build(struct aw22127 *aw)
{
	static const struct aw22127_step dark = { .ms = AW22127_STATIC_MS };
	u8 *p = aw->next;
	unsigned int g, i;

	memcpy(p, aw22127_cmd_hdr, sizeof(aw22127_cmd_hdr));
	p[AW22127_HDR_NGROUPS] = aw->ngroups;
	p += sizeof(aw22127_cmd_hdr);

	for (g = 0; g < aw->ngroups; g++, p += AW22127_LINE_LEN) {
		memset(p, 0, AW22127_LINE_LEN);
		p[0] = 0x01;
		p[1] = 0x01;
		p[AW22127_LINE_NSTEPS] = max(aw->groups[g].nsteps, 1U);
	}

	for (g = 0; g < aw->ngroups; g++) {
		struct aw22127_group *grp = &aw->groups[g];
		const struct aw22127_step *steps = grp->nsteps ? grp->steps : &dark;
		unsigned int n = max(grp->nsteps, 1U);

		*p++ = g;
		for (i = 0; i < n; i++, p += AW22127_STEP_LEN) {
			put_unaligned_be16(steps[i].ms, p);
			p[2] = steps[i].from;
			p[3] = steps[i].to;
			p[4] = grp->subled[0].intensity;
			p[5] = grp->subled[1].intensity;
			p[6] = grp->subled[2].intensity;
		}
	}

	return p - aw->next;
}

static int aw22127_update(struct aw22127 *aw)
{
	unsigned int g;
	size_t len;
	int ret;

	for (g = 0; g < aw->ngroups; g++)
		if (aw->groups[g].nsteps)
			break;
	if (g == aw->ngroups) {
		aw22127_off(aw);
		return 0;
	}

	len = aw22127_build(aw);
	/* Sending a command restarts the program, so don't repeat one */
	if (aw->on && len == aw->cmd_len && !memcmp(aw->next, aw->cmd, len))
		return 0;

	if (!aw->on)
		aw22127_power(aw, true);
	ret = aw22127_play(aw, aw->next, len);
	if (ret) {
		aw22127_power(aw, false);
		aw->on = false;
		aw->cmd_len = 0;
		return ret;
	}
	aw->on = true;
	memcpy(aw->cmd, aw->next, len);
	aw->cmd_len = len;

	return 0;
}

static struct aw22127_group *cdev_to_group(struct led_classdev *cdev)
{
	return container_of(lcdev_to_mccdev(cdev), struct aw22127_group, mc);
}

static int aw22127_brightness_set(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
	struct aw22127_group *grp = cdev_to_group(cdev);
	struct aw22127 *aw = grp->aw;
	int ret;

	mutex_lock(&aw->lock);
	grp->steps[0] = (struct aw22127_step) {
		.ms = AW22127_STATIC_MS,
		.from = brightness,
		.to = brightness,
	};
	grp->nsteps = brightness ? 1 : 0;
	ret = aw22127_update(aw);
	mutex_unlock(&aw->lock);

	return ret;
}

static int aw22127_pattern_set(struct led_classdev *cdev,
			       struct led_pattern *pattern, u32 len, int repeat)
{
	struct aw22127_group *grp = cdev_to_group(cdev);
	struct aw22127 *aw = grp->aw;
	unsigned int i;
	int ret;

	if (repeat > 0 || !len || len > AW22127_MAX_STEPS)
		return -EINVAL;

	for (i = 0; i < len; i++)
		if (!pattern[i].delta_t || pattern[i].delta_t > U16_MAX)
			return -EINVAL;

	mutex_lock(&aw->lock);
	for (i = 0; i < len; i++) {
		grp->steps[i].ms = pattern[i].delta_t;
		grp->steps[i].from = pattern[i].brightness;
		grp->steps[i].to = pattern[(i + 1) % len].brightness;
	}
	grp->nsteps = len;
	ret = aw22127_update(aw);
	mutex_unlock(&aw->lock);

	return ret;
}

static int aw22127_pattern_clear(struct led_classdev *cdev)
{
	struct aw22127_group *grp = cdev_to_group(cdev);
	struct aw22127 *aw = grp->aw;
	int ret;

	mutex_lock(&aw->lock);
	grp->nsteps = 0;
	ret = aw22127_update(aw);
	mutex_unlock(&aw->lock);

	return ret;
}

static int aw22127_add_group(struct aw22127 *aw, struct fwnode_handle *child)
{
	struct led_init_data init = {};
	struct aw22127_group *grp;
	u32 reg;
	int ret;

	ret = fwnode_property_read_u32(child, "reg", &reg);
	if (ret || reg >= AW22127_MAX_GROUPS)
		return dev_err_probe(aw->dev, -EINVAL, "%pfw: bad reg\n", child);

	grp = &aw->groups[reg];
	if (grp->aw)
		return dev_err_probe(aw->dev, -EINVAL, "%pfw: reg %u twice\n", child, reg);
	grp->aw = aw;
	aw->ngroups = max(aw->ngroups, reg + 1);

	grp->subled[0].color_index = LED_COLOR_ID_RED;
	grp->subled[1].color_index = LED_COLOR_ID_GREEN;
	grp->subled[2].color_index = LED_COLOR_ID_BLUE;
	grp->subled[0].intensity = 255;
	grp->subled[1].intensity = 255;
	grp->subled[2].intensity = 255;

	grp->mc.subled_info = grp->subled;
	grp->mc.num_colors = ARRAY_SIZE(grp->subled);
	grp->mc.led_cdev.max_brightness = 255;
	grp->mc.led_cdev.brightness_set_blocking = aw22127_brightness_set;
	grp->mc.led_cdev.pattern_set = aw22127_pattern_set;
	grp->mc.led_cdev.pattern_clear = aw22127_pattern_clear;

	init.fwnode = child;
	init.devicename = "aw22127";

	ret = devm_led_classdev_multicolor_register_ext(aw->dev, &grp->mc, &init);
	if (ret)
		return dev_err_probe(aw->dev, ret, "registering %pfw\n", child);

	return 0;
}

static int aw22127_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct aw22127 *aw;
	unsigned int id, g;
	int ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;

	aw->dev = dev;
	ret = devm_mutex_init(dev, &aw->lock);
	if (ret)
		return ret;

	aw->regmap = devm_regmap_init_i2c(client, &aw22127_regmap);
	if (IS_ERR(aw->regmap))
		return PTR_ERR(aw->regmap);

	aw->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset))
		return dev_err_probe(dev, PTR_ERR(aw->reset), "reset gpio\n");

	aw22127_power(aw, true);
	ret = regmap_read(aw->regmap, AW22127_REG_CHIPID, &id);
	aw22127_power(aw, false);
	if (ret)
		return dev_err_probe(dev, ret, "reading chip ID\n");
	if (id != AW22127_CHIPID)
		return dev_err_probe(dev, -ENODEV, "chip ID 0x%02x, not an AW22127\n", id);

	device_for_each_child_node_scoped(dev, child) {
		ret = aw22127_add_group(aw, child);
		if (ret)
			return ret;
	}
	if (!aw->ngroups)
		return dev_err_probe(dev, -EINVAL, "no LED groups\n");
	for (g = 0; g < aw->ngroups; g++)
		if (!aw->groups[g].aw)
			return dev_err_probe(dev, -EINVAL, "group %u missing\n", g);

	i2c_set_clientdata(client, aw);

	return 0;
}

static void aw22127_shutdown(struct i2c_client *client)
{
	struct aw22127 *aw = i2c_get_clientdata(client);

	mutex_lock(&aw->lock);
	aw22127_off(aw);
	mutex_unlock(&aw->lock);
}

static const struct of_device_id aw22127_of_match[] = {
	{ .compatible = "awinic,aw22127" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw22127_of_match);

static struct i2c_driver aw22127_driver = {
	.driver = {
		.name = "leds-aw22127",
		.of_match_table = aw22127_of_match,
	},
	.probe = aw22127_probe,
	.shutdown = aw22127_shutdown,
};
module_i2c_driver(aw22127_driver);

MODULE_DESCRIPTION("Awinic AW22127 RGB LED driver");
MODULE_LICENSE("GPL");
