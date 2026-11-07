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
 * The program runs up to three effect tasks side by side, each driving its
 * own units. On the NX809J task 0 drives the shoulder trigger lights, task 1
 * the four fan zones and task 2 the logo and the "game mode" light. Each
 * unit is one multicolor LED, addressed in the device tree by task and unit.
 *
 * A task's command carries the steps of each of its units; a step is a
 * duration in ms (big endian), the brightness at its start and at its end,
 * and R, G, B. The MCU ramps within a step and loops over each unit's steps,
 * so hardware patterns run without host writes. Changing a LED sends its
 * task's command only; the other tasks keep running. Changes are sent a few
 * milliseconds later, together: a new pattern clears the old one first, and
 * sending a command takes long enough for the MCU to show the gap. While
 * every LED is off the chip is held in reset.
 *
 * The driver never writes the chip's flash.
 */

#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/led-class-multicolor.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

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
#define AW22127_MCUCTR_RESET	0x01
#define AW22127_MCUCTR_RUN	0x03
#define AW22127_MCUCTR_MAILBOX	0x07

/* The current code the vendor driver programs (its imax 11) */
#define AW22127_IMAX		0x06

#define AW22127_TASKS		3
#define AW22127_MAX_UNITS	4
#define AW22127_MAX_STEPS	16
#define AW22127_STATIC_MS	500
/* How long changes gather before they are sent */
#define AW22127_SETTLE_MS	10

/*
 * A task's command: this header with the task in its first byte, its start
 * code in the second and the unit count in the last, twelve zero bytes, one
 * line per unit with its step count, then each unit's index and its steps.
 */
static const u8 aw22127_cmd_hdr[] = {
	0x00, 0x00, 0x01, 0x01, 0x07, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#define AW22127_HDR_TASK	0
#define AW22127_HDR_START	1
#define AW22127_HDR_NUNITS	8
#define AW22127_LINE_LEN	9
#define AW22127_LINE_NSTEPS	2
#define AW22127_STEP_LEN	7
#define AW22127_CMD_MAX		(sizeof(aw22127_cmd_hdr) + AW22127_MAX_UNITS * \
				 (AW22127_LINE_LEN + 1 + AW22127_MAX_STEPS * AW22127_STEP_LEN))

/* What TASK0 is set to before a task's command, from the vendor files */
static const u8 aw22127_task_open[AW22127_TASKS] = { 0x01, 0x41, 0x51 };

struct aw22127_step {
	u16 ms;
	u8 from;
	u8 to;
};

struct aw22127;

struct aw22127_led {
	struct aw22127 *aw;
	unsigned int task;
	struct led_classdev_mc mc;
	struct mc_subled subled[3];
	struct aw22127_step steps[AW22127_MAX_STEPS];
	/* zero while the LED is off */
	unsigned int nsteps;
};

struct aw22127_task {
	struct aw22127_led *units[AW22127_MAX_UNITS];
	unsigned int nunits;
	u8 cmd[AW22127_CMD_MAX];
	/* zero until the running command is known */
	size_t cmd_len;
};

struct aw22127 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset;
	struct mutex lock;	/* the chip, on, tasks, dirty */
	bool on;
	/* Tasks changed since the last send, by bit */
	unsigned long dirty;
	struct delayed_work work;
	struct aw22127_task tasks[AW22127_TASKS];
	u8 next[AW22127_CMD_MAX];
};

static const struct regmap_config aw22127_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int aw22127_power_on(struct aw22127 *aw)
{
	struct regmap *rm = aw->regmap;
	int ret;

	/* reset-gpios is active low: deasserted = running */
	gpiod_set_value_cansleep(aw->reset, 0);
	usleep_range(2000, 3000);

	ret = regmap_write(rm, AW22127_REG_PAGE, 0);
	ret = ret ?: regmap_write(rm, AW22127_REG_GCR, AW22127_GCR_CHIPEN);
	if (ret)
		goto err;
	usleep_range(2000, 3000);

	/* Start the program */
	ret = regmap_write(rm, AW22127_REG_IMAX, AW22127_IMAX);
	ret = ret ?: regmap_write(rm, AW22127_REG_PST, 0);
	ret = ret ?: regmap_write(rm, AW22127_REG_TASK0, 0x01);
	ret = ret ?: regmap_write(rm, AW22127_REG_MCUCTR, AW22127_MCUCTR_RESET);
	ret = ret ?: regmap_write(rm, AW22127_REG_INTEN, 0x01);
	ret = ret ?: regmap_write(rm, AW22127_REG_MCUCTR, AW22127_MCUCTR_RUN);
	if (ret)
		goto err;

	aw->on = true;
	return 0;
err:
	gpiod_set_value_cansleep(aw->reset, 1);
	return ret;
}

static void aw22127_power_off(struct aw22127 *aw)
{
	struct regmap *rm = aw->regmap;
	unsigned int t;

	if (!aw->on)
		return;

	regmap_write(rm, AW22127_REG_PAGE, 0);
	regmap_write(rm, AW22127_REG_TASK0, 0);
	regmap_write(rm, AW22127_REG_MCUCTR, AW22127_MCUCTR_RESET);
	regmap_write(rm, AW22127_REG_GCR, 0);
	regmap_write(rm, AW22127_REG_SRST, AW22127_SRST_RESET);
	gpiod_set_value_cansleep(aw->reset, 1);
	aw->on = false;
	for (t = 0; t < AW22127_TASKS; t++)
		aw->tasks[t].cmd_len = 0;
}

static int aw22127_send(struct aw22127 *aw, unsigned int task,
			const u8 *cmd, size_t len)
{
	struct regmap *rm = aw->regmap;
	size_t i;
	int ret;

	ret = regmap_write(rm, AW22127_REG_PAGE, 0);
	ret = ret ?: regmap_write(rm, AW22127_REG_PST, 0);
	ret = ret ?: regmap_write(rm, AW22127_REG_TASK0, aw22127_task_open[task]);
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

/* Build task's command for its units' current state into aw->next. */
static size_t aw22127_build(struct aw22127 *aw, unsigned int task)
{
	static const struct aw22127_step dark = { .ms = AW22127_STATIC_MS };
	struct aw22127_task *t = &aw->tasks[task];
	u8 *p = aw->next;
	unsigned int u, i;

	memcpy(p, aw22127_cmd_hdr, sizeof(aw22127_cmd_hdr));
	p[AW22127_HDR_TASK] = task;
	p[AW22127_HDR_START] = 0x80 | task;
	p[AW22127_HDR_NUNITS] = t->nunits;
	p += sizeof(aw22127_cmd_hdr);

	for (u = 0; u < t->nunits; u++, p += AW22127_LINE_LEN) {
		memset(p, 0, AW22127_LINE_LEN);
		p[0] = 0x01;
		p[1] = 0x01;
		p[AW22127_LINE_NSTEPS] = max(t->units[u]->nsteps, 1U);
	}

	for (u = 0; u < t->nunits; u++) {
		struct aw22127_led *led = t->units[u];
		const struct aw22127_step *steps = led->nsteps ? led->steps : &dark;
		unsigned int n = max(led->nsteps, 1U);

		*p++ = u;
		for (i = 0; i < n; i++, p += AW22127_STEP_LEN) {
			put_unaligned_be16(steps[i].ms, p);
			p[2] = steps[i].from;
			p[3] = steps[i].to;
			p[4] = led->subled[0].intensity;
			p[5] = led->subled[1].intensity;
			p[6] = led->subled[2].intensity;
		}
	}

	return p - aw->next;
}

static bool aw22127_task_lit(struct aw22127 *aw, unsigned int task)
{
	struct aw22127_task *t = &aw->tasks[task];
	unsigned int u;

	for (u = 0; u < t->nunits; u++)
		if (t->units[u]->nsteps)
			return true;
	return false;
}

/* Send task's command after its LEDs changed. */
static int aw22127_send_task(struct aw22127 *aw, unsigned int task)
{
	struct aw22127_task *t = &aw->tasks[task];
	unsigned int i;
	size_t len;
	int ret;

	for (i = 0; i < AW22127_TASKS; i++)
		if (aw22127_task_lit(aw, i))
			break;
	if (i == AW22127_TASKS) {
		aw22127_power_off(aw);
		return 0;
	}

	if (!aw->on) {
		ret = aw22127_power_on(aw);
		if (ret)
			return ret;
	}

	len = aw22127_build(aw, task);
	/* Sending a command restarts the task, so don't repeat one */
	if (len == t->cmd_len && !memcmp(aw->next, t->cmd, len))
		return 0;

	ret = aw22127_send(aw, task, aw->next, len);
	if (ret) {
		aw22127_power_off(aw);
		return ret;
	}
	memcpy(t->cmd, aw->next, len);
	t->cmd_len = len;

	return 0;
}

static void aw22127_work(struct work_struct *work)
{
	struct aw22127 *aw = container_of(to_delayed_work(work), struct aw22127, work);
	unsigned int task;

	mutex_lock(&aw->lock);
	for_each_set_bit(task, &aw->dirty, AW22127_TASKS) {
		int ret = aw22127_send_task(aw, task);

		if (ret)
			dev_err(aw->dev, "sending task %u: %d\n", task, ret);
	}
	aw->dirty = 0;
	mutex_unlock(&aw->lock);
}

/* Send task's command once the changes coming with this one are in. */
static void aw22127_update(struct aw22127 *aw, unsigned int task)
{
	aw->dirty |= BIT(task);
	mod_delayed_work(system_wq, &aw->work, msecs_to_jiffies(AW22127_SETTLE_MS));
}

static struct aw22127_led *cdev_to_led(struct led_classdev *cdev)
{
	return container_of(lcdev_to_mccdev(cdev), struct aw22127_led, mc);
}

static int aw22127_brightness_set(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
	struct aw22127_led *led = cdev_to_led(cdev);
	struct aw22127 *aw = led->aw;

	mutex_lock(&aw->lock);
	led->steps[0] = (struct aw22127_step) {
		.ms = AW22127_STATIC_MS,
		.from = brightness,
		.to = brightness,
	};
	led->nsteps = brightness ? 1 : 0;
	aw22127_update(aw, led->task);
	mutex_unlock(&aw->lock);

	return 0;
}

static int aw22127_pattern_set(struct led_classdev *cdev,
			       struct led_pattern *pattern, u32 len, int repeat)
{
	struct aw22127_led *led = cdev_to_led(cdev);
	struct aw22127 *aw = led->aw;
	unsigned int i;

	if (repeat > 0 || !len || len > AW22127_MAX_STEPS)
		return -EINVAL;

	for (i = 0; i < len; i++)
		if (!pattern[i].delta_t || pattern[i].delta_t > U16_MAX)
			return -EINVAL;

	mutex_lock(&aw->lock);
	for (i = 0; i < len; i++) {
		led->steps[i].ms = pattern[i].delta_t;
		led->steps[i].from = pattern[i].brightness;
		led->steps[i].to = pattern[(i + 1) % len].brightness;
	}
	led->nsteps = len;
	aw22127_update(aw, led->task);
	mutex_unlock(&aw->lock);

	return 0;
}

static int aw22127_pattern_clear(struct led_classdev *cdev)
{
	struct aw22127_led *led = cdev_to_led(cdev);
	struct aw22127 *aw = led->aw;

	mutex_lock(&aw->lock);
	led->nsteps = 0;
	aw22127_update(aw, led->task);
	mutex_unlock(&aw->lock);

	return 0;
}

static int aw22127_add_led(struct aw22127 *aw, struct fwnode_handle *child)
{
	struct led_init_data init = {};
	struct aw22127_led *led;
	struct aw22127_task *t;
	u32 reg[2];
	int ret;

	ret = fwnode_property_read_u32_array(child, "reg", reg, ARRAY_SIZE(reg));
	if (ret || reg[0] >= AW22127_TASKS || reg[1] >= AW22127_MAX_UNITS)
		return dev_err_probe(aw->dev, -EINVAL, "%pfw: bad reg\n", child);

	t = &aw->tasks[reg[0]];
	if (t->units[reg[1]])
		return dev_err_probe(aw->dev, -EINVAL, "%pfw: unit used twice\n", child);

	led = devm_kzalloc(aw->dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;
	led->aw = aw;
	led->task = reg[0];
	t->units[reg[1]] = led;
	t->nunits = max(t->nunits, reg[1] + 1);

	led->subled[0].color_index = LED_COLOR_ID_RED;
	led->subled[1].color_index = LED_COLOR_ID_GREEN;
	led->subled[2].color_index = LED_COLOR_ID_BLUE;
	led->subled[0].intensity = 255;
	led->subled[1].intensity = 255;
	led->subled[2].intensity = 255;

	led->mc.subled_info = led->subled;
	led->mc.num_colors = ARRAY_SIZE(led->subled);
	led->mc.led_cdev.max_brightness = 255;
	led->mc.led_cdev.brightness_set_blocking = aw22127_brightness_set;
	led->mc.led_cdev.pattern_set = aw22127_pattern_set;
	led->mc.led_cdev.pattern_clear = aw22127_pattern_clear;

	init.fwnode = child;
	init.devicename = "aw22127";

	ret = devm_led_classdev_multicolor_register_ext(aw->dev, &led->mc, &init);
	if (ret)
		return dev_err_probe(aw->dev, ret, "registering %pfw\n", child);

	return 0;
}

static void aw22127_release(void *data)
{
	struct aw22127 *aw = data;

	mutex_lock(&aw->lock);
	aw22127_power_off(aw);
	mutex_unlock(&aw->lock);
}

static int aw22127_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct aw22127 *aw;
	unsigned int id, t, u;
	int ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;

	aw->dev = dev;
	ret = devm_mutex_init(dev, &aw->lock);
	if (ret)
		return ret;
	/* Released in reverse: the work stops, then the chip goes off */
	ret = devm_add_action_or_reset(dev, aw22127_release, aw);
	if (ret)
		return ret;
	ret = devm_delayed_work_autocancel(dev, &aw->work, aw22127_work);
	if (ret)
		return ret;

	aw->regmap = devm_regmap_init_i2c(client, &aw22127_regmap);
	if (IS_ERR(aw->regmap))
		return PTR_ERR(aw->regmap);

	aw->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset))
		return dev_err_probe(dev, PTR_ERR(aw->reset), "reset gpio\n");

	gpiod_set_value_cansleep(aw->reset, 0);
	usleep_range(2000, 3000);
	ret = regmap_read(aw->regmap, AW22127_REG_CHIPID, &id);
	gpiod_set_value_cansleep(aw->reset, 1);
	if (ret)
		return dev_err_probe(dev, ret, "reading chip ID\n");
	if (id != AW22127_CHIPID)
		return dev_err_probe(dev, -ENODEV, "chip ID 0x%02x, not an AW22127\n", id);

	device_for_each_child_node_scoped(dev, child) {
		ret = aw22127_add_led(aw, child);
		if (ret)
			return ret;
	}
	for (t = 0; t < AW22127_TASKS; t++)
		for (u = 0; u < aw->tasks[t].nunits; u++)
			if (!aw->tasks[t].units[u])
				return dev_err_probe(dev, -EINVAL,
						     "task %u unit %u missing\n", t, u);

	i2c_set_clientdata(client, aw);

	return 0;
}

static void aw22127_shutdown(struct i2c_client *client)
{
	struct aw22127 *aw = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&aw->work);
	mutex_lock(&aw->lock);
	aw22127_power_off(aw);
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
