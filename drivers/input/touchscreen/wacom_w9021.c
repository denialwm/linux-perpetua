// SPDX-License-Identifier: GPL-2.0-only
/*
 * Wacom W9021 EMR digitizer (Samsung S Pen) on I2C, as on the Galaxy Tab S7 FE.
 *
 * Samsung's firmware speaks its own packet format, not the HID-style feature
 * reports of wacom_i2c.c. Protocol from Samsung's GPL driver
 * (drivers/input/wacom/wez01/wacom_i2c.c, SM-T736B kernel source):
 *  - query: a plain 32-byte read; bytes 16..31 are the query, header 0x0f;
 *  - on each (level-low) interrupt, a plain 17-byte read; the low nibble of
 *    byte 0 is the packet id, 1 = coordinates.
 * Firmware update, survey (garage/AOP) and factory modes are left out: the
 * chip runs the firmware stock left on it.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#define W9021_QUERY_LEN		32
#define W9021_QUERY_POS		16
#define W9021_QUERY_HEADER	0x0f
#define W9021_PACKET_LEN	17
#define W9021_COORD_PACKET	1

/* byte 0 of a coordinate packet */
#define W9021_RDY		BIT(7)	/* pen in range */
#define W9021_ERASER		BIT(6)
#define W9021_SIDE		BIT(5)	/* side button */
#define W9021_TIP		BIT(4)	/* tip touches */

struct w9021 {
	struct i2c_client *client;
	struct input_dev *input;
	struct regulator *vdd;
	struct touchscreen_properties prop;
	unsigned int tool;	/* BTN_TOOL_PEN or BTN_TOOL_RUBBER while in range, else 0 */
};

static void w9021_out_of_range(struct w9021 *w)
{
	if (!w->tool)
		return;
	input_report_abs(w->input, ABS_PRESSURE, 0);
	input_report_abs(w->input, ABS_DISTANCE, 0);
	input_report_key(w->input, BTN_TOUCH, 0);
	input_report_key(w->input, BTN_STYLUS, 0);
	input_report_key(w->input, w->tool, 0);
	input_sync(w->input);
	w->tool = 0;
}

static irqreturn_t w9021_irq(int irq, void *data)
{
	struct w9021 *w = data;
	u8 d[W9021_PACKET_LEN];
	int tilt_x, tilt_y, ret;

	ret = i2c_master_recv(w->client, d, sizeof(d));
	if (ret != sizeof(d)) {
		dev_err_ratelimited(&w->client->dev, "read failed: %d\n", ret);
		w9021_out_of_range(w);
		return IRQ_HANDLED;
	}

	if ((d[0] & 0x0f) != W9021_COORD_PACKET)
		return IRQ_HANDLED;	/* notifications and replies: nothing asked for them */

	if (!(d[0] & W9021_RDY)) {
		w9021_out_of_range(w);
		return IRQ_HANDLED;
	}

	/* The tool is fixed for a stroke: chosen when the pen enters range. */
	if (!w->tool) {
		w->tool = d[0] & W9021_ERASER ? BTN_TOOL_RUBBER : BTN_TOOL_PEN;
		input_report_key(w->input, w->tool, 1);
	}

	/*
	 * Tilt follows the axes: negated with an inverted axis, exchanged with swapped ones (as
	 * stock).
	 */
	tilt_x = (s8)d[8];
	tilt_y = (s8)d[9];
	if (w->prop.invert_x)
		tilt_x = -tilt_x;
	if (w->prop.invert_y)
		tilt_y = -tilt_y;
	if (w->prop.swap_x_y)
		swap(tilt_x, tilt_y);

	touchscreen_report_pos(w->input, &w->prop, get_unaligned_be16(&d[1]),
			       get_unaligned_be16(&d[3]), false);
	input_report_abs(w->input, ABS_PRESSURE, get_unaligned_be16(&d[5]) & 0x0fff);
	input_report_abs(w->input, ABS_DISTANCE, d[7]);
	input_report_abs(w->input, ABS_TILT_X, tilt_x);
	input_report_abs(w->input, ABS_TILT_Y, tilt_y);
	input_report_key(w->input, BTN_TOUCH, !!(d[0] & W9021_TIP));
	input_report_key(w->input, BTN_STYLUS, !!(d[0] & W9021_SIDE));
	input_sync(w->input);

	return IRQ_HANDLED;
}

static void w9021_power_off(void *data)
{
	regulator_disable(data);
}

static int w9021_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	u8 buf[W9021_QUERY_LEN], *q = buf + W9021_QUERY_POS;
	struct input_dev *input;
	struct w9021 *w;
	int ret, i;

	w = devm_kzalloc(dev, sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;
	w->client = client;

	w->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(w->vdd))
		return dev_err_probe(dev, PTR_ERR(w->vdd), "no vdd supply\n");
	ret = regulator_enable(w->vdd);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable vdd\n");
	ret = devm_add_action_or_reset(dev, w9021_power_off, w->vdd);
	if (ret)
		return ret;
	msleep(200);	/* stock waits 200 ms after power-on before the query */

	for (i = 0; i < 3; i++) {
		ret = i2c_master_recv(client, buf, sizeof(buf));
		if (ret == sizeof(buf) && q[0] == W9021_QUERY_HEADER)
			break;
		msleep(50);
	}
	if (i == 3)
		return dev_err_probe(dev, ret < 0 ? ret : -EIO,
				     "no query response (%d, header %#x)\n", ret, q[0]);

	dev_info(dev, "W90xx mpu %#x fw %02x%02x, %ux%u, pressure %u, tilt %u/%u, height %u\n",
		 q[9], q[7], q[8], get_unaligned_be16(&q[1]), get_unaligned_be16(&q[3]),
		 get_unaligned_be16(&q[5]), q[11], q[12], q[13]);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	w->input = input;
	input->name = "Wacom W9021 S Pen";
	input->id.bustype = BUS_I2C;

	input_set_abs_params(input, ABS_X, 0, get_unaligned_be16(&q[1]), 4, 0);
	input_set_abs_params(input, ABS_Y, 0, get_unaligned_be16(&q[3]), 4, 0);
	/*
	 * libinput ignores a tablet without a resolution. 26596 x 16622 over the 12.4" 16:10 panel
	 * (about 267 x 167 mm) is 100 units/mm, Wacom's usual pitch.
	 */
	input_abs_set_res(input, ABS_X, 100);
	input_abs_set_res(input, ABS_Y, 100);
	input_set_abs_params(input, ABS_PRESSURE, 0, get_unaligned_be16(&q[5]), 0, 0);
	input_set_abs_params(input, ABS_DISTANCE, 0, q[13], 0, 0);
	input_set_abs_params(input, ABS_TILT_X, -q[11], q[11], 0, 0);
	input_set_abs_params(input, ABS_TILT_Y, -q[12], q[12], 0, 0);
	touchscreen_parse_properties(input, false, &w->prop);
	input_set_capability(input, EV_KEY, BTN_TOOL_PEN);
	input_set_capability(input, EV_KEY, BTN_TOOL_RUBBER);
	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_capability(input, EV_KEY, BTN_STYLUS);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	ret = input_register_device(input);
	if (ret)
		return ret;

	/* trigger from the device tree: level low, as stock (IRQF_TRIGGER_LOW) */
	return devm_request_threaded_irq(dev, client->irq, NULL, w9021_irq, IRQF_ONESHOT,
					 "wacom-w9021", w);
}

static const struct of_device_id w9021_of_match[] = {
	{ .compatible = "wacom,w9021" },
	{ }
};
MODULE_DEVICE_TABLE(of, w9021_of_match);

static struct i2c_driver w9021_driver = {
	.driver = {
		.name = "wacom-w9021",
		.of_match_table = w9021_of_match,
	},
	.probe = w9021_probe,
};
module_i2c_driver(w9021_driver);

MODULE_DESCRIPTION("Wacom W9021 EMR digitizer (Samsung S Pen)");
MODULE_LICENSE("GPL");
