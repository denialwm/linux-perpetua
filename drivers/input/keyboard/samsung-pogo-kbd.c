// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Book Cover Keyboard on the pogo connector (EF-DT970, EF-DT730 Slim), as on the Galaxy Tab
 * S7 FE.
 *  The keyboard's STM32 MCU sits behind the pogo pins at I2C 0x2a. The tablet powers it (vdd) while
 * the connect line is high. Protocol from Samsung's stm32_pogo driver
 * (drivers/input/sec_input/stm32/, v1): every transfer starts with a 3-byte header {length lo,
 * length hi, endpoint}, length counting the header itself. An event is read while the interrupt
 * line is low (level, as stock): write the header {3, 0, 1}, read a 3-byte header back {length,
 * event id}, then the event. An empty event is the MCU's hello after power on; the MCU reports keys
 * only once the host has read its version, mode, CRC and touchpad version, as stock does. Key
 * events are little-endian u16 words: bit 0 press, bits 1-5 column, bits 6-8 row. The keyboard's
 * firmware update, the EF-DT970's touchpad and the caps lock LED are not handled.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define POGO_ID_MCU		1
#define POGO_ID_TOUCHPAD	2
#define POGO_ID_KEYPAD		3
#define POGO_ID_HALL		4
#define POGO_CMD_GET_MODE	0x01
#define POGO_CMD_VERSION	0x02
#define POGO_CMD_CRC		0x03
#define POGO_CMD_ABORT		0x17	/* leave the bootloader for the application */
#define POGO_CMD_TC_VERSION	0x18
#define POGO_MODE_APP		1
#define POGO_MODE_EXCEPTION	3
#define POGO_MAX_EVENT		100
#define POGO_ROWS		8
#define POGO_COLS		24
#define POGO_ROW_SHIFT		5	/* get_count_order(POGO_COLS) */
#define POGO_MAX_CYCLES		3	/* power cycles per attach before giving up */

struct pogo_kbd {
	struct i2c_client *client;
	struct regulator *vdd;
	struct gpio_desc *conn;
	struct input_dev *input;
	struct delayed_work conn_work;
	struct mutex lock;	/* powered, input */
	bool powered;
	bool irq_on;		/* the event interrupt is enabled */
	bool stuck;		/* an event read failed: power-cycle the keyboard */
	unsigned long on_since;	/* jiffies of the last power on */
	int cycles;		/* power cycles since the last good handshake */
	int model;
	unsigned short keycode[POGO_ROWS << POGO_ROW_SHIFT];
};

static const char * const pogo_names[] = {
	"Book Cover Keyboard (EF-DT970)",
	"Book Cover Keyboard Slim (EF-DT730)",
};

static int pogo_send(struct pogo_kbd *kbd, const u8 *buf, int len)
{
	int ret = i2c_master_send(kbd->client, buf, len);

	return ret == len ? 0 : ret < 0 ? ret : -EIO;
}

static int pogo_recv(struct pogo_kbd *kbd, u8 *buf, int len)
{
	int ret = i2c_master_recv(kbd->client, buf, len);

	return ret == len ? 0 : ret < 0 ? ret : -EIO;
}

/*
 * Header plus optional command byte, then the answer's header; returns the answer's payload length.
 */
static int pogo_request(struct pogo_kbd *kbd, u8 ep, const u8 *cmd, u8 *id)
{
	u8 hdr[3] = { cmd ? 4 : 3, 0, ep };
	int ret, len;

	ret = pogo_send(kbd, hdr, 3);
	if (!ret && cmd)
		ret = pogo_send(kbd, cmd, 1);
	if (!ret)
		ret = pogo_recv(kbd, hdr, 3);
	if (ret)
		return ret;
	len = get_unaligned_le16(hdr) - 3;
	if (len < 0 || len > POGO_MAX_EVENT)
		return -EPROTO;
	if (id)
		*id = hdr[2];
	return len;
}

static int pogo_ep_read(struct pogo_kbd *kbd, u8 ep, u8 cmd, u8 *val, int size)
{
	int len = pogo_request(kbd, ep, &cmd, NULL);

	if (len < 0)
		return len;
	if (!len || len > size)
		return -EPROTO;
	return pogo_recv(kbd, val, len);
}

static int pogo_reg_read(struct pogo_kbd *kbd, u8 cmd, u8 *val, int size)
{
	return pogo_ep_read(kbd, POGO_ID_MCU, cmd, val, size);
}

static void pogo_release_keys(struct pogo_kbd *kbd)
{
	int i;

	if (!kbd->input)
		return;
	for (i = 0; i < ARRAY_SIZE(kbd->keycode); i++)
		if (kbd->keycode[i] && test_bit(kbd->keycode[i], kbd->input->key))
			input_report_key(kbd->input, kbd->keycode[i], 0);
	input_sync(kbd->input);
}

static void pogo_remove_input(struct pogo_kbd *kbd)
{
	if (!kbd->input)
		return;
	pogo_release_keys(kbd);
	input_unregister_device(kbd->input);
	kbd->input = NULL;
}

/* The MCU's hello: read its model, make sure it runs the application, register the keyboard. */
static void pogo_attach(struct pogo_kbd *kbd)
{
	struct device *dev = &kbd->client->dev;
	struct input_dev *input;
	u8 ver[4], mode, cmd = POGO_CMD_ABORT, hdr[3] = { 4, 0, POGO_ID_MCU };
	int ret;

	ret = pogo_reg_read(kbd, POGO_CMD_VERSION, ver, sizeof(ver));
	if (ret) {
		dev_err(dev, "no version from the keyboard: %d\n", ret);
		return;
	}
	kbd->model = ver[1] < ARRAY_SIZE(pogo_names) ? ver[1] : 0;
	dev_info(dev, "%s, firmware %u.%u, hw rev %u\n", pogo_names[kbd->model],
		 ver[3], ver[2], ver[0]);

	ret = pogo_reg_read(kbd, POGO_CMD_GET_MODE, &mode, 1);
	dev_dbg(dev, "mode read %d, mode %u\n", ret, ret ? 0 : mode);
	if (!ret && mode != POGO_MODE_APP && mode != POGO_MODE_EXCEPTION) {
		dev_info(dev, "keyboard in mode %u, starting its application\n", mode);
		if (!pogo_send(kbd, hdr, 3))
			pogo_send(kbd, &cmd, 1);
		msleep(200);
	}

	/*
	 * The rest of stock's check_ic_work: its CRC and the touchpad's version (0xff00 on the
	 * Slim, which has none). The MCU reports keys only after this handshake (no
	 * interrupt on any key without it).
	 */
	ret = pogo_reg_read(kbd, POGO_CMD_CRC, ver, sizeof(ver));
	dev_dbg(dev, "crc read %d: %4ph\n", ret, ver);
	{
		u8 tc[6] = { 0 };

		ret = pogo_ep_read(kbd, POGO_ID_TOUCHPAD, POGO_CMD_TC_VERSION, tc, sizeof(tc));
		dev_dbg(dev, "touchpad version read %d: %6ph\n", ret, tc);
	}

	kbd->cycles = 0;
	if (kbd->input)
		return;
	input = input_allocate_device();
	if (!input)
		return;
	input->name = pogo_names[kbd->model];
	input->phys = "pogo/input0";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x04e8;
	input->id.product = 0xa035;
	input->dev.parent = dev;
	input_set_capability(input, EV_MSC, MSC_SCAN);
	/* The Slim cover's matrix differs from the EF-DT970's in two keys (stock keymap2) */
	ret = matrix_keypad_build_keymap(NULL,
					 kbd->model == 1 ? "samsung,keymap-slim" : "linux,keymap",
					 POGO_ROWS, POGO_COLS, kbd->keycode, input);
	if (!ret)
		ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "keyboard input device: %d\n", ret);
		input_free_device(input);
		return;
	}
	kbd->input = input;
}

static void pogo_keys(struct pogo_kbd *kbd, const u8 *ev, int len)
{
	int i;

	if (!kbd->input)
		return;
	for (i = 0; i + 1 < len; i += 2) {
		u16 w = get_unaligned_le16(ev + i);
		unsigned int row = (w >> 6) & 7, col = (w >> 1) & 0x1f;
		int code = MATRIX_SCAN_CODE(row, col, POGO_ROW_SHIFT);

		if (col >= POGO_COLS)
			continue;
		input_event(kbd->input, EV_MSC, MSC_SCAN, code);
		input_report_key(kbd->input, kbd->keycode[code], w & 1);
		input_sync(kbd->input);
	}
}

static irqreturn_t pogo_irq(int irq, void *data)
{
	struct pogo_kbd *kbd = data;
	u8 ev[POGO_MAX_EVENT], id = 0;
	int len;

	mutex_lock(&kbd->lock);
	if (!kbd->powered)
		goto out;
	/* Endpoint 1: caps lock LED off (2 would light it) */
	len = pogo_request(kbd, POGO_ID_MCU, NULL, &id);
	dev_dbg(&kbd->client->dev, "irq: event id %u, length %d\n", id, len);
	if (len == 0) {
		pogo_attach(kbd);
		goto out;
	}
	if (len < 0 || pogo_recv(kbd, ev, len)) {
		/* An MCU in this state stays silent, as after a driver reload: power-cycle it */
		dev_err_ratelimited(&kbd->client->dev, "event read failed: %d\n", len);
		kbd->stuck = true;
		kbd->irq_on = false;
		disable_irq_nosync(irq);
		mod_delayed_work(system_dfl_wq, &kbd->conn_work, 0);
		goto out;
	}
	dev_dbg(&kbd->client->dev, "event %*ph\n", len, ev);
	if (id == POGO_ID_KEYPAD)
		pogo_keys(kbd, ev, len);
	else if (id == POGO_ID_HALL)	/* cover folded or closed: nothing stays pressed */
		pogo_release_keys(kbd);
out:
	mutex_unlock(&kbd->lock);
	return IRQ_HANDLED;
}

/*
 * Under kbd->lock; returns with it released. The event interrupt is off while the MCU is
 * unpowered.
 */
static void pogo_power(struct pogo_kbd *kbd, bool on)
{
	struct device *dev = &kbd->client->dev;
	bool irq_on = kbd->irq_on;
	int ret;

	if (!on) {
		kbd->powered = false;
		kbd->irq_on = false;
		mutex_unlock(&kbd->lock);
		if (irq_on)
			disable_irq(kbd->client->irq);
		regulator_disable(kbd->vdd);
		return;
	}
	ret = regulator_enable(kbd->vdd);
	if (ret) {
		dev_err(dev, "vdd on: %d\n", ret);
		mutex_unlock(&kbd->lock);
		return;
	}
	kbd->powered = true;
	kbd->stuck = false;
	kbd->on_since = jiffies;
	msleep(50);
	kbd->irq_on = true;
	mutex_unlock(&kbd->lock);
	enable_irq(kbd->client->irq);
	/* look again once the MCU has had time to say hello */
	queue_delayed_work(system_dfl_wq, &kbd->conn_work, HZ + 1);
}

static void pogo_conn_work(struct work_struct *work)
{
	struct pogo_kbd *kbd = container_of(work, struct pogo_kbd, conn_work.work);
	struct device *dev = &kbd->client->dev;
	bool attached = gpiod_get_value_cansleep(kbd->conn);

	mutex_lock(&kbd->lock);
	if (attached && kbd->powered && (kbd->stuck || !kbd->input)) {
		/*
		 * No hello within a second of power on (the MCU keeps its connect line
		 * pulsing until a host has handshaken with it), or a failed event read. An
		 * MCU left powered by a previous driver instance never says hello again;
		 * both clear on a power cycle, as on a physical reattach.
		 */
		if (!kbd->stuck && time_before(jiffies, kbd->on_since + HZ))
			goto out;
		if (kbd->cycles >= POGO_MAX_CYCLES) {
			if (kbd->cycles++ == POGO_MAX_CYCLES)
				dev_err(dev, "keyboard not answering: detach and reattach it\n");
			goto out;
		}
		kbd->cycles++;
		dev_info(dev, "keyboard not answering (%s), power-cycling it\n",
			 kbd->stuck ? "read error" : "no hello");
		pogo_release_keys(kbd);
		pogo_power(kbd, false);
		msleep(100);
		mutex_lock(&kbd->lock);
		pogo_power(kbd, true);
		return;
	}
	if (attached == kbd->powered)
		goto out;
	dev_info(dev, "keyboard %s\n", attached ? "attached" : "detached");
	if (attached) {
		kbd->cycles = 0;
		pogo_power(kbd, true);
		return;
	}
	pogo_remove_input(kbd);
	pogo_power(kbd, false);
	return;
out:
	mutex_unlock(&kbd->lock);
}

static irqreturn_t pogo_conn_irq(int irq, void *data)
{
	struct pogo_kbd *kbd = data;

	/*
	 * The pins bounce as the cover seats: act on the line once it has settled. Do not push a
	 * pending check back: an MCU waiting for its handshake pulses this line without end, which
	 * would starve the worker (and with it the no-hello recovery).
	 */
	queue_delayed_work(system_dfl_wq, &kbd->conn_work, msecs_to_jiffies(250));
	return IRQ_HANDLED;
}

static int pogo_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct pogo_kbd *kbd;
	int ret;

	kbd = devm_kzalloc(dev, sizeof(*kbd), GFP_KERNEL);
	if (!kbd)
		return -ENOMEM;
	kbd->client = client;
	mutex_init(&kbd->lock);
	INIT_DELAYED_WORK(&kbd->conn_work, pogo_conn_work);
	i2c_set_clientdata(client, kbd);

	kbd->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(kbd->vdd))
		return dev_err_probe(dev, PTR_ERR(kbd->vdd), "vdd\n");
	kbd->conn = devm_gpiod_get(dev, "connect", GPIOD_IN);
	if (IS_ERR(kbd->conn))
		return dev_err_probe(dev, PTR_ERR(kbd->conn), "connect gpio\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, pogo_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN, "pogo-kbd", kbd);
	if (ret)
		return dev_err_probe(dev, ret, "irq\n");
	ret = devm_request_threaded_irq(dev, gpiod_to_irq(kbd->conn), NULL, pogo_conn_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					"pogo-kbd-connect", kbd);
	if (ret)
		return dev_err_probe(dev, ret, "connect irq\n");

	queue_delayed_work(system_dfl_wq, &kbd->conn_work, 0);
	return 0;
}

static void pogo_remove(struct i2c_client *client)
{
	struct pogo_kbd *kbd = i2c_get_clientdata(client);

	disable_irq(gpiod_to_irq(kbd->conn));
	disable_irq(client->irq);
	cancel_delayed_work_sync(&kbd->conn_work);
	mutex_lock(&kbd->lock);
	/* also registered when a power cycle left the keyboard unpowered */
	pogo_remove_input(kbd);
	if (kbd->powered)
		pogo_power(kbd, false);
	else
		mutex_unlock(&kbd->lock);
}

static const struct of_device_id pogo_of_match[] = {
	{ .compatible = "samsung,pogo-keyboard" },
	{ }
};
MODULE_DEVICE_TABLE(of, pogo_of_match);

static struct i2c_driver pogo_driver = {
	.driver = {
		.name = "samsung-pogo-kbd",
		.of_match_table = pogo_of_match,
	},
	.probe = pogo_probe,
	.remove = pogo_remove,
};
module_i2c_driver(pogo_driver);

MODULE_DESCRIPTION("Samsung Book Cover Keyboard on the pogo connector");
MODULE_LICENSE("GPL");
