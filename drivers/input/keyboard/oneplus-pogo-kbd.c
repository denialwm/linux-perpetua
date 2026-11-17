// SPDX-License-Identifier: GPL-2.0-only
/*
 * OnePlus Pad 3 pogo-pin keyboard
 *
 * The keyboard cover talks to the tablet over one data line driven by a
 * UART: the tablet enables its line driver only while it transmits and hears
 * its own frames. A keyboard pulls the line low when attached; the tablet then
 * powers it and the keyboard announces itself and sends a heartbeat every
 * 100 ms, key reports (HID boot keyboard and consumer usages) and touchpad
 * frames. The keyboard is gone when nothing arrives for 500 ms.
 *
 * The keyboard is also a Bluetooth keyboard. Its first frames carry its
 * Bluetooth address, which is exposed for userspace to pair it. It has a
 * battery, whose level it reports.
 *
 * Frame: 55 x8, F1 (F2 for a repeat), source, destination, command, length,
 * data[length], CRC16 (big endian, polynomial 0x8005, initial value 0xc596,
 * over F1..data), FE, AA x4.
 *
 * The protocol and timings follow OnePlus' driver.
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define POGO_SYNC		0x55
#define POGO_START		0xf1
#define POGO_START_REPEAT	0xf2
#define POGO_END		0xfe
#define POGO_TAIL		0xaa
#define POGO_ADDR_KBD		0xa1
#define POGO_ADDR_PAD		0xa2

#define POGO_CRC_POLY		0x8005
#define POGO_CRC_INIT		0xc596

#define POGO_CMD_KEY		0x01
#define POGO_CMD_MEDIA		0x02
#define POGO_CMD_TOUCHPAD	0x03
#define POGO_CMD_PARAM_SET	0x20
#define POGO_CMD_SYNC_UPLOAD	0x2f
#define POGO_CMD_GENERAL	0x3a
#define POGO_CMD_GENERAL_ACK	0x3b

#define POGO_SYNC_POWER_UP	0x01
#define POGO_SYNC_HEARTBEAT	0x05
#define POGO_GENERAL_SLEEP	0x02
#define POGO_GENERAL_BATTERY	0x0e

/* sync, start, source, destination, command, length, CRC, end, tail */
#define POGO_FRAME_OVERHEAD	(8 + 4 + 1 + 2 + 1 + 4)
#define POGO_MAX_DATA		64
#define POGO_RX_MAX		256

#define POGO_TX_ON_US		450
#define POGO_TX_OFF_US		300
#define POGO_TX_TIMEOUT		msecs_to_jiffies(50)
#define POGO_ACK_TIMEOUT	msecs_to_jiffies(100)

#define POGO_DETECT_MS		20
#define POGO_DETECT_ON		6	/* line low for 120 ms: attached */
#define POGO_DETECT_OFF		3
#define POGO_HEARTBEAT_MS	50
#define POGO_RX_TIMEOUT		msecs_to_jiffies(500)
#define POGO_POWER_UP_TIME	msecs_to_jiffies(2000)
#define POGO_RETRY_MS		50
#define POGO_RETRY_ON		12	/* line low for 600 ms: power up again */
#define POGO_RETRY_OFF		20
#define POGO_RETRY_MAX		20

#define POGO_FINGERS		5

#define POGO_KBD_VENDOR		0x22d9
#define POGO_KBD_PRODUCT	0x3869

enum pogo_state {
	POGO_IDLE,		/* unpowered, waiting for the line to go low */
	POGO_DETECT,		/* debouncing the line */
	POGO_POWERED,		/* powered, waiting for or talking to a keyboard */
	POGO_RETRY,		/* lost the keyboard, watching the line */
};

struct pogo_kbd {
	struct serdev_device *serdev;
	struct device *dev;
	struct regulator *vcc;
	struct gpio_desc *tx_gpio;
	struct gpio_desc *detect_gpio;
	int detect_irq;

	u32 tp_max[2];
	u32 tp_res[2];
	bool has_touchpad;

	/*
	 * lock serializes the state machine and transmission, rx_lock the
	 * receive path and the input devices; lock is taken first. The
	 * receive path never takes lock, so a sender can wait for an answer.
	 */
	struct mutex lock;
	struct mutex rx_lock;
	enum pogo_state state;
	int lows, highs, retries;
	unsigned long powered_at;
	unsigned long last_rx;
	bool connected;
	bool caps_lock;
	bool mute;
	bool mic_mute;
	struct delayed_work poll_work;
	struct work_struct connect_work;
	struct work_struct led_work;
	struct work_struct battery_work;
	struct led_classdev mic_mute_led;

	struct input_dev *kbd;
	struct input_dev *tp;
	u8 keys[8];
	u16 media[2];
	unsigned long fingers;

	/* receive state, under rx_lock */
	u8 rx[POGO_RX_MAX];
	unsigned int rx_len;
	unsigned int sync_cnt;

	/*
	 * Bluetooth address and brand from the hello frames, under rx_lock,
	 * and the ones announced to userspace, under lock; the address most
	 * significant byte first.
	 */
	u8 rx_addr[6];
	u8 rx_brand;
	bool rx_have_addr;
	u8 bt_addr[6];
	u8 bt_brand;
	bool bt_have_addr;

	/*
	 * Battery level in percent, -1 until the keyboard reports it; the
	 * battery is registered on the first report, under lock.
	 */
	int battery_level;
	struct power_supply_desc battery_desc;
	struct power_supply *battery;

	/* waiting for an acknowledgment */
	struct completion ack;
	u8 ack_cmd;
	u8 ack_sub;
};

/*
 * HID keyboard usage to key code, as hid-input's table, except for the keys
 * the keyboard sends as F13-F24 for a function printed on the key cap:
 * F13 (0x68) mic mute, F16/F17 (0x6b/0x6c) touchpad off/on (the keyboard
 * itself turns the touchpad off and on, and sends the new state), F23
 * (0x72) search and F24 (0x73) lock screen. OnePlus' key layout gives
 * F13, F23 and F24 the same functions (MUTE, SEARCH, LOCK).
 */
#define unk KEY_UNKNOWN
static const u16 pogo_hid_keyboard[256] = {
	0, 0, 0, 0, 30, 48, 46, 32,
	18, 33, 34, 35, 23, 36, 37, 38,
	50, 49, 24, 25, 16, 19, 31, 20,
	22, 47, 17, 45, 21, 44, 2, 3,
	4, 5, 6, 7, 8, 9, 10, 11,
	28, 1, 14, 15, 57, 12, 13, 26,
	27, 43, 43, 39, 40, 41, 51, 52,
	53, 58, 59, 60, 61, 62, 63, 64,
	65, 66, 67, 68, 87, 88, 99, 70,
	119, 110, 102, 104, 111, 107, 109, 106,
	105, 108, 103, 69, 98, 55, 74, 78,
	96, 79, 80, 81, 75, 76, 77, 71,
	72, 73, 82, 83, 86, 127, 116, 117,
	KEY_MICMUTE, 184, 185, KEY_TOUCHPAD_OFF,
	KEY_TOUCHPAD_ON, 188, 189, 190,
	191, 192, KEY_SEARCH, KEY_SCREENLOCK, 134, 138, 130, 132,
	128, 129, 131, 137, 133, 135, 136, 113,
	115, 114, unk, unk, unk, 121, unk, 89,
	93, 124, 92, 94, 95, unk, unk, unk,
	122, 123, 90, 91, 85, unk, unk, unk,
	unk, unk, unk, unk, 111, unk, unk, unk,
	unk, unk, unk, unk, unk, unk, unk, unk,
	unk, unk, unk, unk, unk, unk, unk, unk,
	unk, unk, unk, unk, unk, unk, 179, 180,
	unk, unk, unk, unk, unk, unk, unk, unk,
	unk, unk, unk, unk, unk, unk, unk, unk,
	unk, unk, unk, unk, unk, unk, unk, unk,
	unk, unk, unk, unk, unk, unk, unk, unk,
	111, unk, unk, unk, unk, unk, unk, unk,
	29, 42, 56, 125, 97, 54, 100, 126,
	164, 166, 165, 163, 161, 115, 114, 113,
	150, 158, 159, 128, 136, 177, 178, 176,
	142, 152, 173, 140, unk, unk, unk, unk,
};
#undef unk

/*
 * Consumer usages the keyboard sends; 0x38e-0x399 are OnePlus' own. There is
 * no key code for split screen, which becomes a programmable key like the
 * two custom application keys; the "super interconnect" key, which links the
 * tablet to a phone, becomes the phone link key.
 */
static const struct {
	u16 usage;
	u16 code;
} pogo_media_keys[] = {
	{ 0x006f, KEY_BRIGHTNESSUP },
	{ 0x0070, KEY_BRIGHTNESSDOWN },
	{ 0x00b5, KEY_NEXTSONG },
	{ 0x00b6, KEY_PREVIOUSSONG },
	{ 0x00cd, KEY_PLAYPAUSE },
	{ 0x00e2, KEY_MUTE },
	{ 0x00e9, KEY_VOLUMEUP },
	{ 0x00ea, KEY_VOLUMEDOWN },
	{ 0x01d0, KEY_FN },
	{ 0x0224, KEY_ESC },		/* AC Back, on the Esc key */
	{ 0x0244, KEY_APPSELECT },
	{ 0x02fa, KEY_ASSISTANT },
	{ 0x038e, KEY_SCREENLOCK },
	{ 0x0390, KEY_LANGUAGE },
	{ 0x0391, KEY_MICMUTE },
	{ 0x0392, KEY_TOUCHPAD_TOGGLE },
	{ 0x0393, KEY_SEARCH },
	{ 0x0394, KEY_FULL_SCREEN },
	{ 0x0395, KEY_PROG3 },		/* split screen */
	{ 0x0397, KEY_LINK_PHONE },	/* super interconnect */
	{ 0x0398, KEY_PROG1 },
	{ 0x0399, KEY_PROG2 },
};

/* Bluetooth names of the keyboards, by the brand in their hello frames */
static const char * const pogo_bt_names[] = {
	[1] = "OPLUS Pad 4 Pro Keyboard",
	[2] = "OnePlus Pad 2 Pro Keyboard",
	[3] = "OnePlus Pad 3 Keyboard",
};

static u16 pogo_crc16(const u8 *buf, size_t len)
{
	u16 crc = POGO_CRC_INIT;
	size_t i;
	int bit;

	for (i = 0; i < len; i++) {
		crc ^= buf[i] << 8;
		for (bit = 0; bit < 8; bit++)
			crc = crc & 0x8000 ? crc << 1 ^ POGO_CRC_POLY : crc << 1;
	}

	return crc;
}

/* Called with the lock held. */
static int pogo_send(struct pogo_kbd *pk, u8 cmd, const u8 *data, u8 len)
{
	u8 frame[POGO_FRAME_OVERHEAD + POGO_MAX_DATA];
	unsigned int n = 0;
	int ret;
	u16 crc;

	if (len > POGO_MAX_DATA)
		return -EINVAL;

	memset(frame, POGO_SYNC, 8);
	n = 8;
	frame[n++] = POGO_START;
	frame[n++] = POGO_ADDR_PAD;
	frame[n++] = POGO_ADDR_KBD;
	frame[n++] = cmd;
	frame[n++] = len;
	memcpy(&frame[n], data, len);
	n += len;
	crc = pogo_crc16(&frame[8], n - 8);
	frame[n++] = crc >> 8;
	frame[n++] = crc & 0xff;
	frame[n++] = POGO_END;
	memset(&frame[n], POGO_TAIL, 4);
	n += 4;

	gpiod_set_value_cansleep(pk->tx_gpio, 1);
	usleep_range(POGO_TX_ON_US, POGO_TX_ON_US + 50);

	ret = serdev_device_write(pk->serdev, frame, n, POGO_TX_TIMEOUT);
	if (ret >= 0)
		serdev_device_wait_until_sent(pk->serdev, POGO_TX_TIMEOUT);

	usleep_range(POGO_TX_OFF_US, POGO_TX_OFF_US + 50);
	gpiod_set_value_cansleep(pk->tx_gpio, 0);

	if (ret < 0)
		return ret;

	return ret == n ? 0 : -EIO;
}

/* Send a general command and wait for its acknowledgment. Lock held. */
static int pogo_general(struct pogo_kbd *pk, const u8 *data, u8 len)
{
	int ret = -ETIMEDOUT;
	int i;

	pk->ack_sub = data[0];
	WRITE_ONCE(pk->ack_cmd, POGO_CMD_GENERAL_ACK);

	for (i = 0; i < 3; i++) {
		reinit_completion(&pk->ack);
		ret = pogo_send(pk, POGO_CMD_GENERAL, data, len);
		if (ret)
			continue;
		if (wait_for_completion_timeout(&pk->ack, POGO_ACK_TIMEOUT))
			break;
		ret = -ETIMEDOUT;
	}

	WRITE_ONCE(pk->ack_cmd, 0);

	return ret;
}

/* Caps lock, (none), mute, mic mute. Called with the lock held. */
static int pogo_set_leds(struct pogo_kbd *pk)
{
	u8 leds[] = {
		0x0d, 0x04, READ_ONCE(pk->caps_lock), 0x00,
		READ_ONCE(pk->mute), READ_ONCE(pk->mic_mute),
	};

	return pogo_send(pk, POGO_CMD_PARAM_SET, leds, sizeof(leds));
}

/* Tell the keyboard the tablet goes to sleep (true) or wakes up. Lock held. */
static int pogo_set_sleep(struct pogo_kbd *pk, bool sleep)
{
	u8 cmd[] = { POGO_GENERAL_SLEEP, 0x01, sleep };

	return pogo_general(pk, cmd, sizeof(cmd));
}

static int pogo_kbd_event(struct input_dev *input, unsigned int type,
			  unsigned int code, int value)
{
	struct pogo_kbd *pk = input_get_drvdata(input);

	if (type != EV_LED)
		return -EINVAL;

	switch (code) {
	case LED_CAPSL:
		WRITE_ONCE(pk->caps_lock, !!value);
		break;
	case LED_MUTE:
		WRITE_ONCE(pk->mute, !!value);
		break;
	default:
		return -EINVAL;
	}
	schedule_work(&pk->led_work);

	return 0;
}

/* There is no input LED for mic mute. */
static void pogo_mic_mute_set(struct led_classdev *led, enum led_brightness value)
{
	struct pogo_kbd *pk = container_of(led, struct pogo_kbd, mic_mute_led);

	WRITE_ONCE(pk->mic_mute, !!value);
	schedule_work(&pk->led_work);
}

static void pogo_led_work(struct work_struct *work)
{
	struct pogo_kbd *pk = container_of(work, struct pogo_kbd, led_work);

	guard(mutex)(&pk->lock);

	if (pk->connected)
		pogo_set_leds(pk);
}

static struct input_dev *pogo_alloc_input(struct pogo_kbd *pk, const char *name)
{
	struct input_dev *input;

	input = input_allocate_device();
	if (!input)
		return NULL;

	input->name = name;
	input->phys = "pogo/input0";
	input->id.bustype = BUS_HOST;
	input->id.vendor = POGO_KBD_VENDOR;
	input->id.product = POGO_KBD_PRODUCT;
	input->dev.parent = pk->dev;
	input_set_drvdata(input, pk);

	return input;
}

static int pogo_register_keyboard(struct pogo_kbd *pk)
{
	struct input_dev *input;
	int i, ret;

	input = pogo_alloc_input(pk, "OnePlus Pad 3 Keyboard");
	if (!input)
		return -ENOMEM;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_LED, input->evbit);
	__set_bit(EV_REP, input->evbit);
	__set_bit(LED_CAPSL, input->ledbit);
	__set_bit(LED_MUTE, input->ledbit);
	input_set_capability(input, EV_MSC, MSC_SCAN);

	for (i = 0; i < ARRAY_SIZE(pogo_hid_keyboard); i++)
		if (pogo_hid_keyboard[i] && pogo_hid_keyboard[i] != KEY_UNKNOWN)
			__set_bit(pogo_hid_keyboard[i], input->keybit);
	for (i = 0; i < ARRAY_SIZE(pogo_media_keys); i++)
		__set_bit(pogo_media_keys[i].code, input->keybit);

	input->event = pogo_kbd_event;

	ret = input_register_device(input);
	if (ret) {
		input_free_device(input);
		return ret;
	}

	scoped_guard(mutex, &pk->rx_lock) {
		memset(pk->keys, 0, sizeof(pk->keys));
		memset(pk->media, 0, sizeof(pk->media));
		pk->kbd = input;
	}

	return 0;
}

static int pogo_register_touchpad(struct pogo_kbd *pk)
{
	struct input_dev *input;
	int ret;

	input = pogo_alloc_input(pk, "OnePlus Pad 3 Keyboard Touchpad");
	if (!input)
		return -ENOMEM;

	input->phys = "pogo/input1";
	input_set_capability(input, EV_KEY, BTN_LEFT);
	input_set_capability(input, EV_KEY, BTN_RIGHT);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, pk->tp_max[0], 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, pk->tp_max[1], 0, 0);
	input_abs_set_res(input, ABS_MT_POSITION_X, pk->tp_res[0]);
	input_abs_set_res(input, ABS_MT_POSITION_Y, pk->tp_res[1]);
	__set_bit(INPUT_PROP_POINTER, input->propbit);

	ret = input_mt_init_slots(input, POGO_FINGERS, INPUT_MT_POINTER);
	if (ret)
		goto err_free;

	ret = input_register_device(input);
	if (ret)
		goto err_free;

	scoped_guard(mutex, &pk->rx_lock) {
		pk->fingers = 0;
		pk->tp = input;
	}

	return 0;

err_free:
	input_free_device(input);
	return ret;
}

/* Called with rx_lock held. */
static void pogo_release_all(struct pogo_kbd *pk)
{
	int i;

	if (pk->kbd) {
		for (i = 0; i < KEY_CNT; i++)
			if (test_bit(i, pk->kbd->key))
				input_report_key(pk->kbd, i, 0);
		input_sync(pk->kbd);
	}
	memset(pk->keys, 0, sizeof(pk->keys));
	memset(pk->media, 0, sizeof(pk->media));

	if (pk->tp && pk->fingers) {
		for (i = 0; i < POGO_FINGERS; i++) {
			input_mt_slot(pk->tp, i);
			input_mt_report_slot_inactive(pk->tp);
		}
		input_mt_sync_frame(pk->tp);
		input_report_key(pk->tp, BTN_LEFT, 0);
		input_report_key(pk->tp, BTN_RIGHT, 0);
		input_sync(pk->tp);
	}
	pk->fingers = 0;
}

/* Tell userspace that a keyboard came or went, with its address. */
static void pogo_send_event(struct pogo_kbd *pk, bool present)
{
	char addr[sizeof("ACCESSORY_BT_ADDRESS=00:00:00:00:00:00")];
	char *envp[] = {
		"ACCESSORY_TYPE=keyboard",
		present ? "ACCESSORY_PRESENT=1" : "ACCESSORY_PRESENT=0",
		addr,
		NULL
	};

	snprintf(addr, sizeof(addr), "ACCESSORY_BT_ADDRESS=%pM", pk->bt_addr);
	kobject_uevent_env(&pk->dev->kobj, KOBJ_CHANGE, envp);
}

/*
 * Announce the address the keyboard sent, if it is new. Called with the lock
 * held.
 */
static void pogo_update_address(struct pogo_kbd *pk)
{
	u8 addr[6];
	bool have;
	u8 brand;

	scoped_guard(mutex, &pk->rx_lock) {
		have = pk->rx_have_addr;
		memcpy(addr, pk->rx_addr, sizeof(addr));
		brand = pk->rx_brand;
	}

	if (!have || (pk->bt_have_addr && !memcmp(addr, pk->bt_addr, sizeof(addr))))
		return;

	/* a different keyboard after a quick re-attach */
	if (pk->bt_have_addr)
		pogo_send_event(pk, false);

	memcpy(pk->bt_addr, addr, sizeof(addr));
	pk->bt_brand = brand;
	pk->bt_have_addr = true;
	pogo_send_event(pk, true);
}

/* Called with the lock held. */
static void pogo_disconnect(struct pogo_kbd *pk)
{
	struct input_dev *kbd, *tp;

	if (!pk->connected)
		return;

	scoped_guard(mutex, &pk->rx_lock) {
		pogo_release_all(pk);
		pk->rx_have_addr = false;
		kbd = pk->kbd;
		tp = pk->tp;
		pk->kbd = NULL;
		pk->tp = NULL;
	}
	if (tp)
		input_unregister_device(tp);
	if (kbd)
		input_unregister_device(kbd);
	WRITE_ONCE(pk->connected, false);
	WRITE_ONCE(pk->battery_level, -1);
	if (pk->battery) {
		power_supply_unregister(pk->battery);
		pk->battery = NULL;
	}
	dev_info(pk->dev, "keyboard detached\n");

	if (pk->bt_have_addr) {
		pogo_send_event(pk, false);
		pk->bt_have_addr = false;
	}
}

/* Called with the lock held. */
static void pogo_power_on(struct pogo_kbd *pk)
{
	int ret;

	ret = regulator_enable(pk->vcc);
	if (ret) {
		dev_err(pk->dev, "failed to power the keyboard: %d\n", ret);
		return;
	}

	pk->state = POGO_POWERED;
	pk->powered_at = jiffies;
	WRITE_ONCE(pk->last_rx, jiffies);
	schedule_delayed_work(&pk->poll_work, msecs_to_jiffies(POGO_HEARTBEAT_MS));
}

/* Called with the lock held. */
static void pogo_power_off(struct pogo_kbd *pk)
{
	pogo_disconnect(pk);
	gpiod_set_value_cansleep(pk->tx_gpio, 0);
	regulator_disable(pk->vcc);
}

/* Called with the lock held. */
static void pogo_go_idle(struct pogo_kbd *pk)
{
	pk->state = POGO_IDLE;
	enable_irq(pk->detect_irq);
}

static void pogo_poll_work(struct work_struct *work)
{
	struct pogo_kbd *pk = container_of(to_delayed_work(work), struct pogo_kbd,
					   poll_work);
	int low;

	guard(mutex)(&pk->lock);

	low = gpiod_get_value_cansleep(pk->detect_gpio) == 0;

	switch (pk->state) {
	case POGO_IDLE:
		return;

	case POGO_DETECT:
		if (low) {
			pk->highs = 0;
			if (++pk->lows >= POGO_DETECT_ON) {
				pogo_power_on(pk);
				return;
			}
		} else {
			pk->lows = 0;
			if (++pk->highs >= POGO_DETECT_OFF) {
				pogo_go_idle(pk);
				return;
			}
		}
		schedule_delayed_work(&pk->poll_work, msecs_to_jiffies(POGO_DETECT_MS));
		return;

	case POGO_POWERED:
		if (time_after(jiffies, READ_ONCE(pk->last_rx) + POGO_RX_TIMEOUT) &&
		    time_after(jiffies, pk->powered_at + POGO_POWER_UP_TIME)) {
			dev_dbg(pk->dev, "no data from the keyboard, powering off\n");
			pogo_power_off(pk);
			if (pk->retries++ >= POGO_RETRY_MAX) {
				pogo_go_idle(pk);
				return;
			}
			pk->state = POGO_RETRY;
			pk->lows = 0;
			pk->highs = 0;
			schedule_delayed_work(&pk->poll_work, msecs_to_jiffies(POGO_RETRY_MS));
			return;
		}
		schedule_delayed_work(&pk->poll_work, msecs_to_jiffies(POGO_HEARTBEAT_MS));
		return;

	case POGO_RETRY:
		if (low) {
			pk->highs = 0;
			if (++pk->lows >= POGO_RETRY_ON) {
				pogo_power_on(pk);
				return;
			}
		} else {
			pk->lows = 0;
			if (++pk->highs >= POGO_RETRY_OFF) {
				pogo_go_idle(pk);
				return;
			}
		}
		schedule_delayed_work(&pk->poll_work, msecs_to_jiffies(POGO_RETRY_MS));
		return;
	}
}

static irqreturn_t pogo_detect_irq(int irq, void *data)
{
	struct pogo_kbd *pk = data;

	guard(mutex)(&pk->lock);

	if (pk->state != POGO_IDLE)
		return IRQ_HANDLED;

	disable_irq_nosync(irq);
	pk->state = POGO_DETECT;
	pk->lows = 0;
	pk->highs = 0;
	schedule_delayed_work(&pk->poll_work, msecs_to_jiffies(POGO_DETECT_MS));

	return IRQ_HANDLED;
}

static void pogo_connect_work(struct work_struct *work)
{
	struct pogo_kbd *pk = container_of(work, struct pogo_kbd, connect_work);
	int ret;

	guard(mutex)(&pk->lock);

	if (pk->state != POGO_POWERED)
		return;

	if (pk->connected) {
		/* powered up again after a quick re-attach */
		scoped_guard(mutex, &pk->rx_lock)
			pogo_release_all(pk);
		pogo_update_address(pk);
		return;
	}

	ret = pogo_register_keyboard(pk);
	if (!ret && pk->has_touchpad) {
		ret = pogo_register_touchpad(pk);
		if (ret) {
			scoped_guard(mutex, &pk->rx_lock) {
				input_unregister_device(pk->kbd);
				pk->kbd = NULL;
			}
		}
	}
	if (ret) {
		dev_err(pk->dev, "failed to register the input devices: %d\n", ret);
		return;
	}

	WRITE_ONCE(pk->connected, true);
	pk->retries = 0;
	dev_info(pk->dev, "keyboard attached\n");

	pogo_update_address(pk);
	pogo_set_leds(pk);
	/* the battery may have been reported before the keyboard registered */
	schedule_work(&pk->battery_work);
}

static const enum power_supply_property pogo_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int pogo_battery_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct pogo_kbd *pk = power_supply_get_drvdata(psy);
	int level;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		level = READ_ONCE(pk->battery_level);
		if (level < 0)
			return -ENODATA;
		val->intval = level;
		return 0;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "OnePlus Pad 3 Keyboard";
		return 0;
	default:
		return -EINVAL;
	}
}

static void pogo_battery_work(struct work_struct *work)
{
	struct pogo_kbd *pk = container_of(work, struct pogo_kbd, battery_work);
	struct power_supply_config cfg = { .drv_data = pk };
	struct power_supply *battery;

	guard(mutex)(&pk->lock);

	if (!pk->connected || READ_ONCE(pk->battery_level) < 0)
		return;

	if (pk->battery) {
		power_supply_changed(pk->battery);
		return;
	}

	battery = power_supply_register(pk->dev, &pk->battery_desc, &cfg);
	if (IS_ERR(battery)) {
		dev_err(pk->dev, "failed to register the battery: %pe\n", battery);
		return;
	}
	pk->battery = battery;
}

/*
 * Battery report: "0e length level state", the charging state in bits 5:4 of
 * state. Called with rx_lock held.
 */
static void pogo_handle_battery(struct pogo_kbd *pk, const u8 *d, u8 len)
{
	if (len < 3 || d[2] > 100)
		return;

	dev_dbg(pk->dev, "keyboard battery %u%%, state %#04x\n", d[2],
		len >= 4 ? d[3] : 0);

	if (READ_ONCE(pk->battery_level) != d[2]) {
		WRITE_ONCE(pk->battery_level, d[2]);
		schedule_work(&pk->battery_work);
	}
}

static void pogo_report_keys(struct pogo_kbd *pk, const u8 *d, u8 len)
{
	struct input_dev *input = pk->kbd;
	int i;

	if (len < 8)
		return;

	for (i = 0; i < 8; i++)
		input_report_key(input, pogo_hid_keyboard[0xe0 + i], (d[0] >> i) & 1);

	for (i = 2; i < 8; i++) {
		if (pk->keys[i] > 3 && !memchr(d + 2, pk->keys[i], 6)) {
			input_event(input, EV_MSC, MSC_SCAN, 0x70000 | pk->keys[i]);
			input_report_key(input, pogo_hid_keyboard[pk->keys[i]], 0);
		}
		if (d[i] > 3 && !memchr(pk->keys + 2, d[i], 6)) {
			input_event(input, EV_MSC, MSC_SCAN, 0x70000 | d[i]);
			input_report_key(input, pogo_hid_keyboard[d[i]], 1);
		}
	}

	input_sync(input);
	memcpy(pk->keys, d, sizeof(pk->keys));
}

static u16 pogo_media_code(u16 usage)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pogo_media_keys); i++)
		if (pogo_media_keys[i].usage == usage)
			return pogo_media_keys[i].code;

	return KEY_RESERVED;
}

static void pogo_report_media(struct pogo_kbd *pk, const u8 *d, u8 len)
{
	struct input_dev *input = pk->kbd;
	u16 new[2];
	u16 code;
	int i;

	if (len < 4)
		return;

	new[0] = d[0] | d[1] << 8;
	new[1] = d[2] | d[3] << 8;

	for (i = 0; i < 2; i++) {
		if (pk->media[i] && pk->media[i] != new[0] && pk->media[i] != new[1]) {
			code = pogo_media_code(pk->media[i]);
			if (code) {
				input_event(input, EV_MSC, MSC_SCAN, 0xc0000 | pk->media[i]);
				input_report_key(input, code, 0);
			}
		}
		if (new[i] && new[i] != pk->media[0] && new[i] != pk->media[1]) {
			code = pogo_media_code(new[i]);
			if (code) {
				input_event(input, EV_MSC, MSC_SCAN, 0xc0000 | new[i]);
				input_report_key(input, code, 1);
			} else {
				dev_dbg(pk->dev, "unknown consumer usage %#06x\n", new[i]);
			}
		}
	}

	input_sync(input);
	memcpy(pk->media, new, sizeof(new));
}

/*
 * 5 bytes per contact (id << 4 | down << 1, x, y little endian), the number
 * of contacts, the buttons (left in bit 0, right in bit 1).
 */
static void pogo_report_touchpad(struct pogo_kbd *pk, const u8 *d, u8 len)
{
	struct input_dev *input = pk->tp;
	unsigned int count, i, id;
	const u8 *c;
	bool down;

	if (!input || len < 2)
		return;

	count = d[len - 2];
	if (count > POGO_FINGERS || count * 5 + 2 > len)
		return;

	for (i = 0; i < count; i++) {
		c = &d[i * 5];
		id = c[0] >> 4;
		if (id >= POGO_FINGERS)
			continue;
		down = c[0] & BIT(1);

		input_mt_slot(input, id);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, down);
		if (down) {
			input_report_abs(input, ABS_MT_POSITION_X, get_unaligned_le16(&c[1]));
			input_report_abs(input, ABS_MT_POSITION_Y, get_unaligned_le16(&c[3]));
			__set_bit(id, &pk->fingers);
		} else {
			__clear_bit(id, &pk->fingers);
		}
	}

	input_mt_sync_frame(input);
	input_report_key(input, BTN_LEFT, d[len - 1] & BIT(0));
	input_report_key(input, BTN_RIGHT, d[len - 1] & BIT(1));
	input_sync(input);
}

/*
 * The hello frames carry the brand and the Bluetooth address: power-up
 * "01 02 brand addr[6]", heartbeat "05 02 state brand addr[6]". The brand
 * selects the keyboard's Bluetooth name; the address comes least significant
 * byte first, as on the air.
 * Called with rx_lock held.
 */
static void pogo_store_address(struct pogo_kbd *pk, const u8 *d, u8 len)
{
	unsigned int off = d[0] == POGO_SYNC_POWER_UP ? 3 : 4;
	int i;

	if (len < off + 6)
		return;

	dev_dbg(pk->dev, "hello %*ph\n", len, d);

	for (i = 0; i < 6; i++)
		pk->rx_addr[i] = d[off + 5 - i];
	pk->rx_brand = d[off - 1];
	pk->rx_have_addr = true;
}

/* Called with rx_lock held. */
static void pogo_handle_sync(struct pogo_kbd *pk, const u8 *d, u8 len)
{
	if (len < 3 || d[1] != 0x02)
		return;

	if (d[0] == POGO_SYNC_POWER_UP ||
	    (d[0] == POGO_SYNC_HEARTBEAT && !READ_ONCE(pk->connected))) {
		pogo_store_address(pk, d, len);
		schedule_work(&pk->connect_work);
		return;
	}

	if (d[0] != POGO_SYNC_HEARTBEAT)
		return;

	/* the keyboard sends heartbeats only while idle */
	if (pk->tp && pk->fingers)
		pogo_release_all(pk);

	if ((d[2] & BIT(0)) != READ_ONCE(pk->caps_lock))
		schedule_work(&pk->led_work);
}

static void pogo_handle_frame(struct pogo_kbd *pk, const u8 *f, unsigned int n)
{
	u8 cmd = f[3], len = f[4];
	const u8 *d = &f[5];

	/* our own frames come back on the shared line */
	if (f[1] != POGO_ADDR_KBD)
		return;

	WRITE_ONCE(pk->last_rx, jiffies);

	if (READ_ONCE(pk->ack_cmd) && cmd == pk->ack_cmd && len && d[0] == pk->ack_sub)
		complete(&pk->ack);

	switch (cmd) {
	case POGO_CMD_SYNC_UPLOAD:
		pogo_handle_sync(pk, d, len);
		break;
	case POGO_CMD_KEY:
		if (pk->kbd)
			pogo_report_keys(pk, d, len);
		break;
	case POGO_CMD_MEDIA:
		if (pk->kbd)
			pogo_report_media(pk, d, len);
		break;
	case POGO_CMD_TOUCHPAD:
		pogo_report_touchpad(pk, d, len);
		break;
	case POGO_CMD_GENERAL_ACK:
		if (len && d[0] == POGO_GENERAL_BATTERY)
			pogo_handle_battery(pk, d, len);
		break;
	default:
		break;
	}
}

static size_t pogo_receive_buf(struct serdev_device *serdev, const u8 *buf, size_t count)
{
	struct pogo_kbd *pk = serdev_device_get_drvdata(serdev);
	unsigned int need;
	size_t i;
	u16 crc;
	u8 c;

	guard(mutex)(&pk->rx_lock);

	for (i = 0; i < count; i++) {
		c = buf[i];

		if ((c == POGO_START || c == POGO_START_REPEAT) && pk->sync_cnt >= 4)
			pk->rx_len = 0;
		pk->sync_cnt = c == POGO_SYNC ? pk->sync_cnt + 1 : 0;

		if (pk->rx_len >= POGO_RX_MAX)
			continue;
		if (!pk->rx_len && c != POGO_START && c != POGO_START_REPEAT)
			continue;

		pk->rx[pk->rx_len++] = c;
		if (pk->rx_len < 5)
			continue;

		/* start, source, destination, command, length, data, CRC, end */
		need = 5 + pk->rx[4] + 3;
		if (pk->rx_len < need)
			continue;

		if (pk->rx[need - 1] == POGO_END &&
		    (pk->rx[2] == POGO_ADDR_KBD || pk->rx[2] == POGO_ADDR_PAD)) {
			crc = pk->rx[need - 3] << 8 | pk->rx[need - 2];
			if (crc == pogo_crc16(pk->rx, need - 3))
				pogo_handle_frame(pk, pk->rx, need);
			else
				dev_dbg_ratelimited(pk->dev, "bad CRC\n");
		}
		pk->rx_len = POGO_RX_MAX;	/* wait for the next start */
	}

	return count;
}

static ssize_t bt_address_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct pogo_kbd *pk = dev_get_drvdata(dev);
	u8 addr[6];

	scoped_guard(mutex, &pk->lock) {
		if (!pk->bt_have_addr)
			return -ENODATA;
		memcpy(addr, pk->bt_addr, sizeof(addr));
	}

	return sysfs_emit(buf, "%pM\n", addr);
}
static DEVICE_ATTR_RO(bt_address);

static ssize_t bt_name_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct pogo_kbd *pk = dev_get_drvdata(dev);
	u8 brand;

	scoped_guard(mutex, &pk->lock) {
		if (!pk->bt_have_addr)
			return -ENODATA;
		brand = pk->bt_brand;
	}

	if (brand >= ARRAY_SIZE(pogo_bt_names) || !pogo_bt_names[brand])
		return -ENODATA;

	return sysfs_emit(buf, "%s\n", pogo_bt_names[brand]);
}
static DEVICE_ATTR_RO(bt_name);

static struct attribute *pogo_attrs[] = {
	&dev_attr_bt_address.attr,
	&dev_attr_bt_name.attr,
	NULL
};
ATTRIBUTE_GROUPS(pogo);

static const struct serdev_device_ops pogo_serdev_ops = {
	.receive_buf = pogo_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static void pogo_shutdown(void *data)
{
	struct pogo_kbd *pk = data;

	disable_irq(pk->detect_irq);

	scoped_guard(mutex, &pk->lock) {
		if (pk->state == POGO_POWERED)
			pogo_power_off(pk);
		/* the works and the interrupt do nothing from here on */
		pk->state = POGO_IDLE;
	}

	cancel_delayed_work_sync(&pk->poll_work);
	cancel_work_sync(&pk->connect_work);
	cancel_work_sync(&pk->led_work);
	cancel_work_sync(&pk->battery_work);
}

static int pogo_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct pogo_kbd *pk;
	u32 speed;
	int ret;

	pk = devm_kzalloc(dev, sizeof(*pk), GFP_KERNEL);
	if (!pk)
		return -ENOMEM;

	pk->serdev = serdev;
	pk->dev = dev;
	pk->rx_len = POGO_RX_MAX;
	pk->battery_level = -1;
	init_completion(&pk->ack);
	ret = devm_mutex_init(dev, &pk->lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &pk->rx_lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&pk->poll_work, pogo_poll_work);
	INIT_WORK(&pk->connect_work, pogo_connect_work);
	INIT_WORK(&pk->led_work, pogo_led_work);
	INIT_WORK(&pk->battery_work, pogo_battery_work);
	serdev_device_set_drvdata(serdev, pk);

	pk->battery_desc.name = devm_kasprintf(dev, GFP_KERNEL, "oneplus-pogo-kbd-%s",
					       dev_name(dev));
	if (!pk->battery_desc.name)
		return -ENOMEM;
	pk->battery_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	pk->battery_desc.properties = pogo_battery_props;
	pk->battery_desc.num_properties = ARRAY_SIZE(pogo_battery_props);
	pk->battery_desc.get_property = pogo_battery_get_property;

	pk->mic_mute_led.name = devm_kasprintf(dev, GFP_KERNEL, "%s::micmute",
					       dev_name(dev));
	if (!pk->mic_mute_led.name)
		return -ENOMEM;
	pk->mic_mute_led.max_brightness = 1;
	pk->mic_mute_led.brightness_set = pogo_mic_mute_set;
	pk->mic_mute_led.default_trigger = "audio-micmute";

	ret = device_property_read_u32(dev, "max-speed", &speed);
	if (ret)
		return dev_err_probe(dev, ret, "missing max-speed\n");

	if (!device_property_read_u32_array(dev, "oneplus,touchpad-size", pk->tp_max, 2)) {
		pk->has_touchpad = true;
		device_property_read_u32_array(dev, "oneplus,touchpad-resolution",
					       pk->tp_res, 2);
	}

	pk->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(pk->vcc))
		return dev_err_probe(dev, PTR_ERR(pk->vcc), "failed to get vcc\n");

	pk->tx_gpio = devm_gpiod_get(dev, "tx-enable", GPIOD_OUT_LOW);
	if (IS_ERR(pk->tx_gpio))
		return dev_err_probe(dev, PTR_ERR(pk->tx_gpio), "failed to get tx-enable\n");

	pk->detect_gpio = devm_gpiod_get(dev, "detect", GPIOD_IN);
	if (IS_ERR(pk->detect_gpio))
		return dev_err_probe(dev, PTR_ERR(pk->detect_gpio), "failed to get detect\n");

	pk->detect_irq = gpiod_to_irq(pk->detect_gpio);
	if (pk->detect_irq < 0)
		return pk->detect_irq;

	serdev_device_set_client_ops(serdev, &pogo_serdev_ops);
	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to open the UART\n");

	serdev_device_set_baudrate(serdev, speed);
	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, pk->detect_irq, NULL, pogo_detect_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"pogo-detect", pk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the detect interrupt\n");

	ret = devm_add_action_or_reset(dev, pogo_shutdown, pk);
	if (ret)
		return ret;

	/* unregistered before pogo_shutdown() flushes its last update */
	ret = devm_led_classdev_register(dev, &pk->mic_mute_led);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register the mic mute LED\n");

	/* a keyboard attached at boot already holds the line low */
	scoped_guard(mutex, &pk->lock) {
		pk->state = POGO_DETECT;
		schedule_delayed_work(&pk->poll_work, 0);
	}

	return 0;
}

static int pogo_suspend(struct device *dev)
{
	struct pogo_kbd *pk = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&pk->poll_work);

	guard(mutex)(&pk->lock);

	if (pk->connected) {
		scoped_guard(mutex, &pk->rx_lock)
			pogo_release_all(pk);
		pogo_set_sleep(pk, true);
	}

	return 0;
}

static int pogo_resume(struct device *dev)
{
	struct pogo_kbd *pk = dev_get_drvdata(dev);

	guard(mutex)(&pk->lock);

	if (pk->connected)
		pogo_set_sleep(pk, false);

	if (pk->state != POGO_IDLE) {
		WRITE_ONCE(pk->last_rx, jiffies);
		schedule_delayed_work(&pk->poll_work, 0);
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(pogo_pm_ops, pogo_suspend, pogo_resume);

static const struct of_device_id pogo_of_match[] = {
	{ .compatible = "oneplus,erhai-pogo-keyboard" },
	{}
};
MODULE_DEVICE_TABLE(of, pogo_of_match);

static struct serdev_device_driver pogo_driver = {
	.driver = {
		.name = "oneplus-pogo-kbd",
		.of_match_table = pogo_of_match,
		.pm = pm_sleep_ptr(&pogo_pm_ops),
		.dev_groups = pogo_groups,
	},
	.probe = pogo_probe,
};
module_serdev_device_driver(pogo_driver);

MODULE_DESCRIPTION("OnePlus Pad 3 pogo-pin keyboard");
MODULE_LICENSE("GPL");
