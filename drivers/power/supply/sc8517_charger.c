// SPDX-License-Identifier: GPL-2.0-only
/*
 * Southchip SC8517 direct charger and SuperVOOC PHY.
 *
 * Register sequences and protocol adapted from the Oplus SC8517 and AP
 * VOOC PHY drivers, Copyright (C) 2018-2021 Oplus. All rights reserved.
 * The adapter regulates the current; the SC8517 connects it to a 2S pack.
 */
#include <linux/bitfield.h>
#include <linux/bitrev.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/random.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define SC8517_MOS		0x02
#define SC8517_WATCHDOG		0x04
#define SC8517_RESET		0x06
#define SC8517_STATUS		0x09
#define SC8517_VBUS_STATUS	0x0b
#define SC8517_REVISION		0x1c
#define SC8517_DPDM		0x20
#define SC8517_PHY_CTRL		0x24
#define SC8517_TX		0x25
#define SC8517_RX		0x27
#define SC8517_IRQ_MASK		0x29
#define SC8517_PREDATA		0x2a
#define SC8517_RX_DONE		BIT(1)
#define SC8517_TX_FAILED		BIT(3)

#define SVOOC_HEAD		4
#define VOOC2_HEAD		5
#define SVOOC_GET_VBAT		1
#define SVOOC_VBUS_OK		2
#define SVOOC_GET_CURRENT		3
#define SVOOC_FASTCHG		4
#define SVOOC_MODEL		5
#define SVOOC_BAD_CABLE		9
#define SVOOC_AP_STATUS		10
#define SVOOC_IDENTIFY		12
#define SVOOC_BATTERY		13

#define SC8517_POLL_MS		250
#define SC8517_IDLE_POLL_MS	2000
#define SC8517_TIMEOUT_MS		3000
#define SC8517_RETRY_MS		10000
#define SC8517_PACK_MIN_UV	6700000
#define SC8517_PACK_MAX_UV	9088000
#define SC8517_INPUT_MAX_UV	9500000
#define SC8517_POWER_MAX_UW	50000000
/* Protocol current is in 100 mA units: stay below 50 W at the input ceiling. */
#define SC8517_CURRENT_MAX	(SC8517_POWER_MAX_UW * 10 / SC8517_INPUT_MAX_UV)
#define SC8517_CURRENT_TRIP_UA	6000000

enum sc8517_exchange {
	SC8517_COMMAND,
	SC8517_IDENTIFICATION,
	SC8517_MODEL,
	SC8517_BATTERY,
};

struct sc8517 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *switch_ctrl;
	struct gpio_desc *vphy_switch;
	struct power_supply *gauge;
	struct power_supply *input;
	struct power_supply *psy;
	struct notifier_block notifier;
	struct delayed_work work;
	/* Serializes PHY responses, GPIO routing and the charging session. */
	struct mutex lock;
	enum sc8517_exchange exchange;
	unsigned long last_rx;
	unsigned long last_sample;
	unsigned long retry_at;
	u16 challenge;
	u16 response;
	u8 challenge_reply;
	u8 remaining;
	u8 adapter_model;
	u8 requested_current;
	u8 current_limit;
	u8 attempts;
	int voltage_uv;
	bool running;
	bool enabled;
	bool identified;
	bool battery_sent;
	bool current_change;
	bool input_suspended;
	bool stopping;
};

static int sc8517_read_supply(struct power_supply *psy,
			      enum power_supply_property prop, int *value)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(psy, prop, &val);
	if (!ret)
		*value = val.intval;
	return ret;
}

static int sc8517_suspend_input(struct sc8517 *sc, bool suspend)
{
	union power_supply_propval val = { .intval = suspend ? 0 : -1 };
	int ret;

	ret = power_supply_set_property(sc->input,
					POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
	if (!ret)
		sc->input_suspended = suspend;
	return ret;
}

/* The PHY sends ten bits MSB first, in two little-endian I2C registers. */
static int sc8517_send(struct sc8517 *sc, u8 head, u8 payload, bool shifted,
		       bool predata)
{
	u16 frame = (head << 7) | payload;
	u8 data[2];
	int ret;

	if (shifted)
		frame >>= 1;
	data[0] = frame >> 8;
	data[1] = frame;
	ret = regmap_bulk_write(sc->regmap, SC8517_TX, data, sizeof(data));
	if (ret || !predata)
		return ret;
	return regmap_bulk_write(sc->regmap, SC8517_PREDATA, data, sizeof(data));
}

static int sc8517_predata(struct sc8517 *sc, u16 value)
{
	u8 data[] = { value, value >> 8 };

	return regmap_bulk_write(sc->regmap, SC8517_PREDATA, data, sizeof(data));
}

/* Caller holds lock. Restore the buck input outside lock: GLINK can sleep. */
static void sc8517_stop(struct sc8517 *sc)
{
	int ret;

	if (!sc->running)
		return;

	ret = regmap_write(sc->regmap, SC8517_MOS, 0x78);
	regmap_write(sc->regmap, SC8517_PHY_CTRL, 0);
	regmap_write(sc->regmap, SC8517_IRQ_MASK, 0x7f);
	regmap_write(sc->regmap, SC8517_DPDM, 0xc0);
	/* Leave the hardware watchdog armed if opening the MOS failed. */
	if (!ret)
		regmap_write(sc->regmap, SC8517_WATCHDOG, 0x06);
	gpiod_set_value_cansleep(sc->switch_ctrl, 0);
	gpiod_set_value_cansleep(sc->vphy_switch, 0);
	sc->running = false;
	sc->enabled = false;
	sc->retry_at = jiffies + msecs_to_jiffies(SC8517_RETRY_MS);
	pm_relax(sc->dev);
	power_supply_changed(sc->psy);
}

static int sc8517_start(struct sc8517 *sc)
{
	static const struct reg_sequence setup[] = {
		{ 0x01, 0x5e }, /* V1X OVP enabled, vendor 4.725 V threshold */
		{ 0x06, 0x85 }, /* Audio mode */
		{ 0x08, 0xa6 }, /* Reference skip: 40 mV */
		{ 0x10, 0x79 }, /* Charger interrupt mask */
		{ 0x03, 0xff }, /* Forward and reverse overcurrent protection */
		{ 0x12, 0x10 }, /* Overcurrent trigger: 10 us */
		{ SC8517_MOS, 0x78 },
		{ SC8517_PHY_CTRL, 0x02 },
	};
	static const struct reg_sequence handshake[] = {
		{ SC8517_DPDM, 0x21 },
		{ 0x21, 0x80 },
		{ 0x2c, 0xd1 },
		{ SC8517_IRQ_MASK, 0x25 },
		{ SC8517_PHY_CTRL, 0x81 },
	};
	int ret;

	sc->attempts++;
	sc->running = true;
	sc->enabled = false;
	sc->identified = false;
	sc->battery_sent = false;
	sc->exchange = SC8517_COMMAND;
	sc->remaining = 0;
	sc->adapter_model = 0;
	sc->requested_current = 0;
	sc->current_change = false;
	sc->last_rx = jiffies;
	pm_stay_awake(sc->dev);

	ret = regmap_update_bits(sc->regmap, SC8517_RESET, BIT(3), BIT(3));
	if (ret)
		return ret;
	usleep_range(10000, 11000);
	/* V2X overvoltage protection enabled at its lowest threshold, 9.5 V. */
	ret = regmap_update_bits(sc->regmap, 0x00, GENMASK(7, 5), 0);
	if (ret)
		return ret;
	ret = regmap_multi_reg_write(sc->regmap, setup, ARRAY_SIZE(setup));
	if (ret)
		return ret;
	ret = sc8517_predata(sc, 0);
	if (ret)
		return ret;
	gpiod_set_value_cansleep(sc->switch_ctrl, 1);
	gpiod_set_value_cansleep(sc->vphy_switch, 1);
	msleep(50);
	ret = regmap_multi_reg_write(sc->regmap, handshake, ARRAY_SIZE(handshake));
	if (!ret)
		dev_info(sc->dev, "SuperVOOC handshake started\n");
	return ret;
}

/* Oplus identification permutes two seven-bit random values over two frames. */
static u8 sc8517_identify(struct sc8517 *sc)
{
	u8 lo = get_random_u8() & 0x7f;
	u8 hi = get_random_u8() & 0x7f;
	u8 first, second;

	sc->challenge = (hi << 9) | (lo << 1);
	sc->response = 0;
	first = ((lo >> 6) & 1) | (((lo >> 4) & 1) << 1) |
		(((hi >> 6) & 1) << 2) | ((lo & 1) << 3) |
		((hi & 1) << 4) | (((hi >> 4) & 1) << 5) |
		(((lo >> 2) & 1) << 6);
	second = ((lo >> 3) & 1) | (((hi >> 5) & 1) << 1) |
		(((lo >> 5) & 1) << 2) | (((hi >> 1) & 1) << 3) |
		(((hi >> 2) & 1) << 4) | (((lo >> 1) & 1) << 5) |
		(((hi >> 3) & 1) << 6);
	sc->challenge_reply = bitrev8(second) >> 1;
	sc->exchange = SC8517_IDENTIFICATION;
	sc->remaining = 5;
	return bitrev8(first) >> 1;
}

static int sc8517_voltage_protected(struct sc8517 *sc)
{
	unsigned int pack, cell;
	int ret;

	ret = regmap_read(sc->regmap, 0x00, &pack);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, 0x01, &cell);
	if (ret)
		return ret;
	/* Never connect with either OVP disabled or a raised pack threshold. */
	if ((pack & GENMASK(7, 5)) || (cell & BIT(7)) ||
	    (cell & GENMASK(6, 2)) > (0x5e & GENMASK(6, 2)))
		return -ERANGE;
	return 0;
}

static int sc8517_reply(struct sc8517 *sc, u8 rx)
{
	u8 head = rx >> 5, cmd, reply = 0;
	unsigned int vbus;
	bool shifted = false, predata = false, battery_done = false;
	int ret;

	if (sc->voltage_uv < SC8517_PACK_MIN_UV ||
	    sc->voltage_uv >= SC8517_PACK_MAX_UV ||
	    time_after(jiffies, sc->last_sample + msecs_to_jiffies(SC8517_TIMEOUT_MS)))
		return -ERANGE;

	/* A VOOC2 discovery packet precedes SuperVOOC adapter identification. */
	if (head != SVOOC_HEAD && head != VOOC2_HEAD) {
		head = (rx >> 4) & 7;
		shifted = true;
	}
	if (head != SVOOC_HEAD && head != VOOC2_HEAD)
		return -EPROTO;
	cmd = shifted ? rx & 0x0f : (rx >> 1) & 0x0f;
	if (head == VOOC2_HEAD) {
		if (cmd != SVOOC_FASTCHG)
			return -EPROTO;
		return sc8517_send(sc, SVOOC_HEAD, 0x10, shifted, false);
	}

	if (sc->exchange == SC8517_IDENTIFICATION) {
		if (sc->remaining == 5) {
			reply = sc->challenge_reply;
		} else {
			sc->response = (sc->response << 4) | cmd;
			reply = 0x10;
		}
		if (!--sc->remaining) {
			if ((sc->response & 0xfefe) != sc->challenge)
				return -EPROTO;
			sc->identified = true;
			sc->exchange = SC8517_COMMAND;
		}
	} else if (sc->exchange == SC8517_MODEL) {
		sc->adapter_model = (sc->adapter_model << 4) | cmd;
		reply = 0x10;
		if (!--sc->remaining) {
			sc->adapter_model &= 0x7f;
			/* Higher current is qualified only for Aston's tested adapter. */
			if (sc->adapter_model != 0x4b) {
				sc->current_limit = min_t(u8, sc->current_limit, 30);
				sc->current_change = true;
			}
			sc->exchange = SC8517_COMMAND;
			dev_info(sc->dev, "SuperVOOC adapter 0x%02x\n", sc->adapter_model);
		}
	} else if (sc->exchange == SC8517_BATTERY) {
		/* Three current/voltage pairs, in the vendor transmission order. */
		if (cmd != (6 - sc->remaining) / 2 + 1)
			return -EPROTO;
		switch (sc->remaining) {
		case 1:
			reply = sc->current_limit;
			break;
		case 3:
			reply = min_t(u8, sc->current_limit, 20);
			break;
		case 5:
			reply = min_t(u8, sc->current_limit, 12);
			break;
		case 2:
			reply = (4324 - 3404) / 10;
			break;
		default:
			reply = (4544 - 3404) / 10;
			break;
		}
		if (!--sc->remaining) {
			sc->exchange = SC8517_COMMAND;
			sc->battery_sent = true;
			battery_done = true;
		}
	} else {
		switch (cmd) {
		case SVOOC_FASTCHG:
			/* Bidirectional SC8517 circuit resistance and permission bit. */
			reply = bitrev8(0x04 | sc->battery_sent) >> 1;
			break;
		case SVOOC_IDENTIFY:
			reply = sc8517_identify(sc);
			break;
		case SVOOC_MODEL:
			sc->exchange = SC8517_MODEL;
			sc->remaining = 2;
			sc->adapter_model = 0;
			reply = 0x28;
			break;
		case SVOOC_BATTERY:
			if (!sc->identified || !sc->adapter_model)
				return -EPROTO;
			sc->exchange = SC8517_BATTERY;
			sc->remaining = 6;
			reply = bitrev8(0x0c) >> 1;
			break;
		case SVOOC_VBUS_OK:
			if (!sc->battery_sent)
				return -EPROTO;
			ret = regmap_read(sc->regmap, SC8517_VBUS_STATUS, &vbus);
			if (ret)
				return ret;
			vbus = FIELD_GET(GENMASK(7, 6), vbus);
			/* Vendor encoding: low=bit5, high=bit4, normal=both. */
			reply = vbus == 1 ? 0x10 : vbus ? 0x20 : 0x30;
			if (!vbus && !sc->enabled) {
				ret = sc8517_voltage_protected(sc);
				if (ret)
					return ret;
				/* Arm the watchdog before connecting the battery. */
				ret = regmap_write(sc->regmap, SC8517_WATCHDOG, 0x36);
				if (ret)
					return ret;
				ret = regmap_write(sc->regmap, SC8517_MOS, 0x7a);
				if (ret)
					return ret;
				ret = regmap_write(sc->regmap, SC8517_MOS, 0x7b);
				if (ret)
					return ret;
				sc->enabled = true;
				power_supply_changed(sc->psy);
				dev_info(sc->dev, "SuperVOOC direct charging enabled\n");
			}
			break;
		case SVOOC_GET_CURRENT:
			if (!sc->enabled)
				return -EPROTO;
			reply = sc->current_limit;
			sc->requested_current = reply;
			sc->current_change = false;
			break;
		case SVOOC_GET_VBAT:
			if (!sc->enabled)
				return -EPROTO;
			/* 0x7f asks the adapter for a new current command. */
			if (sc->current_change)
				reply = 0x7f;
			else
				reply = clamp((sc->voltage_uv / 2000 - 3404) / 10, 0, 125);
			predata = true;
			break;
		case SVOOC_BAD_CABLE:
			return -ENOLINK;
		case SVOOC_AP_STATUS:
			break;
		default:
			return -EPROTO;
		}
	}

	ret = sc8517_send(sc, SVOOC_HEAD, reply, shifted, predata);
	if (!ret && battery_done)
		ret = sc8517_predata(sc, 0x2002);
	return ret;
}

static irqreturn_t sc8517_irq(int irq, void *data)
{
	struct sc8517 *sc = data;
	u8 rx[2] = {}, status[6];
	int ret;

	mutex_lock(&sc->lock);
	ret = regmap_bulk_read(sc->regmap, SC8517_RX, rx, sizeof(rx));
	if (!sc->running)
		goto out;
	if (ret)
		goto stop;
	if (rx[1] & SC8517_RX_DONE) {
		sc->last_rx = jiffies;
		ret = sc8517_reply(sc, rx[0]);
		dev_dbg(sc->dev, "rx=%02x flags=%02x ret=%d\n", rx[0], rx[1], ret);
		if (ret)
			goto stop;
	}
	ret = regmap_bulk_read(sc->regmap, SC8517_STATUS, status, sizeof(status));
	if (ret)
		goto stop;
	/* Latched overvoltage, overcurrent, thermal and watchdog faults. */
	if ((status[3] & 0xf9) || (status[5] & 0xd3) ||
	    (sc->enabled && !(status[1] & BIT(0)))) {
		ret = -EIO;
		goto stop;
	}
	if (sc->enabled) {
		ret = regmap_write(sc->regmap, SC8517_WATCHDOG, 0x36);
		if (ret)
			goto stop;
	}
	goto out;
stop:
	dev_warn_ratelimited(sc->dev, "SuperVOOC stopped: %d (rx=%02x flags=%02x)\n",
			     ret, rx[0], rx[1]);
	sc8517_stop(sc);
	if (!sc->stopping)
		mod_delayed_work(system_percpu_wq, &sc->work, 0);
out:
	mutex_unlock(&sc->lock);
	return IRQ_HANDLED;
}

static bool sc8517_battery_safe(int voltage, int current_ua, int temp, int soc,
				int health, int input_uv)
{
	return health == POWER_SUPPLY_HEALTH_GOOD &&
		voltage >= SC8517_PACK_MIN_UV && voltage < SC8517_PACK_MAX_UV &&
		input_uv > 0 && input_uv < SC8517_INPUT_MAX_UV &&
		temp >= 120 && temp < 430 && soc < 90 &&
		current_ua > -SC8517_CURRENT_TRIP_UA && current_ua < SC8517_CURRENT_TRIP_UA;
}

static u8 sc8517_current_limit(int voltage, int temp, int soc, u8 previous)
{
	u8 limit = SC8517_CURRENT_MAX;

	/* Conservative subset of the vendor 2S curve, in 100 mA units. */
	if (temp >= 350)
		limit = 30;
	if (temp < 200 || temp >= 400 || soc >= 75 || voltage >= 8648000)
		limit = 20;
	if (soc >= 85 || voltage >= 9000000)
		limit = 12;
	/* Keep the taper when the lower current removes the battery's IR rise. */
	if (previous)
		limit = min_t(u8, limit, previous);
	return limit;
}

static bool sc8517_can_negotiate(int type)
{
	return type == POWER_SUPPLY_USB_TYPE_DCP ||
		type == POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID;
}

static void sc8517_monitor(struct work_struct *work)
{
	struct sc8517 *sc = container_of(to_delayed_work(work), struct sc8517, work);
	int online = 0, type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	int voltage = 0, current_ua = 0, temp = 0, soc = 100, health = 0, input_uv = 0;
	unsigned long sampled = jiffies;
	bool safe, start = false;
	u8 limit;
	int ret;

	/* These can wait for GLINK; never hold the PHY response lock here. */
	ret = sc8517_read_supply(sc->input, POWER_SUPPLY_PROP_ONLINE, &online);
	ret |= sc8517_read_supply(sc->input, POWER_SUPPLY_PROP_USB_TYPE, &type);
	ret |= sc8517_read_supply(sc->input, POWER_SUPPLY_PROP_VOLTAGE_NOW, &input_uv);
	ret |= sc8517_read_supply(sc->gauge, POWER_SUPPLY_PROP_VOLTAGE_NOW, &voltage);
	ret |= sc8517_read_supply(sc->gauge, POWER_SUPPLY_PROP_CURRENT_NOW, &current_ua);
	ret |= sc8517_read_supply(sc->gauge, POWER_SUPPLY_PROP_TEMP, &temp);
	ret |= sc8517_read_supply(sc->gauge, POWER_SUPPLY_PROP_CAPACITY, &soc);
	ret |= sc8517_read_supply(sc->gauge, POWER_SUPPLY_PROP_HEALTH, &health);
	safe = !ret && sc8517_battery_safe(voltage, current_ua, temp, soc, health, input_uv);
	mutex_lock(&sc->lock);
	limit = sc8517_current_limit(voltage, temp, soc, sc->running ? sc->current_limit : 0);
	sc->voltage_uv = voltage;
	if (safe)
		sc->last_sample = sampled;
	if (sc->current_limit != limit) {
		sc->current_limit = limit;
		sc->current_change = true;
	}
	if (!online) {
		sc8517_stop(sc);
		sc->attempts = 0;
		sc->retry_at = 0;
	} else if (!safe || sc->stopping) {
		sc8517_stop(sc);
	} else if (sc->running) {
		if (time_after(jiffies, sc->last_rx + msecs_to_jiffies(SC8517_TIMEOUT_MS))) {
			dev_warn(sc->dev, "SuperVOOC communication timeout\n");
			sc8517_stop(sc);
		} else if (sc->enabled &&
			   regmap_write(sc->regmap, SC8517_WATCHDOG, 0x36)) {
			sc8517_stop(sc);
		}
	} else if (sc8517_can_negotiate(type) && sc->attempts < 3 &&
		   (!sc->retry_at || time_after_eq(jiffies, sc->retry_at))) {
		start = true;
	}
	mutex_unlock(&sc->lock);

	if (start) {
		ret = sc8517_suspend_input(sc, true);
		if (!ret) {
			mutex_lock(&sc->lock);
			ret = sc8517_start(sc);
			if (ret)
				sc8517_stop(sc);
			mutex_unlock(&sc->lock);
		} else {
			mutex_lock(&sc->lock);
			sc->attempts++;
			sc->retry_at = jiffies + msecs_to_jiffies(SC8517_RETRY_MS);
			mutex_unlock(&sc->lock);
		}
		if (ret)
			dev_err_ratelimited(sc->dev, "Cannot start SuperVOOC: %d\n", ret);
	}
	if (!READ_ONCE(sc->running) && sc->input_suspended) {
		ret = sc8517_suspend_input(sc, false);
		if (ret)
			dev_err_ratelimited(sc->dev, "Cannot restore charger input: %d\n", ret);
	}
	if (!sc->stopping) {
		unsigned int poll_ms = READ_ONCE(sc->running) ?
					SC8517_POLL_MS : SC8517_IDLE_POLL_MS;

		schedule_delayed_work(&sc->work, msecs_to_jiffies(poll_ms));
	}
}

static int sc8517_supply_changed(struct notifier_block *nb, unsigned long event,
				 void *data)
{
	struct sc8517 *sc = container_of(nb, struct sc8517, notifier);

	if (event == PSY_EVENT_PROP_CHANGED && data == sc->input &&
	    !READ_ONCE(sc->stopping))
		mod_delayed_work(system_percpu_wq, &sc->work, 0);
	return NOTIFY_OK;
}

static enum power_supply_property sc8517_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int sc8517_get_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       union power_supply_propval *val)
{
	struct sc8517 *sc = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&sc->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sc->enabled;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = sc->enabled ? POWER_SUPPLY_STATUS_CHARGING :
			POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = sc->enabled ? POWER_SUPPLY_CHARGE_TYPE_FAST :
			POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = sc->enabled ? sc->requested_current * 100000 : 0;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SC8517 SuperVOOC";
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&sc->lock);
	return ret;
}

static const struct power_supply_desc sc8517_psy_desc = {
	.name = "sc8517-charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = sc8517_props,
	.num_properties = ARRAY_SIZE(sc8517_props),
	.get_property = sc8517_get_property,
};

static void sc8517_put_input(void *data)
{
	power_supply_put(data);
}

static void sc8517_cleanup(void *data)
{
	struct sc8517 *sc = data;

	if (xchg(&sc->stopping, true))
		return;
	power_supply_unreg_notifier(&sc->notifier);
	disable_irq(to_i2c_client(sc->dev)->irq);
	cancel_delayed_work_sync(&sc->work);
	mutex_lock(&sc->lock);
	sc8517_stop(sc);
	mutex_unlock(&sc->lock);
	if (sc->input_suspended)
		sc8517_suspend_input(sc, false);
	device_init_wakeup(sc->dev, false);
}

static int sc8517_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config cfg = {};
	struct sc8517 *sc;
	unsigned int rev;
	int ret;
	static const struct regmap_config regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0x2d,
	};

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "Missing PHY interrupt\n");
	sc = devm_kzalloc(dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;
	sc->dev = dev;
	mutex_init(&sc->lock);
	INIT_DELAYED_WORK(&sc->work, sc8517_monitor);
	i2c_set_clientdata(client, sc);
	sc->gauge = devm_power_supply_get_by_reference(dev, "fuel-gauge");
	if (IS_ERR(sc->gauge))
		return dev_err_probe(dev, PTR_ERR(sc->gauge), "Cannot get fuel gauge\n");
	if (!sc->gauge)
		return -EPROBE_DEFER;
	/* The Qualcomm USB endpoint owns cable detection and the parallel buck. */
	sc->input = power_supply_get_by_name("qcom-battmgr-usb");
	if (!sc->input)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, sc8517_put_input, sc->input);
	if (ret)
		return ret;
	sc->switch_ctrl = devm_gpiod_get(dev, "switch-ctrl", GPIOD_OUT_LOW);
	if (IS_ERR(sc->switch_ctrl))
		return dev_err_probe(dev, PTR_ERR(sc->switch_ctrl), "Cannot get path switch\n");
	sc->vphy_switch = devm_gpiod_get(dev, "vphy-switch", GPIOD_OUT_LOW);
	if (IS_ERR(sc->vphy_switch))
		return dev_err_probe(dev, PTR_ERR(sc->vphy_switch), "Cannot get PHY switch\n");
	sc->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(sc->regmap))
		return PTR_ERR(sc->regmap);
	ret = regmap_read(sc->regmap, SC8517_REVISION, &rev);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot read SC8517 revision\n");
	cfg.drv_data = sc;
	cfg.fwnode = dev_fwnode(dev);
	sc->psy = devm_power_supply_register(dev, &sc8517_psy_desc, &cfg);
	if (IS_ERR(sc->psy))
		return dev_err_probe(dev, PTR_ERR(sc->psy), "Cannot register charger\n");
	ret = devm_request_threaded_irq(dev, client->irq, NULL, sc8517_irq,
					IRQF_ONESHOT, dev_name(dev), sc);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot request PHY interrupt\n");
	sc->notifier.notifier_call = sc8517_supply_changed;
	ret = power_supply_reg_notifier(&sc->notifier);
	if (ret)
		return ret;
	device_init_wakeup(dev, true);
	ret = devm_add_action_or_reset(dev, sc8517_cleanup, sc);
	if (ret)
		return ret;
	dev_info(dev, "SC8517 revision %02x, kernel SuperVOOC policy ready\n", rev);
	schedule_delayed_work(&sc->work, 0);
	return 0;
}

static void sc8517_shutdown(struct i2c_client *client)
{
	sc8517_cleanup(i2c_get_clientdata(client));
}

static const struct of_device_id sc8517_of_match[] = {
	{ .compatible = "southchip,sc8517" },
	{}
};
MODULE_DEVICE_TABLE(of, sc8517_of_match);

static struct i2c_driver sc8517_driver = {
	.driver = {
		.name = "sc8517-charger",
		.of_match_table = sc8517_of_match,
	},
	.probe = sc8517_probe,
	.shutdown = sc8517_shutdown,
};
module_i2c_driver(sc8517_driver);

MODULE_DESCRIPTION("Southchip SC8517 SuperVOOC direct charger");
MODULE_LICENSE("GPL");
