// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 fuel gauge (read only) and charger status.
 *
 * The gauge runs on the battery and keeps the battery model the vendor
 * firmware loaded into it across reboots. This driver only reads it: the
 * state of charge, voltage, current and temperature words of its SRAM, and
 * the status of the SM5714 charger block next to it. The one write is the
 * SRAM read address. No battery table or reset is ever written.
 *
 * Battery temperature comes from the battery's thermistor on a PMIC ADC
 * (io-channels), as Samsung's software reads it; the gauge's own temperature
 * word follows the chip, about 10 C above the cells on stock.
 *
 * The charger settings: on a wall charger (BC1.2 DCP or CDP, as the chip's
 * MUIC detects it) inside 10-45 C, the input limit goes from the power-on
 * 500 mA to 1500 mA and the charge current from 500 mA to 1000 mA. Anything
 * else, a computer port included, keeps the power-on 500/500 mA. At or below
 * 0 C and at or above 50 C, charging stops (ENQ4FET off) until the battery is
 * back inside 2-48 C, as Samsung's sec_battery does on this tablet. Float
 * voltage, termination, watchdog and every other register keep their
 * power-on values.
 *
 * Plug events: with an interrupt in the device tree (the chip's shared
 * active-low line), every charger and MUIC source but VBUSPOK (plug-in) and
 * VBUSUVLO (unplug) is masked and
 * the flags are cleared at probe; a plug then re-runs the status check at
 * once, and the timer only follows the temperature window. Without one, or
 * when the line storms, the status is polled every 2 s.
 *
 * Register map and conversions as in Samsung's sm5714_fuelgauge.c and
 * sm5714_charger.c (Galaxy Tab S7 FE kernel source).
 */

#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>

#define SM5714_FG_SYSTEM_STATUS		0x10
#define SM5714_FG_INIT_DONE		BIT(12)	/* clear: the gauge lost its battery model */
/* Samsung reports 0 current then (jig, nENQ4 low) */
#define SM5714_FG_NO_BATT_PATH		BIT(15)
#define SM5714_FG_AUX_STAT		0x22
#define SM5714_FG_AUX_X2_5		BIT(10)	/* current reads are x2.5 */
#define SM5714_FG_SRAM_RADDR		0x8c
#define SM5714_FG_SRAM_RDATA		0x8d

#define SM5714_FG_SRAM_SOC		0x00
#define SM5714_FG_SRAM_VBAT		0x03
#define SM5714_FG_SRAM_CURRENT		0x05
#define SM5714_FG_SRAM_CURRENT_AVG	0x09

#define SM5714_CHG_ADDR			0x49
#define SM5714_CHG_STATUS1		0x0d
#define SM5714_CHG_VBUS_POK		BIT(0)
#define SM5714_CHG_VBUS_OVP		BIT(2)
#define SM5714_CHG_STATUS2		0x0e
#define SM5714_CHG_BAT_ABSENT		BIT(2)
#define SM5714_CHG_CHARGING		BIT(3)
#define SM5714_CHG_TOPOFF		BIT(5)
#define SM5714_CHG_CNTL1		0x13
#define SM5714_CHG_ENQ4FET		BIT(3)	/* battery charging path on */
#define SM5714_CHG_VBUSCNTL		0x15	/* [6:0] input limit, 100 mA + 25 mA/step */
#define SM5714_CHG_VBUS_LIMIT		GENMASK(6, 0)
#define SM5714_CHG_CHGCNTL2		0x18	/* fast charge, 109.375 mA + 15.625 mA/(step - 7) */
#define SM5714_CHG_IN_SLOW		0x10	/* 500 mA, the power-on value */
#define SM5714_CHG_IN_FAST		0x38	/* 1500 mA */
#define SM5714_CHG_CUR_SLOW		0x20	/* 500 mA, the power-on value */
#define SM5714_CHG_CUR_FAST		0x40	/* 1000 mA */

#define SM5714_MUIC_ADDR		0x25
#define SM5714_MUIC_DEVICETYPE1		0x07
#define SM5714_MUIC_DCP			BIT(2)
#define SM5714_MUIC_CDP			BIT(3)

/* Battery temperature window for the faster rate, in 0.1 C. */
#define SM5714_FAST_TEMP_MIN		100
#define SM5714_FAST_TEMP_MAX		450
#define SM5714_FAST_TEMP_RESUME		400
/*
 * Charging off at or below COLD and at or above HOT, back on HYST inside: Samsung's
 * wire_cold_cool3_thresh and wire_warm_overheat_thresh on this tablet, THERMAL_HYSTERESIS_2.
 */
#define SM5714_CHG_COLD			0
#define SM5714_CHG_HOT			500
#define SM5714_CHG_HYST			19

#define SM5714_CHG_POLL			msecs_to_jiffies(2000)
#define SM5714_CHG_POLL_IRQ		msecs_to_jiffies(60000)	/* temperature window only */

/*
 * Interrupts (Samsung sm5714-irq.c): one source register, flags that clear on read, a mask per flag
 * register.
 */
#define SM5714_CHG_INT_SOURCE		0x00
#define SM5714_IRQSRC_MUIC		BIT(0)
#define SM5714_IRQSRC_CHG		BIT(1)
#define SM5714_IRQSRC_FG		BIT(2)
#define SM5714_CHG_INT1			0x01	/* INT1..INT5 at 0x01..0x05 */
#define SM5714_CHG_INTMSK1		0x07	/* INTMSK1..INTMSK5 at 0x07..0x0b */
#define SM5714_CHG_INT1_VBUSPOK		BIT(0)	/* plug-in */
#define SM5714_CHG_INT1_VBUSUVLO	BIT(1)	/* unplug */
#define SM5714_CHG_INT1_PLUG		(SM5714_CHG_INT1_VBUSPOK | SM5714_CHG_INT1_VBUSUVLO)
#define SM5714_MUIC_INT1		0x01	/* INT1, INT2 */
#define SM5714_MUIC_INTMASK1		0x03	/* INTMASK1, INTMASK2 */
/* the gauge's flags: never armed here, only cleared */
#define SM5714_FG_INTFG			0x02
/* interrupts in a second before falling back to polling */
#define SM5714_IRQ_STORM		20

struct sm5714_fg {
	struct i2c_client *client;
	struct i2c_client *chg;
	struct i2c_client *muic;
	struct mutex lock;	/* the SRAM address and data reads are one transaction */
	struct power_supply *bat;
	struct power_supply *usb;
	struct iio_channel *therm;
	struct delayed_work poll;
	unsigned long poll_period;
	unsigned long storm_start;
	unsigned int storm_count;
	int online;
	unsigned long plugged;	/* jiffies of the last plug-in */
	bool fast;
	bool chg_off;	/* ENQ4FET off: battery outside the charging window */
	bool broken;	/* a read-back did not match: stay at the power-on values */
};

static int sm5714_fg_sram(struct sm5714_fg *fg, u8 addr)
{
	int ret;

	mutex_lock(&fg->lock);
	ret = i2c_smbus_write_word_data(fg->client, SM5714_FG_SRAM_RADDR, addr);
	if (!ret)
		ret = i2c_smbus_read_word_data(fg->client, SM5714_FG_SRAM_RDATA);
	mutex_unlock(&fg->lock);

	return ret;
}

static int sm5714_fg_signed(int raw, int mag);

/* Battery temperature from its thermistor, in 0.1 C. */
static int sm5714_bat_temp(struct sm5714_fg *fg, int *val)
{
	int mc, ret = iio_read_channel_processed(fg->therm, &mc);

	if (ret < 0)
		return ret;
	*val = mc / 100;
	return 0;
}

/* Sign and magnitude: bit 15 is the sign. */
static int sm5714_fg_signed(int raw, int mag)
{
	return raw & 0x8000 ? -mag : mag;
}

static int sm5714_fg_current_ua(struct sm5714_fg *fg, u8 addr, int *val)
{
	int raw = sm5714_fg_sram(fg, addr);
	int aux = i2c_smbus_read_word_data(fg->client, SM5714_FG_AUX_STAT);
	int sys = i2c_smbus_read_word_data(fg->client, SM5714_FG_SYSTEM_STATUS);
	int ma;

	if (raw < 0)
		return raw;
	if (aux < 0)
		return aux;
	if (sys < 0)
		return sys;
	/* mA = (raw & 0x7fff) * 1000 / 2044; bit 15 set = discharging */
	ma = sm5714_fg_signed(raw, (raw & 0x7fff) * 1000 / 2044);
	if (aux & SM5714_FG_AUX_X2_5)
		ma = ma * 25 / 10;
	if (sys & SM5714_FG_NO_BATT_PATH)
		ma = 0;
	*val = ma * 1000;
	return 0;
}

static int sm5714_fg_status(struct sm5714_fg *fg, int *val)
{
	int st1 = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS1);
	int st2 = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS2);

	if (st1 < 0)
		return st1;
	if (st2 < 0)
		return st2;

	if (st2 & SM5714_CHG_TOPOFF)
		*val = POWER_SUPPLY_STATUS_FULL;
	else if (st2 & SM5714_CHG_CHARGING)
		*val = POWER_SUPPLY_STATUS_CHARGING;
	else if (st1 & SM5714_CHG_VBUS_POK)
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
	return 0;
}

/*
 * Reported only, never written: stock DT battery,battery_full_capacity = <0x276a>.
 */
#define SM5714_BATTERY_FULL_UAH	10090000

static int sm5714_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5714_fg *fg = power_supply_get_drvdata(psy);
	int raw, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return sm5714_fg_status(fg, &val->intval);
	case POWER_SUPPLY_PROP_PRESENT:
		raw = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS2);
		if (raw < 0)
			return raw;
		val->intval = !(raw & SM5714_CHG_BAT_ABSENT);
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		raw = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS1);
		if (raw < 0)
			return raw;
		val->intval = raw & SM5714_CHG_VBUS_OVP ? POWER_SUPPLY_HEALTH_OVERVOLTAGE
						       : POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		raw = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS2);
		if (raw < 0)
			return raw;
		val->intval = raw & SM5714_CHG_CHARGING ? POWER_SUPPLY_CHARGE_TYPE_FAST
							: POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = SM5714_BATTERY_FULL_UAH;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = sm5714_fg_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, val);
		if (ret)
			return ret;
		val->intval = SM5714_BATTERY_FULL_UAH / 100 * val->intval;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = i2c_smbus_read_word_data(fg->client, SM5714_FG_SYSTEM_STATUS);
		if (ret < 0)
			return ret;
		if (!(ret & SM5714_FG_INIT_DONE))
			/* no battery model: the number would be meaningless */
			return -ENODATA;
		raw = sm5714_fg_sram(fg, SM5714_FG_SRAM_SOC);
		if (raw < 0)
			return raw;
		/* (raw * 10) >> 8 is in 0.1 % */
		val->intval = clamp(((raw * 10) >> 8) / 10, 0, 100);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		raw = sm5714_fg_sram(fg, SM5714_FG_SRAM_VBAT);
		if (raw < 0)
			return raw;
		/* mV = 2700 +/- (raw & 0x7fff) * 10 / 109 */
		val->intval = (2700 + sm5714_fg_signed(raw, (raw & 0x7fff) * 10 / 109)) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return sm5714_fg_current_ua(fg, SM5714_FG_SRAM_CURRENT, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		return sm5714_fg_current_ua(fg, SM5714_FG_SRAM_CURRENT_AVG, &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		return sm5714_bat_temp(fg, &val->intval);
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property sm5714_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc sm5714_fg_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sm5714_fg_props,
	.num_properties = ARRAY_SIZE(sm5714_fg_props),
	.get_property = sm5714_fg_get_property,
};

/* The charger input: online while VBUS is good (STATUS1 VBUS_POK). */
static int sm5714_usb_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sm5714_fg *fg = power_supply_get_drvdata(psy);
	int raw;

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;
	raw = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS1);
	if (raw < 0)
		return raw;
	val->intval = !!(raw & SM5714_CHG_VBUS_POK);
	return 0;
}

static const enum power_supply_property sm5714_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sm5714_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm5714_usb_props,
	.num_properties = ARRAY_SIZE(sm5714_usb_props),
	.get_property = sm5714_usb_get_property,
};

/*
 * Writes the two current settings and reads them back. Only these two registers are ever written.
 */
static int sm5714_chg_set_rate(struct sm5714_fg *fg, bool fast)
{
	u8 in = fast ? SM5714_CHG_IN_FAST : SM5714_CHG_IN_SLOW;
	u8 cur = fast ? SM5714_CHG_CUR_FAST : SM5714_CHG_CUR_SLOW;
	int vbus, ret;

	vbus = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_VBUSCNTL);
	if (vbus < 0)
		return vbus;
	ret = i2c_smbus_write_byte_data(fg->chg, SM5714_CHG_VBUSCNTL,
					(vbus & ~SM5714_CHG_VBUS_LIMIT) | in);
	if (!ret)
		ret = i2c_smbus_write_byte_data(fg->chg, SM5714_CHG_CHGCNTL2, cur);
	if (ret)
		return ret;

	vbus = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_VBUSCNTL);
	ret = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_CHGCNTL2);
	if (vbus < 0 || ret < 0)
		return vbus < 0 ? vbus : ret;
	if ((vbus & SM5714_CHG_VBUS_LIMIT) != in || ret != cur)
		return -EIO;
	return 0;
}

static void sm5714_chg_slow(void *data)
{
	sm5714_chg_set_rate(data, false);
}

/*
 * The faster rate needs a wall charger and a battery inside the window (with hysteresis on the hot
 * side).
 */
static bool sm5714_chg_want_fast(struct sm5714_fg *fg)
{
	int dev1, temp;

	if (!fg->online || fg->broken || fg->chg_off)
		return false;
	dev1 = i2c_smbus_read_byte_data(fg->muic, SM5714_MUIC_DEVICETYPE1);
	if (dev1 < 0 || !(dev1 & (SM5714_MUIC_DCP | SM5714_MUIC_CDP)))
		return false;
	if (sm5714_bat_temp(fg, &temp))
		return false;
	return temp >= SM5714_FAST_TEMP_MIN &&
	       temp < (fg->fast ? SM5714_FAST_TEMP_MAX : SM5714_FAST_TEMP_RESUME);
}

/* Turns the battery charging path on or off and reads it back. */
static int sm5714_chg_enable(struct sm5714_fg *fg, bool on)
{
	int val = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_CNTL1);

	if (val < 0)
		return val;
	val = on ? val | SM5714_CHG_ENQ4FET : val & ~SM5714_CHG_ENQ4FET;
	val = i2c_smbus_write_byte_data(fg->chg, SM5714_CHG_CNTL1, val);
	if (!val)
		val = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_CNTL1);
	if (val < 0)
		return val;
	return !!(val & SM5714_CHG_ENQ4FET) == on ? 0 : -EIO;
}

/* Charging off outside the battery's window; an unreadable temperature keeps the last state. */
static void sm5714_chg_check_temp(struct sm5714_fg *fg)
{
	int temp, hyst = fg->chg_off ? SM5714_CHG_HYST : 0;
	bool off;

	if (sm5714_bat_temp(fg, &temp)) {
		dev_warn_ratelimited(&fg->client->dev, "battery temperature unreadable\n");
		return;
	}
	off = temp <= SM5714_CHG_COLD + hyst || temp >= SM5714_CHG_HOT - hyst;
	if (off == fg->chg_off)
		return;
	/* back on at the 500 mA input limit the poll set while off, as Samsung's ramp needs */
	if (sm5714_chg_enable(fg, !off)) {
		dev_err(&fg->client->dev, "charging %s not confirmed\n", off ? "off" : "on");
		return;
	}
	dev_info(&fg->client->dev, "battery at %d.%d C: charging %s\n", temp / 10,
		 abs(temp % 10), off ? "off" : "on again");
	fg->chg_off = off;
	power_supply_changed(fg->bat);
}

static void sm5714_fg_poll(struct work_struct *work)
{
	struct sm5714_fg *fg = container_of(work, struct sm5714_fg, poll.work);
	int raw = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS1);
	bool fast;

	if (raw >= 0 && !!(raw & SM5714_CHG_VBUS_POK) != fg->online) {
		fg->online = !!(raw & SM5714_CHG_VBUS_POK);
		fg->plugged = jiffies;
		power_supply_changed(fg->usb);
		power_supply_changed(fg->bat);
	}

	sm5714_chg_check_temp(fg);
	fast = sm5714_chg_want_fast(fg);
	if (fast != fg->fast) {
		if (sm5714_chg_set_rate(fg, fast)) {
			dev_err(&fg->client->dev,
				"charge rate write not confirmed: back to 500 mA, no further writes\n");
			fg->broken = true;
			sm5714_chg_set_rate(fg, false);
			fg->fast = false;
		} else {
			dev_info(&fg->client->dev, "charge rate: %s\n",
				 fast ? "wall charger, 1500 mA in, 1000 mA charge"
				      : "500 mA in, 500 mA charge");
			fg->fast = fast;
		}
	}
	/*
	 * Freezable: the poll stops before the I2C controller suspends (s2idle timed out on
	 * 984000.i2c otherwise)
	 */
	/* The MUIC's charger detection ends after VBUSPOK: look again until it has settled */
	queue_delayed_work(system_freezable_wq, &fg->poll,
			   fg->online && !fg->fast && !fg->broken &&
			   time_before(jiffies, fg->plugged + 10 * HZ) ?
			   SM5714_CHG_POLL : fg->poll_period);
}

/* Reads every flag register the source names; reading clears them and releases the line. */
static void sm5714_irq_clear(struct sm5714_fg *fg, int src)
{
	u8 buf[5];

	if (src & SM5714_IRQSRC_CHG)
		i2c_smbus_read_i2c_block_data(fg->chg, SM5714_CHG_INT1, 5, buf);
	if (src & SM5714_IRQSRC_MUIC)
		i2c_smbus_read_i2c_block_data(fg->muic, SM5714_MUIC_INT1, 2, buf);
	if (src & SM5714_IRQSRC_FG)
		i2c_smbus_read_word_data(fg->client, SM5714_FG_INTFG);
}

static irqreturn_t sm5714_irq(int irq, void *data)
{
	struct sm5714_fg *fg = data;
	int src = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_INT_SOURCE);

	if (src < 0)
		return IRQ_NONE;
	sm5714_irq_clear(fg, src);

	/* A line that will not let go would run this thread forever: poll instead. */
	if (time_after(jiffies, fg->storm_start + HZ)) {
		fg->storm_start = jiffies;
		fg->storm_count = 0;
	}
	if (++fg->storm_count > SM5714_IRQ_STORM) {
		dev_warn(&fg->client->dev,
			 "interrupt storm (source %#x): polling every 2 s instead\n", src);
		disable_irq_nosync(irq);
		fg->poll_period = SM5714_CHG_POLL;
	}

	mod_delayed_work(system_freezable_wq, &fg->poll, 0);
	return IRQ_HANDLED;
}

/* Mask every charger and MUIC source but VBUSPOK and VBUSUVLO, then clear what is pending. */
static int sm5714_irq_setup(struct sm5714_fg *fg)
{
	int i, ret;

	ret = i2c_smbus_write_byte_data(fg->chg, SM5714_CHG_INTMSK1, (u8)~SM5714_CHG_INT1_PLUG);
	for (i = 1; !ret && i < 5; i++)
		ret = i2c_smbus_write_byte_data(fg->chg, SM5714_CHG_INTMSK1 + i, 0xff);
	for (i = 0; !ret && i < 2; i++)
		ret = i2c_smbus_write_byte_data(fg->muic, SM5714_MUIC_INTMASK1 + i, 0xff);
	if (ret)
		return ret;
	ret = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_INTMSK1);
	if (ret != (u8)~SM5714_CHG_INT1_PLUG)
		return ret < 0 ? ret : -EIO;
	sm5714_irq_clear(fg, SM5714_IRQSRC_CHG | SM5714_IRQSRC_MUIC | SM5714_IRQSRC_FG);
	return 0;
}

static char *sm5714_usb_supplied_to[] = { "battery" };

static int sm5714_fg_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = { .fwnode = dev_fwnode(&client->dev) };
	struct power_supply_config usb_cfg = { };
	struct device *dev = &client->dev;
	struct power_supply *psy;
	struct sm5714_fg *fg;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;
	fg->client = client;
	i2c_set_clientdata(client, fg);
	ret = devm_mutex_init(dev, &fg->lock);
	if (ret)
		return ret;

	fg->therm = devm_iio_channel_get(dev, "batt-therm");
	if (IS_ERR(fg->therm))
		return dev_err_probe(dev, PTR_ERR(fg->therm), "battery thermistor\n");

	ret = i2c_smbus_read_word_data(client, SM5714_FG_SYSTEM_STATUS);
	if (ret < 0)
		return dev_err_probe(dev, ret, "gauge does not answer\n");
	if (!(ret & SM5714_FG_INIT_DONE))
		dev_warn(dev, "gauge has no battery model (status %#06x): capacity unavailable\n",
			 ret);

	/* The charger block of the same chip, for the charging status only. */
	fg->chg = devm_i2c_new_dummy_device(dev, client->adapter, SM5714_CHG_ADDR);
	if (IS_ERR(fg->chg))
		return dev_err_probe(dev, PTR_ERR(fg->chg), "charger block\n");

	cfg.drv_data = fg;
	psy = devm_power_supply_register(dev, &sm5714_fg_desc, &cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy), "failed to register battery\n");
	fg->bat = psy;

	usb_cfg.drv_data = fg;
	usb_cfg.supplied_to = sm5714_usb_supplied_to;
	usb_cfg.num_supplicants = ARRAY_SIZE(sm5714_usb_supplied_to);
	fg->usb = devm_power_supply_register(dev, &sm5714_usb_desc, &usb_cfg);
	if (IS_ERR(fg->usb))
		return dev_err_probe(dev, PTR_ERR(fg->usb), "failed to register usb\n");

	fg->muic = devm_i2c_new_dummy_device(dev, client->adapter, SM5714_MUIC_ADDR);
	if (IS_ERR(fg->muic))
		return dev_err_probe(dev, PTR_ERR(fg->muic), "muic block\n");

	/*
	 * Start from the power-on rate; registered before the poll, so it runs after the poll is
	 * cancelled.
	 */
	ret = sm5714_chg_set_rate(fg, false);
	if (ret)
		return dev_err_probe(dev, ret, "charger does not take its power-on rate\n");
	ret = devm_add_action_or_reset(dev, sm5714_chg_slow, fg);
	if (ret)
		return ret;
	/* the chip keeps ENQ4FET across reboots: the first poll decides again */
	ret = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_CNTL1);
	if (ret < 0)
		return dev_err_probe(dev, ret, "charger control\n");
	fg->chg_off = !(ret & SM5714_CHG_ENQ4FET);

	fg->online = -1;	/* the first poll announces the state */
	fg->poll_period = SM5714_CHG_POLL;
	ret = devm_delayed_work_autocancel(dev, &fg->poll, sm5714_fg_poll);
	if (ret)
		return ret;

	if (client->irq > 0) {
		ret = sm5714_irq_setup(fg);
		if (!ret)
			ret = devm_request_threaded_irq(dev, client->irq, NULL, sm5714_irq,
								IRQF_ONESHOT, "sm5714", fg);
		if (ret)
			dev_warn(dev, "no plug interrupt (%d): polling every 2 s\n", ret);
		else
			fg->poll_period = SM5714_CHG_POLL_IRQ;
	}
	queue_delayed_work(system_freezable_wq, &fg->poll, 0);

	return 0;
}

static void sm5714_fg_shutdown(struct i2c_client *client)
{
	struct sm5714_fg *fg = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&fg->poll);
	sm5714_chg_set_rate(fg, false);
}

static const struct of_device_id sm5714_fg_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-fuel-gauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_fg_of_match);

static struct i2c_driver sm5714_fg_driver = {
	.driver = {
		.name = "sm5714-fuel-gauge",
		.of_match_table = sm5714_fg_of_match,
	},
	.probe = sm5714_fg_probe,
	.shutdown = sm5714_fg_shutdown,
};
module_i2c_driver(sm5714_fg_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 fuel gauge and charger status");
MODULE_LICENSE("GPL");
