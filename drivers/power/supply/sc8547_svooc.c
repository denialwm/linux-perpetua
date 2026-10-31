// SPDX-License-Identifier: GPL-2.0-only
/*
 * SuperVOOC direct charging through the SC8547D VOOC PHY.
 *
 * Session sequence and limits adapted from the Oplus AP VOOC PHY and
 * SC8547D drivers, Copyright (C) 2018-2023 Oplus. All rights reserved.
 * The adapter regulates its output current; one or two 2:1 pumps halve its
 * voltage into the battery. The Qualcomm USB supply keeps detecting the
 * charger and gives up its input for the session.
 */
#include <linux/auxiliary_bus.h>
#include <linux/debugfs.h>
#include <linux/iio/consumer.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mux/consumer.h>
#include <linux/pm_wakeup.h>
#include <linux/power/oplus_svooc.h>
#include <linux/power/sc8547.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/random.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>

#define SC8547_DPDM_CTRL	0x21
#define SC8547_DPDM_CTRL2	0x22
#define SC8547_VOOC_CTRL	0x2b
#define SC8547_TX		0x2c
#define SC8547_RX		0x2e
#define SC8547_PHY_IRQ_MASK	0x30
#define SC8547_PREDATA		0x31
#define SC8547_DET_CTRL		0x33
#define SC8547_PHY_CTRL		0x34

#define SC8547_PHY_TX_DONE	BIT(0)
#define SC8547_PHY_RX_DONE	BIT(1)
#define SC8547_PHY_TX_FAILED	BIT(3)
#define SC8547_PHY_ERRORS	(BIT(7) | BIT(6) | BIT(4) | SC8547_PHY_TX_FAILED)
/* Interrupt masks: replies only, or also the end of a current reply. */
#define SC8547_PHY_MASK_RX	0x05
#define SC8547_PHY_MASK_TX	0x04

/* A 2:1 pump and its battery path as qualified on this board. */
#define SVOOC_CIRCUIT		0x30
#define SVOOC_MAX_UA		6200000
#define SVOOC_PUMP_MAX_UA	3300000
#define SVOOC_MIN_UA		1500000
#define SVOOC_POWER_UW		55000000LL
#define SVOOC_POWER_MAX_UW	60000000LL
#define SVOOC_VBAT_MAX_UV	4480000
#define SVOOC_SECOND_UA		1400000

#define SVOOC_TICK_MS		50
#define SVOOC_BUCK_TICK_MS	20
#define SVOOC_RETRY_MS		10000
#define SVOOC_FIRST_FRAME_MS	3000
#define SVOOC_FRAME_MS		1000
#define SVOOC_IDENTIFY_MS	30000
#define SVOOC_START_MS		500
#define SVOOC_ACK_MS		1000
#define SVOOC_RAMP_MS		3000
#define SVOOC_CHANGE_MS		1000
#define SVOOC_DISCHARGE_MS	7000
#define SVOOC_BUCK_MS		400
#define SVOOC_RESTORE_MS	3000
#define SVOOC_PLUG_MS		2000

enum svooc_therm {
	SVOOC_USB_CONNECTOR,
	SVOOC_BATTERY_CONNECTOR,
	SVOOC_BOARD,
	SVOOC_MOS,
	SVOOC_THERMS,
};

static const char * const svooc_therm_names[] = {
	"usb-connector", "battery-connector", "board", "mos",
};

/* Full current up to the soft limit, none at the hard limit, in m°C. */
static const struct {
	int soft;
	int hard;
} svooc_therm_limits[] = {
	[SVOOC_USB_CONNECTOR] = { 60000, 70000 },
	[SVOOC_BATTERY_CONNECTOR] = { 60000, 70000 },
	[SVOOC_BOARD] = { 45000, 55000 },
	[SVOOC_MOS] = { 60000, 70000 },
};

enum svooc_state {
	SVOOC_IDLE,
	SVOOC_IDENTIFY,		/* buck charging, answering identification */
	SVOOC_SUSPEND,		/* identified, buck input being suspended */
	SVOOC_GRANTED,		/* adapter raising VBUS to twice the battery */
	SVOOC_STARTING,		/* main pump enabled, first current pending */
	SVOOC_RUNNING,
	SVOOC_STOPPING,		/* hardware off, buck not yet restored */
	SVOOC_DONE,		/* until the adapter is unplugged */
};

static const char * const svooc_state_names[] = {
	"idle", "identify", "suspend", "granted", "starting", "running",
	"stopping", "done",
};

struct svooc_battery {
	int uv;
	int ua;
	int capacity;
	int temp;
	int health;
};

struct sc8547_svooc {
	struct device *dev;
	struct sc8547 *main;
	struct sc8547 *second;
	struct regmap *regmap;
	struct mux_state *dpdm;
	struct iio_channel *therm[SVOOC_THERMS];
	struct power_supply *usb;
	struct power_supply *battery;
	struct power_supply *psy;
	struct notifier_block notifier;
	struct delayed_work work;
	struct dentry *debugfs;
	/* Serializes the session between the PHY interrupt and the worker. */
	struct mutex lock;
	struct oplus_svooc proto;
	enum svooc_state state;
	enum power_supply_charge_type mode;
	struct sc8547_sample sample[2];
	struct svooc_battery bat;
	int therm_mc[SVOOC_THERMS];
	unsigned long plugged_at;
	unsigned long started;
	unsigned long last_rx;
	unsigned long enabled_at;
	unsigned long second_at;
	unsigned long ack_deadline;
	unsigned long change_deadline;
	unsigned long ramp_after;
	unsigned long discharge_since;
	unsigned long stopped_at;
	unsigned int ticks;
	unsigned int frames;
	unsigned int quiet;
	unsigned int disagree;
	int limit_ua;
	int taper_ua;
	int target_ua;
	int applied_ua;
	int pending_ua;
	int error;
	bool phy_on;
	bool route;
	bool buck_off;
	bool second_on;
	bool dual;
	bool change;
	bool skipped;
	bool stopping;
};

static int svooc_get(struct power_supply *psy, enum power_supply_property prop,
		     int *value)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(psy, prop, &val);
	if (!ret)
		*value = val.intval;
	return ret;
}

static int svooc_set_input(struct sc8547_svooc *sv, bool suspend)
{
	union power_supply_propval val = { .intval = suspend ? 0 : -1 };

	return power_supply_set_property(sv->usb,
					 POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
}

static int svooc_read_battery(struct sc8547_svooc *sv, struct svooc_battery *bat)
{
	int ret;

	ret = svooc_get(sv->battery, POWER_SUPPLY_PROP_VOLTAGE_NOW, &bat->uv);
	if (!ret)
		ret = svooc_get(sv->battery, POWER_SUPPLY_PROP_CURRENT_NOW, &bat->ua);
	if (!ret)
		ret = svooc_get(sv->battery, POWER_SUPPLY_PROP_CAPACITY, &bat->capacity);
	if (!ret)
		ret = svooc_get(sv->battery, POWER_SUPPLY_PROP_TEMP, &bat->temp);
	if (!ret)
		ret = svooc_get(sv->battery, POWER_SUPPLY_PROP_HEALTH, &bat->health);
	return ret;
}

static int svooc_read_therms(struct sc8547_svooc *sv, int *mc)
{
	int i, ret;

	for (i = 0; i < SVOOC_THERMS; i++) {
		ret = iio_read_channel_processed(sv->therm[i], &mc[i]);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static bool svooc_charger(struct sc8547_svooc *sv, int *vbus_uv)
{
	int online, type;

	if (svooc_get(sv->usb, POWER_SUPPLY_PROP_ONLINE, &online) || !online ||
	    svooc_get(sv->usb, POWER_SUPPLY_PROP_USB_TYPE, &type) ||
	    svooc_get(sv->usb, POWER_SUPPLY_PROP_VOLTAGE_NOW, vbus_uv))
		return false;
	/* SuperVOOC adapters present as a dedicated or proprietary charger. */
	return type == POWER_SUPPLY_USB_TYPE_DCP ||
	       type == POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID;
}

static int svooc_scale(int value, int soft, int hard)
{
	if (value <= soft)
		return SVOOC_MAX_UA;
	if (value >= hard)
		return 0;
	return div_s64((s64)SVOOC_MAX_UA * (hard - value), hard - soft);
}

/* The current the temperatures allow; the battery keeps 1.5 A up to 44 °C. */
static int svooc_thermal_ua(struct sc8547_svooc *sv)
{
	int ua, die, i;

	die = max(sv->sample[0].temp, sv->second ? sv->sample[1].temp : 0);
	ua = svooc_scale(die, 600, 700);
	for (i = 0; i < SVOOC_THERMS; i++)
		ua = min(ua, svooc_scale(sv->therm_mc[i], svooc_therm_limits[i].soft,
					 svooc_therm_limits[i].hard));
	if (sv->bat.temp < 440)
		ua = min(ua, max(svooc_scale(sv->bat.temp, 380, 440), SVOOC_MIN_UA));
	else
		ua = 0;
	if (sv->bat.temp < 210)
		ua = min(ua, 2500000);
	return ua;
}

static int svooc_vbat(struct sc8547_svooc *sv)
{
	/* Neither sense point may hide the taper from the other. */
	return max(sv->bat.uv, sv->sample[0].vbat_uv);
}

static int svooc_taper_ua(int uv)
{
	if (uv >= 4460000)
		return 1500000;
	if (uv >= 4440000)
		return 2000000;
	if (uv >= 4420000)
		return 3000000;
	if (uv >= 4400000)
		return 4000000;
	return SVOOC_MAX_UA;
}

static bool svooc_admissible(struct sc8547_svooc *sv)
{
	const struct svooc_battery *bat = &sv->bat;
	int i;

	if (bat->health != POWER_SUPPLY_HEALTH_GOOD || bat->capacity >= 90 ||
	    bat->temp < 150 || bat->temp >= 440 ||
	    bat->uv < 3300000 || bat->uv >= 4400000)
		return false;
	for (i = 0; i < SVOOC_THERMS; i++)
		if (sv->therm_mc[i] < 0 || sv->therm_mc[i] >= svooc_therm_limits[i].soft)
			return false;
	return true;
}

/* The session's current, chosen at entry from the charge level. */
static int svooc_entry_ua(struct sc8547_svooc *sv)
{
	int ua = sv->bat.capacity <= 55 ? SVOOC_MAX_UA :
		 sv->bat.capacity < 85 ? 3000000 : 2000000;

	ua = min(ua, svooc_thermal_ua(sv));
	ua = min(ua, svooc_taper_ua(sv->bat.uv));
	/* Within the power budget at the top of the VBUS window. */
	ua = min_t(int, ua, div_s64(SVOOC_POWER_UW * 1000000,
				    2 * sv->bat.uv + 700000));
	return ua / 100000 * 100000;
}

static int svooc_ibus(struct sc8547_svooc *sv)
{
	return sv->sample[0].ibus_ua + (sv->second_on ? sv->sample[1].ibus_ua : 0);
}

static int svooc_write_frame(struct sc8547_svooc *sv, unsigned int reg,
			     u8 payload, bool shifted)
{
	u16 frame = oplus_svooc_frame(payload, shifted);
	u8 data[2] = { frame >> 8, frame & 0xff };

	return regmap_bulk_write(sv->regmap, reg, data, sizeof(data));
}

static int svooc_write_verify(struct sc8547_svooc *sv, unsigned int reg,
			      unsigned int value)
{
	unsigned int read;
	int ret;

	ret = regmap_write(sv->regmap, reg, value);
	if (!ret)
		ret = regmap_read(sv->regmap, reg, &read);
	if (!ret && read != value)
		ret = -EIO;
	return ret;
}

static int svooc_phy_setup(struct sc8547_svooc *sv)
{
	static const u8 data[2];
	int ret;

	ret = regmap_bulk_write(sv->regmap, SC8547_PREDATA, data, sizeof(data));
	/* Hold bit 0 for 800 us, or the adapter misreads current commands. */
	if (!ret)
		ret = regmap_set_bits(sv->regmap, SC8547_PHY_CTRL, BIT(6));
	if (!ret)
		ret = svooc_write_verify(sv, SC8547_DPDM_CTRL, 0x21);
	if (!ret)
		ret = svooc_write_verify(sv, SC8547_DPDM_CTRL2, 0);
	if (!ret)
		ret = svooc_write_verify(sv, SC8547_DET_CTRL, 0xd1);
	if (!ret)
		ret = svooc_write_verify(sv, SC8547_PHY_IRQ_MASK, SC8547_PHY_MASK_RX);
	return ret;
}

static int svooc_phy_stop(struct sc8547_svooc *sv)
{
	static const u8 data[2];
	int ret, err;

	/* Attempt every step: a failed write may still have landed. */
	ret = regmap_write(sv->regmap, SC8547_VOOC_CTRL, 0);
	err = regmap_write(sv->regmap, SC8547_PHY_IRQ_MASK, 0x7f);
	ret = ret ?: err;
	err = svooc_write_verify(sv, SC8547_DPDM_CTRL, 0xc0);
	ret = ret ?: err;
	err = regmap_bulk_write(sv->regmap, SC8547_TX, data, sizeof(data));
	ret = ret ?: err;
	err = regmap_bulk_write(sv->regmap, SC8547_PREDATA, data, sizeof(data));
	return ret ?: err;
}

/* Turn the hardware off at once; the worker restores the buck. */
static void svooc_stop_locked(struct sc8547_svooc *sv, int error)
{
	if (sv->state < SVOOC_IDENTIFY || sv->state >= SVOOC_STOPPING)
		return;
	sv->error = error;
	if (error)
		dev_warn(sv->dev, "SuperVOOC stopped in %s: %d\n",
			 svooc_state_names[sv->state], error);
	else
		dev_info(sv->dev, "SuperVOOC finished\n");
	if (sv->second && sc8547_disable(sv->second))
		dev_crit(sv->dev, "cannot stop the second pump\n");
	if (sc8547_disable(sv->main))
		dev_crit(sv->dev, "cannot stop the main pump\n");
	sv->phy_on = false;
	if (svooc_phy_stop(sv))
		dev_err(sv->dev, "cannot stop the VOOC PHY\n");
	sv->proto.permit = false;
	sv->second_on = false;
	sv->dual = false;
	sv->pending_ua = 0;
	sv->applied_ua = 0;
	sv->state = SVOOC_STOPPING;
	sv->stopped_at = jiffies;
	mod_delayed_work(system_highpri_wq, &sv->work, 0);
}

static int svooc_send(struct sc8547_svooc *sv, u8 payload, bool shifted)
{
	return svooc_write_frame(sv, SC8547_TX, payload, shifted);
}

/* GET_VBAT: the battery voltage the adapter regulates against. */
static u8 svooc_vbat_reply(struct sc8547_svooc *sv)
{
	const struct sc8547_sample *s = &sv->sample[0];
	int uv;

	if (sv->change)
		return OPLUS_SVOOC_CHANGE_CURRENT;
	/* Half the input, less the drop to the battery under load. */
	uv = s->vbus_uv / 2 - (s->vout_uv - sv->bat.uv) / 4 -
	     div_s64((s64)svooc_ibus(sv) * 75, 1000);
	return oplus_svooc_vbat(uv);
}

static int svooc_power_command(struct sc8547_svooc *sv,
			       const struct oplus_svooc_request *req, u8 *payload)
{
	int ret, ua, vbus;

	switch (req->command) {
	case OPLUS_SVOOC_VBUS_OK:
		if (sv->state != SVOOC_GRANTED && sv->state != SVOOC_STARTING &&
		    sv->state != SVOOC_RUNNING)
			return -EPROTO;
		ret = sc8547_read_vbus(sv->main, &vbus);
		if (ret)
			return ret;
		*payload = oplus_svooc_vbus(vbus, sv->bat.uv);
		/* The pump switches on before the adapter hears "normal". */
		if (*payload == OPLUS_SVOOC_VBUS_NORMAL && sv->state == SVOOC_GRANTED) {
			ret = sc8547_enable(sv->main);
			if (ret)
				return ret;
			sv->state = SVOOC_STARTING;
			sv->enabled_at = jiffies;
		}
		return 0;
	case OPLUS_SVOOC_GET_CURRENT:
		if (sv->state == SVOOC_STARTING && !sv->applied_ua)
			ua = sv->limit_ua;
		else if (sv->state == SVOOC_STARTING || sv->state == SVOOC_RUNNING)
			ua = sv->change ? sv->target_ua : sv->applied_ua;
		else
			return -EPROTO;
		ret = oplus_svooc_current(ua, payload);
		if (ret)
			return ret;
		ret = regmap_write(sv->regmap, SC8547_PHY_IRQ_MASK, SC8547_PHY_MASK_TX);
		if (ret)
			return ret;
		sv->pending_ua = ua;
		sv->ack_deadline = jiffies + msecs_to_jiffies(SVOOC_ACK_MS);
		return 0;
	case OPLUS_SVOOC_GET_VBAT:
		if (sv->state != SVOOC_STARTING && sv->state != SVOOC_RUNNING)
			return -EPROTO;
		*payload = svooc_vbat_reply(sv);
		return 0;
	case OPLUS_SVOOC_AP_STATUS:
		*payload = 0;
		return 0;
	default:
		return -EPROTO;
	}
}

static int svooc_receive(struct sc8547_svooc *sv, u8 rx, u8 flags)
{
	struct oplus_svooc_request req;
	u8 payload;
	int ret;

	if (sv->pending_ua && (flags & SC8547_PHY_TX_DONE)) {
		sv->applied_ua = sv->pending_ua;
		sv->pending_ua = 0;
		sv->change = false;
		sv->ramp_after = jiffies + msecs_to_jiffies(SVOOC_RAMP_MS);
		ret = regmap_write(sv->regmap, SC8547_PHY_IRQ_MASK, SC8547_PHY_MASK_RX);
		if (ret)
			return ret;
	} else if (sv->pending_ua && (flags & SC8547_PHY_RX_DONE)) {
		return -EPROTO;
	}
	if (flags & SC8547_PHY_ERRORS) {
		/*
		 * A lone failed reply to a battery query happens during long
		 * sessions; the adapter asks again. Anything more is fatal.
		 */
		if ((flags & SC8547_PHY_ERRORS) == SC8547_PHY_TX_FAILED &&
		    sv->state == SVOOC_RUNNING && !sv->skipped && !sv->change &&
		    !oplus_svooc_decode(rx, &req) &&
		    req.command == OPLUS_SVOOC_GET_VBAT) {
			sv->skipped = true;
			return 0;
		}
		return -EPROTO;
	}
	if (!(flags & SC8547_PHY_RX_DONE))
		return 0;

	sv->skipped = false;
	ret = oplus_svooc_decode(rx, &req);
	if (ret)
		return ret;
	sv->last_rx = jiffies;
	sv->frames++;
	ret = oplus_svooc_handle(&sv->proto, &req, get_random_u16(), &payload);
	if (ret < 0)
		return ret;
	/* Suspend the buck while the adapter reports its model and curve. */
	if (sv->state == SVOOC_IDENTIFY && sv->proto.identified)
		mod_delayed_work(system_highpri_wq, &sv->work, 0);
	if (ret) {
		if (req.command == OPLUS_SVOOC_FASTCHG && sv->proto.granted &&
		    sv->state == SVOOC_SUSPEND) {
			/* Missed replies make the PHY ask for a lower voltage. */
			ret = svooc_write_frame(sv, SC8547_PREDATA,
						OPLUS_SVOOC_VBUS_HIGH, false);
			if (ret)
				return ret;
			sv->state = SVOOC_GRANTED;
		}
		return svooc_send(sv, payload, req.shifted);
	}
	ret = svooc_power_command(sv, &req, &payload);
	if (!ret)
		ret = svooc_send(sv, payload, req.shifted);
	if (!ret && req.command == OPLUS_SVOOC_GET_VBAT)
		ret = svooc_write_frame(sv, SC8547_PREDATA, payload, req.shifted);
	return ret;
}

static void svooc_phy_irq(void *data)
{
	struct sc8547_svooc *sv = data;
	u8 rx[2];
	int ret;

	mutex_lock(&sv->lock);
	if (sv->phy_on) {
		ret = regmap_bulk_read(sv->regmap, SC8547_RX, rx, sizeof(rx));
		if (!ret)
			ret = svooc_receive(sv, rx[0], rx[1]);
		if (ret)
			svooc_stop_locked(sv, ret);
	}
	mutex_unlock(&sv->lock);
}

static void svooc_start(struct sc8547_svooc *sv)
{
	struct svooc_battery bat;
	int therm[SVOOC_THERMS], vbus, ret;

	if (sv->mode != POWER_SUPPLY_CHARGE_TYPE_FAST || !svooc_charger(sv, &vbus) ||
	    vbus < 4500000 || vbus > 5500000)
		return;
	if (time_before(jiffies, sv->plugged_at + msecs_to_jiffies(SVOOC_PLUG_MS))) {
		mod_delayed_work(system_highpri_wq, &sv->work,
				 sv->plugged_at + msecs_to_jiffies(SVOOC_PLUG_MS) - jiffies);
		return;
	}
	if (svooc_read_battery(sv, &bat) || svooc_read_therms(sv, therm))
		return;

	mutex_lock(&sv->lock);
	sv->bat = bat;
	memcpy(sv->therm_mc, therm, sizeof(therm));
	memset(sv->sample, 0, sizeof(sv->sample));
	sv->limit_ua = svooc_entry_ua(sv);
	if (!sv->second)
		sv->limit_ua = min(sv->limit_ua, SVOOC_PUMP_MAX_UA);
	mutex_unlock(&sv->lock);
	if (!svooc_admissible(sv) || sv->limit_ua < SVOOC_MIN_UA) {
		mod_delayed_work(system_highpri_wq, &sv->work,
				 msecs_to_jiffies(SVOOC_RETRY_MS));
		return;
	}

	ret = sc8547_prepare(sv->main);
	if (!ret && sv->second)
		ret = sc8547_prepare(sv->second);
	if (!ret) {
		ret = mux_state_try_select(sv->dpdm);
		sv->route = !ret;
	}
	if (!ret)
		ret = svooc_phy_setup(sv);
	if (ret) {
		dev_warn(sv->dev, "cannot start SuperVOOC: %d\n", ret);
		svooc_phy_stop(sv);
		if (sv->second)
			sc8547_disable(sv->second);
		sc8547_disable(sv->main);
		if (sv->route)
			mux_state_deselect(sv->dpdm);
		sv->route = false;
		mutex_lock(&sv->lock);
		sv->error = ret;
		sv->state = SVOOC_DONE;
		mutex_unlock(&sv->lock);
		return;
	}

	pm_stay_awake(sv->dev);
	mutex_lock(&sv->lock);
	oplus_svooc_reset(&sv->proto);
	sv->state = SVOOC_IDENTIFY;
	sv->error = 0;
	sv->frames = 0;
	sv->ticks = 0;
	sv->taper_ua = sv->limit_ua;
	sv->target_ua = 0;
	sv->change = false;
	sv->skipped = false;
	sv->disagree = 0;
	sv->discharge_since = 0;
	sv->started = jiffies;
	sv->last_rx = jiffies;
	sv->phy_on = true;
	/* The handshake starts the adapter talking: be ready first. */
	ret = regmap_write(sv->regmap, SC8547_VOOC_CTRL, 0x81);
	if (ret)
		svooc_stop_locked(sv, ret);
	else
		mod_delayed_work(system_highpri_wq, &sv->work,
				 msecs_to_jiffies(SVOOC_TICK_MS));
	mutex_unlock(&sv->lock);
}

static int svooc_check_running(struct sc8547_svooc *sv)
{
	const struct sc8547_sample *m = &sv->sample[0], *s = &sv->sample[1];
	int ibus = svooc_ibus(sv), vbat = sv->bat.uv;
	s64 power = (s64)m->vbus_uv * ibus;

	if (!m->switching)
		return -EIO;
	if (m->vbus_uv < 2 * vbat - 100000 || m->vbus_uv > 2 * vbat + 700000)
		return -ERANGE;
	if (ibus > sv->limit_ua + 300000 || m->ibus_ua > SVOOC_PUMP_MAX_UA ||
	    (!sv->dual && ibus > SVOOC_PUMP_MAX_UA) ||
	    power > SVOOC_POWER_MAX_UW * 1000000)
		return -ERANGE;
	if (sv->second_on && s->ibus_ua > SVOOC_PUMP_MAX_UA)
		return -ERANGE;
	/* The pumps are sampled in turn: one pair may straddle a step. */
	if (sv->second_on && abs(s->vbus_uv - m->vbus_uv) > 250000) {
		if (++sv->disagree > 1)
			return -ERANGE;
	} else {
		sv->disagree = 0;
	}
	if (sv->dual && !s->switching)
		return -EIO;
	if (sv->state == SVOOC_RUNNING && !sv->pending_ua && !sv->change &&
	    time_after_eq(jiffies, sv->ramp_after) && ibus > sv->applied_ua + 300000)
		return -ERANGE;
	return 0;
}

/* Bring in the second pump once the main one carries enough to share. */
static int svooc_second(struct sc8547_svooc *sv, bool *enable)
{
	const struct sc8547_sample *m = &sv->sample[0], *s = &sv->sample[1];

	if (!sv->second || sv->dual || sv->state != SVOOC_RUNNING)
		return 0;
	if (!sv->second_on) {
		if (m->ibus_ua > SVOOC_SECOND_UA &&
		    abs(s->vbus_uv - m->vbus_uv) <= 250000) {
			sv->second_on = true;
			sv->second_at = jiffies;
			*enable = true;
		}
		return 0;
	}
	if (time_before(jiffies, sv->second_at + msecs_to_jiffies(300)))
		return 0;
	if (s->switching && m->ibus_ua >= 500000 && s->ibus_ua >= 500000 &&
	    abs(m->ibus_ua - s->ibus_ua) <= 1000000) {
		sv->dual = true;
		return 0;
	}
	return time_after(jiffies, sv->second_at + msecs_to_jiffies(500)) ? -EIO : 0;
}

/*
 * Reductions apply at once, increases by 100 mA every three seconds. A
 * change goes out as the reply to the next battery query, which makes the
 * adapter ask for the current. Returns 1 when the buck should finish.
 */
static int svooc_regulate(struct sc8547_svooc *sv)
{
	int vbus = sv->sample[0].vbus_uv, ua;
	s64 power = (s64)vbus * svooc_ibus(sv);

	if (sv->state != SVOOC_RUNNING || (sv->second && !sv->dual &&
					    sv->applied_ua > SVOOC_PUMP_MAX_UA))
		return 0;
	sv->taper_ua = min(sv->taper_ua, svooc_taper_ua(svooc_vbat(sv)));
	ua = min(sv->limit_ua, sv->taper_ua);
	ua = min(ua, svooc_thermal_ua(sv));
	ua = min_t(int, ua, div_s64(SVOOC_POWER_UW * 1000000, vbus) - 100000);
	if (power > SVOOC_POWER_UW * 1000000)
		ua = min_t(int, ua, sv->applied_ua - 100000 -
			   div_s64(power - (SVOOC_POWER_UW - 500000) * 1000000, vbus));
	ua = ua / 100000 * 100000;
	if (ua < SVOOC_MIN_UA)
		return 1;
	if (sv->pending_ua || sv->change ||
	    time_before(jiffies, sv->change_deadline))
		return 0;
	if (ua >= sv->applied_ua) {
		if (time_before(jiffies, sv->ramp_after))
			return 0;
		ua = min(ua, sv->applied_ua + 100000);
	}
	if (ua == sv->applied_ua)
		return 0;
	sv->target_ua = ua;
	sv->change = true;
	sv->change_deadline = jiffies + msecs_to_jiffies(SVOOC_CHANGE_MS);
	return 0;
}

/* Returns 1 to finish normally, or a negative error to stop. */
static int svooc_check(struct sc8547_svooc *sv)
{
	unsigned long now = jiffies;
	int frame_ms = sv->frames ? SVOOC_FRAME_MS : SVOOC_FIRST_FRAME_MS;
	int i, vbat = svooc_vbat(sv);

	if (sv->mode != POWER_SUPPLY_CHARGE_TYPE_FAST || sv->bat.capacity >= 90)
		return 1;
	if (time_after(now, sv->last_rx + msecs_to_jiffies(frame_ms)))
		return -ETIMEDOUT;
	if (sv->state < SVOOC_RUNNING &&
	    time_after(now, sv->started + msecs_to_jiffies(SVOOC_IDENTIFY_MS)))
		return -ETIMEDOUT;
	if (sv->pending_ua && time_after(now, sv->ack_deadline))
		return -ETIMEDOUT;
	if (sv->change && time_after(now, sv->change_deadline))
		return -ETIMEDOUT;
	if (sv->bat.health != POWER_SUPPLY_HEALTH_GOOD || vbat >= SVOOC_VBAT_MAX_UV ||
	    sv->bat.temp >= 440)
		return -ERANGE;
	for (i = 0; i < SVOOC_THERMS; i++)
		if (sv->therm_mc[i] >= svooc_therm_limits[i].hard)
			return -ERANGE;
	if (max(sv->sample[0].temp, sv->sample[1].temp) >= 700)
		return -ERANGE;
	if (sv->bat.ua <= -3000000) {
		if (!sv->discharge_since)
			sv->discharge_since = now;
		else if (time_after(now, sv->discharge_since +
				    msecs_to_jiffies(SVOOC_DISCHARGE_MS)))
			return -ERANGE;
	} else {
		sv->discharge_since = 0;
	}
	/* Before the grant the buck charges and VBUS stays at 5 V. */
	if (sv->state < SVOOC_GRANTED && (sv->sample[0].switching ||
					   sv->sample[0].vbus_uv > 5500000))
		return -EPROTO;
	if (sv->state == SVOOC_STARTING) {
		if (sv->sample[0].switching && sv->applied_ua) {
			sv->state = SVOOC_RUNNING;
			sv->ramp_after = now + msecs_to_jiffies(SVOOC_RAMP_MS);
			dev_info(sv->dev, "SuperVOOC adapter %02x at %d mA\n",
				 sv->proto.model, sv->applied_ua / 1000);
		} else if (time_after(now, sv->enabled_at +
				      msecs_to_jiffies(SVOOC_START_MS))) {
			return -ETIMEDOUT;
		}
	}
	if (sv->state == SVOOC_STARTING || sv->state == SVOOC_RUNNING) {
		i = svooc_check_running(sv);
		if (i && (sv->state == SVOOC_RUNNING || sv->sample[0].switching))
			return i;
	}
	return svooc_regulate(sv);
}

static void svooc_tick(struct sc8547_svooc *sv)
{
	struct sc8547_sample sample[2] = {};
	struct svooc_battery bat;
	int therm[SVOOC_THERMS], usb_ua = 0, usb_uv = 0, online = 0;
	bool read_bat, read_therm, read_usb, suspend = false, second = false, changed;
	enum svooc_state state;
	int ret;

	mutex_lock(&sv->lock);
	state = sv->state;
	sv->ticks++;
	read_bat = !(sv->ticks % 4);
	read_therm = !(sv->ticks % 10);
	mutex_unlock(&sv->lock);
	read_usb = read_bat || state == SVOOC_SUSPEND;

	/* Slow reads outside the lock: the PHY must answer within milliseconds. */
	ret = sc8547_sample(sv->main, &sample[0]);
	if (!ret && sv->second)
		ret = sc8547_sample(sv->second, &sample[1]);
	if (!ret && read_bat)
		ret = svooc_read_battery(sv, &bat);
	if (!ret && read_therm)
		ret = svooc_read_therms(sv, therm);
	if (!ret && read_usb) {
		ret = svooc_get(sv->usb, POWER_SUPPLY_PROP_ONLINE, &online);
		if (!ret)
			ret = svooc_get(sv->usb, POWER_SUPPLY_PROP_VOLTAGE_NOW, &usb_uv);
		if (!ret)
			ret = svooc_get(sv->usb, POWER_SUPPLY_PROP_CURRENT_NOW, &usb_ua);
	}

	mutex_lock(&sv->lock);
	state = sv->state;
	if (sv->state < SVOOC_IDENTIFY || sv->state >= SVOOC_STOPPING)
		goto unlock;
	if (ret)
		goto stop;
	memcpy(sv->sample, sample, sizeof(sample));
	if (read_bat)
		sv->bat = bat;
	if (read_therm)
		memcpy(sv->therm_mc, therm, sizeof(therm));
	if (read_usb && !online) {
		ret = -ENOTCONN;
		goto stop;
	}

	if (sv->state == SVOOC_IDENTIFY && sv->proto.identified) {
		sv->state = SVOOC_SUSPEND;
		sv->quiet = 0;
		sv->buck_off = true;
		sv->ack_deadline = jiffies + msecs_to_jiffies(SVOOC_BUCK_MS);
		suspend = true;
	} else if (sv->state == SVOOC_SUSPEND && !sv->proto.permit) {
		/* The buck must be seen idle three times before the grant. */
		if (usb_ua <= 25000 && usb_uv >= 4500000 && usb_uv <= 5500000)
			sv->quiet++;
		else
			sv->quiet = 0;
		if (sv->quiet >= 3)
			sv->proto.permit = true;
		else if (time_after(jiffies, sv->ack_deadline))
			ret = -ETIMEDOUT;
	}
	if (!ret)
		ret = svooc_check(sv);
	if (!ret)
		ret = svooc_second(sv, &second);
	if (ret)
		goto stop;
	mutex_unlock(&sv->lock);

	if (suspend)
		ret = svooc_set_input(sv, true);
	if (!ret && second)
		ret = sc8547_enable(sv->second);
	mutex_lock(&sv->lock);
	if (ret)
		goto stop;
	mod_delayed_work(system_highpri_wq, &sv->work,
			 msecs_to_jiffies(sv->state == SVOOC_SUSPEND ? SVOOC_BUCK_TICK_MS :
					  SVOOC_TICK_MS));
	goto unlock;
stop:
	svooc_stop_locked(sv, ret > 0 ? 0 : ret);
unlock:
	changed = sv->state != state;
	mutex_unlock(&sv->lock);
	if (changed)
		power_supply_changed(sv->psy);
}

/* With the pumps and PHY off, give D+/D- and the input back. */
static void svooc_restore(struct sc8547_svooc *sv)
{
	int online = 0, vbus = 0;

	if (sv->route) {
		if (mux_state_deselect(sv->dpdm))
			dev_err(sv->dev, "cannot release D+/D-\n");
		sv->route = false;
	}
	if (sv->buck_off) {
		/* The adapter drops to 5 V once replies stop. */
		svooc_get(sv->usb, POWER_SUPPLY_PROP_ONLINE, &online);
		svooc_get(sv->usb, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbus);
		if (online && vbus > 5500000 && !sv->stopping &&
		    time_before(jiffies, sv->stopped_at + msecs_to_jiffies(SVOOC_RESTORE_MS))) {
			mod_delayed_work(system_highpri_wq, &sv->work, msecs_to_jiffies(100));
			return;
		}
		if (svooc_set_input(sv, false))
			dev_err(sv->dev, "cannot restore the charger input\n");
		sv->buck_off = false;
	}
	mutex_lock(&sv->lock);
	sv->state = SVOOC_DONE;
	mutex_unlock(&sv->lock);
	pm_relax(sv->dev);
	power_supply_changed(sv->psy);
	/* The adapter may already be gone. */
	if (!sv->stopping)
		mod_delayed_work(system_highpri_wq, &sv->work, 0);
}

static void svooc_work(struct work_struct *work)
{
	struct sc8547_svooc *sv = container_of(work, struct sc8547_svooc, work.work);
	enum svooc_state state;
	int online = 0;

	mutex_lock(&sv->lock);
	state = sv->state;
	mutex_unlock(&sv->lock);

	switch (state) {
	case SVOOC_IDLE:
		if (!sv->stopping)
			svooc_start(sv);
		break;
	case SVOOC_STOPPING:
		svooc_restore(sv);
		break;
	case SVOOC_DONE:
		/* A new attempt only for a new plug-in. */
		svooc_get(sv->usb, POWER_SUPPLY_PROP_ONLINE, &online);
		if (!online) {
			mutex_lock(&sv->lock);
			sv->state = SVOOC_IDLE;
			sv->error = 0;
			mutex_unlock(&sv->lock);
		}
		break;
	default:
		svooc_tick(sv);
	}
}

static int svooc_supply_changed(struct notifier_block *nb, unsigned long event,
				void *data)
{
	struct sc8547_svooc *sv = container_of(nb, struct sc8547_svooc, notifier);
	int online = 0;

	if (data != sv->usb)
		return NOTIFY_DONE;
	if (!svooc_get(sv->usb, POWER_SUPPLY_PROP_ONLINE, &online) && !online)
		sv->plugged_at = jiffies;
	if (READ_ONCE(sv->state) == SVOOC_IDLE || READ_ONCE(sv->state) == SVOOC_DONE)
		mod_delayed_work(system_highpri_wq, &sv->work, 0);
	return NOTIFY_OK;
}

static enum power_supply_property svooc_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPES,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int svooc_get_property(struct power_supply *psy,
			      enum power_supply_property prop,
			      union power_supply_propval *val)
{
	struct sc8547_svooc *sv = power_supply_get_drvdata(psy);
	bool running;
	int ret = 0;

	mutex_lock(&sv->lock);
	running = sv->state == SVOOC_RUNNING;
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = running;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = running ? POWER_SUPPLY_STATUS_CHARGING :
			POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPES:
		val->intval = sv->mode;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = running ? sv->sample[0].vbus_uv : 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = running ? svooc_ibus(sv) : 0;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = running ? div_s64((s64)sv->sample[0].vbus_uv *
						svooc_ibus(sv), 1000000) : 0;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SuperVOOC";
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&sv->lock);
	return ret;
}

static int svooc_set_property(struct power_supply *psy,
			      enum power_supply_property prop,
			      const union power_supply_propval *val)
{
	struct sc8547_svooc *sv = power_supply_get_drvdata(psy);

	if (prop != POWER_SUPPLY_PROP_CHARGE_TYPES ||
	    (val->intval != POWER_SUPPLY_CHARGE_TYPE_STANDARD &&
	     val->intval != POWER_SUPPLY_CHARGE_TYPE_FAST))
		return -EINVAL;
	mutex_lock(&sv->lock);
	sv->mode = val->intval;
	if (sv->mode == POWER_SUPPLY_CHARGE_TYPE_FAST && sv->state == SVOOC_DONE &&
	    !sv->error)
		sv->state = SVOOC_IDLE;
	mutex_unlock(&sv->lock);
	/* The worker stops a running session at its next tick. */
	mod_delayed_work(system_highpri_wq, &sv->work, 0);
	return 0;
}

static int svooc_property_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_CHARGE_TYPES;
}

static const struct power_supply_desc svooc_psy_desc = {
	.name = "sc8547-svooc",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.charge_types = BIT(POWER_SUPPLY_CHARGE_TYPE_STANDARD) |
			BIT(POWER_SUPPLY_CHARGE_TYPE_FAST),
	.properties = svooc_props,
	.num_properties = ARRAY_SIZE(svooc_props),
	.get_property = svooc_get_property,
	.set_property = svooc_set_property,
	.property_is_writeable = svooc_property_is_writeable,
};

static int svooc_status_show(struct seq_file *s, void *data)
{
	struct sc8547_svooc *sv = s->private;
	const struct sc8547_sample *m = &sv->sample[0], *c = &sv->sample[1];

	mutex_lock(&sv->lock);
	seq_printf(s, "state %s error %d frames %u model %02x\n",
		   svooc_state_names[sv->state], sv->error, sv->frames, sv->proto.model);
	seq_printf(s, "limit %d target %d applied %d pending %d taper %d uA\n",
		   sv->limit_ua, sv->change ? sv->target_ua : 0, sv->applied_ua,
		   sv->pending_ua, sv->taper_ua);
	seq_printf(s, "main %s vbus %d ibus %d vout %d vbat %d uV/uA die %d\n",
		   m->switching ? "on" : "off", m->vbus_uv, m->ibus_ua,
		   m->vout_uv, m->vbat_uv, m->temp);
	if (sv->second)
		seq_printf(s, "second %s%s vbus %d ibus %d die %d\n",
			   c->switching ? "on" : "off", sv->dual ? " shared" : "",
			   c->vbus_uv, c->ibus_ua, c->temp);
	seq_printf(s, "battery %d uV %d uA %d%% %d dC\n", sv->bat.uv, sv->bat.ua,
		   sv->bat.capacity, sv->bat.temp);
	seq_printf(s, "thermistors %d %d %d %d mC\n", sv->therm_mc[0],
		   sv->therm_mc[1], sv->therm_mc[2], sv->therm_mc[3]);
	mutex_unlock(&sv->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(svooc_status);

static void svooc_put_supply(void *psy)
{
	power_supply_put(psy);
}

static void svooc_cleanup(void *data)
{
	struct sc8547_svooc *sv = data;

	power_supply_unreg_notifier(&sv->notifier);
	sv->stopping = true;
	cancel_delayed_work_sync(&sv->work);
	mutex_lock(&sv->lock);
	svooc_stop_locked(sv, -ESHUTDOWN);
	mutex_unlock(&sv->lock);
	cancel_delayed_work_sync(&sv->work);
	if (sv->state == SVOOC_STOPPING)
		svooc_restore(sv);
	sc8547_set_phy_handler(sv->main, NULL, NULL);
	debugfs_remove_recursive(sv->debugfs);
	device_init_wakeup(sv->dev, false);
}

static int svooc_get_curve(struct sc8547_svooc *sv)
{
	u32 curve[OPLUS_SVOOC_CURVE_LEN];
	int ret, i;

	ret = device_property_read_u32_array(sv->dev, "oneplus,svooc-battery-curve",
					     curve, ARRAY_SIZE(curve));
	if (ret)
		return dev_err_probe(sv->dev, ret, "Missing battery curve\n");
	/*
	 * Pairs of battery voltage and current, sent as protocol codes. The
	 * curve may go one step above the GET_VBAT range, to 4.664 V.
	 */
	for (i = 0; i < OPLUS_SVOOC_CURVE_LEN; i += 2) {
		if (curve[i] < 3404000 || curve[i] > 4664000 ||
		    oplus_svooc_current(curve[i + 1], &sv->proto.curve[i + 1]))
			return dev_err_probe(sv->dev, -EINVAL, "Invalid battery curve\n");
		sv->proto.curve[i] = (curve[i] - 3404000) / 10000;
	}
	return 0;
}

static int svooc_probe(struct auxiliary_device *adev,
		       const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct power_supply_config cfg = {};
	struct fwnode_handle *node;
	struct sc8547_svooc *sv;
	int ret, i;

	sv = devm_kzalloc(dev, sizeof(*sv), GFP_KERNEL);
	if (!sv)
		return -ENOMEM;
	sv->dev = dev;
	sv->mode = POWER_SUPPLY_CHARGE_TYPE_FAST;
	sv->proto.circuit = SVOOC_CIRCUIT;
	sv->plugged_at = jiffies;
	mutex_init(&sv->lock);
	INIT_DELAYED_WORK(&sv->work, svooc_work);
	ret = svooc_get_curve(sv);
	if (ret)
		return ret;

	sv->main = sc8547_get(dev, dev_fwnode(dev->parent));
	if (IS_ERR(sv->main))
		return dev_err_probe(dev, PTR_ERR(sv->main), "Cannot get the pump\n");
	sv->regmap = sc8547_regmap(sv->main);
	node = fwnode_find_reference(dev_fwnode(dev), "southchip,secondary-pump", 0);
	if (!IS_ERR(node)) {
		sv->second = sc8547_get(dev, node);
		fwnode_handle_put(node);
		if (IS_ERR(sv->second))
			return dev_err_probe(dev, PTR_ERR(sv->second),
					     "Cannot get the second pump\n");
	}
	sv->dpdm = devm_mux_state_get(dev, NULL);
	if (IS_ERR(sv->dpdm))
		return dev_err_probe(dev, PTR_ERR(sv->dpdm), "Cannot get the D+/D- mux\n");
	for (i = 0; i < SVOOC_THERMS; i++) {
		sv->therm[i] = devm_iio_channel_get(dev, svooc_therm_names[i]);
		if (IS_ERR(sv->therm[i]))
			return dev_err_probe(dev, PTR_ERR(sv->therm[i]),
					     "Cannot get the %s thermistor\n",
					     svooc_therm_names[i]);
	}
	/* The Qualcomm endpoint detects the charger and owns the buck. */
	sv->usb = power_supply_get_by_name("qcom-battmgr-usb");
	if (!sv->usb)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, svooc_put_supply, sv->usb);
	if (ret)
		return ret;
	sv->battery = power_supply_get_by_name("qcom-battmgr-bat");
	if (!sv->battery)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, svooc_put_supply, sv->battery);
	if (ret)
		return ret;

	cfg.drv_data = sv;
	cfg.fwnode = dev_fwnode(dev);
	sv->psy = devm_power_supply_register(dev, &svooc_psy_desc, &cfg);
	if (IS_ERR(sv->psy))
		return dev_err_probe(dev, PTR_ERR(sv->psy), "Cannot register supply\n");
	device_init_wakeup(dev, true);
	sv->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("status", 0444, sv->debugfs, sv, &svooc_status_fops);
	sc8547_set_phy_handler(sv->main, svooc_phy_irq, sv);
	sv->notifier.notifier_call = svooc_supply_changed;
	ret = power_supply_reg_notifier(&sv->notifier);
	if (ret) {
		sc8547_set_phy_handler(sv->main, NULL, NULL);
		debugfs_remove_recursive(sv->debugfs);
		return ret;
	}
	ret = devm_add_action_or_reset(dev, svooc_cleanup, sv);
	if (ret)
		return ret;
	dev_set_drvdata(dev, sv);
	mod_delayed_work(system_highpri_wq, &sv->work, 0);
	return 0;
}

static const struct auxiliary_device_id svooc_id_table[] = {
	{ .name = "sc8547_charger.vooc-phy" },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, svooc_id_table);

static void svooc_shutdown(struct auxiliary_device *adev)
{
	struct sc8547_svooc *sv = dev_get_drvdata(&adev->dev);

	if (!sv)
		return;
	sv->stopping = true;
	cancel_delayed_work_sync(&sv->work);
	mutex_lock(&sv->lock);
	svooc_stop_locked(sv, -ESHUTDOWN);
	mutex_unlock(&sv->lock);
	cancel_delayed_work_sync(&sv->work);
	if (sv->state == SVOOC_STOPPING)
		svooc_restore(sv);
}

static struct auxiliary_driver svooc_driver = {
	.name = "svooc",
	.probe = svooc_probe,
	.shutdown = svooc_shutdown,
	.id_table = svooc_id_table,
};
module_auxiliary_driver(svooc_driver);

MODULE_DESCRIPTION("SuperVOOC charging through the SC8547D VOOC PHY");
MODULE_LICENSE("GPL");
