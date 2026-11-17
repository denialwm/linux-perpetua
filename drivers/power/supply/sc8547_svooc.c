// SPDX-License-Identifier: GPL-2.0-only
/*
 * SuperVOOC direct charging through the SC8547A/SC8547D VOOC PHY.
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
#include <linux/iio/iio.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
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
/* Noise frames tolerated before the adapter's first one. */
#define SVOOC_IGNORE_MAX	6
/* Late replies in a row tolerated after the battery curve. */
#define SVOOC_SKIP_MAX		3
/* New sessions after protocol failures, until one runs or a replug. */
#define SVOOC_RETRIES		3

#define SVOOC_TICK_MS		50
#define SVOOC_BUCK_TICK_MS	20
/* The adapter's frames come some 45 ms apart; past this it is silent. */
#define SVOOC_SAMPLE_MS		100
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

/*
 * The board's charge strategy, from the vooc-phy node, as the factory
 * driver runs it: oplus_ap_voocphy.c picks a curve by charge level and
 * temperature at the start (:1193-1326, :2997-3111), steps through it as the
 * battery voltage rises (:5549-5672), ends at a full voltage for the
 * temperature (:5778-5862), at a low battery current near full
 * (strategy/oplus_strategy_low_curr_full.c) or after a time limit
 * (:5944-5966). The checks run at its rates: voltage every 500 ms, current
 * every second (oplus_voocphy.h:366-372).
 */
#define SVOOC_BANDS_MAX		8
#define SVOOC_STEPS_MAX		16
#define SVOOC_LCF_MAX		6
#define SVOOC_FFC_ROWS_MAX	40
#define SVOOC_ADAPTERS_MAX	64
#define SVOOC_VOL_MS		500
#define SVOOC_CURR_MS		1000
/* A step ends after more than three readings above its voltage... */
#define SVOOC_STEP_COUNT	4
/* ...within 1 A of its current (TARGET_VOL_OFFSET_THR). */
#define SVOOC_STEP_MARGIN_UA	1000000
/* More than five readings above the held full voltage end a session. */
#define SVOOC_FULL_COUNT	6
/* More than six at the low-current threshold (LCF_LOW_CURR_FULL_COUNTS). */
#define SVOOC_LCF_COUNT		7
/* A temperature band changes 1 °C past its edge (VOOC_TEMP_RANGE_THD). */
#define SVOOC_TEMP_HYST		10
/* A session stops 0.5 °C below the lowest band (vooc_batt_over_low_temp). */
#define SVOOC_TEMP_LOW_MARGIN	5
/*
 * The top-up after a full session, as oplus_chg_comm.c:4602-4920 runs it
 * on each 5 s gauge update: the next step after four readings at the
 * step's battery voltage or two below its current.
 */
#define SVOOC_FFC_MS		5000
#define SVOOC_FFC_VOLT_COUNT	4
#define SVOOC_FFC_CURR_COUNT	2

struct svooc_step {
	u32 uv;
	u32 ua;
};

struct svooc_curve {
	struct svooc_step step[SVOOC_STEPS_MAX];
	unsigned int num;
};

struct svooc_lcf {
	u32 ua;
	u32 uv;
};

struct svooc_ffc_row {
	u32 fv_uv;
	u32 ua;
	u32 cutoff_uv;
	u32 cutoff_ua;
};

struct svooc_strategy {
	unsigned int soc_bands;
	u32 soc_max[SVOOC_BANDS_MAX];
	unsigned int temp_bands;
	s32 temp[SVOOC_BANDS_MAX + 1];
	struct svooc_curve curve[SVOOC_BANDS_MAX][SVOOC_BANDS_MAX];
	u32 full_uv[SVOOC_BANDS_MAX][2];
	u32 overvoltage_uv;
	u32 timeout_s;
	u32 min_ua;
	unsigned int lcf_bands;
	s32 lcf_temp[SVOOC_BANDS_MAX + 1];
	struct svooc_lcf lcf[SVOOC_BANDS_MAX][SVOOC_LCF_MAX];
	unsigned int lcf_num[SVOOC_BANDS_MAX];
	unsigned int ffc_bands;
	unsigned int ffc_steps;
	s32 ffc_temp[SVOOC_BANDS_MAX + 1];
	u32 ffc_skip_uv;
	u32 ffc_restore_uv[SVOOC_BANDS_MAX];
	struct svooc_ffc_row ffc[SVOOC_FFC_ROWS_MAX];
};

struct svooc_adapter {
	u32 first;
	u32 last;
	u32 power_uw;
};

enum svooc_ffc_state {
	SVOOC_FFC_OFF,
	SVOOC_FFC_PENDING,	/* after a full session, before the first step */
	SVOOC_FFC_RUNNING,
};

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
	SVOOC_DONE,		/* until a retry or the adapter is unplugged */
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
	/* The main pump's last sample, @sample[0]. */
	unsigned long sampled_at;
	unsigned int ticks;
	unsigned int frames;
	unsigned int irqs;
	unsigned int ignored;
	/* The slowest reply, from the interrupt thread to the TX write. */
	unsigned int worst_us;
	unsigned int quiet;
	unsigned int disagree;
	int limit_ua;
	/* The board's limit on the adapter current, at most SVOOC_MAX_UA. */
	int max_ua;
	int taper_ua;
	int target_ua;
	int applied_ua;
	int pending_ua;
	int error;
	/* The PHY's last RX byte and flags, for the stop message. */
	u8 rx;
	u8 flags;
	bool phy_on;
	bool route;
	bool buck_off;
	bool second_on;
	bool dual;
	bool change;
	unsigned int skipped;
	unsigned int retries;
	bool stopping;
	/* The board's charge strategy and adapters, or NULL. */
	struct svooc_strategy *strategy;
	/*
	 * Whether sessions follow @strategy and top up after it, from DT;
	 * debugfs may change them for the next session.
	 */
	bool use_strategy;
	bool use_ffc;
	struct svooc_adapter *adapters;
	unsigned int num_adapters;
	/* This session follows @strategy. */
	bool stock;
	/* This session ended full and may top up. */
	bool full;
	bool ffc_armed;
	/* The adapter's power, at most SVOOC_POWER_UW. */
	s64 power_uw;
	int soc_band;
	int temp_band;
	int step;
	unsigned int step_count;
	unsigned int full_count;
	unsigned int lcf_count;
	unsigned long running_at;
	unsigned long vol_at;
	unsigned long curr_at;
	enum svooc_ffc_state ffc;
	int ffc_band;
	unsigned int ffc_step;
	unsigned int ffc_volt_count;
	unsigned int ffc_curr_count;
	unsigned long ffc_at;
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
		if (ret < 0) {
			dev_dbg(sv->dev, "therm[%d] (v_chan %u) read failed: %d\n",
				i, sv->therm[i]->channel ? sv->therm[i]->channel->channel : 0,
				ret);
			return ret;
		}
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
	/* The board's curves have their own cold steps. */
	if (!sv->stock && sv->bat.temp < 210)
		ua = min(ua, 2500000);
	return ua;
}

static int svooc_vbat(struct sc8547_svooc *sv)
{
	/* Neither sense point may hide the taper from the other. */
	return max(sv->bat.uv, sv->sample[0].vbat_uv);
}

/*
 * The battery voltage the overvoltage stop judges, picked as the stock
 * gauge picks it (oplus_mms_gauge.c:3352-3380): the pump's reading, unless
 * it is missing, at or past @ov_uv, or more than 800 mV off the gauge's;
 * then the gauge's. The pump senses ahead of the board's path to the
 * cell, so near full under load it alone can pass the limit.
 */
static int svooc_ov_vbat(int gauge_uv, int pump_uv, int ov_uv)
{
	if (pump_uv <= 0 || pump_uv >= ov_uv || abs(pump_uv - gauge_uv) > 800000)
		return gauge_uv;
	return pump_uv;
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

static int svooc_ibus(struct sc8547_svooc *sv)
{
	return sv->sample[0].ibus_ua + (sv->second_on ? sv->sample[1].ibus_ua : 0);
}

static int svooc_min_ua(struct sc8547_svooc *sv)
{
	return sv->stock ? sv->strategy->min_ua : SVOOC_MIN_UA;
}

/* The band of @value between @bands + 1 rising bounds, or -1. */
static int svooc_band(const s32 *bound, unsigned int bands, int value)
{
	unsigned int i;

	for (i = 0; i < bands; i++)
		if (value >= bound[i] && value < bound[i + 1])
			return i;
	return -1;
}

static int svooc_soc_band(const struct svooc_strategy *st, int capacity)
{
	unsigned int i;

	for (i = 0; i < st->soc_bands; i++)
		if (capacity <= st->soc_max[i])
			return i;
	return -1;
}

static const struct svooc_curve *svooc_curve(struct sc8547_svooc *sv)
{
	return &sv->strategy->curve[sv->soc_band][sv->temp_band];
}

/*
 * The first step whose voltage the battery has not passed; the last step
 * only holds until full. Past them all the factory driver takes the first
 * step (:3093-3097); take the last, lowest one.
 */
static int svooc_first_step(const struct svooc_curve *c, int uv)
{
	unsigned int i;

	for (i = 0; i + 1 < c->num; i++)
		if (uv <= c->step[i].uv)
			return i;
	return c->num - 1;
}

/* The bands for the charge level and temperature, or -1 for none. */
static bool svooc_select(struct sc8547_svooc *sv, int capacity, int temp)
{
	const struct svooc_strategy *st = sv->strategy;
	int soc = svooc_soc_band(st, capacity);
	int band = svooc_band(st->temp, st->temp_bands, temp);

	if (soc < 0 || band < 0 || !st->curve[soc][band].num) {
		sv->soc_band = -1;
		sv->temp_band = -1;
		return false;
	}
	sv->soc_band = soc;
	sv->temp_band = band;
	sv->step = svooc_first_step(svooc_curve(sv), sv->bat.uv);
	sv->step_count = 0;
	return true;
}

/*
 * The board's strategy for a running session. Returns 1 when it is done,
 * with @sv->full set when the battery is full, or 0.
 */
static int svooc_stock_check(struct sc8547_svooc *sv)
{
	const struct svooc_strategy *st = sv->strategy;
	const struct svooc_curve *c;
	int temp = sv->bat.temp, uv = sv->bat.uv, vbat = svooc_vbat(sv);
	unsigned long now = jiffies;
	unsigned int i;
	int band;

	/*
	 * Leaving the temperature band starts the curve over for the present
	 * charge level and temperature, as the factory driver ends the
	 * session and starts a new one (oplus_chg_vooc.c:2290-2420); above
	 * the highest charge level band that ends fast charging.
	 */
	if (temp < st->temp[sv->temp_band] - SVOOC_TEMP_HYST ||
	    temp >= st->temp[sv->temp_band + 1] + SVOOC_TEMP_HYST) {
		if (!svooc_select(sv, sv->bat.capacity, temp))
			return 1;
		dev_info(sv->dev, "SuperVOOC curve %d/%d from step %d at %d.%d C\n",
			 sv->soc_band, sv->temp_band, sv->step, temp / 10, abs(temp) % 10);
	}
	if (st->timeout_s && time_after_eq(now, sv->started + st->timeout_s * HZ)) {
		dev_info(sv->dev, "SuperVOOC time limit\n");
		sv->full = true;
		return 1;
	}
	if (time_after_eq(now, sv->vol_at)) {
		sv->vol_at = now + msecs_to_jiffies(SVOOC_VOL_MS);
		c = svooc_curve(sv);
		if (sv->step + 1 < c->num && uv > c->step[sv->step].uv &&
		    svooc_ibus(sv) < c->step[sv->step].ua + SVOOC_STEP_MARGIN_UA) {
			if (++sv->step_count >= SVOOC_STEP_COUNT) {
				sv->step++;
				sv->step_count = 0;
			}
		} else {
			sv->step_count = 0;
		}
		/* Like the factory driver, the held count does not reset. */
		if (vbat > st->full_uv[sv->temp_band][0] ||
		    (vbat > st->full_uv[sv->temp_band][1] &&
		     ++sv->full_count >= SVOOC_FULL_COUNT)) {
			dev_info(sv->dev, "SuperVOOC full at %d uV\n", vbat);
			sv->full = true;
			return 1;
		}
	}
	if (st->lcf_bands && time_after_eq(now, sv->curr_at)) {
		sv->curr_at = now + msecs_to_jiffies(SVOOC_CURR_MS);
		band = svooc_band(st->lcf_temp, st->lcf_bands, temp);
		for (i = 0; band >= 0 && i < st->lcf_num[band]; i++)
			if (sv->bat.ua <= (int)st->lcf[band][i].ua &&
			    uv >= st->lcf[band][i].uv)
				break;
		if (band >= 0 && i < st->lcf_num[band]) {
			if (++sv->lcf_count >= SVOOC_LCF_COUNT) {
				dev_info(sv->dev, "SuperVOOC full at %d uA, %d uV\n",
					 sv->bat.ua, uv);
				sv->full = true;
				return 1;
			}
		} else {
			sv->lcf_count = 0;
		}
	}
	return 0;
}

static bool svooc_admissible(struct sc8547_svooc *sv)
{
	const struct svooc_battery *bat = &sv->bat;
	int i;

	if (bat->health != POWER_SUPPLY_HEALTH_GOOD || bat->temp >= 440 ||
	    bat->uv < 3300000)
		return false;
	/* The board's bands bound the charge level and temperature. */
	if (sv->stock) {
		if (sv->temp_band < 0 ||
		    bat->uv >= sv->strategy->full_uv[sv->temp_band][1])
			return false;
	} else if (bat->capacity >= 90 || bat->temp < 150 || bat->uv >= 4400000) {
		return false;
	}
	for (i = 0; i < SVOOC_THERMS; i++)
		if (sv->therm_mc[i] < 0 || sv->therm_mc[i] >= svooc_therm_limits[i].soft)
			return false;
	return true;
}

/* The session's current, chosen at entry from the charge level. */
static int svooc_entry_ua(struct sc8547_svooc *sv)
{
	int ua;

	if (sv->stock) {
		if (sv->temp_band < 0)
			return 0;
		ua = svooc_curve(sv)->step[sv->step].ua;
	} else {
		ua = sv->bat.capacity <= 55 ? SVOOC_MAX_UA :
		     sv->bat.capacity < 85 ? 3000000 : 2000000;
		ua = min(ua, svooc_taper_ua(sv->bat.uv));
	}
	ua = min(ua, sv->max_ua);
	ua = min(ua, svooc_thermal_ua(sv));
	/* Within the power budget at the top of the VBUS window. */
	ua = min_t(int, ua, div_s64(SVOOC_POWER_UW * 1000000,
				    2 * sv->bat.uv + 700000));
	return ua / 100000 * 100000;
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
		ret = regmap_set_bits(sv->regmap, sc8547_phy_ctrl_reg(sv->main), BIT(6));
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
		dev_warn(sv->dev, "SuperVOOC stopped in %s: %d (rx %02x flags %02x)\n",
			 svooc_state_names[sv->state], error, sv->rx, sv->flags);
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
	int ret, ua;

	switch (req->command) {
	case OPLUS_SVOOC_VBUS_OK:
		if (sv->state != SVOOC_GRANTED && sv->state != SVOOC_STARTING &&
		    sv->state != SVOOC_RUNNING)
			return -EPROTO;
		/* Read after the last reply, as the factory driver does. */
		*payload = oplus_svooc_vbus(sv->sample[0].vbus_uv, sv->bat.uv);
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

/*
 * The models a garbled exchange leaves, on which the factory driver goes
 * back to standard charging (oplus_vooc.c:1834-1845).
 */
static bool svooc_model_garbled(u8 model)
{
	return model == 0x0f || model == 0x1f || model == 0x3f || model == 0x7f;
}

/*
 * With the board's adapter table, only a known adapter with power may raise
 * VBUS, as the factory driver looks the model up (oplus_chg_vooc.c:467-512,
 * :601-649). The session keeps within the adapter's power. A garbled model
 * never raises it, with a table or without.
 */
static int svooc_check_adapter(struct sc8547_svooc *sv)
{
	u8 model = sv->proto.model;
	unsigned int i;

	if (svooc_model_garbled(model)) {
		dev_warn(sv->dev, "SuperVOOC adapter model %02x garbled\n", model);
		return -EPROTONOSUPPORT;
	}
	if (!sv->num_adapters)
		return 0;
	for (i = 0; i < sv->num_adapters; i++) {
		if (model < sv->adapters[i].first || model > sv->adapters[i].last)
			continue;
		if (!sv->adapters[i].power_uw)
			break;
		sv->power_uw = min_t(s64, SVOOC_POWER_UW, sv->adapters[i].power_uw);
		sv->limit_ua = min_t(int, sv->limit_ua,
				     div_s64(sv->power_uw * 1000000,
					     2 * sv->bat.uv + 700000) / 100000 * 100000);
		if (sv->limit_ua < svooc_min_ua(sv))
			break;
		return 0;
	}
	dev_warn(sv->dev, "SuperVOOC adapter %02x not supported\n", model);
	return -EPROTONOSUPPORT;
}

static int svooc_receive(struct sc8547_svooc *sv, u8 rx, u8 flags)
{
	struct oplus_svooc_request req;
	u8 payload;
	int ret;

	/*
	 * Until the adapter's first frame, ignore what the handshake leaves
	 * on the line, as the factory driver does.
	 */
	if (!sv->frames && (!(flags & SC8547_PHY_RX_DONE) || (flags & SC8547_PHY_ERRORS) ||
			    oplus_svooc_decode(rx, &req)))
		return (flags & SC8547_PHY_RX_DONE) && ++sv->ignored > SVOOC_IGNORE_MAX ?
		       -EPROTO : 0;
	if (flags & SC8547_PHY_ERRORS) {
		/*
		 * After the battery curve the factory driver lets a late reply
		 * pass and the adapter asks again. A lost current reply, or one
		 * during identification, is fatal.
		 */
		if ((flags & SC8547_PHY_ERRORS) == SC8547_PHY_TX_FAILED &&
		    sv->proto.curve_sent && !sv->pending_ua &&
		    ++sv->skipped <= SVOOC_SKIP_MAX)
			return 0;
		return -EPROTO;
	}
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
	if (!(flags & SC8547_PHY_RX_DONE))
		return 0;

	sv->skipped = 0;
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
			ret = svooc_check_adapter(sv);
			if (ret)
				return ret;
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

/*
 * The pump may stretch an ADC read for 4 ms, longer than the adapter waits
 * for a reply. Read it in the quiet gap after a reply, as the factory
 * driver does; only a silent adapter leaves it to the worker.
 */
static int svooc_sample_main(struct sc8547_svooc *sv)
{
	struct sc8547_sample s;
	int ret;

	ret = sc8547_sample(sv->main, &s);
	if (ret)
		return ret;
	sv->sample[0] = s;
	sv->sampled_at = jiffies;
	return 0;
}

static void svooc_phy_irq(void *data)
{
	struct sc8547_svooc *sv = data;
	ktime_t start = ktime_get();
	u8 rx[2];
	int ret;

	mutex_lock(&sv->lock);
	if (sv->phy_on) {
		ret = regmap_bulk_read(sv->regmap, SC8547_RX, rx, sizeof(rx));
		if (!ret) {
			sv->rx = rx[0];
			sv->flags = rx[1];
			sv->irqs++;
			ret = svooc_receive(sv, rx[0], rx[1]);
			if (!ret)
				sv->worst_us = max_t(unsigned int, sv->worst_us,
						     ktime_us_delta(ktime_get(), start));
			/* Every clean frame got a reply. */
			if (!ret && sv->frames && (rx[1] & SC8547_PHY_RX_DONE) &&
			    !(rx[1] & SC8547_PHY_ERRORS))
				ret = svooc_sample_main(sv);
		}
		if (ret)
			svooc_stop_locked(sv, ret);
	}
	mutex_unlock(&sv->lock);
}

static int svooc_set_battery(struct sc8547_svooc *sv,
			     enum power_supply_property prop, int value)
{
	union power_supply_propval val = { .intval = value };

	return power_supply_set_property(sv->battery, prop, &val);
}

static int svooc_ffc_apply(struct sc8547_svooc *sv)
{
	const struct svooc_strategy *st = sv->strategy;
	const struct svooc_ffc_row *row = &st->ffc[sv->ffc_step * st->ffc_bands +
						   sv->ffc_band];
	int ret;

	ret = svooc_set_battery(sv, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
				row->fv_uv);
	if (!ret)
		ret = svooc_set_battery(sv, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
					row->ua);
	return ret;
}

/* Give the standard charger its own current and the board's voltage back. */
static void svooc_ffc_stop(struct sc8547_svooc *sv)
{
	const struct svooc_strategy *st = sv->strategy;

	if (sv->ffc == SVOOC_FFC_RUNNING) {
		if (svooc_set_battery(sv, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, -1) ||
		    svooc_set_battery(sv, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
				      st->ffc_restore_uv[sv->ffc_band]))
			dev_err(sv->dev, "cannot end the top-up charge\n");
		else
			dev_info(sv->dev, "top-up charge ended at step %u\n", sv->ffc_step);
	}
	sv->ffc = SVOOC_FFC_OFF;
}

/*
 * After a full session the standard charger tops the battery up at a
 * raised voltage limit and a low current, in steps, within a temperature
 * window (oplus_chg_comm.c:4602-4700, :4783-4920). Runs in the worker.
 */
static void svooc_ffc_tick(struct sc8547_svooc *sv)
{
	const struct svooc_strategy *st = sv->strategy;
	const struct svooc_ffc_row *row;
	struct svooc_battery bat;
	unsigned int step;
	int band;

	if (time_before(jiffies, sv->ffc_at)) {
		mod_delayed_work(system_highpri_wq, &sv->work, sv->ffc_at - jiffies);
		return;
	}
	sv->ffc_at = jiffies + msecs_to_jiffies(SVOOC_FFC_MS);
	if (svooc_read_battery(sv, &bat) || bat.health != POWER_SUPPLY_HEALTH_GOOD)
		goto stop;
	band = svooc_band(st->ffc_temp, st->ffc_bands, bat.temp);
	if (band < 0)
		goto stop;
	if (sv->ffc == SVOOC_FFC_PENDING) {
		/* A battery already full near its limit skips it (:4609-4635). */
		if (st->ffc_skip_uv && bat.uv >= st->ffc_skip_uv && bat.capacity >= 100)
			goto stop;
		sv->ffc_band = band;
		sv->ffc_step = 0;
		sv->ffc_volt_count = 0;
		sv->ffc_curr_count = 0;
		sv->ffc = SVOOC_FFC_RUNNING;
		if (svooc_ffc_apply(sv))
			goto stop;
		dev_info(sv->dev, "top-up charge at %d.%d C\n", bat.temp / 10,
			 abs(bat.temp) % 10);
		goto next;
	}
	if (band != sv->ffc_band) {
		sv->ffc_band = band;
		if (svooc_ffc_apply(sv))
			goto stop;
	}
	row = &st->ffc[sv->ffc_step * st->ffc_bands + band];
	step = sv->ffc_step;
	/* As the factory driver counts: the voltage count does not reset. */
	if (bat.uv >= row->cutoff_uv)
		sv->ffc_volt_count++;
	if (sv->ffc_volt_count >= SVOOC_FFC_VOLT_COUNT) {
		step++;
	} else {
		if (abs(bat.ua) < row->cutoff_ua)
			sv->ffc_curr_count++;
		else
			sv->ffc_curr_count = 0;
		if (sv->ffc_curr_count >= SVOOC_FFC_CURR_COUNT)
			step++;
	}
	if (step != sv->ffc_step) {
		sv->ffc_volt_count = 0;
		sv->ffc_curr_count = 0;
		if (step >= st->ffc_steps)
			goto stop;
		sv->ffc_step = step;
		if (svooc_ffc_apply(sv))
			goto stop;
	}
next:
	mod_delayed_work(system_highpri_wq, &sv->work, msecs_to_jiffies(SVOOC_FFC_MS));
	return;
stop:
	svooc_ffc_stop(sv);
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
	if (svooc_read_battery(sv, &bat) || svooc_read_therms(sv, therm)) {
		/* Transient ADC failure: keep retrying instead of wedging. */
		dev_dbg(sv->dev, "start: battery/therm read failed, retrying\n");
		mod_delayed_work(system_highpri_wq, &sv->work,
				 msecs_to_jiffies(SVOOC_RETRY_MS));
		return;
	}

	if (sv->ffc != SVOOC_FFC_OFF)
		svooc_ffc_stop(sv);

	mutex_lock(&sv->lock);
	sv->bat = bat;
	memcpy(sv->therm_mc, therm, sizeof(therm));
	memset(sv->sample, 0, sizeof(sv->sample));
	/* A session keeps the rules it started with. */
	sv->stock = sv->strategy && READ_ONCE(sv->use_strategy);
	sv->ffc_armed = sv->stock && sv->strategy->ffc_steps &&
			READ_ONCE(sv->use_ffc);
	sv->full = false;
	sv->full_count = 0;
	sv->lcf_count = 0;
	sv->power_uw = SVOOC_POWER_UW;
	if (sv->stock)
		svooc_select(sv, bat.capacity, bat.temp);
	sv->limit_ua = svooc_entry_ua(sv);
	if (!sv->second)
		sv->limit_ua = min(sv->limit_ua, SVOOC_PUMP_MAX_UA);
	mutex_unlock(&sv->lock);
	if (!svooc_admissible(sv) || sv->limit_ua < svooc_min_ua(sv)) {
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
	sv->irqs = 0;
	sv->ignored = 0;
	sv->worst_us = 0;
	sv->rx = 0;
	sv->flags = 0;
	sv->ticks = 0;
	sv->taper_ua = sv->limit_ua;
	sv->target_ua = 0;
	sv->change = false;
	sv->skipped = 0;
	sv->disagree = 0;
	sv->discharge_since = 0;
	sv->started = jiffies;
	sv->vol_at = jiffies;
	sv->curr_at = jiffies;
	sv->last_rx = jiffies;
	sv->sampled_at = jiffies - msecs_to_jiffies(SVOOC_SAMPLE_MS);
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
	if (sv->stock)
		sv->taper_ua = svooc_curve(sv)->step[sv->step].ua;
	else
		sv->taper_ua = min(sv->taper_ua, svooc_taper_ua(svooc_vbat(sv)));
	ua = min(sv->limit_ua, sv->taper_ua);
	ua = min(ua, svooc_thermal_ua(sv));
	ua = min_t(int, ua, div_s64(sv->power_uw * 1000000, vbus) - 100000);
	if (power > sv->power_uw * 1000000)
		ua = min_t(int, ua, sv->applied_ua - 100000 -
			   div_s64(power - (sv->power_uw - 500000) * 1000000, vbus));
	ua = ua / 100000 * 100000;
	if (ua < svooc_min_ua(sv))
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

	if (sv->mode != POWER_SUPPLY_CHARGE_TYPE_FAST ||
	    (!sv->stock && sv->bat.capacity >= 90))
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
	if (sv->bat.health != POWER_SUPPLY_HEALTH_GOOD || sv->bat.temp >= 440 ||
	    (!sv->stock && vbat >= SVOOC_VBAT_MAX_UV))
		return -ERANGE;
	if (sv->stock && ((sv->strategy->overvoltage_uv &&
			   svooc_ov_vbat(sv->bat.uv, sv->sample[0].vbat_uv,
					 sv->strategy->overvoltage_uv) >=
			   sv->strategy->overvoltage_uv) ||
			  sv->bat.temp < sv->strategy->temp[0] - SVOOC_TEMP_LOW_MARGIN))
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
			sv->retries = 0;
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
	if (sv->stock && sv->state == SVOOC_RUNNING) {
		i = svooc_stock_check(sv);
		if (i)
			return i;
	}
	return svooc_regulate(sv);
}

static void svooc_tick(struct sc8547_svooc *sv)
{
	struct sc8547_sample sample[2] = {};
	struct svooc_battery bat;
	int therm[SVOOC_THERMS], usb_ua = 0, usb_uv = 0, online = 0;
	bool read_main, read_bat, read_therm, read_usb, suspend = false, second = false, changed;
	enum svooc_state state;
	int ret;

	mutex_lock(&sv->lock);
	state = sv->state;
	sv->ticks++;
	read_bat = !(sv->ticks % 4);
	read_therm = !(sv->ticks % 10);
	/* While the adapter talks, the PHY interrupt samples the main pump. */
	read_main = time_after(jiffies, sv->sampled_at + msecs_to_jiffies(SVOOC_SAMPLE_MS));
	mutex_unlock(&sv->lock);
	/* Once VBUS rises the PMIC reports USB offline; the pumps watch it. */
	read_usb = state < SVOOC_GRANTED && (read_bat || state == SVOOC_SUSPEND);

	/* Slow reads outside the lock: the PHY must answer within milliseconds. */
	ret = read_main ? sc8547_sample(sv->main, &sample[0]) : 0;
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
	if (read_main) {
		sv->sample[0] = sample[0];
		sv->sampled_at = jiffies;
	}
	sv->sample[1] = sample[1];
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
	if (sv->full && sv->ffc_armed && !sv->error && !sv->stopping) {
		sv->ffc = SVOOC_FFC_PENDING;
		sv->ffc_at = jiffies;
	}
	mutex_unlock(&sv->lock);
	pm_relax(sv->dev);
	power_supply_changed(sv->psy);
	/* The adapter may already be gone. */
	if (!sv->stopping)
		mod_delayed_work(system_highpri_wq, &sv->work, 0);
}

/*
 * A replug starts over. Like the factory driver, retry after a protocol
 * failure or a timeout while the adapter stays plugged in; it drops back
 * to 5 V and answers a new handshake. A charger that never sent a frame
 * is no SuperVOOC adapter, and each attempt takes its D+/D- lines.
 */
static void svooc_done(struct sc8547_svooc *sv)
{
	unsigned long retry_at;
	bool restart = false;
	int online = 0;

	svooc_get(sv->usb, POWER_SUPPLY_PROP_ONLINE, &online);
	if (sv->ffc != SVOOC_FFC_OFF) {
		if (online && READ_ONCE(sv->mode) == POWER_SUPPLY_CHARGE_TYPE_FAST &&
		    !sv->stopping) {
			svooc_ffc_tick(sv);
			if (sv->ffc != SVOOC_FFC_OFF)
				return;
		} else {
			svooc_ffc_stop(sv);
		}
	}
	mutex_lock(&sv->lock);
	retry_at = sv->stopped_at + msecs_to_jiffies(SVOOC_RETRY_MS);
	if (!online) {
		sv->state = SVOOC_IDLE;
		sv->error = 0;
		sv->retries = 0;
	} else if ((sv->error == -EPROTO || sv->error == -ETIMEDOUT) && sv->frames &&
		   sv->retries < SVOOC_RETRIES && sv->mode == POWER_SUPPLY_CHARGE_TYPE_FAST) {
		if (time_before(jiffies, retry_at)) {
			mod_delayed_work(system_highpri_wq, &sv->work, retry_at - jiffies);
		} else {
			sv->state = SVOOC_IDLE;
			sv->error = 0;
			sv->retries++;
			restart = true;
		}
	}
	mutex_unlock(&sv->lock);
	if (restart)
		mod_delayed_work(system_highpri_wq, &sv->work, 0);
}

static void svooc_work(struct work_struct *work)
{
	struct sc8547_svooc *sv = container_of(work, struct sc8547_svooc, work.work);
	enum svooc_state state;

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
		svooc_done(sv);
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
	seq_printf(s, "state %s error %d frames %u irqs %u rx %02x flags %02x model %02x\n",
		   svooc_state_names[sv->state], sv->error, sv->frames, sv->irqs,
		   sv->rx, sv->flags, sv->proto.model);
	seq_printf(s, "worst reply %u us\n", sv->worst_us);
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
	if (sv->strategy)
		seq_printf(s, "strategy %s curve %d/%d step %d full %d/%u/%u power %lld uW top-up %d step %u\n",
			   sv->stock ? "on" : "off", sv->soc_band, sv->temp_band,
			   sv->step, sv->full, sv->full_count, sv->lcf_count,
			   sv->power_uw, sv->ffc, sv->ffc_step);
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
	svooc_ffc_stop(sv);
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

/* Rising temperature bounds, in tenths of a degree, for up to 8 bands. */
static int svooc_read_bands(struct fwnode_handle *node, const char *name,
			    s32 *bound, unsigned int *bands)
{
	int n, i, ret;

	n = fwnode_property_count_u32(node, name);
	if (n < 2 || n > SVOOC_BANDS_MAX + 1)
		return -EINVAL;
	ret = fwnode_property_read_u32_array(node, name, (u32 *)bound, n);
	if (ret)
		return ret;
	for (i = 1; i < n; i++)
		if (bound[i] <= bound[i - 1])
			return -EINVAL;
	*bands = n - 1;
	return 0;
}

/* A property of up to @max rows of @cols cells; returns the rows. */
static int svooc_read_rows(struct fwnode_handle *node, const char *name,
			   u32 *data, int cols, int max)
{
	int n, ret;

	n = fwnode_property_count_u32(node, name);
	if (n <= 0 || n % cols || n / cols > max)
		return n < 0 ? n : -EINVAL;
	ret = fwnode_property_read_u32_array(node, name, data, n);
	return ret ? ret : n / cols;
}

static bool svooc_uv_valid(u32 uv)
{
	return uv >= 3500000 && uv <= 4700000;
}

static int svooc_get_curves(struct device *dev, struct svooc_strategy *st)
{
	struct fwnode_handle *curves, *soc;
	char name[24];
	unsigned int i, j, k;
	int n, ret = 0;

	curves = device_get_named_child_node(dev, "charge-curves");
	if (!curves)
		return -EINVAL;
	st->min_ua = SVOOC_MAX_UA;
	for (i = 0; i < st->soc_bands && !ret; i++) {
		snprintf(name, sizeof(name), "soc-band-%u", i);
		soc = fwnode_get_named_child_node(curves, name);
		if (!soc) {
			ret = -EINVAL;
			break;
		}
		for (j = 0; j < st->temp_bands; j++) {
			struct svooc_curve *c = &st->curve[i][j];

			snprintf(name, sizeof(name), "temp-band-%u", j);
			/* A band without a curve does not fast charge. */
			if (!fwnode_property_present(soc, name))
				continue;
			n = svooc_read_rows(soc, name, (u32 *)c->step, 2, SVOOC_STEPS_MAX);
			if (n < 0) {
				ret = n;
				break;
			}
			c->num = n;
			for (k = 0; k < c->num; k++) {
				u8 code;

				if (!svooc_uv_valid(c->step[k].uv) ||
				    (k && c->step[k].uv < c->step[k - 1].uv) ||
				    oplus_svooc_current(c->step[k].ua, &code)) {
					ret = -EINVAL;
					break;
				}
				st->min_ua = min(st->min_ua, c->step[k].ua);
			}
			if (ret)
				break;
		}
		fwnode_handle_put(soc);
	}
	fwnode_handle_put(curves);
	return ret;
}

static int svooc_get_lcf(struct device *dev, struct svooc_strategy *st)
{
	struct fwnode_handle *node;
	char name[24];
	unsigned int i, k;
	int n, ret;

	node = device_get_named_child_node(dev, "low-current-full");
	if (!node)
		return 0;
	ret = svooc_read_bands(node, "temp-bands", st->lcf_temp, &st->lcf_bands);
	for (i = 0; !ret && i < st->lcf_bands; i++) {
		snprintf(name, sizeof(name), "temp-band-%u", i);
		if (!fwnode_property_present(node, name))
			continue;
		n = svooc_read_rows(node, name, (u32 *)st->lcf[i], 2, SVOOC_LCF_MAX);
		if (n < 0) {
			ret = n;
			break;
		}
		st->lcf_num[i] = n;
		for (k = 0; k < st->lcf_num[i]; k++)
			if (!svooc_uv_valid(st->lcf[i][k].uv) ||
			    st->lcf[i][k].ua > SVOOC_MAX_UA * 2)
				ret = -EINVAL;
	}
	fwnode_handle_put(node);
	return ret;
}

static int svooc_get_ffc(struct device *dev, struct svooc_strategy *st)
{
	struct fwnode_handle *node;
	unsigned int i;
	int n, ret;

	node = device_get_named_child_node(dev, "fast-full-charge");
	if (!node)
		return 0;
	ret = svooc_read_bands(node, "temp-bands", st->ffc_temp, &st->ffc_bands);
	if (!ret && fwnode_property_count_u32(node, "restore-microvolt") != st->ffc_bands)
		ret = -EINVAL;
	if (!ret)
		ret = fwnode_property_read_u32_array(node, "restore-microvolt",
						     st->ffc_restore_uv, st->ffc_bands);
	if (!ret) {
		n = svooc_read_rows(node, "steps", (u32 *)st->ffc, 4, SVOOC_FFC_ROWS_MAX);
		if (n < 0 || n % st->ffc_bands)
			ret = n < 0 ? n : -EINVAL;
		else
			st->ffc_steps = n / st->ffc_bands;
	}
	fwnode_property_read_u32(node, "skip-microvolt", &st->ffc_skip_uv);
	for (i = 0; !ret && i < st->ffc_bands; i++)
		if (!svooc_uv_valid(st->ffc_restore_uv[i]))
			ret = -EINVAL;
	for (i = 0; !ret && i < st->ffc_steps * st->ffc_bands; i++)
		if (!svooc_uv_valid(st->ffc[i].fv_uv) || !svooc_uv_valid(st->ffc[i].cutoff_uv) ||
		    !st->ffc[i].ua || st->ffc[i].ua > SVOOC_MAX_UA)
			ret = -EINVAL;
	if (ret)
		st->ffc_steps = 0;
	fwnode_handle_put(node);
	return ret;
}

/*
 * The board's charge strategy and adapters. A broken table leaves the
 * driver's own rules in place rather than failing the probe.
 */
static void svooc_get_strategy(struct sc8547_svooc *sv)
{
	struct device *dev = sv->dev;
	struct svooc_strategy *st;
	int n, i, ret;

	n = device_property_count_u32(dev, "oneplus,svooc-adapters");
	if (n > 0) {
		sv->adapters = devm_kcalloc(dev, n / 3, sizeof(*sv->adapters), GFP_KERNEL);
		if (sv->adapters && n % 3 == 0 && n / 3 <= SVOOC_ADAPTERS_MAX &&
		    !device_property_read_u32_array(dev, "oneplus,svooc-adapters",
						    (u32 *)sv->adapters, n))
			sv->num_adapters = n / 3;
		else
			dev_warn(dev, "invalid adapter table, ignored\n");
	}

	if (!device_property_present(dev, "oneplus,svooc-soc-bands"))
		return;
	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return;
	n = device_property_count_u32(dev, "oneplus,svooc-soc-bands");
	ret = n > 0 && n <= SVOOC_BANDS_MAX ? 0 : -EINVAL;
	if (!ret) {
		st->soc_bands = n;
		ret = device_property_read_u32_array(dev, "oneplus,svooc-soc-bands",
						     st->soc_max, n);
	}
	for (i = 1; !ret && i < st->soc_bands; i++)
		if (st->soc_max[i] <= st->soc_max[i - 1])
			ret = -EINVAL;
	if (!ret)
		ret = svooc_read_bands(dev_fwnode(dev), "oneplus,svooc-temp-bands",
				       st->temp, &st->temp_bands);
	if (!ret && device_property_count_u32(dev, "oneplus,svooc-full-microvolt") !=
		    2 * st->temp_bands)
		ret = -EINVAL;
	if (!ret)
		ret = device_property_read_u32_array(dev, "oneplus,svooc-full-microvolt",
						     (u32 *)st->full_uv, 2 * st->temp_bands);
	for (i = 0; !ret && i < st->temp_bands; i++)
		if (!svooc_uv_valid(st->full_uv[i][0]) || !svooc_uv_valid(st->full_uv[i][1]))
			ret = -EINVAL;
	if (!ret)
		ret = svooc_get_curves(dev, st);
	if (!ret)
		ret = svooc_get_lcf(dev, st);
	if (ret) {
		dev_warn(dev, "invalid charge strategy, ignored: %d\n", ret);
		devm_kfree(dev, st);
		return;
	}
	device_property_read_u32(dev, "oneplus,svooc-overvoltage-microvolt",
				 &st->overvoltage_uv);
	device_property_read_u32(dev, "oneplus,svooc-timeout-sec", &st->timeout_s);
	if (svooc_get_ffc(dev, st))
		dev_warn(dev, "invalid top-up charge, ignored\n");
	sv->strategy = st;
	sv->use_strategy = device_property_read_bool(dev, "oneplus,svooc-stock-strategy");
	sv->use_ffc = device_property_read_bool(dev, "oneplus,svooc-fast-full-charge");
}

static int svooc_probe(struct auxiliary_device *adev,
		       const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct power_supply_config cfg = {};
	struct fwnode_handle *node;
	struct sc8547_svooc *sv;
	u32 max_ua;
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
	svooc_get_strategy(sv);
	sv->soc_band = -1;
	sv->temp_band = -1;
	sv->power_uw = SVOOC_POWER_UW;
	max_ua = SVOOC_MAX_UA;
	device_property_read_u32(dev, "oneplus,svooc-max-microamp", &max_ua);
	if (max_ua < SVOOC_MIN_UA || max_ua > SVOOC_MAX_UA)
		return dev_err_probe(dev, -EINVAL, "Invalid current limit %u\n", max_ua);
	sv->max_ua = max_ua;

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
	if (sv->strategy) {
		debugfs_create_bool("stock_strategy", 0644, sv->debugfs, &sv->use_strategy);
		debugfs_create_bool("fast_full_charge", 0644, sv->debugfs, &sv->use_ffc);
	}
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
	svooc_ffc_stop(sv);
}

static struct auxiliary_driver svooc_driver = {
	.name = "svooc",
	.probe = svooc_probe,
	.shutdown = svooc_shutdown,
	.id_table = svooc_id_table,
};
module_auxiliary_driver(svooc_driver);

MODULE_DESCRIPTION("SuperVOOC charging through the SC8547A/SC8547D VOOC PHY");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
