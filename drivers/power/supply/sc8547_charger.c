// SPDX-License-Identifier: GPL-2.0-only
/*
 * Southchip SC8547D 2:1 switched-capacitor charge pump.
 *
 * Register sequences adapted from the Oplus SC8547D driver,
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 * The pump never switches on its own: a direct charging driver prepares it,
 * enables it once the adapter voltage is in range, and must sample it at least
 * once per watchdog period while it runs. A fault stops it until the next
 * prepare.
 */
#include <linux/auxiliary_bus.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power/sc8547.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define SC8547_VBAT_OVP		0x00
#define SC8547_VBAT_OCP		0x01
#define SC8547_VAC_OVP		0x02
#define SC8547_VBUS_OVP		0x04
#define SC8547_IBUS_OCP		0x05
#define SC8547_STATUS		0x06
#define SC8547_CONTROL		0x07
#define SC8547_MODE		0x09
#define SC8547_PMID2OUT		0x0d
#define SC8547_PROTECTION	0x0e
#define SC8547_IRQ_MASK		0x10
#define SC8547_ADC_CTRL		0x11
#define SC8547_ADC_IBUS		0x13
#define SC8547_ADC_VBUS		0x15
#define SC8547_ADC_TDIE		0x1f
#define SC8547_VOOC_CTRL	0x2b
#define SC8547_DET_CTRL		0x33
#define SC8547_PHY_CTRL		0x34
#define SC8547_DEVICE_ID	0x36
#define SC8547_OTG		0x3b
#define SC8547_MAX_REGISTER	0x3c

#define SC8547D_ID		0x49

#define SC8547_STATUS_TSHUT_FLAG	BIT(7)
#define SC8547_STATUS_TSHUT		BIT(6)
#define SC8547_STATUS_VBUS_LOW		BIT(5)
#define SC8547_STATUS_VBUS_HIGH		BIT(4)
#define SC8547_STATUS_SS_TIMEOUT	BIT(3)
#define SC8547_STATUS_SWITCHING		BIT(2)
#define SC8547_STATUS_PIN_DIAG		BIT(0)
#define SC8547_CONTROL_EN		BIT(7)
#define SC8547_CONTROL_RESET		BIT(6)
#define SC8547_CONTROL_600KHZ		BIT(0)
#define SC8547_MODE_UCP_RISE_MASK	BIT(4)
#define SC8547_MODE_WD_TIMEOUT		BIT(3)
#define SC8547_MODE_WATCHDOG		GENMASK(2, 0)
#define SC8547_WATCHDOG_5S		4
#define SC8547_PROTECTION_VOUT_OVP	BIT(7)
#define SC8547_PROTECTION_VBAT_OVP	BIT(6)
#define SC8547_PROTECTION_IBAT_OCP	BIT(5)
#define SC8547_PROTECTION_VBUS_OVP	BIT(4)
#define SC8547_PROTECTION_IBUS_OCP	BIT(3)
#define SC8547_ADC_EN			BIT(7)
#define SC8547_ADC_FREEZE		BIT(5)
#define SC8547_OTG_EN			BIT(2)
#define SC8547_IBUS_UCP_50MS		BIT(5)

#define SC8547_VBAT_OVP_MIN_UV		3500000
#define SC8547_VBAT_OVP_STEP_UV		25000
#define SC8547_VBUS_OVP_MIN_UV		6000000
#define SC8547_VBUS_OVP_STEP_UV		50000
#define SC8547_IBUS_OCP_MIN_UA		1200000
#define SC8547_IBUS_OCP_STEP_UA		300000

struct sc8547 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	/* Serializes the switching state, fault latch and each ADC snapshot. */
	struct mutex lock;
	/* Apart from @lock, so a reply never waits for an ADC read. */
	struct mutex phy_lock;
	void (*phy_handler)(void *data);
	void *phy_data;
	/* The last sample, for the properties: a read there could delay a reply. */
	struct sc8547_sample last;
	int health;
	u8 vbat_ovp;
	u8 vbus_ovp;
	u8 ibus_ocp;
	u8 control;
	bool always_otg;
	bool prepared;
	bool enabled;
};

static const struct reg_sequence sc8547_defaults[] = {
	{ SC8547_ADC_CTRL, 0 },
	{ SC8547_VAC_OVP, 0x01 },
	{ SC8547_VBAT_OCP, 0xbf },
	{ SC8547_VOOC_CTRL, 0 },
	{ SC8547_PHY_CTRL, 0x60 },
	{ SC8547_IRQ_MASK, 0x02 },
	{ SC8547_MODE, SC8547_MODE_UCP_RISE_MASK },
};

static int sc8547_write_limits(struct sc8547 *cp)
{
	int ret;

	ret = regmap_write(cp->regmap, SC8547_VBUS_OVP, cp->vbus_ovp);
	if (!ret)
		ret = regmap_write(cp->regmap, SC8547_VBAT_OVP, cp->vbat_ovp);
	if (!ret)
		ret = regmap_write(cp->regmap, SC8547_IBUS_OCP,
				   SC8547_IBUS_UCP_50MS | cp->ibus_ocp);
	return ret;
}

/* Register reset, then the factory protection setup with switching off. */
static int sc8547_init(struct sc8547 *cp)
{
	int ret;

	ret = regmap_write(cp->regmap, SC8547_CONTROL,
			   cp->control | SC8547_CONTROL_RESET);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	ret = regmap_multi_reg_write(cp->regmap, sc8547_defaults,
				     ARRAY_SIZE(sc8547_defaults));
	if (!ret)
		ret = sc8547_write_limits(cp);
	if (!ret)
		ret = regmap_write(cp->regmap, SC8547_CONTROL, cp->control);
	if (!ret && cp->always_otg)
		ret = regmap_set_bits(cp->regmap, SC8547_OTG, SC8547_OTG_EN);
	return ret;
}

static int sc8547_stop_locked(struct sc8547 *cp)
{
	unsigned int control, status;
	int ret;

	ret = regmap_write(cp->regmap, SC8547_CONTROL, cp->control);
	if (!ret)
		ret = regmap_read(cp->regmap, SC8547_CONTROL, &control);
	if (!ret && (control & SC8547_CONTROL_EN))
		ret = -EIO;
	if (ret)
		return ret;
	/* The enable bit clears at once; switching stops shortly after. */
	usleep_range(1000, 2000);
	ret = regmap_read(cp->regmap, SC8547_STATUS, &status);
	if (!ret && (status & SC8547_STATUS_SWITCHING))
		ret = -EIO;
	if (ret)
		return ret;
	/* Only a pump verified off may lose its watchdog. */
	cp->enabled = false;
	return regmap_update_bits(cp->regmap, SC8547_MODE, SC8547_MODE_WATCHDOG, 0);
}

static int sc8547_fault_health(unsigned int status, unsigned int protection,
			       unsigned int mode)
{
	if (status & (SC8547_STATUS_TSHUT_FLAG | SC8547_STATUS_TSHUT))
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (protection & (SC8547_PROTECTION_VOUT_OVP | SC8547_PROTECTION_VBAT_OVP |
			  SC8547_PROTECTION_VBUS_OVP))
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	if (protection & (SC8547_PROTECTION_IBAT_OCP | SC8547_PROTECTION_IBUS_OCP))
		return POWER_SUPPLY_HEALTH_OVERCURRENT;
	if (mode & SC8547_MODE_WD_TIMEOUT)
		return POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
	if (status & SC8547_STATUS_SS_TIMEOUT)
		return POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
	if (status & SC8547_STATUS_PIN_DIAG)
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	/* An input out of the 2:1 window is normal until the pump switches. */
	if ((status & SC8547_STATUS_SWITCHING) &&
	    (status & (SC8547_STATUS_VBUS_LOW | SC8547_STATUS_VBUS_HIGH)))
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	return POWER_SUPPLY_HEALTH_GOOD;
}

/*
 * Reading the status clears its flags, so every reader latches what it saw.
 * A fault stops a running pump.
 */
static int sc8547_check_locked(struct sc8547 *cp, bool *switching)
{
	u8 regs[SC8547_PROTECTION + 1 - SC8547_STATUS];
	unsigned int status, protection, mode;
	int health, ret;

	/*
	 * One transfer, so the bus stays free for VOOC PHY replies. Reading
	 * the control register in it services the watchdog.
	 */
	ret = regmap_bulk_read(cp->regmap, SC8547_STATUS, regs, sizeof(regs));
	if (ret)
		return ret;
	status = regs[0];
	mode = regs[SC8547_MODE - SC8547_STATUS];
	protection = regs[SC8547_PROTECTION - SC8547_STATUS];
	if (switching)
		*switching = status & SC8547_STATUS_SWITCHING;
	health = sc8547_fault_health(status, protection, mode);
	if (health == POWER_SUPPLY_HEALTH_GOOD)
		return cp->health == POWER_SUPPLY_HEALTH_GOOD ? 0 : -EIO;
	if (cp->health == POWER_SUPPLY_HEALTH_GOOD) {
		dev_err(cp->dev, "fault: status %02x protection %02x mode %02x\n",
			status, protection, mode);
		cp->health = health;
	}
	if (cp->enabled && sc8547_stop_locked(cp))
		dev_crit(cp->dev, "cannot stop the pump after a fault\n");
	cp->prepared = false;
	return -EIO;
}

static int sc8547_adc(const u8 *adc, int n, u8 high_mask)
{
	return ((adc[2 * n] & high_mask) << 8) | adc[2 * n + 1];
}

static int sc8547_read_adc_locked(struct sc8547 *cp, struct sc8547_sample *s)
{
	u8 adc[SC8547_ADC_TDIE + 2 - SC8547_ADC_IBUS];
	int ret, thaw;

	/* A prepared pump's ADC control holds only the enable bit. */
	ret = regmap_write(cp->regmap, SC8547_ADC_CTRL, SC8547_ADC_EN | SC8547_ADC_FREEZE);
	if (ret)
		return ret;
	ret = regmap_bulk_read(cp->regmap, SC8547_ADC_IBUS, adc, sizeof(adc));
	thaw = regmap_write(cp->regmap, SC8547_ADC_CTRL, SC8547_ADC_EN);
	if (ret)
		return ret;
	if (thaw)
		return thaw;
	s->ibus_ua = sc8547_adc(adc, 0, 0x0f) * 1875;
	s->vbus_uv = sc8547_adc(adc, 1, 0x0f) * 3750;
	s->vac_uv = sc8547_adc(adc, 2, 0x0f) * 5000;
	s->vout_uv = sc8547_adc(adc, 3, 0x0f) * 1250;
	s->vbat_uv = sc8547_adc(adc, 4, 0x0f) * 1250;
	s->temp = sc8547_adc(adc, 6, 0x01) * 5;
	return 0;
}

static void sc8547_put_device(void *dev)
{
	put_device(dev);
}

static void sc8547_clear_clientdata(void *client)
{
	i2c_set_clientdata(client, NULL);
}

/**
 * sc8547_get() - get a charge pump for a direct charging driver
 * @consumer: the driver's device, which unbinds before the pump does
 * @fwnode: the pump's node
 *
 * Return: the pump, or an ERR_PTR(); -EPROBE_DEFER until the pump is bound.
 */
struct sc8547 *sc8547_get(struct device *consumer, struct fwnode_handle *fwnode)
{
	struct i2c_client *client;
	struct sc8547 *cp;
	int ret;

	client = i2c_find_device_by_fwnode(fwnode);
	if (!client)
		return ERR_PTR(-EPROBE_DEFER);
	ret = devm_add_action_or_reset(consumer, sc8547_put_device, &client->dev);
	if (ret)
		return ERR_PTR(ret);
	if (!device_link_add(consumer, &client->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return ERR_PTR(-EINVAL);
	/* Set only once the pump is fully probed. */
	cp = i2c_get_clientdata(client);
	return cp ?: ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(sc8547_get);

/** sc8547_regmap() - the pump's registers, for its VOOC PHY */
struct regmap *sc8547_regmap(struct sc8547 *cp)
{
	return cp->regmap;
}
EXPORT_SYMBOL_GPL(sc8547_regmap);

/**
 * sc8547_prepare() - clear the fault latch and start the ADC, switching off
 * @cp: the pump
 *
 * Return: 0, or a negative error with the pump off.
 */
int sc8547_prepare(struct sc8547 *cp)
{
	static const struct reg_sequence setup[] = {
		{ SC8547_VAC_OVP, 0x01 },
		{ SC8547_PMID2OUT, 0x70 },
		{ SC8547_DET_CTRL, 0xd1 },
		{ SC8547_PHY_CTRL, 0x60 },
		{ SC8547_ADC_CTRL, SC8547_ADC_EN },
	};
	unsigned int vbat_ovp, ibus_ocp;
	int ret;

	mutex_lock(&cp->lock);
	ret = cp->enabled ? -EBUSY : 0;
	if (!ret)
		ret = sc8547_write_limits(cp);
	if (!ret)
		ret = regmap_multi_reg_write(cp->regmap, setup, ARRAY_SIZE(setup));
	if (!ret)
		ret = regmap_read(cp->regmap, SC8547_VBAT_OVP, &vbat_ovp);
	if (!ret)
		ret = regmap_read(cp->regmap, SC8547_IBUS_OCP, &ibus_ocp);
	if (!ret && ((vbat_ovp & 0xbf) != cp->vbat_ovp ||
		     (ibus_ocp & 0x0f) != cp->ibus_ocp))
		ret = -EIO;
	if (!ret) {
		cp->health = POWER_SUPPLY_HEALTH_GOOD;
		ret = sc8547_check_locked(cp, NULL);
	}
	cp->last = (struct sc8547_sample){};
	cp->prepared = !ret;
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547_prepare);

/**
 * sc8547_enable() - start switching under a five second watchdog
 * @cp: a prepared pump
 *
 * The caller must keep calling sc8547_sample() while the pump runs.
 */
int sc8547_enable(struct sc8547 *cp)
{
	int ret;

	mutex_lock(&cp->lock);
	ret = cp->prepared ? sc8547_check_locked(cp, NULL) : -EINVAL;
	if (!ret)
		ret = regmap_update_bits(cp->regmap, SC8547_MODE,
					 SC8547_MODE_WATCHDOG, SC8547_WATCHDOG_5S);
	if (!ret) {
		cp->enabled = true;
		ret = regmap_write(cp->regmap, SC8547_CONTROL,
				   cp->control | SC8547_CONTROL_EN);
	}
	if (ret && cp->enabled)
		sc8547_stop_locked(cp);
	mutex_unlock(&cp->lock);
	if (!ret)
		power_supply_changed(cp->psy);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547_enable);

/**
 * sc8547_disable() - stop switching and the ADC
 * @cp: the pump
 *
 * Return: 0 once switching is verified stopped.
 */
int sc8547_disable(struct sc8547 *cp)
{
	int ret;

	mutex_lock(&cp->lock);
	ret = sc8547_stop_locked(cp);
	if (!ret) {
		cp->prepared = false;
		ret = regmap_write(cp->regmap, SC8547_ADC_CTRL, 0);
	}
	mutex_unlock(&cp->lock);
	power_supply_changed(cp->psy);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547_disable);

/**
 * sc8547_sample() - check faults, service the watchdog and read the ADC
 * @cp: a prepared pump
 * @sample: the snapshot
 *
 * Return: 0, or -EIO after a fault, which has stopped the pump.
 */
int sc8547_sample(struct sc8547 *cp, struct sc8547_sample *sample)
{
	int ret;

	/*
	 * Two short holds: a VOOC PHY reply may need the pump in between, and
	 * the adapter waits only a few milliseconds for it.
	 */
	mutex_lock(&cp->lock);
	ret = cp->prepared ? sc8547_check_locked(cp, &sample->switching) : -EINVAL;
	sample->health = cp->health;
	mutex_unlock(&cp->lock);
	if (ret)
		return ret;
	/* A fault in between unprepares the pump. */
	mutex_lock(&cp->lock);
	ret = cp->prepared ? sc8547_read_adc_locked(cp, sample) : -EIO;
	if (!ret)
		cp->last = *sample;
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547_sample);

/**
 * sc8547_set_phy_handler() - dispatch the shared interrupt to the VOOC PHY
 * @cp: the pump
 * @handler: called in the interrupt thread before the pump's own check, or NULL
 * @data: the handler's argument
 *
 * After clearing the handler, it is no longer running or going to run.
 */
void sc8547_set_phy_handler(struct sc8547 *cp, void (*handler)(void *data),
			    void *data)
{
	mutex_lock(&cp->phy_lock);
	cp->phy_handler = handler;
	cp->phy_data = data;
	mutex_unlock(&cp->phy_lock);
}
EXPORT_SYMBOL_GPL(sc8547_set_phy_handler);

static irqreturn_t sc8547_irq(int irq, void *data)
{
	struct sc8547 *cp = data;
	int health;

	/* The PHY answers the adapter within milliseconds: it goes first. */
	mutex_lock(&cp->phy_lock);
	if (cp->phy_handler)
		cp->phy_handler(cp->phy_data);
	mutex_unlock(&cp->phy_lock);
	mutex_lock(&cp->lock);
	health = cp->health;
	sc8547_check_locked(cp, NULL);
	health ^= cp->health;
	mutex_unlock(&cp->lock);
	if (health)
		power_supply_changed(cp->psy);
	return IRQ_HANDLED;
}

static enum power_supply_property sc8547_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int sc8547_get_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       union power_supply_propval *val)
{
	struct sc8547 *cp = power_supply_get_drvdata(psy);
	struct sc8547_sample s;
	bool prepared;
	int health;

	mutex_lock(&cp->lock);
	s = cp->last;
	prepared = cp->prepared;
	health = cp->health;
	mutex_unlock(&cp->lock);

	switch (prop) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SC8547D";
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = prepared && s.switching ? POWER_SUPPLY_STATUS_CHARGING :
			POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = prepared && s.switching;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = health;
		return 0;
	default:
		break;
	}
	if (!prepared)
		return -ENODATA;
	switch (prop) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = s.vbat_uv;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = s.ibus_ua;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = s.temp;
		return 0;
	default:
		return -EINVAL;
	}
}

static int sc8547_read_limit(struct device *dev, const char *name, u32 def,
			     u32 min, u32 step, u32 max_code, u8 *code)
{
	u32 value = def;

	device_property_read_u32(dev, name, &value);
	if (value < min || (value - min) % step || (value - min) / step > max_code)
		return dev_err_probe(dev, -EINVAL, "Invalid %s %u\n", name, value);
	*code = (value - min) / step;
	return 0;
}

static int sc8547_parse(struct sc8547 *cp)
{
	struct device *dev = cp->dev;
	u32 frequency = 500000;
	int ret;

	ret = sc8547_read_limit(dev, "southchip,vbat-ovp-microvolt", 4650000,
				SC8547_VBAT_OVP_MIN_UV, SC8547_VBAT_OVP_STEP_UV,
				0x3f, &cp->vbat_ovp);
	if (!ret)
		ret = sc8547_read_limit(dev, "southchip,vbus-ovp-microvolt", 10000000,
					SC8547_VBUS_OVP_MIN_UV, SC8547_VBUS_OVP_STEP_UV,
					0x7f, &cp->vbus_ovp);
	if (!ret)
		ret = sc8547_read_limit(dev, "southchip,ibus-ocp-microamp", 3600000,
					SC8547_IBUS_OCP_MIN_UA, SC8547_IBUS_OCP_STEP_UA,
					0x0f, &cp->ibus_ocp);
	if (ret)
		return ret;
	device_property_read_u32(dev, "southchip,switching-frequency-hz", &frequency);
	if (frequency != 500000 && frequency != 600000)
		return dev_err_probe(dev, -EINVAL, "Invalid switching frequency %u\n",
				     frequency);
	cp->control = frequency == 600000 ? SC8547_CONTROL_600KHZ : 0;
	cp->always_otg = device_property_read_bool(dev, "southchip,always-otg-en");
	return 0;
}

static void sc8547_cleanup(void *data)
{
	struct sc8547 *cp = data;

	mutex_lock(&cp->lock);
	if (sc8547_stop_locked(cp))
		dev_crit(cp->dev, "cannot stop the pump\n");
	cp->prepared = false;
	regmap_write(cp->regmap, SC8547_ADC_CTRL, 0);
	mutex_unlock(&cp->lock);
}

static void sc8547_release_phy(struct device *dev)
{
	struct auxiliary_device *adev = to_auxiliary_dev(dev);

	fwnode_handle_put(dev_fwnode(dev));
	kfree(adev);
}

static void sc8547_remove_phy(void *data)
{
	auxiliary_device_delete(data);
	auxiliary_device_uninit(data);
}

/* The PHY gets its own node: it describes the charging path around it. */
static int sc8547_add_phy(struct sc8547 *cp, struct fwnode_handle *phy, u32 id)
{
	struct auxiliary_device *adev;
	int ret;

	adev = kzalloc(sizeof(*adev), GFP_KERNEL);
	if (!adev) {
		fwnode_handle_put(phy);
		return -ENOMEM;
	}
	adev->name = "vooc-phy";
	adev->id = id;
	adev->dev.parent = cp->dev;
	adev->dev.release = sc8547_release_phy;
	device_set_node(&adev->dev, phy);
	ret = auxiliary_device_init(adev);
	if (ret) {
		fwnode_handle_put(phy);
		kfree(adev);
		return ret;
	}
	ret = auxiliary_device_add(adev);
	if (ret) {
		auxiliary_device_uninit(adev);
		return ret;
	}
	return devm_add_action_or_reset(cp->dev, sc8547_remove_phy, adev);
}

static int sc8547_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config cfg = {};
	struct power_supply_desc *desc;
	struct fwnode_handle *phy;
	struct sc8547 *cp;
	unsigned int id;
	int ret;
	static const struct regmap_config regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = SC8547_MAX_REGISTER,
	};

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "Missing interrupt\n");
	cp = devm_kzalloc(dev, sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;
	cp->dev = dev;
	cp->health = POWER_SUPPLY_HEALTH_GOOD;
	mutex_init(&cp->lock);
	mutex_init(&cp->phy_lock);
	ret = sc8547_parse(cp);
	if (ret)
		return ret;
	cp->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(cp->regmap))
		return PTR_ERR(cp->regmap);
	ret = regmap_read(cp->regmap, SC8547_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot read the device ID\n");
	if (id != SC8547D_ID)
		return dev_err_probe(dev, -ENODEV, "Unknown device ID %02x\n", id);
	ret = sc8547_init(cp);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot initialize\n");
	ret = devm_add_action_or_reset(dev, sc8547_cleanup, cp);
	if (ret)
		return ret;

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	desc->name = devm_kasprintf(dev, GFP_KERNEL, "sc8547-%s", dev_name(dev));
	if (!desc->name)
		return -ENOMEM;
	desc->type = POWER_SUPPLY_TYPE_MAINS;
	desc->properties = sc8547_props;
	desc->num_properties = ARRAY_SIZE(sc8547_props);
	desc->get_property = sc8547_get_property;
	cfg.drv_data = cp;
	cfg.fwnode = dev_fwnode(dev);
	cp->psy = devm_power_supply_register(dev, desc, &cfg);
	if (IS_ERR(cp->psy))
		return dev_err_probe(dev, PTR_ERR(cp->psy), "Cannot register supply\n");
	ret = devm_request_threaded_irq(dev, client->irq, NULL, sc8547_irq,
					IRQF_ONESHOT, dev_name(dev), cp);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot request interrupt\n");

	phy = device_get_named_child_node(dev, "vooc-phy");
	if (phy) {
		ret = sc8547_add_phy(cp, phy, client->addr);
		if (ret)
			return dev_err_probe(dev, ret, "Cannot add the VOOC PHY\n");
	}
	i2c_set_clientdata(client, cp);
	return devm_add_action_or_reset(dev, sc8547_clear_clientdata, client);
}

static void sc8547_shutdown(struct i2c_client *client)
{
	struct sc8547 *cp = i2c_get_clientdata(client);

	if (cp)
		sc8547_cleanup(cp);
}

static const struct of_device_id sc8547_of_match[] = {
	{ .compatible = "southchip,sc8547d" },
	{}
};
MODULE_DEVICE_TABLE(of, sc8547_of_match);

static struct i2c_driver sc8547_driver = {
	.driver = {
		.name = "sc8547-charger",
		.of_match_table = sc8547_of_match,
	},
	.probe = sc8547_probe,
	.shutdown = sc8547_shutdown,
};
module_i2c_driver(sc8547_driver);

MODULE_DESCRIPTION("Southchip SC8547D charge pump");
MODULE_LICENSE("GPL");
