// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Pixel 5a (barbet) battery gauge.
 *
 * Reads the PM7250B VADC vbat_sns channel and reports a voltage-derived
 * state of charge. This is not a coulomb-counting gauge: the mainline tree
 * has no Qualcomm QG driver, so the capacity follows a linear OCV window.
 * Status is inherited from the PM7250B charger when it is present.
 */

#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

struct barbet_battery {
	struct iio_channel *vbat;
	u32 min_uv;
	u32 max_uv;
};

static int barbet_battery_voltage(struct barbet_battery *b, int *uv)
{
	return iio_read_channel_processed(b->vbat, uv);
}

static int barbet_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct barbet_battery *b = power_supply_get_drvdata(psy);
	struct power_supply *chg;
	int uv, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = barbet_battery_voltage(b, &uv);
		if (ret)
			return ret;
		val->intval = uv;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = barbet_battery_voltage(b, &uv);
		if (ret)
			return ret;
		if (uv <= (int)b->min_uv)
			val->intval = 0;
		else if (uv >= (int)b->max_uv)
			val->intval = 100;
		else
			val->intval = (uv - b->min_uv) * 100 /
				      (b->max_uv - b->min_uv);
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		chg = power_supply_get_by_name("pm7250b-usb");
		if (!chg) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			return 0;
		}
		ret = power_supply_get_property(chg, POWER_SUPPLY_PROP_STATUS, val);
		power_supply_put(chg);
		return ret;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property barbet_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static const struct power_supply_desc barbet_battery_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = barbet_battery_props,
	.num_properties = ARRAY_SIZE(barbet_battery_props),
	.get_property = barbet_battery_get_property,
};

static int barbet_battery_probe(struct platform_device *pdev)
{
	struct power_supply_config cfg = {};
	struct barbet_battery *b;
	struct power_supply *psy;

	b = devm_kzalloc(&pdev->dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	b->vbat = devm_iio_channel_get(&pdev->dev, NULL);
	if (IS_ERR(b->vbat))
		return dev_err_probe(&pdev->dev, PTR_ERR(b->vbat),
				     "cannot get vbat_sns channel\n");

	if (device_property_read_u32(&pdev->dev, "voltage-min-microvolt",
				     &b->min_uv))
		b->min_uv = 3200000;
	if (device_property_read_u32(&pdev->dev, "voltage-max-microvolt",
				     &b->max_uv))
		b->max_uv = 4350000;

	cfg.drv_data = b;
	cfg.fwnode = dev_fwnode(&pdev->dev);

	psy = devm_power_supply_register(&pdev->dev, &barbet_battery_desc, &cfg);
	if (IS_ERR(psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(psy),
				     "cannot register battery supply\n");
	return 0;
}

static const struct of_device_id barbet_battery_of_match[] = {
	{ .compatible = "google,barbet-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, barbet_battery_of_match);

static struct platform_driver barbet_battery_driver = {
	.probe = barbet_battery_probe,
	.driver = {
		.name = "barbet-battery",
		.of_match_table = barbet_battery_of_match,
	},
};
module_platform_driver(barbet_battery_driver);

MODULE_DESCRIPTION("Google Pixel 5a battery gauge");
MODULE_LICENSE("GPL");
