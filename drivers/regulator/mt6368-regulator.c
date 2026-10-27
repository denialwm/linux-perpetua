// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6368 PMIC LDO regulators
 *
 * The MT6368 sits on the SPMI bus beside the MT6363 and feeds the
 * connectivity, audio, card and touch rails of MT6897 boards. Each LDO has
 * an enable register with the enable and low-power bits, and a voltage
 * selector that indexes a fixed table.
 */

#include <linux/bits.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define MT6368_LDO_EN_BIT		BIT(0)
#define MT6368_LDO_LP_BIT		BIT(1)
#define MT6368_LDO_SW_OP_EN_BIT		BIT(7)
#define MT6368_LDO_VOSEL_MASK		GENMASK(3, 0)

#define MT6368_REGULATOR_MODE_NORMAL	0
#define MT6368_REGULATOR_MODE_LP	2

/**
 * struct mt6368_regulator_info - MT6368 LDO description
 * @desc: regulator description
 * @sw_op_en_reg: register whose bit 7 hands the LDO to the enable bit, or 0
 *		  when the LDO always follows it
 */
struct mt6368_regulator_info {
	struct regulator_desc desc;
	u16 sw_op_en_reg;
};

static const unsigned int mt6368_ldo_table1[] = {
	1200000, 1300000, 1500000, 1700000, 1800000, 2000000, 2100000, 2200000,
	2700000, 2800000, 2900000, 3000000, 3100000, 3300000, 3400000, 3500000,
};

static const unsigned int mt6368_ldo_table2[] = {
	1800000, 1900000, 2000000, 2100000, 2200000, 2300000, 2400000, 2500000,
	2600000, 2700000, 2800000, 2900000, 3000000, 3100000, 3200000, 3300000,
};

static const unsigned int mt6368_ldo_table3[] = {
	600000, 700000, 800000, 900000, 1000000, 1100000, 1200000, 1300000,
	1400000, 1500000, 1600000, 1700000, 1800000, 1900000, 2000000, 2100000,
};

static const unsigned int mt6368_ldo_table4[] = {
	1200000, 1300000, 1500000, 1700000, 1800000, 2000000, 2500000, 2600000,
	2700000, 2800000, 2900000, 3000000, 3100000, 3300000, 3400000, 3500000,
};

static int mt6368_ldo_enable(struct regulator_dev *rdev)
{
	const struct mt6368_regulator_info *info = rdev_get_drvdata(rdev);
	int ret;

	/*
	 * Some LDOs follow the enable bit only while software operation is
	 * enabled; otherwise the PMIC's hardware requests alone drive them.
	 */
	if (info->sw_op_en_reg) {
		ret = regmap_set_bits(rdev->regmap, info->sw_op_en_reg,
				      MT6368_LDO_SW_OP_EN_BIT);
		if (ret)
			return ret;
	}

	return regulator_enable_regmap(rdev);
}

static unsigned int mt6368_map_mode(unsigned int mode)
{
	switch (mode) {
	case MT6368_REGULATOR_MODE_NORMAL:
		return REGULATOR_MODE_NORMAL;
	case MT6368_REGULATOR_MODE_LP:
		return REGULATOR_MODE_IDLE;
	default:
		return REGULATOR_MODE_INVALID;
	}
}

static int mt6368_ldo_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	switch (mode) {
	case REGULATOR_MODE_NORMAL:
		return regmap_clear_bits(rdev->regmap, rdev->desc->enable_reg,
					 MT6368_LDO_LP_BIT);
	case REGULATOR_MODE_IDLE:
		return regmap_set_bits(rdev->regmap, rdev->desc->enable_reg,
				       MT6368_LDO_LP_BIT);
	default:
		return -EINVAL;
	}
}

static unsigned int mt6368_ldo_get_mode(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, rdev->desc->enable_reg, &val);
	if (ret)
		return REGULATOR_MODE_INVALID;

	return val & MT6368_LDO_LP_BIT ? REGULATOR_MODE_IDLE :
					 REGULATOR_MODE_NORMAL;
}

static const struct regulator_ops mt6368_ldo_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = mt6368_ldo_enable,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_mode = mt6368_ldo_set_mode,
	.get_mode = mt6368_ldo_get_mode,
};

#define MT6368_LDO(_match, _name, _table, _en, _vosel, _sw_op_en)	\
{									\
	.desc = {							\
		.name = #_name,						\
		.of_match = _match,					\
		.ops = &mt6368_ldo_ops,					\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.n_voltages = ARRAY_SIZE(_table),			\
		.volt_table = _table,					\
		.enable_reg = _en,					\
		.enable_mask = MT6368_LDO_EN_BIT,			\
		.vsel_reg = _vosel,					\
		.vsel_mask = MT6368_LDO_VOSEL_MASK,			\
		.of_map_mode = mt6368_map_mode,				\
	},								\
	.sw_op_en_reg = _sw_op_en,					\
}

static const struct mt6368_regulator_info mt6368_regulators[] = {
	MT6368_LDO("vusb", vusb, mt6368_ldo_table1, 0x1bc1, 0x1d95, 0),
	MT6368_LDO("vaux18", vaux18, mt6368_ldo_table2, 0x1bcf, 0x1d89, 0),
	MT6368_LDO("vrf13-aif", vrf13_aif, mt6368_ldo_table3, 0x1c07, 0x1e1a, 0),
	MT6368_LDO("vrf18-aif", vrf18_aif, mt6368_ldo_table3, 0x1c15, 0x1e11, 0),
	MT6368_LDO("vant18", vant18, mt6368_ldo_table3, 0x1c23, 0x1e09, 0x1c2a),
	MT6368_LDO("vibr", vibr, mt6368_ldo_table1, 0x1c31, 0x1d99, 0),
	MT6368_LDO("vio28", vio28, mt6368_ldo_table1, 0x1c3f, 0x1d9d, 0),
	MT6368_LDO("vfp", vfp, mt6368_ldo_table1, 0x1c4d, 0x1da1, 0),
	MT6368_LDO("vtp", vtp, mt6368_ldo_table1, 0x1c87, 0x1da5, 0),
	MT6368_LDO("vmch", vmch, mt6368_ldo_table4, 0x1c95, 0x1da9, 0),
	MT6368_LDO("vmc", vmc, mt6368_ldo_table1, 0x1ca4, 0x1dad, 0),
	MT6368_LDO("vaud18", vaud18, mt6368_ldo_table3, 0x1cb2, 0x1db9, 0),
	MT6368_LDO("vcn33-1", vcn33_1, mt6368_ldo_table4, 0x1cc0, 0x1db1, 0x1cc7),
	MT6368_LDO("vcn33-2", vcn33_2, mt6368_ldo_table4, 0x1cce, 0x1db5, 0x1cd5),
	MT6368_LDO("vefuse", vefuse, mt6368_ldo_table1, 0x1d07, 0x1e0d, 0),
	MT6368_LDO("vsim2", vsim2, mt6368_ldo_table1, 0x1b96, 0x1d91, 0),
};

static int mt6368_regulator_probe(struct platform_device *pdev)
{
	struct regulator_config config = {};
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;
	struct regmap *regmap;
	int i;

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return dev_err_probe(dev, -ENODEV, "Cannot get the PMIC regmap\n");

	config.dev = dev;
	config.regmap = regmap;

	for (i = 0; i < ARRAY_SIZE(mt6368_regulators); i++) {
		config.driver_data = (void *)&mt6368_regulators[i];
		rdev = devm_regulator_register(dev, &mt6368_regulators[i].desc,
					       &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "Cannot register %s\n",
					     mt6368_regulators[i].desc.name);
	}

	return 0;
}

static const struct of_device_id mt6368_regulator_of_match[] = {
	{ .compatible = "mediatek,mt6368-regulator" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6368_regulator_of_match);

static struct platform_driver mt6368_regulator_driver = {
	.driver = {
		.name = "mt6368-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = mt6368_regulator_of_match,
	},
	.probe = mt6368_regulator_probe,
};
module_platform_driver(mt6368_regulator_driver);

MODULE_DESCRIPTION("MediaTek MT6368 PMIC LDO regulators");
MODULE_LICENSE("GPL");
