// SPDX-License-Identifier: GPL-2.0+
/*
 * Will Semiconductor WL2866D: four I2C programmable LDOs for camera modules.
 * LDO1 and LDO2 run from VIN1, 0.6 V in 6 mV steps; LDO3 and LDO4 from VIN2,
 * 1.2 V in 12.5 mV steps. Register map as in Motorola's driver.
 */
#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define WL2866D_CHIP_REV		0x00
#define WL2866D_DISCHARGE		0x02
#define WL2866D_LDO1_VOUT		0x03
#define WL2866D_LDO_EN			0x0e
#define WL2866D_SEQ_STATUS		0x0f

static const struct regulator_ops wl2866d_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_active_discharge = regulator_set_active_discharge_regmap,
};

#define WL2866D_LDO(_num, _supply, _min, _step)				\
	{								\
		.name = "ldo" #_num,					\
		.of_match = "ldo" #_num,				\
		.regulators_node = "regulators",			\
		.supply_name = _supply,					\
		.id = _num - 1,						\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.ops = &wl2866d_ops,					\
		.n_voltages = 256,					\
		.min_uV = _min,						\
		.uV_step = _step,					\
		.vsel_reg = WL2866D_LDO1_VOUT + _num - 1,		\
		.vsel_mask = 0xff,					\
		.enable_reg = WL2866D_LDO_EN,				\
		.enable_mask = BIT(_num - 1),				\
		.active_discharge_reg = WL2866D_DISCHARGE,		\
		.active_discharge_mask = BIT(_num - 1),			\
		.active_discharge_on = BIT(_num - 1),			\
	}

static const struct regulator_desc wl2866d_regulators[] = {
	WL2866D_LDO(1, "vin1", 600000, 6000),
	WL2866D_LDO(2, "vin1", 600000, 6000),
	WL2866D_LDO(3, "vin2", 1200000, 12500),
	WL2866D_LDO(4, "vin2", 1200000, 12500),
};

static const struct regmap_config wl2866d_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WL2866D_SEQ_STATUS,
};

static int wl2866d_probe(struct i2c_client *i2c)
{
	struct regulator_config config = { .dev = &i2c->dev };
	struct regulator_dev *rdev;
	unsigned int rev;
	int i, ret;

	config.regmap = devm_regmap_init_i2c(i2c, &wl2866d_regmap);
	if (IS_ERR(config.regmap))
		return dev_err_probe(&i2c->dev, PTR_ERR(config.regmap),
				     "failed to set up the register map\n");

	ret = regmap_read(config.regmap, WL2866D_CHIP_REV, &rev);
	if (ret)
		return dev_err_probe(&i2c->dev, ret, "no answer\n");
	dev_info(&i2c->dev, "revision %#x\n", rev);

	for (i = 0; i < ARRAY_SIZE(wl2866d_regulators); i++) {
		rdev = devm_regulator_register(&i2c->dev,
					       &wl2866d_regulators[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(&i2c->dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     wl2866d_regulators[i].name);
	}

	return 0;
}

static const struct of_device_id wl2866d_of_match[] = {
	{ .compatible = "willsemi,wl2866d" },
	{ }
};
MODULE_DEVICE_TABLE(of, wl2866d_of_match);

static struct i2c_driver wl2866d_driver = {
	.driver = {
		.name = "wl2866d",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = wl2866d_of_match,
	},
	.probe = wl2866d_probe,
};
module_i2c_driver(wl2866d_driver);

MODULE_DESCRIPTION("Will Semiconductor WL2866D camera LDO regulator driver");
MODULE_LICENSE("GPL");
