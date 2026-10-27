// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek connectivity subsystem power control
 *
 * The connectivity subsystem holds the Wi-Fi, Bluetooth and GNSS cores
 * behind a shared infrastructure, conninfra, and talks to a separate radio
 * die. On MT6897 the secure world owns the low-level power-on: the chip ID
 * check, the D-die and A-die setup, the radio die's SPI, the PLLs and the
 * calibration are SiP calls. This driver supplies the rails, the pins and
 * the power domain around those calls, and powers conninfra for the radios'
 * own drivers, which are its children.
 */

#include <linux/arm-smccc.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/mediatek/mtk-consys.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>

#define MTK_SIP_KERNEL_CONNSYS_CONTROL	MTK_SIP_SMC_CMD(0x534)

enum mtk_consys_op {
	CONSYS_INIT_ATF_DATA = 0,
	CONSYS_POLLING_CHIPID = 10,
	CONSYS_D_DIE_CFG = 11,
	CONSYS_SPI_MASTER_CFG = 12,
	CONSYS_A_DIE_CFG = 13,
	CONSYS_AFE_SW_PATCH = 14,
	CONSYS_AFE_WBG_CAL = 15,
	CONSYS_SUBSYS_PLL_INITIAL = 16,
	CONSYS_LOW_POWER_SETTING = 17,
	CONSYS_CONNINFRA_WAKEUP = 18,
	CONSYS_CONNINFRA_SLEEP = 19,
	CONSYS_SUBSYS_STATUS_UPDATE = 24,
	CONSYS_EMI_SET_REMAPPING_REG = 70,
};

/* How the radio die gets its clock, as the secure world numbers it */
enum mtk_consys_clock_type {
	CONSYS_CLOCK_26M_CO = 0,
	CONSYS_CLOCK_52M_CO = 1,
	CONSYS_CLOCK_26M_TCXO = 2,
	CONSYS_CLOCK_52M_TCXO = 3,
};

static const char * const mtk_consys_radio_supplies[] = {
	"vant18", "vcn33-1", "vcn33-2",
};

struct mtk_consys {
	struct device *dev;
	struct regulator *vrf_io18;
	struct regulator *vcn13;
	struct regulator_bulk_data radio_supplies[ARRAY_SIZE(mtk_consys_radio_supplies)];
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	enum mtk_consys_clock_type clock_type;
	bool secure_world_ready;
	/* The radios that are powered, one bit per enum mtk_consys_radio */
	unsigned int status;
	/* Serialises power changes */
	struct mutex lock;
};

static struct platform_driver mtk_consys_driver;

static long mtk_consys_call(struct mtk_consys *consys, enum mtk_consys_op op,
			    unsigned long arg1, unsigned long arg2,
			    unsigned long *ret1)
{
	struct arm_smccc_res res;
	long ret;

	arm_smccc_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, op, arg1, arg2, 0, 0, 0,
		      0, &res);
	ret = (int)res.a0;
	if (ret)
		dev_err(consys->dev, "secure call %d failed: %ld\n", op, ret);
	if (ret1)
		*ret1 = res.a1;

	return ret;
}

static int mtk_consys_conninfra_on(struct mtk_consys *consys,
				   enum mtk_consys_radio radio)
{
	unsigned int next = BIT(radio);
	unsigned long adie;
	long ret;

	/* MediaTek's driver goes on whatever this first call returns */
	if (!consys->secure_world_ready) {
		mtk_consys_call(consys, CONSYS_INIT_ATF_DATA, 0,
				consys->clock_type, NULL);
		consys->secure_world_ready = true;
	}

	ret = regulator_enable(consys->vrf_io18);
	if (ret)
		return ret;

	ret = regulator_enable(consys->vcn13);
	if (ret)
		goto err_vrf_io18;

	ret = pinctrl_select_state(consys->pinctrl, consys->pins_default);
	if (ret)
		goto err_vcn13;

	ret = pm_runtime_resume_and_get(consys->dev);
	if (ret)
		goto err_pins;

	if (mtk_consys_call(consys, CONSYS_POLLING_CHIPID, 0, 0, NULL) ||
	    mtk_consys_call(consys, CONSYS_EMI_SET_REMAPPING_REG, 0, 0, NULL) ||
	    mtk_consys_call(consys, CONSYS_D_DIE_CFG, 0, 0, NULL) ||
	    mtk_consys_call(consys, CONSYS_SPI_MASTER_CFG, 0, next, NULL) ||
	    mtk_consys_call(consys, CONSYS_A_DIE_CFG, 0, next, &adie)) {
		ret = -EIO;
		goto err_pm;
	}

	dev_dbg(consys->dev, "A-die version %#lx\n", adie);

	/* The rest of the sequence goes on after a failed step, as MediaTek's */
	mtk_consys_call(consys, CONSYS_AFE_SW_PATCH, 0, 0, NULL);
	mtk_consys_call(consys, CONSYS_AFE_WBG_CAL, 0, 0, NULL);
	mtk_consys_call(consys, CONSYS_SUBSYS_PLL_INITIAL, 0, 0, NULL);
	mtk_consys_call(consys, CONSYS_SUBSYS_STATUS_UPDATE, 1, radio, NULL);
	mtk_consys_call(consys, CONSYS_LOW_POWER_SETTING, 0, next, NULL);

	ret = regulator_bulk_enable(ARRAY_SIZE(consys->radio_supplies),
				    consys->radio_supplies);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	pm_runtime_put_sync(consys->dev);
err_pins:
	pinctrl_select_state(consys->pinctrl, consys->pins_sleep);
err_vcn13:
	regulator_disable(consys->vcn13);
err_vrf_io18:
	regulator_disable(consys->vrf_io18);

	return ret;
}

static void mtk_consys_conninfra_off(struct mtk_consys *consys)
{
	regulator_bulk_disable(ARRAY_SIZE(consys->radio_supplies),
			       consys->radio_supplies);
	pm_runtime_put_sync(consys->dev);
	pinctrl_select_state(consys->pinctrl, consys->pins_sleep);
	regulator_disable(consys->vcn13);
	regulator_disable(consys->vrf_io18);
}

/* Moves a running conninfra from one set of radios to another */
static int mtk_consys_conninfra_update(struct mtk_consys *consys,
				       enum mtk_consys_radio radio, bool on)
{
	unsigned int curr = consys->status;
	unsigned int next = on ? curr | BIT(radio) : curr & ~BIT(radio);

	if (mtk_consys_call(consys, CONSYS_CONNINFRA_WAKEUP, 0, 0, NULL))
		return -EIO;

	if (on)
		mtk_consys_call(consys, CONSYS_A_DIE_CFG, curr, next, NULL);
	mtk_consys_call(consys, CONSYS_SUBSYS_STATUS_UPDATE, on, radio, NULL);
	mtk_consys_call(consys, CONSYS_SPI_MASTER_CFG, curr, next, NULL);
	mtk_consys_call(consys, CONSYS_LOW_POWER_SETTING, curr, next, NULL);
	mtk_consys_call(consys, CONSYS_CONNINFRA_SLEEP, 0, 0, NULL);

	return 0;
}

/**
 * mtk_consys_get() - get the connectivity subsystem of a radio
 * @dev: the radio's device, a child of the connectivity subsystem's node
 *
 * Return: the connectivity subsystem, or an ERR_PTR() when @dev is not a
 * child of one.
 */
struct mtk_consys *mtk_consys_get(struct device *dev)
{
	struct device *parent = dev->parent;

	if (!parent || parent->driver != &mtk_consys_driver.driver)
		return ERR_PTR(-ENODEV);

	return dev_get_drvdata(parent);
}
EXPORT_SYMBOL_GPL(mtk_consys_get);

/**
 * mtk_consys_power_on() - power the connectivity subsystem for a radio
 * @consys: the connectivity subsystem
 * @radio: the radio that needs it
 *
 * Powers conninfra and the radio die on for the first radio, and adds
 * @radio to them for the next ones. The radio's own driver then powers
 * on its core.
 *
 * Return: 0 on success, or a negative error code.
 */
int mtk_consys_power_on(struct mtk_consys *consys, enum mtk_consys_radio radio)
{
	int ret = 0;

	mutex_lock(&consys->lock);

	if (consys->status & BIT(radio))
		goto out;

	if (consys->status)
		ret = mtk_consys_conninfra_update(consys, radio, true);
	else
		ret = mtk_consys_conninfra_on(consys, radio);
	if (!ret)
		consys->status |= BIT(radio);
out:
	mutex_unlock(&consys->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(mtk_consys_power_on);

/**
 * mtk_consys_power_off() - release the connectivity subsystem for a radio
 * @consys: the connectivity subsystem
 * @radio: the radio that no longer needs it, with its core already off
 */
void mtk_consys_power_off(struct mtk_consys *consys,
			  enum mtk_consys_radio radio)
{
	mutex_lock(&consys->lock);

	if (consys->status & BIT(radio)) {
		if (consys->status == BIT(radio))
			mtk_consys_conninfra_off(consys);
		else
			mtk_consys_conninfra_update(consys, radio, false);
		consys->status &= ~BIT(radio);
	}

	mutex_unlock(&consys->lock);
}
EXPORT_SYMBOL_GPL(mtk_consys_power_off);

static int mtk_consys_get_clock_type(struct mtk_consys *consys)
{
	struct device *dev = consys->dev;
	bool tcxo = of_property_read_bool(dev->of_node, "mediatek,tcxo");
	unsigned long rate;
	struct clk *clk;

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Cannot get the reference clock\n");

	rate = clk_get_rate(clk);
	switch (rate) {
	case 26000000:
		consys->clock_type = tcxo ? CONSYS_CLOCK_26M_TCXO :
					    CONSYS_CLOCK_26M_CO;
		return 0;
	case 52000000:
		consys->clock_type = tcxo ? CONSYS_CLOCK_52M_TCXO :
					    CONSYS_CLOCK_52M_CO;
		return 0;
	default:
		return dev_err_probe(dev, -EINVAL,
				     "Unsupported reference clock rate %lu\n",
				     rate);
	}
}

static int mtk_consys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_consys *consys;
	int i, ret;

	consys = devm_kzalloc(dev, sizeof(*consys), GFP_KERNEL);
	if (!consys)
		return -ENOMEM;

	consys->dev = dev;
	ret = devm_mutex_init(dev, &consys->lock);
	if (ret)
		return ret;

	ret = mtk_consys_get_clock_type(consys);
	if (ret)
		return ret;

	consys->vrf_io18 = devm_regulator_get(dev, "vrf-io18");
	if (IS_ERR(consys->vrf_io18))
		return dev_err_probe(dev, PTR_ERR(consys->vrf_io18),
				     "Cannot get the vrf-io18 supply\n");

	consys->vcn13 = devm_regulator_get(dev, "vcn13");
	if (IS_ERR(consys->vcn13))
		return dev_err_probe(dev, PTR_ERR(consys->vcn13),
				     "Cannot get the vcn13 supply\n");

	for (i = 0; i < ARRAY_SIZE(consys->radio_supplies); i++)
		consys->radio_supplies[i].supply = mtk_consys_radio_supplies[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(consys->radio_supplies),
				      consys->radio_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot get the radio supplies\n");

	consys->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(consys->pinctrl))
		return dev_err_probe(dev, PTR_ERR(consys->pinctrl),
				     "Cannot get the pins\n");

	consys->pins_default = pinctrl_lookup_state(consys->pinctrl,
						    PINCTRL_STATE_DEFAULT);
	if (IS_ERR(consys->pins_default))
		return dev_err_probe(dev, PTR_ERR(consys->pins_default),
				     "Cannot get the default pin state\n");

	consys->pins_sleep = pinctrl_lookup_state(consys->pinctrl,
						  PINCTRL_STATE_SLEEP);
	if (IS_ERR(consys->pins_sleep))
		return dev_err_probe(dev, PTR_ERR(consys->pins_sleep),
				     "Cannot get the sleep pin state\n");

	/* The interface pins stay parked until the first radio powers on */
	ret = pinctrl_select_state(consys->pinctrl, consys->pins_sleep);
	if (ret)
		return ret;

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, consys);

	return devm_of_platform_populate(dev);
}

static const struct of_device_id mtk_consys_of_match[] = {
	{ .compatible = "mediatek,mt6897-consys" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_consys_of_match);

static struct platform_driver mtk_consys_driver = {
	.driver = {
		.name = "mtk-consys",
		.of_match_table = mtk_consys_of_match,
	},
	.probe = mtk_consys_probe,
};
module_platform_driver(mtk_consys_driver);

MODULE_DESCRIPTION("MediaTek connectivity subsystem power control");
MODULE_LICENSE("GPL");
