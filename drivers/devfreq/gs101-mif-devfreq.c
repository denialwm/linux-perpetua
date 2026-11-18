// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs101 memory interface (MIF) frequency scaling.
 *
 * The MIF rate is an ACPM DVFS domain: setting its clock asks the firmware
 * for the rate, and the firmware sets the voltage. Nothing in the kernel
 * measures memory traffic on this SoC yet, so the rate follows the CPUs:
 * the passive governor maps each cluster's frequency onto the MIF range
 * and takes the highest.
 */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>

struct gs101_mif {
	struct clk *clk;
	struct devfreq_dev_profile profile;
	struct devfreq_passive_data passive_data;
};

static int gs101_mif_target(struct device *dev, unsigned long *freq,
			    u32 flags)
{
	struct dev_pm_opp *opp;
	int ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	ret = dev_pm_opp_set_opp(dev, opp);
	dev_pm_opp_put(opp);

	return ret;
}

static int gs101_mif_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct gs101_mif *mif = dev_get_drvdata(dev);

	*freq = clk_get_rate(mif->clk);

	return 0;
}

static int gs101_mif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs101_mif *mif;
	struct devfreq *devfreq;
	int ret;

	mif = devm_kzalloc(dev, sizeof(*mif), GFP_KERNEL);
	if (!mif)
		return -ENOMEM;

	platform_set_drvdata(pdev, mif);

	mif->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(mif->clk))
		return dev_err_probe(dev, PTR_ERR(mif->clk),
				     "failed to get the MIF clock\n");

	ret = devm_pm_opp_of_add_table(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add the OPP table\n");

	mif->profile.initial_freq = clk_get_rate(mif->clk);
	mif->profile.target = gs101_mif_target;
	mif->profile.get_cur_freq = gs101_mif_get_cur_freq;
	mif->passive_data.parent_type = CPUFREQ_PARENT_DEV;

	/* Defers until cpufreq has a policy for every CPU. */
	devfreq = devm_devfreq_add_device(dev, &mif->profile,
					  DEVFREQ_GOV_PASSIVE,
					  &mif->passive_data);
	if (IS_ERR(devfreq))
		return dev_err_probe(dev, PTR_ERR(devfreq),
				     "failed to add the devfreq device\n");

	return 0;
}

static const struct of_device_id gs101_mif_of_match[] = {
	{ .compatible = "google,gs101-mif" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs101_mif_of_match);

static struct platform_driver gs101_mif_driver = {
	.probe = gs101_mif_probe,
	.driver = {
		.name = "gs101-mif-devfreq",
		.of_match_table = gs101_mif_of_match,
	},
};
module_platform_driver(gs101_mif_driver);

MODULE_DESCRIPTION("Google gs101 MIF frequency scaling");
MODULE_LICENSE("GPL");
