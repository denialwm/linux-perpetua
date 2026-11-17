// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe PHY driver for the Google gs101 SoC
 *
 * The PMA and PCS values are tuning data from the vendor's calibration
 * code, which documents neither their meaning nor their derivation; they
 * are written in the vendor's order.
 *
 * The PHY's reset inputs (common, init and port reset) are outputs of the
 * controller, so the controller drives the sequence:
 *
 *   phy_init()       supplies on, PMU isolation off, power-down cleared,
 *                    PCS mux set up
 *   controller       pulses the resets, holds the init reset
 *   phy_power_on()   PMA and PCS configuration
 *   controller       releases the common and port resets
 *   phy_calibrate()  post-reset PMA configuration
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define GS101_PCIE_PHY_LANES		2
#define GS101_PCIE_PHY_LANE_OFFSET	0x800

/* PMU PCIe PHY control register */
#define PMU_PCIE_PHY_ISOLATION_OFF	BIT(0)

/* PMA */
#define PMA_CMN_INPUT_CLK		0x0d8
#define   PMA_CMN_INPUT_CLK_INIT	0x28
#define PMA_CMN_PMA_CTRL		0x400
#define   PMA_CMN_PMA_EN		BIT(7)
#define PMA_CMN_AFC			0x5d0
#define   PMA_CMN_AFC_EN		BIT(4)
#define   PMA_CMN_AFC_PREV		BIT(3)
#define PMA_LANE_AFC_CAL_MODE		0xbf4
#define   PMA_LANE_AFC_CAL_INIT		0x4

/* PCS, per lane */
#define PCS_LANE_MUX			0x008
#define   PCS_MUX_GLITCH_WA		BIT(7)
#define   PCS_ERIO_CLK_GATE_L12		BIT(4)
#define   PCS_ERIO_CLK_GATE_MASK	GENMASK(5, 4)
#define PCS_LANE_AGGREGATION		0x004
#define PCS_LANE_L2_ENTRY		0x150
#define PCS_LANE_RC			0x154
#define PCS_LANE_L2_PD_DELAY		0x170

struct gs101_pcie_phy_reg {
	u16 off;
	u8 val;
};

/* Power-down: common and lane overrides, written before the PMA is off */
static const struct gs101_pcie_phy_reg gs101_pwrdn[] = {
	{ 0x404, 0xa8 }, { 0x408, 0x20 }, { 0x40c, 0x0a },
	{ 0x800, 0x0a }, { 0x804, 0xbf }, { 0xa40, 0x02 }, { 0xa44, 0x2a },
	{ 0xa48, 0xaa }, { 0xa4c, 0xa8 }, { 0xa50, 0x80 },
	{ 0x1000, 0x0a }, { 0x1004, 0xbf }, { 0x1240, 0x02 }, { 0x1244, 0x2a },
	{ 0x1248, 0xaa }, { 0x124c, 0xa8 }, { 0x1250, 0x80 },
};

/* PMA common block */
static const struct gs101_pcie_phy_reg gs101_pma_cmn[] = {
	{ 0x018, 0x50 }, { 0x048, 0x33 }, { 0x068, 0x01 }, { 0x070, 0x12 },
	{ 0x08c, 0x00 }, { 0x090, 0x21 }, { 0x0b0, 0x14 }, { 0x0b8, 0x50 },
	{ 0x0e0, 0x51 }, { 0x100, 0x00 }, { 0x104, 0x80 }, { 0x140, 0x38 },
	{ 0x180, 0xa4 },
	{ 0x188, 0x03 },		/* LC PLL: 100 MHz reference, undivided */
	{ 0x2a8, 0x38 }, { 0x2e4, 0x12 },
	{ 0x400, 0x80 },		/* PMA on, lanes aggregated */
	{ 0x408, 0x20 }, { 0x550, 0x00 }, { 0x5a8, 0x00 }, { 0x5ec, 0xff },
	/* root complex */
	{ 0x458, 0x02 },		/* 100 MHz clock on */
	{ 0x5b0, 0x34 },		/* differential reference clock */
	{ 0x450, 0x20 },		/* delay on L1.2 entry */
};

/* PMA lane block, at 0x800 per lane; later entries override earlier ones */
static const struct gs101_pcie_phy_reg gs101_pma_lane[] = {
	{ 0x02c, 0x08 }, { 0x030, 0x24 }, { 0x078, 0x80 }, { 0x094, 0x40 },
	{ 0x0c0, 0x00 }, { 0x0f4, 0x30 }, { 0x108, 0x05 }, { 0x10c, 0xe0 },
	{ 0x114, 0xd4 }, { 0x11c, 0xd3 }, { 0x120, 0xce }, { 0x124, 0x01 },
	{ 0x128, 0x35 }, { 0x12c, 0xba }, { 0x130, 0x41 }, { 0x134, 0x15 },
	{ 0x138, 0x13 }, { 0x13c, 0x4e }, { 0x148, 0x43 }, { 0x14c, 0xfc },
	{ 0x154, 0x10 }, { 0x158, 0x69 }, { 0x164, 0x40 }, { 0x1b4, 0xf6 },
	{ 0x1c0, 0x2d }, { 0x1c4, 0xb7 }, { 0x1cc, 0x3c }, { 0x1dc, 0x7e },
	{ 0x240, 0x02 }, { 0x270, 0x26 }, { 0x274, 0x00 },
	{ 0x340, 0x06 }, { 0x344, 0x06 }, { 0x348, 0x04 }, { 0x34c, 0x03 },
	{ 0x350, 0x03 }, { 0x354, 0x03 }, { 0x358, 0x03 }, { 0x35c, 0x03 },
	{ 0x410, 0x1b }, { 0x444, 0x10 }, { 0x448, 0x10 }, { 0x44c, 0x10 },
	{ 0x450, 0x10 }, { 0x454, 0x02 }, { 0x458, 0x02 }, { 0x45c, 0x02 },
	{ 0x460, 0x02 }, { 0x46c, 0x02 }, { 0x470, 0x02 }, { 0x4a8, 0xe7 },
	{ 0x4ac, 0x00 }, { 0x4b0, 0x0e }, { 0x4cc, 0x1c }, { 0x4d4, 0x05 },
	{ 0x4d8, 0x77 }, { 0x4dc, 0x7a }, { 0x5b4, 0x2f },
	/* Gen4 TX termination */
	{ 0x1cc, 0x00 }, { 0x4d8, 0xff }, { 0x4dc, 0x6e },
	/* TX driver current */
	{ 0x02c, 0x0f }, { 0x030, 0x60 }, { 0x034, 0x7e },
	/* TX pre- and post-cursor */
	{ 0x024, 0x10 }, { 0x02c, 0x4f }, { 0x018, 0x1f }, { 0x020, 0x00 },
	/* RX tuning */
	{ 0x128, 0x3d }, { 0x120, 0xca }, { 0x208, 0x2f }, { 0x12c, 0xb8 },
	{ 0x114, 0xf4 }, { 0x16c, 0x55 }, { 0x188, 0x78 }, { 0x134, 0x17 },
	{ 0x13c, 0x4c }, { 0x194, 0x3b }, { 0x148, 0x73 }, { 0x39c, 0x3f },
	{ 0x1c8, 0x20 }, { 0x1c4, 0xff }, { 0x408, 0x05 }, { 0x43c, 0x04 },
	{ 0x440, 0x04 },
};

struct gs101_pcie_phy {
	void __iomem *pma;
	void __iomem *pcs;
	struct regmap *pmu;
	u32 pmu_offset;
	struct clk_bulk_data *clks;
	int num_clks;
	struct regulator_bulk_data supplies[2];
};

static void gs101_pcie_phy_write_table(void __iomem *base,
				       const struct gs101_pcie_phy_reg *regs,
				       size_t count)
{
	size_t i;

	for (i = 0; i < count; i++)
		writel(regs[i].val, base + regs[i].off);
}

static void gs101_pcie_phy_pcs_rmw(struct gs101_pcie_phy *phy, u32 reg,
				   u32 clr, u32 set)
{
	unsigned int lane;
	u32 val;

	for (lane = 0; lane < GS101_PCIE_PHY_LANES; lane++) {
		void __iomem *addr = phy->pcs + lane * GS101_PCIE_PHY_LANE_OFFSET + reg;

		val = readl(addr);
		val &= ~clr;
		val |= set;
		writel(val, addr);
	}
}

static void gs101_pcie_phy_pcs_write(struct gs101_pcie_phy *phy, u32 reg,
				     u32 val)
{
	unsigned int lane;

	for (lane = 0; lane < GS101_PCIE_PHY_LANES; lane++)
		writel(val, phy->pcs + lane * GS101_PCIE_PHY_LANE_OFFSET + reg);
}

static int gs101_pcie_phy_init(struct phy *p)
{
	struct gs101_pcie_phy *phy = phy_get_drvdata(p);
	unsigned int i;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(phy->supplies), phy->supplies);
	if (ret)
		return ret;

	/* The PHY's clocks run only once it is no longer isolated. */
	ret = regmap_update_bits(phy->pmu, phy->pmu_offset,
				 PMU_PCIE_PHY_ISOLATION_OFF,
				 PMU_PCIE_PHY_ISOLATION_OFF);
	if (ret)
		goto err_supplies;

	ret = clk_bulk_prepare_enable(phy->num_clks, phy->clks);
	if (ret)
		goto err_isolate;

	/* Clear the power-down overrides. */
	writel(PMA_CMN_INPUT_CLK_INIT, phy->pma + PMA_CMN_INPUT_CLK);
	mdelay(1);
	for (i = 0; i < ARRAY_SIZE(gs101_pwrdn); i++)
		writel(0, phy->pma + gs101_pwrdn[i].off);

	writel(PMA_CMN_INPUT_CLK_INIT, phy->pma + PMA_CMN_INPUT_CLK);

	/* Keep the PCS clock mux from glitching across the resets. */
	gs101_pcie_phy_pcs_rmw(phy, PCS_LANE_MUX, 0, PCS_MUX_GLITCH_WA);

	return 0;

err_isolate:
	regmap_update_bits(phy->pmu, phy->pmu_offset,
			   PMU_PCIE_PHY_ISOLATION_OFF, 0);
err_supplies:
	regulator_bulk_disable(ARRAY_SIZE(phy->supplies), phy->supplies);
	return ret;
}

static int gs101_pcie_phy_power_on(struct phy *p)
{
	struct gs101_pcie_phy *phy = phy_get_drvdata(p);
	unsigned int lane;

	gs101_pcie_phy_write_table(phy->pma, gs101_pma_cmn,
				   ARRAY_SIZE(gs101_pma_cmn));
	for (lane = 0; lane < GS101_PCIE_PHY_LANES; lane++)
		gs101_pcie_phy_write_table(phy->pma + GS101_PCIE_PHY_LANE_OFFSET +
					   lane * GS101_PCIE_PHY_LANE_OFFSET,
					   gs101_pma_lane,
					   ARRAY_SIZE(gs101_pma_lane));

	/* No bifurcation, root complex, L2 entry and power-down delay. */
	gs101_pcie_phy_pcs_write(phy, PCS_LANE_AGGREGATION, 0);
	gs101_pcie_phy_pcs_write(phy, PCS_LANE_RC, 0x700d5);
	gs101_pcie_phy_pcs_write(phy, PCS_LANE_L2_ENTRY, 0x300ff);
	gs101_pcie_phy_pcs_write(phy, PCS_LANE_L2_PD_DELAY, 0x40);

	/* Gate the ERIO clock in L1.2. */
	gs101_pcie_phy_pcs_rmw(phy, PCS_LANE_MUX, PCS_ERIO_CLK_GATE_MASK,
			       PCS_ERIO_CLK_GATE_L12);

	return 0;
}

static int gs101_pcie_phy_calibrate(struct phy *p)
{
	struct gs101_pcie_phy *phy = phy_get_drvdata(p);
	u32 val;

	val = readl(phy->pma + PMA_CMN_AFC);
	val |= PMA_CMN_AFC_EN;
	val &= ~PMA_CMN_AFC_PREV;
	writel(val, phy->pma + PMA_CMN_AFC);

	/*
	 * Start the AFC calibration from its initial value rather than from
	 * the previous run's result: with the latter the AFC can report done
	 * too early and the CDR fails to lock.
	 */
	writel(PMA_LANE_AFC_CAL_INIT, phy->pma + PMA_LANE_AFC_CAL_MODE);

	return 0;
}

static int gs101_pcie_phy_power_off(struct phy *p)
{
	struct gs101_pcie_phy *phy = phy_get_drvdata(p);
	u32 val;

	gs101_pcie_phy_write_table(phy->pma, gs101_pwrdn,
				   ARRAY_SIZE(gs101_pwrdn));

	val = readl(phy->pma + PMA_CMN_PMA_CTRL);
	val &= ~PMA_CMN_PMA_EN;
	writel(val, phy->pma + PMA_CMN_PMA_CTRL);

	return 0;
}

static int gs101_pcie_phy_exit(struct phy *p)
{
	struct gs101_pcie_phy *phy = phy_get_drvdata(p);
	int ret;

	clk_bulk_disable_unprepare(phy->num_clks, phy->clks);

	ret = regmap_update_bits(phy->pmu, phy->pmu_offset,
				 PMU_PCIE_PHY_ISOLATION_OFF, 0);

	regulator_bulk_disable(ARRAY_SIZE(phy->supplies), phy->supplies);

	return ret;
}

static const struct phy_ops gs101_pcie_phy_ops = {
	.init		= gs101_pcie_phy_init,
	.power_on	= gs101_pcie_phy_power_on,
	.calibrate	= gs101_pcie_phy_calibrate,
	.power_off	= gs101_pcie_phy_power_off,
	.exit		= gs101_pcie_phy_exit,
	.owner		= THIS_MODULE,
};

static int gs101_pcie_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct gs101_pcie_phy *phy;
	struct phy *generic_phy;
	int ret;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	phy->pma = devm_platform_ioremap_resource_byname(pdev, "pma");
	if (IS_ERR(phy->pma))
		return PTR_ERR(phy->pma);

	phy->pcs = devm_platform_ioremap_resource_byname(pdev, "pcs");
	if (IS_ERR(phy->pcs))
		return PTR_ERR(phy->pcs);

	phy->pmu = syscon_regmap_lookup_by_phandle_args(dev->of_node,
							"samsung,pmu-syscon",
							1, &phy->pmu_offset);
	if (IS_ERR(phy->pmu))
		return dev_err_probe(dev, PTR_ERR(phy->pmu),
				     "failed to get the PMU\n");

	phy->num_clks = devm_clk_bulk_get_all(dev, &phy->clks);
	if (phy->num_clks < 0)
		return dev_err_probe(dev, phy->num_clks,
				     "failed to get clocks\n");

	phy->supplies[0].supply = "vdd085";
	phy->supplies[1].supply = "vdd18";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(phy->supplies),
				      phy->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	generic_phy = devm_phy_create(dev, NULL, &gs101_pcie_phy_ops);
	if (IS_ERR(generic_phy))
		return dev_err_probe(dev, PTR_ERR(generic_phy),
				     "failed to create PHY\n");

	phy_set_drvdata(generic_phy, phy);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id gs101_pcie_phy_of_match[] = {
	{ .compatible = "google,gs101-pcie-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs101_pcie_phy_of_match);

static struct platform_driver gs101_pcie_phy_driver = {
	.probe = gs101_pcie_phy_probe,
	.driver = {
		.name = "gs101-pcie-phy",
		.of_match_table = gs101_pcie_phy_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(gs101_pcie_phy_driver);

MODULE_DESCRIPTION("Google gs101 PCIe PHY driver");
MODULE_LICENSE("GPL");
