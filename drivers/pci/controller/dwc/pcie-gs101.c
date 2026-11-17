// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe host controller driver for the Google gs101 SoC
 *
 * gs101 has two DesignWare PCIe Gen4 root complexes, each with its own
 * two-lane PHY. The vendor's driver was read as documentation for the
 * bring-up order and the ELBI registers; no code is taken from it.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/pci-pwrctrl.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#include "../../pci.h"
#include "pcie-designware.h"

#define to_gs101_pcie(x)	dev_get_drvdata((x)->dev)

/* ELBI registers */
#define PCIE_IRQ2			0x008
#define PCIE_IRQ2_EN			0x018
#define   PCIE_IRQ2_MSI			BIT(17)
#define PCIE_APP_LTSSM_ENABLE		0x054
#define   PCIE_LTSSM_ENABLE		BIT(0)
#define PCIE_ELBI_LTSSM_STATE		0x2c8
#define   PCIE_LTSSM_STATE		GENMASK(5, 0)
#define PCIE_APP_REQ_EXIT_L1		0x3bc
#define   PCIE_APP_REQ_EXIT_L1_MODE	BIT(0)
#define   PCIE_L1_REQ_NAK_CONTROL	BIT(4)
/* The PHY's reset inputs, active low */
#define PCIE_PHY_CMN_RST_N		0x1400
#define PCIE_PHY_INIT_RST_N		0x1404
#define PCIE_PHY_PORT_RST_N		0x1408

/* LTSSM states in which the data link is up */
#define LTSSM_RCVRY_LOCK		0x0d
#define LTSSM_L1_IDLE			0x14

struct gs101_pcie {
	struct dw_pcie pci;
	struct clk_bulk_data *clks;
	int num_clks;
	struct phy *phy;
	struct gpio_desc *perst;
	int irq;
	/* the controller is clocked and its IRQ2 can be read */
	bool irq_ready;
};

static u32 gs101_pcie_readl(struct gs101_pcie *pcie, u32 reg)
{
	return readl(pcie->pci.elbi_base + reg);
}

static void gs101_pcie_writel(struct gs101_pcie *pcie, u32 val, u32 reg)
{
	writel(val, pcie->pci.elbi_base + reg);
}

static void gs101_pcie_phy_resets(struct gs101_pcie *pcie, u32 val)
{
	gs101_pcie_writel(pcie, val, PCIE_PHY_INIT_RST_N);
	gs101_pcie_writel(pcie, val, PCIE_PHY_PORT_RST_N);
	gs101_pcie_writel(pcie, val, PCIE_PHY_CMN_RST_N);
}

static int gs101_pcie_phy_on(struct gs101_pcie *pcie)
{
	int ret;

	ret = phy_init(pcie->phy);
	if (ret)
		return ret;

	/* Pulse all PHY resets, then take the PHY out of its init reset. */
	gs101_pcie_phy_resets(pcie, 1);
	gs101_pcie_phy_resets(pcie, 0);
	usleep_range(10, 20);
	gs101_pcie_writel(pcie, 1, PCIE_PHY_INIT_RST_N);
	usleep_range(10, 20);

	ret = phy_power_on(pcie->phy);
	if (ret)
		goto err_exit;

	gs101_pcie_writel(pcie, 1, PCIE_PHY_CMN_RST_N);
	gs101_pcie_writel(pcie, 1, PCIE_PHY_PORT_RST_N);

	ret = phy_calibrate(pcie->phy);
	if (ret)
		goto err_power_off;

	return 0;

err_power_off:
	phy_power_off(pcie->phy);
err_exit:
	phy_exit(pcie->phy);
	return ret;
}

static void gs101_pcie_phy_off(struct gs101_pcie *pcie)
{
	phy_power_off(pcie->phy);
	phy_exit(pcie->phy);
}

static bool gs101_pcie_link_up(struct dw_pcie *pci)
{
	struct gs101_pcie *pcie = to_gs101_pcie(pci);
	u32 state;

	state = FIELD_GET(PCIE_LTSSM_STATE,
			  gs101_pcie_readl(pcie, PCIE_ELBI_LTSSM_STATE));

	return state >= LTSSM_RCVRY_LOCK && state <= LTSSM_L1_IDLE;
}

static int gs101_pcie_start_link(struct dw_pcie *pci)
{
	gs101_pcie_writel(to_gs101_pcie(pci), PCIE_LTSSM_ENABLE,
			  PCIE_APP_LTSSM_ENABLE);

	return 0;
}

static void gs101_pcie_stop_link(struct dw_pcie *pci)
{
	gs101_pcie_writel(to_gs101_pcie(pci), 0, PCIE_APP_LTSSM_ENABLE);
}

/*
 * The MSI receiver's output reaches the GIC only through IRQ2, which
 * latches its rising edge. That output is the OR of all unmasked pending
 * vectors, so a vector that becomes pending while another is being
 * handled raises no new edge, and would never interrupt again. Mask and
 * unmask all vectors after handling them, so that a vector still pending
 * raises a new edge.
 */
static void gs101_pcie_msi_rearm(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	unsigned int i, num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;

	guard(raw_spinlock_irqsave)(&pp->lock);

	for (i = 0; i < num_ctrls; i++) {
		u32 reg = PCIE_MSI_INTR0_MASK + i * MSI_REG_CTRL_BLOCK_SIZE;

		dw_pcie_writel_dbi(pci, reg, ~0);
		dw_pcie_readl_dbi(pci, reg);
		dw_pcie_writel_dbi(pci, reg, pp->irq_mask[i]);
	}
}

static irqreturn_t gs101_pcie_irq_handler(int irq, void *arg)
{
	struct gs101_pcie *pcie = arg;
	struct dw_pcie_rp *pp = &pcie->pci.pp;
	u32 val;

	/*
	 * The line is shared and requested before the host is up: the ELBI
	 * registers are not readable until host_init has enabled the clocks.
	 */
	if (!READ_ONCE(pcie->irq_ready))
		return IRQ_NONE;

	val = gs101_pcie_readl(pcie, PCIE_IRQ2);
	if (!val)
		return IRQ_NONE;

	gs101_pcie_writel(pcie, val, PCIE_IRQ2);

	if (val & PCIE_IRQ2_MSI) {
		dw_handle_msi_irq(pp);
		gs101_pcie_msi_rearm(pp);
	}

	return IRQ_HANDLED;
}

static int gs101_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct gs101_pcie *pcie = to_gs101_pcie(pci);
	u32 val;
	int ret;

	gpiod_set_value_cansleep(pcie->perst, 1);

	ret = clk_bulk_prepare_enable(pcie->num_clks, pcie->clks);
	if (ret)
		return ret;

	ret = pci_pwrctrl_create_devices(pci->dev);
	if (ret)
		goto err_clk;

	ret = pci_pwrctrl_power_on_devices(pci->dev);
	if (ret)
		goto err_pwrctrl_destroy;

	ret = gs101_pcie_phy_on(pcie);
	if (ret)
		goto err_pwrctrl_off;

	gpiod_set_value_cansleep(pcie->perst, 0);
	msleep(PCIE_T_PVPERL_MS);

	val = gs101_pcie_readl(pcie, PCIE_APP_REQ_EXIT_L1);
	val |= PCIE_APP_REQ_EXIT_L1_MODE | PCIE_L1_REQ_NAK_CONTROL;
	gs101_pcie_writel(pcie, val, PCIE_APP_REQ_EXIT_L1);

	val = gs101_pcie_readl(pcie, PCIE_IRQ2_EN);
	gs101_pcie_writel(pcie, val | PCIE_IRQ2_MSI, PCIE_IRQ2_EN);

	WRITE_ONCE(pcie->irq_ready, true);

	return 0;

err_pwrctrl_off:
	pci_pwrctrl_power_off_devices(pci->dev);
err_pwrctrl_destroy:
	if (ret != -EPROBE_DEFER)
		pci_pwrctrl_destroy_devices(pci->dev);
err_clk:
	clk_bulk_disable_unprepare(pcie->num_clks, pcie->clks);
	return ret;
}

static void gs101_pcie_host_deinit(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct gs101_pcie *pcie = to_gs101_pcie(pci);

	gs101_pcie_writel(pcie, 0, PCIE_IRQ2_EN);
	WRITE_ONCE(pcie->irq_ready, false);
	synchronize_irq(pcie->irq);
	gpiod_set_value_cansleep(pcie->perst, 1);
	gs101_pcie_phy_off(pcie);
	pci_pwrctrl_power_off_devices(pci->dev);
	pci_pwrctrl_destroy_devices(pci->dev);
	clk_bulk_disable_unprepare(pcie->num_clks, pcie->clks);
}

static const struct dw_pcie_host_ops gs101_pcie_host_ops = {
	.init = gs101_pcie_host_init,
	.deinit = gs101_pcie_host_deinit,
};

static const struct dw_pcie_ops gs101_pcie_ops = {
	.link_up = gs101_pcie_link_up,
	.start_link = gs101_pcie_start_link,
	.stop_link = gs101_pcie_stop_link,
};

static int gs101_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs101_pcie *pcie;
	struct dw_pcie_rp *pp;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->pci.dev = dev;
	pcie->pci.ops = &gs101_pcie_ops;
	pp = &pcie->pci.pp;
	pp->ops = &gs101_pcie_host_ops;
	platform_set_drvdata(pdev, pcie);

	pcie->pci.elbi_base = devm_platform_ioremap_resource_byname(pdev, "elbi");
	if (IS_ERR(pcie->pci.elbi_base))
		return PTR_ERR(pcie->pci.elbi_base);

	pcie->num_clks = devm_clk_bulk_get_all(dev, &pcie->clks);
	if (pcie->num_clks < 0)
		return dev_err_probe(dev, pcie->num_clks,
				     "failed to get clocks\n");

	pcie->phy = devm_phy_get(dev, NULL);
	if (IS_ERR(pcie->phy))
		return dev_err_probe(dev, PTR_ERR(pcie->phy),
				     "failed to get the PHY\n");

	pcie->perst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pcie->perst))
		return dev_err_probe(dev, PTR_ERR(pcie->perst),
				     "failed to get the PERST# GPIO\n");

	/*
	 * The controller's interrupts, MSI among them, share one line and
	 * are told apart in IRQ2. Demultiplex the MSIs here rather than in a
	 * chained handler of the DesignWare core. The root port's services
	 * (AER, PME, bandwidth control) share the line too.
	 */
	pcie->irq = platform_get_irq(pdev, 0);
	if (pcie->irq < 0)
		return pcie->irq;

	ret = devm_request_irq(dev, pcie->irq, gs101_pcie_irq_handler,
			       IRQF_SHARED, dev_name(dev), pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the IRQ\n");

	pp->msi_irq[0] = -ENODEV;

	return dw_pcie_host_init(pp);
}

static const struct of_device_id gs101_pcie_of_match[] = {
	{ .compatible = "google,gs101-pcie" },
	{ }
};

static struct platform_driver gs101_pcie_driver = {
	.probe = gs101_pcie_probe,
	.driver = {
		.name = "gs101-pcie",
		.of_match_table = gs101_pcie_of_match,
		.suppress_bind_attrs = true,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
builtin_platform_driver(gs101_pcie_driver);
