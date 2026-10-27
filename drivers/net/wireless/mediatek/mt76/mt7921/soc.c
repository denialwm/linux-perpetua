// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * MT7921 Wi-Fi core of MediaTek SoCs
 *
 * The connectivity subsystem of SoCs such as the MT6897 holds the MT7921's
 * Wi-Fi core and WFDMA at SoC addresses instead of behind PCIe. The
 * bootloader loads the firmware into reserved memory, and the connectivity
 * subsystem's driver powers the subsystem. This backend powers the Wi-Fi
 * MCU, maps the registers and starts the firmware; the rest is the MT7921.
 */

#include <linux/etherdevice.h>
#include <linux/firmware.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/mtk-consys.h>
#include <linux/unaligned.h>

#include "mt7921.h"
#include "../mt76_connac2_mac.h"
#include "../dma.h"
#include "mcu.h"

#define MT7921_SOC_REG_INVALID		U32_MAX

/* The rings the SoC's firmware uses for commands, events and TX-free reports */
#define MT7921_SOC_TXQ_MCU_WM		15
#define MT7921_SOC_RXQ_MCU_WM		2
#define MT7921_SOC_RXQ_TX_FREE		3

/* Conninfra, at its SoC addresses */
#define MT_SOC_INFRA_RGU_WFSYS_PWR	0x18000010
#define MT_SOC_INFRA_RGU_WFSYS_PWR_KEY	GENMASK(31, 16)
#define MT_SOC_INFRA_RGU_WFSYS_PWR_ON	BIT(7)
#define MT_SOC_INFRA_RGU_WFSYS_CPU_RST	0x18000120
#define MT_SOC_INFRA_RGU_WFSYS_CPU_RST_B	BIT(0)
#define MT_SOC_INFRA_RGU_SEMA_RST	0x18000158
#define MT_SOC_INFRA_RGU_SEMA_RST_B	BIT(0)
#define MT_SOC_INFRA_PWRCTRL1		0x18001210
#define MT_SOC_INFRA_PWRCTRL1_RDY	BIT(16)
#define MT_SOC_INFRA_WF_SLP_CTRL	0x18001440
#define MT_SOC_INFRA_WF_SLP_PROT_EN	BIT(0)
#define MT_SOC_INFRA_WF_SLP_STATUS	0x18001444
#define MT_SOC_INFRA_WFDMA2CONN_PROT_RDY	BIT(25)
#define MT_SOC_INFRA_CONN2WF_PROT_RDY	BIT(29)
#define MT_SOC_INFRA_WF2CONN_PROT_HW_EN	BIT(30)
#define MT_SOC_INFRA_WF2CONN_PROT_RDY	BIT(31)
#define MT_SOC_INFRA_TOP_CK_1		0x18003124
#define MT_SOC_INFRA_TOP_CK_1_EN	BIT(0)
#define MT_SOC_INFRA_TOP_CK_1_BSY	BIT(1)
#define MT_SOC_INFRA_VERSION		0x18011000
#define MT_SOC_INFRA_EMI_CTL_WF		0x18011114
#define MT_SOC_INFRA_EMI_CTL_WF_EMI_REQ	BIT(0)
#define MT_SOC_INFRA_EMI_CTL_WF_INFRA_REQ	BIT(5)
#define MT_SOC_INFRA_COEX0_SET		0x18012064
#define MT_SOC_INFRA_COEX0_CLR		0x18012068
#define MT_SOC_INFRA_COEX0_PTA_CKEN	0x01010101
#define MT_SOC_INFRA_COEX1_SET		0x18012074
#define MT_SOC_INFRA_COEX1_CLR		0x18012078
#define MT_SOC_INFRA_COEX1_PTA_CKEN	0x00000101
#define MT_SOC_INFRA_CCIF_ACK		0x1803d014
#define MT_SOC_INFRA_HOST_IRQ_ENA	0x18060018
#define MT_SOC_INFRA_HOST_IRQ_ENA_WFDMA	BIT(0)
#define MT_SOC_INFRA_WAKEUP_WF		0x180601a4
#define MT_SOC_INFRA_WAKEUP_WF_EN	BIT(0)
#define MT_SOC_INFRA_PWR_STATES		0x18060a10
#define MT_SOC_INFRA_PWR_STATES_WF_RST_B	BIT(30)
#define MT_SOC_INFRA_PWR_STATES_WF_AWAKE	BIT(4)
#define MT_SOC_INFRA_WF_MONFLG_EN	0x18060b00
#define MT_SOC_INFRA_WF_MONFLG_SEL	0x18060b04
#define MT_SOC_INFRA_WF_MONFLG_SEL_MASK	GENMASK(4, 0)
#define MT_SOC_INFRA_WF_MONFLG_SEL_ROM	0xc
#define MT_SOC_INFRA_WF_MONFLG_SEL_WAKE	0x6
#define MT_SOC_INFRA_WF_MONFLG_SUBSEL	0x18060b14
#define MT_SOC_INFRA_WF_MONFLG_SUBSEL_MASK	GENMASK(2, 0)
#define MT_SOC_INFRA_WF_MONFLG_OUT	0x18060b10
#define MT_SOC_INFRA_WF_MONFLG_WAKE_LOST	(BIT(10) | BIT(0))
#define MT_SOC_INFRA_WF_MONFLG_ROM_IDLE	0x1d1e

/* WFSYS, at its SoC addresses */
#define MT_SOC_WF_AP2WF_REMAP_1		0x18400120
#define MT_SOC_WF_AP2WF_REMAP_DEBUG_AO	0x810f0000
#define MT_SOC_WF_IP_VERSION		0x184b0010
#define MT_SOC_WF_ROMCODE_INDEX		0x184c1604
#define MT_SOC_WF_SLPPROT_STATUS	0x184c300c
#define MT_SOC_WF_SLPPROT_WFDMA_TX	BIT(23)
#define MT_SOC_WF_SLPPROT_WFDMA_RX	BIT(21)
#define MT_SOC_WF_BUSHANGCR		0x184f0440
#define MT_SOC_WF_BUSHANGCR_LIMIT	GENMASK(7, 0)
#define MT_SOC_WF_BUSHANGCR_DEBUG_EN	BIT(31)
#define MT_SOC_WF_BUSHANGCR_DEBUG_CG_EN	BIT(28)
/* The debug control block, through the AP2WF remap window */
#define MT_SOC_WF_PWA_CTRL0		0x18500000
#define MT_SOC_WF_PWA_TIMEOUT_THRES	GENMASK(31, 16)
#define MT_SOC_WF_PWA_TIMEOUT_CLR	BIT(9)
#define MT_SOC_WF_PWA_DEBUG_EN_TOP	BIT(4)
#define MT_SOC_WF_PWA_DEBUG_CKEN	BIT(3)
#define MT_SOC_WF_PWA_DEBUG_EN		BIT(2)
#define MT_SOC_WF_PWA_CTRL3		0x1850000c
#define MT_SOC_WF_PWA_MASK_BUSY		BIT(8)

struct mt7921_soc_data {
	u16 chip;
	u32 infra_version;
	u32 wf_version;
};

struct mt7921_soc {
	struct mt792x_dev dev;
	const struct mt7921_soc_data *data;
	struct mtk_consys *consys;
	const char *nvram;
	int irq;
};

static struct mt7921_soc *mt7921_soc_get(struct mt792x_dev *dev)
{
	return container_of(dev, struct mt7921_soc, dev);
}

static u32 mt7921_soc_reg_addr(struct mt792x_dev *dev, u32 addr)
{
	static const struct mt76_connac_reg_map fixed_map[] = {
		{ 0x18000000, 0x000000, 0x700000 }, /* the SoC's own view */
		{ 0x7c000000, 0x000000, 0x100000 }, /* CONN_INFRA */
		{ 0x830c0000, 0x400000, 0x001000 }, /* WF_MCU_BUS_CR_REMAP */
		{ 0x54000000, 0x402000, 0x001000 }, /* WFDMA PCIE0 MCU DMA0 */
		{ 0x55000000, 0x403000, 0x001000 }, /* WFDMA PCIE0 MCU DMA1 */
		{ 0x56000000, 0x404000, 0x001000 }, /* WFDMA reserved */
		{ 0x57000000, 0x405000, 0x001000 }, /* WFDMA MCU wrap CR */
		{ 0x58000000, 0x406000, 0x001000 }, /* WFDMA PCIE1 MCU DMA0 */
		{ 0x59000000, 0x407000, 0x001000 }, /* WFDMA PCIE1 MCU DMA1 */
		{ 0x820c0000, 0x408000, 0x004000 }, /* WF_UMAC_TOP (PLE) */
		{ 0x820c8000, 0x40c000, 0x002000 }, /* WF_UMAC_TOP (PSE) */
		{ 0x820cc000, 0x40e000, 0x002000 }, /* WF_UMAC_TOP (PP, MDP) */
		{ 0x83000000, 0x410000, 0x010000 }, /* WF_PHY_MAP3 */
		{ 0x820e0000, 0x420000, 0x000400 }, /* WF_LMAC_TOP BN0 (WF_CFG) */
		{ 0x820e1000, 0x420400, 0x000200 }, /* WF_LMAC_TOP BN0 (WF_TRB) */
		{ 0x820e2000, 0x420800, 0x000400 }, /* WF_LMAC_TOP BN0 (WF_AGG) */
		{ 0x820e3000, 0x420c00, 0x000400 }, /* WF_LMAC_TOP BN0 (WF_ARB) */
		{ 0x820e4000, 0x421000, 0x000400 }, /* WF_LMAC_TOP BN0 (WF_TMAC) */
		{ 0x820e5000, 0x421400, 0x000800 }, /* WF_LMAC_TOP BN0 (WF_RMAC) */
		{ 0x820ce000, 0x421c00, 0x000200 }, /* WF_LMAC_TOP (WF_SEC) */
		{ 0x820e7000, 0x421e00, 0x000200 }, /* WF_LMAC_TOP BN0 (WF_DMA) */
		{ 0x820cf000, 0x422000, 0x001000 }, /* WF_LMAC_TOP (WF_PF) */
		{ 0x820e9000, 0x423400, 0x000200 }, /* WF_LMAC_TOP BN0 (WF_WTBLOFF) */
		{ 0x820ea000, 0x424000, 0x000200 }, /* WF_LMAC_TOP BN0 (WF_ETBF) */
		{ 0x820eb000, 0x424200, 0x000400 }, /* WF_LMAC_TOP BN0 (WF_LPON) */
		{ 0x820ec000, 0x424600, 0x000200 }, /* WF_LMAC_TOP BN0 (WF_INT) */
		{ 0x820ed000, 0x424800, 0x000800 }, /* WF_LMAC_TOP BN0 (WF_MIB) */
		{ 0x820ca000, 0x426000, 0x002000 }, /* WF_LMAC_TOP BN0 (WF_MUCOP) */
		{ 0x820d0000, 0x430000, 0x010000 }, /* WF_LMAC_TOP (WF_WTBLON) */
		{ 0x83080000, 0x450000, 0x010000 }, /* WF_PHY_MAP1 */
		{ 0x83090000, 0x460000, 0x010000 }, /* WF_PHY_MAP2 */
		{ 0xe0200000, 0x470000, 0x010000 }, /* WF_UMAC_SYSRAM */
		{ 0x00400000, 0x480000, 0x010000 }, /* WF_MCU_SYSRAM */
		{ 0x00410000, 0x490000, 0x010000 }, /* WF_MCU_SYSRAM (config) */
		{ 0x820f0000, 0x4a0000, 0x000400 }, /* WF_LMAC_TOP BN1 (WF_CFG) */
		{ 0x820f1000, 0x4a0600, 0x000200 }, /* WF_LMAC_TOP BN1 (WF_TRB) */
		{ 0x820f2000, 0x4a0800, 0x000400 }, /* WF_LMAC_TOP BN1 (WF_AGG) */
		{ 0x820f3000, 0x4a0c00, 0x000400 }, /* WF_LMAC_TOP BN1 (WF_ARB) */
		{ 0x820f4000, 0x4a1000, 0x000400 }, /* WF_LMAC_TOP BN1 (WF_TMAC) */
		{ 0x820f5000, 0x4a1400, 0x000800 }, /* WF_LMAC_TOP BN1 (WF_RMAC) */
		{ 0x820f7000, 0x4a1e00, 0x000200 }, /* WF_LMAC_TOP BN1 (WF_DMA) */
		{ 0x820f9000, 0x4a3400, 0x000200 }, /* WF_LMAC_TOP BN1 (WF_WTBLOFF) */
		{ 0x820fa000, 0x4a4000, 0x000200 }, /* WF_LMAC_TOP BN1 (WF_ETBF) */
		{ 0x820fb000, 0x4a4200, 0x000400 }, /* WF_LMAC_TOP BN1 (WF_LPON) */
		{ 0x820fc000, 0x4a4600, 0x000200 }, /* WF_LMAC_TOP BN1 (WF_INT) */
		{ 0x820fd000, 0x4a4800, 0x000800 }, /* WF_LMAC_TOP BN1 (WF_MIB) */
		{ 0x820c4000, 0x4a8000, 0x004000 }, /* WF_LMAC_TOP BN1 (WF_UWTBL) */
		{ 0x820b0000, 0x4ae000, 0x001000 }, /* [APB2] WFSYS_ON */
		{ 0x80020000, 0x4b0000, 0x010000 }, /* WF_TOP_MISC_OFF */
		{ 0x81020000, 0x4c0000, 0x010000 }, /* WF_TOP_MISC_ON */
		{ 0x80010000, 0x4d4000, 0x001000 }, /* WF_AXIDMA */
		{ 0x83010000, 0x4e0000, 0x010000 }, /* WF_PHY_MAP4 */
		{ 0x88000000, 0x4f0000, 0x010000 }, /* WF_MCU_CFG_LS */
	};
	int i;

	/*
	 * Below 1 MiB, the address is an offset into the MT7921's PCIe BAR,
	 * which mt76 uses for the registers it shares with it. The SoC maps
	 * conninfra at 0 and the Wi-Fi blocks 4 MiB up.
	 */
	if (addr < 0x100000) {
		if (addr >= 0xf0000)		/* CONN_INFRA */
			return addr - 0xf0000;
		if (addr >= 0xe0000)		/* CONN_INFRA, conn_host_csr_top */
			return addr - 0x80000;
		if (addr >= 0xd0000)		/* CONN_INFRA, WFDMA */
			return addr - 0xb0000;
		if (addr < 0x10000 || addr >= 0x20000)	/* not the PCIe MAC */
			return addr + 0x400000;
		goto invalid;
	}

	for (i = 0; i < ARRAY_SIZE(fixed_map); i++) {
		u32 ofs = addr - fixed_map[i].phys;

		if (addr >= fixed_map[i].phys && ofs < fixed_map[i].size)
			return fixed_map[i].maps + ofs;
	}

invalid:
	dev_err_ratelimited(dev->mt76.dev, "Unsupported register %08x\n",
			    addr);

	return MT7921_SOC_REG_INVALID;
}

static u32 mt7921_soc_rr(struct mt76_dev *mdev, u32 offset)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 addr = mt7921_soc_reg_addr(dev, offset);

	if (addr == MT7921_SOC_REG_INVALID)
		return ~0;

	return dev->bus_ops->rr(mdev, addr);
}

static void mt7921_soc_wr(struct mt76_dev *mdev, u32 offset, u32 val)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 addr = mt7921_soc_reg_addr(dev, offset);

	if (addr != MT7921_SOC_REG_INVALID)
		dev->bus_ops->wr(mdev, addr, val);
}

static u32 mt7921_soc_rmw(struct mt76_dev *mdev, u32 offset, u32 mask, u32 val)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 addr = mt7921_soc_reg_addr(dev, offset);

	if (addr == MT7921_SOC_REG_INVALID)
		return ~0;

	return dev->bus_ops->rmw(mdev, addr, mask, val);
}

static int mt7921_soc_wake_conninfra(struct mt7921_soc *soc)
{
	struct mt792x_dev *dev = &soc->dev;

	mt76_set(dev, MT_SOC_INFRA_WAKEUP_WF, MT_SOC_INFRA_WAKEUP_WF_EN);
	/* A ready bit right after the wake-up request can be stale */
	udelay(200);

	if (!mt76_poll_msec_tick(dev, MT_SOC_INFRA_VERSION, ~0,
				 soc->data->infra_version, 10, 1)) {
		dev_err(dev->mt76.dev, "conninfra version %08x\n",
			mt76_rr(dev, MT_SOC_INFRA_VERSION));
		return -ETIMEDOUT;
	}

	if (!mt76_poll(dev, MT_SOC_INFRA_PWRCTRL1, MT_SOC_INFRA_PWRCTRL1_RDY,
		       MT_SOC_INFRA_PWRCTRL1_RDY, 5000)) {
		dev_err(dev->mt76.dev, "conninfra not ready\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void mt7921_soc_release_conninfra(struct mt7921_soc *soc)
{
	mt76_clear(&soc->dev, MT_SOC_INFRA_WAKEUP_WF, MT_SOC_INFRA_WAKEUP_WF_EN);
}

/* Powers the Wi-Fi MCU on and waits for its ROM to idle */
/*
 * Taking ownership back wakes a sleeping MCU through an interrupt, which is
 * lost when the MCU is just falling asleep. MediaTek's driver selects the
 * wake flags and clears the ownership again while the flags show it lost.
 */
static int __mt7921_soc_drv_own(struct mt792x_dev *dev)
{
	int i, try;

	for (i = 0; i < MT792x_DRV_OWN_RETRY_COUNT; i++) {
		for (try = 0; try < 3; try++) {
			u32 state, flag;

			mt76_set(dev, MT_SOC_INFRA_WF_MONFLG_EN, BIT(0));
			mt76_rmw(dev, MT_SOC_INFRA_WF_MONFLG_SEL,
				 MT_SOC_INFRA_WF_MONFLG_SEL_MASK,
				 MT_SOC_INFRA_WF_MONFLG_SEL_WAKE);
			mt76_rmw(dev, MT_SOC_INFRA_WF_MONFLG_SUBSEL,
				 MT_SOC_INFRA_WF_MONFLG_SUBSEL_MASK, 1);
			state = mt76_rr(dev, MT_SOC_INFRA_PWR_STATES);
			mt76_wr(dev, MT_CONN_ON_LPCTL, PCIE_LPCR_HOST_CLR_OWN);
			if (state & MT_SOC_INFRA_PWR_STATES_WF_AWAKE)
				break;

			udelay(156);
			flag = mt76_rr(dev, MT_SOC_INFRA_WF_MONFLG_OUT);
			dev_info(dev->mt76.dev, "EXPERIMENT wake state %08x flag %08x\n",
				 state, flag);
			if ((flag & MT_SOC_INFRA_WF_MONFLG_WAKE_LOST) !=
			    MT_SOC_INFRA_WF_MONFLG_WAKE_LOST)
				break;
		}

		if (mt76_poll_msec_tick(dev, MT_CONN_ON_LPCTL,
					PCIE_LPCR_HOST_OWN_SYNC, 0, 50, 1))
			return 0;
	}

	dev_err(dev->mt76.dev, "driver own failed\n");

	return -EIO;
}

static int mt7921_soc_drv_own(struct mt792x_dev *dev)
{
	struct mt76_connac_pm *pm = &dev->pm;
	int err;

	err = __mt7921_soc_drv_own(dev);
	if (err)
		return err;

	mt792x_wpdma_reinit_cond(dev);
	clear_bit(MT76_STATE_PM, &dev->mphy.state);

	pm->stats.last_wake_event = jiffies;
	pm->stats.doze_time += pm->stats.last_wake_event -
			       pm->stats.last_doze_event;

	return 0;
}

static int mt7921_soc_mcu_power_on(struct mt7921_soc *soc)
{
	struct mt792x_dev *dev = &soc->dev;
	struct device *mdev = dev->mt76.dev;
	int ret;

	ret = mt7921_soc_wake_conninfra(soc);
	if (ret)
		return ret;

	mt76_wr(dev, MT_SOC_INFRA_COEX0_SET, MT_SOC_INFRA_COEX0_PTA_CKEN);
	mt76_wr(dev, MT_SOC_INFRA_COEX1_SET, MT_SOC_INFRA_COEX1_PTA_CKEN);

	/* Hold the MCU in reset while WFSYS powers on */
	mt76_clear(dev, MT_SOC_INFRA_RGU_WFSYS_CPU_RST,
		   MT_SOC_INFRA_RGU_WFSYS_CPU_RST_B);
	mt76_clear(dev, MT_SOC_INFRA_WF_SLP_CTRL, MT_SOC_INFRA_WF_SLP_PROT_EN);
	mt76_rmw(dev, MT_SOC_INFRA_RGU_WFSYS_PWR,
		 MT_SOC_INFRA_RGU_WFSYS_PWR_KEY | MT_SOC_INFRA_RGU_WFSYS_PWR_ON,
		 FIELD_PREP(MT_SOC_INFRA_RGU_WFSYS_PWR_KEY, 0x5746) |
		 MT_SOC_INFRA_RGU_WFSYS_PWR_ON);

	if (!mt76_poll(dev, MT_SOC_INFRA_PWR_STATES,
		       MT_SOC_INFRA_PWR_STATES_WF_RST_B,
		       MT_SOC_INFRA_PWR_STATES_WF_RST_B, 5000)) {
		dev_err(mdev, "WFSYS did not power on\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	/* WFSYS registers hang the bus until the sleep protection is off */
	if (!mt76_poll(dev, MT_SOC_INFRA_WF_SLP_STATUS,
		       MT_SOC_INFRA_WF2CONN_PROT_RDY |
		       MT_SOC_INFRA_CONN2WF_PROT_RDY, 0, 50000) ||
	    !mt76_poll(dev, MT_SOC_WF_SLPPROT_STATUS,
		       MT_SOC_WF_SLPPROT_WFDMA_TX | MT_SOC_WF_SLPPROT_WFDMA_RX,
		       0, 50000)) {
		dev_err(mdev, "WFSYS sleep protection stays on\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	if (!mt76_poll(dev, MT_SOC_WF_IP_VERSION, ~0, soc->data->wf_version,
		       5000)) {
		dev_err(mdev, "WFSYS version %08x\n",
			mt76_rr(dev, MT_SOC_WF_IP_VERSION));
		ret = -ETIMEDOUT;
		goto out;
	}

	/* Bus timeouts, so that a stuck access reports instead of hanging */
	mt76_rmw_field(dev, MT_SOC_WF_BUSHANGCR, MT_SOC_WF_BUSHANGCR_LIMIT, 3);
	mt76_set(dev, MT_SOC_WF_BUSHANGCR, MT_SOC_WF_BUSHANGCR_DEBUG_EN |
		 MT_SOC_WF_BUSHANGCR_DEBUG_CG_EN);

	mt76_wr(dev, MT_SOC_WF_AP2WF_REMAP_1, MT_SOC_WF_AP2WF_REMAP_DEBUG_AO);
	mt76_set(dev, MT_SOC_WF_PWA_CTRL0, MT_SOC_WF_PWA_DEBUG_CKEN);
	mt76_set(dev, MT_SOC_WF_PWA_CTRL0, MT_SOC_WF_PWA_TIMEOUT_CLR);
	mt76_clear(dev, MT_SOC_WF_PWA_CTRL0, MT_SOC_WF_PWA_TIMEOUT_CLR);
	mt76_rmw_field(dev, MT_SOC_WF_PWA_CTRL0, MT_SOC_WF_PWA_TIMEOUT_THRES,
		       0x04f5);
	mt76_set(dev, MT_SOC_WF_PWA_CTRL3, MT_SOC_WF_PWA_MASK_BUSY);
	mt76_set(dev, MT_SOC_WF_PWA_CTRL0, MT_SOC_WF_PWA_DEBUG_EN |
		 MT_SOC_WF_PWA_DEBUG_CKEN | MT_SOC_WF_PWA_DEBUG_EN_TOP);

	/* Let the host read the ROM's progress */
	mt76_set(dev, MT_SOC_INFRA_WF_MONFLG_EN, BIT(0));
	mt76_rmw_field(dev, MT_SOC_INFRA_WF_MONFLG_SEL,
		       MT_SOC_INFRA_WF_MONFLG_SEL_MASK,
		       MT_SOC_INFRA_WF_MONFLG_SEL_ROM);

	mt76_set(dev, MT_SOC_INFRA_RGU_WFSYS_CPU_RST,
		 MT_SOC_INFRA_RGU_WFSYS_CPU_RST_B);

	if (!mt76_poll_msec_tick(dev, MT_SOC_INFRA_WF_MONFLG_OUT, ~0,
				 MT_SOC_INFRA_WF_MONFLG_ROM_IDLE, 1000, 1)) {
		dev_err(mdev, "Wi-Fi ROM did not idle: flag %08x index %08x\n",
			mt76_rr(dev, MT_SOC_INFRA_WF_MONFLG_OUT),
			mt76_rr(dev, MT_SOC_WF_ROMCODE_INDEX));
		ret = -ETIMEDOUT;
		goto out;
	}

	/* The ROM leaves the host with ownership; hand it to the firmware */
	ret = mt792xe_mcu_fw_pmctrl(dev);
	if (!ret)
		ret = __mt7921_soc_drv_own(dev);
out:
	mt7921_soc_release_conninfra(soc);

	return ret;
}

/*
 * Powers the Wi-Fi MCU off. Cutting WFSYS while one of its bus transfers is
 * in flight hangs the SoC's bus, so WFSYS stays on unless the sleep
 * protection isolates it.
 */
static int mt7921_soc_mcu_power_off(struct mt7921_soc *soc)
{
	struct mt792x_dev *dev = &soc->dev;
	struct device *mdev = dev->mt76.dev;
	u32 val;
	int ret;

	/* MediaTek gives the firmware ownership back before powering off */
	mt792xe_mcu_fw_pmctrl(dev);

	ret = mt7921_soc_wake_conninfra(soc);
	if (ret)
		return ret;

	mt76_set(dev, MT_SOC_INFRA_WF_SLP_CTRL, MT_SOC_INFRA_WF_SLP_PROT_EN);
	if (!mt76_poll(dev, MT_SOC_INFRA_WF_SLP_STATUS,
		       MT_SOC_INFRA_WFDMA2CONN_PROT_RDY,
		       MT_SOC_INFRA_WFDMA2CONN_PROT_RDY, 50000) ||
	    read_poll_timeout(mt76_rr, val,
			      val & (MT_SOC_INFRA_WF2CONN_PROT_RDY |
				     MT_SOC_INFRA_WF2CONN_PROT_HW_EN),
			      500, 50000, false, dev,
			      MT_SOC_INFRA_WF_SLP_STATUS)) {
		dev_err(mdev,
			"WFSYS cannot be isolated (sleep protection %08x, WFDMA %08x), leaving it on\n",
			mt76_rr(dev, MT_SOC_INFRA_WF_SLP_STATUS),
			mt76_rr(dev, MT_WFDMA0_GLO_CFG));
		mt76_clear(dev, MT_SOC_INFRA_WF_SLP_CTRL,
			   MT_SOC_INFRA_WF_SLP_PROT_EN);
		mt76_poll(dev, MT_SOC_INFRA_WF_SLP_STATUS,
			  MT_SOC_INFRA_WF2CONN_PROT_RDY |
			  MT_SOC_INFRA_CONN2WF_PROT_RDY, 0, 50000);
		ret = -EBUSY;
		goto out;
	}

	mt76_rmw(dev, MT_SOC_INFRA_RGU_WFSYS_PWR,
		 MT_SOC_INFRA_RGU_WFSYS_PWR_KEY | MT_SOC_INFRA_RGU_WFSYS_PWR_ON,
		 FIELD_PREP(MT_SOC_INFRA_RGU_WFSYS_PWR_KEY, 0x5746));
	if (!mt76_poll(dev, MT_SOC_INFRA_PWR_STATES,
		       MT_SOC_INFRA_PWR_STATES_WF_RST_B, 0, 5000)) {
		dev_err(mdev, "WFSYS did not power off\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	/* Release what the MCU held: semaphores, CCIF, A-die clock, EMI */
	mt76_clear(dev, MT_SOC_INFRA_RGU_SEMA_RST, MT_SOC_INFRA_RGU_SEMA_RST_B);
	mt76_set(dev, MT_SOC_INFRA_RGU_SEMA_RST, MT_SOC_INFRA_RGU_SEMA_RST_B);
	mt76_set(dev, MT_SOC_INFRA_CCIF_ACK, 0xff);

	mt76_clear(dev, MT_SOC_INFRA_TOP_CK_1, MT_SOC_INFRA_TOP_CK_1_EN);
	if (!mt76_poll(dev, MT_SOC_INFRA_TOP_CK_1, MT_SOC_INFRA_TOP_CK_1_BSY,
		       0, 50000))
		dev_warn(mdev, "A-die clock stays busy\n");

	mt76_set(dev, MT_SOC_INFRA_EMI_CTL_WF, MT_SOC_INFRA_EMI_CTL_WF_EMI_REQ);
	mt76_clear(dev, MT_SOC_INFRA_EMI_CTL_WF,
		   MT_SOC_INFRA_EMI_CTL_WF_EMI_REQ);
	mt76_set(dev, MT_SOC_INFRA_EMI_CTL_WF,
		 MT_SOC_INFRA_EMI_CTL_WF_INFRA_REQ);
	mt76_clear(dev, MT_SOC_INFRA_EMI_CTL_WF,
		   MT_SOC_INFRA_EMI_CTL_WF_INFRA_REQ);

	mt76_wr(dev, MT_SOC_INFRA_COEX0_CLR, MT_SOC_INFRA_COEX0_PTA_CKEN);
	mt76_wr(dev, MT_SOC_INFRA_COEX1_CLR, MT_SOC_INFRA_COEX1_PTA_CKEN);
out:
	mt7921_soc_release_conninfra(soc);

	return ret;
}

static int mt7921_soc_wfsys_reset(struct mt792x_dev *dev)
{
	struct mt7921_soc *soc = mt7921_soc_get(dev);
	int ret;

	/* Stop the DMA first, as MediaTek does before powering WFSYS off */
	mt792x_dma_disable(dev, false);

	ret = mt7921_soc_mcu_power_off(soc);
	if (ret) {
		/* Nothing reaches the firmware until a reboot powers it off */
		atomic_set(&dev->mt76.bus_hung, true);
		return ret;
	}

	return mt7921_soc_mcu_power_on(soc);
}

/*
 * The board's NVRAM: a 256-byte header with the MAC address, then the
 * calibration as records of a type, a 16-bit length and data, until an
 * empty one. The firmware takes the header with the first record, then
 * one record per command, each padded to 1500 bytes.
 */
#define MT_SOC_NVRAM_HDR_LEN		256
#define MT_SOC_NVRAM_MAC		4
#define MT_SOC_NVRAM_REC_HDR_LEN	3
#define MT_SOC_NVRAM_CMD_LEN		1500

/*
 * Before the firmware starts, the ROM calibrates the radio from a PHY action:
 * the request, the NVRAM records it needs, and the front-end modules and
 * their pins. A board without external modules describes none.
 */
#define MT_SOC_PHY_ACTION_MAGIC		0x556789aa
#define MT_SOC_PHY_ACTION_NVRAM_MAX	512
#define MT_SOC_PHY_CAL_FORCE		0x01
#define MT_SOC_PHY_CAL_FORCE_RSP	0x81

enum {
	MT_SOC_PHY_TAG_NVRAM,
	MT_SOC_PHY_TAG_CAL,
	MT_SOC_PHY_TAG_FEM,
	MT_SOC_PHY_TAG_LAA,
};

struct mt7921_soc_phy_hdr {
	__le32 magic;
	u8 tags;
	u8 version;
	__le16 len;
} __packed;

struct mt7921_soc_phy_tlv {
	__le16 tag;
	__le16 len;
} __packed;

struct mt7921_soc_phy_cal {
	u8 cmd;
	u8 save;
	u8 skip;
	u8 rsv;
} __packed;

struct mt7921_soc_phy_fem {
	__le32 id;
	__le32 pins;
	u8 pin[32][3];
} __packed;

struct mt7921_soc_phy_laa {
	__le32 chip;
	__le32 pins;
	u8 pin[8][4];
} __packed;

struct mt7921_soc_phy_rsp {
	struct mt7921_soc_phy_hdr hdr;
	struct mt7921_soc_phy_tlv tlv;
	u8 event;
	u8 status;
	u8 rsv[2];
	__le32 emi_addr;
	__le32 emi_len;
	__le32 temperature;
} __packed;

static void *mt7921_soc_phy_tag(u8 **pos, u16 tag, u16 len)
{
	struct mt7921_soc_phy_tlv *tlv = (void *)*pos;

	tlv->tag = cpu_to_le16(tag);
	tlv->len = cpu_to_le16(len);
	*pos += sizeof(*tlv) + len;

	return tlv + 1;
}

/* The common and system records, and any the NVRAM flags for it */
static bool mt7921_soc_phy_record(u8 id)
{
	return id == 7 || id == 8 || id == 9 || id == 21 || (id & 0x80);
}

static int mt7921_soc_phy_action(struct mt792x_dev *dev,
				 const struct firmware *fw)
{
	size_t pos = MT_SOC_NVRAM_HDR_LEN, nvram_len = 0;
	struct mt7921_soc_phy_cal *cal;
	struct mt7921_soc_phy_rsp *rsp;
	struct mt7921_soc_phy_hdr *hdr;
	struct sk_buff *skb;
	u8 *req, *p, *nvram;
	size_t len;
	int ret;

	len = sizeof(*hdr) + 4 * sizeof(struct mt7921_soc_phy_tlv) +
	      sizeof(*cal) + MT_SOC_PHY_ACTION_NVRAM_MAX +
	      sizeof(struct mt7921_soc_phy_fem) +
	      sizeof(struct mt7921_soc_phy_laa);
	req = kzalloc(len, GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	hdr = (void *)req;
	p = req + sizeof(*hdr);

	cal = mt7921_soc_phy_tag(&p, MT_SOC_PHY_TAG_CAL, sizeof(*cal));
	cal->cmd = MT_SOC_PHY_CAL_FORCE;

	nvram = p + sizeof(struct mt7921_soc_phy_tlv);
	while (pos + MT_SOC_NVRAM_REC_HDR_LEN <= fw->size) {
		const u8 *rec = fw->data + pos;
		size_t rec_len = MT_SOC_NVRAM_REC_HDR_LEN +
				 get_unaligned_le16(rec + 1);

		if (!rec[0] && rec_len == MT_SOC_NVRAM_REC_HDR_LEN)
			break;

		pos += rec_len;
		if (pos > fw->size)
			break;

		if (!mt7921_soc_phy_record(rec[0]))
			continue;

		if (nvram_len + rec_len > MT_SOC_PHY_ACTION_NVRAM_MAX) {
			ret = -E2BIG;
			goto out;
		}

		memcpy(nvram + nvram_len, rec, rec_len);
		nvram_len += rec_len;
	}
	mt7921_soc_phy_tag(&p, MT_SOC_PHY_TAG_NVRAM, ALIGN(nvram_len, 4));

	mt7921_soc_phy_tag(&p, MT_SOC_PHY_TAG_FEM,
			   sizeof(struct mt7921_soc_phy_fem));
	mt7921_soc_phy_tag(&p, MT_SOC_PHY_TAG_LAA,
			   sizeof(struct mt7921_soc_phy_laa));

	hdr->magic = cpu_to_le32(MT_SOC_PHY_ACTION_MAGIC);
	hdr->tags = 4;
	hdr->version = 1;
	hdr->len = cpu_to_le16(p - req - sizeof(*hdr));

	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_CMD(PHY_ACTION), req,
					p - req, true, &skb);
	if (ret)
		goto out;

	/* The parser dropped a full event header, longer than an init one */
	rsp = skb_push(skb, 4);
	if (skb->len < sizeof(*rsp) ||
	    le32_to_cpu(rsp->hdr.magic) != MT_SOC_PHY_ACTION_MAGIC ||
	    rsp->event != MT_SOC_PHY_CAL_FORCE_RSP || rsp->status) {
		dev_err(dev->mt76.dev, "Calibration failed: event %#x status %u\n",
			rsp->event, rsp->status);
		ret = -EIO;
	}
	dev_kfree_skb(skb);
out:
	kfree(req);

	return ret;
}

static int mt7921_soc_start_firmware(struct mt792x_dev *dev)
{
	struct mt7921_soc *soc = mt7921_soc_get(dev);
	struct {
		__le32 option;
		__le32 addr;
	} req = {};
	int ret;

	if (soc->nvram) {
		const struct firmware *fw;

		ret = request_firmware(&fw, soc->nvram, dev->mt76.dev);
		if (ret)
			return ret;

		ret = mt7921_soc_phy_action(dev, fw);
		release_firmware(fw);
		if (ret)
			return ret;
	}

	/* The bootloader loaded the firmware; the ROM only starts it */
	ret = mt76_mcu_send_msg(&dev->mt76, MCU_CMD(FW_IMAGE_START), &req,
				sizeof(req), true);
	if (ret) {
		struct mt76_queue *cmd = dev->mt76.q_mcu[MT_MCUQ_WM];
		struct mt76_queue *evt = &dev->mt76.q_rx[MT_RXQ_MCU];

		dev_err(dev->mt76.dev,
			"Firmware start failed: %d, command ring %u/%u, event ring %u/%u, interrupts %#x\n",
			ret, readl(&cmd->regs->cpu_idx),
			readl(&cmd->regs->dma_idx), readl(&evt->regs->cpu_idx),
			readl(&evt->regs->dma_idx),
			mt76_rr(dev, MT_WFDMA0_HOST_INT_STA));
		return ret < 0 ? ret : -EIO;
	}

	if (!mt76_poll_msec(dev, MT_CONN_ON_MISC, MT_TOP_MISC2_FW_N9_RDY,
			    MT_TOP_MISC2_FW_N9_RDY, 1500)) {
		dev_err(dev->mt76.dev, "Timeout for initializing firmware\n");
		return -EIO;
	}

	/* EXPERIMENT: DMASHDL has pages only once the firmware runs */
	mt76_clear(dev, MT_DMASHDL_SW_CONTROL, MT_DMASHDL_DMASHDL_BYPASS);
	mt76_set(dev, MT_WFDMA0_GLO_CFG_EXT0,
		 MT_WFDMA0_GLO_CFG_EXT0_CSR_TX_DMASHDL_EN);

	return 0;
}

static int mt7921_soc_load_calibration(struct mt792x_dev *dev)
{
	struct mt7921_soc *soc = mt7921_soc_get(dev);
	struct device *mdev = dev->mt76.dev;
	size_t start = 0, pos = MT_SOC_NVRAM_HDR_LEN;
	const struct firmware *fw;
	u8 *cmd;
	int ret;

	if (!soc->nvram)
		return 0;

	/* EXPERIMENT */
	dev_info(mdev, "EXPERIMENT reg read %08x\n",
		 mt76_connac_mcu_reg_rr(&dev->mt76, MT_PSE_BASE));

	ret = request_firmware(&fw, soc->nvram, mdev);
	if (ret)
		return ret;

	cmd = kmalloc(MT_SOC_NVRAM_CMD_LEN, GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	while (pos + MT_SOC_NVRAM_REC_HDR_LEN <= fw->size) {
		const u8 *rec = fw->data + pos;
		size_t len = get_unaligned_le16(rec + 1);

		if (!rec[0] && !len)
			break;

		pos += MT_SOC_NVRAM_REC_HDR_LEN + len;
		if (pos > fw->size || pos - start > MT_SOC_NVRAM_CMD_LEN) {
			dev_err(mdev, "Malformed NVRAM record at %zu\n",
				rec - fw->data);
			ret = -EINVAL;
			goto out;
		}

		memcpy(cmd, fw->data + start, pos - start);
		memset(cmd + pos - start, 0, start + MT_SOC_NVRAM_CMD_LEN - pos);
		ret = mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SET_NVRAM_SETTINGS),
					cmd, MT_SOC_NVRAM_CMD_LEN, false);
		if (ret)
			goto out;

		start = pos;
	}

	if (!start) {
		dev_err(mdev, "NVRAM has no calibration\n");
		ret = -EINVAL;
		goto out;
	}


	if (is_valid_ether_addr(fw->data + MT_SOC_NVRAM_MAC))
		ether_addr_copy(dev->mphy.macaddr, fw->data + MT_SOC_NVRAM_MAC);
out:
	kfree(cmd);
	release_firmware(fw);

	return ret;
}

#define MT_SOC_TX_RING_EXT_CTRL(n)	MT_WFDMA0(0x600 + (n) * 4)
#define MT_SOC_RX_RING_EXT_CTRL(n)	MT_WFDMA0(0x680 + (n) * 4)
#define MT_SOC_PREFETCH(base, depth)	((base) << 16 | (depth))

/* The host rings, then the modem's, in MediaTek's order */
static void mt7921_soc_dma_prefetch(struct mt792x_dev *dev)
{
	static const u8 rx_rings[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	static const u8 tx_rings[] = { 0, 1, 2, 15, 16, 8, 9, 10, 14 };
	u32 base = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(rx_rings); i++, base += 0x40)
		mt76_wr(dev, MT_SOC_RX_RING_EXT_CTRL(rx_rings[i]),
			MT_SOC_PREFETCH(base, 0x4));

	for (i = 0; i < ARRAY_SIZE(tx_rings); i++, base += 0x40)
		mt76_wr(dev, MT_SOC_TX_RING_EXT_CTRL(tx_rings[i]),
			MT_SOC_PREFETCH(base, 0x4));

	/* EXPERIMENT: DMASHDL quotas as MediaTek sets them on soc7_0 */
	mt76_rmw(dev, MT_DMASHDL_PKT_MAX_SIZE,
		 MT_DMASHDL_PKT_MAX_SIZE_PLE | MT_DMASHDL_PKT_MAX_SIZE_PSE,
		 FIELD_PREP(MT_DMASHDL_PKT_MAX_SIZE_PLE, 0x1) |
		 FIELD_PREP(MT_DMASHDL_PKT_MAX_SIZE_PSE, 0x18));
	mt76_rmw(dev, MT_DMASHDL_REFILL, MT_DMASHDL_REFILL_MASK, 0xfff00000);
	for (i = 0; i < 16; i++) {
		u32 max = i < 3 ? 0x200 : i == 3 ? 0x1ff : i == 15 ? 0x30 : 0;
		u32 min = i < 4 ? 0x10 : i == 15 ? 0x30 : 0;

		mt76_wr(dev, MT_DMASHDL_GROUP_QUOTA(i),
			FIELD_PREP(MT_DMASHDL_GROUP_QUOTA_MIN, min) |
			FIELD_PREP(MT_DMASHDL_GROUP_QUOTA_MAX, max));
	}
	mt76_wr(dev, MT_DMASHDL_Q_MAP(0), 0x21112000);
	mt76_wr(dev, MT_DMASHDL_Q_MAP(1), 0x21112000);
	mt76_wr(dev, MT_DMASHDL_Q_MAP(2), 0x11111000);
	mt76_wr(dev, MT_DMASHDL_Q_MAP(3), 0x11111000);
	mt76_wr(dev, MT_DMASHDL_SCHED_SET(0), 0x76543210);
	mt76_wr(dev, MT_DMASHDL_SCHED_SET(1), 0xfedcba98);
	mt76_clear(dev, MT_DMASHDL_PAGE, MT_DMASHDL_GROUP_SEQ_ORDER);
	mt76_rmw(dev, MT_DMASHDL_OPTIONAL, GENMASK(23, 0), 0x00048007);
}

/*
 * The firmware reports itself sleepy once it is idle. MediaTek's driver then
 * hands it ownership and takes ownership back before the next command; the
 * firmware leaves commands unanswered until it has been woken that way.
 */
static int mt7921_soc_mcu_send_message(struct mt76_dev *mdev,
				       struct sk_buff *skb, int cmd, int *seq)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	int ret;

	/* The firmware ignores unified commands: fail them at once */
	if (dev->fw_legacy_cmd && (cmd & __MCU_CMD_FIELD_UNI)) {
		dev_kfree_skb(skb);
		return -EOPNOTSUPP;
	}

	if (READ_ONCE(dev->fw_sleepy)) {
		WRITE_ONCE(dev->fw_sleepy, false);
		mt792xe_mcu_fw_pmctrl(dev);
		ret = __mt7921_soc_drv_own(dev);
		if (ret) {
			dev_kfree_skb(skb);
			return ret;
		}
	}

	if (cmd == MCU_CE_CMD(START_HW_SCAN) && skb->len == 1186 &&
	    skb_tailroom(skb) >= 6) {
		/* EXPERIMENT: firmware's naturally aligned layout (1192):
		 * ext_ssids/bssid/random_mac 2 bytes later, source mask 1164
		 */
		u8 *d = skb->data;

		skb_put_zero(skb, 6);
		memmove(d + 896, d + 894, 1122 - 894);
		d[894] = 0;
		d[895] = 0;
		memset(d + 1124, 0, 1192 - 1124);
		d[1164] = 1;
		d[6] &= ~SCAN_FUNC_SPLIT_SCAN;
		d[6] |= BIT(1); /* no DBDC scan */
		dev_info(mdev->dev, "EXPERIMENT scan %*ph type %u n %u len %u\n",
			 8, d, d[158], d[159], skb->len);
	}

	if (cmd == MCU_CE_CMD(FWLOG_2_HOST) ||
	    cmd == MCU_CE_CMD(CHIP_CONFIG) ||
	    cmd == MCU_CE_CMD(SET_RATE_TX_POWER)) { /* EXPERIMENT */
		dev_kfree_skb(skb);
		return 0;
	}

	{ /* EXPERIMENT: one command in the DMA at a time */
		struct mt76_queue *q = mdev->q_mcu[MT_MCUQ_WM];
		u32 didx;

		if (read_poll_timeout(readl, didx,
				      didx == readl(&q->regs->cpu_idx),
				      100, 200000, false, &q->regs->dma_idx))
		{
			int i;

			dev_warn(mdev->dev, "EXPERIMENT command ring stuck %u/%u int %08x\n",
				 readl(&q->regs->cpu_idx), didx,
				 mt76_rr(dev, MT_WFDMA0_HOST_INT_STA));
			dev_warn(mdev->dev, "EXPERIMENT lpctl %08x\n",
				 mt76_rr(dev, MT_CONN_ON_LPCTL));
			mt792xe_mcu_fw_pmctrl(dev);
			i = __mt7921_soc_drv_own(dev);
			dev_warn(mdev->dev, "EXPERIMENT round trip %d lpctl %08x ring %u/%u\n",
				 i, mt76_rr(dev, MT_CONN_ON_LPCTL),
				 readl(&q->regs->cpu_idx), readl(&q->regs->dma_idx));
			mt76_for_each_q_rx(mdev, i)
				dev_warn(mdev->dev, "EXPERIMENT rx %d ndesc %u cpu %u dma %u\n",
					 i, mdev->q_rx[i].ndesc,
					 readl(&mdev->q_rx[i].regs->cpu_idx),
					 readl(&mdev->q_rx[i].regs->dma_idx));
		}
		dev_info(mdev->dev, "EXPERIMENT cmd %08x len %u slot %u pc %08x shdl %08x %08x %08x sw %08x int %08x/%08x ccif %08x\n",
			 cmd, skb->len, readl(&q->regs->cpu_idx),
			 mt76_rr(dev, 0x7c023610), mt76_rr(dev, 0x7c026100),
			 mt76_rr(dev, 0x7c026140), mt76_rr(dev, 0x7c02617c),
			 mt76_rr(dev, MT_MCU_CMD), mt76_rr(dev, MT_WFDMA0_HOST_INT_STA),
			 mt76_rr(dev, MT_WFDMA0_HOST_INT_ENA),
			 mt76_rr(dev, MT_SOC_INFRA_CCIF_ACK - 4));
	}

	ret = mt76_connac2_mcu_fill_message(mdev, skb, cmd, seq);
	if (ret)
		return ret;

	{ /* EXPERIMENT: the command header as MediaTek's driver fills it */
		struct mt76_connac2_mcu_txd *txd = (void *)skb->data;

		txd->txd[0] &= ~cpu_to_le32(MT_TXD0_Q_IDX);
		txd->txd[1] &= ~cpu_to_le32(MT_TXD1_LONG_FORMAT);
		txd->pq_id = 0;
	}

	/* The ROM calibrates the radio before it answers a PHY action */
	mdev->mcu.timeout = cmd == MCU_CMD(PHY_ACTION) ? 5 * HZ : 3 * HZ;

	return mt76_tx_queue_skb_raw(dev, mdev->q_mcu[MT_MCUQ_WM], skb, 0);
}

static int mt7921_soc_mcu_init(struct mt792x_dev *dev)
{
	static const struct mt76_mcu_ops mt7921_soc_mcu_ops = {
		.headroom = sizeof(struct mt76_connac2_mcu_txd),
		.mcu_skb_send_msg = mt7921_soc_mcu_send_message,
		.mcu_parse_response = mt7921_mcu_parse_response,
	};
	int err;

	dev->mt76.mcu_ops = &mt7921_soc_mcu_ops;

	err = __mt7921_soc_drv_own(dev);
	if (err)
		return err;

	return mt7921_run_firmware(dev);
}

static int mt7921_soc_init_reset(struct mt792x_dev *dev)
{
	return mt792x_wpdma_reset(dev, true);
}

static int mt7921_soc_mac_reset(struct mt792x_dev *dev)
{
	int i, err;

	mt792xe_mcu_drv_pmctrl(dev);

	mt76_connac_free_pending_tx_skbs(&dev->pm, NULL);

	mt76_wr(dev, dev->irq_map->host_irq_enable, 0);
	mt76_wr(dev, MT_SOC_INFRA_HOST_IRQ_ENA, 0);

	set_bit(MT76_MCU_RESET, &dev->mphy.state);
	wake_up(&dev->mt76.mcu.wait);
	skb_queue_purge(&dev->mt76.mcu.res_q);

	mt76_txq_schedule_all(&dev->mphy);

	mt76_worker_disable(&dev->mt76.tx_worker);
	mt76_for_each_q_rx(&dev->mt76, i)
		napi_disable(&dev->mt76.napi[i]);
	napi_disable(&dev->mt76.tx_napi);

	mt76_connac2_tx_token_put(&dev->mt76);
	idr_init(&dev->mt76.token);

	mt792x_wpdma_reset(dev, true);

	mt76_for_each_q_rx(&dev->mt76, i)
		napi_enable(&dev->mt76.napi[i]);

	local_bh_disable();
	mt76_for_each_q_rx(&dev->mt76, i)
		napi_schedule(&dev->mt76.napi[i]);
	local_bh_enable();

	dev->fw_assert = false;
	clear_bit(MT76_MCU_RESET, &dev->mphy.state);

	mt76_wr(dev, dev->irq_map->host_irq_enable,
		dev->irq_map->tx.all_complete_mask |
		dev->irq_map->rx.all_complete_mask |
		MT_INT_MCU_CMD);
	mt76_wr(dev, MT_SOC_INFRA_HOST_IRQ_ENA, MT_SOC_INFRA_HOST_IRQ_ENA_WFDMA);

	err = mt7921_soc_mcu_init(dev);
	if (err)
		goto out;

	err = mt7921_mcu_set_eeprom(dev);
	if (err)
		goto out;

	err = mt7921_mac_init(dev);
	if (err)
		goto out;

	err = __mt7921_start(&dev->phy);
out:
	napi_enable(&dev->mt76.tx_napi);
	local_bh_disable();
	napi_schedule(&dev->mt76.tx_napi);
	local_bh_enable();

	mt76_worker_enable(&dev->mt76.tx_worker);

	return err;
}

/*
 * mt76 reaches the ring registers through the mapped window directly, not
 * through the register accessors, so they take the SoC's offsets of the
 * host WFDMA.
 */
#define MT_SOC_TX_RING_BASE		0x24300
#define MT_SOC_RX_RING_BASE		0x24500

static int mt7921_soc_dma_init(struct mt792x_dev *dev)
{
	static const struct mt792x_dma_layout layout = {
		.tx_data0 = mt792x_dma_ring(MT7921_TXQ_BAND0,
					    MT7921_TX_RING_SIZE,
					    MT_SOC_TX_RING_BASE),
		.tx_mcu = mt792x_dma_ring(MT7921_SOC_TXQ_MCU_WM,
					  MT7921_TX_MCU_RING_SIZE,
					  MT_SOC_TX_RING_BASE),
		.tx_fwdl = mt792x_dma_ring(MT7921_TXQ_FWDL,
					   MT7921_TX_FWDL_RING_SIZE,
					   MT_SOC_TX_RING_BASE),
		.tx_done = mt792x_dma_ring(MT7921_SOC_RXQ_TX_FREE,
					   MT7921_RX_MCU_WA_RING_SIZE,
					   MT_SOC_RX_RING_BASE),
		.rx_data = mt792x_dma_ring(MT7921_RXQ_BAND0, MT7921_RX_RING_SIZE,
					   MT_SOC_RX_RING_BASE),
		.rx_mcu = mt792x_dma_ring(MT7921_SOC_RXQ_MCU_WM,
					  MT7921_RX_MCU_WA_RING_SIZE,
					  MT_SOC_RX_RING_BASE),
	};
	int ret;

	dev_info(dev->mt76.dev, "EXPERIMENT reset dmashdl ctrl %08x ext0 %08x glo %08x opt %08x\n",
		 mt76_rr(dev, MT_DMASHDL_SW_CONTROL),
		 mt76_rr(dev, MT_WFDMA0_GLO_CFG_EXT0),
		 mt76_rr(dev, MT_WFDMA0_GLO_CFG),
		 mt76_rr(dev, MT_DMASHDL_OPTIONAL));

	ret = mt792x_dma_alloc_queues(dev, &layout);
	if (ret)
		return ret;

	ret = mt76_init_queues(dev, mt792x_poll_rx);
	if (ret < 0)
		return ret;

	netif_napi_add_tx(dev->mt76.tx_napi_dev, &dev->mt76.tx_napi,
			  mt792x_poll_tx);
	napi_enable(&dev->mt76.tx_napi);

	return mt792x_dma_enable(dev);
}

static int mt7921_soc_unregister_device(struct mt7921_soc *soc)
{
	struct mt792x_dev *dev = &soc->dev;
	struct mt76_connac_pm *pm = &dev->pm;
	struct ieee80211_hw *hw = mt76_hw(dev);
	int i, ret;

	if (dev->phy.chip_cap & MT792x_CHIP_CAP_WF_RF_PIN_CTRL_EVT_EN)
		wiphy_rfkill_stop_polling(hw->wiphy);

	cancel_work_sync(&dev->init_work);
	mt76_unregister_device(&dev->mt76);
	mt76_for_each_q_rx(&dev->mt76, i)
		napi_disable(&dev->mt76.napi[i]);
	cancel_delayed_work_sync(&pm->ps_work);
	cancel_work_sync(&pm->wake_work);
	cancel_work_sync(&dev->reset_work);

	mt76_connac2_tx_token_put(&dev->mt76);
	__mt792x_mcu_drv_pmctrl(dev);
	mt792x_dma_cleanup(dev);
	ret = mt7921_soc_mcu_power_off(soc);
	skb_queue_purge(&dev->mt76.mcu.res_q);

	tasklet_disable(&dev->mt76.irq_tasklet);

	return ret;
}

static int mt7921_soc_probe(struct platform_device *pdev)
{
	static const struct mt76_driver_ops drv_ops = {
		/* txwi_size = txd size + txp size */
		.txwi_size = MT_TXD_SIZE + sizeof(struct mt76_connac_hw_txp),
		.drv_flags = MT_DRV_TXWI_NO_FREE | MT_DRV_HW_MGMT_TXQ |
			     MT_DRV_AMSDU_OFFLOAD,
		.survey_flags = SURVEY_INFO_TIME_TX |
				SURVEY_INFO_TIME_RX |
				SURVEY_INFO_TIME_BSS_RX,
		.token_size = MT7921_TOKEN_SIZE,
		.tx_prepare_skb = mt7921e_tx_prepare_skb,
		.tx_complete_skb = mt76_connac_tx_complete_skb,
		.rx_check = mt7921_rx_check,
		.rx_skb = mt7921_queue_rx_skb,
		.rx_poll_complete = mt792x_rx_poll_complete,
		.sta_add = mt7921_mac_sta_add,
		.sta_event = mt7921_mac_sta_event,
		.sta_remove = mt7921_mac_sta_remove,
		.update_survey = mt792x_update_channel,
		.set_channel = mt7921_set_channel,
	};
	static const struct mt792x_hif_ops mt7921_soc_ops = {
		.init_reset = mt7921_soc_init_reset,
		.reset = mt7921_soc_mac_reset,
		.mcu_init = mt7921_soc_mcu_init,
		.drv_own = mt7921_soc_drv_own,
		.fw_own = mt792xe_mcu_fw_pmctrl,
		.wfsys_reset = mt7921_soc_wfsys_reset,
		.start_firmware = mt7921_soc_start_firmware,
		.dma_prefetch = mt7921_soc_dma_prefetch,
		.load_calibration = mt7921_soc_load_calibration,
	};
	static const struct mt792x_irq_map irq_map = {
		.host_irq_enable = MT_WFDMA0_HOST_INT_ENA,
		.tx = {
			.all_complete_mask = MT_INT_TX_DONE_ALL,
			.mcu_complete_mask = MT_INT_TX_DONE_MCU,
		},
		.rx = {
			.all_complete_mask = HOST_RX_DONE_INT_ENA0 |
					     HOST_RX_DONE_INT_ENA2 |
					     HOST_RX_DONE_INT_ENA3,
			.data_complete_mask = HOST_RX_DONE_INT_ENA0,
			.wm_complete_mask = HOST_RX_DONE_INT_ENA2,
			.wm2_complete_mask = HOST_RX_DONE_INT_ENA3,
		},
	};
	struct device *pdev_dev = &pdev->dev;
	struct mt76_bus_ops *bus_ops;
	struct mtk_consys *consys;
	struct mt7921_soc *soc;
	struct mt792x_dev *dev;
	struct mt76_dev *mdev;
	void __iomem *regs;
	int irq, ret;

	consys = mtk_consys_get(pdev_dev);
	if (IS_ERR(consys))
		return dev_err_probe(pdev_dev, PTR_ERR(consys),
				     "Not in a connectivity subsystem\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	ret = dma_set_mask_and_coherent(pdev_dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	mdev = mt76_alloc_device(pdev_dev, sizeof(*soc), &mt7921_ops, &drv_ops);
	if (!mdev)
		return -ENOMEM;

	dev = container_of(mdev, struct mt792x_dev, mt76);
	soc = mt7921_soc_get(dev);
	soc->data = of_device_get_match_data(pdev_dev);
	soc->consys = consys;
	soc->irq = irq;
	of_property_read_string(pdev_dev->of_node, "firmware-name", &soc->nvram);

	/* The firmware in memory runs the channel manager */
	dev->fw_features = MT792x_FW_CAP_CNM;
	dev->hif_ops = &mt7921_soc_ops;
	dev->fw_legacy_cmd = true;
	dev->irq_map = &irq_map;
	mt76_mmio_init(&dev->mt76, regs);
	tasklet_init(&mdev->irq_tasklet, mt792x_irq_tasklet, (unsigned long)dev);

	dev->phy.dev = dev;
	dev->phy.mt76 = &dev->mt76.phy;
	dev->mt76.phy.priv = &dev->phy;
	dev->bus_ops = dev->mt76.bus;
	bus_ops = devm_kmemdup(pdev_dev, dev->bus_ops, sizeof(*bus_ops),
			       GFP_KERNEL);
	if (!bus_ops) {
		ret = -ENOMEM;
		goto err_free_dev;
	}

	bus_ops->rr = mt7921_soc_rr;
	bus_ops->wr = mt7921_soc_wr;
	bus_ops->rmw = mt7921_soc_rmw;
	dev->mt76.bus = bus_ops;

	ret = mtk_consys_power_on(soc->consys, MTK_CONSYS_WIFI);
	if (ret)
		goto err_free_dev;

	ret = mt7921_soc_mcu_power_on(soc);
	if (ret)
		goto err_consys;

	mdev->rev = (soc->data->chip << 16) |
		    (mt76_rr(dev, MT_SOC_WF_IP_VERSION) & 0xff);
	dev_info(pdev_dev, "ASIC revision: %08x\n", mdev->rev);

	mt76_wr(dev, irq_map.host_irq_enable, 0);
	/* Route the WFDMA interrupt to the host */
	mt76_wr(dev, MT_SOC_INFRA_HOST_IRQ_ENA, MT_SOC_INFRA_HOST_IRQ_ENA_WFDMA);

	ret = devm_request_irq(pdev_dev, soc->irq, mt792x_irq_handler, 0,
			       KBUILD_MODNAME, dev);
	if (ret)
		goto err_mcu;

	ret = mt7921_soc_dma_init(dev);
	if (ret)
		goto err_free_irq;

	ret = mt7921_register_device(dev);
	if (ret)
		goto err_free_irq;

	return 0;

err_free_irq:
	devm_free_irq(pdev_dev, soc->irq, dev);
err_mcu:
	if (mt7921_soc_mcu_power_off(soc))
		goto err_free_dev;
err_consys:
	mtk_consys_power_off(soc->consys, MTK_CONSYS_WIFI);
err_free_dev:
	mt76_free_device(&dev->mt76);

	return ret;
}

static void mt7921_soc_remove(struct platform_device *pdev)
{
	struct mt76_dev *mdev = platform_get_drvdata(pdev);
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	struct mt7921_soc *soc = mt7921_soc_get(dev);
	int ret;

	ret = mt7921_soc_unregister_device(soc);
	set_bit(MT76_REMOVED, &dev->mphy.state);
	devm_free_irq(&pdev->dev, soc->irq, dev);
	/* A WFSYS that could not be powered off keeps the subsystem on */
	if (!ret)
		mtk_consys_power_off(soc->consys, MTK_CONSYS_WIFI);
	mt76_free_device(&dev->mt76);
}

static const struct mt7921_soc_data mt6897_wifi_data = {
	.chip = 0x7961,
	.infra_version = 0x02050400,
	.wf_version = 0x02040200,
};

static const struct of_device_id mt7921_soc_of_match[] = {
	{ .compatible = "mediatek,mt6897-wifi", .data = &mt6897_wifi_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt7921_soc_of_match);

struct platform_driver mt7921_soc_driver = {
	.driver = {
		.name = "mt7921-soc",
		.of_match_table = mt7921_soc_of_match,
	},
	.probe = mt7921_soc_probe,
	.remove = mt7921_soc_remove,
};
