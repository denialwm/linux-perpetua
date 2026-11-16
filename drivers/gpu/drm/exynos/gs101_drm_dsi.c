// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs101 MIPI DSI host (DSIM).
 *
 * The bootloader trains the D-PHY, sets up the link for its panel in
 * command mode and leaves it running. This host keeps that configuration:
 * it carries DECON's output and sends DCS commands in low-power mode over
 * the link as found, and does not program the PLL, the D-PHY or the lanes.
 * Of the command mode timers it only rescales the TE window, which is a
 * fraction of a frame, when the panel runs at another rate.
 *
 * The register layout is not the one of the samsung-dsim bridge: the
 * offsets differ, and so do CONFIG, FIFOCTRL and SWRST, whose function
 * reset bit there is the D-PHY reset here.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_encoder.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"

#define DSIM_VERSION			0x0000
#define DSIM_LINK_STATUS1		0x000c
#define DSIM_DPHY_STATUS		0x001c
#define DSIM_CLK_CTRL			0x0020
#define DSIM_ESCMODE			0x002c
#define  ESCMODE_CMD_LPDT		BIT(7)
#define DSIM_CONFIG			0x004c
#define  CONFIG_CPRS_EN			BIT(19)
#define  CONFIG_VIDEO_MODE		BIT(18)
#define DSIM_INTSRC			0x0050
#define  INTSRC_SFR_PL_FIFO_EMPTY	BIT(29)
#define  INTSRC_SFR_PH_FIFO_EMPTY	BIT(28)
#define  INTSRC_SFR_PH_FIFO_OVERFLOW	BIT(27)
#define DSIM_PKTHDR			0x0058
#define  PKTHDR_DATA1(x)		((x) << 16)
#define  PKTHDR_DATA0(x)		((x) << 8)
#define  PKTHDR_ID(x)			((x) << 0)
#define DSIM_PAYLOAD			0x005c
#define DSIM_FIFOCTRL			0x0068
#define  FIFOCTRL_FULL_PH_SFR		BIT(11)
#define  FIFOCTRL_FULL_PL_SFR		BIT(9)

#define DSIM_CMD_TE_CTRL1		0x0088
#define  TE_CTRL1_PROTECT_ON		GENMASK(31, 16)
#define  TE_CTRL1_TIMEOUT		GENMASK(15, 0)

#define DSIM_FIFO_TIMEOUT_US		20000

/* The rate the bootloader set the TE window up for. */
#define DSIM_BOOT_VREFRESH		60

struct gs101_dsi {
	struct device *dev;
	void __iomem *regs;
	struct mipi_dsi_host host;
	struct mipi_dsi_device *device;
	struct drm_encoder encoder;
	/* Serialises transfers. */
	struct mutex lock;
	/* The bootloader's TE window, at DSIM_BOOT_VREFRESH. */
	u32 te_protect_on;
	u32 te_timeout;
};

static inline struct gs101_dsi *host_to_dsi(struct mipi_dsi_host *host)
{
	return container_of(host, struct gs101_dsi, host);
}

static const struct component_ops gs101_dsi_component_ops;

static int gs101_dsi_host_attach(struct mipi_dsi_host *host,
				 struct mipi_dsi_device *device)
{
	struct gs101_dsi *dsi = host_to_dsi(host);
	int ret;

	if (dsi->device)
		return -EBUSY;

	dsi->device = device;

	/*
	 * Join the DRM device only once the panel is here. The panel holds
	 * its supplies from probe, so they stay on when DRM removes the
	 * firmware framebuffer, which also held them.
	 */
	ret = component_add(dsi->dev, &gs101_dsi_component_ops);
	if (ret)
		dsi->device = NULL;

	return ret;
}

static int gs101_dsi_host_detach(struct mipi_dsi_host *host,
				 struct mipi_dsi_device *device)
{
	struct gs101_dsi *dsi = host_to_dsi(host);

	if (dsi->device != device)
		return -EINVAL;

	component_del(dsi->dev, &gs101_dsi_component_ops);
	dsi->device = NULL;

	return 0;
}

static int gs101_dsi_wait_fifo(struct gs101_dsi *dsi, u32 full)
{
	u32 val;

	return readl_poll_timeout(dsi->regs + DSIM_FIFOCTRL, val,
				  !(val & full), 10, DSIM_FIFO_TIMEOUT_US);
}

static ssize_t gs101_dsi_host_transfer(struct mipi_dsi_host *host,
				       const struct mipi_dsi_msg *msg)
{
	struct gs101_dsi *dsi = host_to_dsi(host);
	struct mipi_dsi_packet packet;
	const u8 *tx;
	size_t i, j;
	u32 val;
	int ret;

	/* Reads need bus turnaround handling, which nothing uses yet. */
	if (msg->rx_len)
		return -EOPNOTSUPP;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		return ret;

	mutex_lock(&dsi->lock);

	val = readl(dsi->regs + DSIM_ESCMODE);
	writel(val | ESCMODE_CMD_LPDT, dsi->regs + DSIM_ESCMODE);

	writel(INTSRC_SFR_PH_FIFO_EMPTY | INTSRC_SFR_PL_FIFO_EMPTY |
	       INTSRC_SFR_PH_FIFO_OVERFLOW, dsi->regs + DSIM_INTSRC);

	tx = packet.payload;
	for (i = 0; i < packet.payload_length; i += 4) {
		val = 0;
		for (j = 0; j < 4 && i + j < packet.payload_length; j++)
			val |= (u32)tx[i + j] << (8 * j);

		ret = gs101_dsi_wait_fifo(dsi, FIFOCTRL_FULL_PL_SFR);
		if (ret)
			goto out;

		writel(val, dsi->regs + DSIM_PAYLOAD);
	}

	ret = gs101_dsi_wait_fifo(dsi, FIFOCTRL_FULL_PH_SFR);
	if (ret)
		goto out;

	writel(PKTHDR_ID(packet.header[0]) | PKTHDR_DATA0(packet.header[1]) |
	       PKTHDR_DATA1(packet.header[2]), dsi->regs + DSIM_PKTHDR);

	ret = readl_poll_timeout(dsi->regs + DSIM_INTSRC, val,
				 val & INTSRC_SFR_PH_FIFO_EMPTY, 10,
				 DSIM_FIFO_TIMEOUT_US);

out:
	val = readl(dsi->regs + DSIM_ESCMODE);
	writel(val & ~ESCMODE_CMD_LPDT, dsi->regs + DSIM_ESCMODE);

	mutex_unlock(&dsi->lock);

	if (ret) {
		dev_err(dsi->dev, "transfer of type %#04x timed out, INTSRC %#010x\n",
			msg->type, readl(dsi->regs + DSIM_INTSRC));
		return ret;
	}

	return msg->tx_len;
}

static const struct mipi_dsi_host_ops gs101_dsi_host_ops = {
	.attach		= gs101_dsi_host_attach,
	.detach		= gs101_dsi_host_detach,
	.transfer	= gs101_dsi_host_transfer,
};

static const struct drm_encoder_funcs gs101_dsi_encoder_funcs = {
	.destroy	= drm_encoder_cleanup,
};

/*
 * After TE, commands may go until the protect time and the next TE is due
 * by the timeout: 95 % and 110 % of a frame in the vendor's setup. Scale
 * the bootloader's values to the new rate.
 */
static void gs101_dsi_atomic_mode_set(struct drm_encoder *encoder,
				      struct drm_crtc_state *crtc_state,
				      struct drm_connector_state *conn_state)
{
	struct gs101_dsi *dsi = container_of(encoder, struct gs101_dsi,
					     encoder);
	int vrefresh = drm_mode_vrefresh(&crtc_state->adjusted_mode);

	if (vrefresh <= 0 || !dsi->te_protect_on || !dsi->te_timeout)
		return;

	writel(FIELD_PREP(TE_CTRL1_PROTECT_ON,
			  dsi->te_protect_on * DSIM_BOOT_VREFRESH / vrefresh) |
	       FIELD_PREP(TE_CTRL1_TIMEOUT,
			  dsi->te_timeout * DSIM_BOOT_VREFRESH / vrefresh),
	       dsi->regs + DSIM_CMD_TE_CTRL1);
}

static const struct drm_encoder_helper_funcs gs101_dsi_encoder_helper_funcs = {
	.atomic_mode_set	= gs101_dsi_atomic_mode_set,
};

static int gs101_dsi_bind(struct device *dev, struct device *master,
			  void *data)
{
	struct gs101_dsi *dsi = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &dsi->encoder;
	struct drm_device *drm = data;
	struct drm_connector *connector;
	struct exynos_drm_crtc *crtc;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	u32 config;
	int ret;

	config = readl(dsi->regs + DSIM_CONFIG);
	dev_info(dev,
		 "version %#010x link %#010x dphy %#010x clk %#010x esc %#010x config %#010x (%s mode%s)\n",
		 readl(dsi->regs + DSIM_VERSION),
		 readl(dsi->regs + DSIM_LINK_STATUS1),
		 readl(dsi->regs + DSIM_DPHY_STATUS),
		 readl(dsi->regs + DSIM_CLK_CTRL),
		 readl(dsi->regs + DSIM_ESCMODE), config,
		 config & CONFIG_VIDEO_MODE ? "video" : "command",
		 config & CONFIG_CPRS_EN ? ", DSC" : "");

	panel = of_drm_find_panel(dsi->device->dev.of_node);
	if (IS_ERR(panel))
		return PTR_ERR(panel);

	bridge = drmm_panel_bridge_add(drm, panel);
	if (IS_ERR(bridge))
		return PTR_ERR(bridge);

	ret = drm_encoder_init(drm, encoder, &gs101_dsi_encoder_funcs,
			       DRM_MODE_ENCODER_DSI, NULL);
	if (ret)
		return ret;

	drm_encoder_helper_add(encoder, &gs101_dsi_encoder_helper_funcs);

	ret = exynos_drm_set_possible_crtcs(encoder, EXYNOS_DISPLAY_TYPE_LCD);
	if (ret)
		return ret;

	ret = drm_bridge_attach(encoder, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;

	connector = drm_bridge_connector_init(drm, encoder);
	if (IS_ERR(connector))
		return PTR_ERR(connector);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		return ret;

	crtc = exynos_drm_crtc_get_by_type(drm, EXYNOS_DISPLAY_TYPE_LCD);
	if (!IS_ERR(crtc))
		crtc->i80_mode = !(dsi->device->mode_flags & MIPI_DSI_MODE_VIDEO);

	return 0;
}

static void gs101_dsi_unbind(struct device *dev, struct device *master,
			     void *data)
{
}

static const struct component_ops gs101_dsi_component_ops = {
	.bind	= gs101_dsi_bind,
	.unbind	= gs101_dsi_unbind,
};

static int gs101_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_bulk_data *clks;
	struct gs101_dsi *dsi;
	u32 val;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->dev = dev;

	ret = devm_mutex_init(dev, &dsi->lock);
	if (ret)
		return ret;

	dsi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsi->regs))
		return PTR_ERR(dsi->regs);

	/* The registers only answer with the DPU clocks running. */
	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	/* The bootloader's 60 Hz TE window, before any modeset scales it. */
	val = readl(dsi->regs + DSIM_CMD_TE_CTRL1);
	dsi->te_protect_on = FIELD_GET(TE_CTRL1_PROTECT_ON, val);
	dsi->te_timeout = FIELD_GET(TE_CTRL1_TIMEOUT, val);

	platform_set_drvdata(pdev, dsi);

	/*
	 * Register the host now, so that the panel can probe and attach
	 * before the DRM device binds.
	 */
	dsi->host.dev = dev;
	dsi->host.ops = &gs101_dsi_host_ops;

	ret = mipi_dsi_host_register(&dsi->host);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register DSI host\n");

	return 0;
}

static void gs101_dsi_remove(struct platform_device *pdev)
{
	struct gs101_dsi *dsi = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&dsi->host);

	/* Leave the bootloader's TE window for the next probe. */
	writel(FIELD_PREP(TE_CTRL1_PROTECT_ON, dsi->te_protect_on) |
	       FIELD_PREP(TE_CTRL1_TIMEOUT, dsi->te_timeout),
	       dsi->regs + DSIM_CMD_TE_CTRL1);
}

static const struct of_device_id gs101_dsi_of_match[] = {
	{ .compatible = "google,gs101-dsim" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs101_dsi_of_match);

struct platform_driver gs101_dsi_driver = {
	.probe		= gs101_dsi_probe,
	.remove		= gs101_dsi_remove,
	.driver		= {
		.name		= "gs101-dsim",
		.of_match_table	= gs101_dsi_of_match,
	},
};
