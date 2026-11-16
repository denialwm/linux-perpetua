// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S6E3FC3 DDIC with a 1080x2400 AMOLED panel, driven in DSI command
 * mode, as fitted to the Google Pixel 6.
 *
 * The bootloader powers the panel up, configures it (DSC among the rest)
 * and leaves it displaying from its own frame memory. The panel is taken
 * over in that state: its supplies are held from probe, so that they stay
 * on when the firmware framebuffer that also held them goes away, and the
 * reset line is left as it is.
 */

#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

static const char * const s6e3fc3_supplies[] = { "vci", "vddi" };

struct s6e3fc3 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset;
};

static const struct drm_display_mode s6e3fc3_mode = {
	.clock = 168498,
	.hdisplay = 1080,
	.hsync_start = 1080 + 32,
	.hsync_end = 1080 + 32 + 12,
	.htotal = 1080 + 32 + 12 + 26,
	.vdisplay = 2400,
	.vsync_start = 2400 + 12,
	.vsync_end = 2400 + 12 + 4,
	.vtotal = 2400 + 12 + 4 + 26,
	.width_mm = 67,
	.height_mm = 148,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int s6e3fc3_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &s6e3fc3_mode);
}

static const struct drm_panel_funcs s6e3fc3_panel_funcs = {
	.get_modes = s6e3fc3_get_modes,
};

static int s6e3fc3_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e3fc3 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e3fc3, panel,
				   &s6e3fc3_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	/* Already on: this takes the references that keep them on. */
	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(s6e3fc3_supplies),
					     s6e3fc3_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");

	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset),
				     "failed to get reset GPIO\n");

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void s6e3fc3_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3fc3 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id s6e3fc3_of_match[] = {
	{ .compatible = "samsung,s6e3fc3" },
	{ }
};
MODULE_DEVICE_TABLE(of, s6e3fc3_of_match);

static struct mipi_dsi_driver s6e3fc3_driver = {
	.probe = s6e3fc3_probe,
	.remove = s6e3fc3_remove,
	.driver = {
		.name = "panel-samsung-s6e3fc3",
		.of_match_table = s6e3fc3_of_match,
	},
};
module_mipi_dsi_driver(s6e3fc3_driver);

MODULE_DESCRIPTION("Samsung S6E3FC3 DSI panel driver");
MODULE_LICENSE("GPL");
