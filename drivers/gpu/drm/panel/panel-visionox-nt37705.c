// SPDX-License-Identifier: GPL-2.0-only
/*
 * Visionox NT37705 1080x2412 120 Hz DSC command mode AMOLED panel, as fitted
 * to the Nothing Phone (2).
 *
 * The bootloader powers the panel and runs its init sequence; this driver
 * keeps that state and never resets the panel or cuts its supplies. Screen
 * off is the panel's sleep mode, which keeps its registers.
 *
 * Based on Danila Tikhonov's 6.11 port.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

static const struct regulator_bulk_data nt37705_supplies[] = {
	{ .supply = "vddio" },
	{ .supply = "vdd" },
	{ .supply = "vci" },
};

struct nt37705 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	bool powered;
};

static inline struct nt37705 *to_nt37705(struct drm_panel *panel)
{
	return container_of(panel, struct nt37705, panel);
}

static int nt37705_off(struct nt37705 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);

	return dsi_ctx.accum_err;
}

static int nt37705_prepare(struct drm_panel *panel)
{
	struct nt37705 *ctx = to_nt37705(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	/* The supplies stay on from the first prepare: the panel needs them. */
	if (!ctx->powered) {
		ret = regulator_bulk_enable(ARRAY_SIZE(nt37705_supplies),
					    ctx->supplies);
		if (ret)
			return ret;
		ctx->powered = true;
	}

	/* Tell the decoder what the encoder does, then wake the panel. */
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
	mipi_dsi_compression_mode_multi(&dsi_ctx, true);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int nt37705_unprepare(struct drm_panel *panel)
{
	return nt37705_off(to_nt37705(panel));
}

/*
 * Command mode: the panel refreshes from its own memory and the horizontal
 * blanking only sets the link rate. This blanking gives the stock rate,
 * 897 Mbit/s per lane (stock porches of 100/8/92 would ask for 1076).
 */
static const struct drm_display_mode nt37705_mode = {
	.clock = (1080 + 54 + 8 + 56) * (2412 + 14 + 2 + 22) * 120 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 54,
	.hsync_end = 1080 + 54 + 8,
	.htotal = 1080 + 54 + 8 + 56,
	.vdisplay = 2412,
	.vsync_start = 2412 + 14,
	.vsync_end = 2412 + 14 + 2,
	.vtotal = 2412 + 14 + 2 + 22,
	.width_mm = 69,
	.height_mm = 156,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int nt37705_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &nt37705_mode);
}

static const struct drm_panel_funcs nt37705_panel_funcs = {
	.prepare = nt37705_prepare,
	.unprepare = nt37705_unprepare,
	.get_modes = nt37705_get_modes,
};

static int nt37705_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret < 0 ? ret : 0;
}

static const struct backlight_ops nt37705_bl_ops = {
	.update_status = nt37705_bl_update_status,
};

static struct backlight_device *nt37705_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 2047,
		.max_brightness = 4095,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &nt37705_bl_ops, &props);
}

static int nt37705_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt37705 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct nt37705, panel,
				   &nt37705_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(nt37705_supplies),
					    nt37705_supplies, &ctx->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	/* Held out of reset: the panel stays black after a reset. */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB101010;
	/* Both slices of a line travel in one packet, as the stock tree says. */
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT |
			  MIPI_DSI_MODE_DSC_TABLE_E5;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = nt37705_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	/* The panel only takes DSC: 10 bits per component, 8 bits per pixel. */
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 12;
	ctx->dsc.slice_width = 540;
	ctx->dsc.slice_count = 1080 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 10;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;
	dsi->dsc = &ctx->dsc;

	drm_panel_add(&ctx->panel);

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void nt37705_remove(struct mipi_dsi_device *dsi)
{
	struct nt37705 *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id nt37705_of_match[] = {
	{ .compatible = "visionox,nt37705" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt37705_of_match);

static struct mipi_dsi_driver nt37705_driver = {
	.probe = nt37705_probe,
	.remove = nt37705_remove,
	.driver = {
		.name = "panel-visionox-nt37705",
		.of_match_table = nt37705_of_match,
	},
};
module_mipi_dsi_driver(nt37705_driver);

MODULE_DESCRIPTION("DRM driver for the Visionox NT37705 DSC command mode AMOLED panel");
MODULE_LICENSE("GPL");
