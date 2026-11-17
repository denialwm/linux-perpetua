// SPDX-License-Identifier: GPL-2.0-only
/*
 * Visionox NT37705 1080x2412 120 Hz DSC command mode AMOLED panel, as fitted
 * to the Nothing Phone (2).
 *
 * The init sequence is the stock device tree's
 * (dsi-panel-nt37705-visionox-dsc-fhd-plus-120hz-cmd), through
 * linux-mdss-dsi-panel-driver-generator and Danila Tikhonov's 6.11 port.
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

/*
 * Keep the panel as the bootloader set it up: no reset and no init sequence
 * (the panel stays black after Linux resets it, cause unknown, see
 * devices/pong/README.md). The bootloader always lights it at boot. Screen
 * off is then the backlight at zero: the panel is never powered down, so
 * there is no way back from a real power-off, and unprepare does nothing.
 */
#define NT37705_KEEP_BOOTLOADER_STATE 1

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

static void nt37705_reset(struct nt37705 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
}

static int nt37705_on(struct nt37705 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x80, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x87, 0x01, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
				     0x00, 0x00, 0x00, 0x55, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1,
				     0x30, 0x0f, 0x36, 0x64, 0x3b, 0x19, 0x8e, 0x9c,
				     0x3f, 0xc6, 0x26, 0x08, 0x3f, 0xc3, 0xfa, 0x9c,
				     0x00, 0x0f, 0x3d, 0x5e, 0x64);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1,
				     0x21, 0x00, 0x00, 0x7b, 0x00, 0x00, 0x82, 0x33,
				     0xf0, 0x7e, 0x85, 0x3e, 0x76, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1,
				     0x22, 0x19, 0x1f, 0x00, 0x21, 0x2d, 0x00, 0x00,
				     0x00, 0x21, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2,
				     0x36, 0x00, 0x36, 0x64, 0x3b, 0x19, 0x71, 0x64,
				     0x00, 0x05, 0x95, 0xe0, 0x3f, 0x8d, 0x7f, 0x90,
				     0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2,
				     0x21, 0x34, 0xbc, 0x37, 0x00, 0x00, 0x82, 0x33,
				     0x30, 0x82, 0x85, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2,
				     0x22, 0x19, 0x1f, 0x00, 0x4e, 0x5a, 0x00, 0x00,
				     0x00, 0x21, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3,
				     0x2a, 0x0f, 0x36, 0x64, 0x3b, 0x19, 0x8e, 0x9c,
				     0x00, 0x3f, 0x2a, 0xa8, 0x00, 0x36, 0x7b, 0x0c,
				     0x7f, 0xd5, 0x18, 0x25, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3,
				     0x60, 0x34, 0xbc, 0x37, 0x89, 0xe9, 0x6b, 0x33,
				     0x0f, 0x82, 0x7b, 0xc1, 0x8a, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x60, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4,
				     0x2c, 0x00, 0x36, 0x64, 0x3b, 0x19, 0x71, 0x64,
				     0x3f, 0x8c, 0x91, 0x40, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4,
				     0x60, 0x00, 0x00, 0x7b, 0x89, 0xe9, 0x6b, 0x33,
				     0xc0, 0x7e, 0x7b, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x60, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5,
				     0x21, 0x00, 0x03, 0x49, 0x03, 0x49, 0x00, 0x00,
				     0x3f, 0xff, 0x41, 0x76, 0x3f, 0xff, 0x41, 0x76,
				     0x00, 0x00, 0x0a, 0xca, 0xd1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5,
				     0x08, 0x12, 0xfe, 0x1b, 0x00, 0x20, 0x3d, 0x33,
				     0xf0, 0xe3, 0xe3, 0x03, 0x49, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x08, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6,
				     0x27, 0x00, 0x03, 0x49, 0x03, 0x49, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x41, 0x76,
				     0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6,
				     0x08, 0x22, 0x1c, 0x39, 0x00, 0x20, 0x3d, 0x33,
				     0x30, 0x1d, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x08, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc7,
				     0x2b, 0x00, 0x03, 0x49, 0x03, 0x49, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x7f, 0xff, 0xf5, 0x35, 0x2f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc7,
				     0x14, 0x22, 0x1c, 0x39, 0x00, 0x3e, 0x5b, 0x33,
				     0x0f, 0x1d, 0x1d, 0xfc, 0xb7, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc7,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x14, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc8,
				     0x2d, 0x00, 0x03, 0x49, 0x03, 0x49, 0x00, 0x00,
				     0x3f, 0xff, 0x41, 0x76, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc8,
				     0x14, 0x12, 0xfe, 0x1b, 0x00, 0x3e, 0x5b, 0x33,
				     0xc0, 0xe3, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc8,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x14, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01,
				     0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x3c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x00, 0x01, 0x01, 0x00, 0x00, 0x03, 0x03, 0x00,
				     0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x4c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x00, 0x01, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00,
				     0x00, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x5c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x6c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x01, 0x02, 0x00, 0x04, 0x0b, 0x77, 0x03, 0x0b,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x7c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x01, 0x02, 0x00, 0x04, 0x0b, 0x77, 0x03, 0x0b,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x8c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x9c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x11, 0x11, 0x70, 0x13, 0x70, 0x03, 0x70, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0xa4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba, 0x00, 0xf6, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0xa8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x01, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x01, 0x1d, 0x1d, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x01, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x15, 0x15, 0x15, 0xdd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x02, 0x32, 0x22, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x02, 0x32, 0x22, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3,
				     0xdd, 0x06, 0x22, 0x0e, 0xf4, 0x00, 0x06, 0x20,
				     0x0e, 0xff, 0x00, 0x03, 0xd0, 0x0e, 0x05, 0x2c,
				     0x13, 0x03, 0xd0, 0x0e, 0x05, 0x2c, 0x13, 0x03,
				     0xd0, 0x0e, 0x05, 0x2c, 0x13, 0x03, 0xd0, 0x0e,
				     0x05, 0x2c, 0x13, 0x03, 0xd0, 0x0e, 0x05, 0x2c,
				     0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x55);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x19);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x28);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x81);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfe, 0x34);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf9, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb,
				     0x00, 0x03, 0x04, 0x55, 0x77, 0x77, 0x77, 0x99,
				     0x9b, 0x10, 0x00, 0x1e, 0x48, 0x9a, 0xbb, 0xbc,
				     0xde, 0xf0, 0x11, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x19);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfd, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x83);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x12);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfe, 0x41);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x31);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf8, 0x00, 0xfd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf8, 0x01, 0x4f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc7, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x17, 0x10);

	mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, 0x0000, 0x0437);

	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0x0000, 0x096b);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_GAMMA_CURVE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2f, 0x00);

	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0xbb0d);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x04);

	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0xff0f);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0xc5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6d, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x03, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91,
				     0xab, 0x28, 0x00, 0x0c, 0xc2, 0x00, 0x02, 0x0e,
				     0x01, 0x1f, 0x00, 0x07, 0x08, 0xbb, 0x08, 0x7a,
				     0x10, 0xf0);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0, 0x01);

	return dsi_ctx.accum_err;
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

	if (NT37705_KEEP_BOOTLOADER_STATE && ctx->powered)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(nt37705_supplies), ctx->supplies);
	if (ret)
		return ret;
	ctx->powered = true;

	if (NT37705_KEEP_BOOTLOADER_STATE) {
		/* Keep the panel's state, but tell its decoder what the encoder does. */
		ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
		drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
		mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
		mipi_dsi_compression_mode_multi(&dsi_ctx, true);
		return dsi_ctx.accum_err;
	}

	nt37705_reset(ctx);

	ret = nt37705_on(ctx);
	if (ret)
		goto err;

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
	mipi_dsi_compression_mode_multi(&dsi_ctx, true);
	mipi_dsi_msleep(&dsi_ctx, 28);
	ret = dsi_ctx.accum_err;
	if (ret)
		goto err;

	/* Bring-up: does the panel answer, and is it awake and showing? */
	{
		u8 mode = 0;
		int r = mipi_dsi_dcs_get_power_mode(ctx->dsi, &mode);

		dev_info(panel->dev, "power mode read %d: 0x%02x\n", r, mode);
	}

	return 0;

err:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(nt37705_supplies), ctx->supplies);
	return ret;
}

static int nt37705_unprepare(struct drm_panel *panel)
{
	struct nt37705 *ctx = to_nt37705(panel);

	if (NT37705_KEEP_BOOTLOADER_STATE)
		return 0;

	ctx->powered = false;
	nt37705_off(ctx);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(nt37705_supplies), ctx->supplies);

	return 0;
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

/*
 * Bring-up: write "HEX HEX..." to send one DCS write in LP mode, or
 * "r HEX N" to read N bytes of a DCS register (answer in the kernel log).
 */
static ssize_t dcs_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	u8 data[64];
	unsigned int v, n = 0, len = 0;
	int ret, used;
	const char *p = buf;
	bool read = false;

	if (*p == 'r') {
		read = true;
		p++;
	}
	while (n < sizeof(data) && sscanf(p, " %x%n", &v, &used) == 1) {
		data[n++] = v;
		p += used;
	}
	if (!n)
		return -EINVAL;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	if (read) {
		len = n > 1 ? min_t(unsigned int, data[1], sizeof(data)) : 1;
		ret = mipi_dsi_dcs_read(dsi, data[0], data, len);
		dev_info(dev, "dcs read: %d: %*ph\n", ret, (int)len, data);
	} else {
		ret = mipi_dsi_dcs_write_buffer(dsi, data, n);
		dev_info(dev, "dcs write %*ph: %d\n", (int)n, data, ret);
	}

	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_WO(dcs);

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

	ctx->reset_gpio = devm_gpiod_get(dev, "reset",
					 NT37705_KEEP_BOOTLOADER_STATE ?
					 GPIOD_OUT_LOW : GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB101010;
	/* Both slices of a line travel in one packet, as the stock tree says. */
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;

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
	device_create_file(dev, &dev_attr_dcs);

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
