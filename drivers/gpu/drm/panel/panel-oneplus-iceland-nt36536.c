// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36536 DriverIC panels driver
 *
 * Copyright (c) 2026 map220v <map220v300@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

#define DSI_NUM_MIN 1

/*
 * The error flags are sampled at the stock kernel's interval. A raised flag
 * is sampled again shortly after, and only a flag raised at both samples
 * resets the pipe, so that a glitch on the line cannot blank a working
 * panel. Resets that do not clear the flag stop after a few tries.
 */
#define ESD_CHECK_INTERVAL_MS	5000
#define ESD_RECHECK_MS		500
#define ESD_MAX_RESETS		3

struct panel_info {
	struct drm_panel panel;
	struct drm_connector *connector;
	struct mipi_dsi_device *dsi[2];
	struct panel_desc *desc;
	enum drm_panel_orientation orientation;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[3];

	/*
	 * Out of reset since the bootloader, with its picture: until the first
	 * prepare.
	 */
	bool lit;
	/*
	 * Blanking keeps the panel powered and out of reset, as the stock
	 * kernel does: the touch controller, part of the same chip, then waits
	 * for a double tap. Shutdown cuts the power.
	 */
	bool retained, shutting_down;

	/*
	 * The panel raises its error flags, one per link, when it detected
	 * an error (electrostatic discharge, as a rule) and its picture may
	 * be lost. The panel then needs a reset and its setup again.
	 */
	struct gpio_descs *error_gpios;
	struct delayed_work esd_work;
	struct work_struct reset_work;
	unsigned int esd_resets;
	bool esd_raised;
	bool resetting;
};

struct panel_desc {
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned int bpc;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct mipi_dsi_device_info dsi_info;
	int (*init_sequence)(struct panel_info *pinfo);

	bool is_dual_dsi;

	struct drm_dsc_config dsc;
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, panel);
}

static int iceland_csot_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi[0] };
	struct drm_dsc_picture_parameter_set pps;

	/* Commands sent only to dsi0 here. qcom,sync-dual-dsi in dsi nodes will send them to both dsi ports */

	/* Manufacture Command Set (CMD2_P3): CABC Function Setting */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xFF, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xFB, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x09, 0x5A);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x12, 0xA0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x13, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x15, 0xEA);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x16, 0x17);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0A, 0x8E);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0B, 0x8E);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0C, 0x8E);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0D, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1A, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1B, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1C, 0x0C);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1D, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1E, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1F, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x20, 0x1C);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x21, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x22, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x23, 0x28);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x24, 0x2C);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x25, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x26, 0x34);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x27, 0x38);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x28, 0x3C);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2A, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2B, 0x20);

	/* UI MODE 55=01 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x30, 0xFF);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x31, 0xFD);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x32, 0xFC);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0xFA);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x34, 0xF8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0xF6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x36, 0xF4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x37, 0xF2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x38, 0xF0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x39, 0xEE);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3A, 0xEC);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3B, 0xEA);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3D, 0xE9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3F, 0xE8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x40, 0xE7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x41, 0xE6);

	/* STILL MODE 55=02 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x45, 0xFF);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x46, 0xF9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x47, 0xF6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0xF2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x49, 0xF0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4A, 0xEC);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4B, 0xE8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4C, 0xE4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4D, 0xE0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4E, 0xDE);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4F, 0xD9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x50, 0xD6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0xD4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x52, 0xC2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0xD0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x54, 0xCD);

	/* MOVING MODE 55=03 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x58, 0xFF);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0xF6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5A, 0xF0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5B, 0xEB);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5C, 0xE8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5D, 0xE5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5E, 0xE3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5F, 0xE0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0xDE);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x61, 0xDA);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x62, 0xD7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x63, 0xD4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x64, 0xD2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0xD0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x66, 0xCC);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x67, 0xC8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6E, 0xB3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0xD0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x70, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x71, 0xB3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x89, 0xA4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x8B, 0xC1);

	/* Manufacture Command Set (CMD2_P0): Power / DDB Code / MTP / Gamma setting */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x46);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x30, 0x10);

	/* Manufacture Command Set (CMD2_P6): LTPS Display and In-cell Timing page3 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x26);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0d, 0x03);

	/* Engineer Command Set (CMD3_PageA): Engineer Page A */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x00);

	/* Engineer Command Set (CMD3_PageB): Engineer Page B */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe6, 0x02);

	/* Manufacture Command Set (CMD2_P7): LTPS Display and In-cell Timing page4 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd0, 0x31);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd1, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xde, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdf, 0x02);

	/* User Command Set (CMD1): Basic Display Control Setting */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);//Don't reload registers from MTP/Default values
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x03, 0x12, 0x1a, 0x04, 0x04, 0x01, 0x80, 0x36, 0x36);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x07, 0xff);//Set brightness(controls PWM signal to KTZ8869 chips)
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0x2c);//Enable backlight and dimming for CABC
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x55, 0x00);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x03);//Enable DSC

	drm_dsc_pps_payload_pack(&pps, &pinfo->desc->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 120);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 10);

	return dsi_ctx.accum_err;
}

static const struct drm_display_mode iceland_csot_modes[] = {
	/* First is preferred: the bootloader runs 120 Hz, the stock default. */
	{
		/* 120Hz */
		.clock = (3392 + 100 + 64 + 104) * (2400 + 26 + 2 + 272) * 120 / 1000,
		.hdisplay = 3392,
		.hsync_start = 3392 + 100,
		.hsync_end = 3392 + 100 + 64,
		.htotal = 3392 + 100 + 64 + 104,
		.vdisplay = 2400,
		.vsync_start = 2400 + 26,
		.vsync_end = 2400 + 26 + 2,
		.vtotal = 2400 + 26 + 2 + 272,
	},
	{
		/* 144Hz */
		.clock = (3392 + 64 + 32 + 40) * (2400 + 26 + 2 + 52) * 144 / 1000,
		.hdisplay = 3392,
		.hsync_start = 3392 + 64,
		.hsync_end = 3392 + 64 + 32,
		.htotal = 3392 + 64 + 32 + 40,
		.vdisplay = 2400,
		.vsync_start = 2400 + 26,
		.vsync_end = 2400 + 26 + 2,
		.vtotal = 2400 + 26 + 2 + 52,
	},
	{
		/* 90Hz */
		.clock = (3392 + 420 + 160 + 320) * (2400 + 26 + 2 + 52) * 90 / 1000,
		.hdisplay = 3392,
		.hsync_start = 3392 + 420,
		.hsync_end = 3392 + 420 + 160,
		.htotal = 3392 + 420 + 160 + 320,
		.vdisplay = 2400,
		.vsync_start = 2400 + 26,
		.vsync_end = 2400 + 26 + 2,
		.vtotal = 2400 + 26 + 2 + 52,
	},
	{
		/* 60Hz */
		.clock = (3392 + 100 + 64 + 104) * (2400 + 2726 + 2 + 272) * 60 / 1000,
		.hdisplay = 3392,
		.hsync_start = 3392 + 100,
		.hsync_end = 3392 + 100 + 64,
		.htotal = 3392 + 100 + 64 + 104,
		.vdisplay = 2400,
		.vsync_start = 2400 + 2726,
		.vsync_end = 2400 + 2726 + 2,
		.vtotal = 2400 + 2726 + 2 + 272,
	},
	{
		/* 30Hz */
		.clock = (3392 + 100 + 64 + 104) * (2400 + 8126 + 2 + 272) * 30 / 1000,
		.hdisplay = 3392,
		.hsync_start = 3392 + 100,
		.hsync_end = 3392 + 100 + 64,
		.htotal = 3392 + 100 + 64 + 104,
		.vdisplay = 2400,
		.vsync_start = 2400 + 8126,
		.vsync_end = 2400 + 8126 + 2,
		.vtotal = 2400 + 8126 + 2 + 272,
	}
};

static struct panel_desc iceland_csot_desc = {
	.modes = iceland_csot_modes,
	.num_modes = ARRAY_SIZE(iceland_csot_modes),
	.dsi_info = {
		.type = "CSOT-iceland",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 274,
	.height_mm = 193,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS |
		      MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT,
	.init_sequence = iceland_csot_init_sequence,
	.is_dual_dsi = true,
	.dsc = {
		.dsc_version_major = 0x1,
		/* As the bootloader and the stock kernel run the panel. */
		.dsc_version_minor = 0x2,
		.slice_height = 20,
		.slice_width = 848,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
	},
};

/*
 * The OnePlus Pad 3 (erhai) panel, which OnePlus names XN242 P_D A0024: the
 * same NT36536 controller with its own settings. The commands, their order
 * and delays are those of the stock panel description of this tablet,
 * qcom,mdss_dsi_panel_XN242_P_D_A0024_vid.
 */
static int erhai_xn242_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi[0] };
	struct drm_dsc_picture_parameter_set pps;

	/* Sent to DSI0, and by qcom,sync-dual-dsi to both links. */
	/* CMD2 page 0 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x46);

	/* CMD3 page A */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x00);

	/* CMD2 page 4 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x98, 0x80);

	/* CMD2 page 3 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x89, 0xa4);

	/* CMD3 page B */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe6, 0x02);

	/* CMD2 page 7 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd0, 0x31);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd1, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xde, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdf, 0x02);

	/* CMD1: user command set */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b,
				     0x03, 0xe8, 0x1a, 0x04, 0x04, 0x00, 0x80,
				     0x36, 0x36);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x40);

	/* CMD1: user command set */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x07, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x55, 0x00);

	/* CMD2 page 3 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x60);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x09, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x12, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x13, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x15, 0xea);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x16, 0x17);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0a, 0x8e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0b, 0x8e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0c, 0x8e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0d, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1a, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1b, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1c, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1d, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1e, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1f, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x20, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x21, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x22, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x23, 0x28);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x24, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x25, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x26, 0x34);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x27, 0x38);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x28, 0x3c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x30, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x31, 0xfd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x32, 0xfc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0xfa);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x34, 0xf8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0xf6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x36, 0xf4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x37, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x38, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x39, 0xee);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3a, 0xec);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0xea);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3d, 0xe9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3f, 0xe8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x40, 0xe7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x41, 0xe6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x45, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x46, 0xf9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x47, 0xf6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x49, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4a, 0xec);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4b, 0xe8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4c, 0xe4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4d, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4e, 0xde);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4f, 0xd9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x50, 0xd6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0xd4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x52, 0xc2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0xd0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x54, 0xcd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x58, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0xf6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5a, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5b, 0xeb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5c, 0xe8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5d, 0xe5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5e, 0xe3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5f, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0xde);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x61, 0xda);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x62, 0xd7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x63, 0xd4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x64, 0xd2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0xd0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x66, 0xcc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x67, 0xc8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6e, 0xb3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0xd0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x70, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x71, 0xb3);

	/* CMD1: user command set */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x03);
	/* The controller's own compression setup, as stock sends it */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91,
				     0x89, 0xa8, 0x00, 0x14, 0xd2, 0x00, 0x00,
				     0x00, 0x02, 0x38, 0x00, 0x0b, 0x05, 0x7a,
				     0x03, 0x37);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0x10, 0xe0);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 10);

	/* Stock sends the picture parameter set after the display is on. */
	drm_dsc_pps_payload_pack(&pps, &pinfo->desc->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	return dsi_ctx.accum_err;
}

/*
 * The stock timing of each link is 1696 pixels wide with 50, 32 and 64
 * pixels of front porch, sync and back porch, doubled here for the two
 * links. Its other refresh rates change only the porches.
 */
#define ERHAI_XN242_MODE(hfp, hsw, hbp, vfp, vbp, fps)				\
	{									\
		.clock = (3392 + (hfp) + (hsw) + (hbp)) *			\
			 (2400 + (vfp) + 2 + (vbp)) * (fps) / 1000,		\
		.hdisplay = 3392,						\
		.hsync_start = 3392 + (hfp),					\
		.hsync_end = 3392 + (hfp) + (hsw),				\
		.htotal = 3392 + (hfp) + (hsw) + (hbp),				\
		.vdisplay = 2400,						\
		.vsync_start = 2400 + (vfp),					\
		.vsync_end = 2400 + (vfp) + 2,					\
		.vtotal = 2400 + (vfp) + 2 + (vbp),				\
	}

static const struct drm_display_mode erhai_xn242_modes[] = {
	/* First is preferred: the bootloader runs 120 Hz, the stock default. */
	ERHAI_XN242_MODE(100, 64, 128, 26, 230, 120),
	ERHAI_XN242_MODE(68, 32, 40, 26, 52, 144),
	ERHAI_XN242_MODE(420, 160, 320, 26, 52, 90),
	ERHAI_XN242_MODE(100, 64, 128, 2684, 230, 60),
	/* 50 and 48 Hz stretch the 90 Hz frame: its pixel clock exactly. */
	ERHAI_XN242_MODE(420, 160, 320, 2010, 52, 50),
	ERHAI_XN242_MODE(420, 160, 320, 2196, 52, 48),
	ERHAI_XN242_MODE(100, 64, 128, 8000, 230, 30),
};

static struct panel_desc erhai_xn242_desc = {
	.modes = erhai_xn242_modes,
	.num_modes = ARRAY_SIZE(erhai_xn242_modes),
	.dsi_info = {
		.type = "XN242-erhai",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 274,
	.height_mm = 193,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS |
		      MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT,
	.init_sequence = erhai_xn242_init_sequence,
	.is_dual_dsi = true,
	.dsc = {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x2,
		.slice_height = 20,
		.slice_width = 848,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
	},
};

static void nt36536_reset(struct panel_info *pinfo)
{
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int nt36536_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	if (!pinfo->retained) {
		ret = regulator_bulk_enable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);
		if (ret < 0) {
			dev_err(panel->dev, "failed to enable regulators: %d\n", ret);
			return ret;
		}
	}
	pinfo->retained = false;

	if (pinfo->lit) {
		pinfo->lit = false;
		/*
		 * The hosts never stopped the bootloader's video stream: the
		 * panel shows it as it is, with nothing to reset or set up.
		 */
		if (pinfo->dsi[0]->host->boot_stream &&
		    pinfo->dsi[1]->host->boot_stream)
			return 0;
		/*
		 * The stream stopped under a running panel. The reset pulse
		 * below alone left such a panel with a corrupted picture;
		 * held in reset first, as long as it used to be between
		 * probe and here, it came up clean.
		 */
		gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
		msleep(200);
	}

	nt36536_reset(pinfo);

	ret = pinfo->desc->init_sequence(pinfo);
	if (ret < 0) {
		regulator_bulk_disable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		return ret;
	}

	return 0;
}

static bool nt36536_error_raised(struct panel_info *pinfo)
{
	struct gpio_descs *gpios = pinfo->error_gpios;
	DECLARE_BITMAP(values, 2);
	int ret;

	ret = gpiod_get_array_value_cansleep(gpios->ndescs, gpios->desc,
					     gpios->info, values);
	/* A flag that cannot be read is no reason to blank the panel. */
	if (ret)
		return false;

	return !bitmap_empty(values, gpios->ndescs);
}

/*
 * Power-cycle the pipe feeding the panel, as drm_bridge_helper_reset_crtc()
 * does for a bridge: the panel is disabled, reset and set up again.
 */
static void nt36536_reset_work(struct work_struct *work)
{
	struct panel_info *pinfo = container_of(work, struct panel_info,
						reset_work);
	struct drm_connector *connector = pinfo->connector;
	struct drm_modeset_acquire_ctx ctx;
	int ret;

	drm_modeset_acquire_init(&ctx, 0);
retry:
	ret = drm_modeset_lock(&connector->dev->mode_config.connection_mutex,
			       &ctx);
	if (!ret && connector->state && connector->state->crtc)
		ret = drm_atomic_helper_reset_crtc(connector->state->crtc, &ctx);
	if (ret == -EDEADLK) {
		drm_modeset_backoff(&ctx);
		goto retry;
	}
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	if (ret)
		dev_err(pinfo->panel.dev, "failed to reset the display pipe: %d\n",
			ret);

	WRITE_ONCE(pinfo->resetting, false);
}

static void nt36536_schedule_reset(struct panel_info *pinfo)
{
	if (!pinfo->connector || READ_ONCE(pinfo->resetting))
		return;

	WRITE_ONCE(pinfo->resetting, true);
	schedule_work(&pinfo->reset_work);
}

static void nt36536_esd_work(struct work_struct *work)
{
	struct panel_info *pinfo = container_of(to_delayed_work(work),
						struct panel_info, esd_work);
	unsigned int delay = ESD_CHECK_INTERVAL_MS;

	if (!nt36536_error_raised(pinfo)) {
		pinfo->esd_raised = false;
		pinfo->esd_resets = 0;
	} else if (!pinfo->esd_raised) {
		pinfo->esd_raised = true;
		delay = ESD_RECHECK_MS;
	} else if (pinfo->esd_resets < ESD_MAX_RESETS) {
		pinfo->esd_raised = false;
		pinfo->esd_resets++;
		dev_warn(pinfo->panel.dev,
			 "error flag raised, resetting the display pipe\n");
		/* Enabling the panel again restarts the checks. */
		nt36536_schedule_reset(pinfo);
		return;
	} else if (pinfo->esd_resets == ESD_MAX_RESETS) {
		pinfo->esd_resets++;
		dev_err(pinfo->panel.dev,
			"error flag still raised after %u resets, leaving the panel as it is\n",
			ESD_MAX_RESETS);
	}

	schedule_delayed_work(&pinfo->esd_work, msecs_to_jiffies(delay));
}

static int nt36536_enable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	if (pinfo->error_gpios) {
		pinfo->esd_raised = false;
		schedule_delayed_work(&pinfo->esd_work,
				      msecs_to_jiffies(ESD_CHECK_INTERVAL_MS));
	}

	return 0;
}

static int nt36536_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = pinfo->dsi[0] };

	if (pinfo->error_gpios)
		cancel_delayed_work_sync(&pinfo->esd_work);

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 10);

	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 65);

	return dsi_ctx.accum_err;
}

static int nt36536_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	if (!pinfo->shutting_down) {
		pinfo->retained = true;
		return 0;
	}

	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);

	return 0;
}

static void nt36536_remove(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);

	/* A reset in flight enables the panel and restarts the checks. */
	cancel_work_sync(&pinfo->reset_work);
	cancel_delayed_work_sync(&pinfo->esd_work);
	drm_panel_remove(&pinfo->panel);
}

/* Shutdown blanks with the power cut. */
static void nt36536_shutdown(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);

	/* A reset in flight enables the panel and restarts the checks. */
	cancel_work_sync(&pinfo->reset_work);
	cancel_delayed_work_sync(&pinfo->esd_work);
	pinfo->shutting_down = true;
}

static int nt36536_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int i;

	for (i = 0; i < pinfo->desc->num_modes; i++) {
		const struct drm_display_mode *m = &pinfo->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode) {
			dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
			return -ENOMEM;
		}

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = pinfo->desc->width_mm;
	connector->display_info.height_mm = pinfo->desc->height_mm;
	connector->display_info.bpc = pinfo->desc->bpc;
	pinfo->connector = connector;

	return pinfo->desc->num_modes;
}

static enum drm_panel_orientation nt36536_get_orientation(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	return pinfo->orientation;
}

/* Writing to "recover" resets the display pipe as a raised error flag does. */
static ssize_t nt36536_recover_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct panel_info *pinfo = file->private_data;

	nt36536_schedule_reset(pinfo);

	return count;
}

static const struct file_operations nt36536_recover_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = nt36536_recover_write,
	.llseek = noop_llseek,
};

static void nt36536_debugfs_init(struct drm_panel *panel, struct dentry *root)
{
	struct panel_info *pinfo = to_panel_info(panel);

	debugfs_create_file("recover", 0200, root, pinfo, &nt36536_recover_fops);
}

static const struct drm_panel_funcs nt36536_panel_funcs = {
	.disable = nt36536_disable,
	.enable = nt36536_enable,
	.prepare = nt36536_prepare,
	.unprepare = nt36536_unprepare,
	.get_modes = nt36536_get_modes,
	.get_orientation = nt36536_get_orientation,
	.debugfs_init = nt36536_debugfs_init,
};

static int nt36536_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct panel_info *pinfo;
	const struct mipi_dsi_device_info *info;
	int i, ret;

	pinfo = devm_drm_panel_alloc(dev, struct panel_info, panel,
				     &nt36536_panel_funcs,
				     DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(pinfo))
		return PTR_ERR(pinfo);

	pinfo->supplies[0].supply = "vddio";
	pinfo->supplies[1].supply = "avdd";
	pinfo->supplies[2].supply = "avee";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(pinfo->supplies), pinfo->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	/*
	 * The line stays as the bootloader left it: driven and deasserted, the
	 * panel runs, and is not stopped before the host says, at the first
	 * prepare, whether the bootloader's picture goes on.
	 */
	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(pinfo->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pinfo->reset_gpio), "failed to get reset gpio\n");
	pinfo->lit = gpiod_get_direction(pinfo->reset_gpio) == GPIO_LINE_DIRECTION_OUT &&
		     !gpiod_get_value_cansleep(pinfo->reset_gpio);
	ret = gpiod_direction_output(pinfo->reset_gpio, !pinfo->lit);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set reset gpio\n");

	pinfo->desc = (struct panel_desc *)of_device_get_match_data(dev);
	if (!pinfo->desc)
		return -ENODEV;

	pinfo->error_gpios = devm_gpiod_get_array_optional(dev, "error-flag",
							   GPIOD_IN);
	if (IS_ERR(pinfo->error_gpios))
		return dev_err_probe(dev, PTR_ERR(pinfo->error_gpios),
				     "failed to get error flag gpios\n");
	if (pinfo->error_gpios && pinfo->error_gpios->ndescs > 2)
		return dev_err_probe(dev, -EINVAL, "too many error flag gpios\n");
	INIT_DELAYED_WORK(&pinfo->esd_work, nt36536_esd_work);
	INIT_WORK(&pinfo->reset_work, nt36536_reset_work);

	/* If the panel is dual dsi, register DSI1 */
	if (pinfo->desc->is_dual_dsi) {
		info = &pinfo->desc->dsi_info;

		dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
		if (!dsi1) {
			dev_err(dev, "cannot get secondary DSI node.\n");
			return -ENODEV;
		}

		dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
		of_node_put(dsi1);
		if (!dsi1_host)
			return dev_err_probe(dev, -EPROBE_DEFER, "cannot get secondary DSI host\n");

		pinfo->dsi[1] = devm_mipi_dsi_device_register_full(dev, dsi1_host, info);
		if (IS_ERR(pinfo->dsi[1])) {
			dev_err(dev, "cannot get secondary DSI device\n");
			return PTR_ERR(pinfo->dsi[1]);
		}
	}

	pinfo->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	ret = drm_of_get_panel_orientation(dev->of_node, &pinfo->orientation);
	if (ret < 0) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
		return ret;
	}

	pinfo->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&pinfo->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&pinfo->panel);

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		pinfo->dsi[i]->lanes = pinfo->desc->lanes;
		pinfo->dsi[i]->format = pinfo->desc->format;
		pinfo->dsi[i]->mode_flags = pinfo->desc->mode_flags;
		pinfo->dsi[i]->dsc = &pinfo->desc->dsc;

		ret = devm_mipi_dsi_attach(dev, pinfo->dsi[i]);
		if (ret < 0)
			return dev_err_probe(dev, ret, "cannot attach to DSI%d host.\n", i);
	}

	return 0;
}

static const struct of_device_id nt36536_of_match[] = {
	{
		.compatible = "oneplus,iceland-nt36536",
		.data = &iceland_csot_desc,
	},
	{
		.compatible = "oneplus,erhai-nt36536",
		.data = &erhai_xn242_desc,
	},
	{},
};
MODULE_DEVICE_TABLE(of, nt36536_of_match);

static struct mipi_dsi_driver nt36536_driver = {
	.probe = nt36536_probe,
	.remove = nt36536_remove,
	.shutdown = nt36536_shutdown,
	.driver = {
		.name = "panel-oneplus-iceland-nt36536",
		.of_match_table = nt36536_of_match,
	},
};
module_mipi_dsi_driver(nt36536_driver);

MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_DESCRIPTION("DRM driver for Novatek NT36536 based MIPI DSI panels");
MODULE_LICENSE("GPL");
