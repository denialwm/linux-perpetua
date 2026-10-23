// SPDX-License-Identifier: GPL-2.0-only
/*
 * Raydium RM692H5 based 6.8" 1216x2688 AMOLED DSC command mode panel, as found
 * in the Nubia RedMagic 11 Pro (NX809J), downstream name
 * "BF375_RM692H5_6P8_MAGIC_DSC_CMD".
 *
 * Init sequence generated from the downstream device tree with
 * linux-mdss-dsi-panel-driver-generator (--dumb-dcs: most writes target
 * vendor register pages selected with 0xfe, so they are sent as raw bytes).
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <video/mipi_display.h>

#define RM692H5_MAX_BRIGHTNESS	8190

/* Power-on order and delays as in the downstream panel power sequence */
static const char * const rm692h5_supply_names[] = { "vddio", "vdd", "vci" };

struct rm692h5 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(rm692h5_supply_names)];
};

static inline struct rm692h5 *to_rm692h5(struct drm_panel *panel)
{
	return container_of(panel, struct rm692h5, panel);
}

static int rm692h5_power_on(struct rm692h5 *ctx)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++) {
		ret = regulator_enable(ctx->supplies[i].consumer);
		if (ret) {
			while (--i >= 0)
				regulator_disable(ctx->supplies[i].consumer);
			return ret;
		}
		usleep_range(2000, 3000);
	}

	return 0;
}

static void rm692h5_power_off(struct rm692h5 *ctx)
{
	int i;

	for (i = ARRAY_SIZE(ctx->supplies) - 1; i >= 0; i--) {
		regulator_disable(ctx->supplies[i].consumer);
		usleep_range(2000, 3000);
	}
}

static void rm692h5_reset(struct rm692h5 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(40);
}

static void rm692h5_on(struct mipi_dsi_multi_context *dsi_ctx)
{
	/* Downstream sends the on commands in LP mode */
	dsi_ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0xd4);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x40, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0xfd);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xee, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xef, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0xd0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x7e, 0x80);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0xb6);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x01, 0x74);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0xa1);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x74, 0x72);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc3, 0xc3);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc4, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc5, 0x7f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfa, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0xd2);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x97, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x36, 0x11);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x39, 0xab);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3a, 0x30);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3b, 0x80);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3d, 0x0a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3f, 0x80);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x40, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x41, 0xc0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x42, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x43, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x44, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x45, 0x60);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x46, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x47, 0x60);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x48, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x49, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x4a, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x4b, 0x31);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x4d, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x4e, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x4f, 0x68);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x50, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x51, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x53, 0x0c);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x54, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x55, 0x67);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x56, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x58, 0xad);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x59, 0x18);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5a, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5b, 0x11);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5c, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5d, 0x07);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5e, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5f, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x60, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x61, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x62, 0x0f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x63, 0x0f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x64, 0x33);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x65, 0x0e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x66, 0x1c);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x67, 0x2a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x68, 0x38);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x69, 0x46);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6a, 0x54);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6b, 0x62);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6c, 0x69);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6d, 0x70);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6e, 0x77);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x79);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x70, 0x7b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x71, 0x7d);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x72, 0x7e);
	/*
	 * Page 0xd2 holds the panel's DSC decoder config, which it appears to
	 * use instead of the PPS we send.
	 * RC range 0 is {min_qp 0, max_qp 8, offset 2} downstream, as in the
	 * DSC model pre-SCR config; the kernel encoder uses the VESA DSC 1.1
	 * value max_qp 4 (see drm_dsc_helper.c). Everything else matches, and
	 * the mismatch corrupts the picture, so program {0, 4, 2} (0x0102).
	 */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x73, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x74, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x75, 0x22);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x76, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x77, 0x2a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x78, 0x40);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x79, 0x2a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x7a, 0xbe);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x7b, 0x3a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x7c, 0xfc);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x7d, 0x3a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x7e, 0xfa);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x7f, 0x3a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x81, 0x3b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x82, 0x38);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x3b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x84, 0x78);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x85, 0x3b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x86, 0xb6);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x87, 0x4b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x88, 0xf6);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x89, 0x4c);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8a, 0x34);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8b, 0x4c);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8c, 0x74);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8d, 0x5c);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8e, 0x74);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8f, 0x8c);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x90, 0xf4);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x91, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x92, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x93, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x94, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x95, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x96, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x34);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x82, 0xc0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x00, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x1c, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x1d, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x49, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x1e, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x1f, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x20, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x21, 0x70);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x22, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x23, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x24, 0x77);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x25, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x26, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x2f, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x30, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x31, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x32, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x33, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x34, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x35, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x36, 0xc4);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x37, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x38, 0x87);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x39, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3a, 0x4b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3b, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3d, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3f, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x40, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x41, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x42, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x54, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x55, 0x77);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x56, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x58, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x59, 0x77);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5a, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5b, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5c, 0x77);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5d, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5e, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5f, 0x77);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x60, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x61, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x62, 0x77);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x63, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x64, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x65, 0x77);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x66, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x67, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x77, 0x07);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x78, 0xf8);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x2f, 0x07);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x5a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x47, 0xf0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x48, 0xdf);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x49, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x4a, 0xee);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x4b, 0xee);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x4c, 0xee);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x4f, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x50, 0x16);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x51, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x53, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x58, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x59, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x60, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x61, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x68, 0xf6);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x69, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x70, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x79, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x7a, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x7f, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x88, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x89, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8a, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8c, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x90, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x91, 0x30);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x92, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x93, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x94, 0x09);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x95, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x98, 0x0c);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x99, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x9a, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa0, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa2, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa3, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xaa, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xab, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb2, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb3, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xba, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbb, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbc, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbd, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc2, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc3, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc4, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc9, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xca, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcb, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xda, 0x40);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xdb, 0x30);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xdc, 0x2a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xdd, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xde, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xdf, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xe2, 0x1f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xe3, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xe4, 0x0d);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xea, 0x12);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xeb, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xec, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf3, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf4, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x5b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x01, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x02, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x03, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x09, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x0a, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x11, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x12, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x18, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x19, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x1b, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x21, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x22, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x23, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x24, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x25, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x29, 0x7f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x2b, 0xfd);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x2d, 0x1f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x2f, 0x1b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x30, 0x36);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x33, 0xf9);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x34, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x35, 0xfc);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3b, 0xfd);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3d, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x45, 0xfb);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x46, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x55, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x56, 0xfd);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5e, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x5f, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x60, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x65, 0xfe);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x66, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x67, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6e, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x70, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x71, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x9b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x0b, 0x4e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc2, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x35, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x53, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x51, 0x00, 0x00);
	mipi_dsi_dcs_exit_sleep_mode_multi(dsi_ctx);
	mipi_dsi_msleep(dsi_ctx, 100);
	mipi_dsi_dcs_set_display_on_multi(dsi_ctx);
	mipi_dsi_msleep(dsi_ctx, 20);
}

static int rm692h5_off(struct rm692h5 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfe, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 16000, 17000);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int rm692h5_prepare(struct drm_panel *panel)
{
	struct rm692h5 *ctx = to_rm692h5(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = rm692h5_power_on(ctx);
	if (ret)
		return ret;

	rm692h5_reset(ctx);
	rm692h5_on(&dsi_ctx);

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
	mipi_dsi_compression_mode_multi(&dsi_ctx, true);
	if (dsi_ctx.accum_err) {
		ret = dsi_ctx.accum_err;
		goto err;
	}

	msleep(28);

	return 0;

err:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	rm692h5_power_off(ctx);

	return ret;
}

static int rm692h5_unprepare(struct drm_panel *panel)
{
	struct rm692h5 *ctx = to_rm692h5(panel);
	int ret;

	ret = rm692h5_off(ctx);
	if (ret < 0)
		dev_err(&ctx->dsi->dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	rm692h5_power_off(ctx);

	return 0;
}

/* Downstream timing@0 (60 Hz); 90 and 120 Hz differ in porches and init */
static const struct drm_display_mode rm692h5_mode = {
	.clock = (1216 + 248 + 8 + 272) * (2688 + 56 + 8 + 8) * 60 / 1000,
	.hdisplay = 1216,
	.hsync_start = 1216 + 248,
	.hsync_end = 1216 + 248 + 8,
	.htotal = 1216 + 248 + 8 + 272,
	.vdisplay = 2688,
	.vsync_start = 2688 + 56,
	.vsync_end = 2688 + 56 + 8,
	.vtotal = 2688 + 56 + 8 + 8,
	.width_mm = 72,
	.height_mm = 158,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int rm692h5_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &rm692h5_mode);
}

static const struct drm_panel_funcs rm692h5_panel_funcs = {
	.prepare = rm692h5_prepare,
	.unprepare = rm692h5_unprepare,
	.get_modes = rm692h5_get_modes,
};

static int rm692h5_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static const struct backlight_ops rm692h5_bl_ops = {
	.update_status = rm692h5_bl_update_status,
};

static struct backlight_device *rm692h5_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = RM692H5_MAX_BRIGHTNESS / 2,
		.max_brightness = RM692H5_MAX_BRIGHTNESS,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &rm692h5_bl_ops, &props);
}

static int rm692h5_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct rm692h5 *ctx;
	int i, ret;

	ctx = devm_drm_panel_alloc(dev, struct rm692h5, panel,
				   &rm692h5_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++)
		ctx->supplies[i].supply = rm692h5_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* Command mode, EoT packets on (downstream tx-eot-append) */
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;

	ctx->panel.prepare_prev_first = true;
	ctx->panel.backlight = rm692h5_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 16;
	ctx->dsc.slice_width = 608;
	ctx->dsc.slice_count = 1216 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 10;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void rm692h5_remove(struct mipi_dsi_device *dsi)
{
	struct rm692h5 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id rm692h5_of_match[] = {
	{ .compatible = "nubia,nx809j-rm692h5" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rm692h5_of_match);

static struct mipi_dsi_driver rm692h5_driver = {
	.probe = rm692h5_probe,
	.remove = rm692h5_remove,
	.driver = {
		.name = "panel-raydium-rm692h5",
		.of_match_table = rm692h5_of_match,
	},
};
module_mipi_dsi_driver(rm692h5_driver);

MODULE_DESCRIPTION("DRM driver for Raydium RM692H5 based DSC panels");
MODULE_LICENSE("GPL");
