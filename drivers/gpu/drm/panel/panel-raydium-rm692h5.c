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
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>

#include <video/mipi_display.h>

#define RM692H5_MAX_BRIGHTNESS	8190

/*
 * Blanking keeps the panel powered, initialized and out of sleep with only
 * its display off, so waking is a display-on instead of the power cycle and
 * initialization (about 250 ms). Off restores the full power cycle.
 */
static bool retain = true;
module_param(retain, bool, 0644);
MODULE_PARM_DESC(retain, "Keep the panel powered while blanked for a fast wake");

/*
 * Always-on display, set by the compositor while its lock screen shows the
 * always-on picture. Blanking then puts the panel in DCS idle mode (stock
 * lp1) at ambient_brightness instead of turning the display off, so it keeps
 * showing the last frame from its own memory while the host powers down, and
 * an unblank keeps idle mode: the host only woke to draw a new frame.
 * Clearing it leaves idle mode, at once on a lit panel or else at the next
 * unblank. The backlight keeps its level across the blank, so a level faded
 * to match ambient_brightness goes into idle mode and back without a jump or
 * a dark frame.
 */
static bool ambient;
static int rm692h5_set_ambient(const char *val, const struct kernel_param *kp);
static const struct kernel_param_ops rm692h5_ambient_ops = {
	.set = rm692h5_set_ambient,
	.get = param_get_bool,
};
module_param_cb(ambient, &rm692h5_ambient_ops, &ambient, 0644);
MODULE_PARM_DESC(ambient, "Blank into idle mode, keeping the last frame lit");

static uint ambient_brightness = 0x0120;
module_param(ambient_brightness, uint, 0644);
MODULE_PARM_DESC(ambient_brightness, "Brightness code for a blank into idle mode (stock 0x0120)");

/*
 * The panel takes a while to switch idle mode. Powering the host down in
 * the middle of the switch dims the picture for a moment, and pixels
 * written before it ends can leave a line across the screen.
 */
static uint idle_settle_ms = 150;
module_param(idle_settle_ms, uint, 0644);
MODULE_PARM_DESC(idle_settle_ms, "Wait after entering idle mode before the host powers down");

static uint idle_exit_ms = 50;
module_param(idle_exit_ms, uint, 0644);
MODULE_PARM_DESC(idle_exit_ms, "Wait after leaving idle mode before new pixels");

struct rm692h5_rate;

/* Power-on order and delays as in the downstream panel power sequence */
static const char * const rm692h5_supply_names[] = { "vddio", "vdd", "vci" };

struct rm692h5 {
	struct drm_panel panel;
	struct drm_connector *connector;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(rm692h5_supply_names)];
	/* The refresh rate the panel runs; it outlives a retained blank. */
	const struct rm692h5_rate *rate;
	bool retained, shutting_down;
	/* Lit (between enable and disable), and in DCS idle mode. */
	bool enabled, idle;
	ktime_t wake;
};

/* The one panel, for the ambient parameter. */
static struct rm692h5 *rm692h5_panel;

static inline struct rm692h5 *to_rm692h5(struct drm_panel *panel)
{
	return container_of(panel, struct rm692h5, panel);
}

/*
 * The level right after the switch, with no wait between: idle mode at the
 * old level flashes, and a level first shows normal mode at the always-on
 * level for a moment, part of a frame darker.
 */
static void rm692h5_idle_on(struct mipi_dsi_multi_context *dsi_ctx, u16 level)
{
	u8 set_level[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, level >> 8, level & 0xff };

	dsi_ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x00);
	mipi_dsi_dcs_write_buffer_multi(dsi_ctx, (u8[]){ MIPI_DCS_ENTER_IDLE_MODE }, 1);
	mipi_dsi_dcs_write_buffer_multi(dsi_ctx, set_level, sizeof(set_level));
	mipi_dsi_msleep(dsi_ctx, idle_settle_ms);
}

/*
 * Stock nolp sets brightness 0 before idle off; the backlight's own level
 * instead leaves idle mode where the backlight is about to restore it.
 */
static void rm692h5_idle_off(struct mipi_dsi_multi_context *dsi_ctx,
			     struct backlight_device *bl)
{
	u16 level = min_t(u32, bl->props.brightness, RM692H5_MAX_BRIGHTNESS);
	u8 set_level[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, level >> 8, level & 0xff };

	dsi_ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x00);
	mipi_dsi_dcs_write_buffer_multi(dsi_ctx, set_level, sizeof(set_level));
	mipi_dsi_dcs_write_buffer_multi(dsi_ctx, (u8[]){ MIPI_DCS_EXIT_IDLE_MODE }, 1);
	mipi_dsi_msleep(dsi_ctx, idle_exit_ms);
}

static int rm692h5_set_ambient(const char *val, const struct kernel_param *kp)
{
	struct rm692h5 *ctx = READ_ONCE(rm692h5_panel);
	struct mipi_dsi_multi_context dsi_ctx;
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;
	WRITE_ONCE(ambient, on);
	if (on || !ctx || !ctx->enabled || !ctx->idle)
		return 0;

	dsi_ctx = (struct mipi_dsi_multi_context){ .dsi = ctx->dsi };
	rm692h5_idle_off(&dsi_ctx, ctx->panel.backlight);
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;
	ctx->idle = false;
	dev_info(&ctx->dsi->dev, "idle mode off on the lit panel\n");
	return 0;
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

/*
 * Refresh rates of the downstream timings. In command mode the panel paces
 * itself: the porches only size the DSI link, and each rate has its own
 * register set (the downstream timing switch commands).
 */
#define RM692H5_MODE(_fps, _hfp, _hbp) {				\
	.clock = (1216 + (_hfp) + 8 + (_hbp)) * (2688 + 56 + 8 + 8) *	\
		 (_fps) / 1000,						\
	.hdisplay = 1216,						\
	.hsync_start = 1216 + (_hfp),					\
	.hsync_end = 1216 + (_hfp) + 8,					\
	.htotal = 1216 + (_hfp) + 8 + (_hbp),				\
	.vdisplay = 2688,						\
	.vsync_start = 2688 + 56,					\
	.vsync_end = 2688 + 56 + 8,					\
	.vtotal = 2688 + 56 + 8 + 8,					\
	.width_mm = 72,							\
	.height_mm = 158,						\
	.type = DRM_MODE_TYPE_DRIVER,					\
}

static const u8 rm692h5_60hz[][2] = {
	{ 0xfe, 0x00 }, { 0x2f, 0x07 }, { 0xfe, 0x5a }, { 0x47, 0xf0 },
	{ 0x48, 0xdf }, { 0x49, 0x20 }, { 0x4a, 0xee }, { 0x4b, 0xee },
	{ 0x4c, 0xee }, { 0x4f, 0x10 }, { 0x50, 0x16 }, { 0x51, 0x00 },
	{ 0x53, 0x08 }, { 0x58, 0x00 }, { 0x59, 0x00 }, { 0x60, 0x00 },
	{ 0x61, 0x00 }, { 0x68, 0xf6 }, { 0x69, 0x00 }, { 0x70, 0x00 },
	{ 0x79, 0x00 }, { 0x7a, 0xff }, { 0x7f, 0x01 }, { 0x80, 0x00 },
	{ 0x88, 0xff }, { 0x89, 0xff }, { 0x8a, 0xff }, { 0x8c, 0x00 },
	{ 0x90, 0x20 }, { 0x91, 0x30 }, { 0x92, 0x00 }, { 0x93, 0x10 },
	{ 0x94, 0x09 }, { 0x95, 0x10 }, { 0x98, 0x0c }, { 0x99, 0x01 },
	{ 0x9a, 0x02 }, { 0xa0, 0x02 }, { 0xa2, 0x06 }, { 0xa3, 0x04 },
	{ 0xaa, 0x00 }, { 0xab, 0x02 }, { 0xb2, 0x06 }, { 0xb3, 0x02 },
	{ 0xba, 0x01 }, { 0xbb, 0x03 }, { 0xbc, 0x02 }, { 0xbd, 0x03 },
	{ 0xc2, 0x03 }, { 0xc3, 0x03 }, { 0xc4, 0x02 }, { 0xc9, 0x02 },
	{ 0xca, 0x01 }, { 0xcb, 0x01 }, { 0xda, 0x40 }, { 0xdb, 0x30 },
	{ 0xdc, 0x2a }, { 0xdd, 0x06 }, { 0xde, 0x10 }, { 0xdf, 0x06 },
	{ 0xe2, 0x1f }, { 0xe3, 0x20 }, { 0xe4, 0x0d }, { 0xea, 0x12 },
	{ 0xeb, 0x10 }, { 0xec, 0x10 }, { 0xf3, 0x04 }, { 0xf4, 0x06 },
	{ 0xfe, 0x5b }, { 0x01, 0x10 }, { 0x02, 0x06 }, { 0x03, 0x02 },
	{ 0x09, 0x03 }, { 0x0a, 0x02 }, { 0x11, 0x02 }, { 0x12, 0x02 },
	{ 0x18, 0x02 }, { 0x19, 0x02 }, { 0x1b, 0xff }, { 0x21, 0x02 },
	{ 0x22, 0x01 }, { 0x23, 0x01 }, { 0x24, 0x01 }, { 0x25, 0x02 },
	{ 0x29, 0x7f }, { 0x2b, 0xfd }, { 0x2d, 0x1f }, { 0x2f, 0x1b },
	{ 0x30, 0x36 }, { 0x33, 0xf9 }, { 0x34, 0x00 }, { 0x35, 0xfc },
	{ 0x3b, 0xfd }, { 0x3d, 0xff }, { 0x45, 0xfb }, { 0x46, 0x00 },
	{ 0x55, 0x00 }, { 0x56, 0xfd }, { 0x5e, 0xff }, { 0x5f, 0x01 },
	{ 0x60, 0x02 }, { 0x65, 0xfe }, { 0x66, 0xff }, { 0x67, 0x01 },
	{ 0x6e, 0x01 }, { 0x6f, 0x01 }, { 0x70, 0x01 }, { 0x71, 0x00 },
	{ 0xfe, 0x00 }
};

static const u8 rm692h5_90hz[][2] = {
	{ 0xfe, 0x00 }, { 0x2f, 0x06 }, { 0xfe, 0x5a }, { 0x47, 0xff },
	{ 0x48, 0xdf }, { 0x49, 0x10 }, { 0x4a, 0xfc }, { 0x4b, 0xff },
	{ 0x4f, 0x10 }, { 0x50, 0xff }, { 0x51, 0xfe }, { 0x53, 0x08 },
	{ 0x58, 0x05 }, { 0x60, 0xfd }, { 0x61, 0x00 }, { 0x68, 0xfd },
	{ 0x69, 0x00 }, { 0x70, 0x01 }, { 0x79, 0x00 }, { 0x7a, 0xff },
	{ 0x7f, 0x01 }, { 0x80, 0xfe }, { 0x88, 0xff }, { 0x89, 0xff },
	{ 0x8a, 0xff }, { 0x8c, 0x00 }, { 0x90, 0x00 }, { 0x91, 0x40 },
	{ 0x92, 0x0b }, { 0x93, 0x14 }, { 0x94, 0x0f }, { 0x95, 0x03 },
	{ 0x98, 0x0c }, { 0x99, 0x08 }, { 0x9a, 0x08 }, { 0xa0, 0x0b },
	{ 0xa2, 0x06 }, { 0xa3, 0x06 }, { 0xaa, 0x07 }, { 0xab, 0x02 },
	{ 0xb2, 0x07 }, { 0xb3, 0x05 }, { 0xba, 0x04 }, { 0xbb, 0x03 },
	{ 0xbc, 0x02 }, { 0xbd, 0x03 }, { 0xc2, 0x03 }, { 0xc3, 0x03 },
	{ 0xc4, 0x02 }, { 0xc9, 0x02 }, { 0xca, 0x02 }, { 0xcb, 0x01 },
	{ 0xda, 0x70 }, { 0xdb, 0x30 }, { 0xdc, 0x30 }, { 0xdd, 0x1c },
	{ 0xde, 0x02 }, { 0xdf, 0x1b }, { 0xe2, 0x1f }, { 0xe3, 0x20 },
	{ 0xe4, 0x18 }, { 0xea, 0x18 }, { 0xeb, 0x10 }, { 0xec, 0x14 },
	{ 0xf3, 0x0c }, { 0xf4, 0x06 }, { 0xfe, 0x5b }, { 0x01, 0x10 },
	{ 0x02, 0x08 }, { 0x03, 0x04 }, { 0x09, 0x06 }, { 0x0a, 0x05 },
	{ 0x11, 0x02 }, { 0x12, 0x02 }, { 0x18, 0x02 }, { 0x19, 0x07 },
	{ 0x1b, 0xff }, { 0x21, 0x02 }, { 0x22, 0x01 }, { 0x23, 0x01 },
	{ 0x24, 0x01 }, { 0x25, 0x02 }, { 0x29, 0x10 }, { 0x2b, 0x20 },
	{ 0x2d, 0x1c }, { 0x2f, 0x10 }, { 0x30, 0x22 }, { 0x33, 0xf9 },
	{ 0x34, 0xff }, { 0x35, 0x00 }, { 0x3b, 0xf9 }, { 0x3d, 0x00 },
	{ 0x45, 0xfe }, { 0x46, 0x00 }, { 0x55, 0xff }, { 0x56, 0xfd },
	{ 0x5e, 0xff }, { 0x5f, 0x01 }, { 0x60, 0x02 }, { 0x65, 0xfe },
	{ 0x66, 0x01 }, { 0x67, 0xfc }, { 0x6e, 0x01 }, { 0x6f, 0x01 },
	{ 0x70, 0x01 }, { 0x71, 0x00 }, { 0xfe, 0x00 }
};

static const u8 rm692h5_120hz[][2] = {
	{ 0xfe, 0x00 }, { 0x2f, 0x05 }, { 0xfe, 0x5a }, { 0x47, 0xff },
	{ 0x48, 0xf0 }, { 0x49, 0x3a }, { 0x4a, 0x00 }, { 0x4b, 0x10 },
	{ 0x4c, 0xfc }, { 0x4f, 0x22 }, { 0x50, 0x30 }, { 0x51, 0x00 },
	{ 0x53, 0x08 }, { 0x58, 0x05 }, { 0x59, 0x00 }, { 0x60, 0x00 },
	{ 0x61, 0x00 }, { 0x68, 0xf6 }, { 0x69, 0x00 }, { 0x70, 0x00 },
	{ 0x79, 0x01 }, { 0x7a, 0x00 }, { 0x7f, 0x02 }, { 0x80, 0x01 },
	{ 0x88, 0x00 }, { 0x89, 0x00 }, { 0x8a, 0x00 }, { 0x8c, 0xff },
	{ 0x90, 0x00 }, { 0x91, 0x30 }, { 0x92, 0x10 }, { 0x93, 0x10 },
	{ 0x94, 0x09 }, { 0x95, 0x1d }, { 0x98, 0x0e }, { 0x99, 0x16 },
	{ 0x9a, 0x10 }, { 0xa0, 0x10 }, { 0xa2, 0x09 }, { 0xa3, 0x06 },
	{ 0xaa, 0x00 }, { 0xab, 0x00 }, { 0xb2, 0x07 }, { 0xb3, 0x03 },
	{ 0xba, 0x01 }, { 0xbb, 0x01 }, { 0xbc, 0x02 }, { 0xbd, 0x00 },
	{ 0xc2, 0x04 }, { 0xc3, 0x04 }, { 0xc4, 0x02 }, { 0xc9, 0x05 },
	{ 0xca, 0x01 }, { 0xcb, 0x01 }, { 0xda, 0x70 }, { 0xdb, 0x4c },
	{ 0xdc, 0x3f }, { 0xdd, 0x15 }, { 0xde, 0x12 }, { 0xdf, 0x1b },
	{ 0xe2, 0x23 }, { 0xe3, 0x3c }, { 0xe4, 0x28 }, { 0xea, 0x1f },
	{ 0xeb, 0x1a }, { 0xec, 0x14 }, { 0xf3, 0x10 }, { 0xf4, 0x08 },
	{ 0xfe, 0x5b }, { 0x01, 0x12 }, { 0x02, 0x0b }, { 0x03, 0x04 },
	{ 0x09, 0x08 }, { 0x0a, 0x05 }, { 0x11, 0x08 }, { 0x12, 0x05 },
	{ 0x18, 0x0a }, { 0x19, 0x05 }, { 0x1b, 0x00 }, { 0x21, 0x03 },
	{ 0x22, 0x02 }, { 0x23, 0x02 }, { 0x24, 0x02 }, { 0x25, 0x04 },
	{ 0x29, 0x10 }, { 0x2b, 0xdc }, { 0x2d, 0x1f }, { 0x2f, 0xfc },
	{ 0x30, 0x22 }, { 0x33, 0xf8 }, { 0x34, 0xe0 }, { 0x35, 0x00 },
	{ 0x3b, 0xf9 }, { 0x3d, 0xff }, { 0x45, 0x00 }, { 0x46, 0xff },
	{ 0x55, 0x00 }, { 0x56, 0xfc }, { 0x5e, 0xfe }, { 0x5f, 0xff },
	{ 0x60, 0x02 }, { 0x65, 0xfa }, { 0x66, 0xff }, { 0x67, 0xff },
	{ 0x6e, 0x00 }, { 0x6f, 0x00 }, { 0x70, 0x00 }, { 0x71, 0x01 },
	{ 0xfe, 0x00 }
};

static const u8 rm692h5_144hz[][2] = {
	{ 0xfe, 0x00 }, { 0x2f, 0x00 }, { 0xfe, 0x5a }, { 0x47, 0xff },
	{ 0x48, 0xf0 }, { 0x49, 0x3a }, { 0x4a, 0x00 }, { 0x4b, 0x10 },
	{ 0x4c, 0xfc }, { 0x4f, 0x22 }, { 0x50, 0x30 }, { 0x51, 0x00 },
	{ 0x53, 0x06 }, { 0x58, 0x05 }, { 0x59, 0x00 }, { 0x60, 0x00 },
	{ 0x61, 0x00 }, { 0x68, 0xf6 }, { 0x69, 0x00 }, { 0x70, 0x00 },
	{ 0x79, 0x01 }, { 0x7a, 0x00 }, { 0x7f, 0x02 }, { 0x80, 0x01 },
	{ 0x88, 0x00 }, { 0x89, 0x00 }, { 0x8a, 0x00 }, { 0x8c, 0xff },
	{ 0x90, 0x00 }, { 0x91, 0x30 }, { 0x92, 0x10 }, { 0x93, 0x10 },
	{ 0x94, 0x09 }, { 0x95, 0x1d }, { 0x98, 0x0e }, { 0x99, 0x20 },
	{ 0x9a, 0x10 }, { 0xa0, 0x10 }, { 0xa2, 0x09 }, { 0xa3, 0x06 },
	{ 0xaa, 0x00 }, { 0xab, 0x02 }, { 0xb2, 0x07 }, { 0xb3, 0x02 },
	{ 0xba, 0x01 }, { 0xbb, 0x01 }, { 0xbc, 0x02 }, { 0xbd, 0x00 },
	{ 0xc2, 0x04 }, { 0xc3, 0x04 }, { 0xc4, 0x02 }, { 0xc9, 0x05 },
	{ 0xca, 0x01 }, { 0xcb, 0x01 }, { 0xda, 0x70 }, { 0xdb, 0x4c },
	{ 0xdc, 0x38 }, { 0xdd, 0x15 }, { 0xde, 0x12 }, { 0xdf, 0x1b },
	{ 0xe2, 0x23 }, { 0xe3, 0x3c }, { 0xe4, 0x28 }, { 0xea, 0x1f },
	{ 0xeb, 0x1a }, { 0xec, 0x14 }, { 0xf3, 0x10 }, { 0xf4, 0x0d },
	{ 0xfe, 0x5b }, { 0x01, 0x12 }, { 0x02, 0x0b }, { 0x03, 0x04 },
	{ 0x09, 0x08 }, { 0x0a, 0x05 }, { 0x11, 0x08 }, { 0x12, 0x05 },
	{ 0x18, 0x0a }, { 0x19, 0x05 }, { 0x1b, 0x00 }, { 0x21, 0x03 },
	{ 0x22, 0x02 }, { 0x23, 0x02 }, { 0x24, 0x02 }, { 0x25, 0x04 },
	{ 0x29, 0x10 }, { 0x2a, 0xff }, { 0x2b, 0xdc }, { 0x2d, 0x1f },
	{ 0x2f, 0xfc }, { 0x30, 0x22 }, { 0x33, 0xf8 }, { 0x34, 0xe0 },
	{ 0x35, 0x00 }, { 0x37, 0x06 }, { 0x3b, 0xf9 }, { 0x3d, 0xff },
	{ 0x45, 0x00 }, { 0x46, 0xff }, { 0x55, 0x00 }, { 0x56, 0x00 },
	{ 0x5e, 0xfe }, { 0x5f, 0xff }, { 0x60, 0x02 }, { 0x65, 0xfa },
	{ 0x66, 0xff }, { 0x67, 0xff }, { 0x6e, 0x00 }, { 0x6f, 0x00 },
	{ 0x70, 0x00 }, { 0x71, 0x01 }, { 0xfe, 0x00 }
};
struct rm692h5_rate {
	struct drm_display_mode mode;
	const u8 (*regs)[2];
	unsigned int num_regs;
};

#define RM692H5_RATE(_fps, _hfp, _hbp) {				\
	.mode = RM692H5_MODE(_fps, _hfp, _hbp),				\
	.regs = rm692h5_##_fps##hz,					\
	.num_regs = ARRAY_SIZE(rm692h5_##_fps##hz),			\
}

/* The first one is the preferred mode */
static const struct rm692h5_rate rm692h5_rates[] = {
	RM692H5_RATE(60, 248, 272),
	RM692H5_RATE(90, 108, 128),
	RM692H5_RATE(120, 32, 60),
	RM692H5_RATE(144, 8, 12),
};

static const struct rm692h5_rate *rm692h5_get_rate(struct rm692h5 *ctx)
{
	struct drm_connector *connector = ctx->connector;
	const struct drm_display_mode *mode;
	unsigned int i;

	if (!connector || !connector->state || !connector->state->crtc)
		return &rm692h5_rates[0];

	mode = &connector->state->crtc->state->mode;
	for (i = 0; i < ARRAY_SIZE(rm692h5_rates); i++)
		if (drm_mode_match(mode, &rm692h5_rates[i].mode,
				   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK))
			return &rm692h5_rates[i];

	return &rm692h5_rates[0];
}

static void rm692h5_send_rate(struct mipi_dsi_multi_context *dsi_ctx,
			      const struct rm692h5_rate *rate)
{
	unsigned int i;

	for (i = 0; i < rate->num_regs; i++)
		mipi_dsi_dcs_write_buffer_multi(dsi_ctx, rate->regs[i], 2);
}

static void rm692h5_on(struct mipi_dsi_multi_context *dsi_ctx,
		       const struct rm692h5_rate *rate)
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
	 * use instead of the PPS we send. RC range 0 is {min_qp 0, max_qp 8,
	 * offset 2} (0x0202), the DSC model's pre-SCR value that the msm DSI
	 * host also programs at 10 bpc and 8 bpp; any other value corrupts
	 * the picture.
	 */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x73, 0x02);
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
	rm692h5_send_rate(dsi_ctx, rate);
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

/* Power the panel up from off and initialize it, display on. */
static int rm692h5_cold(struct rm692h5 *ctx, const struct rm692h5_rate *rate)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = rm692h5_power_on(ctx);
	if (ret)
		return ret;

	rm692h5_reset(ctx);
	rm692h5_on(&dsi_ctx, rate);

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
	mipi_dsi_compression_mode_multi(&dsi_ctx, true);
	if (dsi_ctx.accum_err) {
		ret = dsi_ctx.accum_err;
		goto err;
	}

	msleep(28);
	ctx->rate = rate;
	ctx->idle = false;

	return 0;

err:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	rm692h5_power_off(ctx);

	return ret;
}

static int rm692h5_prepare(struct drm_panel *panel)
{
	struct rm692h5 *ctx = to_rm692h5(panel);
	const struct rm692h5_rate *rate = rm692h5_get_rate(ctx);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->wake = ktime_get();
	if (!ctx->retained)
		return rm692h5_cold(ctx, rate);

	/* Still initialized: only a new refresh rate goes out. */
	if (rate != ctx->rate) {
		ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
		rm692h5_send_rate(&dsi_ctx, rate);
		if (dsi_ctx.accum_err) {
			ctx->retained = false;
			gpiod_set_value_cansleep(ctx->reset_gpio, 1);
			rm692h5_power_off(ctx);
			return rm692h5_cold(ctx, rate);
		}
		ctx->rate = rate;
	}

	return 0;
}

static int rm692h5_enable(struct drm_panel *panel)
{
	struct rm692h5 *ctx = to_rm692h5(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	bool retained = ctx->retained;
	int ret = 0;

	/*
	 * No power-mode read here: the controller flags a FIFO error after
	 * this panel's answer, and the next command can then time out.
	 */
	if (retained) {
		bool leave = ctx->idle && !READ_ONCE(ambient);

		ctx->retained = false;
		/* While ambient stays set the host only woke to draw. */
		if (leave)
			rm692h5_idle_off(&dsi_ctx, ctx->panel.backlight);
		ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
		mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
		ret = dsi_ctx.accum_err;
		if (!ret && leave)
			ctx->idle = false;
		if (ret) {
			/* Never leave a blank panel behind: start it over. */
			dev_warn(&ctx->dsi->dev, "retained panel did not wake (%d); restarting it\n", ret);
			gpiod_set_value_cansleep(ctx->reset_gpio, 1);
			rm692h5_power_off(ctx);
			ret = rm692h5_cold(ctx, ctx->rate);
		}
	}

	ctx->enabled = !ret;
	dev_info(&ctx->dsi->dev, "wake retained=%d took %lld us result=%d\n",
		 retained, ktime_us_delta(ktime_get(), ctx->wake), ret);
	return ret;
}

static int rm692h5_disable(struct drm_panel *panel)
{
	struct rm692h5 *ctx = to_rm692h5(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->enabled = false;
	if (ctx->shutting_down)
		return 0;

	if (READ_ONCE(ambient)) {
		u16 level = min_t(uint, ambient_brightness, RM692H5_MAX_BRIGHTNESS);
		u8 set_level[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, level >> 8, level & 0xff };

		bool was_idle = ctx->idle;

		if (was_idle)
			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, set_level, sizeof(set_level));
		else
			rm692h5_idle_on(&dsi_ctx, level);
		ctx->retained = !dsi_ctx.accum_err;
		ctx->idle = ctx->retained;
		if (!was_idle)
			dev_info(&ctx->dsi->dev, "blank into idle mode at 0x%04x: %d\n",
				 level, dsi_ctx.accum_err);
		return 0;
	}

	if (!retain)
		return 0;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	ctx->retained = !dsi_ctx.accum_err;

	return 0;
}

static int rm692h5_unprepare(struct drm_panel *panel)
{
	struct rm692h5 *ctx = to_rm692h5(panel);
	int ret;

	if (ctx->retained)
		return 0;

	ret = rm692h5_off(ctx);
	if (ret < 0)
		dev_err(&ctx->dsi->dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	rm692h5_power_off(ctx);

	return 0;
}

static int rm692h5_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct rm692h5 *ctx = to_rm692h5(panel);
	struct drm_display_mode *mode;
	unsigned int i;

	ctx->connector = connector;

	for (i = 0; i < ARRAY_SIZE(rm692h5_rates); i++) {
		mode = drm_mode_duplicate(connector->dev, &rm692h5_rates[i].mode);
		if (!mode)
			return -ENOMEM;

		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = rm692h5_rates[0].mode.width_mm;
	connector->display_info.height_mm = rm692h5_rates[0].mode.height_mm;

	return ARRAY_SIZE(rm692h5_rates);
}

static const struct drm_panel_funcs rm692h5_panel_funcs = {
	.prepare = rm692h5_prepare,
	.enable = rm692h5_enable,
	.disable = rm692h5_disable,
	.unprepare = rm692h5_unprepare,
	.get_modes = rm692h5_get_modes,
};

static int rm692h5_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct rm692h5 *ctx = mipi_dsi_get_drvdata(dsi);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	/*
	 * Idle mode keeps ambient_brightness, also while the host wakes only to
	 * draw, and leaving it restores this level. A blank into idle mode sets
	 * its own level: brightness 0 first would show a dark frame, and the
	 * host may already be off.
	 */
	if (ctx->idle || (READ_ONCE(ambient) && backlight_is_blank(bl)))
		return 0;

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
	rm692h5_panel = ctx;

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
	rm692h5_panel = NULL;
}

/* Shutdown blanks with the full power cycle. */
static void rm692h5_shutdown(struct mipi_dsi_device *dsi)
{
	struct rm692h5 *ctx = mipi_dsi_get_drvdata(dsi);

	ctx->shutting_down = true;
}

static const struct of_device_id rm692h5_of_match[] = {
	{ .compatible = "nubia,nx809j-rm692h5" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rm692h5_of_match);

static struct mipi_dsi_driver rm692h5_driver = {
	.probe = rm692h5_probe,
	.remove = rm692h5_remove,
	.shutdown = rm692h5_shutdown,
	.driver = {
		.name = "panel-raydium-rm692h5",
		.of_match_table = rm692h5_of_match,
	},
};
module_mipi_dsi_driver(rm692h5_driver);

MODULE_DESCRIPTION("DRM driver for Raydium RM692H5 based DSC panels");
MODULE_LICENSE("GPL");
