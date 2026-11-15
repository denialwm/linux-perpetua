// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT37706 AMOLED DSI panel, as fitted by CSOT to the Motorola
 * Edge 70 (roadstr): 1220x2712, command mode, DSC 1.1 at 10 bits per
 * component.
 *
 * The initialisation sequence is the vendor's "v3" table for this panel.
 * The vendor's video and command mode tables differ only in what follows
 * it, so the panel takes either link; in command mode it keeps the last
 * frame in its own memory. Its AOD blocks are kept: the panel was
 * validated with them, and they only take effect in AOD.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/fp_illumination.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

#define NT37706_BRIGHTNESS_DEFAULT	6426
#define NT37706_BRIGHTNESS_MAX		16180

/*
 * Always-on display, set by the compositor while its lock screen shows the
 * always-on picture. Blanking then puts the panel in DCS idle mode at
 * ambient_brightness, as stock's command mode switch does, instead of
 * turning it off: it keeps showing the last frame from its own memory while
 * the host powers down, and an unblank keeps idle mode, since the host only
 * woke to draw a new frame. Clearing it leaves idle mode, at once on a lit
 * panel or else at the next unblank. The backlight writes nothing while
 * ambient blanks or idle mode holds, so the switch shows no dark frame.
 */
static bool ambient;
static int nt37706_set_ambient(const char *val, const struct kernel_param *kp);
static const struct kernel_param_ops nt37706_ambient_ops = {
	.set = nt37706_set_ambient,
	.get = param_get_bool,
};
module_param_cb(ambient, &nt37706_ambient_ops, &ambient, 0644);
MODULE_PARM_DESC(ambient, "Blank into idle mode, keeping the last frame lit");

static uint ambient_brightness = 0x0554;
module_param(ambient_brightness, uint, 0644);
MODULE_PARM_DESC(ambient_brightness, "Idle mode brightness code (stock 0x0554)");

struct nt37706_fp;

struct novatek_nt37706 {
	struct drm_panel panel;
	struct nt37706_fp *fp;
	struct mipi_dsi_device *dsi;
	struct drm_connector *connector;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator *vddio;
	struct regulator *vdd;
	struct regulator *vci;
	/* The rate code the panel runs; it outlives a retained blank. */
	u8 rate;
	/* Kept powered through a blank; lit (enable to disable); idle mode. */
	bool retained, enabled, idle;
};

/* The one panel, for the ambient parameter. */
static struct novatek_nt37706 *nt37706_panel;

static inline struct novatek_nt37706 *to_novatek_nt37706(struct drm_panel *panel)
{
	return container_of(panel, struct novatek_nt37706, panel);
}

#define nt37706_switch_page(dsi_ctx, page) \
	mipi_dsi_dcs_write_seq_multi((dsi_ctx), 0xf0, 0x55, 0xaa, 0x52, 0x08, (page))

#include "panel-novatek-nt37706-fp.h"

/* Stock cmd-mode-switch-in: the idle level (second 0x51 register), idle on. */
static int nt37706_idle_on(struct novatek_nt37706 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u16 level = min_t(uint, ambient_brightness, NT37706_BRIGHTNESS_MAX);
	u8 set_level[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, level >> 8, level & 0xff };

	mutex_lock(&ctx->fp->lock);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x04);
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, set_level, sizeof(set_level));
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, (u8[]){ MIPI_DCS_ENTER_IDLE_MODE }, 1);
	mutex_unlock(&ctx->fp->lock);

	return dsi_ctx.accum_err;
}

/* Stock cmd-mode-switch-out: the backlight's level, then idle off. */
static int nt37706_idle_off(struct novatek_nt37706 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u16 level = min_t(u32, ctx->panel.backlight->props.brightness,
			  NT37706_BRIGHTNESS_MAX);
	u8 set_level[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, level >> 8, level & 0xff };

	mutex_lock(&ctx->fp->lock);
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, set_level, sizeof(set_level));
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, (u8[]){ MIPI_DCS_EXIT_IDLE_MODE }, 1);
	mutex_unlock(&ctx->fp->lock);

	return dsi_ctx.accum_err;
}

static int nt37706_set_ambient(const char *val, const struct kernel_param *kp)
{
	struct novatek_nt37706 *ctx = READ_ONCE(nt37706_panel);
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;
	WRITE_ONCE(ambient, on);
	if (on || !ctx || !ctx->enabled || !ctx->idle)
		return 0;

	ret = nt37706_idle_off(ctx);
	if (ret)
		return ret;
	ctx->idle = false;
	return 0;
}

static int novatek_nt37706_power_on(struct novatek_nt37706 *ctx)
{
	int ret;

	/* Vendor order and settle times: vddio, vdd, then vci. */
	ret = regulator_enable(ctx->vddio);
	if (ret)
		return ret;
	usleep_range(1000, 1500);

	ret = regulator_enable(ctx->vdd);
	if (ret)
		goto err_vddio;
	usleep_range(1000, 1500);

	ret = regulator_enable(ctx->vci);
	if (ret)
		goto err_vdd;
	usleep_range(2000, 2500);

	/* Vendor reset: high 2 ms, low 5 ms, high 20 ms. */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(2000, 2500);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 5500);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(20000, 21000);

	return 0;

err_vdd:
	regulator_disable(ctx->vdd);
err_vddio:
	regulator_disable(ctx->vddio);
	return ret;
}

static void novatek_nt37706_power_off(struct novatek_nt37706 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vci);
	usleep_range(1000, 1500);
	regulator_disable(ctx->vdd);
	usleep_range(1000, 1500);
	regulator_disable(ctx->vddio);
}

static int novatek_nt37706_on(struct novatek_nt37706 *ctx, u8 rate)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x46);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x07, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x4a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x08, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x56);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x44, 0x44);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x73);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x2a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x08);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf5, 0x02);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x81);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x3c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf5, 0x84);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x81, 0x01, 0x19);

	/* AOD configuration. */
	nt37706_switch_page(&dsi_ctx, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x10, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x17, 0x21);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x71, 0x10);
	nt37706_switch_page(&dsi_ctx, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6, 0xff, 0x80, 0x00);
	nt37706_switch_page(&dsi_ctx, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd2, 0x03);
	nt37706_switch_page(&dsi_ctx, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x8d,
				     0x00, 0x00, 0x04, 0xc3, 0x00, 0x00, 0x05, 0xdb);

	mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, 0, 1220 - 1);
	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0, 2712 - 1);

	/* Panel-side DSC: enable, then 10 bpc, line buffer depth 11. */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x03, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x43);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91,
				     0xab, 0x28, 0x00, 0x0c, 0xc2, 0x00, 0x02,
				     0x32, 0x01, 0x31, 0x00, 0x08, 0x08, 0xbb,
				     0x07, 0x7b, 0x10, 0xf0);

	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, (u8[]){ 0x2f, rate }, 2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_GAMMA_CURVE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x00, 0x18, 0x00, 0x34);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x00, 0x18, 0x00, 0x34);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x00, 0x18, 0x03, 0xd8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x00, 0x18, 0x0b, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x00, 0x18, 0x00, 0x34);

	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	/* Brightness starts at zero; the backlight device sets it later. */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x00, 0x00);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5f, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x57, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x88, 0x01, 0x02, 0x62, 0x09, 0x88);

	nt37706_switch_page(&dsi_ctx, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0, 0x33);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 105);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int novatek_nt37706_off(struct novatek_nt37706 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 17);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

/*
 * The pixel clock is fixed; the vendor changes the refresh rate through
 * the vertical front porch alone.
 */
#define NT37706_MODE(fp)						\
	.clock = 448979,						\
	.hdisplay = 1220,						\
	.hsync_start = 1220 + 26,					\
	.hsync_end = 1220 + 26 + 16,					\
	.htotal = 1220 + 26 + 16 + 80,					\
	.vdisplay = 2712,						\
	.vsync_start = 2712 + (fp),					\
	.vsync_end = 2712 + (fp) + 2,					\
	.vtotal = 2712 + (fp) + 2 + 22,					\
	.width_mm = 69,							\
	.height_mm = 154,						\
	.type = DRM_MODE_TYPE_DRIVER

static const struct {
	struct drm_display_mode mode;
	u8 rate; /* DCS 0x2f argument */
} novatek_nt37706_modes[] = {
	{ { NT37706_MODE(52) }, 0x00 },		/* 120 Hz */
	{ { NT37706_MODE(981) }, 0x02 },	/* 90 Hz */
	{ { NT37706_MODE(2840) }, 0x03 },	/* 60 Hz */
};

/* The rate code for the mode being enabled, as panel-novatek-nt35950 does. */
static u8 novatek_nt37706_rate(struct novatek_nt37706 *ctx)
{
	struct drm_connector *connector = ctx->connector;
	struct drm_crtc_state *crtc_state;
	int i;

	if (!connector || !connector->state || !connector->state->crtc)
		return novatek_nt37706_modes[0].rate;

	crtc_state = connector->state->crtc->state;
	for (i = 0; i < ARRAY_SIZE(novatek_nt37706_modes); i++)
		if (drm_mode_match(&crtc_state->mode, &novatek_nt37706_modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK))
			return novatek_nt37706_modes[i].rate;

	return novatek_nt37706_modes[0].rate;
}


static int novatek_nt37706_prepare(struct drm_panel *panel)
{
	struct novatek_nt37706 *ctx = to_novatek_nt37706(panel);
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u8 rate = novatek_nt37706_rate(ctx);
	int ret;

	/* Still initialized: only a new refresh rate goes out. */
	if (ctx->retained) {
		if (rate != ctx->rate) {
			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, (u8[]){ 0x2f, rate }, 2);
			if (!dsi_ctx.accum_err)
				ctx->rate = rate;
		}
		nt37706_fp_panel(ctx->fp, true);
		return 0;
	}

	ret = novatek_nt37706_power_on(ctx);
	if (ret)
		return ret;

	/* The vendor sends everything until display-on in low-power mode. */
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/* As downstream does, the PPS goes out before the panel is woken. */
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	if (ret < 0) {
		dev_err(dev, "failed to send PPS: %d\n", ret);
		goto err;
	}

	ret = novatek_nt37706_on(ctx, rate);
	if (ret < 0) {
		dev_err(dev, "failed to initialise panel: %d\n", ret);
		goto err;
	}

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ctx->rate = rate;
	ctx->idle = false;
	nt37706_fp_panel(ctx->fp, true);
	return 0;

err:
	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	novatek_nt37706_power_off(ctx);
	return ret;
}

static int novatek_nt37706_enable(struct drm_panel *panel)
{
	struct novatek_nt37706 *ctx = to_novatek_nt37706(panel);
	int ret;

	/* While ambient stays set the host only woke to draw. */
	if (ctx->retained) {
		ctx->retained = false;
		if (ctx->idle && !READ_ONCE(ambient)) {
			ret = nt37706_idle_off(ctx);
			if (ret)
				dev_err(&ctx->dsi->dev, "failed to leave idle mode: %d\n", ret);
			else
				ctx->idle = false;
		}
	}

	ctx->enabled = true;
	return 0;
}

static int novatek_nt37706_disable(struct drm_panel *panel)
{
	struct novatek_nt37706 *ctx = to_novatek_nt37706(panel);
	int ret;

	ctx->enabled = false;
	if (!READ_ONCE(ambient))
		return 0;

	if (!ctx->idle) {
		ret = nt37706_idle_on(ctx);
		if (ret) {
			dev_err(&ctx->dsi->dev, "failed to enter idle mode: %d\n", ret);
			return 0;
		}
		ctx->idle = true;
	}
	ctx->retained = true;
	return 0;
}

static int novatek_nt37706_unprepare(struct drm_panel *panel)
{
	struct novatek_nt37706 *ctx = to_novatek_nt37706(panel);
	int ret;

	nt37706_fp_panel(ctx->fp, false);
	if (ctx->retained)
		return 0;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = novatek_nt37706_off(ctx);
	if (ret < 0)
		dev_err(&ctx->dsi->dev, "failed to power down panel: %d\n", ret);
	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	novatek_nt37706_power_off(ctx);
	return 0;
}

static int novatek_nt37706_get_modes(struct drm_panel *panel,
				     struct drm_connector *connector)
{
	struct novatek_nt37706 *ctx = to_novatek_nt37706(panel);
	int i;

	for (i = 0; i < ARRAY_SIZE(novatek_nt37706_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &novatek_nt37706_modes[i].mode);
		if (!mode)
			return -ENOMEM;
		drm_mode_set_name(mode);
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = novatek_nt37706_modes[0].mode.width_mm;
	connector->display_info.height_mm = novatek_nt37706_modes[0].mode.height_mm;
	ctx->connector = connector;

	return ARRAY_SIZE(novatek_nt37706_modes);
}

static const struct drm_panel_funcs novatek_nt37706_panel_funcs = {
	.prepare = novatek_nt37706_prepare,
	.enable = novatek_nt37706_enable,
	.disable = novatek_nt37706_disable,
	.unprepare = novatek_nt37706_unprepare,
	.get_modes = novatek_nt37706_get_modes,
};

static int novatek_nt37706_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct novatek_nt37706 *ctx = mipi_dsi_get_drvdata(dsi);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	/* The link is down while the panel is off; enabling it sends this. */
	if (!ctx->panel.prepared)
		return 0;

	/* Idle mode keeps its own level; leaving it restores this one. */
	if (ctx->idle || (READ_ONCE(ambient) && backlight_is_blank(bl)))
		return 0;

	/* Sent in high-speed mode, big-endian, as the vendor does. */
	mutex_lock(&ctx->fp->lock);
	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	mutex_unlock(&ctx->fp->lock);
	return ret;
}

static int novatek_nt37706_bl_get_brightness(struct backlight_device *bl)
{
	return backlight_get_brightness(bl);
}

static const struct backlight_ops novatek_nt37706_bl_ops = {
	.update_status = novatek_nt37706_bl_update_status,
	.get_brightness = novatek_nt37706_bl_get_brightness,
};

static struct backlight_device *
novatek_nt37706_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = NT37706_BRIGHTNESS_DEFAULT,
		.max_brightness = NT37706_BRIGHTNESS_MAX,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &novatek_nt37706_bl_ops, &props);
}

static int novatek_nt37706_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct novatek_nt37706 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct novatek_nt37706, panel,
				   &novatek_nt37706_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio), "failed to get vddio\n");
	ctx->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ctx->vdd))
		return dev_err_probe(dev, PTR_ERR(ctx->vdd), "failed to get vdd\n");
	ctx->vci = devm_regulator_get(dev, "vci");
	if (IS_ERR(ctx->vci))
		return dev_err_probe(dev, PTR_ERR(ctx->vci), "failed to get vci\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB101010;
	/*
	 * Command mode, paced by the panel's tearing signal; both DSC slices
	 * of a line in one packet.
	 */
	dsi->mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;

	/* Before the backlight: its writes take the illumination lock. */
	ret = nt37706_fp_init(ctx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register fingerprint illumination\n");

	ctx->panel.prepare_prev_first = true;
	ctx->panel.backlight = novatek_nt37706_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "failed to create backlight\n");

	drm_panel_add(&ctx->panel);
	WRITE_ONCE(nt37706_panel, ctx);

	dsi->dsc = &ctx->dsc;
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 12;
	ctx->dsc.slice_width = 610;
	ctx->dsc.slice_count = 1220 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 10;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void novatek_nt37706_remove(struct mipi_dsi_device *dsi)
{
	struct novatek_nt37706 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	WRITE_ONCE(nt37706_panel, NULL);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id novatek_nt37706_of_match[] = {
	{ .compatible = "csot,nt37706-v3" },
	{}
};
MODULE_DEVICE_TABLE(of, novatek_nt37706_of_match);

static struct mipi_dsi_driver novatek_nt37706_driver = {
	.probe = novatek_nt37706_probe,
	.remove = novatek_nt37706_remove,
	.driver = {
		.name = "panel-novatek-nt37706",
		.of_match_table = novatek_nt37706_of_match,
	},
};
module_mipi_dsi_driver(novatek_nt37706_driver);

MODULE_AUTHOR("Doctor Logix <doctor.logix@gmail.com>");
MODULE_DESCRIPTION("Panel driver for the CSOT Novatek NT37706 AMOLED panel");
MODULE_LICENSE("GPL");
