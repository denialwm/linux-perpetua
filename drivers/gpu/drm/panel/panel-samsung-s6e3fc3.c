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
 *
 * The panel keeps that configuration across sleep, so it is blanked with
 * display off and sleep in, and brought back with sleep out, tear on and
 * display on. It is never reset or powered down: the full initialisation
 * after a reset is not known.
 *
 * It runs at 60 or 90 Hz; a change of rate is a full modeset.
 *
 * The first enable after takeover sends nothing at 60 Hz: the panel is
 * already out of sleep with tear and display on, and while it is awake the
 * DSI host holds a command sent after tear on until DECON sends a frame,
 * which it does only after the panel is enabled.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/fp_illumination.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <video/mipi_display.h>

#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

/* WRCTRLD: brightness control on, high brightness mode off. */
#define S6E3FC3_WRCTRLD_BCTRL		BIT(5)

/*
 * Levels from 2048 up need high brightness mode; they are not offered.
 * The lowest few levels do not light the panel evenly.
 */
#define S6E3FC3_BRIGHTNESS_MIN		4
#define S6E3FC3_BRIGHTNESS_MAX		2047
#define S6E3FC3_BRIGHTNESS_DEFAULT	1023

/* A brightness written right after display on is not latched. */
#define S6E3FC3_BL_REAPPLY_MS		100

static const char * const s6e3fc3_supplies[] = { "vci", "vddi" };

struct s6e3fc3_fp;

struct s6e3fc3 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_connector *connector;
	struct gpio_desc *reset;
	struct delayed_work bl_work;
	struct s6e3fc3_fp *fp;
	/* Serialises brightness writes against display on and off. */
	struct mutex lock;
	/* The frequency register of the mode last enabled. */
	u8 freq;
	/* Display on was sent: brightness writes take effect. */
	bool on;
	/* Still as the bootloader left it: awake, tear and display on. */
	bool boot_on;
};

static inline struct s6e3fc3 *to_s6e3fc3(struct drm_panel *panel)
{
	return container_of(panel, struct s6e3fc3, panel);
}

#define S6E3FC3_MODE(_clock, _type)		\
	.clock = _clock,			\
	.hdisplay = 1080,			\
	.hsync_start = 1080 + 32,		\
	.hsync_end = 1080 + 32 + 12,		\
	.htotal = 1080 + 32 + 12 + 26,		\
	.vdisplay = 2400,			\
	.vsync_start = 2400 + 12,		\
	.vsync_end = 2400 + 12 + 4,		\
	.vtotal = 2400 + 12 + 4 + 26,		\
	.width_mm = 67,				\
	.height_mm = 148,			\
	.type = DRM_MODE_TYPE_DRIVER | (_type)

/*
 * In command mode the panel sets the frame rate itself, from its frequency
 * register, and the host follows its TE. The bootloader leaves it at 60 Hz.
 */
static const struct {
	struct drm_display_mode mode;
	u8 freq;	/* register 0x60 */
} s6e3fc3_modes[] = {
	{ { S6E3FC3_MODE(168498, DRM_MODE_TYPE_PREFERRED) }, 0x00 },	/* 60 Hz */
	{ { S6E3FC3_MODE(252747, 0) }, 0x08 },				/* 90 Hz */
};

/* The frequency for the mode being enabled, as panel-novatek-nt35950 does. */
static u8 s6e3fc3_freq(struct s6e3fc3 *ctx)
{
	struct drm_connector *connector = ctx->connector;
	struct drm_crtc_state *crtc_state;
	int i;

	if (!connector || !connector->state || !connector->state->crtc)
		return s6e3fc3_modes[0].freq;

	crtc_state = connector->state->crtc->state;
	for (i = 0; i < ARRAY_SIZE(s6e3fc3_modes); i++)
		if (drm_mode_match(&crtc_state->mode, &s6e3fc3_modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK))
			return s6e3fc3_modes[i].freq;

	return s6e3fc3_modes[0].freq;
}

static void s6e3fc3_set_freq(struct mipi_dsi_multi_context *dsi_ctx, u8 freq)
{
	/* Unlock level 2, write, latch, lock again. */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_var_seq_multi(dsi_ctx, 0x60, freq);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf7, 0x0f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf0, 0xa5, 0xa5);
}

#include "panel-samsung-s6e3fc3-fp.h"

/* Called with ctx->lock held. */
static int s6e3fc3_write_brightness(struct s6e3fc3 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	int level = backlight_get_brightness(ctx->panel.backlight);

	if (!ctx->on)
		return 0;

	if (level && level < S6E3FC3_BRIGHTNESS_MIN)
		level = S6E3FC3_BRIGHTNESS_MIN;

	mutex_lock(&ctx->fp->lock);
	mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
					 s6e3fc3_fp_wrctrld(ctx->fp));
	if (!dsi_ctx.accum_err)
		dsi_ctx.accum_err =
			mipi_dsi_dcs_set_display_brightness_large(ctx->dsi, level);
	mutex_unlock(&ctx->fp->lock);

	return dsi_ctx.accum_err;
}

static void s6e3fc3_bl_work(struct work_struct *work)
{
	struct s6e3fc3 *ctx = container_of(to_delayed_work(work),
					   struct s6e3fc3, bl_work);

	mutex_lock(&ctx->lock);
	s6e3fc3_write_brightness(ctx);
	mutex_unlock(&ctx->lock);
}

static int s6e3fc3_bl_update_status(struct backlight_device *bl)
{
	struct s6e3fc3 *ctx = bl_get_data(bl);
	int ret;

	mutex_lock(&ctx->lock);
	ret = s6e3fc3_write_brightness(ctx);
	mutex_unlock(&ctx->lock);

	return ret;
}

static const struct backlight_ops s6e3fc3_bl_ops = {
	.update_status = s6e3fc3_bl_update_status,
};

static int s6e3fc3_prepare(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = to_s6e3fc3(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (ctx->boot_on)
		return 0;

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	/* DECON's hardware trigger runs on TE. */
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	return dsi_ctx.accum_err;
}

static int s6e3fc3_enable(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = to_s6e3fc3(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u8 freq = s6e3fc3_freq(ctx);

	mutex_lock(&ctx->lock);
	if (!ctx->boot_on || freq != s6e3fc3_modes[0].freq)
		s6e3fc3_set_freq(&dsi_ctx, freq);
	if (!ctx->boot_on)
		mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	ctx->boot_on = false;
	ctx->freq = freq;
	ctx->on = !dsi_ctx.accum_err;
	s6e3fc3_fp_panel(ctx->fp, ctx->on);
	mutex_unlock(&ctx->lock);

	if (ctx->on)
		schedule_delayed_work(&ctx->bl_work,
				      msecs_to_jiffies(S6E3FC3_BL_REAPPLY_MS));

	return dsi_ctx.accum_err;
}

static int s6e3fc3_disable(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = to_s6e3fc3(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	cancel_delayed_work_sync(&ctx->bl_work);

	mutex_lock(&ctx->lock);
	s6e3fc3_fp_panel(ctx->fp, false);
	ctx->on = false;
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mutex_unlock(&ctx->lock);

	return dsi_ctx.accum_err;
}

static int s6e3fc3_unprepare(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = to_s6e3fc3(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->boot_on = false;
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	/* Sleep out must not follow sleep in sooner. */
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int s6e3fc3_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct s6e3fc3 *ctx = to_s6e3fc3(panel);
	struct drm_display_mode *mode;
	int i;

	ctx->connector = connector;

	for (i = 0; i < ARRAY_SIZE(s6e3fc3_modes); i++) {
		mode = drm_mode_duplicate(connector->dev,
					  &s6e3fc3_modes[i].mode);
		if (!mode)
			return -ENOMEM;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = s6e3fc3_modes[0].mode.width_mm;
	connector->display_info.height_mm = s6e3fc3_modes[0].mode.height_mm;

	return ARRAY_SIZE(s6e3fc3_modes);
}

static const struct drm_panel_funcs s6e3fc3_panel_funcs = {
	.prepare = s6e3fc3_prepare,
	.enable = s6e3fc3_enable,
	.disable = s6e3fc3_disable,
	.unprepare = s6e3fc3_unprepare,
	.get_modes = s6e3fc3_get_modes,
};

static int s6e3fc3_probe(struct mipi_dsi_device *dsi)
{
	const struct backlight_properties bl_props = {
		.type = BACKLIGHT_RAW,
		.brightness = S6E3FC3_BRIGHTNESS_DEFAULT,
		.max_brightness = S6E3FC3_BRIGHTNESS_MAX,
	};
	struct device *dev = &dsi->dev;
	struct s6e3fc3 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e3fc3, panel,
				   &s6e3fc3_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	ctx->boot_on = true;
	mipi_dsi_set_drvdata(dsi, ctx);

	ret = devm_mutex_init(dev, &ctx->lock);
	if (ret)
		return ret;

	/* Already on: this takes the references that keep them on. */
	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(s6e3fc3_supplies),
					     s6e3fc3_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");

	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset),
				     "failed to get reset GPIO\n");

	/* Before the backlight: its writes take the illumination lock. */
	ret = s6e3fc3_fp_init(ctx);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register fingerprint illumination\n");

	ctx->panel.backlight = devm_backlight_device_register(dev, dev_name(dev),
							      dev, ctx,
							      &s6e3fc3_bl_ops,
							      &bl_props);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "failed to register backlight\n");

	/* After the backlight, so that it is cancelled before it goes. */
	ret = devm_delayed_work_autocancel(dev, &ctx->bl_work, s6e3fc3_bl_work);
	if (ret)
		return ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

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
