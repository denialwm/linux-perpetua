// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 FIXME
 * Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
 *   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)
 * Copyright (c) 2026 Doctor Logix <doctor.logix@gmail.com>
 *
 * BOE AA551 (vendor name AA551-P-3-A0004) 1264x2780 AMOLED panel of the
 * OnePlus 12R and Ace 3 (aston), driven in command mode with DSC 1.2.
 *
 * The DSI lanes reach the panel through a Pixelworks Iris7P display
 * processor, which this driver powers and keeps in its analog bypass.
 */

#include <linux/backlight.h>
#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/aston_fp_illumination.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/gpio/consumer.h>
#include <linux/irqflags.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

#define AA551_BRIGHTNESS_DEFAULT	1433
#define AA551_BRIGHTNESS_MAX		4094

/*
 * Iris7P analog bypass (ABYP). After a reset the Iris7P should report the
 * bypass on its ABYP-ready line. If it does not, the host forces it with a
 * one-wire command of six pulses on the wakeup line. A bounded status wait
 * is followed by direct factory panel status validation, without reset loops.
 */
#define IRIS_ONE_WIRE_PULSE_US		56
#define IRIS_ONE_WIRE_COMMAND_US	10
#define IRIS_ONE_WIRE_END_US		227
#define IRIS_FORCE_ABYP_PULSES		6
#define IRIS_ABYP_POLL_COUNT		50

struct aa551;

struct aa551_fp {
	struct kref refs;
	struct mutex lock;
	struct miscdevice misc;
	struct delayed_work timeout;
	struct aa551 *ctx;
	u16 brightness;
	u8 gamma[5][6];
	bool opened;
	bool prepared;
	bool panel_on;
	bool active;
	bool gamma_valid;
	bool gamma_programmed;
	bool registered;
};

struct aa551 {
	struct drm_panel panel;
	struct aa551_fp *fp;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct regulator *vddio;
	struct regulator *vci;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *iris_vdd0p9_gpio;
	struct gpio_desc *iris_reset_gpio;
	struct gpio_desc *iris_wakeup_gpio;
	struct gpio_desc *iris_abyp_ready_gpio;
	bool powered, standby_enabled, retained, iris_confirmed, shutting_down;
	u32 cold_wakes, retained_wakes;
	u64 prepare_us, enable_us;
	u8 power_mode;
};

static inline struct aa551 *to_aa551(struct drm_panel *panel)
{
	return container_of(panel, struct aa551, panel);
}

#define aa551_switch_page(dsi_ctx, page) \
	mipi_dsi_dcs_write_seq_multi((dsi_ctx), 0xff, 0x08, 0x38, (page))

#include "panel-oneplus-aston-aa551-alpha.h"
#include "panel-oneplus-aston-aa551-state.h"

#define AA551_FP_TIMEOUT_MS 30000
#define AA551_FP_MAX_DBV 0x0dbb

static int aa551_bl_write_locked(struct aa551 *ctx, u16 brightness);

/* Factory OTP gamma extrapolation for the local fingerprint region. This is
 * read/programmed only on the first explicit illumination enable, never probe.
 */
static int aa551_fp_gamma_locked(struct aa551_fp *fp)
{
	struct mipi_dsi_device *dsi = fp->ctx->dsi;
	struct mipi_dsi_multi_context dc = { .dsi = dsi };
	struct mipi_dsi_multi_context close = { .dsi = dsi };
	static const int levels[] = { 173, 205, 226, 247, 255 };
	u8 a[5] = {}, b[5] = {};
	int rgb_a[3], rgb_b[3], rgb[3];
	unsigned long flags = dsi->mode_flags;
	int i, c, ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	if (!fp->gamma_valid) {
		aa551_switch_page(&dc, 0x2f);
		mipi_dsi_dcs_write_seq_multi(&dc, 0x99, 0xb2);
		if (!dc.accum_err) {
			ret = mipi_dsi_dcs_read(dsi, 0x96, a, sizeof(a));
			if (ret != sizeof(a))
				dc.accum_err = ret < 0 ? ret : -EIO;
		}
		if (!dc.accum_err) {
			ret = mipi_dsi_dcs_read(dsi, 0x97, b, sizeof(b));
			if (ret != sizeof(b))
				dc.accum_err = ret < 0 ? ret : -EIO;
		}
		/* Always close OTP access, including failed/short reads. */
		mipi_dsi_dcs_write_seq_multi(&close, 0x99, 0x00);
		aa551_switch_page(&close, 0x00);
		if (dc.accum_err || close.accum_err) {
			ret = dc.accum_err ?: close.accum_err;
			goto out;
		}
		rgb_a[0] = ((a[0] & 0xf) << 8) | a[2];
		rgb_a[1] = ((a[1] & 0xf0) << 4) | a[3];
		rgb_a[2] = ((a[1] & 0xf) << 8) | a[4];
		rgb_b[0] = ((b[0] & 0xf) << 8) | b[2];
		rgb_b[1] = ((b[1] & 0xf0) << 4) | b[3];
		rgb_b[2] = ((b[1] & 0xf) << 8) | b[4];
		for (i = 0; i < ARRAY_SIZE(levels); i++) {
			for (c = 0; c < 3; c++) {
				rgb[c] = (levels[i] * 110 - 24700) *
					 (rgb_b[c] - rgb_a[c]) / 800 + rgb_a[c];
				if (rgb[c] <= 0 || rgb[c] > 0xfff) {
					ret = -ERANGE;
					goto out;
				}
			}
			fp->gamma[i][0] = 0x93 + i;
			fp->gamma[i][1] = rgb[0] >> 8;
			fp->gamma[i][2] = ((rgb[1] >> 4) & 0xf0) | (rgb[2] >> 8);
			fp->gamma[i][3] = rgb[0];
			fp->gamma[i][4] = rgb[1];
			fp->gamma[i][5] = rgb[2];
		}
		fp->gamma_valid = true;
	}

	aa551_switch_page(&dc, 0x2f);
	mipi_dsi_dcs_write_seq_multi(&dc, 0x99, 0xe4);
	for (i = 0; i < ARRAY_SIZE(fp->gamma); i++)
		mipi_dsi_dcs_write_buffer_multi(&dc, fp->gamma[i], sizeof(fp->gamma[i]));
	close.accum_err = 0;
	mipi_dsi_dcs_write_seq_multi(&close, 0x99, 0x00);
	aa551_switch_page(&close, 0x00);
	ret = dc.accum_err ?: close.accum_err;
	if (!ret)
		fp->gamma_programmed = true;
out:
	dsi->mode_flags = flags;
	return ret;
}

static int aa551_fp_alpha_locked(struct aa551_fp *fp)
{
	struct mipi_dsi_device *dsi = fp->ctx->dsi;
	struct mipi_dsi_multi_context dc = { .dsi = dsi };
	unsigned long flags = dsi->mode_flags;
	u16 alpha = aa551_fp_alpha[min_t(u16, fp->brightness,
					 ARRAY_SIZE(aa551_fp_alpha) - 1)];

	/* Factory alpha writes use HS; icon/gamma commands use LP. */
	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	aa551_switch_page(&dc, 0x00);
	mipi_dsi_dcs_write_var_seq_multi(&dc, 0x9a, alpha >> 8, alpha & 0xff);
	dsi->mode_flags = flags;
	return dc.accum_err;
}

static int aa551_fp_off_locked(struct aa551_fp *fp)
{
	struct aa551 *ctx = fp->ctx;
	struct mipi_dsi_multi_context dc;
	unsigned long flags;
	int ret;

	cancel_delayed_work(&fp->timeout);
	if (!ctx || !fp->prepared) {
		fp->active = false;
		return 0;
	}
	if (!fp->active)
		return 0;
	dc = (struct mipi_dsi_multi_context) { .dsi = ctx->dsi };
	flags = ctx->dsi->mode_flags;
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	aa551_switch_page(&dc, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dc, 0x95, 0x10);
	ctx->dsi->mode_flags = flags;
	ret = dc.accum_err;
	if (ret) {
		/* Retain ownership and retry if hardware state is unconfirmed. */
		mod_delayed_work(system_wq, &fp->timeout, msecs_to_jiffies(1000));
		return ret;
	}
	fp->active = false;
	if (fp->panel_on)
		ret = aa551_bl_write_locked(ctx, fp->brightness);
	return ret;
}

static int aa551_fp_on_locked(struct aa551_fp *fp)
{
	struct aa551 *ctx = fp->ctx;
	struct mipi_dsi_multi_context dc = { .dsi = ctx->dsi };
	unsigned long flags = ctx->dsi->mode_flags;
	int ret;

	if (!fp->prepared || !fp->panel_on || !fp->brightness)
		return -EHOSTDOWN;
	if (fp->active)
		return 0; /* Repeated enable cannot extend the independent deadline. */
	if (!fp->gamma_programmed) {
		ret = aa551_fp_gamma_locked(fp);
		if (ret)
			return ret;
	}
	/* Only local-HBM operation caps normal brightness, following factory
	 * 900-nit handling. The requested brightness is retained for restoration.
	 */
	fp->active = true;
	ret = aa551_bl_write_locked(ctx, min_t(u16, fp->brightness, AA551_FP_MAX_DBV));
	if (ret)
		goto failed;
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	aa551_switch_page(&dc, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dc, 0x95, 0x13);
	ctx->dsi->mode_flags = flags;
	ret = dc.accum_err ?: aa551_fp_alpha_locked(fp);
	if (ret)
		goto failed;
	mod_delayed_work(system_wq, &fp->timeout, msecs_to_jiffies(AA551_FP_TIMEOUT_MS));
	return 0;
failed:
	aa551_fp_off_locked(fp);
	return ret;
}

static void aa551_fp_timeout(struct work_struct *work)
{
	struct aa551_fp *fp = container_of(to_delayed_work(work), struct aa551_fp, timeout);

	mutex_lock(&fp->lock);
	if (aa551_fp_off_locked(fp) && fp->ctx)
		dev_err(&fp->ctx->dsi->dev, "Failed to restore fingerprint illumination\n");
	mutex_unlock(&fp->lock);
}

static void aa551_fp_free(struct kref *refs)
{
	struct aa551_fp *fp = container_of(refs, struct aa551_fp, refs);

	mutex_destroy(&fp->lock);
	kfree(fp);
}

static int aa551_fp_open(struct inode *inode, struct file *file)
{
	struct aa551_fp *fp = container_of(file->private_data, struct aa551_fp, misc);
	int ret = 0;

	mutex_lock(&fp->lock);
	if (!fp->ctx)
		ret = -ENODEV;
	else if (fp->opened || fp->active)
		ret = -EBUSY;
	else {
		fp->opened = true;
		kref_get(&fp->refs);
		file->private_data = fp;
	}
	mutex_unlock(&fp->lock);
	return ret ?: nonseekable_open(inode, file);
}

static int aa551_fp_release(struct inode *inode, struct file *file)
{
	struct aa551_fp *fp = file->private_data;

	mutex_lock(&fp->lock);
	aa551_fp_off_locked(fp);
	fp->opened = false;
	mutex_unlock(&fp->lock);
	kref_put(&fp->refs, aa551_fp_free);
	return 0;
}

static long aa551_fp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct aa551_fp *fp = file->private_data;
	struct aston_fp_illumination_info info = {
		.version = 1,
		.panel_width = 1264,
		.panel_height = 2780,
		.max_duration_ms = AA551_FP_TIMEOUT_MS,
		.compatible = "oneplus,aston-boe-aa551",
	};
	u32 on;
	int ret = 0;

	mutex_lock(&fp->lock);
	if (!fp->ctx) {
		ret = -ENODEV;
		goto out;
	}
	switch (cmd) {
	case ASTON_FP_ILLUMINATION_GET_INFO:
		info.enabled = fp->active;
		info.panel_on = fp->panel_on;
		if (copy_to_user((void __user *)arg, &info, sizeof(info)))
			ret = -EFAULT;
		break;
	case ASTON_FP_ILLUMINATION_SET:
		if (copy_from_user(&on, (void __user *)arg, sizeof(on)))
			ret = -EFAULT;
		else if (on > 1)
			ret = -EINVAL;
		else
			ret = on ? aa551_fp_on_locked(fp) : aa551_fp_off_locked(fp);
		break;
	default:
		ret = -ENOTTY;
	}
out:
	mutex_unlock(&fp->lock);
	return ret;
}

static const struct file_operations aa551_fp_fops = {
	.owner = THIS_MODULE,
	.open = aa551_fp_open,
	.release = aa551_fp_release,
	.unlocked_ioctl = aa551_fp_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static void aa551_fp_stop(struct aa551_fp *fp)
{
	mutex_lock(&fp->lock);
	aa551_fp_off_locked(fp);
	fp->ctx = NULL;
	mutex_unlock(&fp->lock);
	if (fp->registered) {
		misc_deregister(&fp->misc);
		fp->registered = false;
	}
	cancel_delayed_work_sync(&fp->timeout);
}

static void aa551_fp_destroy(void *data)
{
	struct aa551_fp *fp = data;

	aa551_fp_stop(fp);
	kref_put(&fp->refs, aa551_fp_free);
}

static int aa551_fp_init(struct aa551 *ctx)
{
	struct aa551_fp *fp;
	int ret;

	fp = kzalloc_obj(*fp);
	if (!fp)
		return -ENOMEM;
	kref_init(&fp->refs);
	mutex_init(&fp->lock);
	INIT_DELAYED_WORK(&fp->timeout, aa551_fp_timeout);
	fp->ctx = ctx;
	fp->brightness = AA551_BRIGHTNESS_DEFAULT;
	fp->misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "aston_fp_illumination",
		.fops = &aa551_fp_fops,
		.parent = &ctx->dsi->dev,
		.mode = 0600,
	};
	ret = misc_register(&fp->misc);
	if (ret) {
		kref_put(&fp->refs, aa551_fp_free);
		return ret;
	}
	fp->registered = true;
	ctx->fp = fp;
	return devm_add_action_or_reset(&ctx->dsi->dev, aa551_fp_destroy, fp);
}

static void aa551_iris_power_on(struct aa551 *ctx)
{
	/* The Iris7P 1.8 V rail is vddio; its 0.9 V rail is switched by GPIO. */
	gpiod_set_value_cansleep(ctx->iris_reset_gpio, 1);
	gpiod_set_value_cansleep(ctx->iris_wakeup_gpio, 0);
	gpiod_set_value_cansleep(ctx->iris_vdd0p9_gpio, 1);
	usleep_range(5000, 5100);
}

static void aa551_iris_power_off(struct aa551 *ctx)
{
	gpiod_set_value_cansleep(ctx->iris_reset_gpio, 1);
	gpiod_set_value_cansleep(ctx->iris_wakeup_gpio, 0);
	gpiod_set_value_cansleep(ctx->iris_vdd0p9_gpio, 0);
}

static int aa551_iris_force_abyp(struct aa551 *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	unsigned long flags;
	int i, ready;

	if (gpiod_cansleep(ctx->iris_wakeup_gpio)) {
		dev_err(dev, "Iris7P wakeup GPIO cannot provide timed pulses\n");
		return -EINVAL;
	}

	/*
	 * As in the vendor driver, the pulse train and the command gap are
	 * sent with interrupts disabled: an interrupt in the middle of the
	 * waveform makes the Iris7P discard the command.
	 */
	local_irq_save(flags);
	for (i = 0; i < IRIS_FORCE_ABYP_PULSES; i++) {
		gpiod_set_value(ctx->iris_wakeup_gpio, 1);
		udelay(IRIS_ONE_WIRE_PULSE_US);
		gpiod_set_value(ctx->iris_wakeup_gpio, 0);
		udelay(IRIS_ONE_WIRE_PULSE_US);
	}
	udelay(IRIS_ONE_WIRE_COMMAND_US);
	local_irq_restore(flags);
	udelay(IRIS_ONE_WIRE_END_US);

	for (i = 0; i < IRIS_ABYP_POLL_COUNT; i++) {
		ready = gpiod_get_value_cansleep(ctx->iris_abyp_ready_gpio);
		if (ready < 0) {
			dev_err(dev, "Failed to read Iris7P ABYP-ready GPIO: %d\n", ready);
			return ready;
		}
		if (ready)
			return 0;
		usleep_range(3000, 3100);
	}

	return -ETIMEDOUT;
}

static void aa551_iris_enter_abyp(struct aa551 *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	int ready, ret;

	gpiod_set_value_cansleep(ctx->iris_wakeup_gpio, 0);
	gpiod_set_value_cansleep(ctx->iris_reset_gpio, 1);
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(ctx->iris_reset_gpio, 0);
	usleep_range(2000, 2100);
	ready = gpiod_get_value_cansleep(ctx->iris_abyp_ready_gpio);
	ret = ready == 1 ? 0 : ready < 0 ? ready : aa551_iris_force_abyp(ctx);
	ctx->iris_confirmed = !ret;
	/* Repeating resets on an unasserted status pin never proved that the
	 * data path failed. Keep the factory bounded wait, then validate the
	 * actual panel response in enable before accepting a working display.
	 */
	if (ret)
		dev_warn(dev, "Iris7P ABYP status unconfirmed (%d); panel link must validate\n", ret);
}

static void aa551_reset(struct aa551 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(25);
}

static int aa551_power_on(struct aa551 *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->powered)
		return 0;
	/* vddio comes first: it also feeds the Iris7P 1.8 V rail. */
	ret = regulator_enable(ctx->vddio);
	if (ret) {
		dev_err(dev, "Failed to enable vddio regulator: %d\n", ret);
		return ret;
	}

	aa551_iris_power_on(ctx);

	ret = regulator_enable(ctx->vci);
	if (ret) {
		dev_err(dev, "Failed to enable vci regulator: %d\n", ret);
		aa551_iris_power_off(ctx);
		regulator_disable(ctx->vddio);
		return ret;
	}

	aa551_iris_enter_abyp(ctx);
	aa551_reset(ctx);
	ctx->powered = true;

	return 0;
}

static void aa551_power_off(struct aa551 *ctx)
{
	if (!ctx->powered)
		return;
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vci);
	aa551_iris_power_off(ctx);
	regulator_disable(ctx->vddio);
	ctx->powered = false;
	ctx->retained = false;
}

static void aa551_init(struct mipi_dsi_multi_context *dsi_ctx)
{
	aa551_switch_page(dsi_ctx, 0x00);
	mipi_dsi_dcs_set_tear_on_multi(dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	/* 1-BIT ESD */
	aa551_switch_page(dsi_ctx, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc6, 0x01);
	aa551_switch_page(dsi_ctx, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xed, 0xff, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xbf, 0xff);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xee, 0xfe, 0xef, 0xc1, 0xe0, 0x00, 0xc0, 0x01, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd2, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd3, 0x01);
	aa551_switch_page(dsi_ctx, 0x02);
	/* FUNCTION SELECT */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf8, 0x01);
	aa551_switch_page(dsi_ctx, 0x20);
	/* TE TEST MODE */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb3, 0x50);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb5, 0x03);
	aa551_switch_page(dsi_ctx, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc8, 0x62);
	aa551_switch_page(dsi_ctx, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x19);
	/* AOD EL INTERNAL */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd0, 0xff, 0xaf, 0x56, 0x3d, 0x2d, 0x2d, 0x2d, 0x2d,
				     0xff);
	aa551_switch_page(dsi_ctx, 0x00);
	aa551_switch_page(dsi_ctx, 0x31);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa0, 0xf3);
	/* CHANGE PPS TABLE */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00,
				     0xab, 0x30, 0x80, 0x0a, 0xdc, 0x04, 0xf0, 0x00, 0x14, 0x02,
				     0x78, 0x02, 0x78, 0x02, 0x00, 0x02, 0x57, 0x00, 0x20, 0x01,
				     0xf8, 0x00, 0x08, 0x00, 0x0d, 0x05, 0x7a, 0x04, 0x4f, 0x18,
				     0x00, 0x10, 0xe0, 0x07, 0x10, 0x20, 0x00, 0x06, 0x0f, 0x0f,
				     0x33, 0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70,
				     0x77, 0x79, 0x7b, 0x7d, 0x7e, 0x02, 0x02, 0x22, 0x00, 0x2a,
				     0x40, 0x2a, 0xbe, 0x3a, 0xfc, 0x3a, 0xfa, 0x3a, 0xf8, 0x3b,
				     0x38, 0x3b, 0x78, 0x3b, 0xb6, 0x4b, 0xb6, 0x4b, 0xf4, 0x4b,
				     0xf4, 0x6c, 0x34, 0x84, 0x74, 0x74, 0x00, 0x00, 0x00, 0x00,
				     0x00);
	/* OSC 138.6MHZ */
	aa551_switch_page(dsi_ctx, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf2, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf5, 0x33);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf6, 0xb7);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf7, 0x98);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xf2, 0x01);
	/* MIPI=556.8MHz_1113.6Mbps OSC */
	aa551_switch_page(dsi_ctx, 0x22);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd0, 0x00, 0xd6, 0x11, 0x10, 0x32, 0x14, 0x14);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd3, 0x00, 0xd6, 0x11, 0x10, 0x32, 0x14, 0x14);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd6, 0x00, 0xd6, 0x11, 0x10, 0x32, 0x14, 0x14);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd9, 0x00, 0xd6, 0x11, 0x10, 0x32, 0x14, 0x14);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xdc, 0x84, 0x94, 0xbb, 0x00, 0x20, 0xc4, 0xa1, 0xcc,
				     0x10);
	/* TRIM_CMD enable */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xdd, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xde, 0xf1);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xdf, 0x80);
	/* P6_cmd key=3C */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xe0, 0x02, 0x00, 0x5f, 0x21, 0x00, 0x3c, 0x28, 0x00);
	/* DCDC Setting */
	aa551_switch_page(dsi_ctx, 0x1f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0xdb, 0x7f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x84, 0x2d, 0x7f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x85, 0x5f, 0x79, 0x07);
	/* ESD fixed H lines */
	aa551_switch_page(dsi_ctx, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd2, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd3, 0x01);
	/* 72HZ--EM_DUTY 93.3 */
	aa551_switch_page(dsi_ctx, 0x24);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x92, 0x8e);
	aa551_switch_page(dsi_ctx, 0x23);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcb, 0x01);
	aa551_switch_page(dsi_ctx, 0x64);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x00, 0x00, 0x4e, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x81, 0x00, 0x00, 0x4e, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x82, 0x00, 0x00, 0x4e, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x00, 0x00, 0x4e, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x84, 0x00, 0x00, 0x4e, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x85, 0x00, 0x00, 0x4e, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x86, 0x00, 0x00, 0xcb, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x87, 0x00, 0x02, 0xd7, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x88, 0x00, 0x03, 0x31, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x89, 0x00, 0x03, 0xb8, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8a, 0x00, 0x04, 0x03, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8b, 0x00, 0x04, 0x36, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8c, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8d, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8e, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8f, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x90, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x91, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x92, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x93, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x94, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x95, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x96, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x97, 0x00, 0x04, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x98, 0x00, 0x04, 0xb0, 0x1e);
	aa551_switch_page(dsi_ctx, 0x23);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcb, 0x00);
	/* HRST */
	aa551_switch_page(dsi_ctx, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc8, 0x62);
	aa551_switch_page(dsi_ctx, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x82, 0x00, 0x00, 0x40, 0x0a, 0x00, 0x11, 0x08, 0x04,
				     0x00, 0x0f, 0x00, 0x00, 0x3e, 0xfc);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x00, 0x00, 0xc0, 0x00, 0x10, 0x18, 0x08, 0x18,
				     0x00, 0x00, 0x00, 0x00, 0x3e, 0xfc);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x94, 0x09);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x98, 0x08);
	aa551_switch_page(dsi_ctx, 0x09);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8e, 0x08, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb4, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xee, 0x40);
	aa551_switch_page(dsi_ctx, 0x0a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x82, 0x08, 0x08, 0x00, 0x40);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x08, 0x18, 0x00, 0xc0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x90, 0x08, 0x08, 0x00, 0x40);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x91, 0x08, 0x18, 0x00, 0xc0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x9e, 0x20, 0x20, 0x00, 0xc0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x9f, 0x20, 0x20, 0x00, 0xc0);
	aa551_switch_page(dsi_ctx, 0x0b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x86, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x87, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x88, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x81, 0x08, 0x08, 0x08, 0x08, 0x18, 0x18, 0x18, 0x18);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x08, 0x08, 0x08, 0x08, 0x18, 0x18, 0x18, 0x18);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x85, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8c, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8d, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8e, 0xbf);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8f, 0x33, 0xbf, 0xbf, 0x33, 0xbf, 0xbf, 0x03, 0xbf);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x90, 0x00, 0x05, 0x05, 0x00, 0x05, 0x05, 0x00, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x91, 0x08, 0x08, 0x08, 0x08, 0x18, 0x18, 0x18, 0x18);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x93, 0x08, 0x00, 0x00, 0x00, 0x10);
	aa551_switch_page(dsi_ctx, 0x0d);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcf, 0x84);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd0, 0x40);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd1, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd2, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd3, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd4, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd5, 0x17);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd6, 0x17);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd7, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd8, 0x00);
	/* min fps */
	aa551_switch_page(dsi_ctx, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa6, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa7, 0x40);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa2, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa0, 0x07, 0x0b, 0x63, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb0, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb1, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb6, 0x80);
	/* TE */
	aa551_switch_page(dsi_ctx, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbc, 0x32);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbd, 0x34);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbe, 0x56);
	aa551_switch_page(dsi_ctx, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x81, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x01);
	aa551_switch_page(dsi_ctx, 0x2d);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x81, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xce, 0x52);
	aa551_switch_page(dsi_ctx, 0x1f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xaa, 0x04);
	/* Vint23 */
	aa551_switch_page(dsi_ctx, 0x4e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd4, 0x00, 0x06, 0x06, 0x06, 0x06, 0x06, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80);
	/* 120HZ */
	aa551_switch_page(dsi_ctx, 0x2d);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd0, 0x01);
	/* VINT2 DELAY */
	aa551_switch_page(dsi_ctx, 0x4e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb1, 0x04);
	/* Peaklum 1023 */
	aa551_switch_page(dsi_ctx, 0x63);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa0, 0x81);
	aa551_switch_page(dsi_ctx, 0x00);
	/* ODC */
	aa551_switch_page(dsi_ctx, 0x49);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x98, 0x08, 0x00, 0x8c, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa0, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
				     0x04, 0x00, 0x04, 0x05, 0x04, 0x04, 0x03, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa4, 0x04, 0x01, 0x01, 0x00, 0x01, 0x00, 0x04, 0x04,
				     0x00, 0x04, 0x04, 0x04, 0x05, 0x04, 0x04, 0x04, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa5, 0x04, 0x04, 0x04, 0x04, 0xfa, 0x0f, 0x10, 0x04,
				     0x04, 0x00, 0x04, 0x09, 0x05, 0x04, 0x03, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa9, 0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x04, 0x04,
				     0x00, 0x03, 0x04, 0x04, 0x05, 0x04, 0x04, 0x04, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xaa, 0x04, 0x04, 0x04, 0x04, 0x08, 0x08, 0x04, 0x04,
				     0x04, 0x00, 0x04, 0x05, 0x04, 0x04, 0x04, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xae, 0x02, 0x01, 0x01, 0x00, 0x01, 0x00, 0x04, 0x04,
				     0x00, 0x03, 0x04, 0x04, 0x05, 0x04, 0x04, 0x04, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb0, 0x01, 0x01, 0x01, 0x01, 0x0b, 0x01, 0x16, 0x00,
				     0x05, 0x03, 0x35, 0x03, 0x15, 0x0f, 0x05, 0x20, 0x10, 0x23);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb1, 0x00, 0x00, 0x00, 0x01, 0x05, 0xfe, 0xfe, 0x05,
				     0x01, 0x01, 0x69, 0xfe, 0x37, 0x30, 0x24, 0x16, 0x08, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb4, 0x03, 0x06, 0x06, 0x06, 0x06, 0x06, 0x14, 0x06,
				     0x05, 0x01, 0x24, 0x02, 0x13, 0x0e, 0x06, 0x20, 0x0f, 0x22);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb5, 0x1d, 0x13, 0x04, 0xff, 0x10, 0x0f, 0x0f, 0x0c,
				     0x02, 0x03, 0x74, 0x3c, 0x38, 0x2f, 0x26, 0x15, 0x0a, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb8, 0x04, 0x04, 0x02, 0x08, 0x02, 0x01, 0x18, 0x07,
				     0x05, 0x03, 0x34, 0x16, 0x13, 0x0f, 0x08, 0x55, 0x28, 0x24);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb9, 0x1d, 0x13, 0x07, 0x04, 0x30, 0x2b, 0x24, 0x05,
				     0x02, 0x01, 0x70, 0x3b, 0x33, 0x2b, 0x1f, 0x10, 0x05, 0x01);
	/* ODC POWER ON */
	aa551_switch_page(dsi_ctx, 0x49);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xdc, 0x03);
	/* VDDI 0.979V */
	aa551_switch_page(dsi_ctx, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x95, 0x66);
	/* PHBM */
	aa551_switch_page(dsi_ctx, 0x1e);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc0, 0x94);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc1, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc4, 0x04, 0x40);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc9, 0x40);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc2, 0xff, 0xff, 0xed, 0x61, 0x00, 0x62);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc5, 0x44, 0x00, 0x00, 0x44, 0x00, 0x00, 0x44, 0x00,
				     0x00, 0x43, 0x07, 0x73, 0x21, 0x75, 0xa7, 0x00, 0xc3, 0x00,
				     0x44, 0x00, 0x00, 0x44, 0x00, 0x00, 0x44, 0x00, 0x00, 0x44,
				     0x00, 0x00, 0x44, 0x00, 0x00, 0x44, 0x00, 0x00, 0x04, 0x00);
	aa551_switch_page(dsi_ctx, 0x13);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd1, 0x88);
	aa551_switch_page(dsi_ctx, 0x17);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa0, 0xcc);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xae, 0x00);
	aa551_switch_page(dsi_ctx, 0x24);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x04);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x81, 0x40);
	aa551_switch_page(dsi_ctx, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x2b);
	aa551_switch_page(dsi_ctx, 0x1f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x08);
	aa551_switch_page(dsi_ctx, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x96, 0x07);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x97, 0xe9);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x98, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x99, 0x78);
	aa551_switch_page(dsi_ctx, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc8, 0x62);
	aa551_switch_page(dsi_ctx, 0x00);
}

/*
 * The vendor's timing-switch command of its 60 Hz timing, which ends with
 * a write-memory-start. It is sent after the initialisation sequence on
 * every power-on, whatever the mode.
 */
static void aa551_timing_switch(struct mipi_dsi_multi_context *dsi_ctx)
{
	aa551_switch_page(dsi_ctx, 0x2d);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xd0, 0x01);
	aa551_switch_page(dsi_ctx, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa6, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb0, 0x01, 0x11, 0x80, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb1, 0xff, 0xff, 0xff, 0x01, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xfe, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc1, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc2, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xc3, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb6, 0x80);
	aa551_switch_page(dsi_ctx, 0x23);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcb, 0x01);
	aa551_switch_page(dsi_ctx, 0x65);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x81, 0x00, 0x00, 0x30, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x82, 0x00, 0x00, 0x30, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x00, 0x00, 0x30, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x84, 0x00, 0x00, 0x30, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x85, 0x00, 0x00, 0x30, 0x03);
	aa551_switch_page(dsi_ctx, 0x23);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcb, 0x00);
	aa551_switch_page(dsi_ctx, 0x23);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xcb, 0x00);
	aa551_switch_page(dsi_ctx, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x81, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x88, 0x78);
	aa551_switch_page(dsi_ctx, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa7, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa2, 0x00);
	aa551_switch_page(dsi_ctx, 0x20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbc, 0x52);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbd, 0x34);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xbe, 0x56);
	aa551_switch_page(dsi_ctx, 0x0b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8c, 0x10);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8d, 0x03);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8e, 0xbf);
	aa551_switch_page(dsi_ctx, 0x4f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x8b, 0x78);
	aa551_switch_page(dsi_ctx, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MIPI_DCS_WRITE_MEMORY_START);
}

static int aa551_off(struct aa551 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/* The vendor's loading-effect-off command. */
	aa551_switch_page(&dsi_ctx, 0x63);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x95, 0xfc, 0xfc, 0xeb, 0xda, 0xc9, 0xb9, 0xa8, 0x97,
				     0x86, 0x75, 0x64, 0x54, 0x43, 0x32, 0x21, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x96, 0xff, 0x88, 0x90, 0x98, 0xa0, 0xff, 0xa8, 0xb0,
				     0xb8, 0xc0, 0xff, 0xc7, 0xcf, 0xd7, 0xdf, 0xff, 0xe7, 0xef,
				     0xf7, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x97, 0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88,
				     0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x01);
	aa551_switch_page(&dsi_ctx, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 120);

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int aa551_prepare(struct drm_panel *panel)
{
	struct aa551 *ctx = to_aa551(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	struct drm_dsc_picture_parameter_set pps;
	int ret;
	u64 start = ktime_get_ns();

	mutex_lock(&ctx->fp->lock);
	if (ctx->retained && ctx->powered) {
		ctx->fp->prepared = true;
		ctx->retained_wakes++;
		ctx->prepare_us = div_u64(ktime_get_ns() - start, 1000);
		mutex_unlock(&ctx->fp->lock);
		return 0;
	}
	ctx->cold_wakes++;
	ret = aa551_power_on(ctx);
	if (ret) {
		mutex_unlock(&ctx->fp->lock);
		return ret;
	}

	aa551_init(&dsi_ctx);
	aa551_timing_switch(&dsi_ctx);

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
	mipi_dsi_compression_mode_multi(&dsi_ctx, true);
	mipi_dsi_msleep(&dsi_ctx, 28);

	if (dsi_ctx.accum_err) {
		dev_err(&ctx->dsi->dev, "Failed to initialize panel: %d\n",
			dsi_ctx.accum_err);
		aa551_power_off(ctx);
		mutex_unlock(&ctx->fp->lock);
		return dsi_ctx.accum_err;
	}

	ctx->fp->prepared = true;
	ctx->prepare_us = div_u64(ktime_get_ns() - start, 1000);
	mutex_unlock(&ctx->fp->lock);
	return 0;
}

static int aa551_enable(struct drm_panel *panel)
{
	struct aa551 *ctx = to_aa551(panel);
	struct mipi_dsi_multi_context dc = { .dsi = ctx->dsi };
	unsigned long flags = ctx->dsi->mode_flags;
	u64 start = ktime_get_ns();
	u8 status = 0;
	int ret;

	mutex_lock(&ctx->fp->lock);
	if (ctx->retained) {
		/* Normal-mode display blanking preserves the validated black level.
		 * The factory LP1/NOLP transition lifts black after retained wake.
		 * No reset, sleep exit or mode transition is needed here.
		 */
		ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	} else {
		mipi_dsi_dcs_exit_sleep_mode_multi(&dc);
		mipi_dsi_msleep(&dc, 120);
	}
	mipi_dsi_dcs_set_display_on_multi(&dc);
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = dc.accum_err ?: mipi_dsi_dcs_get_power_mode(ctx->dsi, &status);
	ctx->dsi->mode_flags = flags;
	/* Check defined DCS power-state bits, not reserved response bits. A
	 * low Iris pin is accepted only after a working panel-link response.
	 */
	if (!ret && !aa551_power_mode_ready(status))
		ret = -EIO;
	ctx->power_mode = status;
	ctx->fp->panel_on = !ret;
	if (!ret)
		ctx->retained = false;
	ctx->enable_us = div_u64(ktime_get_ns() - start, 1000);
	dev_info(&ctx->dsi->dev, "wake prepare=%llu us enable=%llu us status=%02x cold=%u retained=%u result=%d\n",
		 ctx->prepare_us, ctx->enable_us, status, ctx->cold_wakes,
		 ctx->retained_wakes, ret);
	mutex_unlock(&ctx->fp->lock);
	return ret;
}

static int aa551_disable(struct drm_panel *panel)
{
	struct aa551 *ctx = to_aa551(panel);
	struct mipi_dsi_multi_context dc = { .dsi = ctx->dsi };
	unsigned long flags = ctx->dsi->mode_flags;
	int ret;

	mutex_lock(&ctx->fp->lock);
	aa551_fp_off_locked(ctx->fp);
	if (ctx->standby_enabled && !ctx->shutting_down) {
		ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
		/* Keep the normal panel configuration while blanked. Entering
		 * factory LP1 here caused a global brown black level after NOLP,
		 * reproduced by fingerprint scanout and absent after cold wake.
		 * DISPLAY_OFF hides the retained frame without that transition.
		 */
		mipi_dsi_dcs_set_display_off_multi(&dc);
		ctx->dsi->mode_flags = flags;
		ret = dc.accum_err;
		ctx->retained = !ret;
	} else {
		ret = aa551_off(ctx);
		ctx->retained = false;
	}
	if (ret < 0)
		dev_err(&ctx->dsi->dev, "Failed to disable panel: %d\n", ret);
	ctx->fp->panel_on = false;
	mutex_unlock(&ctx->fp->lock);
	return ret;
}

static int aa551_unprepare(struct drm_panel *panel)
{
	struct aa551 *ctx = to_aa551(panel);
	int ret;

	/*
	 * The power-down commands go out a second time before the supplies
	 * are cut, as in the sequence validated on the device.
	 */
	mutex_lock(&ctx->fp->lock);
	aa551_fp_off_locked(ctx->fp);
	if (ctx->retained && !ctx->shutting_down) {
		ctx->fp->prepared = false;
		mutex_unlock(&ctx->fp->lock);
		return 0;
	}
	ret = aa551_off(ctx);
	if (ret < 0)
		dev_err(&ctx->dsi->dev, "Failed to un-initialize panel: %d\n", ret);

	aa551_power_off(ctx);
	ctx->fp->prepared = false;
	ctx->fp->panel_on = false;
	ctx->fp->active = false;
	ctx->fp->gamma_programmed = false;
	cancel_delayed_work(&ctx->fp->timeout);
	mutex_unlock(&ctx->fp->lock);
	return 0;
}

/*
 * One horizontal timing for every rate. 165 Hz raises the pixel clock; the
 * rates below 120 Hz keep about the 120 Hz clock and stretch the vertical
 * front porch.
 */
static const struct drm_display_mode aa551_modes[] = {
	{
		.clock = (1264 + 26 + 2 + 26) * (2780 + 22 + 2 + 42) * 165 / 1000,
		.hdisplay = 1264,
		.hsync_start = 1264 + 26,
		.hsync_end = 1264 + 26 + 2,
		.htotal = 1264 + 26 + 2 + 26,
		.vdisplay = 2780,
		.vsync_start = 2780 + 22,
		.vsync_end = 2780 + 22 + 2,
		.vtotal = 2780 + 22 + 2 + 42,
		.width_mm = 71,
		.height_mm = 157,
	},
	{
		.clock = (1264 + 26 + 2 + 26) * (2780 + 22 + 2 + 42) * 120 / 1000,
		.hdisplay = 1264,
		.hsync_start = 1264 + 26,
		.hsync_end = 1264 + 26 + 2,
		.htotal = 1264 + 26 + 2 + 26,
		.vdisplay = 2780,
		.vsync_start = 2780 + 22,
		.vsync_end = 2780 + 22 + 2,
		.vtotal = 2780 + 22 + 2 + 42,
		.width_mm = 71,
		.height_mm = 157,
	},
	{
		.clock = (1264 + 26 + 2 + 26) * (2780 + 971 + 2 + 42) * 90 / 1000,
		.hdisplay = 1264,
		.hsync_start = 1264 + 26,
		.hsync_end = 1264 + 26 + 2,
		.htotal = 1264 + 26 + 2 + 26,
		.vdisplay = 2780,
		.vsync_start = 2780 + 971,
		.vsync_end = 2780 + 971 + 2,
		.vtotal = 2780 + 971 + 2 + 42,
		.width_mm = 71,
		.height_mm = 157,
	},
	{
		.clock = (1264 + 26 + 2 + 26) * (2780 + 2868 + 2 + 42) * 60 / 1000,
		.hdisplay = 1264,
		.hsync_start = 1264 + 26,
		.hsync_end = 1264 + 26 + 2,
		.htotal = 1264 + 26 + 2 + 26,
		.vdisplay = 2780,
		.vsync_start = 2780 + 2868,
		.vsync_end = 2780 + 2868 + 2,
		.vtotal = 2780 + 2868 + 2 + 42,
		.width_mm = 71,
		.height_mm = 157,
	},
	{
		.clock = (1264 + 26 + 2 + 26) * (2780 + 8560 + 2 + 42) * 30 / 1000,
		.hdisplay = 1264,
		.hsync_start = 1264 + 26,
		.hsync_end = 1264 + 26 + 2,
		.htotal = 1264 + 26 + 2 + 26,
		.vdisplay = 2780,
		.vsync_start = 2780 + 8560,
		.vsync_end = 2780 + 8560 + 2,
		.vtotal = 2780 + 8560 + 2 + 42,
		.width_mm = 71,
		.height_mm = 157,
	},
};

/* Index of the 120 Hz mode in aa551_modes */
#define AA551_PREFERRED_MODE	1

static int aa551_get_modes(struct drm_panel *panel,
			   struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(aa551_modes); i++) {
		mode = drm_mode_duplicate(connector->dev, &aa551_modes[i]);
		if (!mode)
			return -ENOMEM;

		drm_mode_set_name(mode);

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == AA551_PREFERRED_MODE)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = aa551_modes[0].width_mm;
	connector->display_info.height_mm = aa551_modes[0].height_mm;

	return ARRAY_SIZE(aa551_modes);
}

static const struct drm_panel_funcs aa551_panel_funcs = {
	.prepare = aa551_prepare,
	.unprepare = aa551_unprepare,
	.enable = aa551_enable,
	.disable = aa551_disable,
	.get_modes = aa551_get_modes,
};

static int aa551_bl_write_locked(struct aa551 *ctx, u16 brightness)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/*
	 * A brightness set while the panel is off is only stored:
	 * drm_panel_enable() writes it when it enables the backlight again.
	 */
	if (!READ_ONCE(ctx->panel.prepared))
		return 0;

	/* The Vint2/Vint3 setting ("Vint23") depends on the brightness. */
	aa551_switch_page(&dsi_ctx, 0x4e);
	if (brightness >= 1088 && brightness <= 1768) {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					     0x00, 0x80);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					     0x80);
	} else {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd4, 0x00, 0x06, 0x06, 0x06, 0x06, 0x06,
					     0x00, 0x80);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd1, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01,
					     0x80);
	}
	aa551_switch_page(&dsi_ctx, 0x00);

	mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
					 brightness >> 8, brightness & 0xff);

	aa551_switch_page(&dsi_ctx, 0x1e);
	/* Factory grayscale follows ordinary brightness changes even outside
	 * LHBM. Do not change the global grayscale table at fingerprint entry
	 * or restore a different table at exit. The PWM mode remains normal.
	 */
	if (brightness) {
		/* Factory normal-PWM local-HBM grayscale by DBV range. */
		static const u8 grayscale[][7] = {
			{ 0xc2, 0xfe, 0xfe, 0xdd, 0x61, 0x00, 0x62 },
			{ 0xc2, 0xf9, 0xf9, 0xda, 0x61, 0x00, 0x62 },
			{ 0xc2, 0xf5, 0xf5, 0xd6, 0x61, 0x00, 0x62 },
			{ 0xc2, 0xef, 0xef, 0xda, 0x61, 0x00, 0x62 },
			{ 0xc2, 0xe9, 0xe9, 0xd4, 0x61, 0x00, 0x62 },
		};
		unsigned int index = brightness <= 0x700 ? 0 : brightness <= 0x8f0 ? 1 :
				     brightness <= 0xa88 ? 2 : brightness <= 0xc00 ? 3 : 4;

		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, grayscale[index], 7);
	} else {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0xff, 0xff, 0xe5, 0x61, 0x00, 0x62);
	}
	aa551_switch_page(&dsi_ctx, 0x00);

	return dsi_ctx.accum_err;
}

static int aa551_bl_update_status(struct backlight_device *bl)
{
	struct aa551 *ctx = bl_get_data(bl);
	struct aa551_fp *fp = ctx->fp;
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	mutex_lock(&fp->lock);
	fp->brightness = brightness;
	if (!brightness)
		aa551_fp_off_locked(fp);
	ret = aa551_bl_write_locked(ctx, fp->active ?
		min_t(u16, brightness, AA551_FP_MAX_DBV) : brightness);
	if (!ret && fp->active)
		ret = aa551_fp_alpha_locked(fp);
	mutex_unlock(&fp->lock);
	return ret;
}

static const struct backlight_ops aa551_bl_ops = {
	.update_status = aa551_bl_update_status,
};

static struct backlight_device *aa551_create_backlight(struct aa551 *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = AA551_BRIGHTNESS_DEFAULT,
		.max_brightness = AA551_BRIGHTNESS_MAX,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
					      &aa551_bl_ops, &props);
}

static int aa551_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct aa551 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct aa551, panel, &aa551_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio),
				     "Failed to get vddio regulator\n");

	ctx->vci = devm_regulator_get(dev, "vci");
	if (IS_ERR(ctx->vci))
		return dev_err_probe(dev, PTR_ERR(ctx->vci),
				     "Failed to get vci regulator\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->iris_vdd0p9_gpio = devm_gpiod_get(dev, "iris-vdd0p9", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->iris_vdd0p9_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->iris_vdd0p9_gpio),
				     "Failed to get iris-vdd0p9-gpios\n");

	ctx->iris_reset_gpio = devm_gpiod_get(dev, "iris-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->iris_reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->iris_reset_gpio),
				     "Failed to get iris-reset-gpios\n");

	ctx->iris_wakeup_gpio = devm_gpiod_get(dev, "iris-wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->iris_wakeup_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->iris_wakeup_gpio),
				     "Failed to get iris-wakeup-gpios\n");

	ctx->iris_abyp_ready_gpio = devm_gpiod_get(dev, "iris-abyp-ready", GPIOD_IN);
	if (IS_ERR(ctx->iris_abyp_ready_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->iris_abyp_ready_gpio),
				     "Failed to get iris-abyp-ready-gpios\n");

	ctx->dsi = dsi;
	ctx->standby_enabled = true;
	mipi_dsi_set_drvdata(dsi, ctx);
	ret = aa551_fp_init(ctx);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register fingerprint illumination\n");

	/*
	 * Without MIPI_DSI_MODE_VIDEO the host runs the link in command mode,
	 * which is the panel's mode; the burst flag only matters in video mode.
	 */
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB101010;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = aa551_create_backlight(ctx);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	/* The panel only takes DSC: two 632x20 slices per line. */
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 2;
	ctx->dsc.slice_height = 20;
	ctx->dsc.slice_width = 632;
	ctx->dsc.slice_count = 1264 / ctx->dsc.slice_width;
	ctx->dsc.convert_rgb = true;
	ctx->dsc.bits_per_component = 10;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	dsi->dsc = &ctx->dsc;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void aa551_shutdown(struct mipi_dsi_device *dsi);

static void aa551_remove(struct mipi_dsi_device *dsi)
{
	struct aa551 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	aa551_shutdown(dsi);
	aa551_fp_stop(ctx->fp);
	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static void aa551_shutdown(struct mipi_dsi_device *dsi)
{
	struct aa551 *ctx = mipi_dsi_get_drvdata(dsi);

	mutex_lock(&ctx->fp->lock);
	ctx->shutting_down = true;
	mutex_unlock(&ctx->fp->lock);
	drm_panel_disable(&ctx->panel);
	drm_panel_unprepare(&ctx->panel);
	mutex_lock(&ctx->fp->lock);
	aa551_power_off(ctx);
	mutex_unlock(&ctx->fp->lock);
}

static ssize_t standby_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct aa551 *ctx = mipi_dsi_get_drvdata(to_mipi_dsi_device(dev));

	return sysfs_emit(buf, "%u\n", READ_ONCE(ctx->standby_enabled));
}

static ssize_t standby_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct aa551 *ctx = mipi_dsi_get_drvdata(to_mipi_dsi_device(dev));
	bool enabled;
	int ret = kstrtobool(buf, &enabled);

	if (ret)
		return ret;
	mutex_lock(&ctx->fp->lock);
	if (ctx->retained || ctx->shutting_down)
		ret = -EBUSY;
	else
		ctx->standby_enabled = enabled;
	mutex_unlock(&ctx->fp->lock);
	return ret ?: count;
}
static DEVICE_ATTR_RW(standby_enabled);

static ssize_t standby_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct aa551 *ctx = mipi_dsi_get_drvdata(to_mipi_dsi_device(dev));
	ssize_t count;

	mutex_lock(&ctx->fp->lock);
	count = sysfs_emit(buf, "powered=%u retained=%u panel_on=%u iris_confirmed=%u power_mode=%02x cold_wakes=%u retained_wakes=%u prepare_us=%llu enable_us=%llu\n",
		ctx->powered, ctx->retained, ctx->fp->panel_on,
		ctx->iris_confirmed, ctx->power_mode, ctx->cold_wakes,
		ctx->retained_wakes, ctx->prepare_us, ctx->enable_us);
	mutex_unlock(&ctx->fp->lock);
	return count;
}
static DEVICE_ATTR_RO(standby_state);

static struct attribute *aa551_attrs[] = {
	&dev_attr_standby_enabled.attr,
	&dev_attr_standby_state.attr,
	NULL,
};
ATTRIBUTE_GROUPS(aa551);

static const struct of_device_id aa551_of_match[] = {
	{ .compatible = "oneplus,aston-boe-aa551" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, aa551_of_match);

static struct mipi_dsi_driver aa551_driver = {
	.probe = aa551_probe,
	.remove = aa551_remove,
	.shutdown = aa551_shutdown,
	.driver = {
		.name = "panel-oneplus-aston-aa551",
		.of_match_table = aa551_of_match,
		.dev_groups = aa551_groups,
	},
};
module_mipi_dsi_driver(aa551_driver);

MODULE_AUTHOR("Doctor Logix <doctor.logix@gmail.com>");
MODULE_DESCRIPTION("DRM driver for the BOE AA551 AMOLED panel of the OnePlus 12R and Ace 3");
MODULE_LICENSE("GPL");
