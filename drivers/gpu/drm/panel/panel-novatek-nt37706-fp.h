/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * /dev/fp_illumination for the NT37706 v3: lights the optical fingerprint
 * sensor's region with the vendor's local-HBM commands. The on batch, its
 * brightness-band register (DF 15 below DBV 3520, DF 05 from there) and the
 * DBV alpha written to B8 follow the vendor's hbm-fod-on handling; off is its
 * hbm-off batch. Both are sent in high-speed mode and followed by the normal
 * brightness, as the vendor does. fp->lock also serialises backlight writes,
 * so no brightness write lands between a 6F offset and its register.
 */

#include "panel-novatek-nt37706-alpha.h"

#define NT37706_FP_TIMEOUT_MS	30000
#define NT37706_FP_BAND_DBV	3520

struct nt37706_fp {
	struct kref refs;
	struct mutex lock;
	struct miscdevice misc;
	struct delayed_work timeout;
	struct novatek_nt37706 *ctx;
	bool opened;
	bool panel_on;
	bool active;
	bool registered;
};

static u16 nt37706_fp_level(struct nt37706_fp *fp)
{
	return backlight_get_brightness(fp->ctx->panel.backlight);
}

static int nt37706_fp_off_locked(struct nt37706_fp *fp)
{
	struct novatek_nt37706 *ctx = fp->ctx;
	struct mipi_dsi_multi_context dc;

	cancel_delayed_work(&fp->timeout);
	if (!ctx || !fp->panel_on) {
		fp->active = false;
		return 0;
	}
	if (!fp->active)
		return 0;
	dc = (struct mipi_dsi_multi_context) { .dsi = ctx->dsi };
	mipi_dsi_dcs_write_seq_multi(&dc, 0xa9, 0x02, 0x00, 0xc0, 0x2e, 0x2f, 0x62,
				     0x30, 0x02, 0x00, 0xc0, 0x30, 0x33, 0x07, 0x54,
				     0x00, 0x99, 0x02, 0x00, 0xc7, 0x01, 0x01, 0x01,
				     0x01, 0x00, 0x5f, 0x00, 0x01, 0x00, 0x00, 0x01,
				     0x00, 0x8b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x87,
				     0x00, 0x00, 0x20);
	if (dc.accum_err) {
		/* Retain ownership and retry if hardware state is unconfirmed. */
		mod_delayed_work(system_wq, &fp->timeout, msecs_to_jiffies(1000));
		return dc.accum_err;
	}
	fp->active = false;
	return mipi_dsi_dcs_set_display_brightness_large(ctx->dsi, nt37706_fp_level(fp));
}

static int nt37706_fp_on_locked(struct nt37706_fp *fp)
{
	struct novatek_nt37706 *ctx = fp->ctx;
	struct mipi_dsi_multi_context dc = { .dsi = ctx->dsi };
	u16 level = nt37706_fp_level(fp);
	u16 alpha = nt37706_fp_alpha[min_t(u16, level, ARRAY_SIZE(nt37706_fp_alpha) - 1)];
	int ret;

	if (!fp->panel_on || !level)
		return -EHOSTDOWN;
	if (fp->active)
		return 0; /* Repeated enable cannot extend the independent deadline. */
	fp->active = true;
	mipi_dsi_dcs_write_seq_multi(&dc, 0xa9, 0x02, 0x00, 0xc0, 0x2e, 0x2f, 0x62,
				     0x30, 0x02, 0x00, 0xc0, 0x30, 0x33, 0x08, 0x88,
				     0x00, 0x99, 0x02, 0x00, 0xc7, 0x01, 0x01, 0x08,
				     0x01, 0x00, 0x5f, 0x00, 0x01, 0x00, 0x40, 0x01,
				     0x00, 0x8b, 0x00, 0x00, 0x10, 0x01, 0x00, 0x87,
				     0x00, 0x02, 0x25, 0x3f, 0xfc);
	nt37706_switch_page(&dc, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dc, 0x6f, 0x32);
	mipi_dsi_dcs_write_var_seq_multi(&dc, 0xdf,
					 level < NT37706_FP_BAND_DBV ? 0x15 : 0x05);
	nt37706_switch_page(&dc, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dc, 0x6f, 0x4a);
	mipi_dsi_dcs_write_var_seq_multi(&dc, 0xb8, alpha >> 8, alpha & 0xff);
	ret = dc.accum_err ?: mipi_dsi_dcs_set_display_brightness_large(ctx->dsi, level);
	if (ret) {
		nt37706_fp_off_locked(fp);
		return ret;
	}
	mod_delayed_work(system_wq, &fp->timeout, msecs_to_jiffies(NT37706_FP_TIMEOUT_MS));
	return 0;
}

static void nt37706_fp_timeout(struct work_struct *work)
{
	struct nt37706_fp *fp = container_of(to_delayed_work(work), struct nt37706_fp, timeout);

	mutex_lock(&fp->lock);
	if (nt37706_fp_off_locked(fp) && fp->ctx)
		dev_err(&fp->ctx->dsi->dev, "Failed to restore fingerprint illumination\n");
	mutex_unlock(&fp->lock);
}

/* Panel power: on after the init sequence, off before the panel is shut. */
static void nt37706_fp_panel(struct nt37706_fp *fp, bool on)
{
	mutex_lock(&fp->lock);
	if (!on) {
		nt37706_fp_off_locked(fp);
		/* The power cycle clears what a failed off left lit. */
		cancel_delayed_work(&fp->timeout);
		fp->active = false;
	}
	fp->panel_on = on;
	mutex_unlock(&fp->lock);
}

static void nt37706_fp_free(struct kref *refs)
{
	struct nt37706_fp *fp = container_of(refs, struct nt37706_fp, refs);

	mutex_destroy(&fp->lock);
	kfree(fp);
}

static int nt37706_fp_open(struct inode *inode, struct file *file)
{
	struct nt37706_fp *fp = container_of(file->private_data, struct nt37706_fp, misc);
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

static int nt37706_fp_release(struct inode *inode, struct file *file)
{
	struct nt37706_fp *fp = file->private_data;

	mutex_lock(&fp->lock);
	nt37706_fp_off_locked(fp);
	fp->opened = false;
	mutex_unlock(&fp->lock);
	kref_put(&fp->refs, nt37706_fp_free);
	return 0;
}

static long nt37706_fp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct nt37706_fp *fp = file->private_data;
	struct fp_illumination_info info = {
		.version = 1,
		.panel_width = 1220,
		.panel_height = 2712,
		.max_duration_ms = NT37706_FP_TIMEOUT_MS,
		.compatible = "csot,nt37706-v3",
	};
	u32 on;
	int ret = 0;

	mutex_lock(&fp->lock);
	if (!fp->ctx) {
		ret = -ENODEV;
		goto out;
	}
	switch (cmd) {
	case FP_ILLUMINATION_GET_INFO:
		info.enabled = fp->active;
		info.panel_on = fp->panel_on;
		if (copy_to_user((void __user *)arg, &info, sizeof(info)))
			ret = -EFAULT;
		break;
	case FP_ILLUMINATION_SET:
		if (copy_from_user(&on, (void __user *)arg, sizeof(on)))
			ret = -EFAULT;
		else if (on > 1)
			ret = -EINVAL;
		else
			ret = on ? nt37706_fp_on_locked(fp) : nt37706_fp_off_locked(fp);
		break;
	default:
		ret = -ENOTTY;
	}
out:
	mutex_unlock(&fp->lock);
	return ret;
}

static const struct file_operations nt37706_fp_fops = {
	.owner = THIS_MODULE,
	.open = nt37706_fp_open,
	.release = nt37706_fp_release,
	.unlocked_ioctl = nt37706_fp_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static void nt37706_fp_destroy(void *data)
{
	struct nt37706_fp *fp = data;

	mutex_lock(&fp->lock);
	nt37706_fp_off_locked(fp);
	fp->ctx = NULL;
	mutex_unlock(&fp->lock);
	if (fp->registered)
		misc_deregister(&fp->misc);
	cancel_delayed_work_sync(&fp->timeout);
	kref_put(&fp->refs, nt37706_fp_free);
}

static int nt37706_fp_init(struct novatek_nt37706 *ctx)
{
	struct nt37706_fp *fp;
	int ret;

	fp = kzalloc_obj(*fp);
	if (!fp)
		return -ENOMEM;
	kref_init(&fp->refs);
	mutex_init(&fp->lock);
	INIT_DELAYED_WORK(&fp->timeout, nt37706_fp_timeout);
	fp->ctx = ctx;
	fp->misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "fp_illumination",
		.fops = &nt37706_fp_fops,
		.parent = &ctx->dsi->dev,
		.mode = 0600,
	};
	ret = misc_register(&fp->misc);
	if (ret) {
		kref_put(&fp->refs, nt37706_fp_free);
		return ret;
	}
	fp->registered = true;
	ctx->fp = fp;
	return devm_add_action_or_reset(&ctx->dsi->dev, nt37706_fp_destroy, fp);
}
