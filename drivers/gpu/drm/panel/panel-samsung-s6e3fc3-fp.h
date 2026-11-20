/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * /dev/fp_illumination for the S6E3FC3: lights the circle above the Pixel
 * 6's optical fingerprint sensor with the panel's local high brightness
 * mode (LHBM), as Google's driver does. Before the first enable the factory
 * LHBM gamma is read from D8 and copied to 65; each enable then sets the
 * circle at (540, 1769) and that gamma, puts the panel at 90 Hz (Google's
 * driver lights it at 90 Hz only) and sets the LHBM bit of WRCTRLD. Disable
 * clears the bit and gives back the mode's rate. fp->lock also serialises
 * the backlight's WRCTRLD writes, so that they keep the bit while lit.
 */

/* WRCTRLD: local high brightness mode. */
#define S6E3FC3_WRCTRLD_LHBM		BIT(4)

/* Google's driver: local_hbm_max_timeout. */
#define S6E3FC3_FP_TIMEOUT_MS		3000

#define S6E3FC3_FP_GAMMA_LEN		5

struct s6e3fc3_fp {
	struct kref refs;
	/* Serialises the state below and every WRCTRLD write. */
	struct mutex lock;
	struct miscdevice misc;
	struct delayed_work timeout;
	struct s6e3fc3 *ctx;
	/* 0x65 and the factory gamma read from D8. */
	u8 gamma[1 + S6E3FC3_FP_GAMMA_LEN];
	bool gamma_ready;
	bool opened;
	bool panel_on;
	bool active;
	bool registered;
};

static u8 s6e3fc3_fp_wrctrld(struct s6e3fc3_fp *fp)
{
	return S6E3FC3_WRCTRLD_BCTRL | (fp->active ? S6E3FC3_WRCTRLD_LHBM : 0);
}

/* Google's s6e3fc3_lhbm_gamma_read(): D8 at 0x22, in 10-bit mode. */
static int s6e3fc3_fp_read_gamma(struct s6e3fc3_fp *fp)
{
	struct mipi_dsi_device *dsi = fp->ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	int ret;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x28, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0xcc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x22, 0xd8);
	ret = dsi_ctx.accum_err;
	if (!ret)
		ret = mipi_dsi_set_maximum_return_packet_size(dsi,
							      S6E3FC3_FP_GAMMA_LEN);
	if (!ret)
		ret = mipi_dsi_dcs_read(dsi, 0xd8, fp->gamma + 1,
					S6E3FC3_FP_GAMMA_LEN);
	/* Back to 8-bit mode and locked, whatever the read did. */
	dsi_ctx.accum_err = 0;
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x28, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);

	if (ret < 0)
		return ret;
	if (ret != S6E3FC3_FP_GAMMA_LEN)
		return -EIO;
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	fp->gamma[0] = 0x65;
	fp->gamma_ready = true;
	return 0;
}

static int s6e3fc3_fp_off_locked(struct s6e3fc3_fp *fp)
{
	struct s6e3fc3 *ctx = fp->ctx;
	struct mipi_dsi_multi_context dsi_ctx;

	cancel_delayed_work(&fp->timeout);
	if (!ctx || !fp->panel_on) {
		fp->active = false;
		return 0;
	}
	if (!fp->active)
		return 0;

	dsi_ctx = (struct mipi_dsi_multi_context) { .dsi = ctx->dsi };
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     S6E3FC3_WRCTRLD_BCTRL);
	if (ctx->freq != s6e3fc3_modes[1].freq)
		s6e3fc3_set_freq(&dsi_ctx, ctx->freq);
	if (dsi_ctx.accum_err) {
		/* Retain ownership and retry if hardware state is unconfirmed. */
		mod_delayed_work(system_dfl_wq, &fp->timeout, msecs_to_jiffies(1000));
		return dsi_ctx.accum_err;
	}
	fp->active = false;

	return 0;
}

static int s6e3fc3_fp_on_locked(struct s6e3fc3_fp *fp)
{
	struct s6e3fc3 *ctx = fp->ctx;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	int ret;

	if (!fp->panel_on || !backlight_get_brightness(ctx->panel.backlight))
		return -EHOSTDOWN;
	if (fp->active)
		return 0; /* Repeated enable cannot extend the independent deadline. */

	if (!fp->gamma_ready) {
		ret = s6e3fc3_fp_read_gamma(fp);
		if (ret) {
			dev_err(&ctx->dsi->dev,
				"failed to read the LHBM gamma: %d\n", ret);
			return ret;
		}
	}

	fp->active = true;
	/* Google's init: the circle's centre as two 12-bit values. */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf1, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x28, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0xcc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x01, 0x34, 0x68);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x68, 0x21, 0xc6, 0xe9);
	/* Google's s6e3fc3_lhbm_gamma_write(). */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03, 0xcd, 0x65);
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, fp->gamma, sizeof(fp->gamma));
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x28, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf1, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	if (ctx->freq != s6e3fc3_modes[1].freq)
		s6e3fc3_set_freq(&dsi_ctx, s6e3fc3_modes[1].freq);
	mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
					 s6e3fc3_fp_wrctrld(fp));
	if (dsi_ctx.accum_err) {
		s6e3fc3_fp_off_locked(fp);
		return dsi_ctx.accum_err;
	}

	mod_delayed_work(system_dfl_wq, &fp->timeout,
			 msecs_to_jiffies(S6E3FC3_FP_TIMEOUT_MS));
	return 0;
}

static void s6e3fc3_fp_timeout(struct work_struct *work)
{
	struct s6e3fc3_fp *fp = container_of(to_delayed_work(work),
					     struct s6e3fc3_fp, timeout);

	mutex_lock(&fp->lock);
	if (s6e3fc3_fp_off_locked(fp) && fp->ctx)
		dev_err(&fp->ctx->dsi->dev,
			"failed to restore fingerprint illumination\n");
	mutex_unlock(&fp->lock);
}

/* Panel: on after display on, off before display off. */
static void s6e3fc3_fp_panel(struct s6e3fc3_fp *fp, bool on)
{
	mutex_lock(&fp->lock);
	if (!on) {
		s6e3fc3_fp_off_locked(fp);
		/* Display off hides what a failed off left lit. */
		cancel_delayed_work(&fp->timeout);
		fp->active = false;
	}
	fp->panel_on = on;
	mutex_unlock(&fp->lock);
}

static void s6e3fc3_fp_free(struct kref *refs)
{
	struct s6e3fc3_fp *fp = container_of(refs, struct s6e3fc3_fp, refs);

	mutex_destroy(&fp->lock);
	kfree(fp);
}

static int s6e3fc3_fp_open(struct inode *inode, struct file *file)
{
	struct s6e3fc3_fp *fp = container_of(file->private_data,
					     struct s6e3fc3_fp, misc);
	int ret = 0;

	mutex_lock(&fp->lock);
	if (!fp->ctx) {
		ret = -ENODEV;
	} else if (fp->opened || fp->active) {
		ret = -EBUSY;
	} else {
		fp->opened = true;
		kref_get(&fp->refs);
		file->private_data = fp;
	}
	mutex_unlock(&fp->lock);

	return ret ?: nonseekable_open(inode, file);
}

static int s6e3fc3_fp_release(struct inode *inode, struct file *file)
{
	struct s6e3fc3_fp *fp = file->private_data;

	mutex_lock(&fp->lock);
	s6e3fc3_fp_off_locked(fp);
	fp->opened = false;
	mutex_unlock(&fp->lock);
	kref_put(&fp->refs, s6e3fc3_fp_free);

	return 0;
}

static long s6e3fc3_fp_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	struct s6e3fc3_fp *fp = file->private_data;
	struct fp_illumination_info info = {
		.version = 1,
		.panel_width = 1080,
		.panel_height = 2400,
		.max_duration_ms = S6E3FC3_FP_TIMEOUT_MS,
		.compatible = "samsung,s6e3fc3",
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
			ret = on ? s6e3fc3_fp_on_locked(fp) :
				   s6e3fc3_fp_off_locked(fp);
		break;
	default:
		ret = -ENOTTY;
	}
out:
	mutex_unlock(&fp->lock);

	return ret;
}

static const struct file_operations s6e3fc3_fp_fops = {
	.owner = THIS_MODULE,
	.open = s6e3fc3_fp_open,
	.release = s6e3fc3_fp_release,
	.unlocked_ioctl = s6e3fc3_fp_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static void s6e3fc3_fp_destroy(void *data)
{
	struct s6e3fc3_fp *fp = data;

	mutex_lock(&fp->lock);
	s6e3fc3_fp_off_locked(fp);
	fp->ctx = NULL;
	mutex_unlock(&fp->lock);
	if (fp->registered)
		misc_deregister(&fp->misc);
	cancel_delayed_work_sync(&fp->timeout);
	kref_put(&fp->refs, s6e3fc3_fp_free);
}

static int s6e3fc3_fp_init(struct s6e3fc3 *ctx)
{
	struct s6e3fc3_fp *fp;
	int ret;

	fp = kzalloc_obj(*fp);
	if (!fp)
		return -ENOMEM;

	kref_init(&fp->refs);
	mutex_init(&fp->lock);
	INIT_DELAYED_WORK(&fp->timeout, s6e3fc3_fp_timeout);
	fp->ctx = ctx;
	fp->misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "fp_illumination",
		.fops = &s6e3fc3_fp_fops,
		.parent = &ctx->dsi->dev,
		.mode = 0600,
	};

	ret = misc_register(&fp->misc);
	if (ret) {
		kref_put(&fp->refs, s6e3fc3_fp_free);
		return ret;
	}
	fp->registered = true;
	ctx->fp = fp;

	return devm_add_action_or_reset(&ctx->dsi->dev, s6e3fc3_fp_destroy, fp);
}
