// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36532 DSI panels driven over two DSI links
 *
 * The panel takes one stripe of the frame on each link. Commands go to
 * both links at once; the host takes care of that.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct nt36532_desc {
	const struct drm_display_mode *mode;
	void (*init)(struct mipi_dsi_multi_context *ctx);
};

struct nt36532 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct nt36532_desc *desc;
	struct regulator *vddi;
	struct regulator *avdd;
	struct regulator *avee;
	struct gpio_desc *reset_gpio;
};

static inline struct nt36532 *to_nt36532(struct drm_panel *panel)
{
	return container_of(panel, struct nt36532, panel);
}

/* Registers from 0xb0 up are manufacturer commands: generic writes. */
#define nt36532_write(ctx, cmd, seq...)					\
	do {								\
		if ((cmd) < 0xb0)					\
			mipi_dsi_dcs_write_seq_multi(ctx, cmd, seq);	\
		else							\
			mipi_dsi_generic_write_seq_multi(ctx, cmd, seq); \
	} while (0)

#define nt36532_page(ctx, page)						\
	do {								\
		nt36532_write(ctx, 0xff, page);				\
		nt36532_write(ctx, 0xfb, 0x01);				\
	} while (0)

static void tb373fu_tianma_init(struct mipi_dsi_multi_context *ctx)
{
	nt36532_page(ctx, 0x24);
	nt36532_write(ctx, 0x93, 0x1a, 0x00);
	nt36532_write(ctx, 0x94, 0xb8, 0x00);
	nt36532_write(ctx, 0xda, 0x06);
	nt36532_write(ctx, 0x5b, 0x53);
	nt36532_write(ctx, 0xdb, 0x78);
	nt36532_write(ctx, 0xde, 0x09);

	nt36532_page(ctx, 0x20);
	nt36532_write(ctx, 0x58, 0x43);
	nt36532_write(ctx, 0x01, 0x55);
	nt36532_write(ctx, 0x02, 0x55);
	nt36532_write(ctx, 0x08, 0x2d);
	nt36532_write(ctx, 0x65, 0xff);
	nt36532_write(ctx, 0x6d, 0xff);

	nt36532_page(ctx, 0x27);
	nt36532_write(ctx, 0x13, 0x06);
	nt36532_write(ctx, 0xd0, 0x71);
	nt36532_write(ctx, 0xd1, 0x84);
	nt36532_write(ctx, 0xd2, 0x38);
	nt36532_write(ctx, 0xde, 0x43);
	nt36532_write(ctx, 0xdf, 0x02);

	nt36532_page(ctx, 0x20);
	nt36532_write(ctx, 0x0d, 0x43);
	nt36532_write(ctx, 0x69, 0xdd);
	nt36532_write(ctx, 0xff, 0x2a);
	nt36532_write(ctx, 0xb1, 0x00, 0x01);

	nt36532_page(ctx, 0x2a);
	nt36532_write(ctx, 0x1a, 0x1d);

	nt36532_page(ctx, 0x25);
	nt36532_write(ctx, 0x0f, 0x20);
	nt36532_write(ctx, 0x20, 0x53);
	nt36532_write(ctx, 0x27, 0x53);
	nt36532_write(ctx, 0x49, 0x53);
	nt36532_write(ctx, 0x62, 0x53);

	nt36532_page(ctx, 0x26);
	nt36532_write(ctx, 0x52, 0x52);
	nt36532_write(ctx, 0x58, 0x52);
	nt36532_write(ctx, 0x5c, 0x52);
	nt36532_write(ctx, 0x65, 0x52);
	nt36532_write(ctx, 0x73, 0x52);
	nt36532_write(ctx, 0x84, 0x1c, 0x1c, 0x1c);

	nt36532_page(ctx, 0x10);
	nt36532_write(ctx, 0x35, 0x00);
	nt36532_write(ctx, 0x3b, 0x03, 0xb8, 0x1a, 0x0a, 0x0a, 0x00);
	nt36532_write(ctx, 0x9d, 0x01);
	nt36532_write(ctx, 0xb2, 0x80);
	nt36532_write(ctx, 0xb3, 0x00);
}

static int nt36532_prepare(struct drm_panel *panel)
{
	struct nt36532 *nt = to_nt36532(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = nt->dsi };
	int ret;

	ret = regulator_enable(nt->vddi);
	if (ret)
		return ret;
	usleep_range(2000, 3000);

	ret = regulator_enable(nt->avdd);
	if (ret)
		goto err_vddi;
	usleep_range(2000, 3000);
	ret = regulator_enable(nt->avee);
	if (ret)
		goto err_vsp;

	gpiod_set_value_cansleep(nt->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(nt->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(nt->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(nt->reset_gpio, 0);
	usleep_range(10000, 11000);

	nt->desc->init(&ctx);
	mipi_dsi_dcs_exit_sleep_mode_multi(&ctx);
	mipi_dsi_msleep(&ctx, 150);
	mipi_dsi_dcs_set_display_on_multi(&ctx);
	mipi_dsi_msleep(&ctx, 20);
	if (ctx.accum_err) {
		ret = ctx.accum_err;
		goto err_reset;
	}

	return 0;

err_reset:
	gpiod_set_value_cansleep(nt->reset_gpio, 1);
	regulator_disable(nt->avee);
err_vsp:
	regulator_disable(nt->avdd);
err_vddi:
	regulator_disable(nt->vddi);
	return ret;
}

static int nt36532_unprepare(struct drm_panel *panel)
{
	struct nt36532 *nt = to_nt36532(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = nt->dsi };

	mipi_dsi_dcs_set_display_off_multi(&ctx);
	mipi_dsi_msleep(&ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&ctx);
	mipi_dsi_msleep(&ctx, 120);

	gpiod_set_value_cansleep(nt->reset_gpio, 1);
	usleep_range(5000, 6000);
	regulator_disable(nt->avee);
	usleep_range(2000, 3000);
	regulator_disable(nt->avdd);
	regulator_disable(nt->vddi);

	return 0;
}

static int nt36532_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct nt36532 *nt = to_nt36532(panel);

	return drm_connector_helper_get_modes_fixed(connector, nt->desc->mode);
}

static const struct drm_panel_funcs nt36532_funcs = {
	.prepare = nt36532_prepare,
	.unprepare = nt36532_unprepare,
	.get_modes = nt36532_get_modes,
};

static int nt36532_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt36532 *nt;
	int ret;

	nt = devm_drm_panel_alloc(dev, struct nt36532, panel, &nt36532_funcs,
				  DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(nt))
		return PTR_ERR(nt);

	nt->desc = of_device_get_match_data(dev);
	nt->dsi = dsi;

	nt->vddi = devm_regulator_get(dev, "vddi");
	if (IS_ERR(nt->vddi))
		return dev_err_probe(dev, PTR_ERR(nt->vddi), "no vddi\n");
	nt->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(nt->avdd))
		return dev_err_probe(dev, PTR_ERR(nt->avdd), "no avdd\n");
	nt->avee = devm_regulator_get(dev, "avee");
	if (IS_ERR(nt->avee))
		return dev_err_probe(dev, PTR_ERR(nt->avee), "no avee\n");

	/* Hold the panel in reset until it is prepared. */
	nt->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(nt->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(nt->reset_gpio),
				     "no reset gpio\n");

	ret = drm_panel_of_backlight(&nt->panel);
	if (ret)
		return dev_err_probe(dev, ret, "no backlight\n");

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM;
	mipi_dsi_set_drvdata(dsi, nt);

	drm_panel_add(&nt->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&nt->panel);
		return dev_err_probe(dev, ret, "cannot attach to the host\n");
	}

	return 0;
}

static void nt36532_remove(struct mipi_dsi_device *dsi)
{
	struct nt36532 *nt = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&nt->panel);
}

/* The 144 Hz mode; the frame is 2 x 1472 pixels wide, one stripe per link. */
static const struct drm_display_mode tb373fu_tianma_mode = {
	DRM_MODE("2944x1840", DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
		 890618, 2944, 2944 + 36, 2944 + 36 + 6, 2944 + 36 + 6 + 31, 0,
		 1840, 1840 + 26, 1840 + 26 + 2, 1840 + 26 + 2 + 182, 0, 0),
	.width_mm = 274,
	.height_mm = 171,
};

static const struct nt36532_desc tb373fu_tianma = {
	.mode = &tb373fu_tianma_mode,
	.init = tb373fu_tianma_init,
};

static const struct of_device_id nt36532_of_match[] = {
	{ .compatible = "lenovo,tb373fu-tianma-nt36532", .data = &tb373fu_tianma },
	{ }
};
MODULE_DEVICE_TABLE(of, nt36532_of_match);

static struct mipi_dsi_driver nt36532_driver = {
	.probe = nt36532_probe,
	.remove = nt36532_remove,
	.driver = {
		.name = "panel-novatek-nt36532",
		.of_match_table = nt36532_of_match,
	},
};
module_mipi_dsi_driver(nt36532_driver);

MODULE_DESCRIPTION("Novatek NT36532 DSI panel driver");
MODULE_LICENSE("GPL");
