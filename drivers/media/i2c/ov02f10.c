// SPDX-License-Identifier: GPL-2.0
/*
 * OmniVision OV02F10 image sensor driver
 *
 * Register tables are the Nubia RedMagic 11 Pro (nx809j) stock module
 * settings for its auxiliary rear camera. Driver structure follows
 * imx890.c.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

/*
 * Stock pixel clock 66 MHz: 10 bits over one lane is 660 Mbit/s, a
 * 330 MHz DDR link clock
 */
#define OV02F10_PIXEL_RATE		66000000ULL
#define OV02F10_LINK_FREQ_330MHZ	(330 * HZ_PER_MHZ)
#define OV02F10_MCLK_FREQ_19_2MHZ	(19200 * HZ_PER_KHZ)
#define OV02F10_DATA_LANES		1

/* Registers are one byte wide, in pages that 0xfd selects */
#define OV02F10_REG_PAGE		CCI_REG8(0xfd)

/* Page 0 */
#define OV02F10_REG_CHIP_ID		CCI_REG8(0x02)
#define OV02F10_CHIP_ID			0x56

#define OV02F10_REG_SOFTWARE_RESET	CCI_REG8(0xfc)

/* Page 3 */
#define OV02F10_REG_STREAM		CCI_REG8(0xc2)

#define to_ov02f10(_sd)			container_of(_sd, struct ov02f10, sd)

static const s64 ov02f10_link_freq_menu[] = {
	OV02F10_LINK_FREQ_330MHZ,
};

struct ov02f10_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct ov02f10_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;

	const struct ov02f10_reg_list reg_list;
};

enum {
	OV02F10_SUPPLY_DOVDD,	/* Digital I/O power */
	OV02F10_SUPPLY_AVDD,	/* Analog power */
	OV02F10_NUM_SUPPLIES,
};

static const char * const ov02f10_supply_names[] = {
	[OV02F10_SUPPLY_DOVDD] = "dovdd",
	[OV02F10_SUPPLY_AVDD] = "avdd",
};

struct ov02f10 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[OV02F10_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;

	const struct ov02f10_mode *mode;
};

/* Global init, after the software reset; 0xfd selects the page */
static const struct cci_reg_sequence ov02f10_init[] = {
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0x24), 0x02 },
	{ CCI_REG8(0x25), 0x19 },
	{ CCI_REG8(0x26), 0x01 },
	{ CCI_REG8(0x29), 0x03 },
	{ CCI_REG8(0x2a), 0x42 },
	{ CCI_REG8(0x27), 0x00 },
	{ CCI_REG8(0x28), 0x00 },
	{ CCI_REG8(0x4f), 0x06 },
	{ CCI_REG8(0x50), 0x40 },
	{ CCI_REG8(0x51), 0x04 },
	{ CCI_REG8(0x52), 0xb0 },
	{ CCI_REG8(0x5b), 0x08 },
	{ CCI_REG8(0x74), 0x01 },
	{ CCI_REG8(0x33), 0x00 },
	{ CCI_REG8(0x35), 0x03 },
	{ CCI_REG8(0x4a), 0x0a },
	{ CCI_REG8(0x46), 0x02 },
	{ CCI_REG8(0x3a), 0x05 },
	{ CCI_REG8(0x3b), 0x02 },
	{ CCI_REG8(0x3c), 0x01 },
	{ CCI_REG8(0x3e), 0x01 },
	{ CCI_REG8(0xfd), 0x01 },
	{ CCI_REG8(0x30), 0x01 },
	{ CCI_REG8(0x31), 0x0c },
	{ CCI_REG8(0x32), 0x04 },
	{ CCI_REG8(0x33), 0x02 },
	{ CCI_REG8(0x34), 0x05 },
	{ CCI_REG8(0x35), 0x60 },
	{ CCI_REG8(0x36), 0x01 },
	{ CCI_REG8(0x37), 0x07 },
	{ CCI_REG8(0x38), 0x01 },
	{ CCI_REG8(0x39), 0x01 },
	{ CCI_REG8(0x3a), 0x01 },
	{ CCI_REG8(0x3b), 0x01 },
	{ CCI_REG8(0x3c), 0x01 },
	{ CCI_REG8(0x3d), 0x01 },
	{ CCI_REG8(0x3e), 0x01 },
	{ CCI_REG8(0x3f), 0x00 },
	{ CCI_REG8(0x40), 0x01 },
	{ CCI_REG8(0x41), 0x01 },
	{ CCI_REG8(0x42), 0x0e },
	{ CCI_REG8(0x43), 0x1a },
	{ CCI_REG8(0x44), 0x01 },
	{ CCI_REG8(0x45), 0x73 },
	{ CCI_REG8(0x46), 0x73 },
	{ CCI_REG8(0x47), 0x73 },
	{ CCI_REG8(0x48), 0x73 },
	{ CCI_REG8(0x49), 0x01 },
	{ CCI_REG8(0x4a), 0x75 },
	{ CCI_REG8(0x4b), 0x75 },
	{ CCI_REG8(0x4c), 0x75 },
	{ CCI_REG8(0x4d), 0x75 },
	{ CCI_REG8(0x4e), 0x05 },
	{ CCI_REG8(0x4f), 0x04 },
	{ CCI_REG8(0x50), 0x01 },
	{ CCI_REG8(0x51), 0x01 },
	{ CCI_REG8(0x52), 0x0a },
	{ CCI_REG8(0x53), 0x01 },
	{ CCI_REG8(0x54), 0x10 },
	{ CCI_REG8(0x55), 0x04 },
	{ CCI_REG8(0x19), 0x25 },
	{ CCI_REG8(0x1b), 0x0b },
	{ CCI_REG8(0x1c), 0x09 },
	{ CCI_REG8(0x56), 0x86 },
	{ CCI_REG8(0x57), 0x0c },
	{ CCI_REG8(0x58), 0x28 },
	{ CCI_REG8(0x59), 0x17 },
	{ CCI_REG8(0x5a), 0x0c },
	{ CCI_REG8(0x5b), 0x8c },
	{ CCI_REG8(0x5c), 0x02 },
	{ CCI_REG8(0x5d), 0x0a },
	{ CCI_REG8(0x5e), 0x01 },
	{ CCI_REG8(0x5f), 0x04 },
	{ CCI_REG8(0x60), 0x01 },
	{ CCI_REG8(0x61), 0x04 },
	{ CCI_REG8(0x62), 0x04 },
	{ CCI_REG8(0x63), 0x03 },
	{ CCI_REG8(0x64), 0x01 },
	{ CCI_REG8(0x65), 0x00 },
	{ CCI_REG8(0x66), 0x2a },
	{ CCI_REG8(0x16), 0x00 },
	{ CCI_REG8(0x17), 0x42 },
	{ CCI_REG8(0x6e), 0x01 },
	{ CCI_REG8(0x98), 0x00 },
	{ CCI_REG8(0x0d), 0x44 },
	{ CCI_REG8(0x6f), 0x22 },
	{ CCI_REG8(0x68), 0x90 },
	{ CCI_REG8(0x69), 0xcd },
	{ CCI_REG8(0x6a), 0x0f },
	{ CCI_REG8(0x6b), 0x00 },
	{ CCI_REG8(0x7c), 0x05 },
	{ CCI_REG8(0x7d), 0x05 },
	{ CCI_REG8(0x7e), 0x05 },
	{ CCI_REG8(0x7f), 0x05 },
	{ CCI_REG8(0x83), 0x14 },
	{ CCI_REG8(0x84), 0x14 },
	{ CCI_REG8(0x85), 0x14 },
	{ CCI_REG8(0x86), 0x14 },
	{ CCI_REG8(0x89), 0x18 },
	{ CCI_REG8(0x8a), 0x18 },
	{ CCI_REG8(0x8b), 0x1b },
	{ CCI_REG8(0x8c), 0x1c },
	{ CCI_REG8(0x8d), 0x1e },
	{ CCI_REG8(0x88), 0x0f },
	{ CCI_REG8(0x94), 0x02 },
	{ CCI_REG8(0x98), 0xd1 },
	{ CCI_REG8(0xfd), 0x01 },
	{ CCI_REG8(0x14), 0x00 },
	{ CCI_REG8(0x15), 0x38 },
	{ CCI_REG8(0x02), 0x00 },
	{ CCI_REG8(0x03), 0x70 },
	{ CCI_REG8(0x04), 0x00 },
	{ CCI_REG8(0x05), 0x10 },
	{ CCI_REG8(0x06), 0x03 },
	{ CCI_REG8(0x07), 0x20 },
	{ CCI_REG8(0x08), 0x04 },
	{ CCI_REG8(0x09), 0xb0 },
	{ CCI_REG8(0x12), 0x03 },
	{ CCI_REG8(0xfd), 0x01 },
	{ CCI_REG8(0x18), 0x00 },
	{ CCI_REG8(0x0e), 0x00 },
	{ CCI_REG8(0x0f), 0x08 },
	{ CCI_REG8(0x21), 0x00 },
	{ CCI_REG8(0x22), 0xf8 },
	{ CCI_REG8(0x20), 0x03 },
	{ CCI_REG8(0x23), 0x04 },
	{ CCI_REG8(0xfd), 0x03 },
	{ CCI_REG8(0xae), 0x0d },
	{ CCI_REG8(0x88), 0x7b },
	{ CCI_REG8(0x89), 0x7c },
	{ CCI_REG8(0xb4), 0x05 },
	{ CCI_REG8(0x81), 0x00 },
	{ CCI_REG8(0x8c), 0x40 },
	{ CCI_REG8(0x8e), 0x40 },
	{ CCI_REG8(0x90), 0x40 },
	{ CCI_REG8(0x92), 0x40 },
	{ CCI_REG8(0x97), 0x60 },
	{ CCI_REG8(0x98), 0x60 },
	{ CCI_REG8(0x99), 0x60 },
	{ CCI_REG8(0x9a), 0x60 },
	{ CCI_REG8(0xbd), 0x02 },
	{ CCI_REG8(0x9b), 0x40 },
	{ CCI_REG8(0xac), 0x40 },
	{ CCI_REG8(0xaf), 0x08 },
	{ CCI_REG8(0xb0), 0x00 },
	{ CCI_REG8(0xfe), 0x02 },
};

static const struct cci_reg_sequence ov02f10_1600x1200_mode[] = {
	{ CCI_REG8(0xfb), 0x01 },
};

static const struct ov02f10_mode ov02f10_supported_modes[] = {
	{
		/* Full resolution, 30.07 fps */
		.width = 1600,
		.height = 1200,
		.hts = 1720,
		.vts = 1276,
		.reg_list = {
			.regs = ov02f10_1600x1200_mode,
			.num_regs = ARRAY_SIZE(ov02f10_1600x1200_mode),
		},
	},
};

static int ov02f10_init_controls(struct ov02f10 *ov02f10)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &ov02f10->ctrl_handler;
	const struct ov02f10_mode *mode = ov02f10->mode;
	struct v4l2_fwnode_device_properties props;
	s64 hblank, vblank;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 6);

	ov02f10->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(ov02f10_link_freq_menu) - 1,
					0, ov02f10_link_freq_menu);
	if (ov02f10->link_freq)
		ov02f10->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ov02f10->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, NULL,
						V4L2_CID_PIXEL_RATE,
						OV02F10_PIXEL_RATE,
						OV02F10_PIXEL_RATE, 1,
						OV02F10_PIXEL_RATE);

	hblank = mode->hts - mode->width;
	ov02f10->hblank = v4l2_ctrl_new_std(ctrl_hdlr, NULL,
					    V4L2_CID_HBLANK, hblank,
					    hblank, 1, hblank);
	if (ov02f10->hblank)
		ov02f10->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	ov02f10->vblank = v4l2_ctrl_new_std(ctrl_hdlr, NULL, V4L2_CID_VBLANK,
					    vblank, vblank, 1, vblank);
	if (ov02f10->vblank)
		ov02f10->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ret = v4l2_fwnode_device_parse(ov02f10->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, NULL, &props);
	if (ret)
		goto error_free_hdlr;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	ov02f10->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int ov02f10_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct ov02f10 *ov02f10 = to_ov02f10(sd);
	const struct ov02f10_reg_list *reg_list = &ov02f10->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(ov02f10->dev);
	if (ret)
		return ret;

	cci_write(ov02f10->regmap, OV02F10_REG_PAGE, 0, &ret);
	cci_write(ov02f10->regmap, OV02F10_REG_SOFTWARE_RESET, 0x01, &ret);
	if (ret)
		goto error;

	usleep_range(5000, 5500);

	cci_multi_reg_write(ov02f10->regmap, ov02f10_init,
			    ARRAY_SIZE(ov02f10_init), &ret);
	cci_multi_reg_write(ov02f10->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	cci_write(ov02f10->regmap, OV02F10_REG_PAGE, 3, &ret);
	cci_write(ov02f10->regmap, OV02F10_REG_STREAM, 1, &ret);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(ov02f10->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(ov02f10->dev);

	return ret;
}

static int ov02f10_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct ov02f10 *ov02f10 = to_ov02f10(sd);
	int ret = 0;

	cci_write(ov02f10->regmap, OV02F10_REG_PAGE, 3, &ret);
	cci_write(ov02f10->regmap, OV02F10_REG_STREAM, 0, &ret);
	if (ret)
		dev_err(ov02f10->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(ov02f10->dev);

	return ret;
}

static void ov02f10_update_pad_format(const struct ov02f10_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = MEDIA_BUS_FMT_SGBRG10_1X10;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int ov02f10_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct ov02f10 *ov02f10 = to_ov02f10(sd);
	const struct ov02f10_mode *mode;

	mode = v4l2_find_nearest_size(ov02f10_supported_modes,
				      ARRAY_SIZE(ov02f10_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	ov02f10_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ov02f10->mode = mode;

	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int ov02f10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGBRG10_1X10;

	return 0;
}

static int ov02f10_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(ov02f10_supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGBRG10_1X10)
		return -EINVAL;

	fse->min_width = ov02f10_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ov02f10_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov02f10_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = 1600;
		sel->r.height = 1200;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ov02f10_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct ov02f10 *ov02f10 = to_ov02f10(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = ov02f10->mode->width,
			.height = ov02f10->mode->height,
		},
	};

	ov02f10_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops ov02f10_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops ov02f10_pad_ops = {
	.set_fmt = ov02f10_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = ov02f10_get_selection,
	.enum_mbus_code = ov02f10_enum_mbus_code,
	.enum_frame_size = ov02f10_enum_frame_size,
	.enable_streams = ov02f10_enable_streams,
	.disable_streams = ov02f10_disable_streams,
};

static const struct v4l2_subdev_ops ov02f10_subdev_ops = {
	.video = &ov02f10_video_ops,
	.pad = &ov02f10_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov02f10_internal_ops = {
	.init_state = ov02f10_init_state,
};

static const struct media_entity_operations ov02f10_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov02f10_identify_sensor(struct ov02f10 *ov02f10)
{
	u64 val;
	int ret = 0;

	cci_write(ov02f10->regmap, OV02F10_REG_PAGE, 0, &ret);
	cci_read(ov02f10->regmap, OV02F10_REG_CHIP_ID, &val, &ret);
	if (ret) {
		dev_err(ov02f10->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != OV02F10_CHIP_ID) {
		dev_err(ov02f10->dev, "chip id mismatch: %x!=%llx\n",
			OV02F10_CHIP_ID, val);
		return -ENODEV;
	}

	return 0;
}

static int ov02f10_check_hwcfg(struct ov02f10 *ov02f10)
{
	struct fwnode_handle *fwnode = dev_fwnode(ov02f10->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OV02F10_DATA_LANES) {
		dev_err(ov02f10->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(ov02f10->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       ov02f10_link_freq_menu,
				       ARRAY_SIZE(ov02f10_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/*
 * Stock order: reset held 3 ms, DOVDD, AVDD 3 ms later, reset release
 * 10 ms later, MCLK 10 ms later, 5 ms to settle
 */
static int ov02f10_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov02f10 *ov02f10 = to_ov02f10(sd);
	int ret;

	gpiod_set_value_cansleep(ov02f10->reset_gpio, 1);
	usleep_range(3000, 3500);

	ret = regulator_enable(ov02f10->supplies[OV02F10_SUPPLY_DOVDD].consumer);
	if (ret)
		return ret;

	usleep_range(3000, 3500);

	ret = regulator_enable(ov02f10->supplies[OV02F10_SUPPLY_AVDD].consumer);
	if (ret)
		goto disable_dovdd;

	usleep_range(10000, 10500);
	gpiod_set_value_cansleep(ov02f10->reset_gpio, 0);
	usleep_range(10000, 10500);

	ret = clk_prepare_enable(ov02f10->mclk);
	if (ret)
		goto assert_reset;

	usleep_range(5000, 5500);

	return 0;

assert_reset:
	gpiod_set_value_cansleep(ov02f10->reset_gpio, 1);
	regulator_disable(ov02f10->supplies[OV02F10_SUPPLY_AVDD].consumer);
disable_dovdd:
	regulator_disable(ov02f10->supplies[OV02F10_SUPPLY_DOVDD].consumer);

	return ret;
}

/* Stock order: MCLK, reset, DOVDD, AVDD, 1 ms apart */
static int ov02f10_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov02f10 *ov02f10 = to_ov02f10(sd);

	clk_disable_unprepare(ov02f10->mclk);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(ov02f10->reset_gpio, 1);
	usleep_range(1000, 1500);
	regulator_disable(ov02f10->supplies[OV02F10_SUPPLY_DOVDD].consumer);
	usleep_range(1000, 1500);
	regulator_disable(ov02f10->supplies[OV02F10_SUPPLY_AVDD].consumer);

	return 0;
}

static int ov02f10_probe(struct i2c_client *client)
{
	struct ov02f10 *ov02f10;
	unsigned long freq;
	unsigned int i;
	int ret;

	ov02f10 = devm_kzalloc(&client->dev, sizeof(*ov02f10), GFP_KERNEL);
	if (!ov02f10)
		return -ENOMEM;

	ov02f10->dev = &client->dev;
	v4l2_i2c_subdev_init(&ov02f10->sd, client, &ov02f10_subdev_ops);

	ov02f10->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(ov02f10->regmap))
		return dev_err_probe(ov02f10->dev, PTR_ERR(ov02f10->regmap),
				     "failed to init CCI\n");

	ov02f10->mclk = devm_v4l2_sensor_clk_get(ov02f10->dev, NULL);
	if (IS_ERR(ov02f10->mclk))
		return dev_err_probe(ov02f10->dev, PTR_ERR(ov02f10->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(ov02f10->mclk);
	if (freq != OV02F10_MCLK_FREQ_19_2MHZ)
		return dev_err_probe(ov02f10->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = ov02f10_check_hwcfg(ov02f10);
	if (ret)
		return dev_err_probe(ov02f10->dev, ret,
				     "failed to check HW configuration\n");

	ov02f10->reset_gpio = devm_gpiod_get_optional(ov02f10->dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(ov02f10->reset_gpio))
		return dev_err_probe(ov02f10->dev, PTR_ERR(ov02f10->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < OV02F10_NUM_SUPPLIES; i++)
		ov02f10->supplies[i].supply = ov02f10_supply_names[i];

	ret = devm_regulator_bulk_get(ov02f10->dev, OV02F10_NUM_SUPPLIES,
				      ov02f10->supplies);
	if (ret)
		return dev_err_probe(ov02f10->dev, ret,
				     "failed to get supply regulators\n");

	ret = ov02f10_power_on(ov02f10->dev);
	if (ret)
		return ret;

	ret = ov02f10_identify_sensor(ov02f10);
	if (ret) {
		dev_err_probe(ov02f10->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	ov02f10->mode = &ov02f10_supported_modes[0];
	ret = ov02f10_init_controls(ov02f10);
	if (ret) {
		dev_err_probe(ov02f10->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	ov02f10->sd.state_lock = ov02f10->ctrl_handler.lock;
	ov02f10->sd.internal_ops = &ov02f10_internal_ops;
	ov02f10->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov02f10->sd.entity.ops = &ov02f10_subdev_entity_ops;
	ov02f10->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov02f10->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ov02f10->sd.entity, 1, &ov02f10->pad);
	if (ret) {
		dev_err_probe(ov02f10->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&ov02f10->sd);
	if (ret < 0) {
		dev_err_probe(ov02f10->dev, ret, "failed to init subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(ov02f10->dev);
	pm_runtime_enable(ov02f10->dev);

	ret = v4l2_async_register_subdev_sensor(&ov02f10->sd);
	if (ret < 0) {
		dev_err_probe(ov02f10->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(ov02f10->dev, 1000);
	pm_runtime_use_autosuspend(ov02f10->dev);
	pm_runtime_idle(ov02f10->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&ov02f10->sd);
	pm_runtime_disable(ov02f10->dev);
	pm_runtime_set_suspended(ov02f10->dev);

media_entity_cleanup:
	media_entity_cleanup(&ov02f10->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(ov02f10->sd.ctrl_handler);

power_off:
	ov02f10_power_off(ov02f10->dev);

	return ret;
}

static void ov02f10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov02f10 *ov02f10 = to_ov02f10(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(ov02f10->dev);

	if (!pm_runtime_status_suspended(ov02f10->dev)) {
		ov02f10_power_off(ov02f10->dev);
		pm_runtime_set_suspended(ov02f10->dev);
	}
}

static const struct dev_pm_ops ov02f10_pm_ops = {
	SET_RUNTIME_PM_OPS(ov02f10_power_off, ov02f10_power_on, NULL)
};

static const struct of_device_id ov02f10_of_match[] = {
	{ .compatible = "ovti,ov02f10" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov02f10_of_match);

static struct i2c_driver ov02f10_i2c_driver = {
	.driver = {
		.name = "ov02f10",
		.pm = &ov02f10_pm_ops,
		.of_match_table = ov02f10_of_match,
	},
	.probe = ov02f10_probe,
	.remove = ov02f10_remove,
};

module_i2c_driver(ov02f10_i2c_driver);

MODULE_DESCRIPTION("OmniVision OV02F10 image sensor driver");
MODULE_LICENSE("GPL");
