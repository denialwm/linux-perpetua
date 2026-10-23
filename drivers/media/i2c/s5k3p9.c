// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K3P9 image sensor driver
 *
 * Register tables are the OnePlus Ace 3 (aston) stock module settings for
 * its Sunny front camera module. Driver structure follows s5k3m5.c.
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

/* Output PLL: 19.2 MHz / 3 * 88, DDR */
#define S5K3P9_LINK_FREQ_563P2MHZ	(563200ULL * HZ_PER_KHZ)
#define S5K3P9_MCLK_FREQ_19P2MHZ	(19200 * HZ_PER_KHZ)
#define S5K3P9_DATA_LANES		4
/* Video timing: 19.2 MHz / 5 * 255 / 7, four pixels per clock */
#define S5K3P9_PIXEL_RATE		559542857ULL

#define S5K3P9_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K3P9_CHIP_ID			0x3109

/* Streaming (byte 0x0100) and orientation (byte 0x0101) together */
#define S5K3P9_REG_CTRL_MODE		CCI_REG16(0x0100)
#define S5K3P9_MODE_STREAMING		BIT(8)
#define S5K3P9_VFLIP			BIT(1)
#define S5K3P9_HFLIP			BIT(0)

#define S5K3P9_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K3P9_EXPOSURE_MIN		3
#define S5K3P9_EXPOSURE_STEP		1
#define S5K3P9_EXPOSURE_MARGIN		3

/* Analogue gain code = gain * 32, 1x to 16x */
#define S5K3P9_REG_AGAIN		CCI_REG16(0x0204)
#define S5K3P9_AGAIN_MIN		0x0020
#define S5K3P9_AGAIN_MAX		0x0200
#define S5K3P9_AGAIN_STEP		1
#define S5K3P9_AGAIN_DEFAULT		0x0020

#define S5K3P9_REG_VTS			CCI_REG16(0x0340)
#define S5K3P9_VTS_MAX			0xffff

#define S5K3P9_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define to_s5k3p9(_sd)			container_of(_sd, struct s5k3p9, sd)

static const s64 s5k3p9_link_freq_menu[] = {
	S5K3P9_LINK_FREQ_563P2MHZ,
};

/* Native order is GRBG; indexed by (vflip << 1) | hflip */
static const u32 s5k3p9_mbus_formats[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,	MEDIA_BUS_FMT_SGBRG10_1X10,
};

struct s5k3p9_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k3p9_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 exposure;

	const struct s5k3p9_reg_list reg_list;
};

static const char * const s5k3p9_test_pattern_menu[] = {
	"Disabled",
	"Solid colour",
	"Colour bars",
	"Fade to grey colour bars",
	"PN9",
};

static const char * const s5k3p9_supply_names[] = {
	"vdda",		/* Analog power */
	"vddd",		/* Digital core power */
	"vddio",	/* Digital I/O power */
};

#define S5K3P9_NUM_SUPPLIES	ARRAY_SIZE(s5k3p9_supply_names)

struct s5k3p9 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[S5K3P9_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;

	const struct s5k3p9_mode *mode;
};

/*
 * Global init. The first part ends with a soft reset (0x6010), which needs
 * 3 ms before the rest. Writes to 0x0000 set the firmware version, as on
 * other Samsung sensors. 0x6028/0x602a/0x6f12 are indirect page, address
 * and auto-incrementing data writes and must keep their order.
 */
static const struct cci_reg_sequence s5k3p9_init_0[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0000), 0x101f },
	{ CCI_REG16(0x0000), 0x3109 },
	{ CCI_REG16(0x6010), 0x0001 },
};

static const struct cci_reg_sequence s5k3p9_init_1[] = {
	{ CCI_REG16(0x6214), 0x7970 },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0x0a02), 0x007e },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x3f4c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0649 },
	{ CCI_REG16(0x6f12), 0x0548 },
	{ CCI_REG16(0x6f12), 0xc1f8 },
	{ CCI_REG16(0x6f12), 0xc405 },
	{ CCI_REG16(0x6f12), 0x0549 },
	{ CCI_REG16(0x6f12), 0x081a },
	{ CCI_REG16(0x6f12), 0x0349 },
	{ CCI_REG16(0x6f12), 0xa1f8 },
	{ CCI_REG16(0x6f12), 0xc805 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x65bc },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x4a84 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2ed0 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x6c00 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x40ba },
	{ CCI_REG16(0x6f12), 0x7047 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xc0ba },
	{ CCI_REG16(0x6f12), 0x7047 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf047 },
	{ CCI_REG16(0x6f12), 0x1c46 },
	{ CCI_REG16(0x6f12), 0x9046 },
	{ CCI_REG16(0x6f12), 0x8946 },
	{ CCI_REG16(0x6f12), 0x0746 },
	{ CCI_REG16(0x6f12), 0xfe48 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x0068 },
	{ CCI_REG16(0x6f12), 0x86b2 },
	{ CCI_REG16(0x6f12), 0x050c },
	{ CCI_REG16(0x6f12), 0x3146 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc5fc },
	{ CCI_REG16(0x6f12), 0x2346 },
	{ CCI_REG16(0x6f12), 0x4246 },
	{ CCI_REG16(0x6f12), 0x4946 },
	{ CCI_REG16(0x6f12), 0x3846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc4fc },
	{ CCI_REG16(0x6f12), 0xf848 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x8b02 },
	{ CCI_REG16(0x6f12), 0x88b1 },
	{ CCI_REG16(0x6f12), 0x788a },
	{ CCI_REG16(0x6f12), 0x04f1 },
	{ CCI_REG16(0x6f12), 0x0054 },
	{ CCI_REG16(0x6f12), 0x04eb },
	{ CCI_REG16(0x6f12), 0x8001 },
	{ CCI_REG16(0x6f12), 0x09e0 },
	{ CCI_REG16(0x6f12), 0x2268 },
	{ CCI_REG16(0x6f12), 0xc2f3 },
	{ CCI_REG16(0x6f12), 0xc360 },
	{ CCI_REG16(0x6f12), 0x90fa },
	{ CCI_REG16(0x6f12), 0xa0f0 },
	{ CCI_REG16(0x6f12), 0x22f0 },
	{ CCI_REG16(0x6f12), 0x7842 },
	{ CCI_REG16(0x6f12), 0x42ea },
	{ CCI_REG16(0x6f12), 0x5000 },
	{ CCI_REG16(0x6f12), 0x01c4 },
	{ CCI_REG16(0x6f12), 0x8c42 },
	{ CCI_REG16(0x6f12), 0xf3d1 },
	{ CCI_REG16(0x6f12), 0x3146 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf047 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xa2bc },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xfc5f },
	{ CCI_REG16(0x6f12), 0x8346 },
	{ CCI_REG16(0x6f12), 0xe748 },
	{ CCI_REG16(0x6f12), 0x8a46 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x4068 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x80b2 },
	{ CCI_REG16(0x6f12), 0xcde9 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x0198 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x93fc },
	{ CCI_REG16(0x6f12), 0xabfb },
	{ CCI_REG16(0x6f12), 0x0a10 },
	{ CCI_REG16(0x6f12), 0xe24b },
	{ CCI_REG16(0x6f12), 0xe04d },
	{ CCI_REG16(0x6f12), 0xe04a },
	{ CCI_REG16(0x6f12), 0x93f8 },
	{ CCI_REG16(0x6f12), 0x9160 },
	{ CCI_REG16(0x6f12), 0x05f5 },
	{ CCI_REG16(0x6f12), 0xaa69 },
	{ CCI_REG16(0x6f12), 0x06fb },
	{ CCI_REG16(0x6f12), 0x0bf6 },
	{ CCI_REG16(0x6f12), 0x0023 },
	{ CCI_REG16(0x6f12), 0x891b },
	{ CCI_REG16(0x6f12), 0x4d46 },
	{ CCI_REG16(0x6f12), 0x60eb },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x03c5 },
	{ CCI_REG16(0x6f12), 0x1d46 },
	{ CCI_REG16(0x6f12), 0xebfb },
	{ CCI_REG16(0x6f12), 0x0a65 },
	{ CCI_REG16(0x6f12), 0x02f5 },
	{ CCI_REG16(0x6f12), 0xab67 },
	{ CCI_REG16(0x6f12), 0x3a46 },
	{ CCI_REG16(0x6f12), 0xd64c },
	{ CCI_REG16(0x6f12), 0x60c2 },
	{ CCI_REG16(0x6f12), 0xa4f8 },
	{ CCI_REG16(0x6f12), 0x4835 },
	{ CCI_REG16(0x6f12), 0x04f5 },
	{ CCI_REG16(0x6f12), 0xa962 },
	{ CCI_REG16(0x6f12), 0x94f8 },
	{ CCI_REG16(0x6f12), 0xa0c4 },
	{ CCI_REG16(0x6f12), 0x94f8 },
	{ CCI_REG16(0x6f12), 0xa144 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0xf858 },
	{ CCI_REG16(0x6f12), 0xbcf1 },
	{ CCI_REG16(0x6f12), 0x010f },
	{ CCI_REG16(0x6f12), 0x03d0 },
	{ CCI_REG16(0x6f12), 0xbcf1 },
	{ CCI_REG16(0x6f12), 0x020f },
	{ CCI_REG16(0x6f12), 0x14d0 },
	{ CCI_REG16(0x6f12), 0x29e0 },
	{ CCI_REG16(0x6f12), 0x08ea },
	{ CCI_REG16(0x6f12), 0x0423 },
	{ CCI_REG16(0x6f12), 0x43f0 },
	{ CCI_REG16(0x6f12), 0x1103 },
	{ CCI_REG16(0x6f12), 0x1380 },
	{ CCI_REG16(0x6f12), 0x2346 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x6afc },
	{ CCI_REG16(0x6f12), 0xc9e9 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x2346 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x3146 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x62fc },
	{ CCI_REG16(0x6f12), 0xc7e9 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x15e0 },
	{ CCI_REG16(0x6f12), 0x08ea },
	{ CCI_REG16(0x6f12), 0x042c },
	{ CCI_REG16(0x6f12), 0x4cf0 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0xa2f8 },
	{ CCI_REG16(0x6f12), 0x00c0 },
	{ CCI_REG16(0x6f12), 0xa1fb },
	{ CCI_REG16(0x6f12), 0x042c },
	{ CCI_REG16(0x6f12), 0x00fb },
	{ CCI_REG16(0x6f12), 0x04c0 },
	{ CCI_REG16(0x6f12), 0x01fb },
	{ CCI_REG16(0x6f12), 0x0301 },
	{ CCI_REG16(0x6f12), 0xc9e9 },
	{ CCI_REG16(0x6f12), 0x0012 },
	{ CCI_REG16(0x6f12), 0xa6fb },
	{ CCI_REG16(0x6f12), 0x0401 },
	{ CCI_REG16(0x6f12), 0x05fb },
	{ CCI_REG16(0x6f12), 0x0411 },
	{ CCI_REG16(0x6f12), 0x06fb },
	{ CCI_REG16(0x6f12), 0x0311 },
	{ CCI_REG16(0x6f12), 0xc7e9 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0xb848 },
	{ CCI_REG16(0x6f12), 0xb949 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0x4805 },
	{ CCI_REG16(0x6f12), 0x0880 },
	{ CCI_REG16(0x6f12), 0xb848 },
	{ CCI_REG16(0x6f12), 0x0cc8 },
	{ CCI_REG16(0x6f12), 0x48f6 },
	{ CCI_REG16(0x6f12), 0x2200 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x43fc },
	{ CCI_REG16(0x6f12), 0xb548 },
	{ CCI_REG16(0x6f12), 0x0830 },
	{ CCI_REG16(0x6f12), 0x0cc8 },
	{ CCI_REG16(0x6f12), 0x48f6 },
	{ CCI_REG16(0x6f12), 0x2a00 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x3cfc },
	{ CCI_REG16(0x6f12), 0x5846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x3efc },
	{ CCI_REG16(0x6f12), 0xad49 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0xc1f8 },
	{ CCI_REG16(0x6f12), 0x68a5 },
	{ CCI_REG16(0x6f12), 0xdde9 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x02b0 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf05f },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x1fbc },
	{ CCI_REG16(0x6f12), 0xa84a },
	{ CCI_REG16(0x6f12), 0x92f8 },
	{ CCI_REG16(0x6f12), 0xd525 },
	{ CCI_REG16(0x6f12), 0x2ab1 },
	{ CCI_REG16(0x6f12), 0xa64a },
	{ CCI_REG16(0x6f12), 0xa54b },
	{ CCI_REG16(0x6f12), 0xd2f8 },
	{ CCI_REG16(0x6f12), 0x6825 },
	{ CCI_REG16(0x6f12), 0xc3f8 },
	{ CCI_REG16(0x6f12), 0x3024 },
	{ CCI_REG16(0x6f12), 0xa34a },
	{ CCI_REG16(0x6f12), 0xd2f8 },
	{ CCI_REG16(0x6f12), 0x3024 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x29bc },
	{ CCI_REG16(0x6f12), 0x10b5 },
	{ CCI_REG16(0x6f12), 0xa049 },
	{ CCI_REG16(0x6f12), 0xa34a },
	{ CCI_REG16(0x6f12), 0xa44b },
	{ CCI_REG16(0x6f12), 0xd1f8 },
	{ CCI_REG16(0x6f12), 0x3c14 },
	{ CCI_REG16(0x6f12), 0x947c },
	{ CCI_REG16(0x6f12), 0x0cb1 },
	{ CCI_REG16(0x6f12), 0x908a },
	{ CCI_REG16(0x6f12), 0x1be0 },
	{ CCI_REG16(0x6f12), 0x9b4a },
	{ CCI_REG16(0x6f12), 0x92f8 },
	{ CCI_REG16(0x6f12), 0xa220 },
	{ CCI_REG16(0x6f12), 0xc2f1 },
	{ CCI_REG16(0x6f12), 0x0c02 },
	{ CCI_REG16(0x6f12), 0xd140 },
	{ CCI_REG16(0x6f12), 0x4843 },
	{ CCI_REG16(0x6f12), 0x010a },
	{ CCI_REG16(0x6f12), 0x9d48 },
	{ CCI_REG16(0x6f12), 0xd0f8 },
	{ CCI_REG16(0x6f12), 0x8400 },
	{ CCI_REG16(0x6f12), 0x0279 },
	{ CCI_REG16(0x6f12), 0x4a43 },
	{ CCI_REG16(0x6f12), 0x4179 },
	{ CCI_REG16(0x6f12), 0xc088 },
	{ CCI_REG16(0x6f12), 0xca40 },
	{ CCI_REG16(0x6f12), 0x00eb },
	{ CCI_REG16(0x6f12), 0x1210 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x8021 },
	{ CCI_REG16(0x6f12), 0xb1fb },
	{ CCI_REG16(0x6f12), 0xf0f0 },
	{ CCI_REG16(0x6f12), 0x0911 },
	{ CCI_REG16(0x6f12), 0x8842 },
	{ CCI_REG16(0x6f12), 0x04d2 },
	{ CCI_REG16(0x6f12), 0x4028 },
	{ CCI_REG16(0x6f12), 0x00d8 },
	{ CCI_REG16(0x6f12), 0x4020 },
	{ CCI_REG16(0x6f12), 0x5880 },
	{ CCI_REG16(0x6f12), 0x10bd },
	{ CCI_REG16(0x6f12), 0x0846 },
	{ CCI_REG16(0x6f12), 0xfbe7 },
	{ CCI_REG16(0x6f12), 0x4168 },
	{ CCI_REG16(0x6f12), 0x4a7b },
	{ CCI_REG16(0x6f12), 0x9149 },
	{ CCI_REG16(0x6f12), 0xa1f8 },
	{ CCI_REG16(0x6f12), 0x8223 },
	{ CCI_REG16(0x6f12), 0x4268 },
	{ CCI_REG16(0x6f12), 0x537b },
	{ CCI_REG16(0x6f12), 0x002b },
	{ CCI_REG16(0x6f12), 0x15d0 },
	{ CCI_REG16(0x6f12), 0x01f5 },
	{ CCI_REG16(0x6f12), 0x6171 },
	{ CCI_REG16(0x6f12), 0x927b },
	{ CCI_REG16(0x6f12), 0x0a80 },
	{ CCI_REG16(0x6f12), 0x4068 },
	{ CCI_REG16(0x6f12), 0xc07b },
	{ CCI_REG16(0x6f12), 0x4880 },
	{ CCI_REG16(0x6f12), 0x8b48 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0xc220 },
	{ CCI_REG16(0x6f12), 0x8a80 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0xc420 },
	{ CCI_REG16(0x6f12), 0xca80 },
	{ CCI_REG16(0x6f12), 0x10f8 },
	{ CCI_REG16(0x6f12), 0xc72f },
	{ CCI_REG16(0x6f12), 0xc078 },
	{ CCI_REG16(0x6f12), 0x5208 },
	{ CCI_REG16(0x6f12), 0x4008 },
	{ CCI_REG16(0x6f12), 0x42ea },
	{ CCI_REG16(0x6f12), 0x8000 },
	{ CCI_REG16(0x6f12), 0x0881 },
	{ CCI_REG16(0x6f12), 0x7047 },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xff4f },
	{ CCI_REG16(0x6f12), 0x8348 },
	{ CCI_REG16(0x6f12), 0x83b0 },
	{ CCI_REG16(0x6f12), 0x1d46 },
	{ CCI_REG16(0x6f12), 0xc079 },
	{ CCI_REG16(0x6f12), 0xddf8 },
	{ CCI_REG16(0x6f12), 0x44b0 },
	{ CCI_REG16(0x6f12), 0x1646 },
	{ CCI_REG16(0x6f12), 0x0f46 },
	{ CCI_REG16(0x6f12), 0x0028 },
	{ CCI_REG16(0x6f12), 0x6ed0 },
	{ CCI_REG16(0x6f12), 0xdff8 },
	{ CCI_REG16(0x6f12), 0xf4a1 },
	{ CCI_REG16(0x6f12), 0x0af1 },
	{ CCI_REG16(0x6f12), 0xba0a },
	{ CCI_REG16(0x6f12), 0xaaf1 },
	{ CCI_REG16(0x6f12), 0x1c00 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0x0090 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0x0480 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xccfb },
	{ CCI_REG16(0x6f12), 0x0399 },
	{ CCI_REG16(0x6f12), 0x109c },
	{ CCI_REG16(0x6f12), 0x0843 },
	{ CCI_REG16(0x6f12), 0x04f1 },
	{ CCI_REG16(0x6f12), 0x8044 },
	{ CCI_REG16(0x6f12), 0x07d0 },
	{ CCI_REG16(0x6f12), 0xa780 },
	{ CCI_REG16(0x6f12), 0xe680 },
	{ CCI_REG16(0x6f12), 0xaaf1 },
	{ CCI_REG16(0x6f12), 0x1c00 },
	{ CCI_REG16(0x6f12), 0x0188 },
	{ CCI_REG16(0x6f12), 0x2181 },
	{ CCI_REG16(0x6f12), 0x8088 },
	{ CCI_REG16(0x6f12), 0x20e0 },
	{ CCI_REG16(0x6f12), 0x6848 },
	{ CCI_REG16(0x6f12), 0x9af8 },
	{ CCI_REG16(0x6f12), 0x0c10 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0xd801 },
	{ CCI_REG16(0x6f12), 0x4843 },
	{ CCI_REG16(0x6f12), 0x0290 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xbafb },
	{ CCI_REG16(0x6f12), 0x0028 },
	{ CCI_REG16(0x6f12), 0x0298 },
	{ CCI_REG16(0x6f12), 0x01d0 },
	{ CCI_REG16(0x6f12), 0x361a },
	{ CCI_REG16(0x6f12), 0x00e0 },
	{ CCI_REG16(0x6f12), 0x0744 },
	{ CCI_REG16(0x6f12), 0xa780 },
	{ CCI_REG16(0x6f12), 0xe680 },
	{ CCI_REG16(0x6f12), 0x6048 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0xda61 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x8a02 },
	{ CCI_REG16(0x6f12), 0x4643 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xaffb },
	{ CCI_REG16(0x6f12), 0x10b1 },
	{ CCI_REG16(0x6f12), 0xa8eb },
	{ CCI_REG16(0x6f12), 0x0608 },
	{ CCI_REG16(0x6f12), 0x00e0 },
	{ CCI_REG16(0x6f12), 0xb144 },
	{ CCI_REG16(0x6f12), 0xa4f8 },
	{ CCI_REG16(0x6f12), 0x0890 },
	{ CCI_REG16(0x6f12), 0x4046 },
	{ CCI_REG16(0x6f12), 0x6081 },
	{ CCI_REG16(0x6f12), 0x0398 },
	{ CCI_REG16(0x6f12), 0x28b1 },
	{ CCI_REG16(0x6f12), 0x5648 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x4f11 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x8902 },
	{ CCI_REG16(0x6f12), 0x03e0 },
	{ CCI_REG16(0x6f12), 0x9af8 },
	{ CCI_REG16(0x6f12), 0x0d10 },
	{ CCI_REG16(0x6f12), 0x9af8 },
	{ CCI_REG16(0x6f12), 0x0c00 },
	{ CCI_REG16(0x6f12), 0x0a01 },
	{ CCI_REG16(0x6f12), 0x5149 },
	{ CCI_REG16(0x6f12), 0x91f8 },
	{ CCI_REG16(0x6f12), 0x4e11 },
	{ CCI_REG16(0x6f12), 0x42ea },
	{ CCI_REG16(0x6f12), 0x8121 },
	{ CCI_REG16(0x6f12), 0x41f0 },
	{ CCI_REG16(0x6f12), 0x0301 },
	{ CCI_REG16(0x6f12), 0xa181 },
	{ CCI_REG16(0x6f12), 0x0121 },
	{ CCI_REG16(0x6f12), 0xff22 },
	{ CCI_REG16(0x6f12), 0x02eb },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0x41ea },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0xe081 },
	{ CCI_REG16(0x6f12), 0x01a9 },
	{ CCI_REG16(0x6f12), 0x6846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x8bfb },
	{ CCI_REG16(0x6f12), 0x9df8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x9df8 },
	{ CCI_REG16(0x6f12), 0x0410 },
	{ CCI_REG16(0x6f12), 0x40ea },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x6f12), 0x2082 },
	{ CCI_REG16(0x6f12), 0x5f46 },
	{ CCI_REG16(0x6f12), 0x3e46 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x7bfb },
	{ CCI_REG16(0x6f12), 0x791e },
	{ CCI_REG16(0x6f12), 0x0028 },
	{ CCI_REG16(0x6f12), 0xa889 },
	{ CCI_REG16(0x6f12), 0x04d0 },
	{ CCI_REG16(0x6f12), 0x4718 },
	{ CCI_REG16(0x6f12), 0x46f6 },
	{ CCI_REG16(0x6f12), 0xa410 },
	{ CCI_REG16(0x6f12), 0x03e0 },
	{ CCI_REG16(0x6f12), 0x3fe0 },
	{ CCI_REG16(0x6f12), 0x4618 },
	{ CCI_REG16(0x6f12), 0x46f6 },
	{ CCI_REG16(0x6f12), 0x2410 },
	{ CCI_REG16(0x6f12), 0xa880 },
	{ CCI_REG16(0x6f12), 0x6782 },
	{ CCI_REG16(0x6f12), 0xe682 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0xa082 },
	{ CCI_REG16(0x6f12), 0xa888 },
	{ CCI_REG16(0x6f12), 0x2080 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x70fb },
	{ CCI_REG16(0x6f12), 0x0128 },
	{ CCI_REG16(0x6f12), 0x0cd1 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x71fb },
	{ CCI_REG16(0x6f12), 0x48b1 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x73fb },
	{ CCI_REG16(0x6f12), 0x30b1 },
	{ CCI_REG16(0x6f12), 0x40f2 },
	{ CCI_REG16(0x6f12), 0x1340 },
	{ CCI_REG16(0x6f12), 0xa081 },
	{ CCI_REG16(0x6f12), 0x40f2 },
	{ CCI_REG16(0x6f12), 0x0110 },
	{ CCI_REG16(0x6f12), 0xe081 },
	{ CCI_REG16(0x6f12), 0x2082 },
	{ CCI_REG16(0x6f12), 0x2b6a },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x8320 },
	{ CCI_REG16(0x6f12), 0x109a },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x6afb },
	{ CCI_REG16(0x6f12), 0xe881 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x58fb },
	{ CCI_REG16(0x6f12), 0x0126 },
	{ CCI_REG16(0x6f12), 0x0128 },
	{ CCI_REG16(0x6f12), 0x12d1 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x58fb },
	{ CCI_REG16(0x6f12), 0x78b1 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x5afb },
	{ CCI_REG16(0x6f12), 0x60b1 },
	{ CCI_REG16(0x6f12), 0x2680 },
	{ CCI_REG16(0x6f12), 0x3048 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x04e0 },
	{ CCI_REG16(0x6f12), 0x0288 },
	{ CCI_REG16(0x6f12), 0x5208 },
	{ CCI_REG16(0x6f12), 0x20f8 },
	{ CCI_REG16(0x6f12), 0x022b },
	{ CCI_REG16(0x6f12), 0x491c },
	{ CCI_REG16(0x6f12), 0xea89 },
	{ CCI_REG16(0x6f12), 0xb1eb },
	{ CCI_REG16(0x6f12), 0x420f },
	{ CCI_REG16(0x6f12), 0xf6db },
	{ CCI_REG16(0x6f12), 0xe989 },
	{ CCI_REG16(0x6f12), 0xa889 },
	{ CCI_REG16(0x6f12), 0x8142 },
	{ CCI_REG16(0x6f12), 0x00d9 },
	{ CCI_REG16(0x6f12), 0xe881 },
	{ CCI_REG16(0x6f12), 0x2680 },
	{ CCI_REG16(0x6f12), 0x07b0 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf08f },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf843 },
	{ CCI_REG16(0x6f12), 0x1a48 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x4069 },
	{ CCI_REG16(0x6f12), 0x85b2 },
	{ CCI_REG16(0x6f12), 0x4fea },
	{ CCI_REG16(0x6f12), 0x1048 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x4046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xfbfa },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x3ffb },
	{ CCI_REG16(0x6f12), 0x204f },
	{ CCI_REG16(0x6f12), 0x97f8 },
	{ CCI_REG16(0x6f12), 0x7300 },
	{ CCI_REG16(0x6f12), 0x30b1 },
	{ CCI_REG16(0x6f12), 0x1348 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x8b02 },
	{ CCI_REG16(0x6f12), 0x10b1 },
	{ CCI_REG16(0x6f12), 0x1d49 },
	{ CCI_REG16(0x6f12), 0x1b20 },
	{ CCI_REG16(0x6f12), 0x0880 },
	{ CCI_REG16(0x6f12), 0x1c48 },
	{ CCI_REG16(0x6f12), 0x0e4e },
	{ CCI_REG16(0x6f12), 0x3436 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0xc046 },
	{ CCI_REG16(0x6f12), 0xb089 },
	{ CCI_REG16(0x6f12), 0x98b9 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0xadf8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0a48 },
	{ CCI_REG16(0x6f12), 0x0222 },
	{ CCI_REG16(0x6f12), 0x6946 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0x0006 },
	{ CCI_REG16(0x6f12), 0x2e30 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x27fb },
	{ CCI_REG16(0x6f12), 0x10b1 },
	{ CCI_REG16(0x6f12), 0xbdf8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xb081 },
	{ CCI_REG16(0x6f12), 0xb089 },
	{ CCI_REG16(0x6f12), 0x10b9 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x8060 },
	{ CCI_REG16(0x6f12), 0xb081 },
	{ CCI_REG16(0x6f12), 0x97f8 },
	{ CCI_REG16(0x6f12), 0x7500 },
	{ CCI_REG16(0x6f12), 0x1de0 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x4a40 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2ed0 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x0e20 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0x8832 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x3420 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x21a0 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x3f40 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x3e70 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xa000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x38c0 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2210 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x8000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2850 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xf47e },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x6f12), 0x28b1 },
	{ CCI_REG16(0x6f12), 0xb089 },
	{ CCI_REG16(0x6f12), 0x18b1 },
	{ CCI_REG16(0x6f12), 0x6043 },
	{ CCI_REG16(0x6f12), 0x00f5 },
	{ CCI_REG16(0x6f12), 0x0070 },
	{ CCI_REG16(0x6f12), 0x840a },
	{ CCI_REG16(0x6f12), 0xfe48 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x8072 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0x7c07 },
	{ CCI_REG16(0x6f12), 0x9042 },
	{ CCI_REG16(0x6f12), 0x01d9 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x00e0 },
	{ CCI_REG16(0x6f12), 0x1146 },
	{ CCI_REG16(0x6f12), 0x8b01 },
	{ CCI_REG16(0x6f12), 0xa3f5 },
	{ CCI_REG16(0x6f12), 0x8043 },
	{ CCI_REG16(0x6f12), 0x9042 },
	{ CCI_REG16(0x6f12), 0x01d9 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x00e0 },
	{ CCI_REG16(0x6f12), 0x1146 },
	{ CCI_REG16(0x6f12), 0x01fb },
	{ CCI_REG16(0x6f12), 0x0431 },
	{ CCI_REG16(0x6f12), 0xff23 },
	{ CCI_REG16(0x6f12), 0xb3eb },
	{ CCI_REG16(0x6f12), 0x112f },
	{ CCI_REG16(0x6f12), 0x0ed9 },
	{ CCI_REG16(0x6f12), 0x9042 },
	{ CCI_REG16(0x6f12), 0x01d9 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x00e0 },
	{ CCI_REG16(0x6f12), 0x1146 },
	{ CCI_REG16(0x6f12), 0x8901 },
	{ CCI_REG16(0x6f12), 0xa1f5 },
	{ CCI_REG16(0x6f12), 0x8041 },
	{ CCI_REG16(0x6f12), 0x9042 },
	{ CCI_REG16(0x6f12), 0x00d8 },
	{ CCI_REG16(0x6f12), 0x1046 },
	{ CCI_REG16(0x6f12), 0x00fb },
	{ CCI_REG16(0x6f12), 0x0410 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x00e0 },
	{ CCI_REG16(0x6f12), 0xff20 },
	{ CCI_REG16(0x6f12), 0xeb49 },
	{ CCI_REG16(0x6f12), 0x0880 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x4046 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf843 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x7aba },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0xe748 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x8169 },
	{ CCI_REG16(0x6f12), 0x0c0c },
	{ CCI_REG16(0x6f12), 0x8db2 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x70fa },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xbefa },
	{ CCI_REG16(0x6f12), 0xe248 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x7410 },
	{ CCI_REG16(0x6f12), 0x11b1 },
	{ CCI_REG16(0x6f12), 0x0021 },
	{ CCI_REG16(0x6f12), 0x80f8 },
	{ CCI_REG16(0x6f12), 0x7010 },
	{ CCI_REG16(0x6f12), 0xe048 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x8071 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x6f20 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x3040 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x5efa },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0x7040 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x57ba },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0x0446 },
	{ CCI_REG16(0x6f12), 0xd648 },
	{ CCI_REG16(0x6f12), 0xd74d },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x0804 },
	{ CCI_REG16(0x6f12), 0xc8b1 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x0906 },
	{ CCI_REG16(0x6f12), 0xa8b1 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0xd5f8 },
	{ CCI_REG16(0x6f12), 0x8423 },
	{ CCI_REG16(0x6f12), 0xc0f8 },
	{ CCI_REG16(0x6f12), 0x1424 },
	{ CCI_REG16(0x6f12), 0x00f2 },
	{ CCI_REG16(0x6f12), 0x1441 },
	{ CCI_REG16(0x6f12), 0x2a46 },
	{ CCI_REG16(0x6f12), 0xd5f8 },
	{ CCI_REG16(0x6f12), 0x9003 },
	{ CCI_REG16(0x6f12), 0xc2f8 },
	{ CCI_REG16(0x6f12), 0x2004 },
	{ CCI_REG16(0x6f12), 0xd5f8 },
	{ CCI_REG16(0x6f12), 0xc043 },
	{ CCI_REG16(0x6f12), 0x1046 },
	{ CCI_REG16(0x6f12), 0xc5f8 },
	{ CCI_REG16(0x6f12), 0xe442 },
	{ CCI_REG16(0x6f12), 0xc0f8 },
	{ CCI_REG16(0x6f12), 0x3044 },
	{ CCI_REG16(0x6f12), 0x0846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x8bfa },
	{ CCI_REG16(0x6f12), 0xc749 },
	{ CCI_REG16(0x6f12), 0xb5f8 },
	{ CCI_REG16(0x6f12), 0xb022 },
	{ CCI_REG16(0x6f12), 0x088f },
	{ CCI_REG16(0x6f12), 0x498f },
	{ CCI_REG16(0x6f12), 0x201a },
	{ CCI_REG16(0x6f12), 0x401e },
	{ CCI_REG16(0x6f12), 0x1144 },
	{ CCI_REG16(0x6f12), 0x8142 },
	{ CCI_REG16(0x6f12), 0x00d9 },
	{ CCI_REG16(0x6f12), 0x0846 },
	{ CCI_REG16(0x6f12), 0xa5f8 },
	{ CCI_REG16(0x6f12), 0xb202 },
	{ CCI_REG16(0x6f12), 0x70bd },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf041 },
	{ CCI_REG16(0x6f12), 0x0646 },
	{ CCI_REG16(0x6f12), 0xbd48 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x006a },
	{ CCI_REG16(0x6f12), 0x85b2 },
	{ CCI_REG16(0x6f12), 0x040c },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x1cfa },
	{ CCI_REG16(0x6f12), 0x3046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x73fa },
	{ CCI_REG16(0x6f12), 0xbb48 },
	{ CCI_REG16(0x6f12), 0xbb4f },
	{ CCI_REG16(0x6f12), 0x0068 },
	{ CCI_REG16(0x6f12), 0x3b68 },
	{ CCI_REG16(0x6f12), 0x418b },
	{ CCI_REG16(0x6f12), 0x090a },
	{ CCI_REG16(0x6f12), 0x83f8 },
	{ CCI_REG16(0x6f12), 0x3610 },
	{ CCI_REG16(0x6f12), 0xc17e },
	{ CCI_REG16(0x6f12), 0x83f8 },
	{ CCI_REG16(0x6f12), 0x3810 },
	{ CCI_REG16(0x6f12), 0xb449 },
	{ CCI_REG16(0x6f12), 0x91f8 },
	{ CCI_REG16(0x6f12), 0x4c21 },
	{ CCI_REG16(0x6f12), 0x002a },
	{ CCI_REG16(0x6f12), 0xd1f8 },
	{ CCI_REG16(0x6f12), 0x3421 },
	{ CCI_REG16(0x6f12), 0x01d0 },
	{ CCI_REG16(0x6f12), 0x521c },
	{ CCI_REG16(0x6f12), 0x5208 },
	{ CCI_REG16(0x6f12), 0xce33 },
	{ CCI_REG16(0x6f12), 0x160a },
	{ CCI_REG16(0x6f12), 0x1e71 },
	{ CCI_REG16(0x6f12), 0x9a71 },
	{ CCI_REG16(0x6f12), 0xb1f8 },
	{ CCI_REG16(0x6f12), 0x3c21 },
	{ CCI_REG16(0x6f12), 0xc2f3 },
	{ CCI_REG16(0x6f12), 0x5712 },
	{ CCI_REG16(0x6f12), 0x1a70 },
	{ CCI_REG16(0x6f12), 0x91f8 },
	{ CCI_REG16(0x6f12), 0x3d21 },
	{ CCI_REG16(0x6f12), 0xd200 },
	{ CCI_REG16(0x6f12), 0x9a70 },
	{ CCI_REG16(0x6f12), 0x91f8 },
	{ CCI_REG16(0x6f12), 0x4d21 },
	{ CCI_REG16(0x6f12), 0xce3b },
	{ CCI_REG16(0x6f12), 0x22b1 },
	{ CCI_REG16(0x6f12), 0xd1f8 },
	{ CCI_REG16(0x6f12), 0x3821 },
	{ CCI_REG16(0x6f12), 0x521c },
	{ CCI_REG16(0x6f12), 0x5608 },
	{ CCI_REG16(0x6f12), 0x01e0 },
	{ CCI_REG16(0x6f12), 0xd1f8 },
	{ CCI_REG16(0x6f12), 0x3861 },
	{ CCI_REG16(0x6f12), 0x7a68 },
	{ CCI_REG16(0x6f12), 0x4fea },
	{ CCI_REG16(0x6f12), 0x162c },
	{ CCI_REG16(0x6f12), 0x01f5 },
	{ CCI_REG16(0x6f12), 0x9071 },
	{ CCI_REG16(0x6f12), 0x82f8 },
	{ CCI_REG16(0x6f12), 0x16c0 },
	{ CCI_REG16(0x6f12), 0x1676 },
	{ CCI_REG16(0x6f12), 0xce8b },
	{ CCI_REG16(0x6f12), 0x00f5 },
	{ CCI_REG16(0x6f12), 0xba70 },
	{ CCI_REG16(0x6f12), 0xc6f3 },
	{ CCI_REG16(0x6f12), 0x5716 },
	{ CCI_REG16(0x6f12), 0x9674 },
	{ CCI_REG16(0x6f12), 0xce7f },
	{ CCI_REG16(0x6f12), 0xf600 },
	{ CCI_REG16(0x6f12), 0x1675 },
	{ CCI_REG16(0x6f12), 0x0e8c },
	{ CCI_REG16(0x6f12), 0xcf68 },
	{ CCI_REG16(0x6f12), 0xf608 },
	{ CCI_REG16(0x6f12), 0x7e43 },
	{ CCI_REG16(0x6f12), 0x360b },
	{ CCI_REG16(0x6f12), 0x370a },
	{ CCI_REG16(0x6f12), 0x03f8 },
	{ CCI_REG16(0x6f12), 0xd67f },
	{ CCI_REG16(0x6f12), 0x7732 },
	{ CCI_REG16(0x6f12), 0x9e70 },
	{ CCI_REG16(0x6f12), 0x0688 },
	{ CCI_REG16(0x6f12), 0x360a },
	{ CCI_REG16(0x6f12), 0x02f8 },
	{ CCI_REG16(0x6f12), 0x276c },
	{ CCI_REG16(0x6f12), 0x4678 },
	{ CCI_REG16(0x6f12), 0x02f8 },
	{ CCI_REG16(0x6f12), 0x256c },
	{ CCI_REG16(0x6f12), 0x8688 },
	{ CCI_REG16(0x6f12), 0x360a },
	{ CCI_REG16(0x6f12), 0x02f8 },
	{ CCI_REG16(0x6f12), 0x1f6c },
	{ CCI_REG16(0x6f12), 0x4679 },
	{ CCI_REG16(0x6f12), 0x02f8 },
	{ CCI_REG16(0x6f12), 0x1d6c },
	{ CCI_REG16(0x6f12), 0x8f4e },
	{ CCI_REG16(0x6f12), 0x96f8 },
	{ CCI_REG16(0x6f12), 0x1064 },
	{ CCI_REG16(0x6f12), 0xd671 },
	{ CCI_REG16(0x6f12), 0x8d4e },
	{ CCI_REG16(0x6f12), 0x96f8 },
	{ CCI_REG16(0x6f12), 0x1164 },
	{ CCI_REG16(0x6f12), 0x5672 },
	{ CCI_REG16(0x6f12), 0x8b4e },
	{ CCI_REG16(0x6f12), 0x96f8 },
	{ CCI_REG16(0x6f12), 0x0b64 },
	{ CCI_REG16(0x6f12), 0xd672 },
	{ CCI_REG16(0x6f12), 0x894e },
	{ CCI_REG16(0x6f12), 0x96f8 },
	{ CCI_REG16(0x6f12), 0x0964 },
	{ CCI_REG16(0x6f12), 0x5673 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x3060 },
	{ CCI_REG16(0x6f12), 0xd673 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0xde00 },
	{ CCI_REG16(0x6f12), 0x02f8 },
	{ CCI_REG16(0x6f12), 0x1f0f },
	{ CCI_REG16(0x6f12), 0x8448 },
	{ CCI_REG16(0x6f12), 0x00f2 },
	{ CCI_REG16(0x6f12), 0x7246 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x7204 },
	{ CCI_REG16(0x6f12), 0x9074 },
	{ CCI_REG16(0x6f12), 0x3078 },
	{ CCI_REG16(0x6f12), 0x1075 },
	{ CCI_REG16(0x6f12), 0xa522 },
	{ CCI_REG16(0x6f12), 0xda70 },
	{ CCI_REG16(0x6f12), 0x0e20 },
	{ CCI_REG16(0x6f12), 0x1871 },
	{ CCI_REG16(0x6f12), 0x11f8 },
	{ CCI_REG16(0x6f12), 0x7e0c },
	{ CCI_REG16(0x6f12), 0xc0f1 },
	{ CCI_REG16(0x6f12), 0x0c01 },
	{ CCI_REG16(0x6f12), 0x7c48 },
	{ CCI_REG16(0x6f12), 0xd0f8 },
	{ CCI_REG16(0x6f12), 0x3c04 },
	{ CCI_REG16(0x6f12), 0xc840 },
	{ CCI_REG16(0x6f12), 0x060a },
	{ CCI_REG16(0x6f12), 0x9e71 },
	{ CCI_REG16(0x6f12), 0x1872 },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x6f12), 0x03f8 },
	{ CCI_REG16(0x6f12), 0x2c0c },
	{ CCI_REG16(0x6f12), 0x7748 },
	{ CCI_REG16(0x6f12), 0xd0f8 },
	{ CCI_REG16(0x6f12), 0x4c04 },
	{ CCI_REG16(0x6f12), 0xc840 },
	{ CCI_REG16(0x6f12), 0xaa21 },
	{ CCI_REG16(0x6f12), 0x03f8 },
	{ CCI_REG16(0x6f12), 0x571d },
	{ CCI_REG16(0x6f12), 0x0226 },
	{ CCI_REG16(0x6f12), 0x5e70 },
	{ CCI_REG16(0x6f12), 0x9a70 },
	{ CCI_REG16(0x6f12), 0x3022 },
	{ CCI_REG16(0x6f12), 0xda70 },
	{ CCI_REG16(0x6f12), 0x5a22 },
	{ CCI_REG16(0x6f12), 0x1a71 },
	{ CCI_REG16(0x6f12), 0x060a },
	{ CCI_REG16(0x6f12), 0x5e71 },
	{ CCI_REG16(0x6f12), 0x9a71 },
	{ CCI_REG16(0x6f12), 0xd871 },
	{ CCI_REG16(0x6f12), 0x1972 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x5872 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf041 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x77b9 },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf041 },
	{ CCI_REG16(0x6f12), 0x0746 },
	{ CCI_REG16(0x6f12), 0x6448 },
	{ CCI_REG16(0x6f12), 0x0c46 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x406a },
	{ CCI_REG16(0x6f12), 0x86b2 },
	{ CCI_REG16(0x6f12), 0x050c },
	{ CCI_REG16(0x6f12), 0x3146 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x6af9 },
	{ CCI_REG16(0x6f12), 0x2146 },
	{ CCI_REG16(0x6f12), 0x3846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc5f9 },
	{ CCI_REG16(0x6f12), 0x6048 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0x9702 },
	{ CCI_REG16(0x6f12), 0x10b9 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x97f9 },
	{ CCI_REG16(0x6f12), 0x20b1 },
	{ CCI_REG16(0x6f12), 0x04f1 },
	{ CCI_REG16(0x6f12), 0x8044 },
	{ CCI_REG16(0x6f12), 0xa08a },
	{ CCI_REG16(0x6f12), 0x401c },
	{ CCI_REG16(0x6f12), 0xa082 },
	{ CCI_REG16(0x6f12), 0x3146 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf041 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x53b9 },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf041 },
	{ CCI_REG16(0x6f12), 0x0746 },
	{ CCI_REG16(0x6f12), 0x5248 },
	{ CCI_REG16(0x6f12), 0x0e46 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x806a },
	{ CCI_REG16(0x6f12), 0x85b2 },
	{ CCI_REG16(0x6f12), 0x040c },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x46f9 },
	{ CCI_REG16(0x6f12), 0x3146 },
	{ CCI_REG16(0x6f12), 0x3846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xa6f9 },
	{ CCI_REG16(0x6f12), 0x4b4f },
	{ CCI_REG16(0x6f12), 0x4df2 },
	{ CCI_REG16(0x6f12), 0x0c26 },
	{ CCI_REG16(0x6f12), 0x3437 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x8061 },
	{ CCI_REG16(0x6f12), 0x3a78 },
	{ CCI_REG16(0x6f12), 0x3046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x38f9 },
	{ CCI_REG16(0x6f12), 0x7878 },
	{ CCI_REG16(0x6f12), 0xc8b3 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x0071 },
	{ CCI_REG16(0x6f12), 0x3046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x30f9 },
	{ CCI_REG16(0x6f12), 0x4848 },
	{ CCI_REG16(0x6f12), 0x0088 },
	{ CCI_REG16(0x6f12), 0x484b },
	{ CCI_REG16(0x6f12), 0xa3f8 },
	{ CCI_REG16(0x6f12), 0x4402 },
	{ CCI_REG16(0x6f12), 0x4648 },
	{ CCI_REG16(0x6f12), 0x001d },
	{ CCI_REG16(0x6f12), 0x0088 },
	{ CCI_REG16(0x6f12), 0xa3f8 },
	{ CCI_REG16(0x6f12), 0x4602 },
	{ CCI_REG16(0x6f12), 0xb3f8 },
	{ CCI_REG16(0x6f12), 0x4402 },
	{ CCI_REG16(0x6f12), 0xb3f8 },
	{ CCI_REG16(0x6f12), 0x4612 },
	{ CCI_REG16(0x6f12), 0x4218 },
	{ CCI_REG16(0x6f12), 0x02d0 },
	{ CCI_REG16(0x6f12), 0x8002 },
	{ CCI_REG16(0x6f12), 0xb0fb },
	{ CCI_REG16(0x6f12), 0xf2f2 },
	{ CCI_REG16(0x6f12), 0x91b2 },
	{ CCI_REG16(0x6f12), 0x404a },
	{ CCI_REG16(0x6f12), 0xa3f8 },
	{ CCI_REG16(0x6f12), 0x4812 },
	{ CCI_REG16(0x6f12), 0x5088 },
	{ CCI_REG16(0x6f12), 0x1288 },
	{ CCI_REG16(0x6f12), 0x3d4b },
	{ CCI_REG16(0x6f12), 0xa3f8 },
	{ CCI_REG16(0x6f12), 0xa605 },
	{ CCI_REG16(0x6f12), 0xa3f8 },
	{ CCI_REG16(0x6f12), 0xa825 },
	{ CCI_REG16(0x6f12), 0x8018 },
	{ CCI_REG16(0x6f12), 0x05d0 },
	{ CCI_REG16(0x6f12), 0x9202 },
	{ CCI_REG16(0x6f12), 0xb2fb },
	{ CCI_REG16(0x6f12), 0xf0f0 },
	{ CCI_REG16(0x6f12), 0x1a46 },
	{ CCI_REG16(0x6f12), 0xa2f8 },
	{ CCI_REG16(0x6f12), 0xaa05 },
	{ CCI_REG16(0x6f12), 0x3648 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0xaa05 },
	{ CCI_REG16(0x6f12), 0x0a18 },
	{ CCI_REG16(0x6f12), 0x01fb },
	{ CCI_REG16(0x6f12), 0x1020 },
	{ CCI_REG16(0x6f12), 0x40f3 },
	{ CCI_REG16(0x6f12), 0x9510 },
	{ CCI_REG16(0x6f12), 0x1028 },
	{ CCI_REG16(0x6f12), 0x06dc },
	{ CCI_REG16(0x6f12), 0x0028 },
	{ CCI_REG16(0x6f12), 0x05da },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x03e0 },
	{ CCI_REG16(0x6f12), 0xffe7 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0xc3e7 },
	{ CCI_REG16(0x6f12), 0x1020 },
	{ CCI_REG16(0x6f12), 0x2f49 },
	{ CCI_REG16(0x6f12), 0x0880 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf041 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xefb8 },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0x2148 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xc16a },
	{ CCI_REG16(0x6f12), 0x0c0c },
	{ CCI_REG16(0x6f12), 0x8db2 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xe5f8 },
	{ CCI_REG16(0x6f12), 0x2148 },
	{ CCI_REG16(0x6f12), 0x0268 },
	{ CCI_REG16(0x6f12), 0xb2f8 },
	{ CCI_REG16(0x6f12), 0x6202 },
	{ CCI_REG16(0x6f12), 0x8301 },
	{ CCI_REG16(0x6f12), 0x92f8 },
	{ CCI_REG16(0x6f12), 0x6002 },
	{ CCI_REG16(0x6f12), 0x10f0 },
	{ CCI_REG16(0x6f12), 0x020f },
	{ CCI_REG16(0x6f12), 0x09d0 },
	{ CCI_REG16(0x6f12), 0x1848 },
	{ CCI_REG16(0x6f12), 0x3430 },
	{ CCI_REG16(0x6f12), 0x8188 },
	{ CCI_REG16(0x6f12), 0x9942 },
	{ CCI_REG16(0x6f12), 0x06d8 },
	{ CCI_REG16(0x6f12), 0x4088 },
	{ CCI_REG16(0x6f12), 0xa0f5 },
	{ CCI_REG16(0x6f12), 0x5141 },
	{ CCI_REG16(0x6f12), 0x2339 },
	{ CCI_REG16(0x6f12), 0x01d1 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x38f9 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0x7040 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc8b8 },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0x0646 },
	{ CCI_REG16(0x6f12), 0x0d48 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x016b },
	{ CCI_REG16(0x6f12), 0x0c0c },
	{ CCI_REG16(0x6f12), 0x8db2 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xbdf8 },
	{ CCI_REG16(0x6f12), 0x3046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x28f9 },
	{ CCI_REG16(0x6f12), 0x0749 },
	{ CCI_REG16(0x6f12), 0x114a },
	{ CCI_REG16(0x6f12), 0x3431 },
	{ CCI_REG16(0x6f12), 0xcb79 },
	{ CCI_REG16(0x6f12), 0xd068 },
	{ CCI_REG16(0x6f12), 0x9840 },
	{ CCI_REG16(0x6f12), 0xd060 },
	{ CCI_REG16(0x6f12), 0x1068 },
	{ CCI_REG16(0x6f12), 0x9840 },
	{ CCI_REG16(0x6f12), 0x1060 },
	{ CCI_REG16(0x6f12), 0x8868 },
	{ CCI_REG16(0x6f12), 0x19e0 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xf474 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x4a40 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2850 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x0e20 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2ed0 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x08d0 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x36e0 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0x9404 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x38c0 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xd214 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xa410 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x3254 },
	{ CCI_REG16(0x6f12), 0xd063 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0x7040 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x8cb8 },
	{ CCI_REG16(0x6f12), 0x10b5 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff6 },
	{ CCI_REG16(0x6f12), 0x9701 },
	{ CCI_REG16(0x6f12), 0x3348 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xf8f8 },
	{ CCI_REG16(0x6f12), 0x334c },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff6 },
	{ CCI_REG16(0x6f12), 0x3f01 },
	{ CCI_REG16(0x6f12), 0x2060 },
	{ CCI_REG16(0x6f12), 0x3148 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xf0f8 },
	{ CCI_REG16(0x6f12), 0x6060 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x4970 },
	{ CCI_REG16(0x6f12), 0x2f49 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xc861 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x9f61 },
	{ CCI_REG16(0x6f12), 0x2e48 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xe5f8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x2d51 },
	{ CCI_REG16(0x6f12), 0x2061 },
	{ CCI_REG16(0x6f12), 0x2b48 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xdef8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x2341 },
	{ CCI_REG16(0x6f12), 0x6061 },
	{ CCI_REG16(0x6f12), 0x2948 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xd7f8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0xe931 },
	{ CCI_REG16(0x6f12), 0xa061 },
	{ CCI_REG16(0x6f12), 0x2648 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xd0f8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x6b71 },
	{ CCI_REG16(0x6f12), 0xe061 },
	{ CCI_REG16(0x6f12), 0x2448 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc9f8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x2371 },
	{ CCI_REG16(0x6f12), 0xa060 },
	{ CCI_REG16(0x6f12), 0x2148 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc2f8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0xb731 },
	{ CCI_REG16(0x6f12), 0xe060 },
	{ CCI_REG16(0x6f12), 0x1f48 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xbbf8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x6121 },
	{ CCI_REG16(0x6f12), 0x2062 },
	{ CCI_REG16(0x6f12), 0x1c48 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xb4f8 },
	{ CCI_REG16(0x6f12), 0x6062 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x04f1 },
	{ CCI_REG16(0x6f12), 0x3401 },
	{ CCI_REG16(0x6f12), 0x0246 },
	{ CCI_REG16(0x6f12), 0x8881 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x3121 },
	{ CCI_REG16(0x6f12), 0x1848 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xa9f8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x7511 },
	{ CCI_REG16(0x6f12), 0xa062 },
	{ CCI_REG16(0x6f12), 0x1548 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xa2f8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x3711 },
	{ CCI_REG16(0x6f12), 0xe062 },
	{ CCI_REG16(0x6f12), 0x1348 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x9bf8 },
	{ CCI_REG16(0x6f12), 0x1249 },
	{ CCI_REG16(0x6f12), 0x2063 },
	{ CCI_REG16(0x6f12), 0x40f6 },
	{ CCI_REG16(0x6f12), 0xf100 },
	{ CCI_REG16(0x6f12), 0x0968 },
	{ CCI_REG16(0x6f12), 0x4883 },
	{ CCI_REG16(0x6f12), 0x10bd },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xde1f },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x4a40 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x5f3b },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x0850 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xd719 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x27ff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x39e3 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x32cf },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x1e3b },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xec45 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x67b9 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xe62b },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x2265 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x8c83 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x5449 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x08d0 },
	{ CCI_REG16(0x6f12), 0x4af2 },
	{ CCI_REG16(0x6f12), 0x2b1c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4df6 },
	{ CCI_REG16(0x6f12), 0x1f6c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x44f6 },
	{ CCI_REG16(0x6f12), 0x655c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x45f6 },
	{ CCI_REG16(0x6f12), 0x433c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x45f6 },
	{ CCI_REG16(0x6f12), 0xe36c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x46f2 },
	{ CCI_REG16(0x6f12), 0x7b1c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x44f6 },
	{ CCI_REG16(0x6f12), 0xd90c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x44f6 },
	{ CCI_REG16(0x6f12), 0x791c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x44f6 },
	{ CCI_REG16(0x6f12), 0x811c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x44f2 },
	{ CCI_REG16(0x6f12), 0xb50c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x44f6 },
	{ CCI_REG16(0x6f12), 0xe90c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x43f6 },
	{ CCI_REG16(0x6f12), 0x155c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x43f6 },
	{ CCI_REG16(0x6f12), 0x1d5c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4df2 },
	{ CCI_REG16(0x6f12), 0xc95c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x42f2 },
	{ CCI_REG16(0x6f12), 0xff7c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x48f2 },
	{ CCI_REG16(0x6f12), 0x712c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x43f6 },
	{ CCI_REG16(0x6f12), 0xe31c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x43f2 },
	{ CCI_REG16(0x6f12), 0x374c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x46f2 },
	{ CCI_REG16(0x6f12), 0xb97c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4ef2 },
	{ CCI_REG16(0x6f12), 0x2b6c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x42f2 },
	{ CCI_REG16(0x6f12), 0x652c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x48f6 },
	{ CCI_REG16(0x6f12), 0x834c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x45f2 },
	{ CCI_REG16(0x6f12), 0x494c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4cf2 },
	{ CCI_REG16(0x6f12), 0x2d1c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x16f0 },
	{ CCI_REG16(0x6f12), 0x2929 },
	{ CCI_REG16(0x602a), 0x16f2 },
	{ CCI_REG16(0x6f12), 0x2929 },
	{ CCI_REG16(0x602a), 0x16fa },
	{ CCI_REG16(0x6f12), 0x0029 },
	{ CCI_REG16(0x602a), 0x16fc },
	{ CCI_REG16(0x6f12), 0x0029 },
	{ CCI_REG16(0x602a), 0x1708 },
	{ CCI_REG16(0x6f12), 0x0029 },
	{ CCI_REG16(0x602a), 0x170a },
	{ CCI_REG16(0x6f12), 0x0029 },
	{ CCI_REG16(0x602a), 0x1712 },
	{ CCI_REG16(0x6f12), 0x2929 },
	{ CCI_REG16(0x602a), 0x1714 },
	{ CCI_REG16(0x6f12), 0x2929 },
	{ CCI_REG16(0x602a), 0x1716 },
	{ CCI_REG16(0x6f12), 0x2929 },
	{ CCI_REG16(0x602a), 0x1722 },
	{ CCI_REG16(0x6f12), 0x152a },
	{ CCI_REG16(0x602a), 0x1724 },
	{ CCI_REG16(0x6f12), 0x152a },
	{ CCI_REG16(0x602a), 0x172c },
	{ CCI_REG16(0x6f12), 0x002a },
	{ CCI_REG16(0x602a), 0x172e },
	{ CCI_REG16(0x6f12), 0x002a },
	{ CCI_REG16(0x602a), 0x1736 },
	{ CCI_REG16(0x6f12), 0x1500 },
	{ CCI_REG16(0x602a), 0x1738 },
	{ CCI_REG16(0x6f12), 0x1500 },
	{ CCI_REG16(0x602a), 0x1740 },
	{ CCI_REG16(0x6f12), 0x152a },
	{ CCI_REG16(0x602a), 0x1742 },
	{ CCI_REG16(0x6f12), 0x152a },
	{ CCI_REG16(0x602a), 0x16be },
	{ CCI_REG16(0x6f12), 0x1515 },
	{ CCI_REG16(0x6f12), 0x1515 },
	{ CCI_REG16(0x602a), 0x16c8 },
	{ CCI_REG16(0x6f12), 0x0029 },
	{ CCI_REG16(0x6f12), 0x0029 },
	{ CCI_REG16(0x602a), 0x16d6 },
	{ CCI_REG16(0x6f12), 0x0015 },
	{ CCI_REG16(0x6f12), 0x0015 },
	{ CCI_REG16(0x602a), 0x16e0 },
	{ CCI_REG16(0x6f12), 0x2929 },
	{ CCI_REG16(0x6f12), 0x2929 },
	{ CCI_REG16(0x6f12), 0x2929 },
	{ CCI_REG16(0x602a), 0x19b8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2224 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0df8 },
	{ CCI_REG16(0x6f12), 0x1001 },
	{ CCI_REG16(0x602a), 0x1eda },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x602a), 0x16a0 },
	{ CCI_REG16(0x6f12), 0x3d09 },
	{ CCI_REG16(0x602a), 0x10a8 },
	{ CCI_REG16(0x6f12), 0x000e },
	{ CCI_REG16(0x602a), 0x1198 },
	{ CCI_REG16(0x6f12), 0x002b },
	{ CCI_REG16(0x602a), 0x1002 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0f70 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x6f12), 0x002f },
	{ CCI_REG16(0x6f12), 0x007f },
	{ CCI_REG16(0x6f12), 0x0030 },
	{ CCI_REG16(0x6f12), 0x0080 },
	{ CCI_REG16(0x6f12), 0x000b },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x6f12), 0xf46e },
	{ CCI_REG16(0x602a), 0x0faa },
	{ CCI_REG16(0x6f12), 0x000d },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x6f12), 0xf464 },
	{ CCI_REG16(0x602a), 0x1698 },
	{ CCI_REG16(0x6f12), 0x0d05 },
	{ CCI_REG16(0x602a), 0x20a0 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0203 },
	{ CCI_REG16(0x602a), 0x4a74 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x1f80 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0ff4 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x1800 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0fea), 0x1440 },
	{ CCI_REG16(0x0b06), 0x0101 },
	{ CCI_REG16(0xf44a), 0x0007 },
	{ CCI_REG16(0xf456), 0x000a },
	{ CCI_REG16(0xf46a), 0xbfa0 },
	{ CCI_REG16(0x0d80), 0x1388 },
	{ CCI_REG16(0xb134), 0x0000 },
	{ CCI_REG16(0xb136), 0x0000 },
	{ CCI_REG16(0xb138), 0x0000 },
};

static const struct cci_reg_sequence s5k3p9_2304x1728_mode[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6214), 0x7970 },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x0ed6 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1cf0 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x0e58 },
	{ CCI_REG16(0x6f12), 0x0023 },
	{ CCI_REG16(0x602a), 0x1694 },
	{ CCI_REG16(0x6f12), 0x170f },
	{ CCI_REG16(0x602a), 0x16aa },
	{ CCI_REG16(0x6f12), 0x009d },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x602a), 0x1098 },
	{ CCI_REG16(0x6f12), 0x0012 },
	{ CCI_REG16(0x602a), 0x2690 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x16a8 },
	{ CCI_REG16(0x6f12), 0x38c0 },
	{ CCI_REG16(0x602a), 0x108c },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x602a), 0x10cc },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x10d0 },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x602a), 0x0f50 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x1758 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0344), 0x0010 },
	{ CCI_REG16(0x0346), 0x0018 },
	{ CCI_REG16(0x0348), 0x121f },
	{ CCI_REG16(0x034a), 0x0d97 },
	{ CCI_REG16(0x034c), 0x0900 },
	{ CCI_REG16(0x034e), 0x06c0 },
	{ CCI_REG16(0x0350), 0x0004 },
	{ CCI_REG16(0x0900), 0x0122 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 },
	{ CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x0404), 0x1000 },
	{ CCI_REG16(0x0402), 0x1010 },
	{ CCI_REG16(0x0400), 0x1010 },
	{ CCI_REG16(0x0114), 0x0300 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0136), 0x1333 },
	{ CCI_REG16(0x0300), 0x0007 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0005 },
	{ CCI_REG16(0x0306), 0x00ff },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0003 },
	{ CCI_REG16(0x0310), 0x0058 },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x16a6 },
	{ CCI_REG16(0x6f12), 0x006c },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0340), 0x0bc0 },
	{ CCI_REG16(0x0342), 0x1800 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x0d00), 0x0000 },
	{ CCI_REG16(0x0d02), 0x0001 },
};

static const struct s5k3p9_mode s5k3p9_supported_modes[] = {
	{
		/* 2x2 binned full field of view, 30.27 fps */
		.width = 2304,
		.height = 1728,
		.hts = 6144,
		.vts = 3008,
		.exposure = 0x0100,
		.reg_list = {
			.regs = s5k3p9_2304x1728_mode,
			.num_regs = ARRAY_SIZE(s5k3p9_2304x1728_mode),
		},
	},
};

static int s5k3p9_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k3p9 *s5k3p9 = container_of(ctrl->handler, struct s5k3p9,
					     ctrl_handler);
	const struct s5k3p9_mode *mode = s5k3p9->mode;
	s64 exposure_max;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/* The orientation settings are applied along with streaming */
		return 0;
	case V4L2_CID_VBLANK:
		exposure_max = mode->height + ctrl->val - S5K3P9_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(s5k3p9->exposure,
					 s5k3p9->exposure->minimum,
					 exposure_max,
					 s5k3p9->exposure->step,
					 s5k3p9->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_active(s5k3p9->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k3p9->regmap, S5K3P9_REG_AGAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k3p9->regmap, S5K3P9_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k3p9->regmap, S5K3P9_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k3p9->regmap, S5K3P9_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k3p9->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k3p9_ctrl_ops = {
	.s_ctrl = s5k3p9_set_ctrl,
};

static int s5k3p9_init_controls(struct s5k3p9 *s5k3p9)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k3p9->ctrl_handler;
	const struct s5k3p9_mode *mode = s5k3p9->mode;
	s64 hblank, vblank, exposure_max;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 11);

	s5k3p9->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &s5k3p9_ctrl_ops,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(s5k3p9_link_freq_menu) - 1,
					0, s5k3p9_link_freq_menu);
	if (s5k3p9->link_freq)
		s5k3p9->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k3p9->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       S5K3P9_PIXEL_RATE,
					       S5K3P9_PIXEL_RATE, 1,
					       S5K3P9_PIXEL_RATE);

	hblank = mode->hts - mode->width;
	s5k3p9->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9_ctrl_ops,
					   V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (s5k3p9->hblank)
		s5k3p9->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	s5k3p9->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   S5K3P9_VTS_MAX - mode->height, 1,
					   vblank);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K3P9_AGAIN_MIN, S5K3P9_AGAIN_MAX,
			  S5K3P9_AGAIN_STEP, S5K3P9_AGAIN_DEFAULT);

	exposure_max = mode->vts - S5K3P9_EXPOSURE_MARGIN;
	s5k3p9->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K3P9_EXPOSURE_MIN,
					     exposure_max,
					     S5K3P9_EXPOSURE_STEP,
					     mode->exposure);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k3p9_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k3p9_test_pattern_menu) - 1,
				     0, 0, s5k3p9_test_pattern_menu);

	s5k3p9->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5k3p9->hflip)
		s5k3p9->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5k3p9->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3p9_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5k3p9->vflip)
		s5k3p9->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	ret = v4l2_fwnode_device_parse(s5k3p9->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k3p9_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	s5k3p9->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int s5k3p9_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k3p9 *s5k3p9 = to_s5k3p9(sd);
	const struct s5k3p9_reg_list *reg_list = &s5k3p9->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5k3p9->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(s5k3p9->regmap, s5k3p9_init_0,
				  ARRAY_SIZE(s5k3p9_init_0), NULL);
	if (ret)
		goto error;

	usleep_range(3 * USEC_PER_MSEC, 4 * USEC_PER_MSEC);

	cci_multi_reg_write(s5k3p9->regmap, s5k3p9_init_1,
			    ARRAY_SIZE(s5k3p9_init_1), &ret);
	cci_multi_reg_write(s5k3p9->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(s5k3p9->sd.ctrl_handler);

	cci_write(s5k3p9->regmap, S5K3P9_REG_CTRL_MODE,
		  S5K3P9_MODE_STREAMING |
		  (s5k3p9->vflip->val ? S5K3P9_VFLIP : 0) |
		  (s5k3p9->hflip->val ? S5K3P9_HFLIP : 0), &ret);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(s5k3p9->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5k3p9->dev);

	return ret;
}

static int s5k3p9_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k3p9 *s5k3p9 = to_s5k3p9(sd);
	int ret;

	ret = cci_write(s5k3p9->regmap, S5K3P9_REG_CTRL_MODE, 0, NULL);
	if (ret)
		dev_err(s5k3p9->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(s5k3p9->dev);

	return ret;
}

static u32 s5k3p9_get_format_code(struct s5k3p9 *s5k3p9)
{
	unsigned int i;

	i = (s5k3p9->vflip->val ? 2 : 0) | (s5k3p9->hflip->val ? 1 : 0);

	return s5k3p9_mbus_formats[i];
}

static void s5k3p9_update_pad_format(struct s5k3p9 *s5k3p9,
				     const struct s5k3p9_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5k3p9_get_format_code(s5k3p9);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k3p9_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k3p9 *s5k3p9 = to_s5k3p9(sd);
	s64 hblank, vblank, exposure_max;
	const struct s5k3p9_mode *mode;

	mode = v4l2_find_nearest_size(s5k3p9_supported_modes,
				      ARRAY_SIZE(s5k3p9_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5k3p9_update_pad_format(s5k3p9, mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5k3p9->mode == mode)
		goto set_format;

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5k3p9->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5k3p9->vblank, vblank,
				 S5K3P9_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5k3p9->vblank, vblank);

	exposure_max = mode->vts - S5K3P9_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(s5k3p9->exposure, S5K3P9_EXPOSURE_MIN,
				 exposure_max, S5K3P9_EXPOSURE_STEP,
				 mode->exposure);
	__v4l2_ctrl_s_ctrl(s5k3p9->exposure, mode->exposure);

	if (s5k3p9->sd.ctrl_handler->error)
		return s5k3p9->sd.ctrl_handler->error;

	s5k3p9->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5k3p9_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k3p9 *s5k3p9 = to_s5k3p9(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = s5k3p9_get_format_code(s5k3p9);

	return 0;
}

static int s5k3p9_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k3p9 *s5k3p9 = to_s5k3p9(sd);

	if (fse->index >= ARRAY_SIZE(s5k3p9_supported_modes))
		return -EINVAL;

	if (fse->code != s5k3p9_get_format_code(s5k3p9))
		return -EINVAL;

	fse->min_width = s5k3p9_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k3p9_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k3p9_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		/* The binned mode reads the full 4608x3456 array */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = 4608;
		sel->r.height = 3456;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5k3p9_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5k3p9 *s5k3p9 = to_s5k3p9(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = s5k3p9->mode->width,
			.height = s5k3p9->mode->height,
		},
	};

	s5k3p9_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5k3p9_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k3p9_pad_ops = {
	.set_fmt = s5k3p9_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5k3p9_get_selection,
	.enum_mbus_code = s5k3p9_enum_mbus_code,
	.enum_frame_size = s5k3p9_enum_frame_size,
	.enable_streams = s5k3p9_enable_streams,
	.disable_streams = s5k3p9_disable_streams,
};

static const struct v4l2_subdev_ops s5k3p9_subdev_ops = {
	.video = &s5k3p9_video_ops,
	.pad = &s5k3p9_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k3p9_internal_ops = {
	.init_state = s5k3p9_init_state,
};

static const struct media_entity_operations s5k3p9_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5k3p9_identify_sensor(struct s5k3p9 *s5k3p9)
{
	u64 val;
	int ret;

	ret = cci_read(s5k3p9->regmap, S5K3P9_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(s5k3p9->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5K3P9_CHIP_ID) {
		dev_err(s5k3p9->dev, "chip id mismatch: %x!=%llx\n",
			S5K3P9_CHIP_ID, val);
		return -ENODEV;
	}

	return 0;
}

static int s5k3p9_check_hwcfg(struct s5k3p9 *s5k3p9)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5k3p9->dev), *ep;
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K3P9_DATA_LANES) {
		dev_err(s5k3p9->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5k3p9->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5k3p9_link_freq_menu,
				       ARRAY_SIZE(s5k3p9_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* Stock order: rails, reset release, then MCLK, 1 ms apart */
static int s5k3p9_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k3p9 *s5k3p9 = to_s5k3p9(sd);
	int ret;

	ret = regulator_bulk_enable(S5K3P9_NUM_SUPPLIES, s5k3p9->supplies);
	if (ret)
		return ret;

	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(s5k3p9->reset_gpio, 0);
	usleep_range(1000, 1500);

	ret = clk_prepare_enable(s5k3p9->mclk);
	if (ret)
		goto disable_regulators;

	usleep_range(2000, 2500);

	return 0;

disable_regulators:
	gpiod_set_value_cansleep(s5k3p9->reset_gpio, 1);
	regulator_bulk_disable(S5K3P9_NUM_SUPPLIES, s5k3p9->supplies);

	return ret;
}

static int s5k3p9_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k3p9 *s5k3p9 = to_s5k3p9(sd);

	clk_disable_unprepare(s5k3p9->mclk);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(s5k3p9->reset_gpio, 1);

	regulator_bulk_disable(S5K3P9_NUM_SUPPLIES, s5k3p9->supplies);

	return 0;
}

static int s5k3p9_probe(struct i2c_client *client)
{
	struct s5k3p9 *s5k3p9;
	unsigned long freq;
	unsigned int i;
	int ret;

	s5k3p9 = devm_kzalloc(&client->dev, sizeof(*s5k3p9), GFP_KERNEL);
	if (!s5k3p9)
		return -ENOMEM;

	s5k3p9->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5k3p9->sd, client, &s5k3p9_subdev_ops);

	s5k3p9->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k3p9->regmap))
		return dev_err_probe(s5k3p9->dev, PTR_ERR(s5k3p9->regmap),
				     "failed to init CCI\n");

	s5k3p9->mclk = devm_v4l2_sensor_clk_get(s5k3p9->dev, NULL);
	if (IS_ERR(s5k3p9->mclk))
		return dev_err_probe(s5k3p9->dev, PTR_ERR(s5k3p9->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5k3p9->mclk);
	if (freq != S5K3P9_MCLK_FREQ_19P2MHZ)
		return dev_err_probe(s5k3p9->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = s5k3p9_check_hwcfg(s5k3p9);
	if (ret)
		return dev_err_probe(s5k3p9->dev, ret,
				     "failed to check HW configuration\n");

	s5k3p9->reset_gpio = devm_gpiod_get_optional(s5k3p9->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5k3p9->reset_gpio))
		return dev_err_probe(s5k3p9->dev, PTR_ERR(s5k3p9->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < S5K3P9_NUM_SUPPLIES; i++)
		s5k3p9->supplies[i].supply = s5k3p9_supply_names[i];

	ret = devm_regulator_bulk_get(s5k3p9->dev, S5K3P9_NUM_SUPPLIES,
				      s5k3p9->supplies);
	if (ret)
		return dev_err_probe(s5k3p9->dev, ret,
				     "failed to get supply regulators\n");

	ret = s5k3p9_power_on(s5k3p9->dev);
	if (ret)
		return ret;

	ret = s5k3p9_identify_sensor(s5k3p9);
	if (ret) {
		dev_err_probe(s5k3p9->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	s5k3p9->mode = &s5k3p9_supported_modes[0];
	ret = s5k3p9_init_controls(s5k3p9);
	if (ret) {
		dev_err_probe(s5k3p9->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	s5k3p9->sd.state_lock = s5k3p9->ctrl_handler.lock;
	s5k3p9->sd.internal_ops = &s5k3p9_internal_ops;
	s5k3p9->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k3p9->sd.entity.ops = &s5k3p9_subdev_entity_ops;
	s5k3p9->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k3p9->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k3p9->sd.entity, 1, &s5k3p9->pad);
	if (ret) {
		dev_err_probe(s5k3p9->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5k3p9->sd);
	if (ret < 0) {
		dev_err_probe(s5k3p9->dev, ret, "failed to init subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5k3p9->dev);
	pm_runtime_enable(s5k3p9->dev);

	ret = v4l2_async_register_subdev_sensor(&s5k3p9->sd);
	if (ret < 0) {
		dev_err_probe(s5k3p9->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5k3p9->dev, 1000);
	pm_runtime_use_autosuspend(s5k3p9->dev);
	pm_runtime_idle(s5k3p9->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&s5k3p9->sd);
	pm_runtime_disable(s5k3p9->dev);
	pm_runtime_set_suspended(s5k3p9->dev);

media_entity_cleanup:
	media_entity_cleanup(&s5k3p9->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5k3p9->sd.ctrl_handler);

power_off:
	s5k3p9_power_off(s5k3p9->dev);

	return ret;
}

static void s5k3p9_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k3p9 *s5k3p9 = to_s5k3p9(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5k3p9->dev);

	if (!pm_runtime_status_suspended(s5k3p9->dev)) {
		s5k3p9_power_off(s5k3p9->dev);
		pm_runtime_set_suspended(s5k3p9->dev);
	}
}

static const struct dev_pm_ops s5k3p9_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k3p9_power_off, s5k3p9_power_on, NULL)
};

static const struct of_device_id s5k3p9_of_match[] = {
	{ .compatible = "samsung,s5k3p9" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k3p9_of_match);

static struct i2c_driver s5k3p9_i2c_driver = {
	.driver = {
		.name = "s5k3p9",
		.pm = &s5k3p9_pm_ops,
		.of_match_table = s5k3p9_of_match,
	},
	.probe = s5k3p9_probe,
	.remove = s5k3p9_remove,
};

module_i2c_driver(s5k3p9_i2c_driver);

MODULE_DESCRIPTION("Samsung S5K3P9 image sensor driver");
MODULE_LICENSE("GPL");
