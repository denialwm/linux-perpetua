// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX890 image sensor driver
 *
 * Register tables are the OnePlus Ace 3 (aston) stock module settings for
 * its QTech main camera module. Driver structure follows s5k3p9.c.
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

/* Output PLL: 19.2 MHz / 2 * 295 / 2 = 1416 Msym/s, two symbols per cycle */
#define IMX890_LINK_FREQ_708MHZ		(708000ULL * HZ_PER_KHZ)
#define IMX890_MCLK_FREQ_19P2MHZ	(19200 * HZ_PER_KHZ)
#define IMX890_DATA_TRIOS		3
/* Video timing: 19.2 MHz / 3 * 166 * 16 / (5 * 2) */
#define IMX890_PIXEL_RATE		1699840000ULL

#define IMX890_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX890_CHIP_ID			0x0890

#define IMX890_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX890_MODE_STREAMING		BIT(0)

/* Coarse integration time in lines, kept on a 2-line lattice */
#define IMX890_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX890_EXPOSURE_MIN		10
#define IMX890_EXPOSURE_STEP		2
#define IMX890_EXPOSURE_MARGIN		48

/* Analogue gain = 16384 / (16384 - code), 1x to 64x */
#define IMX890_REG_AGAIN		CCI_REG16(0x0204)
#define IMX890_AGAIN_MIN		0
#define IMX890_AGAIN_MAX		0x3f00
#define IMX890_AGAIN_STEP		1
#define IMX890_AGAIN_DEFAULT		0

#define IMX890_REG_VTS			CCI_REG16(0x0340)
#define IMX890_VTS_MAX			0xffff

#define IMX890_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define to_imx890(_sd)			container_of(_sd, struct imx890, sd)

static const s64 imx890_link_freq_menu[] = {
	IMX890_LINK_FREQ_708MHZ,
};

struct imx890_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct imx890_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 exposure;

	const struct imx890_reg_list reg_list;
};

static const char * const imx890_test_pattern_menu[] = {
	"Disabled",
	"Solid colour",
	"Colour bars",
	"Fade to grey colour bars",
	"PN9",
};

/* Stock power-up order */
static const char * const imx890_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analog power */
	"vaux1",	/* Module supply, 1.8 V */
	"dvdd",		/* Digital core power */
	"vaux2",	/* Module supply, 2.8 V */
};

#define IMX890_NUM_SUPPLIES	ARRAY_SIZE(imx890_supply_names)

struct imx890 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX890_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;

	const struct imx890_mode *mode;
};

/*
 * Global init. It ends with the stock orientation 0x0101 = 3, under which
 * the output is BGGR.
 */
static const struct cci_reg_sequence imx890_init[] = {
	{ CCI_REG8(0x4c15), 0x07 },
	{ CCI_REG8(0x4c17), 0x1b },
	{ CCI_REG8(0x4c1b), 0x03 },
	{ CCI_REG8(0x4c1f), 0x02 },
	{ CCI_REG8(0x4c21), 0x5f },
	{ CCI_REG8(0x4c27), 0x43 },
	{ CCI_REG8(0x4c29), 0x09 },
	{ CCI_REG8(0x4c2b), 0x4a },
	{ CCI_REG8(0x4c2f), 0x02 },
	{ CCI_REG8(0x4c31), 0xc6 },
	{ CCI_REG8(0x4f07), 0x5f },
	{ CCI_REG8(0x4f49), 0xc6 },
	{ CCI_REG8(0x544f), 0xbd },
	{ CCI_REG8(0x54f7), 0x55 },
	{ CCI_REG8(0x54f9), 0x61 },
	{ CCI_REG8(0x5555), 0x78 },
	{ CCI_REG8(0x5559), 0xdf },
	{ CCI_REG8(0x5669), 0x99 },
	{ CCI_REG8(0x566a), 0x01 },
	{ CCI_REG8(0x566b), 0x95 },
	{ CCI_REG8(0x566c), 0x01 },
	{ CCI_REG8(0x566d), 0x57 },
	{ CCI_REG8(0x566e), 0x02 },
	{ CCI_REG8(0x566f), 0x87 },
	{ CCI_REG8(0x5a01), 0x78 },
	{ CCI_REG8(0x5a17), 0xdf },
	{ CCI_REG8(0x5d90), 0x03 },
	{ CCI_REG8(0x5dc0), 0x06 },
	{ CCI_REG8(0x630d), 0x0d },
	{ CCI_REG8(0x6317), 0x0d },
	{ CCI_REG8(0x632b), 0x34 },
	{ CCI_REG8(0x633f), 0x57 },
	{ CCI_REG8(0x6353), 0x5f },
	{ CCI_REG8(0x6367), 0x5f },
	{ CCI_REG8(0x638b), 0x9b },
	{ CCI_REG8(0x639f), 0xbe },
	{ CCI_REG8(0x63b3), 0xc6 },
	{ CCI_REG8(0x63c7), 0xc6 },
	{ CCI_REG8(0x8be4), 0x00 },
	{ CCI_REG8(0x8be5), 0x00 },
	{ CCI_REG8(0x0136), 0x13 },
	{ CCI_REG8(0x0137), 0x33 },
	{ CCI_REG8(0x33f0), 0x06 },
	{ CCI_REG8(0x33f1), 0x08 },
	{ CCI_REG8(0x0111), 0x03 },
	{ CCI_REG8(0x33d3), 0x01 },
	{ CCI_REG8(0x3892), 0x01 },
	{ CCI_REG8(0x4c14), 0x00 },
	{ CCI_REG8(0x4c15), 0x07 },
	{ CCI_REG8(0x4c16), 0x00 },
	{ CCI_REG8(0x4c17), 0x1b },
	{ CCI_REG8(0x4c1a), 0x00 },
	{ CCI_REG8(0x4c1b), 0x03 },
	{ CCI_REG8(0x4c1c), 0x00 },
	{ CCI_REG8(0x4c1d), 0x00 },
	{ CCI_REG8(0x4c1e), 0x00 },
	{ CCI_REG8(0x4c1f), 0x02 },
	{ CCI_REG8(0x4c20), 0x00 },
	{ CCI_REG8(0x4c21), 0x5f },
	{ CCI_REG8(0x4c26), 0x00 },
	{ CCI_REG8(0x4c27), 0x43 },
	{ CCI_REG8(0x4c28), 0x00 },
	{ CCI_REG8(0x4c29), 0x09 },
	{ CCI_REG8(0x4c2a), 0x00 },
	{ CCI_REG8(0x4c2b), 0x4a },
	{ CCI_REG8(0x4c2c), 0x00 },
	{ CCI_REG8(0x4c2d), 0x00 },
	{ CCI_REG8(0x4c2e), 0x00 },
	{ CCI_REG8(0x4c2f), 0x02 },
	{ CCI_REG8(0x4c30), 0x00 },
	{ CCI_REG8(0x4c31), 0xc6 },
	{ CCI_REG8(0x4c3e), 0x00 },
	{ CCI_REG8(0x4c3f), 0x55 },
	{ CCI_REG8(0x4c52), 0x00 },
	{ CCI_REG8(0x4c53), 0x97 },
	{ CCI_REG8(0x4cb4), 0x00 },
	{ CCI_REG8(0x4cb5), 0x55 },
	{ CCI_REG8(0x4cc8), 0x00 },
	{ CCI_REG8(0x4cc9), 0x97 },
	{ CCI_REG8(0x4d04), 0x00 },
	{ CCI_REG8(0x4d05), 0x4f },
	{ CCI_REG8(0x4d74), 0x00 },
	{ CCI_REG8(0x4d75), 0x55 },
	{ CCI_REG8(0x4f06), 0x00 },
	{ CCI_REG8(0x4f07), 0x5f },
	{ CCI_REG8(0x4f48), 0x00 },
	{ CCI_REG8(0x4f49), 0xc6 },
	{ CCI_REG8(0x544a), 0xff },
	{ CCI_REG8(0x544b), 0xff },
	{ CCI_REG8(0x544e), 0x01 },
	{ CCI_REG8(0x544f), 0xbd },
	{ CCI_REG8(0x5452), 0xff },
	{ CCI_REG8(0x5453), 0xff },
	{ CCI_REG8(0x5456), 0x00 },
	{ CCI_REG8(0x5457), 0xa5 },
	{ CCI_REG8(0x545a), 0xff },
	{ CCI_REG8(0x545b), 0xff },
	{ CCI_REG8(0x545e), 0x00 },
	{ CCI_REG8(0x545f), 0xa5 },
	{ CCI_REG8(0x5496), 0x00 },
	{ CCI_REG8(0x5497), 0xa2 },
	{ CCI_REG8(0x54f6), 0x01 },
	{ CCI_REG8(0x54f7), 0x55 },
	{ CCI_REG8(0x54f8), 0x01 },
	{ CCI_REG8(0x54f9), 0x61 },
	{ CCI_REG8(0x5670), 0x00 },
	{ CCI_REG8(0x5671), 0x85 },
	{ CCI_REG8(0x5672), 0x01 },
	{ CCI_REG8(0x5673), 0x77 },
	{ CCI_REG8(0x5674), 0x01 },
	{ CCI_REG8(0x5675), 0x2f },
	{ CCI_REG8(0x5676), 0x02 },
	{ CCI_REG8(0x5677), 0x55 },
	{ CCI_REG8(0x5678), 0x00 },
	{ CCI_REG8(0x5679), 0x85 },
	{ CCI_REG8(0x567a), 0x01 },
	{ CCI_REG8(0x567b), 0x77 },
	{ CCI_REG8(0x567c), 0x01 },
	{ CCI_REG8(0x567d), 0x2f },
	{ CCI_REG8(0x567e), 0x02 },
	{ CCI_REG8(0x567f), 0x55 },
	{ CCI_REG8(0x5680), 0x00 },
	{ CCI_REG8(0x5681), 0x85 },
	{ CCI_REG8(0x5682), 0x01 },
	{ CCI_REG8(0x5683), 0x77 },
	{ CCI_REG8(0x5684), 0x01 },
	{ CCI_REG8(0x5685), 0x2f },
	{ CCI_REG8(0x5686), 0x02 },
	{ CCI_REG8(0x5687), 0x55 },
	{ CCI_REG8(0x5688), 0x00 },
	{ CCI_REG8(0x5689), 0x85 },
	{ CCI_REG8(0x568a), 0x01 },
	{ CCI_REG8(0x568b), 0x77 },
	{ CCI_REG8(0x568c), 0x01 },
	{ CCI_REG8(0x568d), 0x2f },
	{ CCI_REG8(0x568e), 0x02 },
	{ CCI_REG8(0x568f), 0x55 },
	{ CCI_REG8(0x5690), 0x01 },
	{ CCI_REG8(0x5691), 0x7a },
	{ CCI_REG8(0x5692), 0x02 },
	{ CCI_REG8(0x5693), 0x6c },
	{ CCI_REG8(0x5694), 0x01 },
	{ CCI_REG8(0x5695), 0x35 },
	{ CCI_REG8(0x5696), 0x02 },
	{ CCI_REG8(0x5697), 0x5b },
	{ CCI_REG8(0x5698), 0x01 },
	{ CCI_REG8(0x5699), 0x7a },
	{ CCI_REG8(0x569a), 0x02 },
	{ CCI_REG8(0x569b), 0x6c },
	{ CCI_REG8(0x569c), 0x01 },
	{ CCI_REG8(0x569d), 0x35 },
	{ CCI_REG8(0x569e), 0x02 },
	{ CCI_REG8(0x569f), 0x5b },
	{ CCI_REG8(0x56a0), 0x01 },
	{ CCI_REG8(0x56a1), 0x7a },
	{ CCI_REG8(0x56a2), 0x02 },
	{ CCI_REG8(0x56a3), 0x6c },
	{ CCI_REG8(0x56a4), 0x01 },
	{ CCI_REG8(0x56a5), 0x35 },
	{ CCI_REG8(0x56a6), 0x02 },
	{ CCI_REG8(0x56a7), 0x5b },
	{ CCI_REG8(0x56a8), 0x01 },
	{ CCI_REG8(0x56a9), 0x80 },
	{ CCI_REG8(0x56aa), 0x02 },
	{ CCI_REG8(0x56ab), 0x72 },
	{ CCI_REG8(0x56ac), 0x01 },
	{ CCI_REG8(0x56ad), 0x2f },
	{ CCI_REG8(0x56ae), 0x02 },
	{ CCI_REG8(0x56af), 0x55 },
	{ CCI_REG8(0x5902), 0x0e },
	{ CCI_REG8(0x5a50), 0x04 },
	{ CCI_REG8(0x5a51), 0x04 },
	{ CCI_REG8(0x5a69), 0x01 },
	{ CCI_REG8(0x5c49), 0x0d },
	{ CCI_REG8(0x5d60), 0x08 },
	{ CCI_REG8(0x5d61), 0x08 },
	{ CCI_REG8(0x5d62), 0x08 },
	{ CCI_REG8(0x5d63), 0x08 },
	{ CCI_REG8(0x5d64), 0x08 },
	{ CCI_REG8(0x5d67), 0x08 },
	{ CCI_REG8(0x5d6c), 0x08 },
	{ CCI_REG8(0x5d6e), 0x08 },
	{ CCI_REG8(0x5d71), 0x08 },
	{ CCI_REG8(0x5d8e), 0x14 },
	{ CCI_REG8(0x5d90), 0x03 },
	{ CCI_REG8(0x5d91), 0x0a },
	{ CCI_REG8(0x5d92), 0x1f },
	{ CCI_REG8(0x5d93), 0x05 },
	{ CCI_REG8(0x5d97), 0x1f },
	{ CCI_REG8(0x5d9a), 0x06 },
	{ CCI_REG8(0x5d9c), 0x1f },
	{ CCI_REG8(0x5da1), 0x1f },
	{ CCI_REG8(0x5da6), 0x1f },
	{ CCI_REG8(0x5da8), 0x1f },
	{ CCI_REG8(0x5dab), 0x1f },
	{ CCI_REG8(0x5dc0), 0x06 },
	{ CCI_REG8(0x5dc1), 0x06 },
	{ CCI_REG8(0x5dc2), 0x07 },
	{ CCI_REG8(0x5dc3), 0x06 },
	{ CCI_REG8(0x5dc4), 0x07 },
	{ CCI_REG8(0x5dc7), 0x07 },
	{ CCI_REG8(0x5dcc), 0x07 },
	{ CCI_REG8(0x5dce), 0x07 },
	{ CCI_REG8(0x5dd1), 0x07 },
	{ CCI_REG8(0x5e3e), 0x00 },
	{ CCI_REG8(0x5e3f), 0x00 },
	{ CCI_REG8(0x5e41), 0x00 },
	{ CCI_REG8(0x5e48), 0x00 },
	{ CCI_REG8(0x5e49), 0x00 },
	{ CCI_REG8(0x5e4a), 0x00 },
	{ CCI_REG8(0x5e4c), 0x00 },
	{ CCI_REG8(0x5e4d), 0x00 },
	{ CCI_REG8(0x5e4e), 0x00 },
	{ CCI_REG8(0x6026), 0x03 },
	{ CCI_REG8(0x6028), 0x03 },
	{ CCI_REG8(0x602a), 0x03 },
	{ CCI_REG8(0x602c), 0x03 },
	{ CCI_REG8(0x602f), 0x03 },
	{ CCI_REG8(0x6036), 0x03 },
	{ CCI_REG8(0x6038), 0x03 },
	{ CCI_REG8(0x603a), 0x03 },
	{ CCI_REG8(0x603c), 0x03 },
	{ CCI_REG8(0x603f), 0x03 },
	{ CCI_REG8(0x6074), 0x19 },
	{ CCI_REG8(0x6076), 0x19 },
	{ CCI_REG8(0x6078), 0x19 },
	{ CCI_REG8(0x607a), 0x19 },
	{ CCI_REG8(0x607d), 0x19 },
	{ CCI_REG8(0x6084), 0x32 },
	{ CCI_REG8(0x6086), 0x32 },
	{ CCI_REG8(0x6088), 0x32 },
	{ CCI_REG8(0x608a), 0x32 },
	{ CCI_REG8(0x608d), 0x32 },
	{ CCI_REG8(0x60c2), 0x4a },
	{ CCI_REG8(0x60c4), 0x4a },
	{ CCI_REG8(0x60cb), 0x4a },
	{ CCI_REG8(0x60d2), 0x4a },
	{ CCI_REG8(0x60d4), 0x4a },
	{ CCI_REG8(0x60db), 0x4a },
	{ CCI_REG8(0x62f9), 0x14 },
	{ CCI_REG8(0x6305), 0x13 },
	{ CCI_REG8(0x6307), 0x13 },
	{ CCI_REG8(0x630a), 0x13 },
	{ CCI_REG8(0x630d), 0x0d },
	{ CCI_REG8(0x6317), 0x0d },
	{ CCI_REG8(0x632f), 0x2e },
	{ CCI_REG8(0x6333), 0x2e },
	{ CCI_REG8(0x6339), 0x2e },
	{ CCI_REG8(0x6343), 0x2e },
	{ CCI_REG8(0x6347), 0x2e },
	{ CCI_REG8(0x634d), 0x2e },
	{ CCI_REG8(0x6352), 0x00 },
	{ CCI_REG8(0x6353), 0x5f },
	{ CCI_REG8(0x6366), 0x00 },
	{ CCI_REG8(0x6367), 0x5f },
	{ CCI_REG8(0x638f), 0x95 },
	{ CCI_REG8(0x6393), 0x95 },
	{ CCI_REG8(0x6399), 0x95 },
	{ CCI_REG8(0x63a3), 0x95 },
	{ CCI_REG8(0x63a7), 0x95 },
	{ CCI_REG8(0x63ad), 0x95 },
	{ CCI_REG8(0x63b2), 0x00 },
	{ CCI_REG8(0x63b3), 0xc6 },
	{ CCI_REG8(0x63c6), 0x00 },
	{ CCI_REG8(0x63c7), 0xc6 },
	{ CCI_REG8(0x8bdb), 0x02 },
	{ CCI_REG8(0x8bde), 0x02 },
	{ CCI_REG8(0x8be1), 0x2d },
	{ CCI_REG8(0x8be4), 0x00 },
	{ CCI_REG8(0x8be5), 0x00 },
	{ CCI_REG8(0x8be6), 0x01 },
	{ CCI_REG8(0x9002), 0x14 },
	{ CCI_REG8(0x9200), 0xb5 },
	{ CCI_REG8(0x9201), 0x9e },
	{ CCI_REG8(0x9202), 0xb5 },
	{ CCI_REG8(0x9203), 0x42 },
	{ CCI_REG8(0x9204), 0xb5 },
	{ CCI_REG8(0x9205), 0x43 },
	{ CCI_REG8(0x9206), 0xbd },
	{ CCI_REG8(0x9207), 0x20 },
	{ CCI_REG8(0x9208), 0xbd },
	{ CCI_REG8(0x9209), 0x22 },
	{ CCI_REG8(0x920a), 0xbd },
	{ CCI_REG8(0x920b), 0x23 },
	{ CCI_REG8(0xb5d7), 0x10 },
	{ CCI_REG8(0xbd24), 0x00 },
	{ CCI_REG8(0xbd25), 0x00 },
	{ CCI_REG8(0xbd26), 0x00 },
	{ CCI_REG8(0xbd27), 0x00 },
	{ CCI_REG8(0xbd28), 0x00 },
	{ CCI_REG8(0xbd29), 0x00 },
	{ CCI_REG8(0xbd2a), 0x00 },
	{ CCI_REG8(0xbd2b), 0x00 },
	{ CCI_REG8(0xbd2c), 0x32 },
	{ CCI_REG8(0xbd2d), 0x70 },
	{ CCI_REG8(0xbd2e), 0x25 },
	{ CCI_REG8(0xbd2f), 0x30 },
	{ CCI_REG8(0xbd30), 0x3b },
	{ CCI_REG8(0xbd31), 0xe0 },
	{ CCI_REG8(0xbd32), 0x69 },
	{ CCI_REG8(0xbd33), 0x40 },
	{ CCI_REG8(0xbd34), 0x25 },
	{ CCI_REG8(0xbd35), 0x90 },
	{ CCI_REG8(0xbd36), 0x58 },
	{ CCI_REG8(0xbd37), 0x00 },
	{ CCI_REG8(0xbd38), 0x00 },
	{ CCI_REG8(0xbd39), 0x00 },
	{ CCI_REG8(0xbd3a), 0x00 },
	{ CCI_REG8(0xbd3b), 0x00 },
	{ CCI_REG8(0xbd3c), 0x32 },
	{ CCI_REG8(0xbd3d), 0x70 },
	{ CCI_REG8(0xbd3e), 0x25 },
	{ CCI_REG8(0xbd3f), 0x90 },
	{ CCI_REG8(0xbd40), 0x58 },
	{ CCI_REG8(0xbd41), 0x00 },
	{ CCI_REG8(0x793b), 0x01 },
	{ CCI_REG8(0xacc6), 0x00 },
	{ CCI_REG8(0xacf5), 0x00 },
	{ CCI_REG8(0x793b), 0x00 },
	{ CCI_REG8(0x1f04), 0xb3 },
	{ CCI_REG8(0x1f05), 0x01 },
	{ CCI_REG8(0x1f06), 0x07 },
	{ CCI_REG8(0x1f07), 0x66 },
	{ CCI_REG8(0x1f08), 0x01 },
	{ CCI_REG8(0x4d18), 0x00 },
	{ CCI_REG8(0x4d19), 0x9d },
	{ CCI_REG8(0x4d88), 0x00 },
	{ CCI_REG8(0x4d89), 0x97 },
	{ CCI_REG8(0x5c57), 0x0a },
	{ CCI_REG8(0x5d94), 0x1f },
	{ CCI_REG8(0x5d9e), 0x1f },
	{ CCI_REG8(0x5e50), 0x23 },
	{ CCI_REG8(0x5e51), 0x20 },
	{ CCI_REG8(0x5e52), 0x07 },
	{ CCI_REG8(0x5e53), 0x20 },
	{ CCI_REG8(0x5e54), 0x07 },
	{ CCI_REG8(0x5e55), 0x27 },
	{ CCI_REG8(0x5e56), 0x0b },
	{ CCI_REG8(0x5e57), 0x24 },
	{ CCI_REG8(0x5e58), 0x0b },
	{ CCI_REG8(0x5e60), 0x24 },
	{ CCI_REG8(0x5e61), 0x24 },
	{ CCI_REG8(0x5e62), 0x1b },
	{ CCI_REG8(0x5e63), 0x23 },
	{ CCI_REG8(0x5e64), 0x1b },
	{ CCI_REG8(0x5e65), 0x28 },
	{ CCI_REG8(0x5e66), 0x22 },
	{ CCI_REG8(0x5e67), 0x28 },
	{ CCI_REG8(0x5e68), 0x23 },
	{ CCI_REG8(0x5e70), 0x25 },
	{ CCI_REG8(0x5e71), 0x24 },
	{ CCI_REG8(0x5e72), 0x20 },
	{ CCI_REG8(0x5e73), 0x24 },
	{ CCI_REG8(0x5e74), 0x20 },
	{ CCI_REG8(0x5e75), 0x28 },
	{ CCI_REG8(0x5e76), 0x27 },
	{ CCI_REG8(0x5e77), 0x29 },
	{ CCI_REG8(0x5e78), 0x24 },
	{ CCI_REG8(0x5e80), 0x25 },
	{ CCI_REG8(0x5e81), 0x25 },
	{ CCI_REG8(0x5e82), 0x24 },
	{ CCI_REG8(0x5e83), 0x25 },
	{ CCI_REG8(0x5e84), 0x23 },
	{ CCI_REG8(0x5e85), 0x2a },
	{ CCI_REG8(0x5e86), 0x28 },
	{ CCI_REG8(0x5e87), 0x2a },
	{ CCI_REG8(0x5e88), 0x28 },
	{ CCI_REG8(0x5e90), 0x24 },
	{ CCI_REG8(0x5e91), 0x24 },
	{ CCI_REG8(0x5e92), 0x28 },
	{ CCI_REG8(0x5e93), 0x29 },
	{ CCI_REG8(0x5e97), 0x25 },
	{ CCI_REG8(0x5e98), 0x25 },
	{ CCI_REG8(0x5e99), 0x2a },
	{ CCI_REG8(0x5e9a), 0x2a },
	{ CCI_REG8(0x5e9e), 0x3a },
	{ CCI_REG8(0x5e9f), 0x3f },
	{ CCI_REG8(0x5ea0), 0x17 },
	{ CCI_REG8(0x5ea1), 0x3f },
	{ CCI_REG8(0x5ea2), 0x17 },
	{ CCI_REG8(0x5ea3), 0x32 },
	{ CCI_REG8(0x5ea4), 0x10 },
	{ CCI_REG8(0x5ea5), 0x33 },
	{ CCI_REG8(0x5ea6), 0x10 },
	{ CCI_REG8(0x5eae), 0x3d },
	{ CCI_REG8(0x5eaf), 0x48 },
	{ CCI_REG8(0x5eb0), 0x3b },
	{ CCI_REG8(0x5eb1), 0x45 },
	{ CCI_REG8(0x5eb2), 0x37 },
	{ CCI_REG8(0x5eb3), 0x3a },
	{ CCI_REG8(0x5eb4), 0x31 },
	{ CCI_REG8(0x5eb5), 0x3a },
	{ CCI_REG8(0x5eb6), 0x31 },
	{ CCI_REG8(0x5ebe), 0x40 },
	{ CCI_REG8(0x5ebf), 0x48 },
	{ CCI_REG8(0x5ec0), 0x3f },
	{ CCI_REG8(0x5ec1), 0x45 },
	{ CCI_REG8(0x5ec2), 0x3f },
	{ CCI_REG8(0x5ec3), 0x3a },
	{ CCI_REG8(0x5ec4), 0x32 },
	{ CCI_REG8(0x5ec5), 0x3a },
	{ CCI_REG8(0x5ec6), 0x33 },
	{ CCI_REG8(0x5ece), 0x4b },
	{ CCI_REG8(0x5ecf), 0x4a },
	{ CCI_REG8(0x5ed0), 0x48 },
	{ CCI_REG8(0x5ed1), 0x4c },
	{ CCI_REG8(0x5ed2), 0x45 },
	{ CCI_REG8(0x5ed3), 0x3f },
	{ CCI_REG8(0x5ed4), 0x3a },
	{ CCI_REG8(0x5ed5), 0x3f },
	{ CCI_REG8(0x5ed6), 0x3a },
	{ CCI_REG8(0x5ede), 0x48 },
	{ CCI_REG8(0x5edf), 0x45 },
	{ CCI_REG8(0x5ee0), 0x3a },
	{ CCI_REG8(0x5ee1), 0x3a },
	{ CCI_REG8(0x5ee5), 0x4a },
	{ CCI_REG8(0x5ee6), 0x4c },
	{ CCI_REG8(0x5ee7), 0x3f },
	{ CCI_REG8(0x5ee8), 0x3f },
	{ CCI_REG8(0x5eec), 0x06 },
	{ CCI_REG8(0x5eed), 0x06 },
	{ CCI_REG8(0x5eee), 0x02 },
	{ CCI_REG8(0x5eef), 0x06 },
	{ CCI_REG8(0x5ef0), 0x01 },
	{ CCI_REG8(0x5ef1), 0x09 },
	{ CCI_REG8(0x5ef2), 0x05 },
	{ CCI_REG8(0x5ef3), 0x06 },
	{ CCI_REG8(0x5ef4), 0x04 },
	{ CCI_REG8(0x5efc), 0x07 },
	{ CCI_REG8(0x5efd), 0x09 },
	{ CCI_REG8(0x5efe), 0x05 },
	{ CCI_REG8(0x5eff), 0x08 },
	{ CCI_REG8(0x5f00), 0x04 },
	{ CCI_REG8(0x5f01), 0x09 },
	{ CCI_REG8(0x5f02), 0x05 },
	{ CCI_REG8(0x5f03), 0x09 },
	{ CCI_REG8(0x5f04), 0x04 },
	{ CCI_REG8(0x5f0c), 0x08 },
	{ CCI_REG8(0x5f0d), 0x09 },
	{ CCI_REG8(0x5f0e), 0x06 },
	{ CCI_REG8(0x5f0f), 0x09 },
	{ CCI_REG8(0x5f10), 0x06 },
	{ CCI_REG8(0x5f11), 0x09 },
	{ CCI_REG8(0x5f12), 0x09 },
	{ CCI_REG8(0x5f13), 0x09 },
	{ CCI_REG8(0x5f14), 0x06 },
	{ CCI_REG8(0x5f1c), 0x09 },
	{ CCI_REG8(0x5f1d), 0x09 },
	{ CCI_REG8(0x5f1e), 0x09 },
	{ CCI_REG8(0x5f1f), 0x09 },
	{ CCI_REG8(0x5f20), 0x08 },
	{ CCI_REG8(0x5f21), 0x09 },
	{ CCI_REG8(0x5f22), 0x09 },
	{ CCI_REG8(0x5f23), 0x09 },
	{ CCI_REG8(0x5f24), 0x09 },
	{ CCI_REG8(0x5f2c), 0x09 },
	{ CCI_REG8(0x5f2d), 0x09 },
	{ CCI_REG8(0x5f2e), 0x09 },
	{ CCI_REG8(0x5f2f), 0x09 },
	{ CCI_REG8(0x5f33), 0x09 },
	{ CCI_REG8(0x5f34), 0x09 },
	{ CCI_REG8(0x5f35), 0x09 },
	{ CCI_REG8(0x5f36), 0x09 },
	{ CCI_REG8(0x5f3a), 0x01 },
	{ CCI_REG8(0x5f3d), 0x07 },
	{ CCI_REG8(0x5f3f), 0x01 },
	{ CCI_REG8(0x5f4b), 0x01 },
	{ CCI_REG8(0x5f4d), 0x04 },
	{ CCI_REG8(0x5f4f), 0x02 },
	{ CCI_REG8(0x5f51), 0x02 },
	{ CCI_REG8(0x5f5a), 0x02 },
	{ CCI_REG8(0x5f5b), 0x01 },
	{ CCI_REG8(0x5f5d), 0x03 },
	{ CCI_REG8(0x5f5e), 0x07 },
	{ CCI_REG8(0x5f5f), 0x01 },
	{ CCI_REG8(0x5f60), 0x01 },
	{ CCI_REG8(0x5f61), 0x01 },
	{ CCI_REG8(0x5f6a), 0x01 },
	{ CCI_REG8(0x5f6c), 0x01 },
	{ CCI_REG8(0x5f6d), 0x01 },
	{ CCI_REG8(0x5f6e), 0x04 },
	{ CCI_REG8(0x5f70), 0x02 },
	{ CCI_REG8(0x5f72), 0x02 },
	{ CCI_REG8(0x5f7a), 0x01 },
	{ CCI_REG8(0x5f7b), 0x03 },
	{ CCI_REG8(0x5f7c), 0x01 },
	{ CCI_REG8(0x5f7d), 0x01 },
	{ CCI_REG8(0x5f82), 0x01 },
	{ CCI_REG8(0x60c6), 0x4a },
	{ CCI_REG8(0x60c8), 0x4a },
	{ CCI_REG8(0x60d6), 0x4a },
	{ CCI_REG8(0x60d8), 0x4a },
	{ CCI_REG8(0x62e4), 0x33 },
	{ CCI_REG8(0x62e9), 0x33 },
	{ CCI_REG8(0x62ee), 0x1c },
	{ CCI_REG8(0x62ef), 0x33 },
	{ CCI_REG8(0x62f3), 0x33 },
	{ CCI_REG8(0x62f6), 0x1c },
	{ CCI_REG8(0x33f2), 0x01 },
	{ CCI_REG8(0x1f04), 0xa3 },
	{ CCI_REG8(0x1f05), 0x01 },
	{ CCI_REG8(0x406e), 0x00 },
	{ CCI_REG8(0x406f), 0x08 },
	{ CCI_REG8(0x4d08), 0x00 },
	{ CCI_REG8(0x4d09), 0x2c },
	{ CCI_REG8(0x4d0e), 0x00 },
	{ CCI_REG8(0x4d0f), 0x64 },
	{ CCI_REG8(0x4d18), 0x00 },
	{ CCI_REG8(0x4d19), 0xb1 },
	{ CCI_REG8(0x4d1e), 0x00 },
	{ CCI_REG8(0x4d1f), 0xcb },
	{ CCI_REG8(0x4d3a), 0x00 },
	{ CCI_REG8(0x4d3b), 0x91 },
	{ CCI_REG8(0x4d40), 0x00 },
	{ CCI_REG8(0x4d41), 0x64 },
	{ CCI_REG8(0x4d4c), 0x00 },
	{ CCI_REG8(0x4d4d), 0xe8 },
	{ CCI_REG8(0x4d52), 0x00 },
	{ CCI_REG8(0x4d53), 0xcb },
	{ CCI_REG8(0x4d78), 0x00 },
	{ CCI_REG8(0x4d79), 0x2c },
	{ CCI_REG8(0x4d7e), 0x00 },
	{ CCI_REG8(0x4d7f), 0x64 },
	{ CCI_REG8(0x4d88), 0x00 },
	{ CCI_REG8(0x4d89), 0xab },
	{ CCI_REG8(0x4d8e), 0x00 },
	{ CCI_REG8(0x4d8f), 0xcb },
	{ CCI_REG8(0x4da6), 0x00 },
	{ CCI_REG8(0x4da7), 0xe7 },
	{ CCI_REG8(0x4dac), 0x00 },
	{ CCI_REG8(0x4dad), 0xcb },
	{ CCI_REG8(0x5b98), 0x00 },
	{ CCI_REG8(0x5c52), 0x05 },
	{ CCI_REG8(0x5c57), 0x09 },
	{ CCI_REG8(0x5d94), 0x0a },
	{ CCI_REG8(0x5d9e), 0x0a },
	{ CCI_REG8(0x5e50), 0x22 },
	{ CCI_REG8(0x5e51), 0x22 },
	{ CCI_REG8(0x5e52), 0x07 },
	{ CCI_REG8(0x5e53), 0x20 },
	{ CCI_REG8(0x5e54), 0x06 },
	{ CCI_REG8(0x5e55), 0x23 },
	{ CCI_REG8(0x5e56), 0x0a },
	{ CCI_REG8(0x5e57), 0x23 },
	{ CCI_REG8(0x5e58), 0x0a },
	{ CCI_REG8(0x5e60), 0x25 },
	{ CCI_REG8(0x5e61), 0x29 },
	{ CCI_REG8(0x5e62), 0x1c },
	{ CCI_REG8(0x5e63), 0x26 },
	{ CCI_REG8(0x5e64), 0x1c },
	{ CCI_REG8(0x5e65), 0x2d },
	{ CCI_REG8(0x5e66), 0x1e },
	{ CCI_REG8(0x5e67), 0x2a },
	{ CCI_REG8(0x5e68), 0x1e },
	{ CCI_REG8(0x5e70), 0x26 },
	{ CCI_REG8(0x5e71), 0x26 },
	{ CCI_REG8(0x5e72), 0x22 },
	{ CCI_REG8(0x5e73), 0x23 },
	{ CCI_REG8(0x5e74), 0x20 },
	{ CCI_REG8(0x5e75), 0x28 },
	{ CCI_REG8(0x5e76), 0x23 },
	{ CCI_REG8(0x5e77), 0x28 },
	{ CCI_REG8(0x5e78), 0x23 },
	{ CCI_REG8(0x5e80), 0x28 },
	{ CCI_REG8(0x5e81), 0x28 },
	{ CCI_REG8(0x5e82), 0x29 },
	{ CCI_REG8(0x5e83), 0x27 },
	{ CCI_REG8(0x5e84), 0x26 },
	{ CCI_REG8(0x5e85), 0x2a },
	{ CCI_REG8(0x5e86), 0x2d },
	{ CCI_REG8(0x5e87), 0x2a },
	{ CCI_REG8(0x5e88), 0x2a },
	{ CCI_REG8(0x5e90), 0x26 },
	{ CCI_REG8(0x5e91), 0x23 },
	{ CCI_REG8(0x5e92), 0x28 },
	{ CCI_REG8(0x5e93), 0x28 },
	{ CCI_REG8(0x5e97), 0x2f },
	{ CCI_REG8(0x5e98), 0x2e },
	{ CCI_REG8(0x5e99), 0x32 },
	{ CCI_REG8(0x5e9a), 0x32 },
	{ CCI_REG8(0x5e9e), 0x50 },
	{ CCI_REG8(0x5e9f), 0x50 },
	{ CCI_REG8(0x5ea0), 0x1e },
	{ CCI_REG8(0x5ea1), 0x50 },
	{ CCI_REG8(0x5ea2), 0x1d },
	{ CCI_REG8(0x5ea3), 0x3e },
	{ CCI_REG8(0x5ea4), 0x14 },
	{ CCI_REG8(0x5ea5), 0x3e },
	{ CCI_REG8(0x5ea6), 0x14 },
	{ CCI_REG8(0x5eae), 0x58 },
	{ CCI_REG8(0x5eaf), 0x5e },
	{ CCI_REG8(0x5eb0), 0x4b },
	{ CCI_REG8(0x5eb1), 0x5a },
	{ CCI_REG8(0x5eb2), 0x4b },
	{ CCI_REG8(0x5eb3), 0x4c },
	{ CCI_REG8(0x5eb4), 0x3a },
	{ CCI_REG8(0x5eb5), 0x4c },
	{ CCI_REG8(0x5eb6), 0x38 },
	{ CCI_REG8(0x5ebe), 0x56 },
	{ CCI_REG8(0x5ebf), 0x57 },
	{ CCI_REG8(0x5ec0), 0x50 },
	{ CCI_REG8(0x5ec1), 0x55 },
	{ CCI_REG8(0x5ec2), 0x50 },
	{ CCI_REG8(0x5ec3), 0x46 },
	{ CCI_REG8(0x5ec4), 0x3e },
	{ CCI_REG8(0x5ec5), 0x46 },
	{ CCI_REG8(0x5ec6), 0x3e },
	{ CCI_REG8(0x5ece), 0x5a },
	{ CCI_REG8(0x5ecf), 0x5f },
	{ CCI_REG8(0x5ed0), 0x5e },
	{ CCI_REG8(0x5ed1), 0x5a },
	{ CCI_REG8(0x5ed2), 0x5a },
	{ CCI_REG8(0x5ed3), 0x50 },
	{ CCI_REG8(0x5ed4), 0x4c },
	{ CCI_REG8(0x5ed5), 0x50 },
	{ CCI_REG8(0x5ed6), 0x4c },
	{ CCI_REG8(0x5ede), 0x57 },
	{ CCI_REG8(0x5edf), 0x55 },
	{ CCI_REG8(0x5ee0), 0x46 },
	{ CCI_REG8(0x5ee1), 0x46 },
	{ CCI_REG8(0x5ee5), 0x73 },
	{ CCI_REG8(0x5ee6), 0x6e },
	{ CCI_REG8(0x5ee7), 0x5f },
	{ CCI_REG8(0x5ee8), 0x5a },
	{ CCI_REG8(0x5eec), 0x0a },
	{ CCI_REG8(0x5eed), 0x0a },
	{ CCI_REG8(0x5eee), 0x0f },
	{ CCI_REG8(0x5eef), 0x0a },
	{ CCI_REG8(0x5ef0), 0x0e },
	{ CCI_REG8(0x5ef1), 0x08 },
	{ CCI_REG8(0x5ef2), 0x0c },
	{ CCI_REG8(0x5ef3), 0x0c },
	{ CCI_REG8(0x5ef4), 0x0f },
	{ CCI_REG8(0x5efc), 0x0a },
	{ CCI_REG8(0x5efd), 0x0a },
	{ CCI_REG8(0x5efe), 0x14 },
	{ CCI_REG8(0x5eff), 0x0a },
	{ CCI_REG8(0x5f00), 0x14 },
	{ CCI_REG8(0x5f01), 0x0a },
	{ CCI_REG8(0x5f02), 0x14 },
	{ CCI_REG8(0x5f03), 0x0a },
	{ CCI_REG8(0x5f04), 0x19 },
	{ CCI_REG8(0x5f0c), 0x0a },
	{ CCI_REG8(0x5f0d), 0x0a },
	{ CCI_REG8(0x5f0e), 0x0a },
	{ CCI_REG8(0x5f0f), 0x05 },
	{ CCI_REG8(0x5f10), 0x0a },
	{ CCI_REG8(0x5f11), 0x06 },
	{ CCI_REG8(0x5f12), 0x08 },
	{ CCI_REG8(0x5f13), 0x0a },
	{ CCI_REG8(0x5f14), 0x0c },
	{ CCI_REG8(0x5f1c), 0x0a },
	{ CCI_REG8(0x5f1d), 0x0a },
	{ CCI_REG8(0x5f1e), 0x0a },
	{ CCI_REG8(0x5f1f), 0x0a },
	{ CCI_REG8(0x5f20), 0x0a },
	{ CCI_REG8(0x5f21), 0x0a },
	{ CCI_REG8(0x5f22), 0x0a },
	{ CCI_REG8(0x5f23), 0x0a },
	{ CCI_REG8(0x5f24), 0x0a },
	{ CCI_REG8(0x5f2c), 0x0a },
	{ CCI_REG8(0x5f2d), 0x05 },
	{ CCI_REG8(0x5f2e), 0x06 },
	{ CCI_REG8(0x5f2f), 0x0a },
	{ CCI_REG8(0x5f33), 0x0a },
	{ CCI_REG8(0x5f34), 0x0a },
	{ CCI_REG8(0x5f35), 0x0a },
	{ CCI_REG8(0x5f36), 0x0a },
	{ CCI_REG8(0x5f3a), 0x00 },
	{ CCI_REG8(0x5f3d), 0x02 },
	{ CCI_REG8(0x5f3f), 0x0a },
	{ CCI_REG8(0x5f4a), 0x0a },
	{ CCI_REG8(0x5f4b), 0x0a },
	{ CCI_REG8(0x5f4d), 0x0f },
	{ CCI_REG8(0x5f4f), 0x00 },
	{ CCI_REG8(0x5f51), 0x00 },
	{ CCI_REG8(0x5f5a), 0x00 },
	{ CCI_REG8(0x5f5b), 0x00 },
	{ CCI_REG8(0x5f5d), 0x0a },
	{ CCI_REG8(0x5f5e), 0x02 },
	{ CCI_REG8(0x5f5f), 0x0a },
	{ CCI_REG8(0x5f60), 0x0a },
	{ CCI_REG8(0x5f61), 0x00 },
	{ CCI_REG8(0x5f6a), 0x00 },
	{ CCI_REG8(0x5f6c), 0x0a },
	{ CCI_REG8(0x5f6d), 0x06 },
	{ CCI_REG8(0x5f6e), 0x0f },
	{ CCI_REG8(0x5f70), 0x00 },
	{ CCI_REG8(0x5f72), 0x00 },
	{ CCI_REG8(0x5f7a), 0x00 },
	{ CCI_REG8(0x5f7b), 0x0a },
	{ CCI_REG8(0x5f7c), 0x0a },
	{ CCI_REG8(0x5f7d), 0x00 },
	{ CCI_REG8(0x5f82), 0x06 },
	{ CCI_REG8(0x60c6), 0x36 },
	{ CCI_REG8(0x60c8), 0x36 },
	{ CCI_REG8(0x60d6), 0x36 },
	{ CCI_REG8(0x60d8), 0x36 },
	{ CCI_REG8(0x62df), 0x56 },
	{ CCI_REG8(0x62e0), 0x52 },
	{ CCI_REG8(0x62e4), 0x38 },
	{ CCI_REG8(0x62e5), 0x51 },
	{ CCI_REG8(0x62e9), 0x35 },
	{ CCI_REG8(0x62ea), 0x54 },
	{ CCI_REG8(0x62ee), 0x1d },
	{ CCI_REG8(0x62ef), 0x38 },
	{ CCI_REG8(0x62f3), 0x33 },
	{ CCI_REG8(0x62f6), 0x26 },
	{ CCI_REG8(0x6412), 0x1e },
	{ CCI_REG8(0x6413), 0x1e },
	{ CCI_REG8(0x6414), 0x1e },
	{ CCI_REG8(0x6415), 0x1e },
	{ CCI_REG8(0x6416), 0x1e },
	{ CCI_REG8(0x6417), 0x1e },
	{ CCI_REG8(0x6418), 0x1e },
	{ CCI_REG8(0x641a), 0x1e },
	{ CCI_REG8(0x641b), 0x1e },
	{ CCI_REG8(0x641c), 0x1e },
	{ CCI_REG8(0x641d), 0x1e },
	{ CCI_REG8(0x641e), 0x1e },
	{ CCI_REG8(0x641f), 0x1e },
	{ CCI_REG8(0x6420), 0x1e },
	{ CCI_REG8(0x6421), 0x1e },
	{ CCI_REG8(0x6422), 0x1e },
	{ CCI_REG8(0x6424), 0x1e },
	{ CCI_REG8(0x6425), 0x1e },
	{ CCI_REG8(0x6426), 0x1e },
	{ CCI_REG8(0x6427), 0x1e },
	{ CCI_REG8(0x6428), 0x1e },
	{ CCI_REG8(0x6429), 0x1e },
	{ CCI_REG8(0x642a), 0x1e },
	{ CCI_REG8(0x642b), 0x1e },
	{ CCI_REG8(0x642c), 0x1e },
	{ CCI_REG8(0x642e), 0x1e },
	{ CCI_REG8(0x642f), 0x1e },
	{ CCI_REG8(0x6430), 0x1e },
	{ CCI_REG8(0x6431), 0x1e },
	{ CCI_REG8(0x6432), 0x1e },
	{ CCI_REG8(0x6433), 0x1e },
	{ CCI_REG8(0x6434), 0x1e },
	{ CCI_REG8(0x6435), 0x1e },
	{ CCI_REG8(0x6436), 0x1e },
	{ CCI_REG8(0x6438), 0x1e },
	{ CCI_REG8(0x6439), 0x1e },
	{ CCI_REG8(0x643a), 0x1e },
	{ CCI_REG8(0x643b), 0x1e },
	{ CCI_REG8(0x643d), 0x1e },
	{ CCI_REG8(0x643e), 0x1e },
	{ CCI_REG8(0x643f), 0x1e },
	{ CCI_REG8(0x6441), 0x1e },
	{ CCI_REG8(0x33f2), 0x02 },
	{ CCI_REG8(0x1f08), 0x00 },
	{ CCI_REG8(0xa307), 0x30 },
	{ CCI_REG8(0xa309), 0x30 },
	{ CCI_REG8(0xa30b), 0x30 },
	{ CCI_REG8(0xa406), 0x03 },
	{ CCI_REG8(0xa407), 0x48 },
	{ CCI_REG8(0xa408), 0x03 },
	{ CCI_REG8(0xa409), 0x48 },
	{ CCI_REG8(0xa40a), 0x03 },
	{ CCI_REG8(0xa40b), 0x48 },
	{ CCI_REG8(0x825b), 0x01 },
	{ CCI_REG8(0xec12), 0x01 },
	{ CCI_REG8(0xec14), 0x3f },
	{ CCI_REG8(0xec15), 0x00 },
	{ CCI_REG8(0xec16), 0x20 },
	{ CCI_REG8(0xec17), 0x00 },
	{ CCI_REG8(0xec18), 0x1f },
	{ CCI_REG8(0xec19), 0xff },
	{ CCI_REG8(0xec1a), 0x00 },
	{ CCI_REG8(0xec1b), 0x00 },
	{ CCI_REG8(0x0101), 0x03 },
	{ CCI_REG8(0x0106), 0x01 },
	{ CCI_REG8(0x305c), 0x00 },
	{ CCI_REG8(0x86a9), 0x4e },
};

/* 2x2 binned 4096x3072 at 30 fps: stock mode 1 */
static const struct cci_reg_sequence imx890_4096x3072_mode[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x02 },
	{ CCI_REG8(0x0342), 0x3d },
	{ CCI_REG8(0x0343), 0x00 },
	{ CCI_REG8(0x0340), 0x0e },
	{ CCI_REG8(0x0341), 0x2c },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x1f },
	{ CCI_REG8(0x0349), 0xff },
	{ CCI_REG8(0x034a), 0x17 },
	{ CCI_REG8(0x034b), 0xff },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0902), 0x08 },
	{ CCI_REG8(0x3005), 0x02 },
	{ CCI_REG8(0x3120), 0x04 },
	{ CCI_REG8(0x3121), 0x01 },
	{ CCI_REG8(0x3200), 0x41 },
	{ CCI_REG8(0x3201), 0x41 },
	{ CCI_REG8(0x32d6), 0x00 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x10 },
	{ CCI_REG8(0x040d), 0x00 },
	{ CCI_REG8(0x040e), 0x0c },
	{ CCI_REG8(0x040f), 0x00 },
	{ CCI_REG8(0x034c), 0x10 },
	{ CCI_REG8(0x034d), 0x00 },
	{ CCI_REG8(0x034e), 0x0c },
	{ CCI_REG8(0x034f), 0x00 },
	{ CCI_REG8(0x0301), 0x05 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x03 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0xa6 },
	{ CCI_REG8(0x030b), 0x02 },
	{ CCI_REG8(0x030d), 0x02 },
	{ CCI_REG8(0x030e), 0x01 },
	{ CCI_REG8(0x030f), 0x27 },
	{ CCI_REG8(0x30cb), 0x00 },
	{ CCI_REG8(0x30cc), 0x10 },
	{ CCI_REG8(0x30cd), 0x00 },
	{ CCI_REG8(0x30ce), 0x03 },
	{ CCI_REG8(0x30cf), 0x00 },
	{ CCI_REG8(0x319c), 0x01 },
	{ CCI_REG8(0x3800), 0x01 },
	{ CCI_REG8(0x3801), 0x01 },
	{ CCI_REG8(0x3802), 0x02 },
	{ CCI_REG8(0x3847), 0x03 },
	{ CCI_REG8(0x38b0), 0x00 },
	{ CCI_REG8(0x38b1), 0x00 },
	{ CCI_REG8(0x38b2), 0x00 },
	{ CCI_REG8(0x38b3), 0x00 },
	{ CCI_REG8(0x38c4), 0x01 },
	{ CCI_REG8(0x38c5), 0x2c },
	{ CCI_REG8(0x38e0), 0x00 },
	{ CCI_REG8(0x38e1), 0x00 },
	{ CCI_REG8(0x8be4), 0x00 },
	{ CCI_REG8(0x8be5), 0x00 },
	{ CCI_REG8(0x3890), 0x00 },
	{ CCI_REG8(0x3891), 0x00 },
	{ CCI_REG8(0x3894), 0x00 },
	{ CCI_REG8(0x3895), 0x00 },
	{ CCI_REG8(0x4c3a), 0x02 },
	{ CCI_REG8(0x4c3b), 0xd2 },
	{ CCI_REG8(0x4c68), 0x04 },
	{ CCI_REG8(0x4c69), 0x7e },
	{ CCI_REG8(0x4cf8), 0x07 },
	{ CCI_REG8(0x4cf9), 0x9e },
	{ CCI_REG8(0x4db8), 0x08 },
	{ CCI_REG8(0x4db9), 0x98 },
	{ CCI_REG8(0x0202), 0x0d },
	{ CCI_REG8(0x0203), 0xfc },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x313a), 0x01 },
	{ CCI_REG8(0x313b), 0xf4 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x17 },
	{ CCI_REG8(0x3805), 0xc0 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x313c), 0x00 },
	{ CCI_REG8(0x313d), 0x00 },
	{ CCI_REG8(0x313e), 0x01 },
	{ CCI_REG8(0x313f), 0x00 },
	{ CCI_REG8(0x30b4), 0x01 },
	{ CCI_REG8(0x3066), 0x00 },
	{ CCI_REG8(0x3067), 0x30 },
	{ CCI_REG8(0x3068), 0x00 },
	{ CCI_REG8(0x3069), 0x30 },
	{ CCI_REG8(0x33d0), 0x00 },
	{ CCI_REG8(0x33d1), 0x00 },
	{ CCI_REG8(0x33d4), 0x01 },
	{ CCI_REG8(0x33dc), 0x0a },
	{ CCI_REG8(0x33dd), 0x0a },
	{ CCI_REG8(0x33de), 0x0a },
	{ CCI_REG8(0x33df), 0x0a },
	{ CCI_REG8(0x3070), 0x01 },
	{ CCI_REG8(0x3077), 0x01 },
	{ CCI_REG8(0x3078), 0x30 },
	{ CCI_REG8(0x3079), 0x01 },
	{ CCI_REG8(0x307a), 0x30 },
	{ CCI_REG8(0x307b), 0x01 },
	{ CCI_REG8(0x3080), 0x02 },
	{ CCI_REG8(0x3087), 0x02 },
	{ CCI_REG8(0x3088), 0x30 },
	{ CCI_REG8(0x3089), 0x02 },
	{ CCI_REG8(0x308a), 0x30 },
	{ CCI_REG8(0x308b), 0x02 },
	{ CCI_REG8(0x3901), 0x2b },
	{ CCI_REG8(0x3902), 0x00 },
	{ CCI_REG8(0x3903), 0x12 },
	{ CCI_REG8(0x3905), 0x2b },
	{ CCI_REG8(0x3906), 0x01 },
	{ CCI_REG8(0x3907), 0x12 },
	{ CCI_REG8(0x3909), 0x2b },
	{ CCI_REG8(0x390a), 0x02 },
	{ CCI_REG8(0x390b), 0x12 },
	{ CCI_REG8(0x3911), 0x00 },
	{ CCI_REG8(0x0110), 0x00 },
};

static const struct imx890_mode imx890_supported_modes[] = {
	{
		/* 2x2 binned full field of view, 30.00 fps */
		.width = 4096,
		.height = 3072,
		.hts = 15616,
		.vts = 3628,
		.exposure = 3580,
		.reg_list = {
			.regs = imx890_4096x3072_mode,
			.num_regs = ARRAY_SIZE(imx890_4096x3072_mode),
		},
	},
};

static int imx890_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx890 *imx890 = container_of(ctrl->handler, struct imx890,
					     ctrl_handler);
	const struct imx890_mode *mode = imx890->mode;
	s64 exposure_max;
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = mode->height + ctrl->val - IMX890_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(imx890->exposure,
					 imx890->exposure->minimum,
					 exposure_max,
					 imx890->exposure->step,
					 imx890->exposure->default_value);
	}

	if (!pm_runtime_get_if_active(imx890->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(imx890->regmap, IMX890_REG_AGAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(imx890->regmap, IMX890_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(imx890->regmap, IMX890_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(imx890->regmap, IMX890_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx890->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx890_ctrl_ops = {
	.s_ctrl = imx890_set_ctrl,
};

static int imx890_init_controls(struct imx890 *imx890)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &imx890->ctrl_handler;
	const struct imx890_mode *mode = imx890->mode;
	s64 hblank, vblank, exposure_max;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 9);

	imx890->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx890_ctrl_ops,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(imx890_link_freq_menu) - 1,
					0, imx890_link_freq_menu);
	if (imx890->link_freq)
		imx890->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx890->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx890_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX890_PIXEL_RATE,
					       IMX890_PIXEL_RATE, 1,
					       IMX890_PIXEL_RATE);

	hblank = mode->hts - mode->width;
	imx890->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx890_ctrl_ops,
					   V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (imx890->hblank)
		imx890->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	imx890->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx890_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   IMX890_VTS_MAX - mode->height, 1,
					   vblank);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx890_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX890_AGAIN_MIN, IMX890_AGAIN_MAX,
			  IMX890_AGAIN_STEP, IMX890_AGAIN_DEFAULT);

	exposure_max = mode->vts - IMX890_EXPOSURE_MARGIN;
	imx890->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx890_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX890_EXPOSURE_MIN,
					     exposure_max,
					     IMX890_EXPOSURE_STEP,
					     mode->exposure);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx890_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx890_test_pattern_menu) - 1,
				     0, 0, imx890_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(imx890->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx890_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	imx890->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int imx890_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx890 *imx890 = to_imx890(sd);
	const struct imx890_reg_list *reg_list = &imx890->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(imx890->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(imx890->regmap, imx890_init,
				  ARRAY_SIZE(imx890_init), NULL);
	cci_multi_reg_write(imx890->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(imx890->sd.ctrl_handler);

	cci_write(imx890->regmap, IMX890_REG_MODE_SELECT,
		  IMX890_MODE_STREAMING, &ret);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(imx890->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(imx890->dev);

	return ret;
}

static int imx890_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx890 *imx890 = to_imx890(sd);
	int ret;

	ret = cci_write(imx890->regmap, IMX890_REG_MODE_SELECT, 0, NULL);
	if (ret)
		dev_err(imx890->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(imx890->dev);

	return ret;
}

static void imx890_update_pad_format(const struct imx890_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx890_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx890 *imx890 = to_imx890(sd);
	s64 hblank, vblank, exposure_max;
	const struct imx890_mode *mode;

	mode = v4l2_find_nearest_size(imx890_supported_modes,
				      ARRAY_SIZE(imx890_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	imx890_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || imx890->mode == mode)
		goto set_format;

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(imx890->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(imx890->vblank, vblank,
				 IMX890_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(imx890->vblank, vblank);

	exposure_max = mode->vts - IMX890_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(imx890->exposure, IMX890_EXPOSURE_MIN,
				 exposure_max, IMX890_EXPOSURE_STEP,
				 mode->exposure);
	__v4l2_ctrl_s_ctrl(imx890->exposure, mode->exposure);

	if (imx890->sd.ctrl_handler->error)
		return imx890->sd.ctrl_handler->error;

	imx890->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int imx890_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int imx890_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(imx890_supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = imx890_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx890_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx890_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		/* The binned mode reads the full 8192x6144 array */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = 8192;
		sel->r.height = 6144;
		return 0;
	default:
		return -EINVAL;
	}
}

static int imx890_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct imx890 *imx890 = to_imx890(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = imx890->mode->width,
			.height = imx890->mode->height,
		},
	};

	imx890_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops imx890_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx890_pad_ops = {
	.set_fmt = imx890_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = imx890_get_selection,
	.enum_mbus_code = imx890_enum_mbus_code,
	.enum_frame_size = imx890_enum_frame_size,
	.enable_streams = imx890_enable_streams,
	.disable_streams = imx890_disable_streams,
};

static const struct v4l2_subdev_ops imx890_subdev_ops = {
	.video = &imx890_video_ops,
	.pad = &imx890_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx890_internal_ops = {
	.init_state = imx890_init_state,
};

static const struct media_entity_operations imx890_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx890_identify_sensor(struct imx890 *imx890)
{
	u64 val;
	int ret;

	ret = cci_read(imx890->regmap, IMX890_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(imx890->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != IMX890_CHIP_ID) {
		dev_err(imx890->dev, "chip id mismatch: %x!=%llx\n",
			IMX890_CHIP_ID, val);
		return -ENODEV;
	}

	return 0;
}

static int imx890_check_hwcfg(struct imx890 *imx890)
{
	struct fwnode_handle *fwnode = dev_fwnode(imx890->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_CPHY,
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX890_DATA_TRIOS) {
		dev_err(imx890->dev, "Invalid number of trios: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(imx890->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       imx890_link_freq_menu,
				       ARRAY_SIZE(imx890_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* Stock order: rails, MCLK, then reset release, 1 ms apart */
static int imx890_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx890 *imx890 = to_imx890(sd);
	int ret;

	ret = regulator_bulk_enable(IMX890_NUM_SUPPLIES, imx890->supplies);
	if (ret)
		return ret;

	usleep_range(1000, 1500);

	ret = clk_prepare_enable(imx890->mclk);
	if (ret)
		goto disable_regulators;

	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(imx890->reset_gpio, 0);
	usleep_range(2000, 2500);

	return 0;

disable_regulators:
	regulator_bulk_disable(IMX890_NUM_SUPPLIES, imx890->supplies);

	return ret;
}

static int imx890_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx890 *imx890 = to_imx890(sd);

	gpiod_set_value_cansleep(imx890->reset_gpio, 1);
	usleep_range(1000, 1500);
	clk_disable_unprepare(imx890->mclk);

	regulator_bulk_disable(IMX890_NUM_SUPPLIES, imx890->supplies);

	return 0;
}

static int imx890_probe(struct i2c_client *client)
{
	struct imx890 *imx890;
	unsigned long freq;
	unsigned int i;
	int ret;

	imx890 = devm_kzalloc(&client->dev, sizeof(*imx890), GFP_KERNEL);
	if (!imx890)
		return -ENOMEM;

	imx890->dev = &client->dev;
	v4l2_i2c_subdev_init(&imx890->sd, client, &imx890_subdev_ops);

	imx890->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx890->regmap))
		return dev_err_probe(imx890->dev, PTR_ERR(imx890->regmap),
				     "failed to init CCI\n");

	imx890->mclk = devm_v4l2_sensor_clk_get(imx890->dev, NULL);
	if (IS_ERR(imx890->mclk))
		return dev_err_probe(imx890->dev, PTR_ERR(imx890->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(imx890->mclk);
	if (freq != IMX890_MCLK_FREQ_19P2MHZ)
		return dev_err_probe(imx890->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = imx890_check_hwcfg(imx890);
	if (ret)
		return dev_err_probe(imx890->dev, ret,
				     "failed to check HW configuration\n");

	imx890->reset_gpio = devm_gpiod_get_optional(imx890->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx890->reset_gpio))
		return dev_err_probe(imx890->dev, PTR_ERR(imx890->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < IMX890_NUM_SUPPLIES; i++)
		imx890->supplies[i].supply = imx890_supply_names[i];

	ret = devm_regulator_bulk_get(imx890->dev, IMX890_NUM_SUPPLIES,
				      imx890->supplies);
	if (ret)
		return dev_err_probe(imx890->dev, ret,
				     "failed to get supply regulators\n");

	ret = imx890_power_on(imx890->dev);
	if (ret)
		return ret;

	ret = imx890_identify_sensor(imx890);
	if (ret) {
		dev_err_probe(imx890->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	imx890->mode = &imx890_supported_modes[0];
	ret = imx890_init_controls(imx890);
	if (ret) {
		dev_err_probe(imx890->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	imx890->sd.state_lock = imx890->ctrl_handler.lock;
	imx890->sd.internal_ops = &imx890_internal_ops;
	imx890->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx890->sd.entity.ops = &imx890_subdev_entity_ops;
	imx890->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx890->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx890->sd.entity, 1, &imx890->pad);
	if (ret) {
		dev_err_probe(imx890->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&imx890->sd);
	if (ret < 0) {
		dev_err_probe(imx890->dev, ret, "failed to init subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(imx890->dev);
	pm_runtime_enable(imx890->dev);

	ret = v4l2_async_register_subdev_sensor(&imx890->sd);
	if (ret < 0) {
		dev_err_probe(imx890->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(imx890->dev, 1000);
	pm_runtime_use_autosuspend(imx890->dev);
	pm_runtime_idle(imx890->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&imx890->sd);
	pm_runtime_disable(imx890->dev);
	pm_runtime_set_suspended(imx890->dev);

media_entity_cleanup:
	media_entity_cleanup(&imx890->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(imx890->sd.ctrl_handler);

power_off:
	imx890_power_off(imx890->dev);

	return ret;
}

static void imx890_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx890 *imx890 = to_imx890(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(imx890->dev);

	if (!pm_runtime_status_suspended(imx890->dev)) {
		imx890_power_off(imx890->dev);
		pm_runtime_set_suspended(imx890->dev);
	}
}

static const struct dev_pm_ops imx890_pm_ops = {
	SET_RUNTIME_PM_OPS(imx890_power_off, imx890_power_on, NULL)
};

static const struct of_device_id imx890_of_match[] = {
	{ .compatible = "sony,imx890" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx890_of_match);

static struct i2c_driver imx890_i2c_driver = {
	.driver = {
		.name = "imx890",
		.pm = &imx890_pm_ops,
		.of_match_table = imx890_of_match,
	},
	.probe = imx890_probe,
	.remove = imx890_remove,
};

module_i2c_driver(imx890_i2c_driver);

MODULE_DESCRIPTION("Sony IMX890 image sensor driver");
MODULE_LICENSE("GPL");
