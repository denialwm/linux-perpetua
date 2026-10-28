// SPDX-License-Identifier: GPL-2.0
/*
 * OmniVision OV50E40 image sensor driver
 *
 * Register tables are the Nubia RedMagic 11 Pro (nx809j) stock module
 * settings for its main camera. Driver structure follows imx890.c.
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
 * The MIPI PLL makes 2816 Msymbol/s per trio from the 24 MHz MCLK (x704,
 * /6), a 1408 MHz C-PHY link frequency. Three trios carry 16 bits per 7
 * symbols each: 1931 Mpixel/s of 10 bits.
 */
#define OV50E40_LINK_FREQ_1408MHZ	(1408 * HZ_PER_MHZ)
#define OV50E40_PIXEL_RATE		(OV50E40_LINK_FREQ_1408MHZ * 2 * \
					 OV50E40_TRIOS * 16 / 7 / 10)
#define OV50E40_MCLK_FREQ_24MHZ		(24 * HZ_PER_MHZ)
#define OV50E40_TRIOS			3

#define OV50E40_REG_CHIP_ID		CCI_REG16(0x300b)
#define OV50E40_CHIP_ID			0x5045

#define OV50E40_REG_SOFTWARE_RESET	CCI_REG8(0x0103)

#define OV50E40_REG_MODE_SELECT		CCI_REG8(0x0100)
#define OV50E40_MODE_STREAMING		BIT(0)

#define OV50E40_REG_EXPOSURE		CCI_REG16(0x3501)
#define OV50E40_EXPOSURE_MIN		4
#define OV50E40_EXPOSURE_STEP		1
#define OV50E40_EXPOSURE_MARGIN		32

/*
 * Analogue gain, 1/1024 steps from 0x0400 = 1x; measured linear up to
 * 0x4000 = 16x. Codes below 0x0400 read as 1/256 steps instead.
 */
#define OV50E40_REG_AGAIN		CCI_REG16(0x3508)
#define OV50E40_AGAIN_MIN		0x0400
#define OV50E40_AGAIN_MAX		0x4000
#define OV50E40_AGAIN_STEP		1
#define OV50E40_AGAIN_DEFAULT		0x0400

#define OV50E40_REG_VTS			CCI_REG16(0x380e)
#define OV50E40_VTS_MAX			0x7fff

#define to_ov50e40(_sd)			container_of(_sd, struct ov50e40, sd)

static const s64 ov50e40_link_freq_menu[] = {
	OV50E40_LINK_FREQ_1408MHZ,
};

struct ov50e40_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct ov50e40_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 exposure;

	const struct ov50e40_reg_list reg_list;
};

/* Stock power-up order */
static const char * const ov50e40_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OV50E40_NUM_SUPPLIES	ARRAY_SIZE(ov50e40_supply_names)

struct ov50e40 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[OV50E40_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;

	const struct ov50e40_mode *mode;
};

/* Global init, after the software reset */
static const struct cci_reg_sequence ov50e40_init[] = {
	{ CCI_REG8(0x0102), 0x01 },
	{ CCI_REG8(0x0304), 0x03 },
	{ CCI_REG8(0x0305), 0x6b },
	{ CCI_REG8(0x0306), 0x03 },
	{ CCI_REG8(0x0307), 0x00 },
	{ CCI_REG8(0x0323), 0x12 },
	{ CCI_REG8(0x0324), 0x01 },
	{ CCI_REG8(0x0325), 0xf4 },
	{ CCI_REG8(0x0326), 0xd4 },
	{ CCI_REG8(0x0327), 0x0b },
	{ CCI_REG8(0x0343), 0x00 },
	{ CCI_REG8(0x0345), 0x20 },
	{ CCI_REG8(0x034a), 0x07 },
	{ CCI_REG8(0x300d), 0x11 },
	{ CCI_REG8(0x300e), 0x11 },
	{ CCI_REG8(0x3012), 0x31 },
	{ CCI_REG8(0x3025), 0x03 },
	{ CCI_REG8(0x3027), 0x2c },
	{ CCI_REG8(0x3107), 0xc9 },
	{ CCI_REG8(0x3201), 0x02 },
	{ CCI_REG8(0x3202), 0x20 },
	{ CCI_REG8(0x3203), 0x40 },
	{ CCI_REG8(0x3400), 0x04 },
	{ CCI_REG8(0x340e), 0x40 },
	{ CCI_REG8(0x3421), 0x01 },
	{ CCI_REG8(0x3424), 0x00 },
	{ CCI_REG8(0x3425), 0x00 },
	{ CCI_REG8(0x3426), 0x00 },
	{ CCI_REG8(0x3427), 0xa0 },
	{ CCI_REG8(0x3428), 0x10 },
	{ CCI_REG8(0x3429), 0x05 },
	{ CCI_REG8(0x342a), 0x00 },
	{ CCI_REG8(0x342b), 0x00 },
	{ CCI_REG8(0x3504), 0x0c },
	{ CCI_REG8(0x3506), 0x58 },
	{ CCI_REG8(0x3508), 0x01 },
	{ CCI_REG8(0x3509), 0x00 },
	{ CCI_REG8(0x350d), 0x01 },
	{ CCI_REG8(0x350e), 0x00 },
	{ CCI_REG8(0x350f), 0x00 },
	{ CCI_REG8(0x3544), 0x0c },
	{ CCI_REG8(0x3546), 0x38 },
	{ CCI_REG8(0x3548), 0x0f },
	{ CCI_REG8(0x3549), 0xff },
	{ CCI_REG8(0x3584), 0x0c },
	{ CCI_REG8(0x3586), 0x38 },
	{ CCI_REG8(0x3508), 0x0f },
	{ CCI_REG8(0x3509), 0xff },
	{ CCI_REG8(0x3600), 0x07 },
	{ CCI_REG8(0x3601), 0x38 },
	{ CCI_REG8(0x3604), 0x38 },
	{ CCI_REG8(0x3605), 0x38 },
	{ CCI_REG8(0x3608), 0xf0 },
	{ CCI_REG8(0x3609), 0x46 },
	{ CCI_REG8(0x360a), 0xbf },
	{ CCI_REG8(0x360c), 0x50 },
	{ CCI_REG8(0x360e), 0x07 },
	{ CCI_REG8(0x3616), 0x58 },
	{ CCI_REG8(0x3619), 0x44 },
	{ CCI_REG8(0x361a), 0x56 },
	{ CCI_REG8(0x361b), 0x40 },
	{ CCI_REG8(0x361c), 0x08 },
	{ CCI_REG8(0x361d), 0x0c },
	{ CCI_REG8(0x361e), 0x16 },
	{ CCI_REG8(0x3622), 0xb9 },
	{ CCI_REG8(0x3623), 0x83 },
	{ CCI_REG8(0x3625), 0x84 },
	{ CCI_REG8(0x3626), 0xc2 },
	{ CCI_REG8(0x363b), 0x14 },
	{ CCI_REG8(0x363c), 0xba },
	{ CCI_REG8(0x3642), 0x97 },
	{ CCI_REG8(0x3643), 0x01 },
	{ CCI_REG8(0x3645), 0x14 },
	{ CCI_REG8(0x3647), 0xc1 },
	{ CCI_REG8(0x3648), 0x8e },
	{ CCI_REG8(0x364d), 0x0f },
	{ CCI_REG8(0x3680), 0x00 },
	{ CCI_REG8(0x3682), 0x80 },
	{ CCI_REG8(0x3683), 0x00 },
	{ CCI_REG8(0x3684), 0x00 },
	{ CCI_REG8(0x3688), 0x03 },
	{ CCI_REG8(0x369a), 0x00 },
	{ CCI_REG8(0x36a5), 0x08 },
	{ CCI_REG8(0x36d0), 0x00 },
	{ CCI_REG8(0x3700), 0x1a },
	{ CCI_REG8(0x3701), 0x13 },
	{ CCI_REG8(0x3702), 0x2c },
	{ CCI_REG8(0x3703), 0x31 },
	{ CCI_REG8(0x3704), 0x03 },
	{ CCI_REG8(0x3706), 0x20 },
	{ CCI_REG8(0x3707), 0x04 },
	{ CCI_REG8(0x3708), 0x25 },
	{ CCI_REG8(0x3709), 0x9d },
	{ CCI_REG8(0x370b), 0x3e },
	{ CCI_REG8(0x370c), 0x04 },
	{ CCI_REG8(0x3712), 0x51 },
	{ CCI_REG8(0x3714), 0x74 },
	{ CCI_REG8(0x3715), 0x00 },
	{ CCI_REG8(0x3724), 0x28 },
	{ CCI_REG8(0x372a), 0x00 },
	{ CCI_REG8(0x372b), 0x00 },
	{ CCI_REG8(0x372e), 0x1a },
	{ CCI_REG8(0x372f), 0x13 },
	{ CCI_REG8(0x3734), 0x1a },
	{ CCI_REG8(0x3740), 0x20 },
	{ CCI_REG8(0x3741), 0x80 },
	{ CCI_REG8(0x3743), 0xe0 },
	{ CCI_REG8(0x374f), 0x50 },
	{ CCI_REG8(0x3754), 0x2c },
	{ CCI_REG8(0x3755), 0x70 },
	{ CCI_REG8(0x3756), 0x00 },
	{ CCI_REG8(0x3757), 0x30 },
	{ CCI_REG8(0x3758), 0x00 },
	{ CCI_REG8(0x3759), 0x50 },
	{ CCI_REG8(0x375e), 0x00 },
	{ CCI_REG8(0x375f), 0x02 },
	{ CCI_REG8(0x3763), 0x08 },
	{ CCI_REG8(0x3767), 0x28 },
	{ CCI_REG8(0x376c), 0x00 },
	{ CCI_REG8(0x3780), 0x4c },
	{ CCI_REG8(0x3781), 0x00 },
	{ CCI_REG8(0x3782), 0x01 },
	{ CCI_REG8(0x3785), 0x01 },
	{ CCI_REG8(0x3786), 0x01 },
	{ CCI_REG8(0x3789), 0x01 },
	{ CCI_REG8(0x378a), 0x01 },
	{ CCI_REG8(0x378d), 0x01 },
	{ CCI_REG8(0x378e), 0x01 },
	{ CCI_REG8(0x3791), 0x20 },
	{ CCI_REG8(0x3793), 0x1c },
	{ CCI_REG8(0x3795), 0x1c },
	{ CCI_REG8(0x3797), 0x3e },
	{ CCI_REG8(0x3799), 0x3c },
	{ CCI_REG8(0x379b), 0x3c },
	{ CCI_REG8(0x379f), 0x01 },
	{ CCI_REG8(0x37a0), 0x70 },
	{ CCI_REG8(0x37cb), 0x03 },
	{ CCI_REG8(0x37d0), 0x00 },
	{ CCI_REG8(0x37d1), 0x00 },
	{ CCI_REG8(0x37db), 0x0f },
	{ CCI_REG8(0x37dc), 0x19 },
	{ CCI_REG8(0x37e3), 0x30 },
	{ CCI_REG8(0x37f8), 0x25 },
	{ CCI_REG8(0x3800), 0x00 },
	{ CCI_REG8(0x3801), 0x00 },
	{ CCI_REG8(0x3802), 0x00 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x20 },
	{ CCI_REG8(0x3805), 0x1f },
	{ CCI_REG8(0x3806), 0x18 },
	{ CCI_REG8(0x3807), 0x1f },
	{ CCI_REG8(0x3808), 0x20 },
	{ CCI_REG8(0x3809), 0x00 },
	{ CCI_REG8(0x380a), 0x18 },
	{ CCI_REG8(0x380b), 0x00 },
	{ CCI_REG8(0x380c), 0x01 },
	{ CCI_REG8(0x380d), 0x90 },
	{ CCI_REG8(0x380e), 0x18 },
	{ CCI_REG8(0x380f), 0x6c },
	{ CCI_REG8(0x3810), 0x00 },
	{ CCI_REG8(0x3811), 0x0f },
	{ CCI_REG8(0x3812), 0x00 },
	{ CCI_REG8(0x3813), 0x10 },
	{ CCI_REG8(0x3814), 0x11 },
	{ CCI_REG8(0x3815), 0x11 },
	{ CCI_REG8(0x3820), 0x44 },
	{ CCI_REG8(0x3821), 0x00 },
	{ CCI_REG8(0x3822), 0x00 },
	{ CCI_REG8(0x3823), 0x04 },
	{ CCI_REG8(0x3828), 0x14 },
	{ CCI_REG8(0x382a), 0x80 },
	{ CCI_REG8(0x382e), 0x71 },
	{ CCI_REG8(0x3830), 0x1c },
	{ CCI_REG8(0x3831), 0x00 },
	{ CCI_REG8(0x3837), 0x1c },
	{ CCI_REG8(0x383f), 0x08 },
	{ CCI_REG8(0x384c), 0x01 },
	{ CCI_REG8(0x384d), 0x90 },
	{ CCI_REG8(0x3858), 0x00 },
	{ CCI_REG8(0x3888), 0x00 },
	{ CCI_REG8(0x3889), 0x10 },
	{ CCI_REG8(0x388a), 0x00 },
	{ CCI_REG8(0x388b), 0x10 },
	{ CCI_REG8(0x388c), 0x20 },
	{ CCI_REG8(0x388d), 0x00 },
	{ CCI_REG8(0x388e), 0x18 },
	{ CCI_REG8(0x388f), 0x00 },
	{ CCI_REG8(0x3896), 0x00 },
	{ CCI_REG8(0x3900), 0x40 },
	{ CCI_REG8(0x3903), 0x01 },
	{ CCI_REG8(0x3904), 0xf7 },
	{ CCI_REG8(0x3905), 0xd4 },
	{ CCI_REG8(0x3906), 0x00 },
	{ CCI_REG8(0x3907), 0x00 },
	{ CCI_REG8(0x3917), 0x00 },
	{ CCI_REG8(0x3918), 0x00 },
	{ CCI_REG8(0x391c), 0xba },
	{ CCI_REG8(0x3924), 0xaa },
	{ CCI_REG8(0x3939), 0xc0 },
	{ CCI_REG8(0x393a), 0xff },
	{ CCI_REG8(0x393b), 0xc0 },
	{ CCI_REG8(0x394e), 0x01 },
	{ CCI_REG8(0x394f), 0x01 },
	{ CCI_REG8(0x3981), 0x01 },
	{ CCI_REG8(0x3982), 0x20 },
	{ CCI_REG8(0x398a), 0x01 },
	{ CCI_REG8(0x398b), 0x80 },
	{ CCI_REG8(0x3992), 0x03 },
	{ CCI_REG8(0x3993), 0x01 },
	{ CCI_REG8(0x399d), 0x07 },
	{ CCI_REG8(0x399e), 0x01 },
	{ CCI_REG8(0x39ec), 0x06 },
	{ CCI_REG8(0x39ed), 0x04 },
	{ CCI_REG8(0x39fb), 0x1a },
	{ CCI_REG8(0x39fc), 0x1a },
	{ CCI_REG8(0x39fd), 0x1a },
	{ CCI_REG8(0x39fe), 0x1a },
	{ CCI_REG8(0x3a0d), 0x00 },
	{ CCI_REG8(0x3a13), 0x00 },
	{ CCI_REG8(0x3a14), 0x04 },
	{ CCI_REG8(0x3a15), 0x01 },
	{ CCI_REG8(0x3a16), 0x02 },
	{ CCI_REG8(0x3a17), 0x00 },
	{ CCI_REG8(0x3a1d), 0x1a },
	{ CCI_REG8(0x3a1e), 0x1a },
	{ CCI_REG8(0x3a21), 0x1a },
	{ CCI_REG8(0x3a22), 0x1a },
	{ CCI_REG8(0x3a2a), 0x01 },
	{ CCI_REG8(0x3a2e), 0x01 },
	{ CCI_REG8(0x3a2f), 0x01 },
	{ CCI_REG8(0x3a30), 0x01 },
	{ CCI_REG8(0x3a31), 0x01 },
	{ CCI_REG8(0x3a32), 0x01 },
	{ CCI_REG8(0x3a34), 0x01 },
	{ CCI_REG8(0x3a35), 0x01 },
	{ CCI_REG8(0x3a36), 0x01 },
	{ CCI_REG8(0x3a37), 0x01 },
	{ CCI_REG8(0x3a40), 0x01 },
	{ CCI_REG8(0x3a42), 0x01 },
	{ CCI_REG8(0x3a43), 0x01 },
	{ CCI_REG8(0x3a44), 0x01 },
	{ CCI_REG8(0x3a45), 0x01 },
	{ CCI_REG8(0x3a46), 0x01 },
	{ CCI_REG8(0x3a48), 0x01 },
	{ CCI_REG8(0x3a49), 0x01 },
	{ CCI_REG8(0x3a4a), 0x01 },
	{ CCI_REG8(0x3a4b), 0x01 },
	{ CCI_REG8(0x3a5e), 0x50 },
	{ CCI_REG8(0x3a68), 0x07 },
	{ CCI_REG8(0x3a69), 0x34 },
	{ CCI_REG8(0x3a78), 0x03 },
	{ CCI_REG8(0x3a79), 0x03 },
	{ CCI_REG8(0x3a7c), 0x04 },
	{ CCI_REG8(0x3a7d), 0x04 },
	{ CCI_REG8(0x3ab1), 0x5b },
	{ CCI_REG8(0x3ab6), 0x1a },
	{ CCI_REG8(0x3ab7), 0x1a },
	{ CCI_REG8(0x3ab8), 0x1a },
	{ CCI_REG8(0x3ab9), 0x1a },
	{ CCI_REG8(0x3aed), 0x3a },
	{ CCI_REG8(0x3af1), 0x3f },
	{ CCI_REG8(0x3afd), 0x04 },
	{ CCI_REG8(0x3afe), 0x09 },
	{ CCI_REG8(0x3aff), 0x04 },
	{ CCI_REG8(0x3b00), 0x09 },
	{ CCI_REG8(0x3b01), 0x00 },
	{ CCI_REG8(0x3b02), 0x00 },
	{ CCI_REG8(0x3b03), 0x07 },
	{ CCI_REG8(0x3b04), 0x07 },
	{ CCI_REG8(0x3b05), 0x07 },
	{ CCI_REG8(0x3b08), 0x07 },
	{ CCI_REG8(0x3b14), 0x02 },
	{ CCI_REG8(0x3b15), 0x07 },
	{ CCI_REG8(0x3b19), 0x07 },
	{ CCI_REG8(0x3b1a), 0x07 },
	{ CCI_REG8(0x3b1d), 0x00 },
	{ CCI_REG8(0x3b22), 0x07 },
	{ CCI_REG8(0x3b25), 0x07 },
	{ CCI_REG8(0x3b2b), 0x00 },
	{ CCI_REG8(0x3b2c), 0x00 },
	{ CCI_REG8(0x3b2d), 0x04 },
	{ CCI_REG8(0x3b32), 0x07 },
	{ CCI_REG8(0x3b33), 0x07 },
	{ CCI_REG8(0x3b34), 0x07 },
	{ CCI_REG8(0x3b38), 0x07 },
	{ CCI_REG8(0x3b39), 0x07 },
	{ CCI_REG8(0x3b3a), 0x07 },
	{ CCI_REG8(0x3b3b), 0x07 },
	{ CCI_REG8(0x3b3c), 0x00 },
	{ CCI_REG8(0x3b3d), 0x00 },
	{ CCI_REG8(0x3b41), 0x07 },
	{ CCI_REG8(0x3b42), 0x07 },
	{ CCI_REG8(0x3b43), 0x07 },
	{ CCI_REG8(0x3b44), 0x07 },
	{ CCI_REG8(0x3b45), 0x00 },
	{ CCI_REG8(0x3b49), 0x00 },
	{ CCI_REG8(0x3b56), 0x11 },
	{ CCI_REG8(0x3b57), 0x11 },
	{ CCI_REG8(0x3b58), 0x11 },
	{ CCI_REG8(0x3b59), 0x11 },
	{ CCI_REG8(0x3b5a), 0x11 },
	{ CCI_REG8(0x3b5b), 0x11 },
	{ CCI_REG8(0x3b5c), 0x11 },
	{ CCI_REG8(0x3b5d), 0x11 },
	{ CCI_REG8(0x3b82), 0x11 },
	{ CCI_REG8(0x3ba0), 0x07 },
	{ CCI_REG8(0x3ba1), 0x11 },
	{ CCI_REG8(0x3ba2), 0x07 },
	{ CCI_REG8(0x3ba3), 0x0e },
	{ CCI_REG8(0x3ba4), 0x77 },
	{ CCI_REG8(0x3ba5), 0x77 },
	{ CCI_REG8(0x3ba6), 0x77 },
	{ CCI_REG8(0x3ba7), 0x77 },
	{ CCI_REG8(0x3baa), 0x38 },
	{ CCI_REG8(0x3bab), 0x2c },
	{ CCI_REG8(0x3bac), 0x77 },
	{ CCI_REG8(0x3baf), 0x58 },
	{ CCI_REG8(0x3bba), 0x58 },
	{ CCI_REG8(0x3bca), 0x1a },
	{ CCI_REG8(0x3beb), 0x00 },
	{ CCI_REG8(0x3bf2), 0x33 },
	{ CCI_REG8(0x3bfa), 0x00 },
	{ CCI_REG8(0x3bff), 0x08 },
	{ CCI_REG8(0x3d8c), 0x9b },
	{ CCI_REG8(0x3d8d), 0xa0 },
	{ CCI_REG8(0x4008), 0x00 },
	{ CCI_REG8(0x4009), 0x02 },
	{ CCI_REG8(0x4010), 0x28 },
	{ CCI_REG8(0x4012), 0xfc },
	{ CCI_REG8(0x4015), 0x00 },
	{ CCI_REG8(0x4016), 0x1f },
	{ CCI_REG8(0x4017), 0x00 },
	{ CCI_REG8(0x4018), 0x0f },
	{ CCI_REG8(0x401b), 0x04 },
	{ CCI_REG8(0x401e), 0x01 },
	{ CCI_REG8(0x401f), 0x30 },
	{ CCI_REG8(0x4030), 0x00 },
	{ CCI_REG8(0x4031), 0x00 },
	{ CCI_REG8(0x4032), 0x00 },
	{ CCI_REG8(0x4033), 0x00 },
	{ CCI_REG8(0x4034), 0x00 },
	{ CCI_REG8(0x4035), 0x00 },
	{ CCI_REG8(0x4036), 0x00 },
	{ CCI_REG8(0x4037), 0x00 },
	{ CCI_REG8(0x4038), 0x00 },
	{ CCI_REG8(0x4039), 0x00 },
	{ CCI_REG8(0x403a), 0x00 },
	{ CCI_REG8(0x403b), 0x00 },
	{ CCI_REG8(0x403c), 0x00 },
	{ CCI_REG8(0x403d), 0x00 },
	{ CCI_REG8(0x403e), 0x00 },
	{ CCI_REG8(0x403f), 0x00 },
	{ CCI_REG8(0x4040), 0x00 },
	{ CCI_REG8(0x4041), 0x80 },
	{ CCI_REG8(0x4042), 0x0f },
	{ CCI_REG8(0x4043), 0x80 },
	{ CCI_REG8(0x4044), 0x00 },
	{ CCI_REG8(0x4045), 0x90 },
	{ CCI_REG8(0x4046), 0x0f },
	{ CCI_REG8(0x4047), 0x70 },
	{ CCI_REG8(0x40c4), 0x07 },
	{ CCI_REG8(0x40c5), 0xff },
	{ CCI_REG8(0x40c6), 0x07 },
	{ CCI_REG8(0x40c7), 0xff },
	{ CCI_REG8(0x4288), 0x07 },
	{ CCI_REG8(0x4505), 0x0c },
	{ CCI_REG8(0x4506), 0x01 },
	{ CCI_REG8(0x4509), 0x05 },
	{ CCI_REG8(0x450c), 0x00 },
	{ CCI_REG8(0x450d), 0x30 },
	{ CCI_REG8(0x450e), 0x00 },
	{ CCI_REG8(0x4510), 0x03 },
	{ CCI_REG8(0x4511), 0x08 },
	{ CCI_REG8(0x4512), 0x05 },
	{ CCI_REG8(0x4513), 0x05 },
	{ CCI_REG8(0x4514), 0xaf },
	{ CCI_REG8(0x4515), 0xaf },
	{ CCI_REG8(0x4516), 0x05 },
	{ CCI_REG8(0x4517), 0x05 },
	{ CCI_REG8(0x4518), 0xaf },
	{ CCI_REG8(0x4519), 0xaf },
	{ CCI_REG8(0x451a), 0x11 },
	{ CCI_REG8(0x451b), 0xbb },
	{ CCI_REG8(0x451c), 0x11 },
	{ CCI_REG8(0x451d), 0xbb },
	{ CCI_REG8(0x451e), 0x11 },
	{ CCI_REG8(0x451f), 0xbb },
	{ CCI_REG8(0x4520), 0x11 },
	{ CCI_REG8(0x4521), 0xbb },
	{ CCI_REG8(0x4542), 0x00 },
	{ CCI_REG8(0x4549), 0x00 },
	{ CCI_REG8(0x45c0), 0x80 },
	{ CCI_REG8(0x45c1), 0x00 },
	{ CCI_REG8(0x45c2), 0x02 },
	{ CCI_REG8(0x45c3), 0x00 },
	{ CCI_REG8(0x45c4), 0x13 },
	{ CCI_REG8(0x45c5), 0x00 },
	{ CCI_REG8(0x45c6), 0x00 },
	{ CCI_REG8(0x45c7), 0x00 },
	{ CCI_REG8(0x45c8), 0x00 },
	{ CCI_REG8(0x45c9), 0x00 },
	{ CCI_REG8(0x45ca), 0x00 },
	{ CCI_REG8(0x45cb), 0x00 },
	{ CCI_REG8(0x45cc), 0x00 },
	{ CCI_REG8(0x45cd), 0x07 },
	{ CCI_REG8(0x45ce), 0x13 },
	{ CCI_REG8(0x45cf), 0x13 },
	{ CCI_REG8(0x45d0), 0x13 },
	{ CCI_REG8(0x45d3), 0x00 },
	{ CCI_REG8(0x45d4), 0x00 },
	{ CCI_REG8(0x45d5), 0x00 },
	{ CCI_REG8(0x45d6), 0x00 },
	{ CCI_REG8(0x45d7), 0x00 },
	{ CCI_REG8(0x45d8), 0x00 },
	{ CCI_REG8(0x45d9), 0x00 },
	{ CCI_REG8(0x45da), 0x00 },
	{ CCI_REG8(0x45dd), 0x00 },
	{ CCI_REG8(0x45d2), 0x00 },
	{ CCI_REG8(0x45de), 0x00 },
	{ CCI_REG8(0x45df), 0x00 },
	{ CCI_REG8(0x45e0), 0x00 },
	{ CCI_REG8(0x45e1), 0x00 },
	{ CCI_REG8(0x45e2), 0x00 },
	{ CCI_REG8(0x45e3), 0x00 },
	{ CCI_REG8(0x45e4), 0x00 },
	{ CCI_REG8(0x45e5), 0x00 },
	{ CCI_REG8(0x4602), 0x00 },
	{ CCI_REG8(0x4603), 0x11 },
	{ CCI_REG8(0x460b), 0x07 },
	{ CCI_REG8(0x4640), 0x02 },
	{ CCI_REG8(0x4641), 0x00 },
	{ CCI_REG8(0x4643), 0x08 },
	{ CCI_REG8(0x4644), 0xe0 },
	{ CCI_REG8(0x4647), 0x02 },
	{ CCI_REG8(0x464c), 0x01 },
	{ CCI_REG8(0x4680), 0x11 },
	{ CCI_REG8(0x4684), 0x2b },
	{ CCI_REG8(0x4685), 0x17 },
	{ CCI_REG8(0x468e), 0x30 },
	{ CCI_REG8(0x4690), 0x00 },
	{ CCI_REG8(0x4802), 0x02 },
	{ CCI_REG8(0x4813), 0x10 },
	{ CCI_REG8(0x481b), 0x14 },
	{ CCI_REG8(0x481f), 0x26 },
	{ CCI_REG8(0x4825), 0x40 },
	{ CCI_REG8(0x4836), 0x32 },
	{ CCI_REG8(0x4837), 0x04 },
	{ CCI_REG8(0x4850), 0x42 },
	{ CCI_REG8(0x4851), 0xaa },
	{ CCI_REG8(0x4853), 0x10 },
	{ CCI_REG8(0x4854), 0x05 },
	{ CCI_REG8(0x4855), 0x1c },
	{ CCI_REG8(0x4860), 0x01 },
	{ CCI_REG8(0x4862), 0x30 },
	{ CCI_REG8(0x4883), 0x24 },
	{ CCI_REG8(0x4888), 0x80 },
	{ CCI_REG8(0x4908), 0x00 },
	{ CCI_REG8(0x4909), 0x02 },
	{ CCI_REG8(0x4910), 0x28 },
	{ CCI_REG8(0x4915), 0x00 },
	{ CCI_REG8(0x4916), 0x1f },
	{ CCI_REG8(0x4917), 0x00 },
	{ CCI_REG8(0x4918), 0x0f },
	{ CCI_REG8(0x491b), 0x04 },
	{ CCI_REG8(0x491e), 0x01 },
	{ CCI_REG8(0x491f), 0x30 },
	{ CCI_REG8(0x4930), 0x00 },
	{ CCI_REG8(0x4931), 0x00 },
	{ CCI_REG8(0x4932), 0x00 },
	{ CCI_REG8(0x4933), 0x00 },
	{ CCI_REG8(0x4934), 0x00 },
	{ CCI_REG8(0x4935), 0x00 },
	{ CCI_REG8(0x4936), 0x00 },
	{ CCI_REG8(0x4937), 0x00 },
	{ CCI_REG8(0x4938), 0x00 },
	{ CCI_REG8(0x4939), 0x00 },
	{ CCI_REG8(0x493a), 0x00 },
	{ CCI_REG8(0x493b), 0x00 },
	{ CCI_REG8(0x493c), 0x00 },
	{ CCI_REG8(0x493d), 0x00 },
	{ CCI_REG8(0x493e), 0x00 },
	{ CCI_REG8(0x493f), 0x00 },
	{ CCI_REG8(0x4940), 0x00 },
	{ CCI_REG8(0x4941), 0x80 },
	{ CCI_REG8(0x4942), 0x0f },
	{ CCI_REG8(0x4943), 0x80 },
	{ CCI_REG8(0x4944), 0x00 },
	{ CCI_REG8(0x4945), 0x90 },
	{ CCI_REG8(0x4946), 0x0f },
	{ CCI_REG8(0x4947), 0x70 },
	{ CCI_REG8(0x4a08), 0x00 },
	{ CCI_REG8(0x4a09), 0x02 },
	{ CCI_REG8(0x4a15), 0x00 },
	{ CCI_REG8(0x4a16), 0x1f },
	{ CCI_REG8(0x4a17), 0x00 },
	{ CCI_REG8(0x4a18), 0x0f },
	{ CCI_REG8(0x4a1b), 0x04 },
	{ CCI_REG8(0x4a1e), 0x01 },
	{ CCI_REG8(0x4a1f), 0x30 },
	{ CCI_REG8(0x4d00), 0x04 },
	{ CCI_REG8(0x4d01), 0x9a },
	{ CCI_REG8(0x4d02), 0xbb },
	{ CCI_REG8(0x4d03), 0x33 },
	{ CCI_REG8(0x4d04), 0x61 },
	{ CCI_REG8(0x4d05), 0x9b },
	{ CCI_REG8(0x5000), 0x5b },
	{ CCI_REG8(0x5001), 0x28 },
	{ CCI_REG8(0x5002), 0x00 },
	{ CCI_REG8(0x5004), 0x02 },
	{ CCI_REG8(0x5060), 0x10 },
	{ CCI_REG8(0x50c1), 0x00 },
	{ CCI_REG8(0x5110), 0x90 },
	{ CCI_REG8(0x5111), 0x10 },
	{ CCI_REG8(0x5112), 0x9b },
	{ CCI_REG8(0x5113), 0x27 },
	{ CCI_REG8(0x5155), 0x08 },
	{ CCI_REG8(0x5156), 0x0c },
	{ CCI_REG8(0x5157), 0x0c },
	{ CCI_REG8(0x5159), 0x08 },
	{ CCI_REG8(0x515a), 0x0c },
	{ CCI_REG8(0x515b), 0x0c },
	{ CCI_REG8(0x5180), 0xc1 },
	{ CCI_REG8(0x518a), 0x00 },
	{ CCI_REG8(0x53c1), 0x00 },
	{ CCI_REG8(0x5455), 0x08 },
	{ CCI_REG8(0x5456), 0x0c },
	{ CCI_REG8(0x5457), 0x0c },
	{ CCI_REG8(0x5459), 0x08 },
	{ CCI_REG8(0x545a), 0x0c },
	{ CCI_REG8(0x545b), 0x0c },
	{ CCI_REG8(0x5480), 0xc1 },
	{ CCI_REG8(0x548a), 0x00 },
	{ CCI_REG8(0x56c1), 0x00 },
	{ CCI_REG8(0x5755), 0x08 },
	{ CCI_REG8(0x5756), 0x0c },
	{ CCI_REG8(0x5757), 0x0c },
	{ CCI_REG8(0x5759), 0x08 },
	{ CCI_REG8(0x575a), 0x0c },
	{ CCI_REG8(0x575b), 0x0c },
	{ CCI_REG8(0x5780), 0xc1 },
	{ CCI_REG8(0x578a), 0x00 },
	{ CCI_REG8(0x5853), 0xff },
	{ CCI_REG8(0x5854), 0xff },
	{ CCI_REG8(0x5855), 0xff },
	{ CCI_REG8(0x5856), 0xff },
	{ CCI_REG8(0x5857), 0xff },
	{ CCI_REG8(0x5858), 0xff },
	{ CCI_REG8(0x587b), 0x40 },
	{ CCI_REG8(0x58a7), 0x11 },
	{ CCI_REG8(0x58c0), 0x3f },
	{ CCI_REG8(0x58fd), 0x0a },
	{ CCI_REG8(0x5925), 0x00 },
	{ CCI_REG8(0x5926), 0x00 },
	{ CCI_REG8(0x5927), 0x00 },
	{ CCI_REG8(0x5928), 0x00 },
	{ CCI_REG8(0x5929), 0x00 },
	{ CCI_REG8(0x592c), 0x06 },
	{ CCI_REG8(0x592d), 0x00 },
	{ CCI_REG8(0x592e), 0x03 },
	{ CCI_REG8(0x59c2), 0x00 },
	{ CCI_REG8(0x59c3), 0xce },
	{ CCI_REG8(0x59c4), 0x01 },
	{ CCI_REG8(0x59c5), 0x20 },
	{ CCI_REG8(0x59c6), 0x01 },
	{ CCI_REG8(0x59c7), 0x91 },
	{ CCI_REG8(0x59c8), 0x02 },
	{ CCI_REG8(0x59c9), 0x2f },
	{ CCI_REG8(0x59ca), 0x03 },
	{ CCI_REG8(0x59cb), 0x0a },
	{ CCI_REG8(0x59cc), 0x04 },
	{ CCI_REG8(0x59cd), 0x3d },
	{ CCI_REG8(0x59ce), 0x05 },
	{ CCI_REG8(0x59cf), 0xe8 },
	{ CCI_REG8(0x59d0), 0x08 },
	{ CCI_REG8(0x59d1), 0x3c },
	{ CCI_REG8(0x59d2), 0x0b },
	{ CCI_REG8(0x59d3), 0x7a },
	{ CCI_REG8(0x59d4), 0x0f },
	{ CCI_REG8(0x59d5), 0xff },
	{ CCI_REG8(0x59d6), 0x0f },
	{ CCI_REG8(0x59d7), 0xff },
	{ CCI_REG8(0x59d8), 0x0f },
	{ CCI_REG8(0x59d9), 0xff },
	{ CCI_REG8(0x59da), 0x0f },
	{ CCI_REG8(0x59db), 0xff },
	{ CCI_REG8(0x59ef), 0x5f },
	{ CCI_REG8(0x6924), 0x00 },
	{ CCI_REG8(0x6925), 0x00 },
	{ CCI_REG8(0x6926), 0x00 },
	{ CCI_REG8(0x6942), 0x00 },
	{ CCI_REG8(0x6943), 0x00 },
	{ CCI_REG8(0x6944), 0x00 },
	{ CCI_REG8(0x694b), 0x00 },
	{ CCI_REG8(0x6a20), 0x03 },
	{ CCI_REG8(0x6a21), 0x02 },
	{ CCI_REG8(0x6a22), 0x00 },
	{ CCI_REG8(0x6a53), 0xff },
	{ CCI_REG8(0x6a54), 0xff },
	{ CCI_REG8(0x6a55), 0xff },
	{ CCI_REG8(0x6a56), 0xff },
	{ CCI_REG8(0x6a57), 0xff },
	{ CCI_REG8(0x6a58), 0xff },
	{ CCI_REG8(0x6a7b), 0x40 },
	{ CCI_REG8(0x6aa7), 0x11 },
	{ CCI_REG8(0x6ac0), 0x3f },
	{ CCI_REG8(0x6afd), 0x0a },
	{ CCI_REG8(0x6b25), 0x00 },
	{ CCI_REG8(0x6b26), 0x00 },
	{ CCI_REG8(0x6b27), 0x00 },
	{ CCI_REG8(0x6b28), 0x00 },
	{ CCI_REG8(0x6b29), 0x00 },
	{ CCI_REG8(0x6b2c), 0x06 },
	{ CCI_REG8(0x6b2d), 0x00 },
	{ CCI_REG8(0x6b2e), 0x03 },
	{ CCI_REG8(0x6bc2), 0x00 },
	{ CCI_REG8(0x6bc3), 0xce },
	{ CCI_REG8(0x6bc4), 0x01 },
	{ CCI_REG8(0x6bc5), 0x20 },
	{ CCI_REG8(0x6bc6), 0x01 },
	{ CCI_REG8(0x6bc7), 0x91 },
	{ CCI_REG8(0x6bc8), 0x02 },
	{ CCI_REG8(0x6bc9), 0x2f },
	{ CCI_REG8(0x6bca), 0x03 },
	{ CCI_REG8(0x6bcb), 0x0a },
	{ CCI_REG8(0x6bcc), 0x04 },
	{ CCI_REG8(0x6bcd), 0x3d },
	{ CCI_REG8(0x6bce), 0x05 },
	{ CCI_REG8(0x6bcf), 0xe8 },
	{ CCI_REG8(0x6bd0), 0x08 },
	{ CCI_REG8(0x6bd1), 0x3c },
	{ CCI_REG8(0x6bd2), 0x0b },
	{ CCI_REG8(0x6bd3), 0x7a },
	{ CCI_REG8(0x6bd4), 0x0f },
	{ CCI_REG8(0x6bd5), 0xff },
	{ CCI_REG8(0x6bd6), 0x0f },
	{ CCI_REG8(0x6bd7), 0xff },
	{ CCI_REG8(0x6bd8), 0x0f },
	{ CCI_REG8(0x6bd9), 0xff },
	{ CCI_REG8(0x6bda), 0x0f },
	{ CCI_REG8(0x6bdb), 0xff },
	{ CCI_REG8(0x6bef), 0x5f },
	{ CCI_REG8(0xc200), 0x00 },
	{ CCI_REG8(0xc210), 0x00 },
	{ CCI_REG8(0xc211), 0x00 },
	{ CCI_REG8(0xc212), 0x00 },
	{ CCI_REG8(0xc213), 0x00 },
	{ CCI_REG8(0xc231), 0x00 },
	{ CCI_REG8(0xc232), 0x01 },
	{ CCI_REG8(0xc233), 0x8f },
	{ CCI_REG8(0xc240), 0x00 },
	{ CCI_REG8(0xc241), 0x00 },
	{ CCI_REG8(0xc242), 0x00 },
	{ CCI_REG8(0xc243), 0x00 },
	{ CCI_REG8(0xc245), 0x00 },
	{ CCI_REG8(0xc246), 0x01 },
	{ CCI_REG8(0xc247), 0x8f },
	{ CCI_REG8(0xc250), 0x00 },
	{ CCI_REG8(0xc251), 0x00 },
	{ CCI_REG8(0xc252), 0x00 },
	{ CCI_REG8(0xc253), 0x00 },
	{ CCI_REG8(0xc258), 0x00 },
	{ CCI_REG8(0xc259), 0x00 },
	{ CCI_REG8(0xc25a), 0x00 },
	{ CCI_REG8(0xc25b), 0x00 },
	{ CCI_REG8(0xc264), 0x00 },
	{ CCI_REG8(0xc265), 0x00 },
	{ CCI_REG8(0xc266), 0x00 },
	{ CCI_REG8(0xc267), 0x00 },
	{ CCI_REG8(0xc26c), 0x00 },
	{ CCI_REG8(0xc26d), 0x00 },
	{ CCI_REG8(0xc26e), 0x00 },
	{ CCI_REG8(0xc26f), 0x00 },
	{ CCI_REG8(0xc40d), 0x04 },
	{ CCI_REG8(0xc40f), 0x07 },
	{ CCI_REG8(0xc415), 0x01 },
	{ CCI_REG8(0xc44a), 0x01 },
	{ CCI_REG8(0xc44c), 0x01 },
	{ CCI_REG8(0xc44e), 0x03 },
	{ CCI_REG8(0xc44f), 0x03 },
	{ CCI_REG8(0xc450), 0x04 },
	{ CCI_REG8(0xc451), 0x04 },
	{ CCI_REG8(0xc45e), 0x13 },
	{ CCI_REG8(0xc46d), 0x55 },
	{ CCI_REG8(0xc47f), 0x06 },
	{ CCI_REG8(0xc480), 0x03 },
	{ CCI_REG8(0xc48d), 0x80 },
	{ CCI_REG8(0xc48c), 0xc0 },
	{ CCI_REG8(0xc48f), 0x3f },
	{ CCI_REG8(0xc491), 0x00 },
	{ CCI_REG8(0xc49c), 0x08 },
	{ CCI_REG8(0xc49e), 0x20 },
	{ CCI_REG8(0xc49f), 0x20 },
	{ CCI_REG8(0xc4a2), 0x3e },
	{ CCI_REG8(0xc4a3), 0x3e },
	{ CCI_REG8(0xc4a6), 0x1a },
	{ CCI_REG8(0xc4a7), 0x1a },
	{ CCI_REG8(0xc4ae), 0x00 },
	{ CCI_REG8(0xc4af), 0x00 },
	{ CCI_REG8(0xc4b0), 0x00 },
	{ CCI_REG8(0xc4b1), 0x00 },
	{ CCI_REG8(0xc4b2), 0x00 },
	{ CCI_REG8(0xc4c1), 0x00 },
	{ CCI_REG8(0xc4c2), 0x07 },
	{ CCI_REG8(0xc4c3), 0x07 },
	{ CCI_REG8(0xc4c4), 0x07 },
	{ CCI_REG8(0xc4c6), 0x2a },
	{ CCI_REG8(0xc4c7), 0x2a },
	{ CCI_REG8(0xc4c8), 0x35 },
	{ CCI_REG8(0xc4c9), 0x35 },
	{ CCI_REG8(0xc4ca), 0x35 },
	{ CCI_REG8(0xc4cb), 0x35 },
	{ CCI_REG8(0xc4cc), 0xff },
	{ CCI_REG8(0xc4cd), 0x0f },
	{ CCI_REG8(0xc4ce), 0x03 },
	{ CCI_REG8(0xc4cf), 0x01 },
	{ CCI_REG8(0xc4d0), 0x00 },
	{ CCI_REG8(0xc4d1), 0x00 },
	{ CCI_REG8(0xc4d2), 0x0c },
	{ CCI_REG8(0xc4d3), 0x0c },
	{ CCI_REG8(0xc4d4), 0x0c },
	{ CCI_REG8(0xc4d5), 0x0c },
	{ CCI_REG8(0xc4e0), 0x07 },
	{ CCI_REG8(0xc4e1), 0x07 },
	{ CCI_REG8(0xc4e2), 0x07 },
	{ CCI_REG8(0xc4e3), 0x11 },
	{ CCI_REG8(0xc4e9), 0x11 },
	{ CCI_REG8(0xc4ed), 0x07 },
	{ CCI_REG8(0xc4ee), 0x07 },
	{ CCI_REG8(0xc4ef), 0xee },
	{ CCI_REG8(0xc4f0), 0xee },
	{ CCI_REG8(0xc4f1), 0x0e },
	{ CCI_REG8(0xc4fa), 0x07 },
	{ CCI_REG8(0xc4fb), 0x07 },
	{ CCI_REG8(0xc4fd), 0x80 },
	{ CCI_REG8(0xc4fc), 0x80 },
	{ CCI_REG8(0xc4fe), 0x07 },
	{ CCI_REG8(0xc4ff), 0x07 },
	{ CCI_REG8(0xc50b), 0x77 },
	{ CCI_REG8(0xc50e), 0x58 },
	{ CCI_REG8(0xc50f), 0x58 },
	{ CCI_REG8(0xc510), 0x30 },
	{ CCI_REG8(0xc511), 0x1a },
	{ CCI_REG8(0xc512), 0x09 },
	{ CCI_REG8(0xc513), 0x58 },
	{ CCI_REG8(0xc514), 0x58 },
	{ CCI_REG8(0xc515), 0x30 },
	{ CCI_REG8(0xc516), 0x1c },
	{ CCI_REG8(0xc517), 0x0c },
	{ CCI_REG8(0xc518), 0x38 },
	{ CCI_REG8(0xc519), 0x38 },
	{ CCI_REG8(0xc51a), 0x38 },
	{ CCI_REG8(0xc51b), 0x38 },
	{ CCI_REG8(0xc51c), 0x38 },
	{ CCI_REG8(0xc51d), 0x2c },
	{ CCI_REG8(0xc51e), 0x2c },
	{ CCI_REG8(0xc51f), 0x2c },
	{ CCI_REG8(0xc520), 0x2c },
	{ CCI_REG8(0xc521), 0x2c },
	{ CCI_REG8(0xc526), 0x0e },
	{ CCI_REG8(0xc527), 0x0e },
	{ CCI_REG8(0xc528), 0x0e },
	{ CCI_REG8(0xc529), 0x0e },
	{ CCI_REG8(0xc52a), 0x0e },
	{ CCI_REG8(0xc52b), 0x0e },
	{ CCI_REG8(0xc52c), 0x0e },
	{ CCI_REG8(0xc52d), 0x0e },
	{ CCI_REG8(0xc52e), 0x0e },
	{ CCI_REG8(0xc52f), 0x0e },
	{ CCI_REG8(0xc530), 0x0e },
	{ CCI_REG8(0xc531), 0x0e },
	{ CCI_REG8(0xc532), 0x0e },
	{ CCI_REG8(0xc533), 0x0e },
	{ CCI_REG8(0xc534), 0x0e },
	{ CCI_REG8(0xc535), 0x0e },
	{ CCI_REG8(0xc57b), 0xff },
	{ CCI_REG8(0xc583), 0x18 },
	{ CCI_REG8(0xc584), 0x08 },
	{ CCI_REG8(0xc58b), 0x18 },
	{ CCI_REG8(0xc58c), 0x08 },
	{ CCI_REG8(0xc58f), 0x2c },
	{ CCI_REG8(0xc590), 0x2c },
	{ CCI_REG8(0xc591), 0x2c },
	{ CCI_REG8(0xc592), 0x2c },
	{ CCI_REG8(0xc593), 0x2c },
	{ CCI_REG8(0xc594), 0x10 },
	{ CCI_REG8(0xc597), 0x2c },
	{ CCI_REG8(0xc598), 0x2c },
	{ CCI_REG8(0xc599), 0x2c },
	{ CCI_REG8(0xc59a), 0x2c },
	{ CCI_REG8(0xc59b), 0x2c },
	{ CCI_REG8(0xc59c), 0x10 },
	{ CCI_REG8(0xc5c3), 0x00 },
	{ CCI_REG8(0xc5c4), 0x00 },
	{ CCI_REG8(0xc5c5), 0x00 },
	{ CCI_REG8(0xc5c6), 0x00 },
	{ CCI_REG8(0xc5c7), 0x00 },
	{ CCI_REG8(0xc5c8), 0x00 },
	{ CCI_REG8(0xc5c9), 0x48 },
	{ CCI_REG8(0xc5ca), 0x48 },
	{ CCI_REG8(0xc5cb), 0x40 },
	{ CCI_REG8(0xc5cc), 0x40 },
	{ CCI_REG8(0xc5cd), 0x40 },
	{ CCI_REG8(0xc5ce), 0x40 },
	{ CCI_REG8(0xc5e8), 0x00 },
	{ CCI_REG8(0xc5fe), 0x04 },
	{ CCI_REG8(0xc5ff), 0x06 },
	{ CCI_REG8(0xc624), 0x00 },
	{ CCI_REG8(0x3519), 0x00 },
	{ CCI_REG8(0x351a), 0xd9 },
	{ CCI_REG8(0x351b), 0xe0 },
	{ CCI_REG8(0x3559), 0x00 },
	{ CCI_REG8(0x355a), 0xd9 },
	{ CCI_REG8(0x355b), 0xe0 },
	{ CCI_REG8(0x3599), 0x00 },
	{ CCI_REG8(0x359a), 0xd9 },
	{ CCI_REG8(0x359b), 0xe0 },
	{ CCI_REG8(0x5060), 0x0d },
	{ CCI_REG8(0x5061), 0x9e },
	{ CCI_REG8(0x0304), 0x02 },
	{ CCI_REG8(0x0305), 0xc0 },
	{ CCI_REG8(0x4837), 0x04 },
	{ CCI_REG8(0x4800), 0x64 },
};

/* 4096x3072, 30 fps */
static const struct cci_reg_sequence ov50e40_4096x3072_mode[] = {
	{ CCI_REG8(0x0324), 0x01 },
	{ CCI_REG8(0x0325), 0x90 },
	{ CCI_REG8(0x0326), 0xd3 },
	{ CCI_REG8(0x0327), 0x05 },
	{ CCI_REG8(0x3506), 0xf9 },
	{ CCI_REG8(0x350e), 0x21 },
	{ CCI_REG8(0x350f), 0x80 },
	{ CCI_REG8(0x3546), 0xf9 },
	{ CCI_REG8(0x354e), 0x21 },
	{ CCI_REG8(0x354f), 0x80 },
	{ CCI_REG8(0x3586), 0xf9 },
	{ CCI_REG8(0x358e), 0x21 },
	{ CCI_REG8(0x358f), 0x80 },
	{ CCI_REG8(0x3600), 0x07 },
	{ CCI_REG8(0x3608), 0xf0 },
	{ CCI_REG8(0x360a), 0xff },
	{ CCI_REG8(0x3616), 0x18 },
	{ CCI_REG8(0x361a), 0x56 },
	{ CCI_REG8(0x361c), 0x00 },
	{ CCI_REG8(0x361e), 0x14 },
	{ CCI_REG8(0x363b), 0xc2 },
	{ CCI_REG8(0x363c), 0xcb },
	{ CCI_REG8(0x3645), 0x10 },
	{ CCI_REG8(0x3680), 0x00 },
	{ CCI_REG8(0x3682), 0x80 },
	{ CCI_REG8(0x3684), 0x01 },
	{ CCI_REG8(0x3688), 0x01 },
	{ CCI_REG8(0x369a), 0x00 },
	{ CCI_REG8(0x3706), 0x25 },
	{ CCI_REG8(0x3707), 0x04 },
	{ CCI_REG8(0x3709), 0x80 },
	{ CCI_REG8(0x370b), 0x55 },
	{ CCI_REG8(0x370c), 0x04 },
	{ CCI_REG8(0x3712), 0x50 },
	{ CCI_REG8(0x3714), 0x65 },
	{ CCI_REG8(0x3715), 0x00 },
	{ CCI_REG8(0x372a), 0x00 },
	{ CCI_REG8(0x372b), 0x00 },
	{ CCI_REG8(0x374f), 0x50 },
	{ CCI_REG8(0x3755), 0x80 },
	{ CCI_REG8(0x3756), 0x00 },
	{ CCI_REG8(0x3757), 0xa0 },
	{ CCI_REG8(0x3758), 0x00 },
	{ CCI_REG8(0x3759), 0x50 },
	{ CCI_REG8(0x375e), 0x3b },
	{ CCI_REG8(0x375f), 0x02 },
	{ CCI_REG8(0x3763), 0x08 },
	{ CCI_REG8(0x376c), 0x00 },
	{ CCI_REG8(0x3780), 0x4f },
	{ CCI_REG8(0x3781), 0x00 },
	{ CCI_REG8(0x3782), 0x01 },
	{ CCI_REG8(0x3785), 0x01 },
	{ CCI_REG8(0x3786), 0x01 },
	{ CCI_REG8(0x3789), 0x01 },
	{ CCI_REG8(0x378a), 0x01 },
	{ CCI_REG8(0x378d), 0x01 },
	{ CCI_REG8(0x378e), 0x01 },
	{ CCI_REG8(0x3791), 0x25 },
	{ CCI_REG8(0x3793), 0x1c },
	{ CCI_REG8(0x3795), 0x1c },
	{ CCI_REG8(0x3797), 0x55 },
	{ CCI_REG8(0x3799), 0x3c },
	{ CCI_REG8(0x379b), 0x3c },
	{ CCI_REG8(0x37a0), 0xa8 },
	{ CCI_REG8(0x37d0), 0x66 },
	{ CCI_REG8(0x37e3), 0x30 },
	{ CCI_REG8(0x3800), 0x00 },
	{ CCI_REG8(0x3801), 0x00 },
	{ CCI_REG8(0x3802), 0x00 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x20 },
	{ CCI_REG8(0x3805), 0x1f },
	{ CCI_REG8(0x3806), 0x18 },
	{ CCI_REG8(0x3807), 0x1f },
	{ CCI_REG8(0x3808), 0x10 },
	{ CCI_REG8(0x3809), 0x00 },
	{ CCI_REG8(0x380a), 0x0c },
	{ CCI_REG8(0x380b), 0x00 },
	{ CCI_REG8(0x380c), 0x01 },
	{ CCI_REG8(0x380d), 0x90 },
	{ CCI_REG8(0x380e), 0x0c },
	{ CCI_REG8(0x380f), 0x38 },
	{ CCI_REG8(0x3811), 0x07 },
	{ CCI_REG8(0x3813), 0x08 },
	{ CCI_REG8(0x3815), 0x11 },
	{ CCI_REG8(0x3820), 0x46 },
	{ CCI_REG8(0x3821), 0x11 },
	{ CCI_REG8(0x3822), 0x10 },
	{ CCI_REG8(0x3823), 0x04 },
	{ CCI_REG8(0x3828), 0x05 },
	{ CCI_REG8(0x3830), 0x07 },
	{ CCI_REG8(0x3831), 0x02 },
	{ CCI_REG8(0x3837), 0x07 },
	{ CCI_REG8(0x383f), 0x08 },
	{ CCI_REG8(0x384c), 0x01 },
	{ CCI_REG8(0x384d), 0x90 },
	{ CCI_REG8(0x3889), 0x08 },
	{ CCI_REG8(0x388b), 0x08 },
	{ CCI_REG8(0x388c), 0x10 },
	{ CCI_REG8(0x388d), 0x00 },
	{ CCI_REG8(0x388e), 0x0c },
	{ CCI_REG8(0x388f), 0x00 },
	{ CCI_REG8(0x3896), 0x00 },
	{ CCI_REG8(0x3917), 0x00 },
	{ CCI_REG8(0x3981), 0x01 },
	{ CCI_REG8(0x3982), 0x20 },
	{ CCI_REG8(0x398b), 0x80 },
	{ CCI_REG8(0x39ec), 0x04 },
	{ CCI_REG8(0x39ed), 0x04 },
	{ CCI_REG8(0x3a0d), 0x05 },
	{ CCI_REG8(0x3a13), 0x05 },
	{ CCI_REG8(0x3a14), 0x05 },
	{ CCI_REG8(0x3a15), 0x01 },
	{ CCI_REG8(0x3a16), 0x00 },
	{ CCI_REG8(0x3a17), 0x00 },
	{ CCI_REG8(0x3a2a), 0x01 },
	{ CCI_REG8(0x3a2e), 0x01 },
	{ CCI_REG8(0x3a2f), 0x01 },
	{ CCI_REG8(0x3a30), 0x01 },
	{ CCI_REG8(0x3a31), 0x01 },
	{ CCI_REG8(0x3a32), 0x01 },
	{ CCI_REG8(0x3a34), 0x01 },
	{ CCI_REG8(0x3a35), 0x01 },
	{ CCI_REG8(0x3a36), 0x01 },
	{ CCI_REG8(0x3a37), 0x01 },
	{ CCI_REG8(0x3a40), 0x01 },
	{ CCI_REG8(0x3a42), 0x01 },
	{ CCI_REG8(0x3a43), 0x01 },
	{ CCI_REG8(0x3a44), 0x01 },
	{ CCI_REG8(0x3a45), 0x01 },
	{ CCI_REG8(0x3a46), 0x01 },
	{ CCI_REG8(0x3a48), 0x01 },
	{ CCI_REG8(0x3a49), 0x01 },
	{ CCI_REG8(0x3a4a), 0x01 },
	{ CCI_REG8(0x3a4b), 0x01 },
	{ CCI_REG8(0x3a5e), 0x50 },
	{ CCI_REG8(0x3aed), 0x3a },
	{ CCI_REG8(0x3af1), 0x3f },
	{ CCI_REG8(0x3afd), 0x04 },
	{ CCI_REG8(0x3afe), 0x09 },
	{ CCI_REG8(0x3aff), 0x04 },
	{ CCI_REG8(0x3b00), 0x09 },
	{ CCI_REG8(0x3b01), 0xfb },
	{ CCI_REG8(0x3b02), 0x02 },
	{ CCI_REG8(0x3b03), 0x14 },
	{ CCI_REG8(0x3b04), 0x19 },
	{ CCI_REG8(0x3b05), 0x1d },
	{ CCI_REG8(0x3b08), 0x1f },
	{ CCI_REG8(0x3b14), 0x02 },
	{ CCI_REG8(0x3b15), 0x2b },
	{ CCI_REG8(0x3b19), 0x2b },
	{ CCI_REG8(0x3b1a), 0x2c },
	{ CCI_REG8(0x3b1d), 0x00 },
	{ CCI_REG8(0x3b22), 0x2c },
	{ CCI_REG8(0x3b25), 0x28 },
	{ CCI_REG8(0x3b2b), 0x00 },
	{ CCI_REG8(0x3b2c), 0x00 },
	{ CCI_REG8(0x3b2d), 0x04 },
	{ CCI_REG8(0x3b32), 0x29 },
	{ CCI_REG8(0x3b33), 0x2a },
	{ CCI_REG8(0x3b34), 0x2a },
	{ CCI_REG8(0x3b38), 0x22 },
	{ CCI_REG8(0x3b39), 0x25 },
	{ CCI_REG8(0x3b3a), 0x26 },
	{ CCI_REG8(0x3b3b), 0x27 },
	{ CCI_REG8(0x3b41), 0x07 },
	{ CCI_REG8(0x3b42), 0x07 },
	{ CCI_REG8(0x3b43), 0x07 },
	{ CCI_REG8(0x3b44), 0x07 },
	{ CCI_REG8(0x3b45), 0x00 },
	{ CCI_REG8(0x3b49), 0x00 },
	{ CCI_REG8(0x3b56), 0x12 },
	{ CCI_REG8(0x3b57), 0x12 },
	{ CCI_REG8(0x3b58), 0x12 },
	{ CCI_REG8(0x3b59), 0x12 },
	{ CCI_REG8(0x3b5a), 0x1d },
	{ CCI_REG8(0x3b5b), 0x1d },
	{ CCI_REG8(0x3b5c), 0x1d },
	{ CCI_REG8(0x3b5d), 0x1d },
	{ CCI_REG8(0x3b82), 0x1d },
	{ CCI_REG8(0x3ba0), 0x25 },
	{ CCI_REG8(0x3ba1), 0x12 },
	{ CCI_REG8(0x3ba2), 0x26 },
	{ CCI_REG8(0x3bab), 0x2c },
	{ CCI_REG8(0x3baf), 0x58 },
	{ CCI_REG8(0x3bf2), 0x33 },
	{ CCI_REG8(0x3bfa), 0xa0 },
	{ CCI_REG8(0x3bff), 0x08 },
	{ CCI_REG8(0x4010), 0x28 },
	{ CCI_REG8(0x4012), 0x8c },
	{ CCI_REG8(0x4016), 0x0f },
	{ CCI_REG8(0x4018), 0x07 },
	{ CCI_REG8(0x401e), 0x02 },
	{ CCI_REG8(0x401f), 0x80 },
	{ CCI_REG8(0x40c4), 0x03 },
	{ CCI_REG8(0x40c5), 0x00 },
	{ CCI_REG8(0x40c6), 0x06 },
	{ CCI_REG8(0x40c7), 0x00 },
	{ CCI_REG8(0x4288), 0x07 },
	{ CCI_REG8(0x4505), 0x0c },
	{ CCI_REG8(0x4506), 0x01 },
	{ CCI_REG8(0x4509), 0x15 },
	{ CCI_REG8(0x450c), 0x00 },
	{ CCI_REG8(0x450d), 0x60 },
	{ CCI_REG8(0x4510), 0x07 },
	{ CCI_REG8(0x4511), 0x00 },
	{ CCI_REG8(0x4512), 0x00 },
	{ CCI_REG8(0x4513), 0x00 },
	{ CCI_REG8(0x4514), 0x00 },
	{ CCI_REG8(0x4515), 0x00 },
	{ CCI_REG8(0x4516), 0x00 },
	{ CCI_REG8(0x4517), 0x00 },
	{ CCI_REG8(0x4518), 0x00 },
	{ CCI_REG8(0x4519), 0x00 },
	{ CCI_REG8(0x451a), 0x11 },
	{ CCI_REG8(0x451b), 0xbb },
	{ CCI_REG8(0x451c), 0x11 },
	{ CCI_REG8(0x451d), 0xbb },
	{ CCI_REG8(0x451e), 0x11 },
	{ CCI_REG8(0x451f), 0xbb },
	{ CCI_REG8(0x4520), 0x11 },
	{ CCI_REG8(0x4521), 0xbb },
	{ CCI_REG8(0x4542), 0x00 },
	{ CCI_REG8(0x45c0), 0x8e },
	{ CCI_REG8(0x45c1), 0x80 },
	{ CCI_REG8(0x45c2), 0x0a },
	{ CCI_REG8(0x45c3), 0x14 },
	{ CCI_REG8(0x45c4), 0x13 },
	{ CCI_REG8(0x45c5), 0x40 },
	{ CCI_REG8(0x45c6), 0x04 },
	{ CCI_REG8(0x45dd), 0x60 },
	{ CCI_REG8(0x4602), 0x00 },
	{ CCI_REG8(0x4603), 0x11 },
	{ CCI_REG8(0x460b), 0x07 },
	{ CCI_REG8(0x4640), 0x00 },
	{ CCI_REG8(0x4641), 0x80 },
	{ CCI_REG8(0x4643), 0x0c },
	{ CCI_REG8(0x4680), 0x11 },
	{ CCI_REG8(0x468e), 0x30 },
	{ CCI_REG8(0x4916), 0x0f },
	{ CCI_REG8(0x4918), 0x07 },
	{ CCI_REG8(0x491e), 0x02 },
	{ CCI_REG8(0x491f), 0x80 },
	{ CCI_REG8(0x4a16), 0x0f },
	{ CCI_REG8(0x4a18), 0x07 },
	{ CCI_REG8(0x4a1e), 0x02 },
	{ CCI_REG8(0x4a1f), 0x80 },
	{ CCI_REG8(0x5000), 0x2b },
	{ CCI_REG8(0x5001), 0x08 },
	{ CCI_REG8(0x5002), 0x00 },
	{ CCI_REG8(0xc200), 0xaa },
	{ CCI_REG8(0xc210), 0x1f },
	{ CCI_REG8(0xc211), 0x20 },
	{ CCI_REG8(0xc231), 0x02 },
	{ CCI_REG8(0xc232), 0x03 },
	{ CCI_REG8(0xc233), 0x1e },
	{ CCI_REG8(0xc245), 0x02 },
	{ CCI_REG8(0xc246), 0x03 },
	{ CCI_REG8(0xc247), 0x1e },
	{ CCI_REG8(0xc40f), 0x07 },
	{ CCI_REG8(0xc46d), 0x55 },
	{ CCI_REG8(0xc48f), 0x3f },
	{ CCI_REG8(0xc49c), 0x08 },
	{ CCI_REG8(0xc49e), 0x25 },
	{ CCI_REG8(0xc49f), 0x25 },
	{ CCI_REG8(0xc4a2), 0x55 },
	{ CCI_REG8(0xc4a3), 0x55 },
	{ CCI_REG8(0xc4ae), 0x00 },
	{ CCI_REG8(0xc4af), 0x00 },
	{ CCI_REG8(0xc4b0), 0x00 },
	{ CCI_REG8(0xc4b1), 0x00 },
	{ CCI_REG8(0xc4b2), 0x00 },
	{ CCI_REG8(0xc4c2), 0x07 },
	{ CCI_REG8(0xc4c3), 0x07 },
	{ CCI_REG8(0xc4c4), 0x07 },
	{ CCI_REG8(0xc4c6), 0x2c },
	{ CCI_REG8(0xc4c7), 0x2c },
	{ CCI_REG8(0xc4c8), 0x2c },
	{ CCI_REG8(0xc4c9), 0x2c },
	{ CCI_REG8(0xc4ca), 0x2c },
	{ CCI_REG8(0xc4cb), 0x2c },
	{ CCI_REG8(0xc4ce), 0x03 },
	{ CCI_REG8(0xc4cf), 0x01 },
	{ CCI_REG8(0xc4d0), 0x02 },
	{ CCI_REG8(0xc4d1), 0x00 },
	{ CCI_REG8(0xc4e0), 0x07 },
	{ CCI_REG8(0xc4e1), 0x07 },
	{ CCI_REG8(0xc4e2), 0x07 },
	{ CCI_REG8(0xc4e3), 0x1d },
	{ CCI_REG8(0xc4e9), 0x12 },
	{ CCI_REG8(0xc4ed), 0x07 },
	{ CCI_REG8(0xc4ee), 0x07 },
	{ CCI_REG8(0xc4fa), 0x07 },
	{ CCI_REG8(0xc4fb), 0x07 },
	{ CCI_REG8(0xc4fe), 0x21 },
	{ CCI_REG8(0xc4ff), 0x23 },
	{ CCI_REG8(0xc50f), 0x58 },
	{ CCI_REG8(0xc51b), 0x38 },
	{ CCI_REG8(0xc51c), 0x38 },
	{ CCI_REG8(0xc51d), 0x2c },
	{ CCI_REG8(0xc51e), 0x2c },
	{ CCI_REG8(0xc51f), 0x2c },
	{ CCI_REG8(0xc520), 0x2c },
	{ CCI_REG8(0xc521), 0x2c },
	{ CCI_REG8(0xc526), 0x0e },
	{ CCI_REG8(0xc527), 0x0e },
	{ CCI_REG8(0xc528), 0x0e },
	{ CCI_REG8(0xc529), 0x0e },
	{ CCI_REG8(0xc52a), 0x0e },
	{ CCI_REG8(0xc52b), 0x0e },
	{ CCI_REG8(0xc52c), 0x0e },
	{ CCI_REG8(0xc52d), 0x0e },
	{ CCI_REG8(0xc52e), 0x0e },
	{ CCI_REG8(0xc52f), 0x0e },
	{ CCI_REG8(0xc530), 0x0e },
	{ CCI_REG8(0xc531), 0x0e },
	{ CCI_REG8(0xc533), 0x0e },
	{ CCI_REG8(0xc534), 0x0e },
	{ CCI_REG8(0xc535), 0x0e },
	{ CCI_REG8(0xc5c3), 0x48 },
	{ CCI_REG8(0xc5c4), 0x48 },
	{ CCI_REG8(0xc5c5), 0x48 },
	{ CCI_REG8(0xc5c6), 0x48 },
	{ CCI_REG8(0xc5c7), 0x48 },
	{ CCI_REG8(0xc5c8), 0x48 },
	{ CCI_REG8(0xc5c9), 0x48 },
	{ CCI_REG8(0xc5ca), 0x48 },
	{ CCI_REG8(0xc5cb), 0x48 },
	{ CCI_REG8(0xc5cc), 0x48 },
	{ CCI_REG8(0xc5cd), 0x48 },
	{ CCI_REG8(0xc5ce), 0x48 },
	{ CCI_REG8(0xc5fe), 0x04 },
	{ CCI_REG8(0xc5ff), 0x04 },
	{ CCI_REG8(0x3046), 0x01 },
	{ CCI_REG8(0x380c), 0x01 },
	{ CCI_REG8(0x380d), 0x90 },
	{ CCI_REG8(0x380e), 0x18 },
	{ CCI_REG8(0x380f), 0x6a },
	{ CCI_REG8(0x384c), 0x01 },
	{ CCI_REG8(0x384d), 0x90 },
	{ CCI_REG8(0xc232), 0x03 },
	{ CCI_REG8(0xc233), 0x1e },
	{ CCI_REG8(0xc246), 0x03 },
	{ CCI_REG8(0xc247), 0x1e },
	{ CCI_REG8(0x4640), 0x01 },
	{ CCI_REG8(0x4641), 0x00 },
	{ CCI_REG8(0x3501), 0x18 },
	{ CCI_REG8(0x3502), 0x4a },
	{ CCI_REG8(0x3508), 0x01 },
	{ CCI_REG8(0x3509), 0x00 },
	{ CCI_REG8(0x3724), 0x2c },
};

static const struct ov50e40_mode ov50e40_supported_modes[] = {
	{
		/*
		 * 2x2 binned full field of view, 30 fps: 6250 lines of
		 * 5.33 us, 10299 pixel periods
		 */
		.width = 4096,
		.height = 3072,
		.hts = 10299,
		.vts = 6250,
		.exposure = 6218,
		.reg_list = {
			.regs = ov50e40_4096x3072_mode,
			.num_regs = ARRAY_SIZE(ov50e40_4096x3072_mode),
		},
	},
};

static int ov50e40_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov50e40 *ov50e40 = container_of(ctrl->handler, struct ov50e40,
					       ctrl_handler);
	const struct ov50e40_mode *mode = ov50e40->mode;
	s64 exposure_max;
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = mode->height + ctrl->val - OV50E40_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(ov50e40->exposure,
					 ov50e40->exposure->minimum,
					 exposure_max,
					 ov50e40->exposure->step,
					 ov50e40->exposure->default_value);
	}

	if (!pm_runtime_get_if_active(ov50e40->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(ov50e40->regmap, OV50E40_REG_AGAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(ov50e40->regmap, OV50E40_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(ov50e40->regmap, OV50E40_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(ov50e40->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov50e40_ctrl_ops = {
	.s_ctrl = ov50e40_set_ctrl,
};

static int ov50e40_init_controls(struct ov50e40 *ov50e40)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &ov50e40->ctrl_handler;
	const struct ov50e40_mode *mode = ov50e40->mode;
	s64 hblank, vblank, exposure_max;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 8);

	ov50e40->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &ov50e40_ctrl_ops,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(ov50e40_link_freq_menu) - 1,
					0, ov50e40_link_freq_menu);
	if (ov50e40->link_freq)
		ov50e40->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ov50e40->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &ov50e40_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						OV50E40_PIXEL_RATE,
						OV50E40_PIXEL_RATE, 1,
						OV50E40_PIXEL_RATE);

	hblank = mode->hts - mode->width;
	ov50e40->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &ov50e40_ctrl_ops,
					    V4L2_CID_HBLANK, hblank,
					    hblank, 1, hblank);
	if (ov50e40->hblank)
		ov50e40->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	ov50e40->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &ov50e40_ctrl_ops,
					    V4L2_CID_VBLANK, vblank,
					    OV50E40_VTS_MAX - mode->height, 1,
					    vblank);

	v4l2_ctrl_new_std(ctrl_hdlr, &ov50e40_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OV50E40_AGAIN_MIN, OV50E40_AGAIN_MAX,
			  OV50E40_AGAIN_STEP, OV50E40_AGAIN_DEFAULT);

	exposure_max = mode->vts - OV50E40_EXPOSURE_MARGIN;
	ov50e40->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &ov50e40_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      OV50E40_EXPOSURE_MIN,
					      exposure_max,
					      OV50E40_EXPOSURE_STEP,
					      mode->exposure);

	ret = v4l2_fwnode_device_parse(ov50e40->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &ov50e40_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	ov50e40->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int ov50e40_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct ov50e40 *ov50e40 = to_ov50e40(sd);
	const struct ov50e40_reg_list *reg_list = &ov50e40->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(ov50e40->dev);
	if (ret)
		return ret;

	ret = cci_write(ov50e40->regmap, OV50E40_REG_SOFTWARE_RESET, 0x01, NULL);
	if (ret)
		goto error;

	usleep_range(1000, 1500);

	cci_multi_reg_write(ov50e40->regmap, ov50e40_init,
			    ARRAY_SIZE(ov50e40_init), &ret);
	cci_multi_reg_write(ov50e40->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(ov50e40->sd.ctrl_handler);

	cci_write(ov50e40->regmap, OV50E40_REG_MODE_SELECT,
		  OV50E40_MODE_STREAMING, &ret);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(ov50e40->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(ov50e40->dev);

	return ret;
}

static int ov50e40_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct ov50e40 *ov50e40 = to_ov50e40(sd);
	int ret;

	ret = cci_write(ov50e40->regmap, OV50E40_REG_MODE_SELECT, 0, NULL);
	if (ret)
		dev_err(ov50e40->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(ov50e40->dev);

	return ret;
}

static void ov50e40_update_pad_format(const struct ov50e40_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int ov50e40_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct ov50e40 *ov50e40 = to_ov50e40(sd);
	s64 hblank, vblank, exposure_max;
	const struct ov50e40_mode *mode;

	mode = v4l2_find_nearest_size(ov50e40_supported_modes,
				      ARRAY_SIZE(ov50e40_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	ov50e40_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || ov50e40->mode == mode)
		goto set_format;

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(ov50e40->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(ov50e40->vblank, vblank,
				 OV50E40_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(ov50e40->vblank, vblank);

	exposure_max = mode->vts - OV50E40_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(ov50e40->exposure, OV50E40_EXPOSURE_MIN,
				 exposure_max, OV50E40_EXPOSURE_STEP,
				 mode->exposure);
	__v4l2_ctrl_s_ctrl(ov50e40->exposure, mode->exposure);

	if (ov50e40->sd.ctrl_handler->error)
		return ov50e40->sd.ctrl_handler->error;

	ov50e40->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int ov50e40_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int ov50e40_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(ov50e40_supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width = ov50e40_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ov50e40_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov50e40_get_selection(struct v4l2_subdev *sd,
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

static int ov50e40_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct ov50e40 *ov50e40 = to_ov50e40(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = ov50e40->mode->width,
			.height = ov50e40->mode->height,
		},
	};

	ov50e40_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops ov50e40_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops ov50e40_pad_ops = {
	.set_fmt = ov50e40_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = ov50e40_get_selection,
	.enum_mbus_code = ov50e40_enum_mbus_code,
	.enum_frame_size = ov50e40_enum_frame_size,
	.enable_streams = ov50e40_enable_streams,
	.disable_streams = ov50e40_disable_streams,
};

static const struct v4l2_subdev_ops ov50e40_subdev_ops = {
	.video = &ov50e40_video_ops,
	.pad = &ov50e40_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov50e40_internal_ops = {
	.init_state = ov50e40_init_state,
};

static const struct media_entity_operations ov50e40_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov50e40_identify_sensor(struct ov50e40 *ov50e40)
{
	u64 val;
	int ret;

	ret = cci_read(ov50e40->regmap, OV50E40_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(ov50e40->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != OV50E40_CHIP_ID) {
		dev_err(ov50e40->dev, "chip id mismatch: %x!=%llx\n",
			OV50E40_CHIP_ID, val);
		return -ENODEV;
	}

	return 0;
}

static int ov50e40_check_hwcfg(struct ov50e40 *ov50e40)
{
	struct fwnode_handle *fwnode = dev_fwnode(ov50e40->dev), *ep;
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OV50E40_TRIOS) {
		dev_err(ov50e40->dev, "Invalid number of trios: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(ov50e40->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       ov50e40_link_freq_menu,
				       ARRAY_SIZE(ov50e40_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* Stock order: rails, MCLK 1 ms later, reset release 5 ms after it, 10 ms */
static int ov50e40_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov50e40 *ov50e40 = to_ov50e40(sd);
	int ret;

	ret = regulator_bulk_enable(OV50E40_NUM_SUPPLIES, ov50e40->supplies);
	if (ret)
		return ret;

	usleep_range(1000, 1500);

	ret = clk_prepare_enable(ov50e40->mclk);
	if (ret)
		goto disable_regulators;

	usleep_range(5000, 5500);
	gpiod_set_value_cansleep(ov50e40->reset_gpio, 0);
	usleep_range(10000, 11000);

	return 0;

disable_regulators:
	regulator_bulk_disable(OV50E40_NUM_SUPPLIES, ov50e40->supplies);

	return ret;
}

/* Stock order: MCLK, 2 ms, reset, rails */
static int ov50e40_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov50e40 *ov50e40 = to_ov50e40(sd);

	clk_disable_unprepare(ov50e40->mclk);
	usleep_range(2000, 2500);
	gpiod_set_value_cansleep(ov50e40->reset_gpio, 1);

	regulator_bulk_disable(OV50E40_NUM_SUPPLIES, ov50e40->supplies);
	usleep_range(1000, 1500);

	return 0;
}

static int ov50e40_probe(struct i2c_client *client)
{
	struct ov50e40 *ov50e40;
	unsigned long freq;
	unsigned int i;
	int ret;

	ov50e40 = devm_kzalloc(&client->dev, sizeof(*ov50e40), GFP_KERNEL);
	if (!ov50e40)
		return -ENOMEM;

	ov50e40->dev = &client->dev;
	v4l2_i2c_subdev_init(&ov50e40->sd, client, &ov50e40_subdev_ops);

	ov50e40->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ov50e40->regmap))
		return dev_err_probe(ov50e40->dev, PTR_ERR(ov50e40->regmap),
				     "failed to init CCI\n");

	ov50e40->mclk = devm_v4l2_sensor_clk_get(ov50e40->dev, NULL);
	if (IS_ERR(ov50e40->mclk))
		return dev_err_probe(ov50e40->dev, PTR_ERR(ov50e40->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(ov50e40->mclk);
	if (freq != OV50E40_MCLK_FREQ_24MHZ)
		return dev_err_probe(ov50e40->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = ov50e40_check_hwcfg(ov50e40);
	if (ret)
		return dev_err_probe(ov50e40->dev, ret,
				     "failed to check HW configuration\n");

	ov50e40->reset_gpio = devm_gpiod_get_optional(ov50e40->dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(ov50e40->reset_gpio))
		return dev_err_probe(ov50e40->dev, PTR_ERR(ov50e40->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < OV50E40_NUM_SUPPLIES; i++)
		ov50e40->supplies[i].supply = ov50e40_supply_names[i];

	ret = devm_regulator_bulk_get(ov50e40->dev, OV50E40_NUM_SUPPLIES,
				      ov50e40->supplies);
	if (ret)
		return dev_err_probe(ov50e40->dev, ret,
				     "failed to get supply regulators\n");

	ret = ov50e40_power_on(ov50e40->dev);
	if (ret)
		return ret;

	ret = ov50e40_identify_sensor(ov50e40);
	if (ret) {
		dev_err_probe(ov50e40->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	ov50e40->mode = &ov50e40_supported_modes[0];
	ret = ov50e40_init_controls(ov50e40);
	if (ret) {
		dev_err_probe(ov50e40->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	ov50e40->sd.state_lock = ov50e40->ctrl_handler.lock;
	ov50e40->sd.internal_ops = &ov50e40_internal_ops;
	ov50e40->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov50e40->sd.entity.ops = &ov50e40_subdev_entity_ops;
	ov50e40->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov50e40->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ov50e40->sd.entity, 1, &ov50e40->pad);
	if (ret) {
		dev_err_probe(ov50e40->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&ov50e40->sd);
	if (ret < 0) {
		dev_err_probe(ov50e40->dev, ret, "failed to init subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(ov50e40->dev);
	pm_runtime_enable(ov50e40->dev);

	ret = v4l2_async_register_subdev_sensor(&ov50e40->sd);
	if (ret < 0) {
		dev_err_probe(ov50e40->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(ov50e40->dev, 1000);
	pm_runtime_use_autosuspend(ov50e40->dev);
	pm_runtime_idle(ov50e40->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&ov50e40->sd);
	pm_runtime_disable(ov50e40->dev);
	pm_runtime_set_suspended(ov50e40->dev);

media_entity_cleanup:
	media_entity_cleanup(&ov50e40->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(ov50e40->sd.ctrl_handler);

power_off:
	ov50e40_power_off(ov50e40->dev);

	return ret;
}

static void ov50e40_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov50e40 *ov50e40 = to_ov50e40(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(ov50e40->dev);

	if (!pm_runtime_status_suspended(ov50e40->dev)) {
		ov50e40_power_off(ov50e40->dev);
		pm_runtime_set_suspended(ov50e40->dev);
	}
}

static const struct dev_pm_ops ov50e40_pm_ops = {
	SET_RUNTIME_PM_OPS(ov50e40_power_off, ov50e40_power_on, NULL)
};

static const struct of_device_id ov50e40_of_match[] = {
	{ .compatible = "ovti,ov50e40" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov50e40_of_match);

static struct i2c_driver ov50e40_i2c_driver = {
	.driver = {
		.name = "ov50e40",
		.pm = &ov50e40_pm_ops,
		.of_match_table = ov50e40_of_match,
	},
	.probe = ov50e40_probe,
	.remove = ov50e40_remove,
};

module_i2c_driver(ov50e40_i2c_driver);

MODULE_DESCRIPTION("OmniVision OV50E40 image sensor driver");
MODULE_LICENSE("GPL");
