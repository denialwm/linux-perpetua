// SPDX-License-Identifier: GPL-2.0
/*
 * SmartSens CS-series image sensor driver
 *
 * Two sensors that answer at register 0x3107 with chip IDs 0xc658 (13 MP)
 * and 0xd154 (8 MP). Register tables are the OnePlus Pad 4 (iceland) stock
 * module settings for its rear and front cameras. Driver structure follows
 * ov02f10.c.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define SC_MCLK_FREQ_19_2MHZ		(19200 * HZ_PER_KHZ)
#define SC_DATA_LANES			4

#define SC_REG_CHIP_ID			CCI_REG16(0x3107)
#define SC_REG_SOFTWARE_RESET		CCI_REG8(0x0103)
#define SC_REG_STREAM			CCI_REG8(0x0100)

/* Frame length in lines */
#define SC_REG_VTS			CCI_REG16(0x320e)
#define SC_VTS_MAX			0x7fff

/* Exposure in half lines, the low four bits a fraction */
#define SC_REG_EXPOSURE			CCI_REG24(0x3e00)
#define SC_EXPOSURE_MIN			1
#define SC_EXPOSURE_STEP		1
#define SC_EXPOSURE_MARGIN		4

/*
 * Gain in 1/128 steps up to 32x. The analogue register doubles the gain
 * for each bit set from the bottom (0, 1, 3, 7, 15 and 31 are 1x to 32x);
 * the digital fine gain, 0x80 for 1x, fills in between.
 */
#define SC_REG_DGAIN_COARSE		CCI_REG8(0x3e06)
#define SC_REG_DGAIN_FINE		CCI_REG8(0x3e07)
#define SC_GAIN_UNIT			128
#define SC_AGAIN_MIN			SC_GAIN_UNIT
#define SC_AGAIN_MAX			(32 * SC_GAIN_UNIT)
#define SC_AGAIN_STEP			1
#define SC_AGAIN_DEFAULT		SC_GAIN_UNIT

/*
 * One-time programmable memory of the D154, read a page at a time: the page
 * goes to a buffer at 0x8000 + 0x200 * page, and its bytes from offset 0x7a
 * on are module data. The bytes of all pages follow each other in the
 * nvmem device.
 */
#define SC_REG_OTP_VREF(n)		CCI_REG8(0x36b0 + (n))
#define SC_REG_OTP_CTRL			CCI_REG8(0x4400)
#define SC_OTP_CTRL_LOAD		0x11
#define SC_REG_OTP_MODE			CCI_REG8(0x4401)
#define SC_OTP_MODE_READ		0x13
#define SC_REG_OTP_STATUS		CCI_REG8(0x4407)
#define SC_REG_OTP_START		CCI_REG16(0x4408)
#define SC_REG_OTP_END			CCI_REG16(0x440a)
#define SC_REG_OTP_PAGE			CCI_REG8(0x4412)
#define SC_OTP_BUF(page)		(0x8000 + 0x200 * (page))
#define SC_OTP_DATA_OFFSET		0x7a
#define SC_OTP_LOAD_US			(10 * USEC_PER_MSEC)

struct sc_otp_page {
	u8 page;
	u16 size;
};

struct sc_otp {
	const struct sc_otp_page *pages;
	unsigned int num_pages;
	unsigned int size;
};

#define to_sc(_sd)			container_of(_sd, struct sc, sd)

struct sc_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct sc_mode {
	u32 width;
	u32 height;
	const struct sc_reg_list reg_list;
};

/*
 * hts is the line length in pixels at pixel_rate, which is the CSI-2 rate
 * of 10-bit pixels over four lanes; vts is the stock frame length
 */
struct sc_chip {
	u16 chip_id;
	s64 link_freq;
	u64 pixel_rate;
	u32 hts;
	u32 vts;
	u32 native_width;
	u32 native_height;
	u32 again_reg;
	const struct sc_reg_list init;
	const struct sc_mode *modes;
	unsigned int num_modes;
	const struct sc_otp *otp;
};

enum {
	SC_SUPPLY_DOVDD,	/* Digital I/O power */
	SC_SUPPLY_AVDD,		/* Analog power */
	SC_SUPPLY_DVDD,		/* Digital core power */
	SC_NUM_SUPPLIES,
};

static const char * const sc_supply_names[] = {
	[SC_SUPPLY_DOVDD] = "dovdd",
	[SC_SUPPLY_AVDD] = "avdd",
	[SC_SUPPLY_DVDD] = "dvdd",
};

struct sc {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[SC_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;

	const struct sc_chip *chip;
	const struct sc_mode *mode;

	u8 *otp_data;
};

/* Global init after the software reset */
static const struct cci_reg_sequence sc_c658_init[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x36e9), 0x80 },
	{ CCI_REG8(0x37f9), 0x80 },
	{ CCI_REG8(0x36ea), 0x17 },
	{ CCI_REG8(0x36eb), 0x0c },
	{ CCI_REG8(0x36ec), 0x4b },
	{ CCI_REG8(0x36ed), 0x18 },
	{ CCI_REG8(0x37fa), 0x19 },
	{ CCI_REG8(0x37fb), 0xb2 },
	{ CCI_REG8(0x37fc), 0x10 },
	{ CCI_REG8(0x37fd), 0x3a },
	{ CCI_REG8(0x301f), 0x60 },
	{ CCI_REG8(0x320c), 0x04 },
	{ CCI_REG8(0x320d), 0xe2 },
	{ CCI_REG8(0x320e), 0x0c },
	{ CCI_REG8(0x320f), 0x80 },
	{ CCI_REG8(0x3279), 0x10 },
	{ CCI_REG8(0x3301), 0x07 },
	{ CCI_REG8(0x3302), 0x18 },
	{ CCI_REG8(0x3304), 0x38 },
	{ CCI_REG8(0x3306), 0x58 },
	{ CCI_REG8(0x3308), 0x0c },
	{ CCI_REG8(0x3309), 0x80 },
	{ CCI_REG8(0x330b), 0xb8 },
	{ CCI_REG8(0x330d), 0x10 },
	{ CCI_REG8(0x330e), 0x2b },
	{ CCI_REG8(0x3314), 0x15 },
	{ CCI_REG8(0x331c), 0x04 },
	{ CCI_REG8(0x331e), 0x29 },
	{ CCI_REG8(0x331f), 0x71 },
	{ CCI_REG8(0x3320), 0x08 },
	{ CCI_REG8(0x3323), 0x04 },
	{ CCI_REG8(0x3333), 0x10 },
	{ CCI_REG8(0x3334), 0x40 },
	{ CCI_REG8(0x334a), 0x82 },
	{ CCI_REG8(0x3351), 0x06 },
	{ CCI_REG8(0x335d), 0x60 },
	{ CCI_REG8(0x3364), 0x56 },
	{ CCI_REG8(0x337f), 0x13 },
	{ CCI_REG8(0x3390), 0x09 },
	{ CCI_REG8(0x3391), 0x0f },
	{ CCI_REG8(0x3392), 0x1f },
	{ CCI_REG8(0x3393), 0x10 },
	{ CCI_REG8(0x3394), 0x1c },
	{ CCI_REG8(0x3395), 0x20 },
	{ CCI_REG8(0x33ac), 0x0a },
	{ CCI_REG8(0x33ad), 0x14 },
	{ CCI_REG8(0x33ae), 0x14 },
	{ CCI_REG8(0x33af), 0x60 },
	{ CCI_REG8(0x33b0), 0x0f },
	{ CCI_REG8(0x33b2), 0x50 },
	{ CCI_REG8(0x33b3), 0x28 },
	{ CCI_REG8(0x349f), 0x1e },
	{ CCI_REG8(0x34a6), 0x08 },
	{ CCI_REG8(0x34a7), 0x09 },
	{ CCI_REG8(0x34a8), 0x18 },
	{ CCI_REG8(0x34a9), 0x18 },
	{ CCI_REG8(0x34f8), 0x0f },
	{ CCI_REG8(0x34f9), 0x18 },
	{ CCI_REG8(0x3630), 0xc6 },
	{ CCI_REG8(0x3637), 0x47 },
	{ CCI_REG8(0x363c), 0x8d },
	{ CCI_REG8(0x3650), 0x31 },
	{ CCI_REG8(0x3651), 0x9d },
	{ CCI_REG8(0x3652), 0x32 },
	{ CCI_REG8(0x3653), 0x23 },
	{ CCI_REG8(0x3654), 0x31 },
	{ CCI_REG8(0x3670), 0x0c },
	{ CCI_REG8(0x367b), 0x56 },
	{ CCI_REG8(0x367c), 0x67 },
	{ CCI_REG8(0x367d), 0x67 },
	{ CCI_REG8(0x367e), 0x08 },
	{ CCI_REG8(0x367f), 0x0f },
	{ CCI_REG8(0x3690), 0x84 },
	{ CCI_REG8(0x3691), 0x87 },
	{ CCI_REG8(0x3692), 0x8e },
	{ CCI_REG8(0x3693), 0xb3 },
	{ CCI_REG8(0x3694), 0x09 },
	{ CCI_REG8(0x3695), 0x0b },
	{ CCI_REG8(0x3696), 0x0f },
	{ CCI_REG8(0x3697), 0x8b },
	{ CCI_REG8(0x3698), 0x8d },
	{ CCI_REG8(0x3699), 0x8d },
	{ CCI_REG8(0x369a), 0x08 },
	{ CCI_REG8(0x369b), 0x0f },
	{ CCI_REG8(0x370f), 0x01 },
	{ CCI_REG8(0x3724), 0x41 },
	{ CCI_REG8(0x3771), 0x03 },
	{ CCI_REG8(0x3772), 0x03 },
	{ CCI_REG8(0x3773), 0x63 },
	{ CCI_REG8(0x377a), 0x08 },
	{ CCI_REG8(0x377b), 0x0f },
	{ CCI_REG8(0x3900), 0x0d },
	{ CCI_REG8(0x3903), 0x20 },
	{ CCI_REG8(0x3905), 0x0c },
	{ CCI_REG8(0x3908), 0x40 },
	{ CCI_REG8(0x391a), 0x30 },
	{ CCI_REG8(0x391b), 0x21 },
	{ CCI_REG8(0x391c), 0x13 },
	{ CCI_REG8(0x391d), 0x00 },
	{ CCI_REG8(0x391f), 0x41 },
	{ CCI_REG8(0x3926), 0xe0 },
	{ CCI_REG8(0x393f), 0x80 },
	{ CCI_REG8(0x3940), 0x00 },
	{ CCI_REG8(0x3941), 0x00 },
	{ CCI_REG8(0x3942), 0x00 },
	{ CCI_REG8(0x3943), 0x7c },
	{ CCI_REG8(0x3944), 0x79 },
	{ CCI_REG8(0x3945), 0x7d },
	{ CCI_REG8(0x3946), 0x7d },
	{ CCI_REG8(0x39dd), 0x00 },
	{ CCI_REG8(0x39de), 0x04 },
	{ CCI_REG8(0x39e7), 0x04 },
	{ CCI_REG8(0x39e8), 0x03 },
	{ CCI_REG8(0x39e9), 0x80 },
	{ CCI_REG8(0x3e00), 0x00 },
	{ CCI_REG8(0x3e01), 0x0f },
	{ CCI_REG8(0x3e02), 0x90 },
	{ CCI_REG8(0x3f08), 0x0a },
	{ CCI_REG8(0x4401), 0x13 },
	{ CCI_REG8(0x4402), 0x03 },
	{ CCI_REG8(0x4403), 0x0c },
	{ CCI_REG8(0x4404), 0x24 },
	{ CCI_REG8(0x4405), 0x2f },
	{ CCI_REG8(0x440c), 0x3c },
	{ CCI_REG8(0x440d), 0x3c },
	{ CCI_REG8(0x440e), 0x2d },
	{ CCI_REG8(0x440f), 0x4b },
	{ CCI_REG8(0x4413), 0x01 },
	{ CCI_REG8(0x441b), 0x18 },
	{ CCI_REG8(0x4509), 0x18 },
	{ CCI_REG8(0x450d), 0x13 },
	{ CCI_REG8(0x4800), 0x24 },
	{ CCI_REG8(0x4819), 0x71 },
	{ CCI_REG8(0x4837), 0x18 },
	{ CCI_REG8(0x5000), 0x1e },
	{ CCI_REG8(0x5002), 0x00 },
	{ CCI_REG8(0x5015), 0x41 },
	{ CCI_REG8(0x550e), 0x00 },
	{ CCI_REG8(0x550f), 0x7a },
	{ CCI_REG8(0x5780), 0x76 },
	{ CCI_REG8(0x5784), 0x08 },
	{ CCI_REG8(0x5787), 0x0a },
	{ CCI_REG8(0x5788), 0x0a },
	{ CCI_REG8(0x5789), 0x0a },
	{ CCI_REG8(0x578a), 0x0a },
	{ CCI_REG8(0x578b), 0x0a },
	{ CCI_REG8(0x578c), 0x0a },
	{ CCI_REG8(0x578d), 0x40 },
	{ CCI_REG8(0x5790), 0x08 },
	{ CCI_REG8(0x5791), 0x04 },
	{ CCI_REG8(0x5792), 0x04 },
	{ CCI_REG8(0x5793), 0x08 },
	{ CCI_REG8(0x5794), 0x04 },
	{ CCI_REG8(0x5795), 0x04 },
	{ CCI_REG8(0x5799), 0x44 },
	{ CCI_REG8(0x57a6), 0x1c },
	{ CCI_REG8(0x57aa), 0xeb },
	{ CCI_REG8(0x57ab), 0x7f },
	{ CCI_REG8(0x57ac), 0x00 },
	{ CCI_REG8(0x57ad), 0x00 },
	{ CCI_REG8(0x58e0), 0xb4 },
	{ CCI_REG8(0x36e9), 0x25 },
	{ CCI_REG8(0x37f9), 0x24 },
};

/* Full array */
static const struct cci_reg_sequence sc_c658_4160x3120[] = {
	{ CCI_REG8(0x3200), 0x00 },
	{ CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 },
	{ CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x10 },
	{ CCI_REG8(0x3205), 0x7f },
	{ CCI_REG8(0x3206), 0x0c },
	{ CCI_REG8(0x3207), 0x3f },
	{ CCI_REG8(0x3208), 0x10 },
	{ CCI_REG8(0x3209), 0x40 },
	{ CCI_REG8(0x320a), 0x0c },
	{ CCI_REG8(0x320b), 0x30 },
	{ CCI_REG8(0x3210), 0x00 },
	{ CCI_REG8(0x3211), 0x20 },
	{ CCI_REG8(0x3212), 0x00 },
	{ CCI_REG8(0x3213), 0x08 },
	{ CCI_REG8(0x3215), 0x11 },
	{ CCI_REG8(0x3220), 0x00 },
	{ CCI_REG8(0x3221), 0x66 },
	{ CCI_REG8(0x5000), 0x1e },
	{ CCI_REG8(0x5900), 0x01 },
	{ CCI_REG8(0x5901), 0x00 },
};

/* 2x2 binned */
static const struct cci_reg_sequence sc_c658_2104x1560[] = {
	{ CCI_REG8(0x3200), 0x00 },
	{ CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 },
	{ CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x10 },
	{ CCI_REG8(0x3205), 0x7f },
	{ CCI_REG8(0x3206), 0x0c },
	{ CCI_REG8(0x3207), 0x3f },
	{ CCI_REG8(0x3208), 0x08 },
	{ CCI_REG8(0x3209), 0x38 },
	{ CCI_REG8(0x320a), 0x06 },
	{ CCI_REG8(0x320b), 0x18 },
	{ CCI_REG8(0x3210), 0x00 },
	{ CCI_REG8(0x3211), 0x04 },
	{ CCI_REG8(0x3212), 0x00 },
	{ CCI_REG8(0x3213), 0x04 },
	{ CCI_REG8(0x3215), 0x31 },
	{ CCI_REG8(0x3220), 0x01 },
	{ CCI_REG8(0x3221), 0x66 },
	{ CCI_REG8(0x5000), 0x5e },
	{ CCI_REG8(0x5900), 0x01 },
	{ CCI_REG8(0x5901), 0x04 },
};

/* Global init after the software reset */
static const struct cci_reg_sequence sc_d154_init[] = {
	{ CCI_REG8(0x36e9), 0x80 },
	{ CCI_REG8(0x37f9), 0x80 },
	{ CCI_REG8(0x36ea), 0x0b },
	{ CCI_REG8(0x36eb), 0x0c },
	{ CCI_REG8(0x36ec), 0x5a },
	{ CCI_REG8(0x36ed), 0x28 },
	{ CCI_REG8(0x37fa), 0x0b },
	{ CCI_REG8(0x37fb), 0x71 },
	{ CCI_REG8(0x37fc), 0x10 },
	{ CCI_REG8(0x37fd), 0x28 },
	{ CCI_REG8(0x36e9), 0x22 },
	{ CCI_REG8(0x37f9), 0x24 },
	{ CCI_REG8(0x301f), 0x44 },
	{ CCI_REG8(0x3205), 0xc7 },
	{ CCI_REG8(0x3211), 0x04 },
	{ CCI_REG8(0x3221), 0x66 },
	{ CCI_REG8(0x3215), 0x11 },
	{ CCI_REG8(0x3220), 0x00 },
	{ CCI_REG8(0x3221), 0x66 },
	{ CCI_REG8(0x3270), 0x00 },
	{ CCI_REG8(0x3271), 0x00 },
	{ CCI_REG8(0x3272), 0x00 },
	{ CCI_REG8(0x3273), 0x03 },
	{ CCI_REG8(0x3301), 0x08 },
	{ CCI_REG8(0x3303), 0x10 },
	{ CCI_REG8(0x3306), 0x84 },
	{ CCI_REG8(0x3307), 0x04 },
	{ CCI_REG8(0x3309), 0x88 },
	{ CCI_REG8(0x330a), 0x01 },
	{ CCI_REG8(0x330b), 0x0c },
	{ CCI_REG8(0x330c), 0x10 },
	{ CCI_REG8(0x330d), 0x18 },
	{ CCI_REG8(0x330e), 0x30 },
	{ CCI_REG8(0x330f), 0x04 },
	{ CCI_REG8(0x3310), 0x02 },
	{ CCI_REG8(0x3314), 0x15 },
	{ CCI_REG8(0x3317), 0x04 },
	{ CCI_REG8(0x331f), 0x79 },
	{ CCI_REG8(0x3326), 0x0e },
	{ CCI_REG8(0x3327), 0x0a },
	{ CCI_REG8(0x3329), 0x0b },
	{ CCI_REG8(0x3333), 0x10 },
	{ CCI_REG8(0x3334), 0x40 },
	{ CCI_REG8(0x3347), 0x0f },
	{ CCI_REG8(0x335d), 0x60 },
	{ CCI_REG8(0x3364), 0x56 },
	{ CCI_REG8(0x336c), 0xce },
	{ CCI_REG8(0x3390), 0x08 },
	{ CCI_REG8(0x3391), 0x09 },
	{ CCI_REG8(0x3392), 0x0f },
	{ CCI_REG8(0x3393), 0x10 },
	{ CCI_REG8(0x3394), 0x20 },
	{ CCI_REG8(0x3395), 0x28 },
	{ CCI_REG8(0x33ad), 0x3c },
	{ CCI_REG8(0x33af), 0x70 },
	{ CCI_REG8(0x33b2), 0x70 },
	{ CCI_REG8(0x33b3), 0x40 },
	{ CCI_REG8(0x349f), 0x1e },
	{ CCI_REG8(0x34a6), 0x09 },
	{ CCI_REG8(0x34a7), 0x0f },
	{ CCI_REG8(0x34a8), 0x30 },
	{ CCI_REG8(0x34a9), 0x20 },
	{ CCI_REG8(0x34f8), 0x1f },
	{ CCI_REG8(0x34f9), 0x08 },
	{ CCI_REG8(0x3637), 0x43 },
	{ CCI_REG8(0x363c), 0x8d },
	{ CCI_REG8(0x3670), 0x4a },
	{ CCI_REG8(0x3674), 0xf6 },
	{ CCI_REG8(0x3675), 0xdc },
	{ CCI_REG8(0x3676), 0xcc },
	{ CCI_REG8(0x367c), 0x09 },
	{ CCI_REG8(0x367d), 0x0f },
	{ CCI_REG8(0x3690), 0x34 },
	{ CCI_REG8(0x3691), 0x44 },
	{ CCI_REG8(0x3692), 0x55 },
	{ CCI_REG8(0x3698), 0x86 },
	{ CCI_REG8(0x3699), 0x8d },
	{ CCI_REG8(0x369a), 0x99 },
	{ CCI_REG8(0x369b), 0xaf },
	{ CCI_REG8(0x369c), 0x0f },
	{ CCI_REG8(0x369d), 0x1f },
	{ CCI_REG8(0x36a2), 0x09 },
	{ CCI_REG8(0x36a3), 0x0b },
	{ CCI_REG8(0x36a4), 0x0f },
	{ CCI_REG8(0x36b0), 0x48 },
	{ CCI_REG8(0x36b1), 0x38 },
	{ CCI_REG8(0x36b2), 0x41 },
	{ CCI_REG8(0x370f), 0x01 },
	{ CCI_REG8(0x3724), 0xc1 },
	{ CCI_REG8(0x3771), 0x07 },
	{ CCI_REG8(0x3772), 0x03 },
	{ CCI_REG8(0x3773), 0x63 },
	{ CCI_REG8(0x377a), 0x08 },
	{ CCI_REG8(0x377b), 0x0f },
	{ CCI_REG8(0x3901), 0x04 },
	{ CCI_REG8(0x3903), 0xa0 },
	{ CCI_REG8(0x3905), 0x8d },
	{ CCI_REG8(0x391d), 0x01 },
	{ CCI_REG8(0x3926), 0x23 },
	{ CCI_REG8(0x393f), 0x80 },
	{ CCI_REG8(0x3940), 0x00 },
	{ CCI_REG8(0x3941), 0x00 },
	{ CCI_REG8(0x3942), 0x00 },
	{ CCI_REG8(0x3943), 0x63 },
	{ CCI_REG8(0x3944), 0x5f },
	{ CCI_REG8(0x3c04), 0x01 },
	{ CCI_REG8(0x3e00), 0x01 },
	{ CCI_REG8(0x3e01), 0x38 },
	{ CCI_REG8(0x3e02), 0x00 },
	{ CCI_REG8(0x4401), 0x13 },
	{ CCI_REG8(0x4402), 0x03 },
	{ CCI_REG8(0x4403), 0x0e },
	{ CCI_REG8(0x4404), 0x28 },
	{ CCI_REG8(0x4405), 0x34 },
	{ CCI_REG8(0x4407), 0x0e },
	{ CCI_REG8(0x440c), 0x42 },
	{ CCI_REG8(0x440d), 0x42 },
	{ CCI_REG8(0x440e), 0x32 },
	{ CCI_REG8(0x440f), 0x53 },
	{ CCI_REG8(0x4412), 0x01 },
	{ CCI_REG8(0x4424), 0x01 },
	{ CCI_REG8(0x442d), 0x00 },
	{ CCI_REG8(0x442e), 0x00 },
	{ CCI_REG8(0x4509), 0x28 },
	{ CCI_REG8(0x450d), 0x18 },
	{ CCI_REG8(0x451d), 0xc8 },
	{ CCI_REG8(0x4526), 0x09 },
	{ CCI_REG8(0x5000), 0x0e },
	{ CCI_REG8(0x5900), 0x01 },
	{ CCI_REG8(0x5901), 0x00 },
};

/* Full array */
static const struct cci_reg_sequence sc_d154_3264x2448[] = {
	{ CCI_REG8(0x3200), 0x00 },
	{ CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 },
	{ CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x0c },
	{ CCI_REG8(0x3205), 0xc7 },
	{ CCI_REG8(0x3206), 0x09 },
	{ CCI_REG8(0x3207), 0x9f },
	{ CCI_REG8(0x3208), 0x0c },
	{ CCI_REG8(0x3209), 0xc0 },
	{ CCI_REG8(0x320a), 0x09 },
	{ CCI_REG8(0x320b), 0x90 },
	{ CCI_REG8(0x3210), 0x00 },
	{ CCI_REG8(0x3211), 0x04 },
	{ CCI_REG8(0x3212), 0x00 },
	{ CCI_REG8(0x3213), 0x08 },
	{ CCI_REG8(0x3215), 0x11 },
	{ CCI_REG8(0x3220), 0x00 },
	{ CCI_REG8(0x3221), 0x66 },
	{ CCI_REG8(0x5000), 0x0e },
	{ CCI_REG8(0x5900), 0x01 },
	{ CCI_REG8(0x5901), 0x00 },
};

/* 2x2 binned */
static const struct cci_reg_sequence sc_d154_1632x1224[] = {
	{ CCI_REG8(0x3200), 0x00 },
	{ CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 },
	{ CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x0c },
	{ CCI_REG8(0x3205), 0xc7 },
	{ CCI_REG8(0x3206), 0x09 },
	{ CCI_REG8(0x3207), 0x9f },
	{ CCI_REG8(0x3208), 0x06 },
	{ CCI_REG8(0x3209), 0x60 },
	{ CCI_REG8(0x320a), 0x04 },
	{ CCI_REG8(0x320b), 0xc8 },
	{ CCI_REG8(0x3210), 0x00 },
	{ CCI_REG8(0x3211), 0x02 },
	{ CCI_REG8(0x3212), 0x00 },
	{ CCI_REG8(0x3213), 0x04 },
	{ CCI_REG8(0x3215), 0x31 },
	{ CCI_REG8(0x3220), 0x01 },
	{ CCI_REG8(0x3221), 0x66 },
	{ CCI_REG8(0x5000), 0x4e },
	{ CCI_REG8(0x5900), 0xf1 },
	{ CCI_REG8(0x5901), 0x04 },
};

static const struct sc_mode sc_c658_modes[] = {
	{
		.width = 4160,
		.height = 3120,
		.reg_list = {
			.regs = sc_c658_4160x3120,
			.num_regs = ARRAY_SIZE(sc_c658_4160x3120),
		},
	},
	{
		.width = 2104,
		.height = 1560,
		.reg_list = {
			.regs = sc_c658_2104x1560,
			.num_regs = ARRAY_SIZE(sc_c658_2104x1560),
		},
	},
};

static const struct sc_mode sc_d154_modes[] = {
	{
		.width = 3264,
		.height = 2448,
		.reg_list = {
			.regs = sc_d154_3264x2448,
			.num_regs = ARRAY_SIZE(sc_d154_3264x2448),
		},
	},
	{
		.width = 1632,
		.height = 1224,
		.reg_list = {
			.regs = sc_d154_1632x1224,
			.num_regs = ARRAY_SIZE(sc_d154_1632x1224),
		},
	},
};

/* Stock output pixel clock 529.92 MHz: 662.4 MHz link, 30 fps at 3200 lines */
static const struct sc_chip sc_c658 = {
	.chip_id = 0xc658,
	.link_freq = 662400000,
	.pixel_rate = 529920000,
	.hts = 5520,
	.vts = 3200,
	.native_width = 4224,
	.native_height = 3136,
	.again_reg = CCI_REG8(0x3e09),
	.init = {
		.regs = sc_c658_init,
		.num_regs = ARRAY_SIZE(sc_c658_init),
	},
	.modes = sc_c658_modes,
	.num_modes = ARRAY_SIZE(sc_c658_modes),
};

/*
 * Pages 1 to 10 of the D154 hold two copies of the module data, five pages
 * each, with fewer bytes in the last page of each copy
 */
static const struct sc_otp_page sc_d154_otp_pages[] = {
	{ 1, 390 }, { 2, 390 }, { 3, 390 }, { 4, 390 }, { 5, 273 },
	{ 6, 390 }, { 7, 390 }, { 8, 390 }, { 9, 390 }, { 10, 273 },
};

static const struct sc_otp sc_d154_otp = {
	.pages = sc_d154_otp_pages,
	.num_pages = ARRAY_SIZE(sc_d154_otp_pages),
	.size = 2 * (4 * 390 + 273),
};

/* Stock output pixel clock 264 MHz: 330 MHz link, 30 fps at 2500 lines */
static const struct sc_chip sc_d154 = {
	.chip_id = 0xd154,
	.link_freq = 330000000,
	.pixel_rate = 264000000,
	.hts = 3520,
	.vts = 2500,
	.native_width = 3272,
	.native_height = 2464,
	.again_reg = CCI_REG8(0x3e08),
	.init = {
		.regs = sc_d154_init,
		.num_regs = ARRAY_SIZE(sc_d154_init),
	},
	.modes = sc_d154_modes,
	.num_modes = ARRAY_SIZE(sc_d154_modes),
	.otp = &sc_d154_otp,
};

static int sc_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc *sc = container_of(ctrl->handler, struct sc, ctrl_handler);
	const struct sc_mode *mode = sc->mode;
	s64 exposure_max;
	unsigned int shift;
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = mode->height + ctrl->val - SC_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(sc->exposure, sc->exposure->minimum,
					 exposure_max, sc->exposure->step,
					 sc->exposure->default_value);
	}

	if (!pm_runtime_get_if_active(sc->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		shift = ilog2(ctrl->val / SC_GAIN_UNIT);
		cci_write(sc->regmap, sc->chip->again_reg, BIT(shift) - 1, &ret);
		cci_write(sc->regmap, SC_REG_DGAIN_COARSE, 0, &ret);
		cci_write(sc->regmap, SC_REG_DGAIN_FINE, ctrl->val >> shift, &ret);
		break;
	case V4L2_CID_EXPOSURE:
		cci_write(sc->regmap, SC_REG_EXPOSURE, ctrl->val << 5, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(sc->regmap, SC_REG_VTS, mode->height + ctrl->val,
			  &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sc->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc_ctrl_ops = {
	.s_ctrl = sc_set_ctrl,
};

static int sc_init_controls(struct sc *sc)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &sc->ctrl_handler;
	const struct sc_chip *chip = sc->chip;
	const struct sc_mode *mode = sc->mode;
	s64 hblank, vblank, exposure_max;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 8);

	sc->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &sc_ctrl_ops,
					       V4L2_CID_LINK_FREQ, 0, 0,
					       &chip->link_freq);
	if (sc->link_freq)
		sc->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sc->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &sc_ctrl_ops,
					   V4L2_CID_PIXEL_RATE,
					   chip->pixel_rate, chip->pixel_rate,
					   1, chip->pixel_rate);

	hblank = chip->hts - mode->width;
	sc->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &sc_ctrl_ops,
				       V4L2_CID_HBLANK, hblank, hblank, 1,
				       hblank);
	if (sc->hblank)
		sc->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = chip->vts - mode->height;
	sc->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &sc_ctrl_ops,
				       V4L2_CID_VBLANK, vblank,
				       SC_VTS_MAX - mode->height, 1, vblank);

	v4l2_ctrl_new_std(ctrl_hdlr, &sc_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  SC_AGAIN_MIN, SC_AGAIN_MAX, SC_AGAIN_STEP,
			  SC_AGAIN_DEFAULT);

	exposure_max = chip->vts - SC_EXPOSURE_MARGIN;
	sc->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &sc_ctrl_ops,
					 V4L2_CID_EXPOSURE, SC_EXPOSURE_MIN,
					 exposure_max, SC_EXPOSURE_STEP,
					 exposure_max);

	ret = v4l2_fwnode_device_parse(sc->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &sc_ctrl_ops, &props);
	if (ret)
		goto error_free_hdlr;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	sc->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

/* Any mode change resets the blanking ranges to the new size */
static void sc_update_blanking(struct sc *sc)
{
	const struct sc_chip *chip = sc->chip;
	const struct sc_mode *mode = sc->mode;
	s64 hblank = chip->hts - mode->width;
	s64 vblank = chip->vts - mode->height;

	__v4l2_ctrl_modify_range(sc->hblank, hblank, hblank, 1, hblank);
	__v4l2_ctrl_modify_range(sc->vblank, vblank, SC_VTS_MAX - mode->height,
				 1, vblank);
	__v4l2_ctrl_s_ctrl(sc->vblank, vblank);
}

static int sc_enable_streams(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state, u32 pad,
			     u64 streams_mask)
{
	struct sc *sc = to_sc(sd);
	const struct sc_reg_list *init = &sc->chip->init;
	const struct sc_reg_list *reg_list = &sc->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(sc->dev);
	if (ret)
		return ret;

	cci_write(sc->regmap, SC_REG_SOFTWARE_RESET, 0x01, &ret);
	if (ret)
		goto error;

	usleep_range(5000, 5500);

	cci_multi_reg_write(sc->regmap, init->regs, init->num_regs, &ret);
	cci_multi_reg_write(sc->regmap, reg_list->regs, reg_list->num_regs,
			    &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(sc->sd.ctrl_handler);

	cci_write(sc->regmap, SC_REG_STREAM, 1, &ret);
	if (ret)
		goto error;

	/*
	 * Both sensors leave writes unacknowledged for a moment after stream
	 * on; stock waits 10 ms after the rear one's
	 */
	usleep_range(10000, 11000);

	return 0;

error:
	dev_err(sc->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(sc->dev);

	return ret;
}

static int sc_disable_streams(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state, u32 pad,
			      u64 streams_mask)
{
	struct sc *sc = to_sc(sd);
	int ret = 0;

	cci_write(sc->regmap, SC_REG_STREAM, 0, &ret);
	if (ret)
		dev_err(sc->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(sc->dev);

	return ret;
}

static void sc_update_pad_format(const struct sc_mode *mode,
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

static int sc_set_pad_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct sc *sc = to_sc(sd);
	const struct sc_mode *mode;

	mode = v4l2_find_nearest_size(sc->chip->modes, sc->chip->num_modes,
				      width, height, fmt->format.width,
				      fmt->format.height);

	sc_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && sc->mode != mode) {
		sc->mode = mode;
		sc_update_blanking(sc);
	}

	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int sc_enum_mbus_code(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int sc_enum_frame_size(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_frame_size_enum *fse)
{
	const struct sc_chip *chip = to_sc(sd)->chip;

	if (fse->index >= chip->num_modes)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = chip->modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = chip->modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int sc_get_selection(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_selection *sel)
{
	const struct sc_chip *chip = to_sc(sd)->chip;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = chip->native_width;
		sel->r.height = chip->native_height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int sc_init_state(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state)
{
	struct sc *sc = to_sc(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = sc->mode->width,
			.height = sc->mode->height,
		},
	};

	sc_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops sc_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops sc_pad_ops = {
	.set_fmt = sc_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = sc_get_selection,
	.enum_mbus_code = sc_enum_mbus_code,
	.enum_frame_size = sc_enum_frame_size,
	.enable_streams = sc_enable_streams,
	.disable_streams = sc_disable_streams,
};

static const struct v4l2_subdev_ops sc_subdev_ops = {
	.video = &sc_video_ops,
	.pad = &sc_pad_ops,
};

static const struct v4l2_subdev_internal_ops sc_internal_ops = {
	.init_state = sc_init_state,
};

static const struct media_entity_operations sc_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int sc_identify_sensor(struct sc *sc)
{
	u64 val;
	int ret = 0;

	cci_read(sc->regmap, SC_REG_CHIP_ID, &val, &ret);
	if (ret) {
		dev_err(sc->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != sc->chip->chip_id) {
		dev_err(sc->dev, "chip id mismatch: %x!=%llx\n",
			sc->chip->chip_id, val);
		return -ENODEV;
	}

	return 0;
}

/* Copy the OTP of a powered sensor before it streams; it never changes */
static int sc_read_otp(struct sc *sc)
{
	const struct sc_otp *otp = sc->chip->otp;
	unsigned int i, offset = 0;
	int ret = 0;

	sc->otp_data = devm_kzalloc(sc->dev, otp->size, GFP_KERNEL);
	if (!sc->otp_data)
		return -ENOMEM;

	cci_write(sc->regmap, SC_REG_OTP_VREF(0), 0x48, &ret);
	cci_write(sc->regmap, SC_REG_OTP_VREF(1), 0x38, &ret);
	cci_write(sc->regmap, SC_REG_OTP_VREF(2), 0x41, &ret);
	if (ret)
		return ret;

	for (i = 0; i < otp->num_pages; i++) {
		const struct sc_otp_page *page = &otp->pages[i];
		u16 buf = SC_OTP_BUF(page->page);

		cci_write(sc->regmap, SC_REG_OTP_START, buf, &ret);
		cci_write(sc->regmap, SC_REG_OTP_END, buf | 0x1ff, &ret);
		cci_write(sc->regmap, SC_REG_OTP_MODE, SC_OTP_MODE_READ, &ret);
		cci_write(sc->regmap, SC_REG_OTP_PAGE, 2 * page->page + 1, &ret);
		cci_write(sc->regmap, SC_REG_OTP_STATUS, 0, &ret);
		cci_write(sc->regmap, SC_REG_OTP_CTRL, SC_OTP_CTRL_LOAD, &ret);
		if (ret)
			return ret;

		fsleep(SC_OTP_LOAD_US);

		ret = regmap_bulk_read(sc->regmap, buf + SC_OTP_DATA_OFFSET,
				       sc->otp_data + offset, page->size);
		if (ret)
			return ret;

		offset += page->size;
	}

	return 0;
}

static int sc_otp_read(void *priv, unsigned int off, void *val, size_t count)
{
	struct sc *sc = priv;

	memcpy(val, sc->otp_data + off, count);

	return 0;
}

static int sc_register_otp(struct sc *sc)
{
	struct nvmem_config config = {
		.name = dev_name(sc->dev),
		.id = NVMEM_DEVID_NONE,
		.dev = sc->dev,
		.owner = THIS_MODULE,
		.read_only = true,
		.root_only = true,
		.reg_read = sc_otp_read,
		.size = sc->chip->otp->size,
		.word_size = 1,
		.stride = 1,
		.priv = sc,
	};
	int ret;

	ret = sc_read_otp(sc);
	if (ret)
		return ret;

	return PTR_ERR_OR_ZERO(devm_nvmem_register(sc->dev, &config));
}

static int sc_check_hwcfg(struct sc *sc)
{
	struct fwnode_handle *fwnode = dev_fwnode(sc->dev), *ep;
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != SC_DATA_LANES) {
		dev_err(sc->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(sc->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       &sc->chip->link_freq, 1, &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/*
 * Reset held; DOVDD, AVDD and DVDD 1 ms apart; MCLK; reset released 1 ms
 * later; 10 ms before the first access
 */
static int sc_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc *sc = to_sc(sd);
	int ret;

	gpiod_set_value_cansleep(sc->reset_gpio, 1);

	ret = regulator_enable(sc->supplies[SC_SUPPLY_DOVDD].consumer);
	if (ret)
		return ret;

	usleep_range(1000, 1500);

	ret = regulator_enable(sc->supplies[SC_SUPPLY_AVDD].consumer);
	if (ret)
		goto disable_dovdd;

	usleep_range(1000, 1500);

	ret = regulator_enable(sc->supplies[SC_SUPPLY_DVDD].consumer);
	if (ret)
		goto disable_avdd;

	usleep_range(1000, 1500);

	ret = clk_prepare_enable(sc->mclk);
	if (ret)
		goto disable_dvdd;

	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(sc->reset_gpio, 0);
	usleep_range(10000, 10500);

	return 0;

disable_dvdd:
	regulator_disable(sc->supplies[SC_SUPPLY_DVDD].consumer);
disable_avdd:
	regulator_disable(sc->supplies[SC_SUPPLY_AVDD].consumer);
disable_dovdd:
	regulator_disable(sc->supplies[SC_SUPPLY_DOVDD].consumer);

	return ret;
}

static int sc_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc *sc = to_sc(sd);

	gpiod_set_value_cansleep(sc->reset_gpio, 1);
	usleep_range(1000, 1500);
	clk_disable_unprepare(sc->mclk);
	regulator_disable(sc->supplies[SC_SUPPLY_DVDD].consumer);
	regulator_disable(sc->supplies[SC_SUPPLY_AVDD].consumer);
	regulator_disable(sc->supplies[SC_SUPPLY_DOVDD].consumer);

	return 0;
}

static int sc_probe(struct i2c_client *client)
{
	struct sc *sc;
	unsigned long freq;
	unsigned int i;
	int ret;

	sc = devm_kzalloc(&client->dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = &client->dev;
	sc->chip = device_get_match_data(sc->dev);
	v4l2_i2c_subdev_init(&sc->sd, client, &sc_subdev_ops);

	sc->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sc->regmap))
		return dev_err_probe(sc->dev, PTR_ERR(sc->regmap),
				     "failed to init CCI\n");

	sc->mclk = devm_v4l2_sensor_clk_get(sc->dev, NULL);
	if (IS_ERR(sc->mclk))
		return dev_err_probe(sc->dev, PTR_ERR(sc->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(sc->mclk);
	if (freq != SC_MCLK_FREQ_19_2MHZ)
		return dev_err_probe(sc->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = sc_check_hwcfg(sc);
	if (ret)
		return dev_err_probe(sc->dev, ret,
				     "failed to check HW configuration\n");

	sc->reset_gpio = devm_gpiod_get_optional(sc->dev, "reset",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(sc->reset_gpio))
		return dev_err_probe(sc->dev, PTR_ERR(sc->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < SC_NUM_SUPPLIES; i++)
		sc->supplies[i].supply = sc_supply_names[i];

	ret = devm_regulator_bulk_get(sc->dev, SC_NUM_SUPPLIES, sc->supplies);
	if (ret)
		return dev_err_probe(sc->dev, ret,
				     "failed to get supply regulators\n");

	ret = sc_power_on(sc->dev);
	if (ret)
		return ret;

	ret = sc_identify_sensor(sc);
	if (ret) {
		dev_err_probe(sc->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	if (sc->chip->otp) {
		ret = sc_register_otp(sc);
		if (ret)
			dev_warn(sc->dev, "OTP not available: %d\n", ret);
	}

	sc->mode = &sc->chip->modes[0];
	ret = sc_init_controls(sc);
	if (ret) {
		dev_err_probe(sc->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	sc->sd.state_lock = sc->ctrl_handler.lock;
	sc->sd.internal_ops = &sc_internal_ops;
	sc->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sc->sd.entity.ops = &sc_subdev_entity_ops;
	sc->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sc->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sc->sd.entity, 1, &sc->pad);
	if (ret) {
		dev_err_probe(sc->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&sc->sd);
	if (ret < 0) {
		dev_err_probe(sc->dev, ret, "failed to init subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(sc->dev);
	pm_runtime_enable(sc->dev);

	ret = v4l2_async_register_subdev_sensor(&sc->sd);
	if (ret < 0) {
		dev_err_probe(sc->dev, ret, "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(sc->dev, 1000);
	pm_runtime_use_autosuspend(sc->dev);
	pm_runtime_idle(sc->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&sc->sd);
	pm_runtime_disable(sc->dev);
	pm_runtime_set_suspended(sc->dev);

media_entity_cleanup:
	media_entity_cleanup(&sc->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(sc->sd.ctrl_handler);

power_off:
	sc_power_off(sc->dev);

	return ret;
}

static void sc_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc *sc = to_sc(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(sc->dev);

	if (!pm_runtime_status_suspended(sc->dev)) {
		sc_power_off(sc->dev);
		pm_runtime_set_suspended(sc->dev);
	}
}

static const struct dev_pm_ops sc_pm_ops = {
	SET_RUNTIME_PM_OPS(sc_power_off, sc_power_on, NULL)
};

static const struct of_device_id sc_of_match[] = {
	{ .compatible = "smartsens,c658", .data = &sc_c658 },
	{ .compatible = "smartsens,d154", .data = &sc_d154 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sc_of_match);

static struct i2c_driver sc_i2c_driver = {
	.driver = {
		.name = "smartsens-cs",
		.pm = &sc_pm_ops,
		.of_match_table = sc_of_match,
	},
	.probe = sc_probe,
	.remove = sc_remove,
};

module_i2c_driver(sc_i2c_driver);

MODULE_DESCRIPTION("SmartSens CS-series image sensor driver");
MODULE_LICENSE("GPL");
