// SPDX-License-Identifier: GPL-2.0
/*
 * Cirrus Logic CS40L25A boosted haptic driver
 *
 * The chip drives a linear resonant actuator from its boost converter.
 * Its Halo Core DSP runs firmware the host loads: a wavetable of effects
 * and a timed vibration, triggered through mailbox registers.
 *
 * Register map, start-up sequence and firmware interface as in the
 * cs40l2x driver of Google's amplifiers kernel modules (Cirrus Logic,
 * GPL-2.0).
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/firmware/cirrus/cs_dsp.h>
#include <linux/firmware/cirrus/wmfw.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/int_log.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define CS40L25_DEVID			0x00000000
#define CS40L25_REVID			0x00000004
#define CS40L25_TEST_KEY_CTL		0x00000040
#define CS40L25_PROTECT_REL_ERR_IGN	0x00002034
#define CS40L25_PWR_CTRL1		0x00002014
#define CS40L25_PWR_CTRL2		0x00002018
#define CS40L25_PWR_CTRL3		0x0000201C
#define CS40L25_LRCK_PAD_CONTROL	0x00002418
#define CS40L25_SDIN_PAD_CONTROL	0x00002420
#define CS40L25_PWRMGT_CTL		0x00002900
#define CS40L25_PLL_LOOP_PARAM		0x00003008
#define CS40L25_PLL_MISC_CTRL		0x00003014
#define CS40L25_BSTCVRT_VCTRL1		0x00003800
#define CS40L25_BSTCVRT_VCTRL2		0x00003804
#define CS40L25_BSTCVRT_PEAK_CUR	0x00003808
#define CS40L25_BSTCVRT_COEFF		0x00003810
#define CS40L25_BSTCVRT_SLOPE_LBST	0x00003814
#define CS40L25_BSTCVRT_DCM_CTRL	0x0000381C
#define CS40L25_BSTCVRT_OVERVOLT_CTRL	0x00003830
#define CS40L25_TEST_LBST		0x0000391C
#define CS40L25_SPKMON_RESYNC		0x00004100
#define CS40L25_OTP_TRIM_12		0x00004170
#define CS40L25_TEMP_RESYNC		0x00004310
#define CS40L25_OTP_TRIM_13		0x00004360
#define CS40L25_VPVBST_FS_SEL		0x00004400
#define CS40L25_DAC_PCM1_SRC		0x00004C00
#define CS40L25_DSP1RX1_INPUT		0x00004C40
#define CS40L25_DSP1RX2_INPUT		0x00004C44
#define CS40L25_DSP1RX3_INPUT		0x00004C48
#define CS40L25_DSP1RX4_INPUT		0x00004C4C
#define CS40L25_AMP_DIG_VOL_CTRL	0x00006000
#define CS40L25_IRQ1_STATUS3		0x00010018
#define CS40L25_IRQ1_STATUS4		0x0001001C
#define CS40L25_IRQ2_STATUS1		0x00010810
#define CS40L25_IRQ2_MASK1		0x00010910
#define CS40L25_IRQ2_MASK2		0x00010914
#define CS40L25_IRQ2_MASK3		0x00010918
#define CS40L25_IRQ2_MASK4		0x0001091C
#define CS40L25_MBOX_TRIGGERINDEX	0x00013020
#define CS40L25_MBOX_TRIGGER_MS		0x00013024
#define CS40L25_MBOX_POWERCONTROL	0x0001302C
#define CS40L25_DSP1_XMEM_PACKED_0	0x02000000
#define CS40L25_DSP1_XMEM_UNPACKED24_0	0x02800000
#define CS40L25_DSP1_CORE_BASE		0x02B80000
#define CS40L25_DSP1_RX1_RATE		0x02B80080
#define CS40L25_DSP1_TX1_RATE		0x02B80280
#define CS40L25_DSP1_SCRATCH1		0x02B805C0
#define CS40L25_DSP1_XM_ACCEL_PL0_PRI	0x02BC2020
#define CS40L25_DSP1_YM_ACCEL_PL0_PRI	0x02BC20E0
#define CS40L25_DSP1_YMEM_PACKED_0	0x02C00000
#define CS40L25_DSP1_YMEM_UNPACKED24_0	0x03400000
#define CS40L25_DSP1_PMEM_0		0x03800000
#define CS40L25_LAST_REG		0x03804FE8

/* Basic mode: the ROM's own haptics, which run until shut down */
#define CS40L25_BASIC_HALO_HEARTBEAT	0x02800158
#define CS40L25_BASIC_STATEMACHINE	0x0280015C
#define CS40L25_BASIC_AMP_STATUS	0x0280018C
#define CS40L25_BASIC_SHUTDOWNREQUEST	0x02800190
#define CS40L25_BASIC_SHUTDOWN		4
#define CS40L25_BASIC_BOOT_DONE		BIT(0)
#define CS40L25_BASIC_OTP_ERROR		BIT(1)
#define CS40L25_BASIC_AMP_ERROR		BIT(2)
#define CS40L25_BASIC_TEMP_RISE_WARN	BIT(3)
#define CS40L25_BASIC_TEMP_ERROR	BIT(4)

#define CS40L25_DEVID_A			0x40A25A
#define CS40L25_REVID_B1		0xB1

#define CS40L25_OTP_BOOT_DONE		BIT(1)
#define CS40L25_OTP_BOOT_ERR		BIT(31)

#define CS40L25_TEST_KEY_UNLOCK_CODE1	0x55
#define CS40L25_TEST_KEY_UNLOCK_CODE2	0xAA
#define CS40L25_TEST_KEY_RELOCK_CODE1	0xCC
#define CS40L25_TEST_KEY_RELOCK_CODE2	0x33

#define CS40L25_BST_EN_MASK		GENMASK(5, 4)
#define CS40L25_BST_DISABLED		0
#define CS40L25_BST_ENABLED		2
#define CS40L25_CLASSH_EN_MASK		BIT(4)
#define CS40L25_MEM_RDY			BIT(1)
#define CS40L25_BST_CTL_MASK		GENMASK(7, 0)
#define CS40L25_BST_CTL_LIM_EN		BIT(2)
#define CS40L25_BST_CTL_SEL_MASK	GENMASK(1, 0)
#define CS40L25_BST_IPK_MASK		GENMASK(6, 0)
#define CS40L25_BST_K1_MASK		GENMASK(7, 0)
#define CS40L25_BST_K2_MASK		GENMASK(15, 8)
#define CS40L25_BST_SLOPE_MASK		GENMASK(15, 8)
#define CS40L25_BST_LBST_VAL_MASK	GENMASK(1, 0)
#define CS40L25_BST_OVP_EN		BIT(8)
#define CS40L25_BST_OVP_THLD_MASK	GENMASK(5, 0)
#define CS40L25_DCM_DISABLE		0

#define CS40L25_AMP_ERR			BIT(31)
#define CS40L25_TEMP_ERR		BIT(17)
#define CS40L25_TEMP_RISE_WARN		BIT(15)
#define CS40L25_BST_SHORT_ERR		BIT(8)
#define CS40L25_BST_UVP_ERR		BIT(7)
#define CS40L25_BST_OVP_ERR		BIT(6)
#define CS40L25_HW_ERRS			(CS40L25_AMP_ERR | CS40L25_TEMP_ERR | \
					 CS40L25_TEMP_RISE_WARN | \
					 CS40L25_BST_SHORT_ERR | \
					 CS40L25_BST_UVP_ERR | CS40L25_BST_OVP_ERR)
#define CS40L25_HW_ERR_COUNT_MAX	10

/* Firmware */
#define CS40L25_ALG_ID_VIBEGEN		0x0000BD
#define CS40L25_ALG_ID_CLAB		0x000117
#define CS40L25_HALO_STATE_RUN		203
#define CS40L25_MBOX_RESET		0xFFFFFFFF
#define CS40L25_POWERCONTROL_WAKEUP	2
#define CS40L25_EVENT_NONE		0xFFFFFF
#define CS40L25_EVENT_HARDWARE		13
#define CS40L25_EVENT_HARDWARE_ENABLED	BIT(23)
#define CS40L25_IRQMASKSEQ_LEN		8
#define CS40L25_TIMEOUT_MS_MAX		0x02AAAA
#define CS40L25_GAIN_TRIG_MASK		GENMASK(13, 4)
#define CS40L25_ATTEN_MAX		816	/* 0.125 dB steps: -102 dB */
#define CS40L25_INDEX_VIBE		0
#define CS40L25_INDEX_CLICK_MAX		0x7FFF

#define CS40L25_TIMEOUT_COUNT		10
#define CS40L25_EFFECTS_MAX		16

#define CS40L25_DEFAULT_WMFW		"cs40l25a.wmfw"
#define CS40L25_WAVETABLE		"cs40l20.bin"

enum cs40l25_ctl {
	CS40L25_HALO_STATE,
	CS40L25_EVENTCONTROL,
	CS40L25_HARDWAREEVENT,
	CS40L25_ENDPLAYBACK,
	CS40L25_GPIO_BUTTONDETECT,
	CS40L25_GAIN_CONTROL,
	CS40L25_IRQMASKSEQUENCE,
	CS40L25_IRQMASKSEQUENCE_VALID,
	CS40L25_SPK_FORCE_TST_1_AUTO,
	CS40L25_VMON_IMON_OFFSET_ENABLE,
	CS40L25_TIMEOUT_MS,
	CS40L25_NUMBEROFWAVES,
	CS40L25_CLAB_ENABLED,
	CS40L25_NUM_CTLS,
};

/* The algorithm 0 is the firmware's own, whose ID depends on the build */
static const struct {
	const char *name;
	unsigned int alg;
	bool optional;
} cs40l25_ctls[CS40L25_NUM_CTLS] = {
	[CS40L25_HALO_STATE]		  = { "HALO_STATE" },
	[CS40L25_EVENTCONTROL]		  = { "EVENTCONTROL" },
	[CS40L25_HARDWAREEVENT]		  = { "HARDWAREEVENT" },
	[CS40L25_ENDPLAYBACK]		  = { "ENDPLAYBACK" },
	[CS40L25_GPIO_BUTTONDETECT]	  = { "GPIO_BUTTONDETECT" },
	[CS40L25_GAIN_CONTROL]		  = { "GAIN_CONTROL" },
	[CS40L25_IRQMASKSEQUENCE]	  = { "IRQMASKSEQUENCE" },
	[CS40L25_IRQMASKSEQUENCE_VALID]	  = { "IRQMASKSEQUENCE_VALID" },
	[CS40L25_SPK_FORCE_TST_1_AUTO]	  = { "SPK_FORCE_TST_1_AUTO", 0, true },
	[CS40L25_VMON_IMON_OFFSET_ENABLE] = { "VMON_IMON_OFFSET_ENABLE", 0, true },
	[CS40L25_TIMEOUT_MS]	  = { "TIMEOUT_MS", CS40L25_ALG_ID_VIBEGEN },
	[CS40L25_NUMBEROFWAVES]	  = { "NUMBEROFWAVES", CS40L25_ALG_ID_VIBEGEN },
	[CS40L25_CLAB_ENABLED]	  = { "CLAB_ENABLED", CS40L25_ALG_ID_CLAB, true },
};

static const struct {
	u32 irq;
	u32 release;
	bool boost;
	const char *name;
} cs40l25_hw_errs[] = {
	{ CS40L25_AMP_ERR,	  BIT(1), false, "amplifier short" },
	{ CS40L25_TEMP_ERR,	  BIT(6), false, "overtemperature" },
	{ CS40L25_TEMP_RISE_WARN, BIT(5), false, "temperature rise" },
	{ CS40L25_BST_SHORT_ERR,  BIT(2), true,  "boost short" },
	{ CS40L25_BST_UVP_ERR,	  BIT(4), true,  "boost undervoltage" },
	{ CS40L25_BST_OVP_ERR,	  BIT(3), true,  "boost overvoltage" },
};

struct cs40l25_effect {
	u32 index;
	bool timed;
	u16 magnitude;
	u16 length_ms;
};

struct cs40l25 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct cs_dsp dsp;
	/* DSP word addresses of the controls, 0 when absent */
	unsigned int ctl[CS40L25_NUM_CTLS];
	const char *wmfw_name;
	unsigned int num_waves;
	unsigned int hw_err_count[ARRAY_SIZE(cs40l25_hw_errs)];

	struct input_dev *input;
	struct workqueue_struct *wq;
	struct work_struct start_work;
	struct work_struct stop_work;
	struct delayed_work timed_stop_work;
	struct cs40l25_effect effects[CS40L25_EFFECTS_MAX];
	/* Written under the input event lock, read by the start work */
	int play_id;
	u16 gain;
};

static const struct regmap_config cs40l25_regmap = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = CS40L25_LAST_REG,
	.cache_type = REGCACHE_NONE,
};

static const struct cs_dsp_region cs40l25_dsp_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED,	.base = CS40L25_DSP1_PMEM_0 },
	{ .type = WMFW_HALO_XM_PACKED,	.base = CS40L25_DSP1_XMEM_PACKED_0 },
	{ .type = WMFW_HALO_YM_PACKED,	.base = CS40L25_DSP1_YMEM_PACKED_0 },
	{ .type = WMFW_ADSP2_XM,	.base = CS40L25_DSP1_XMEM_UNPACKED24_0 },
	{ .type = WMFW_ADSP2_YM,	.base = CS40L25_DSP1_YMEM_UNPACKED24_0 },
};

static const char * const cs40l25_supplies[] = { "va", "vp" };

/* Fields that basic mode changes, back to their reset values */
static const struct reg_sequence cs40l25_basic_mode_revert[] = {
	{ CS40L25_PWR_CTRL1,		0x00000000 },
	{ CS40L25_PWR_CTRL2,		0x00003321 },
	{ CS40L25_LRCK_PAD_CONTROL,	0x00000007 },
	{ CS40L25_SDIN_PAD_CONTROL,	0x00000007 },
	{ CS40L25_AMP_DIG_VOL_CTRL,	0x00008000 },
	{ CS40L25_IRQ2_MASK1,		0xFFFFFFFF },
	{ CS40L25_IRQ2_MASK2,		0xFFFFFFFF },
};

static const struct reg_sequence cs40l25_errata[] = {
	{ CS40L25_PLL_LOOP_PARAM,	0x000C1837 },
	{ CS40L25_PLL_MISC_CTRL,	0x03008E0E },
	{ CS40L25_TEST_KEY_CTL,		CS40L25_TEST_KEY_UNLOCK_CODE1 },
	{ CS40L25_TEST_KEY_CTL,		CS40L25_TEST_KEY_UNLOCK_CODE2 },
	{ CS40L25_TEST_LBST,		0x004DC080 },
	{ CS40L25_OTP_TRIM_12,		0x002F0065 },
	{ CS40L25_OTP_TRIM_13,		0x00002B4F },
	{ CS40L25_SPKMON_RESYNC,	0x00000000 },
	{ CS40L25_TEMP_RESYNC,		0x00000000 },
	{ CS40L25_TEST_KEY_CTL,		CS40L25_TEST_KEY_RELOCK_CODE1 },
	{ CS40L25_TEST_KEY_CTL,		CS40L25_TEST_KEY_RELOCK_CODE2 },
	{ CS40L25_VPVBST_FS_SEL,	0x00000000 },
};

/* The DSP drives the DAC; its inputs are ASP RX1 and the monitors */
static const struct reg_sequence cs40l25_pcm_routing[] = {
	{ CS40L25_DAC_PCM1_SRC,		0x32 },	/* DSP1 TX1 */
	{ CS40L25_DSP1RX1_INPUT,	0x08 },	/* ASP RX1 */
	{ CS40L25_DSP1RX2_INPUT,	0x18 },	/* VMON */
	{ CS40L25_DSP1RX3_INPUT,	0x19 },	/* IMON */
	{ CS40L25_DSP1RX4_INPUT,	0x28 },	/* VPMON */
};

static const unsigned int cs40l25_irq2_masks[] = {
	0xFFFFFFFF & ~CS40L25_HW_ERRS, 0xFFFFFFFF, 0xFFFF87FF, 0xFEFFFFFF,
};

/* Boost loop coefficients by inductor (rows) and capacitor range */
static const u8 cs40l25_bst_k1[4][5] = {
	{ 0x24, 0x32, 0x32, 0x4F, 0x57 },
	{ 0x24, 0x32, 0x32, 0x4F, 0x57 },
	{ 0x40, 0x32, 0x32, 0x4F, 0x57 },
	{ 0x40, 0x32, 0x32, 0x4F, 0x57 },
};

static const u8 cs40l25_bst_k2[4][5] = {
	{ 0x24, 0x49, 0x66, 0xA3, 0xEA },
	{ 0x24, 0x49, 0x66, 0xA3, 0xEA },
	{ 0x48, 0x49, 0x66, 0xA3, 0xEA },
	{ 0x48, 0x49, 0x66, 0xA3, 0xEA },
};

static const u8 cs40l25_bst_slope[4] = { 0x75, 0x6B, 0x3B, 0x28 };

static int cs40l25_ack_write(struct cs40l25 *cs, unsigned int reg,
			     unsigned int val, unsigned int done)
{
	unsigned int ack;
	int ret;

	ret = regmap_write(cs->regmap, reg, val);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(cs->regmap, reg, ack, ack == done,
				       1000, 1000 * CS40L25_TIMEOUT_COUNT);
	if (ret)
		dev_err(cs->dev, "No acknowledgment of %#x at %#x\n", val, reg);

	return ret;
}

/* Callers hold the DSP's pwr_lock */
static int cs40l25_ctl_read(struct cs40l25 *cs, enum cs40l25_ctl id, u32 *val)
{
	if (!cs->ctl[id])
		return -ENOENT;

	return cs_dsp_read_data_word(&cs->dsp, WMFW_ADSP2_XM, cs->ctl[id], val);
}

static int cs40l25_ctl_write(struct cs40l25 *cs, enum cs40l25_ctl id, u32 val)
{
	if (!cs->ctl[id])
		return -ENOENT;

	return cs_dsp_write_data_word(&cs->dsp, WMFW_ADSP2_XM, cs->ctl[id], val);
}

static int cs40l25_hw_err_release(struct cs40l25 *cs, unsigned int i)
{
	struct regmap *regmap = cs->regmap;
	int ret;

	if (cs40l25_hw_errs[i].boost) {
		ret = regmap_update_bits(regmap, CS40L25_PWR_CTRL2,
					 CS40L25_BST_EN_MASK,
					 FIELD_PREP(CS40L25_BST_EN_MASK,
						    CS40L25_BST_DISABLED));
		if (ret)
			return ret;
	}

	ret = regmap_write(regmap, CS40L25_PROTECT_REL_ERR_IGN, 0);
	if (ret)
		return ret;

	ret = regmap_write(regmap, CS40L25_PROTECT_REL_ERR_IGN,
			   cs40l25_hw_errs[i].release);
	if (ret)
		return ret;

	ret = regmap_write(regmap, CS40L25_PROTECT_REL_ERR_IGN, 0);
	if (ret)
		return ret;

	if (cs40l25_hw_errs[i].boost)
		ret = regmap_update_bits(regmap, CS40L25_PWR_CTRL2,
					 CS40L25_BST_EN_MASK,
					 FIELD_PREP(CS40L25_BST_EN_MASK,
						    CS40L25_BST_ENABLED));

	return ret;
}

static int cs40l25_hw_err_handle(struct cs40l25 *cs)
{
	unsigned int status;
	int i, ret;

	ret = regmap_read(cs->regmap, CS40L25_IRQ2_STATUS1, &status);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(cs40l25_hw_errs); i++) {
		if (!(status & cs40l25_hw_errs[i].irq))
			continue;

		ret = regmap_write(cs->regmap, CS40L25_IRQ2_STATUS1,
				   cs40l25_hw_errs[i].irq);
		if (ret)
			return ret;

		if (cs->hw_err_count[i]++ >= CS40L25_HW_ERR_COUNT_MAX) {
			dev_crit_ratelimited(cs->dev, "%s error, not released\n",
					     cs40l25_hw_errs[i].name);
			continue;
		}

		dev_crit(cs->dev, "%s error\n", cs40l25_hw_errs[i].name);

		ret = cs40l25_hw_err_release(cs, i);
		if (ret)
			return ret;
	}

	return 0;
}

static irqreturn_t cs40l25_irq(int irq, void *data)
{
	struct cs40l25 *cs = data;
	irqreturn_t handled = IRQ_NONE;
	unsigned int scratch;
	u32 event;
	int ret;

	mutex_lock(&cs->dsp.pwr_lock);

	ret = regmap_read(cs->regmap, CS40L25_DSP1_SCRATCH1, &scratch);
	if (ret)
		goto out;

	if (scratch) {
		dev_err(cs->dev, "DSP fault, scratch %#x\n", scratch);
		handled = IRQ_HANDLED;
		goto out;
	}

	ret = cs40l25_ctl_read(cs, CS40L25_HARDWAREEVENT, &event);
	if (ret || event == CS40L25_EVENT_NONE)
		goto out;

	if (event == CS40L25_EVENT_HARDWARE) {
		ret = cs40l25_hw_err_handle(cs);
		if (ret)
			dev_err(cs->dev, "Failed to handle errors: %d\n", ret);
	}

	ret = cs40l25_ctl_write(cs, CS40L25_HARDWAREEVENT, CS40L25_EVENT_NONE);
	if (ret)
		goto out;

	/* Releases the interrupt line; not acknowledged */
	regmap_write(cs->regmap, CS40L25_MBOX_POWERCONTROL,
		     CS40L25_POWERCONTROL_WAKEUP);
	handled = IRQ_HANDLED;
out:
	mutex_unlock(&cs->dsp.pwr_lock);

	return handled;
}

/* Attenuation of a magnitude from full scale, in 0.125 dB steps */
static unsigned int cs40l25_attenuation(unsigned int magnitude)
{
	u64 att;

	if (magnitude >= U16_MAX)
		return 0;
	if (!magnitude)
		return CS40L25_ATTEN_MAX;

	att = (u64)(intlog10(U16_MAX) - intlog10(magnitude)) * 160;

	return min_t(u64, att >> 24, CS40L25_ATTEN_MAX);
}

static void cs40l25_start_worker(struct work_struct *work)
{
	struct cs40l25 *cs = container_of(work, struct cs40l25, start_work);
	struct cs40l25_effect *effect;
	unsigned int magnitude;
	u32 gain;
	int id, ret;

	id = READ_ONCE(cs->play_id);
	effect = &cs->effects[id];
	magnitude = effect->magnitude * READ_ONCE(cs->gain) / U16_MAX;

	cancel_delayed_work(&cs->timed_stop_work);

	mutex_lock(&cs->dsp.pwr_lock);

	ret = cs40l25_ctl_read(cs, CS40L25_GAIN_CONTROL, &gain);
	if (ret)
		goto err;

	gain &= ~CS40L25_GAIN_TRIG_MASK;
	gain |= FIELD_PREP(CS40L25_GAIN_TRIG_MASK,
			   cs40l25_attenuation(magnitude));
	ret = cs40l25_ctl_write(cs, CS40L25_GAIN_CONTROL, gain);
	if (ret)
		goto err;

	if (effect->timed)
		ret = cs40l25_ack_write(cs, CS40L25_MBOX_TRIGGER_MS,
					effect->index, CS40L25_MBOX_RESET);
	else
		ret = cs40l25_ack_write(cs, CS40L25_MBOX_TRIGGERINDEX,
					effect->index, CS40L25_MBOX_RESET);
err:
	mutex_unlock(&cs->dsp.pwr_lock);

	if (ret) {
		dev_err(cs->dev, "Failed to play effect %d: %d\n", id, ret);
		return;
	}

	if (effect->timed && effect->length_ms)
		queue_delayed_work(cs->wq, &cs->timed_stop_work,
				   msecs_to_jiffies(effect->length_ms));
}

static void cs40l25_stop(struct cs40l25 *cs)
{
	int i, ret;

	mutex_lock(&cs->dsp.pwr_lock);

	/* The firmware NAKs while it is busy: try again */
	for (i = 0; i < CS40L25_TIMEOUT_COUNT; i++) {
		ret = cs40l25_ctl_write(cs, CS40L25_ENDPLAYBACK, 1);
		if (!ret)
			break;

		usleep_range(10000, 11000);
	}

	mutex_unlock(&cs->dsp.pwr_lock);

	if (ret)
		dev_err(cs->dev, "Failed to stop playback: %d\n", ret);
}

static void cs40l25_stop_worker(struct work_struct *work)
{
	struct cs40l25 *cs = container_of(work, struct cs40l25, stop_work);

	cancel_delayed_work(&cs->timed_stop_work);
	cs40l25_stop(cs);
}

static void cs40l25_timed_stop_worker(struct work_struct *work)
{
	struct cs40l25 *cs = container_of(to_delayed_work(work), struct cs40l25,
					  timed_stop_work);

	cs40l25_stop(cs);
}

static int cs40l25_upload(struct input_dev *dev, struct ff_effect *ff,
			  struct ff_effect *old)
{
	struct cs40l25 *cs = input_get_drvdata(dev);
	struct cs40l25_effect *effect = &cs->effects[ff->id];
	s16 index;

	/* A timed vibration runs this long; 0 runs until stopped */
	effect->length_ms = ff->replay.length;

	switch (ff->type) {
	case FF_RUMBLE:
		effect->timed = true;
		effect->index = CS40L25_INDEX_VIBE;
		effect->magnitude = max(ff->u.rumble.strong_magnitude,
					ff->u.rumble.weak_magnitude);
		return 0;
	case FF_PERIODIC:
		break;
	default:
		return -EINVAL;
	}

	effect->magnitude = min_t(unsigned int, U16_MAX,
				  2 * abs(ff->u.periodic.magnitude));

	if (ff->u.periodic.waveform != FF_CUSTOM) {
		effect->timed = true;
		effect->index = CS40L25_INDEX_VIBE;
		return 0;
	}

	/* A custom effect names a wavetable entry */
	if (ff->u.periodic.custom_len != 1)
		return -EINVAL;

	if (copy_from_user(&index, ff->u.periodic.custom_data, sizeof(index)))
		return -EFAULT;

	if (index < 1 || index >= cs->num_waves || index > CS40L25_INDEX_CLICK_MAX)
		return -EINVAL;

	effect->timed = false;
	effect->index = index;

	return 0;
}

static int cs40l25_playback(struct input_dev *dev, int effect_id, int val)
{
	struct cs40l25 *cs = input_get_drvdata(dev);

	/* Under the input event lock: the work items talk to the chip */
	if (val > 0) {
		WRITE_ONCE(cs->play_id, effect_id);
		queue_work(cs->wq, &cs->start_work);
	} else {
		queue_work(cs->wq, &cs->stop_work);
	}

	return 0;
}

static void cs40l25_set_gain(struct input_dev *dev, u16 gain)
{
	struct cs40l25 *cs = input_get_drvdata(dev);

	WRITE_ONCE(cs->gain, gain);
}

static int cs40l25_ctls_find(struct cs40l25 *cs)
{
	struct cs_dsp_coeff_ctl *ctl;
	unsigned int alg;
	int i;

	for (i = 0; i < CS40L25_NUM_CTLS; i++) {
		alg = cs40l25_ctls[i].alg ?: cs->dsp.fw_id;
		ctl = cs_dsp_get_ctl(&cs->dsp, cs40l25_ctls[i].name,
				     WMFW_ADSP2_XM, alg);
		if (!ctl) {
			if (cs40l25_ctls[i].optional)
				continue;

			dev_err(cs->dev, "Firmware has no %s\n",
				cs40l25_ctls[i].name);
			return -EINVAL;
		}

		cs->ctl[i] = ctl->alg_region.base + ctl->offset;
	}

	return 0;
}

static int cs40l25_dsp_pre_run(struct cs_dsp *dsp)
{
	struct cs40l25 *cs = container_of(dsp, struct cs40l25, dsp);
	struct regmap *regmap = cs->regmap;
	unsigned int i, val;
	int ret;

	ret = cs40l25_ctls_find(cs);
	if (ret)
		return ret;

	/* Streams at the base rate; no accelerator priorities */
	for (i = 0; i < 8; i++) {
		ret = regmap_write(regmap, CS40L25_DSP1_RX1_RATE + i * 8, 1);
		if (ret)
			return ret;

		ret = regmap_write(regmap, CS40L25_DSP1_TX1_RATE + i * 8, 1);
		if (ret)
			return ret;
	}

	ret = regmap_write(regmap, CS40L25_DSP1_XM_ACCEL_PL0_PRI, 0);
	if (ret)
		return ret;

	ret = regmap_write(regmap, CS40L25_DSP1_YM_ACCEL_PL0_PRI, 0);
	if (ret)
		return ret;

	ret = regmap_set_bits(regmap, CS40L25_PWRMGT_CTL, CS40L25_MEM_RDY);
	if (ret)
		return ret;

	/* No GPIO triggers; the output floats in standby */
	ret = cs40l25_ctl_write(cs, CS40L25_GPIO_BUTTONDETECT, 0);
	if (ret)
		return ret;

	if (cs->ctl[CS40L25_SPK_FORCE_TST_1_AUTO]) {
		ret = cs40l25_ctl_write(cs, CS40L25_SPK_FORCE_TST_1_AUTO, 0);
		if (ret)
			return ret;
	}

	ret = cs40l25_ctl_write(cs, CS40L25_EVENTCONTROL,
				CS40L25_EVENT_HARDWARE_ENABLED);
	if (ret)
		return ret;

	/*
	 * The firmware restores the interrupt masks from this sequence; each
	 * mask register takes two 24-bit words.
	 */
	for (i = 0; i < ARRAY_SIZE(cs40l25_irq2_masks); i++) {
		val = cs40l25_irq2_masks[i];

		ret = cs_dsp_write_data_word(dsp, WMFW_ADSP2_XM,
					     cs->ctl[CS40L25_IRQMASKSEQUENCE] + 2 * i,
					     (val & 0xFF) << 16);
		if (ret)
			return ret;

		ret = cs_dsp_write_data_word(dsp, WMFW_ADSP2_XM,
					     cs->ctl[CS40L25_IRQMASKSEQUENCE] + 2 * i + 1,
					     val >> 8);
		if (ret)
			return ret;
	}

	return cs40l25_ctl_write(cs, CS40L25_IRQMASKSEQUENCE_VALID, 1);
}

static int cs40l25_dsp_post_run(struct cs_dsp *dsp)
{
	struct cs40l25 *cs = container_of(dsp, struct cs40l25, dsp);
	unsigned int scratch;
	u32 state, clab = 0;
	int i, ret;

	for (i = 0; i < CS40L25_TIMEOUT_COUNT; i++) {
		usleep_range(10000, 11000);

		ret = cs40l25_ctl_read(cs, CS40L25_HALO_STATE, &state);
		if (ret)
			return ret;

		if (state == CS40L25_HALO_STATE_RUN)
			break;
	}

	ret = regmap_read(cs->regmap, CS40L25_DSP1_SCRATCH1, &scratch);
	if (ret)
		return ret;

	if (state != CS40L25_HALO_STATE_RUN || scratch) {
		dev_err(cs->dev, "Firmware did not start: state %u, scratch %#x\n",
			state, scratch);
		return -ETIMEDOUT;
	}

	ret = cs40l25_ctl_write(cs, CS40L25_TIMEOUT_MS, CS40L25_TIMEOUT_MS_MAX);
	if (ret)
		return ret;

	/* Closed-loop braking measures the monitors' offsets */
	if (cs->ctl[CS40L25_VMON_IMON_OFFSET_ENABLE]) {
		if (cs->ctl[CS40L25_CLAB_ENABLED]) {
			ret = cs40l25_ctl_read(cs, CS40L25_CLAB_ENABLED, &clab);
			if (ret)
				return ret;
		}

		ret = cs40l25_ctl_write(cs, CS40L25_VMON_IMON_OFFSET_ENABLE,
					clab == 1);
		if (ret)
			return ret;
	}

	return cs40l25_ctl_read(cs, CS40L25_NUMBEROFWAVES, &cs->num_waves);
}

static const struct cs_dsp_client_ops cs40l25_client_ops = {
	.pre_run = cs40l25_dsp_pre_run,
	.post_run = cs40l25_dsp_post_run,
};

static void cs40l25_dsp_remove(void *data)
{
	cs_dsp_remove(data);
}

static void cs40l25_dsp_stop(void *data)
{
	cs_dsp_stop(data);
	cs_dsp_power_down(data);
}

static void cs40l25_wq_destroy(void *data)
{
	struct cs40l25 *cs = data;

	cancel_delayed_work_sync(&cs->timed_stop_work);
	destroy_workqueue(cs->wq);
}

static int cs40l25_input_register(struct cs40l25 *cs)
{
	struct input_dev *input;
	int ret;

	cs->wq = alloc_ordered_workqueue("cs40l25", WQ_HIGHPRI);
	if (!cs->wq)
		return -ENOMEM;

	INIT_WORK(&cs->start_work, cs40l25_start_worker);
	INIT_WORK(&cs->stop_work, cs40l25_stop_worker);
	INIT_DELAYED_WORK(&cs->timed_stop_work, cs40l25_timed_stop_worker);

	ret = devm_add_action_or_reset(cs->dev, cs40l25_wq_destroy, cs);
	if (ret)
		return ret;

	input = devm_input_allocate_device(cs->dev);
	if (!input)
		return -ENOMEM;

	input->name = "cs40l25-haptics";
	input->id.bustype = BUS_I2C;
	input_set_drvdata(input, cs);
	input_set_capability(input, EV_FF, FF_RUMBLE);
	input_set_capability(input, EV_FF, FF_PERIODIC);
	input_set_capability(input, EV_FF, FF_CUSTOM);
	input_set_capability(input, EV_FF, FF_GAIN);

	ret = input_ff_create(input, CS40L25_EFFECTS_MAX);
	if (ret)
		return ret;

	input->ff->upload = cs40l25_upload;
	input->ff->playback = cs40l25_playback;
	input->ff->set_gain = cs40l25_set_gain;
	cs->gain = U16_MAX;
	cs->input = input;

	return input_register_device(input);
}

static int cs40l25_dsp_bringup(struct cs40l25 *cs)
{
	const struct firmware *wmfw, *wavetable, *coeff = NULL;
	struct device *dev = cs->dev;
	char *coeff_name = NULL;
	size_t len;
	int ret;

	ret = request_firmware(&wmfw, cs->wmfw_name, dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to load %s\n",
				     cs->wmfw_name);

	ret = request_firmware(&wavetable, CS40L25_WAVETABLE, dev);
	if (ret) {
		dev_err(dev, "Failed to load %s: %d\n", CS40L25_WAVETABLE, ret);
		goto err_wmfw;
	}

	/* The firmware's tuning, if it has one: <name>.bin */
	len = strlen(cs->wmfw_name);
	if (len > 5 && !strcmp(cs->wmfw_name + len - 5, ".wmfw")) {
		coeff_name = kasprintf(GFP_KERNEL, "%.*s.bin", (int)len - 5,
				       cs->wmfw_name);
		if (!coeff_name) {
			ret = -ENOMEM;
			goto err_wavetable;
		}

		firmware_request_nowarn(&coeff, coeff_name, dev);
	}

	ret = cs_dsp_power_up(&cs->dsp, wmfw, cs->wmfw_name, wavetable,
			      CS40L25_WAVETABLE, "cs40l25a");
	if (ret)
		goto err_coeff;

	if (coeff) {
		mutex_lock(&cs->dsp.pwr_lock);
		ret = cs_dsp_load_coeff(&cs->dsp, coeff, coeff_name);
		mutex_unlock(&cs->dsp.pwr_lock);
		if (ret) {
			cs_dsp_power_down(&cs->dsp);
			goto err_coeff;
		}
	}

	ret = cs_dsp_run(&cs->dsp);
	if (ret) {
		cs_dsp_power_down(&cs->dsp);
		goto err_coeff;
	}

	ret = devm_add_action_or_reset(dev, cs40l25_dsp_stop, &cs->dsp);
err_coeff:
	release_firmware(coeff);
	kfree(coeff_name);
err_wavetable:
	release_firmware(wavetable);
err_wmfw:
	release_firmware(wmfw);

	return ret;
}

static int cs40l25_boost_config(struct cs40l25 *cs)
{
	struct regmap *regmap = cs->regmap;
	struct device *dev = cs->dev;
	u32 ind, cap, ipk, ctl = 11000, ovp;
	unsigned int lbst, cbst;
	int ret;

	ret = device_property_read_u32(dev, "cirrus,boost-ind-nanohenry", &ind);
	if (ret)
		return dev_err_probe(dev, ret, "No boost inductance\n");

	ret = device_property_read_u32(dev, "cirrus,boost-cap-microfarad", &cap);
	if (ret)
		return dev_err_probe(dev, ret, "No boost capacitance\n");

	ret = device_property_read_u32(dev, "cirrus,boost-peak-milliamp", &ipk);
	if (ret)
		return dev_err_probe(dev, ret, "No boost peak current\n");

	device_property_read_u32(dev, "cirrus,boost-ctl-millivolt", &ctl);

	switch (ind) {
	case 1000:
		lbst = 0;
		break;
	case 1200:
		lbst = 1;
		break;
	case 1500:
		lbst = 2;
		break;
	case 2200:
		lbst = 3;
		break;
	default:
		return dev_err_probe(dev, -EINVAL, "Invalid boost inductance %u\n",
				     ind);
	}

	if (cap < 20)
		cbst = 0;
	else if (cap <= 50)
		cbst = 1;
	else if (cap <= 100)
		cbst = 2;
	else if (cap <= 200)
		cbst = 3;
	else
		cbst = 4;

	if (ipk < 1600 || ipk > 4500)
		return dev_err_probe(dev, -EINVAL, "Invalid boost peak current %u\n",
				     ipk);

	if (ctl < 2550 || ctl > 11000)
		return dev_err_probe(dev, -EINVAL, "Invalid boost voltage %u\n",
				     ctl);

	ret = regmap_update_bits(regmap, CS40L25_BSTCVRT_COEFF,
				 CS40L25_BST_K1_MASK | CS40L25_BST_K2_MASK,
				 FIELD_PREP(CS40L25_BST_K1_MASK,
					    cs40l25_bst_k1[lbst][cbst]) |
				 FIELD_PREP(CS40L25_BST_K2_MASK,
					    cs40l25_bst_k2[lbst][cbst]));
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, CS40L25_BSTCVRT_SLOPE_LBST,
				 CS40L25_BST_SLOPE_MASK | CS40L25_BST_LBST_VAL_MASK,
				 FIELD_PREP(CS40L25_BST_SLOPE_MASK,
					    cs40l25_bst_slope[lbst]) |
				 FIELD_PREP(CS40L25_BST_LBST_VAL_MASK, lbst));
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, CS40L25_BSTCVRT_PEAK_CUR,
				 CS40L25_BST_IPK_MASK,
				 (ipk - 1600) / 50 + 0x10);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, CS40L25_BSTCVRT_VCTRL1,
				 CS40L25_BST_CTL_MASK, (ctl - 2550) / 50 + 1);
	if (ret)
		return ret;

	if (!device_property_read_u32(dev, "cirrus,boost-ovp-millivolt", &ovp)) {
		if (ovp < 9000 || ovp > 12875)
			return dev_err_probe(dev, -EINVAL,
					     "Invalid boost OVP %u\n", ovp);

		ret = regmap_update_bits(regmap, CS40L25_BSTCVRT_OVERVOLT_CTRL,
					 CS40L25_BST_OVP_EN |
					 CS40L25_BST_OVP_THLD_MASK,
					 CS40L25_BST_OVP_EN |
					 ((ovp - 9000) / 125) * 2);
		if (ret)
			return ret;
	}

	if (device_property_read_bool(dev, "cirrus,boost-dcm-disable")) {
		ret = regmap_write(regmap, CS40L25_BSTCVRT_DCM_CTRL,
				   CS40L25_DCM_DISABLE);
		if (ret)
			return ret;
	}

	/* Fixed boost voltage: no class H tracking */
	ret = regmap_update_bits(regmap, CS40L25_BSTCVRT_VCTRL2,
				 CS40L25_BST_CTL_SEL_MASK | CS40L25_BST_CTL_LIM_EN,
				 CS40L25_BST_CTL_LIM_EN);
	if (ret)
		return ret;

	return regmap_clear_bits(regmap, CS40L25_PWR_CTRL3,
				 CS40L25_CLASSH_EN_MASK);
}

static int cs40l25_basic_mode_exit(struct cs40l25 *cs)
{
	struct regmap *regmap = cs->regmap;
	unsigned int val, heartbeat;
	int ret;

	ret = regmap_read_poll_timeout(regmap, CS40L25_BASIC_AMP_STATUS, val,
				       val & CS40L25_BASIC_BOOT_DONE,
				       5000, 5000 * CS40L25_TIMEOUT_COUNT);
	if (ret)
		return dev_err_probe(cs->dev, ret, "Basic mode did not boot\n");

	ret = regmap_read(regmap, CS40L25_BASIC_HALO_HEARTBEAT, &heartbeat);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(regmap, CS40L25_BASIC_HALO_HEARTBEAT, val,
				       val > heartbeat,
				       5000, 5000 * CS40L25_TIMEOUT_COUNT);
	if (ret)
		return dev_err_probe(cs->dev, ret, "Basic mode has no heartbeat\n");

	ret = cs40l25_ack_write(cs, CS40L25_BASIC_SHUTDOWNREQUEST, 1, 0);
	if (ret)
		return ret;

	ret = regmap_read(regmap, CS40L25_BASIC_STATEMACHINE, &val);
	if (ret)
		return ret;

	if (val != CS40L25_BASIC_SHUTDOWN)
		return dev_err_probe(cs->dev, -EBUSY,
				     "Basic mode did not shut down: %u\n", val);

	ret = regmap_read(regmap, CS40L25_BASIC_AMP_STATUS, &val);
	if (ret)
		return ret;

	if (val & CS40L25_BASIC_OTP_ERROR)
		return dev_err_probe(cs->dev, -EIO, "OTP error in basic mode\n");

	/* Release the errors that basic mode latched */
	if (val & CS40L25_BASIC_AMP_ERROR) {
		ret = cs40l25_hw_err_release(cs, 0);
		if (ret)
			return ret;
	}

	if (val & CS40L25_BASIC_TEMP_ERROR) {
		ret = cs40l25_hw_err_release(cs, 1);
		if (ret)
			return ret;
	}

	if (val & CS40L25_BASIC_TEMP_RISE_WARN) {
		ret = cs40l25_hw_err_release(cs, 2);
		if (ret)
			return ret;
	}

	return regmap_multi_reg_write(regmap, cs40l25_basic_mode_revert,
				      ARRAY_SIZE(cs40l25_basic_mode_revert));
}

static int cs40l25_chip_init(struct cs40l25 *cs)
{
	struct regmap *regmap = cs->regmap;
	struct device *dev = cs->dev;
	unsigned int val, devid, revid;
	int ret;

	ret = regmap_read_poll_timeout(regmap, CS40L25_IRQ1_STATUS4, val,
				       val & CS40L25_OTP_BOOT_DONE,
				       10000, 10000 * CS40L25_TIMEOUT_COUNT);
	if (ret)
		return dev_err_probe(dev, ret, "OTP did not boot\n");

	ret = regmap_read(regmap, CS40L25_IRQ1_STATUS3, &val);
	if (ret)
		return ret;

	if (val & CS40L25_OTP_BOOT_ERR)
		return dev_err_probe(dev, -EIO, "OTP boot error\n");

	ret = regmap_read(regmap, CS40L25_DEVID, &devid);
	if (ret)
		return ret;

	ret = regmap_read(regmap, CS40L25_REVID, &revid);
	if (ret)
		return ret;

	if (devid != CS40L25_DEVID_A || revid < CS40L25_REVID_B1)
		return dev_err_probe(dev, -ENODEV,
				     "Unsupported device %#x revision %#x\n",
				     devid, revid);

	ret = cs40l25_basic_mode_exit(cs);
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(regmap, cs40l25_errata,
				     ARRAY_SIZE(cs40l25_errata));
	if (ret)
		return ret;

	ret = cs40l25_boost_config(cs);
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(regmap, cs40l25_pcm_routing,
				     ARRAY_SIZE(cs40l25_pcm_routing));
	if (ret)
		return ret;

	dev_info(dev, "Cirrus Logic CS40L25A revision %02X\n", revid);

	return 0;
}

static void cs40l25_reset(void *data)
{
	gpiod_set_value_cansleep(data, 1);
}

static int cs40l25_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cs40l25 *cs;
	struct clk *clk;
	int ret;

	cs = devm_kzalloc(dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;

	cs->dev = dev;
	i2c_set_clientdata(client, cs);

	cs->regmap = devm_regmap_init_i2c(client, &cs40l25_regmap);
	if (IS_ERR(cs->regmap))
		return dev_err_probe(dev, PTR_ERR(cs->regmap),
				     "Failed to initialise regmap\n");

	cs->wmfw_name = CS40L25_DEFAULT_WMFW;
	device_property_read_string(dev, "firmware-name", &cs->wmfw_name);

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(cs40l25_supplies),
					     cs40l25_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable supplies\n");

	/* The ROM runs the DSP from the 32.768 kHz reference clock */
	clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Failed to enable the reference clock\n");

	cs->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(cs->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(cs->reset_gpio),
				     "Failed to get reset GPIO\n");

	/* Reset pulse, then the control port delay, both with margin */
	usleep_range(2000, 2100);
	gpiod_set_value_cansleep(cs->reset_gpio, 0);
	usleep_range(1000, 1100);

	ret = devm_add_action_or_reset(dev, cs40l25_reset, cs->reset_gpio);
	if (ret)
		return ret;

	ret = cs40l25_chip_init(cs);
	if (ret)
		return ret;

	cs->dsp.num = 1;
	cs->dsp.type = WMFW_HALO;
	cs->dsp.dev = dev;
	cs->dsp.regmap = cs->regmap;
	cs->dsp.base = CS40L25_DSP1_CORE_BASE;
	cs->dsp.mem = cs40l25_dsp_regions;
	cs->dsp.num_mems = ARRAY_SIZE(cs40l25_dsp_regions);
	cs->dsp.lock_regions = 0xFFFFFFFF;
	cs->dsp.client_ops = &cs40l25_client_ops;

	ret = cs_dsp_halo_init(&cs->dsp);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, cs40l25_dsp_remove, &cs->dsp);
	if (ret)
		return ret;

	ret = cs40l25_dsp_bringup(cs);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, client->irq, NULL, cs40l25_irq,
					IRQF_ONESHOT, client->name, cs);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");

	ret = cs40l25_input_register(cs);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register input device\n");

	dev_info(dev, "Firmware %#x, %u waveforms\n", cs->dsp.fw_id,
		 cs->num_waves);

	return 0;
}

static const struct of_device_id cs40l25_of_match[] = {
	{ .compatible = "cirrus,cs40l25a" },
	{ }
};
MODULE_DEVICE_TABLE(of, cs40l25_of_match);

static const struct i2c_device_id cs40l25_id[] = {
	{ "cs40l25a" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cs40l25_id);

static struct i2c_driver cs40l25_driver = {
	.driver = {
		.name = "cs40l25",
		.of_match_table = cs40l25_of_match,
		/* Loading the firmware over I2C takes about a second */
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = cs40l25_probe,
	.id_table = cs40l25_id,
};
module_i2c_driver(cs40l25_driver);

MODULE_DESCRIPTION("Cirrus Logic CS40L25A haptic driver");
MODULE_AUTHOR("Noswad <64293615+TheNoswad@users.noreply.github.com>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("FW_CS_DSP");
