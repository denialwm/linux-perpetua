// SPDX-License-Identifier: GPL-2.0-only
/*
 * NXP/Goodix TFA9872, TFA9873 and TFA9874 (TFA2 family) amplifiers
 *
 * These amplifiers have no DSP; speaker protection would run on the host.
 * The driver sets the amplifier up for 16-bit I2S at 48 kHz and plays one
 * slot of the frame, without current or voltage sense.
 *
 * Based on the TFA9873 driver of the OnePlus Ace 3 mainline port by
 * Shandorman <98683030+jiganomegsdfdf@users.noreply.github.com>.
 * Register fields and the TFA9873 B0 optimal settings from the NXP
 * tfa98xx driver,
 * Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <sound/soc.h>

#define TFA987X_SYS_CTRL0		0x00
#define TFA987X_SYS_CTRL0_PWDN		BIT(0)
#define TFA987X_SYS_CTRL0_I2CR		BIT(1)
#define TFA987X_SYS_CTRL0_AMPE		BIT(3)
#define TFA987X_SYS_CTRL0_DCA		BIT(4)

#define TFA987X_SYS_CTRL1		0x01
#define TFA987X_SYS_CTRL1_MANSCONF	BIT(2)

#define TFA987X_REV			0x03

#define TFA987X_CLK_CTRL		0x04
#define TFA987X_CLK_CTRL_FSSYNCEN	BIT(8)

#define TFA987X_HIDE_KEY		0x0f
#define TFA987X_HIDE_KEY_VAL		0x5a6b
#define TFA987X_KEY1			0xa0
#define TFA987X_KEY2			0xa1
#define TFA987X_KEY_UNLOCK		0x005a
#define TFA987X_KEY1_FUSE		0xfb

#define TFA987X_TDM_CFG0		0x20
#define TFA9873_TDM_CFG0_SLOTS		GENMASK(4, 1)
#define TFA987X_TDM_CFG0_NBCK		GENMASK(15, 12)
#define TFA987X_TDM_CFG1		0x21
#define TFA9872_TDM_CFG1_SLOTS		GENMASK(3, 0)
#define TFA987X_TDM_CFG1_SLLN		GENMASK(8, 4)
#define TFA987X_TDM_CFG2		0x22
#define TFA987X_TDM_CFG2_SSIZE		GENMASK(6, 2)
#define TFA987X_TDM_CFG3		0x23
#define TFA987X_TDM_CFG3_SPKE		BIT(0)
#define TFA987X_TDM_CFG3_DCE		BIT(1)
#define TFA987X_TDM_CFG3_CSE		BIT(3)
#define TFA987X_TDM_CFG3_VSE		BIT(4)
#define TFA987X_TDM_CFG6		0x26
#define TFA987X_TDM_CFG6_SPKS		GENMASK(3, 0)

#define TFA987X_AUDIO_CTRL		0x51
#define TFA987X_AUDIO_CTRL_DPSA		BIT(7)

#define TFA9872_MODE1_DET1		0x64
#define TFA9872_MODE1_DET1_LPM1MODE	GENMASK(15, 14)

/* Optimal settings of the NXP tfa98xx driver for TFA9873 B0 (version 16) */
static const struct reg_sequence tfa9873_b0_init[] = {
	{ 0x02, 0x0628 },
	{ 0x61, 0x0183 },
	{ 0x63, 0x005a },
	{ 0x6f, 0x0082 },
	{ 0x70, 0xa3eb },
	{ 0x73, 0x0187 },
	{ 0x83, 0x071c },
	{ 0x85, 0x0380 },
	{ 0xd5, 0x004d },
};

static int tfa987x_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	return snd_soc_component_update_bits(dai->component, TFA987X_SYS_CTRL0,
					     TFA987X_SYS_CTRL0_AMPE,
					     mute ? 0 : TFA987X_SYS_CTRL0_AMPE);
}

static const struct snd_soc_dapm_widget tfa987x_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Speaker"),
	SND_SOC_DAPM_OUT_DRV("PWUP", TFA987X_SYS_CTRL0, 0, 1, NULL, 0),
};

static const struct snd_soc_dapm_route tfa987x_dapm_routes[] = {
	{ "PWUP", NULL, "HiFi Playback" },
	{ "Speaker", NULL, "PWUP" },
};

static const struct snd_soc_component_driver tfa987x_component = {
	.dapm_widgets		= tfa987x_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tfa987x_dapm_widgets),
	.dapm_routes		= tfa987x_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(tfa987x_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct snd_soc_dai_ops tfa987x_dai_ops = {
	.mute_stream = tfa987x_mute_stream,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver tfa987x_dai = {
	.name = "tfa987x-hifi",
	.playback = {
		.stream_name	= "HiFi Playback",
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
		.rates		= SNDRV_PCM_RATE_48000,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.ops = &tfa987x_dai_ops,
};

static const struct regmap_config tfa987x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xff,
};

static int tfa987x_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct gpio_desc *reset;
	struct regmap *regmap;
	unsigned int rev;
	u32 slot = 0;
	int ret;

	ret = devm_regulator_get_enable(dev, "vddd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable vddd\n");

	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "Failed to get reset GPIO\n");

	if (reset) {
		gpiod_set_value_cansleep(reset, 1);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(reset, 0);
		usleep_range(1000, 2000);
	}

	of_property_read_u32(dev->of_node, "nxp,tdm-slot", &slot);
	if (!FIELD_FIT(TFA987X_TDM_CFG6_SPKS, slot))
		return dev_err_probe(dev, -EINVAL, "Invalid TDM slot %u\n", slot);

	regmap = devm_regmap_init_i2c(i2c, &tfa987x_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = regmap_read(regmap, TFA987X_REV, &rev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read revision\n");

	switch (rev) {
	case 0x1b72:
	case 0x2b72:
	case 0x3b72:
	case 0x0b73:
	case 0x0c74:
		dev_info(dev, "TFA98%02x revision 0x%04x\n", rev & 0xff, rev);
		break;
	default:
		return dev_err_probe(dev, -ENODEV, "Unsupported revision 0x%04x\n", rev);
	}

	regmap_write(regmap, TFA987X_SYS_CTRL0, TFA987X_SYS_CTRL0_I2CR);

	if (rev == 0x0b73) {
		unsigned int fuse;

		/* Unlock the hidden registers that the optimal settings write */
		regmap_write(regmap, TFA987X_HIDE_KEY, TFA987X_HIDE_KEY_VAL);
		regmap_read(regmap, TFA987X_KEY1_FUSE, &fuse);
		regmap_write(regmap, TFA987X_KEY1, (fuse ^ TFA987X_KEY_UNLOCK) & 0xffff);
		regmap_write(regmap, TFA987X_KEY2, TFA987X_KEY_UNLOCK);

		ret = regmap_multi_reg_write(regmap, tfa9873_b0_init,
					     ARRAY_SIZE(tfa9873_b0_init));
		if (ret)
			return dev_err_probe(dev, ret, "Failed to write optimal settings\n");

		regmap_update_bits(regmap, TFA987X_CLK_CTRL, TFA987X_CLK_CTRL_FSSYNCEN, 0);
	}

	/* Boost converter on, with its reset defaults */
	regmap_update_bits(regmap, TFA987X_SYS_CTRL0, TFA987X_SYS_CTRL0_DCA,
			   TFA987X_SYS_CTRL0_DCA);
	regmap_update_bits(regmap, TFA987X_AUDIO_CTRL, TFA987X_AUDIO_CTRL_DPSA, 0);

	/* Frame of two 16-bit slots; play the configured slot, sense nothing */
	if ((rev & 0xff) == 0x73)
		regmap_update_bits(regmap, TFA987X_TDM_CFG0, TFA9873_TDM_CFG0_SLOTS,
				   FIELD_PREP(TFA9873_TDM_CFG0_SLOTS, 1));
	else
		regmap_update_bits(regmap, TFA987X_TDM_CFG1, TFA9872_TDM_CFG1_SLOTS,
				   FIELD_PREP(TFA9872_TDM_CFG1_SLOTS, 1));
	regmap_update_bits(regmap, TFA987X_TDM_CFG0, TFA987X_TDM_CFG0_NBCK,
			   FIELD_PREP(TFA987X_TDM_CFG0_NBCK, 0));
	regmap_update_bits(regmap, TFA987X_TDM_CFG1, TFA987X_TDM_CFG1_SLLN,
			   FIELD_PREP(TFA987X_TDM_CFG1_SLLN, 15));
	regmap_update_bits(regmap, TFA987X_TDM_CFG2, TFA987X_TDM_CFG2_SSIZE,
			   FIELD_PREP(TFA987X_TDM_CFG2_SSIZE, 15));
	regmap_update_bits(regmap, TFA987X_TDM_CFG3,
			   TFA987X_TDM_CFG3_SPKE | TFA987X_TDM_CFG3_DCE |
			   TFA987X_TDM_CFG3_CSE | TFA987X_TDM_CFG3_VSE,
			   TFA987X_TDM_CFG3_SPKE);
	regmap_update_bits(regmap, TFA987X_TDM_CFG6, TFA987X_TDM_CFG6_SPKS,
			   FIELD_PREP(TFA987X_TDM_CFG6_SPKS, slot));

	if ((rev & 0xff) == 0x72)
		regmap_update_bits(regmap, TFA9872_MODE1_DET1,
				   TFA9872_MODE1_DET1_LPM1MODE,
				   FIELD_PREP(TFA9872_MODE1_DET1_LPM1MODE, 1));

	/* Configuration done: let the manager leave its configuration state */
	regmap_update_bits(regmap, TFA987X_SYS_CTRL0, TFA987X_SYS_CTRL0_PWDN, 0);
	regmap_update_bits(regmap, TFA987X_SYS_CTRL1, TFA987X_SYS_CTRL1_MANSCONF,
			   TFA987X_SYS_CTRL1_MANSCONF);

	return devm_snd_soc_register_component(dev, &tfa987x_component,
					       &tfa987x_dai, 1);
}

static const struct of_device_id tfa987x_of_match[] = {
	{ .compatible = "nxp,tfa9872" },
	{ .compatible = "nxp,tfa9873" },
	{ .compatible = "nxp,tfa9874" },
	{ }
};
MODULE_DEVICE_TABLE(of, tfa987x_of_match);

static struct i2c_driver tfa987x_i2c_driver = {
	.driver = {
		.name = "tfa987x",
		.of_match_table = tfa987x_of_match,
	},
	.probe = tfa987x_i2c_probe,
};
module_i2c_driver(tfa987x_i2c_driver);

MODULE_DESCRIPTION("ASoC NXP TFA9872/TFA9873/TFA9874 amplifier driver");
MODULE_LICENSE("GPL");
