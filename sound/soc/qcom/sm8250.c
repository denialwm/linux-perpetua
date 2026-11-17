// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/soundwire/sdw.h>
#include <sound/jack.h>
#include <linux/input-event-codes.h>
#include "qdsp6/q6afe.h"
#include "common.h"
#include "usb_offload_utils.h"
#include "sdw.h"
#include "../codecs/rt5514.h"

#define MI2S_BCLK_RATE		1536000

struct sm8250_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct snd_soc_jack jack;
	struct snd_soc_jack usb_offload_jack;
	bool usb_offload_jack_setup;
	struct snd_soc_jack dp_jack;
	bool jack_setup;
};

/*
 * Actual TDM bit clock the codec sees: nslots * slot_width * rate. This is
 * NOT tdm_clk_rate, which is the LPASS interface clock requested from the DSP.
 */
static const int cap_bclk_rate = 12288000;
static const int tdm_slots = 8;
static const int tdm_slot_width = 32;
/*
 * Slot mask must have one bit per active channel. The capture side carries
 * one channel (stock: "PRI_TDM_TX_0 Channels" = One), in slot 0. 0xFF (all
 * eight slots with ch=2) is rejected by the ADSP:
 * AFE_PORT_CMD_DEVICE_START comes back ADSP_EFAILED (-EINVAL at
 * snd_soc_dai_prepare).
 */
static const int tdm_rx_mask = 0x1;
/*
 * The stock machine driver uses four slots, but this mainline clock path
 * rejects 6.144 MHz. Eight slots at 12.288 MHz have been verified with
 * live TAS2562 clock status and acoustic tone captures on barbet.
 */
static const int quin_tdm_slots = 8;
static const int quin_clk_id = Q6AFE_LPASS_CLK_ID_QUIN_TDM_IBIT;
static const int tdm_clk_rate = 12288000;
/* Stock sets the PRI TDM interface clock through LPAIF_DIG_CLK (clk_id 0 in
 * the vendor driver), not through the LPASS TDM IBIT clock id. */
static const int tdm_pri_clk_id = LPAIF_DIG_CLK;

static int sm8250_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	switch (cpu_dai->id) {
	case DISPLAY_PORT_RX:
		return qcom_snd_dp_jack_setup(rtd, &data->dp_jack, 0);
	case USB_RX:
		return qcom_snd_usb_offload_jack_setup(rtd, &data->usb_offload_jack,
						       &data->usb_offload_jack_setup);
	default:
		return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
	}
}

static void sm8250_snd_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	if (cpu_dai->id == USB_RX)
		qcom_snd_usb_offload_jack_remove(rtd,
						 &data->usb_offload_jack_setup);

}

static int sm8250_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	unsigned int format = SNDRV_PCM_FORMAT_S16_LE;

	/*
	 * Stock barbet feeds the TAS2562 speaker on QUIN TDM RX as S24_LE
	 * (mixer_paths_bolero_snd.xml: "QUIN_TDM_RX_0 Format" = S24_LE).
	 * Forcing S16_LE there leaves the amp decoding mostly-zero words.
	 */
	if (cpu_dai && cpu_dai->id == QUINARY_TDM_RX_0)
		format = SNDRV_PCM_FORMAT_S24_LE;

	rate->min = rate->max = 48000;
	if (cpu_dai && cpu_dai->id == PRIMARY_TDM_TX_0)
		channels->min = channels->max = 1;
	else
		channels->min = channels->max = 2;
	/*
	 * snd_mask_set_format() only adds to the allowed set, so the FE's
	 * format would still win.  Clear the mask to actually force the BE
	 * format (stock barbet runs QUIN TDM RX as S24_LE).
	 */
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, format);

	return 0;
}

static int sm8250_snd_startup(struct snd_pcm_substream *substream)
{
	unsigned int fmt = SND_SOC_DAIFMT_BP_FP;
	unsigned int codec_dai_fmt = SND_SOC_DAIFMT_BC_FC;
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);

	dev_info(rtd->card->dev,
		 "startup: link=%s cpu_dai=%s id=%d slots=%d width=%d rx_mask=0x%x clk_rate=%d\n",
		 rtd->dai_link->name, cpu_dai->name, cpu_dai->id, tdm_slots,
		 tdm_slot_width, tdm_rx_mask, tdm_clk_rate);

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	case SECONDARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	case TERTIARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	case QUINARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_QUI_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	case LPI_MI2S_RX_0:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_INT0_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	case LPI_MI2S_TX_3:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_INT3_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_CAPTURE);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	case PRIMARY_TDM_TX_0: {
		/*
		 * Barbet's built-in microphones are DMICs on the RT5514P, read
		 * over PRIMARY_TDM_TX_0. This mirrors the vendor machine driver
		 * (kona_tdm_be_hw_params): 8 slots of 32 bits, all slots in the
		 * mask, one capture channel at byte offset 0, the LPASS
		 * interface clock at rate*width*slots, and the codec in TDM
		 * (PCM_A) mode with only slot 0 as its own TX channel.
		 */
		unsigned int tx_slots[] = { 0 };
		int ret_pll, ret_clk, ret_ctdm, ret_cfmt;
		int ret_cpu_clk, ret_cpu_tdm, ret_cpu_map;

		ret_cpu_clk = snd_soc_dai_set_sysclk(cpu_dai,
				       tdm_pri_clk_id,
				       tdm_clk_rate,
				       SNDRV_PCM_STREAM_CAPTURE);
		ret_cpu_tdm = snd_soc_dai_set_tdm_slot(cpu_dai, tdm_rx_mask, 0,
					 tdm_slots, tdm_slot_width);
		ret_cpu_map = snd_soc_dai_set_channel_map(cpu_dai, 1, tx_slots,
							  0, NULL);

		ret_ctdm = snd_soc_dai_set_tdm_slot(codec_dai, 1, 0,
						    tdm_slots, tdm_slot_width);
		ret_cfmt = snd_soc_dai_set_fmt(codec_dai,
					       SND_SOC_DAIFMT_DSP_A |
					       SND_SOC_DAIFMT_NB_NF);

		/*
		 * There is no MCLK wire to the RT5514 on barbet: the codec PLL
		 * runs from the TDM bit clock, and stock programs it with the
		 * bit clock as both input and output.
		 */
		ret_pll = snd_soc_dai_set_pll(codec_dai, 0,
					    RT5514_PLL_S_BCLK,
					    cap_bclk_rate, cap_bclk_rate);
		ret_clk = snd_soc_dai_set_sysclk(codec_dai,
					       RT5514_SCLK_S_PLL,
					       cap_bclk_rate, SND_SOC_CLOCK_IN);
		dev_info(rtd->card->dev,
			 "cap: cpu(clk=%d tdm=%d map=%d) codec(tdm=%d fmt=%d pll=%d clk=%d) slots=%d width=%d mask=0x%x\n",
			 ret_cpu_clk, ret_cpu_tdm, ret_cpu_map, ret_ctdm, ret_cfmt,
			 ret_pll, ret_clk, tdm_slots, tdm_slot_width,
			 tdm_rx_mask);
		break;
	}

		case QUINARY_TDM_RX_0: {
		unsigned int rx_slots[] = { 0, tdm_slot_width / 8 };
		unsigned int quin_slots = quin_tdm_slots;
		/* Two channels live in slots 0 and 1 (byte offsets 0 and 4). */
		unsigned int quin_mask = (1U << 2) - 1;
		unsigned int quin_rate = quin_slots * tdm_slot_width * 48000;
		int ret_sysclk, ret_tdm, ret_map;
		int i;

		/*
		 * Barbet speakers: TAS2562 on Quinary TDM RX.  Stock runs
		 * 4 slots of 32 bits at 48 kHz (6.144 MHz), both channels in
		 * slots 0 and 1, with DSP_A sync. This clock path requires
		 * eight slots at 12.288 MHz; keep slots 0 and 1 enabled for
		 * the paired amplifiers. RX offset 1 is a bit delay, not a
		 * channel-slot selection.
		 * Stock's mux/clock is re-applied at stream start in
		 * tas2562_hw_params(); the LPASS power-up otherwise wipes the
		 * LPI pad mux.
		 */
		ret_sysclk = snd_soc_dai_set_sysclk(cpu_dai, quin_clk_id,
						    quin_rate,
						    SNDRV_PCM_STREAM_PLAYBACK);
		ret_tdm = snd_soc_dai_set_tdm_slot(cpu_dai, 0, quin_mask,
						   quin_slots, tdm_slot_width);
		ret_map = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL, 2,
						      rx_slots);
		dev_info(rtd->card->dev,
			 "quin cpu: sysclk=%d tdm_slot=%d chan_map=%d slots=%u mask=0x%x rate=%u\n",
			 ret_sysclk, ret_tdm, ret_map, quin_slots, quin_mask,
			 quin_rate);
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_DSP_A;
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		i = 0;
		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			unsigned int mask = 1U << i;
			int ret_ctdm, ret_cfmt;

			ret_ctdm = snd_soc_dai_set_tdm_slot(codec_dai, mask, mask,
							    quin_slots,
							    tdm_slot_width);
			ret_cfmt = snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
			dev_info(rtd->card->dev,
				 "quin codec %s i=%d mask=0x%x tdm_slot=%d fmt=%d\n",
				 codec_dai->name, i, mask, ret_ctdm, ret_cfmt);
		}
		break;
	}
	default:
		break;
	}

	return qcom_snd_sdw_startup(substream);
}

static int sm8250_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);

	return qcom_snd_sdw_prepare(substream, &data->stream_prepared[cpu_dai->id]);
}

static int sm8250_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	return qcom_snd_sdw_hw_free(substream, &data->stream_prepared[cpu_dai->id]);
}

static const struct snd_soc_ops sm8250_be_ops = {
	.startup = sm8250_snd_startup,
	.shutdown = qcom_snd_sdw_shutdown,
	.hw_free = sm8250_snd_hw_free,
	.prepare = sm8250_snd_prepare,
};

/*
 * Barbet captures through the RT5514P, whose ADC/DSP produces 32 kHz data
 * while the primary TDM backend runs at 48 kHz. The ADSP only returns usable
 * samples for a 32 kHz front end; at 48 kHz userspace receives a fixed
 * bit pattern instead of audio, so pin the capture rate.
 */
static int sm8250_fe_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (strcmp(rtd->card->name, "barbet") == 0 &&
	    substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		runtime->hw.rates = SNDRV_PCM_RATE_32000;
		runtime->hw.rate_min = 32000;
		runtime->hw.rate_max = 32000;
	}

	return 0;
}

static const struct snd_soc_ops sm8250_fe_ops = {
	.startup = sm8250_fe_startup,
};

static void sm8250_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->init = sm8250_snd_init;
			link->exit = sm8250_snd_exit;
			link->be_hw_params_fixup = sm8250_be_hw_params_fixup;
			link->ops = &sm8250_be_ops;
		} else if (strcmp(card->name, "barbet") == 0 && !link->ops) {
			link->ops = &sm8250_fe_ops;
		}
	}
}

static void sm8250_disable_tdm_clock(void *data)
{
	clk_disable_unprepare(data);
}

static int sm8250_enable_tdm_clocks(struct device *dev)
{
	static const char * const tdm_clk_names[] = {
		"lpass_pri_tdm_ibit", "lpass_quin_tdm_ibit",
	};
	struct clk *tdm_clk;
	int i, ret;

	/*
	 * Enabling these makes q6afe issue a global AFE_PARAM_ID_CLOCK_SET for
	 * the TDM bit clocks, which is what actually starts the LPASS TDM
	 * clock generators. Do not fail the card if the DSP rejects them.
	 */
	for (i = 0; i < ARRAY_SIZE(tdm_clk_names); i++) {
		tdm_clk = devm_clk_get_optional(dev, tdm_clk_names[i]);
		if (IS_ERR(tdm_clk)) {
			if (PTR_ERR(tdm_clk) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dev_warn(dev, "failed to get %s: %ld\n",
				 tdm_clk_names[i], PTR_ERR(tdm_clk));
			continue;
		}
		if (!tdm_clk)
			continue;
		clk_set_rate(tdm_clk, tdm_clk_rate);
		ret = clk_prepare_enable(tdm_clk);
		dev_info(dev, "%s: rate=%u enable_ret=%d\n", tdm_clk_names[i],
			 tdm_clk_rate, ret);
		if (ret)
			continue;
		ret = devm_add_action_or_reset(dev, sm8250_disable_tdm_clock,
					      tdm_clk);
		if (ret)
			return ret;
	}

	return 0;
}

static int sm8250_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct sm8250_snd_data *data;
	struct device *dev = &pdev->dev;
	struct clk *lpass_clk, *dcodec_clk;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->owner = THIS_MODULE;
	/* Allocate the private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);

	lpass_clk = devm_clk_get_optional_enabled(dev, "lpass_audio_hw_vote");
	if (IS_ERR(lpass_clk))
		return dev_err_probe(dev, PTR_ERR(lpass_clk),
				     "failed to enable LPASS vote clock\n");

	dcodec_clk = devm_clk_get_optional_enabled(dev, "lpass_dcodec_hw_vote");
	if (IS_ERR(dcodec_clk))
		dev_warn(dev, "failed to enable LPASS dcodec vote: %ld\n",
			 PTR_ERR(dcodec_clk));

	ret = sm8250_enable_tdm_clocks(dev);
	if (ret)
		return ret;

	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	card->driver_name = of_device_get_match_data(dev);
	sm8250_add_be_ops(card);
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id snd_sm8250_dt_match[] = {
	{ .compatible = "fairphone,fp4-sndcard", .data = "sm7225" },
	{ .compatible = "fairphone,fp5-sndcard", .data = "qcm6490" },
	{ .compatible = "qcom,qrb2210-sndcard", .data = "qcm2290" },
	{ .compatible = "qcom,qrb4210-rb2-sndcard", .data = "sm4250" },
	{ .compatible = "qcom,qrb5165-rb5-sndcard", .data = "sm8250" },
	{ .compatible = "qcom,sdm660-sndcard", .data = "sdm660" },
	{ .compatible = "qcom,sm8250-sndcard", .data = "sm8250" },
	{}
};

MODULE_DEVICE_TABLE(of, snd_sm8250_dt_match);

static struct platform_driver snd_sm8250_driver = {
	.probe  = sm8250_platform_probe,
	.driver = {
		.name = "snd-sm8250",
		.of_match_table = snd_sm8250_dt_match,
	},
};
module_platform_driver(snd_sm8250_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("SM8250 ASoC Machine Driver");
MODULE_LICENSE("GPL");
