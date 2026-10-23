/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __Q6APM_VOICE_BACKEND_H__
#define __Q6APM_VOICE_BACKEND_H__

#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include <linux/of.h>
#include <sound/soc.h>

#define Q6APM_VOICE_RX_LINK "Voice Playback"
#define Q6APM_VOICE_TX_LINK "Voice Capture"

/* A board whose modem calls run through hostless DSP voice graphs. */
struct q6apm_voice_board {
	const char *compatible;
	/* Graph payloads and ordered commands, generated from the stock ACDB */
	const char *firmware;
	/* Backend links the private voice links copy */
	unsigned int rx_port, tx_port;
	unsigned int rx_rate;
	snd_pcm_format_t rx_format;
	/* Decimators the TX graph's codec DMA endpoint reads */
	unsigned int tx_decimators;
	/* CAL_KEYS volume indices the stock audio HAL uses; index 0 is loudest */
	unsigned int volume_levels;
};

static const struct q6apm_voice_board q6apm_voice_boards[] = {
	{
		.compatible = "oneplus,aston",
		.firmware = "qcom/sm8550/aston-voice.bin",
		.rx_port = SECONDARY_MI2S_RX,
		.tx_port = TX_CODEC_DMA_TX_3,
		.rx_rate = 48000,
		.rx_format = SNDRV_PCM_FORMAT_S16_LE,
		.tx_decimators = 0x1,
		.volume_levels = 6,
	},
};

static inline const struct q6apm_voice_board *q6apm_voice_board(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(q6apm_voice_boards); i++)
		if (of_machine_is_compatible(q6apm_voice_boards[i].compatible))
			return &q6apm_voice_boards[i];
	return NULL;
}

static inline bool q6apm_voice_backend_link(const struct snd_soc_pcm_runtime *rtd)
{
	return !strcmp(rtd->dai_link->name, Q6APM_VOICE_RX_LINK) ||
		!strcmp(rtd->dai_link->name, Q6APM_VOICE_TX_LINK);
}

int q6apm_voice_backend_open(struct snd_soc_component *component, unsigned int graph,
			     struct snd_pcm_substream **stream);
void q6apm_voice_backend_close(struct snd_pcm_substream **stream);

#endif
