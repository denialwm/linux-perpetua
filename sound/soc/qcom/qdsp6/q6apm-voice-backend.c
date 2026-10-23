// SPDX-License-Identifier: GPL-2.0
#include <linux/cleanup.h>
#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <sound/pcm_params.h>
#include "q6apm-voice-backend.h"

static const struct file_operations q6apm_voice_file_ops = {
	.owner = THIS_MODULE,
};

static void q6apm_voice_interval(struct snd_pcm_hw_params *params, int key, unsigned int value)
{
	struct snd_interval *interval = hw_param_interval(params, key);

	*interval = (struct snd_interval) { .min = value, .max = value, .integer = 1 };
}

void q6apm_voice_backend_close(struct snd_pcm_substream **stream)
{
	struct snd_pcm_substream *substream = *stream;

	if (!substream)
		return;
	guard(mutex)(&substream->pcm->open_mutex);
	snd_pcm_release_substream(substream);
	*stream = NULL;
}

int q6apm_voice_backend_open(struct snd_soc_component *component, unsigned int graph,
			     struct snd_pcm_substream **stream)
{
	const struct q6apm_voice_board *board = q6apm_voice_board();
	const char *name = graph ? Q6APM_VOICE_TX_LINK : Q6APM_VOICE_RX_LINK;
	struct snd_soc_pcm_runtime *rtd, *selected = NULL;
	struct snd_soc_dai *dai;
	unsigned int tx_num, rx_num, tx[8], rx[8];
	bool matched = false;
	int i, ret;
	struct file *file __free(fput) = NULL;

	struct snd_pcm_hw_params *params __free(kfree) = kzalloc_obj(*params);
	if (!params)
		return -ENOMEM;
	if (!board)
		return -ENODEV;
	for_each_card_rtds(component->card, rtd)
		if (!strcmp(rtd->dai_link->name, name)) {
			selected = rtd;
			break;
		}
	if (!selected || !selected->pcm)
		return -ENODEV;
	/* ALSA attach reads f_flags; a kernel PCM open still needs a valid file. */
	file = anon_inode_getfile("[q6apm-voice]", &q6apm_voice_file_ops, NULL, O_RDWR);
	if (IS_ERR(file))
		return PTR_ERR(file);
	scoped_guard(mutex, &selected->pcm->open_mutex) {
		ret = snd_pcm_open_substream(selected->pcm, graph ? SNDRV_PCM_STREAM_CAPTURE :
					    SNDRV_PCM_STREAM_PLAYBACK, file, stream);
	}
	if (ret)
		return ret;
	if (graph) {
		for_each_rtd_codec_dais(selected, i, dai) {
			tx_num = 0;
			rx_num = 0;
			memset(tx, 0, sizeof(tx));
			memset(rx, 0, sizeof(rx));
			ret = snd_soc_dai_get_channel_map(dai, &tx_num, tx, &rx_num, rx);
			if (ret == -ENOTSUPP)
				continue;
			if (ret)
				goto out;
			/* The codec DMA endpoint reads these decimators. */
			if (tx_num && (tx[0] & board->tx_decimators) == board->tx_decimators)
				matched = true;
		}
		if (!matched) {
			dev_err(component->dev, "voice backend TX requires DEC mask %#x\n",
				board->tx_decimators);
			ret = -EINVAL;
			goto out;
		}
	}
	_snd_pcm_hw_params_any(params);
	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_ACCESS));
	snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_ACCESS),
		     SNDRV_PCM_ACCESS_RW_INTERLEAVED);
	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
	snd_mask_set_format(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
			    graph ? SNDRV_PCM_FORMAT_S16_LE : board->rx_format);
	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_SUBFORMAT));
	snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_SUBFORMAT), SNDRV_PCM_SUBFORMAT_STD);
	q6apm_voice_interval(params, SNDRV_PCM_HW_PARAM_RATE, graph ? 48000 : board->rx_rate);
	q6apm_voice_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS, 2);
	q6apm_voice_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE, 256);
	q6apm_voice_interval(params, SNDRV_PCM_HW_PARAM_PERIODS, 2);
	q6apm_voice_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_SIZE, 512);
	ret = snd_pcm_kernel_ioctl(*stream, SNDRV_PCM_IOCTL_HW_PARAMS, params);
	if (!ret)
		ret = snd_pcm_kernel_ioctl(*stream, SNDRV_PCM_IOCTL_PREPARE, NULL);
	/* PREPARE powers DAPM/SoundWire; the raw voice graph supplies DSP START. */
out:
	if (ret)
		q6apm_voice_backend_close(stream);
	return ret;
}
