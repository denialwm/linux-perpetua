// SPDX-License-Identifier: GPL-2.0
#include <linux/hrtimer.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include "q6apm-voice.h"

/* A clocked policy PCM: modem samples stay inside the hostless DSP graphs. */
struct q6apm_voice_pcm {
	struct snd_pcm_substream *substream;
	struct hrtimer timer;
	ktime_t base_time;
	ktime_t period_time;
	u64 paused_frames;
	atomic_t running;
};

static const struct snd_pcm_hardware q6apm_voice_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 256 * 1024,
	.period_bytes_min = 256,
	.period_bytes_max = 128 * 1024,
	.periods_min = 2,
	.periods_max = 1024,
};

static enum hrtimer_restart q6apm_voice_pcm_elapsed(struct hrtimer *timer)
{
	struct q6apm_voice_pcm *pcm = container_of(timer, struct q6apm_voice_pcm, timer);

	if (!atomic_read(&pcm->running))
		return HRTIMER_NORESTART;
	snd_pcm_period_elapsed(pcm->substream);
	if (!atomic_read(&pcm->running))
		return HRTIMER_NORESTART;
	hrtimer_forward_now(timer, pcm->period_time);
	return HRTIMER_RESTART;
}

static int q6apm_voice_pcm_open(struct snd_pcm_substream *substream)
{
	struct q6apm_voice_pcm *pcm;
	int ret;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;
	pcm = kzalloc_obj(*pcm);
	if (!pcm) {
		ret = -ENOMEM;
		goto out;
	}
	pcm->substream = substream;
	atomic_set(&pcm->running, 0);
	hrtimer_setup(&pcm->timer, q6apm_voice_pcm_elapsed, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL_SOFT);
	substream->runtime->hw = q6apm_voice_pcm_hardware;
	ret = snd_pcm_hw_constraint_integer(substream->runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		kfree(pcm);
		goto out;
	}
	substream->runtime->private_data = pcm;
	return 0;
out:
	module_put(THIS_MODULE);
	return ret;
}

static int q6apm_voice_pcm_close(struct snd_pcm_substream *substream)
{
	struct q6apm_voice_pcm *pcm = substream->runtime->private_data;

	atomic_set(&pcm->running, 0);
	hrtimer_cancel(&pcm->timer);
	kfree(pcm);
	module_put(THIS_MODULE);
	return 0;
}

static int q6apm_voice_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct q6apm_voice_pcm *pcm = runtime->private_data;

	pcm->base_time = hrtimer_cb_get_time(&pcm->timer);
	pcm->paused_frames = 0;
	pcm->period_time = ns_to_ktime(DIV_ROUND_UP_ULL((u64)runtime->period_size *
						     NSEC_PER_SEC, runtime->rate));
	/* No modem capture/export is provided by this policy endpoint. */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		memset(runtime->dma_area, 0, runtime->dma_bytes);
	return 0;
}

static int q6apm_voice_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct q6apm_voice_pcm *pcm = substream->runtime->private_data;
	ktime_t elapsed;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pcm->paused_frames = 0;
		pcm->base_time = hrtimer_cb_get_time(&pcm->timer);
		atomic_set(&pcm->running, 1);
		hrtimer_start(&pcm->timer, pcm->period_time, HRTIMER_MODE_REL_SOFT);
		return 0;
	case SNDRV_PCM_TRIGGER_RESUME:
		elapsed = ns_to_ktime(mul_u64_u32_div(pcm->paused_frames, NSEC_PER_SEC,
						      substream->runtime->rate));
		pcm->base_time = ktime_sub(hrtimer_cb_get_time(&pcm->timer), elapsed);
		atomic_set(&pcm->running, 1);
		hrtimer_start(&pcm->timer, pcm->period_time, HRTIMER_MODE_REL_SOFT);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		pcm->paused_frames = div_u64(ktime_us_delta(hrtimer_cb_get_time(&pcm->timer),
							    pcm->base_time) *
					     substream->runtime->rate, USEC_PER_SEC);
		atomic_set(&pcm->running, 0);
		/* trigger may hold the PCM stream lock; never wait for callback. */
		hrtimer_try_to_cancel(&pcm->timer);
		return 0;
	default:
		return -EINVAL;
	}
}

static int q6apm_voice_pcm_sync_stop(struct snd_pcm_substream *substream)
{
	struct q6apm_voice_pcm *pcm = substream->runtime->private_data;

	hrtimer_cancel(&pcm->timer);
	return 0;
}

static snd_pcm_uframes_t q6apm_voice_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct q6apm_voice_pcm *pcm = runtime->private_data;
	u64 frames;
	u32 position;

	frames = atomic_read(&pcm->running) ?
		div_u64(ktime_us_delta(hrtimer_cb_get_time(&pcm->timer),
				       pcm->base_time) * runtime->rate, USEC_PER_SEC) :
		pcm->paused_frames;
	div_u64_rem(frames, runtime->buffer_size, &position);
	return position;
}

static const struct snd_pcm_ops q6apm_voice_pcm_ops = {
	.open = q6apm_voice_pcm_open,
	.close = q6apm_voice_pcm_close,
	.prepare = q6apm_voice_pcm_prepare,
	.trigger = q6apm_voice_pcm_trigger,
	.sync_stop = q6apm_voice_pcm_sync_stop,
	.pointer = q6apm_voice_pcm_pointer,
};

int q6apm_voice_pcm_create(struct snd_soc_component *component)
{
	struct snd_pcm *pcm;
	int ret;

	/* IDs 14/15 remain private physical links; topology FEs use 0..2. */
	ret = snd_pcm_new(component->card->snd_card, "Voice Policy", 13, 1, 1, &pcm);
	if (ret)
		return ret;
	strscpy(pcm->name, "Voice Policy", sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &q6apm_voice_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &q6apm_voice_pcm_ops);
	ret = snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_VMALLOC, NULL,
					     64 * 1024, 256 * 1024);
	if (ret)
		snd_device_free(component->card->snd_card, pcm);
	return ret;
}
