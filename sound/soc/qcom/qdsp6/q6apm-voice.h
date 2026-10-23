/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __Q6APM_VOICE_H__
#define __Q6APM_VOICE_H__

#include <linux/types.h>

struct snd_soc_component;
struct q6apm_voice;

struct q6apm_voice *q6apm_voice_create(struct snd_soc_component *component);
/* Caller holds the card PCM mutex, as in the ASoC PCM/compress open paths. */
bool q6apm_voice_pcm_allowed(struct q6apm_voice *voice);
bool q6apm_voice_backend_allowed(struct q6apm_voice *voice);
int q6apm_voice_pcm_create(struct snd_soc_component *component);
void q6apm_voice_free(struct q6apm_voice *voice);

#endif
