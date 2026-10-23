/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __Q6APM_HOSTLESS_H__
#define __Q6APM_HOSTLESS_H__

#include <linux/types.h>

struct device;
struct audioreach_graph_info;
struct q6apm_hostless;

/* Reserve the APM against topology graph opens; release after full cleanup. */
int q6apm_hostless_claim(struct device *dma_dev, const void *owner);
void q6apm_hostless_release(struct device *dma_dev, const void *owner);

/* dma_dev must be q6apm-dais; keep the descriptor alive until destroy succeeds. */
struct q6apm_hostless *q6apm_hostless_create(struct device *dma_dev,
					     const struct audioreach_graph_info *info);
bool q6apm_hostless_is_idle(struct q6apm_hostless *session);
int q6apm_hostless_open(struct q6apm_hostless *session);
/* The validated payload must declare exactly the caller-owned subgraphs. */
int q6apm_hostless_open_payload(struct q6apm_hostless *session,
				const void *data, size_t size);
int q6apm_hostless_set_payload(struct q6apm_hostless *session, u32 destination,
			       const void *data, size_t size);
int q6apm_hostless_set_cfg(struct q6apm_hostless *session, u32 destination,
			   u32 module, u32 parameter, const void *data, size_t size);
int q6apm_hostless_register_cfg(struct q6apm_hostless *session, const void *data, size_t size);
int q6apm_hostless_prepare(struct q6apm_hostless *session);
int q6apm_hostless_start(struct q6apm_hostless *session);
int q6apm_hostless_stop(struct q6apm_hostless *session);
/* Failure preserves the session, GPR port and any DSP-owned buffers for retry. */
int q6apm_hostless_destroy(struct q6apm_hostless *session);

#endif
