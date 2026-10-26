/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek Dimensity display processor
 */

#ifndef __MEDIATEK_DPU_H__
#define __MEDIATEK_DPU_H__

#include <linux/types.h>
#include <linux/wait.h>

#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>
#include <drm/drm_modes.h>
#include <drm/drm_plane.h>

#define DPU_MAX_PIPES	2

/*
 * A frame is split into one vertical stripe per pipe. Neighbouring stripes
 * overlap by @overlap pixels on each side of the split.
 */
struct dpu_soc {
	unsigned int num_pipes;
	unsigned int overlap;
};

/* What the OVLs scan out, as one update for all pipes. */
struct dpu_scanout {
	bool enabled;
	dma_addr_t addr[DPU_MAX_PIPES];
	u32 pitch;
};

struct dpu {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;

	const struct dpu_soc *soc;
	void __iomem *ovl[DPU_MAX_PIPES];
	int irq;

	/* The mode the panel runs, until the panel has a driver. */
	struct drm_display_mode mode;

	/*
	 * The OVLs have no shadow registers: a write takes effect at once,
	 * mid-frame. Updates wait in @pending until the frame-done
	 * interrupt, which leaves a short gap before the next frame starts,
	 * and complete at that frame's start. The DRM event lock guards
	 * them.
	 */
	struct dpu_scanout next;
	struct dpu_scanout pending;
	bool has_pending;
	struct drm_pending_vblank_event *pending_event;
	bool armed;
	struct drm_pending_vblank_event *armed_event;
	wait_queue_head_t applied;
	unsigned int late_updates;
};

static inline struct dpu *to_dpu(struct drm_device *drm)
{
	return container_of(drm, struct dpu, drm);
}

void dpu_ovl_scanout(void __iomem *ovl, const struct dpu_scanout *scanout,
		     unsigned int pipe);
void dpu_ovl_irq_enable(void __iomem *ovl, bool enable);
u32 dpu_ovl_irq_status(void __iomem *ovl);
bool dpu_ovl_frame_done(u32 status);
bool dpu_ovl_frame_start(u32 status);
u32 dpu_ovl_width(void __iomem *ovl);
dma_addr_t dpu_ovl_addr(void __iomem *ovl);

#endif /* __MEDIATEK_DPU_H__ */
