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
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_plane.h>

struct clk;
struct phy;

#define DPU_MAX_PIPES	2

/*
 * A frame is split into one vertical stripe per pipe. Neighbouring stripes
 * overlap by @overlap pixels on each side of the split.
 */
struct dpu_soc {
	unsigned int num_pipes;
	unsigned int overlap;

	/* The display clock and the link buffers, for their thresholds. */
	unsigned int mmsys_mhz;
	unsigned int dsi_buf_size;
	unsigned int dsi_sram_unit;
	unsigned int dsi_buf_unit;
	unsigned int dsi_urgent_high_us;
	unsigned int dsi_urgent_low_us;
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
	void __iomem *dsi[DPU_MAX_PIPES];
	struct clk *dsi_hs[DPU_MAX_PIPES];
	struct phy *dsi_phy[DPU_MAX_PIPES];
	bool dsi_powered;
	int irq;

	/* The frame size the bootloader set the pipes up for. */
	unsigned int width;
	unsigned int height;

	/* The panel on the DSI links; set once, when it attaches. */
	struct mipi_dsi_host dsi_host;
	struct mipi_dsi_device *dsi_device;
	struct drm_panel *panel;
	bool attached;

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

int dpu_bind(struct dpu *dpu, struct mipi_dsi_device *device);
void dpu_unbind(struct dpu *dpu);

int dpu_dsi_host_register(struct dpu *dpu);
int dpu_dsi_power_on(struct dpu *dpu);
void dpu_dsi_power_off(struct dpu *dpu);
void dpu_dsi_config(struct dpu *dpu, const struct drm_display_mode *mode);
int dpu_dsi_video_stop(struct dpu *dpu);
void dpu_dsi_video_start(struct dpu *dpu);

void dpu_ovl_scanout(void __iomem *ovl, const struct dpu_scanout *scanout,
		     unsigned int pipe);
void dpu_ovl_irq_enable(void __iomem *ovl, bool enable);
u32 dpu_ovl_irq_status(void __iomem *ovl);
bool dpu_ovl_frame_done(u32 status);
bool dpu_ovl_frame_start(u32 status);
u32 dpu_ovl_width(void __iomem *ovl);
u32 dpu_ovl_height(void __iomem *ovl);
dma_addr_t dpu_ovl_addr(void __iomem *ovl);

#endif /* __MEDIATEK_DPU_H__ */
