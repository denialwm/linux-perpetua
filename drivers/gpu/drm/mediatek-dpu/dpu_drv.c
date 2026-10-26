// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek Dimensity display processor
 *
 * The bootloader powers the display up, programs the whole pipeline and
 * leaves it scanning out its splash buffer. This driver takes that running
 * pipeline over and flips its overlay engines between framebuffers.
 */

#include <linux/aperture.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include "dpu.h"

/*
 * Lenovo Idea Tab Pro: the 144 Hz mode its bootloader sets up for the
 * Tianma NT36532 panel. It moves to the panel driver once this driver
 * drives DSI itself.
 */
static const struct drm_display_mode dpu_boot_mode = {
	DRM_MODE("2944x1840", DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
		 890618, 2944, 2944 + 36, 2944 + 36 + 6, 2944 + 36 + 6 + 31, 0,
		 1840, 1840 + 26, 1840 + 26 + 2, 1840 + 26 + 2 + 182, 0, 0),
	.width_mm = 274,
	.height_mm = 171,
};

static const u32 dpu_formats[] = {
	DRM_FORMAT_XRGB8888,
};

/* Take over a finished update: the frame that starts next shows it. */
static void dpu_apply(struct dpu *dpu)
{
	unsigned int pipe;

	for (pipe = 0; pipe < dpu->soc->num_pipes; pipe++)
		dpu_ovl_scanout(dpu->ovl[pipe], &dpu->pending, pipe);

	dpu->has_pending = false;
	dpu->armed = true;
	dpu->armed_event = dpu->pending_event;
	dpu->pending_event = NULL;
}

static irqreturn_t dpu_irq(int irq, void *data)
{
	struct dpu *dpu = data;
	u32 status = dpu_ovl_irq_status(dpu->ovl[0]);
	bool done = dpu_ovl_frame_done(status);
	bool start = dpu_ovl_frame_start(status);

	if (!status)
		return IRQ_NONE;

	if (start)
		drm_crtc_handle_vblank(&dpu->crtc);

	spin_lock(&dpu->drm.event_lock);

	/* An update armed before this frame started is on screen now. */
	if (start && dpu->armed) {
		if (dpu->armed_event)
			drm_crtc_send_vblank_event(&dpu->crtc, dpu->armed_event);
		dpu->armed_event = NULL;
		dpu->armed = false;
		drm_crtc_vblank_put(&dpu->crtc);
		wake_up(&dpu->applied);
	}

	if (done && dpu->has_pending) {
		/*
		 * Both bits together mean the next frame already started:
		 * the update lands mid-frame, so it completes one frame
		 * later.
		 */
		if (start)
			dpu->late_updates++;
		dpu_apply(dpu);
	}

	spin_unlock(&dpu->drm.event_lock);

	return IRQ_HANDLED;
}

static int dpu_plane_atomic_check(struct drm_plane *plane,
				  struct drm_atomic_commit *state)
{
	struct drm_plane_state *plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state = NULL;

	if (plane_state->crtc)
		crtc_state = drm_atomic_get_new_crtc_state(state,
							   plane_state->crtc);

	/* The overlays cannot scale or position: the plane fills the CRTC. */
	return drm_atomic_helper_check_plane_state(plane_state, crtc_state,
						   DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   false, true);
}

static void dpu_plane_atomic_update(struct drm_plane *plane,
				    struct drm_atomic_commit *state)
{
	struct dpu *dpu = to_dpu(plane->dev);
	struct drm_plane_state *plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = plane_state->fb;
	unsigned int width = drm_rect_width(&plane_state->dst);
	unsigned int stripe = width / dpu->soc->num_pipes;
	dma_addr_t addr = drm_fb_dma_get_gem_addr(fb, plane_state, 0);
	unsigned int pipe;

	dpu->next.enabled = plane_state->visible;
	dpu->next.pitch = fb->pitches[0];
	for (pipe = 0; pipe < dpu->soc->num_pipes; pipe++) {
		unsigned int x = pipe ? pipe * stripe - dpu->soc->overlap : 0;

		dpu->next.addr[pipe] = addr + x * fb->format->cpp[0];
	}
}

static void dpu_plane_atomic_disable(struct drm_plane *plane,
				     struct drm_atomic_commit *state)
{
	struct dpu *dpu = to_dpu(plane->dev);

	dpu->next.enabled = false;
}

static const struct drm_plane_helper_funcs dpu_plane_helper_funcs = {
	.atomic_check = dpu_plane_atomic_check,
	.atomic_update = dpu_plane_atomic_update,
	.atomic_disable = dpu_plane_atomic_disable,
};

static const struct drm_plane_funcs dpu_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

/* Queue @dpu->next for the next frame; the caller holds a vblank reference. */
static void dpu_queue(struct dpu *dpu, struct drm_pending_vblank_event *event)
{
	unsigned long flags;

	spin_lock_irqsave(&dpu->drm.event_lock, flags);
	if (dpu->has_pending) {
		/* Replaced before it reached the hardware. */
		if (dpu->pending_event)
			drm_crtc_send_vblank_event(&dpu->crtc,
						   dpu->pending_event);
		drm_crtc_vblank_put(&dpu->crtc);
	}
	dpu->pending = dpu->next;
	dpu->pending_event = event;
	dpu->has_pending = true;
	spin_unlock_irqrestore(&dpu->drm.event_lock, flags);
}

static void dpu_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_atomic_commit *state)
{
	struct dpu *dpu = to_dpu(crtc->dev);
	struct drm_pending_vblank_event *event = crtc->state->event;

	crtc->state->event = NULL;

	if (!crtc->state->active || drm_crtc_vblank_get(crtc)) {
		if (event) {
			spin_lock_irq(&crtc->dev->event_lock);
			drm_crtc_send_vblank_event(crtc, event);
			spin_unlock_irq(&crtc->dev->event_lock);
		}
		return;
	}

	dpu_queue(dpu, event);
}

static void dpu_crtc_atomic_enable(struct drm_crtc *crtc,
				   struct drm_atomic_commit *state)
{
	drm_crtc_vblank_on(crtc);
}

static void dpu_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_atomic_commit *state)
{
	struct dpu *dpu = to_dpu(crtc->dev);

	/* Scan out nothing: the panel shows the background colour. */
	if (!drm_crtc_vblank_get(crtc)) {
		dpu->next.enabled = false;
		dpu_queue(dpu, NULL);
		if (!wait_event_timeout(dpu->applied, !dpu->has_pending &&
					!dpu->armed, msecs_to_jiffies(100)))
			drm_warn(crtc->dev, "overlays did not stop\n");
	}
	drm_crtc_vblank_off(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static int dpu_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct dpu *dpu = to_dpu(crtc->dev);

	dpu_ovl_irq_enable(dpu->ovl[0], true);
	return 0;
}

static void dpu_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct dpu *dpu = to_dpu(crtc->dev);

	dpu_ovl_irq_enable(dpu->ovl[0], false);
}

static enum drm_mode_status dpu_crtc_mode_valid(struct drm_crtc *crtc,
						const struct drm_display_mode *mode)
{
	struct dpu *dpu = to_dpu(crtc->dev);

	return drm_mode_equal(mode, &dpu->mode) ? MODE_OK : MODE_BAD;
}

static const struct drm_crtc_helper_funcs dpu_crtc_helper_funcs = {
	.mode_valid = dpu_crtc_mode_valid,
	.atomic_flush = dpu_crtc_atomic_flush,
	.atomic_enable = dpu_crtc_atomic_enable,
	.atomic_disable = dpu_crtc_atomic_disable,
};

static const struct drm_crtc_funcs dpu_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = dpu_crtc_enable_vblank,
	.disable_vblank = dpu_crtc_disable_vblank,
};

static int dpu_connector_get_modes(struct drm_connector *connector)
{
	struct dpu *dpu = to_dpu(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, &dpu->mode);
}

static const struct drm_connector_helper_funcs dpu_connector_helper_funcs = {
	.get_modes = dpu_connector_get_modes,
};

static const struct drm_connector_funcs dpu_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs dpu_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/* Enable the CRTC before its planes: updates queue only on a live CRTC. */
static const struct drm_mode_config_helper_funcs dpu_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

static int dpu_kms_init(struct dpu *dpu)
{
	struct drm_device *drm = &dpu->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = dpu->mode.hdisplay;
	drm->mode_config.max_width = dpu->mode.hdisplay;
	drm->mode_config.min_height = dpu->mode.vdisplay;
	drm->mode_config.max_height = dpu->mode.vdisplay;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &dpu_mode_config_funcs;
	drm->mode_config.helper_private = &dpu_mode_config_helpers;

	ret = drm_universal_plane_init(drm, &dpu->plane, 0, &dpu_plane_funcs,
				       dpu_formats, ARRAY_SIZE(dpu_formats),
				       NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&dpu->plane, &dpu_plane_helper_funcs);

	ret = drm_crtc_init_with_planes(drm, &dpu->crtc, &dpu->plane, NULL,
					&dpu_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&dpu->crtc, &dpu_crtc_helper_funcs);

	ret = drm_simple_encoder_init(drm, &dpu->encoder,
				      DRM_MODE_ENCODER_DSI);
	if (ret)
		return ret;
	dpu->encoder.possible_crtcs = drm_crtc_mask(&dpu->crtc);

	ret = drm_connector_init(drm, &dpu->connector, &dpu_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret)
		return ret;
	drm_connector_helper_add(&dpu->connector, &dpu_connector_helper_funcs);

	ret = drm_connector_attach_encoder(&dpu->connector, &dpu->encoder);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	return 0;
}

/*
 * Check that the pipes run the split this driver assumes: each OVL as
 * wide as its stripe plus the overlap, the second one starting @overlap
 * pixels left of the middle of the bootloader's frame.
 */
static int dpu_check_boot_pipeline(struct dpu *dpu)
{
	unsigned int width = dpu->mode.hdisplay / dpu->soc->num_pipes;
	unsigned int ovl_width = width + dpu->soc->overlap;
	dma_addr_t base = dpu_ovl_addr(dpu->ovl[0]);
	unsigned int pipe;

	for (pipe = 0; pipe < dpu->soc->num_pipes; pipe++) {
		u32 have = dpu_ovl_width(dpu->ovl[pipe]);
		unsigned int x = pipe ? pipe * width - dpu->soc->overlap : 0;
		dma_addr_t addr = dpu_ovl_addr(dpu->ovl[pipe]);

		if (have != ovl_width || addr != base + x * 4) {
			drm_err(&dpu->drm,
				"pipe %u: %u pixels at %pad, expected %u at %pad + %u\n",
				pipe, have, &addr, ovl_width, &base, x * 4);
			return -ENODEV;
		}
	}

	return 0;
}

DEFINE_DRM_GEM_DMA_FOPS(dpu_fops);

static const struct drm_driver dpu_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.fops = &dpu_fops,
	.name = "mediatek-dpu",
	.desc = "MediaTek Dimensity display processor",
	.major = 1,
	.minor = 0,
};

static int dpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dpu *dpu;
	unsigned int pipe;
	int ret;

	dpu = devm_drm_dev_alloc(dev, &dpu_driver, struct dpu, drm);
	if (IS_ERR(dpu))
		return PTR_ERR(dpu);

	dpu->soc = of_device_get_match_data(dev);
	dpu->mode = dpu_boot_mode;
	init_waitqueue_head(&dpu->applied);
	platform_set_drvdata(pdev, dpu);

	for (pipe = 0; pipe < dpu->soc->num_pipes; pipe++) {
		char name[8];

		snprintf(name, sizeof(name), "ovl%u", pipe);
		dpu->ovl[pipe] = devm_platform_ioremap_resource_byname(pdev,
								       name);
		if (IS_ERR(dpu->ovl[pipe]))
			return PTR_ERR(dpu->ovl[pipe]);
	}

	ret = dpu_check_boot_pipeline(dpu);
	if (ret)
		return ret;

	/*
	 * The overlays read physical memory: the bootloader leaves their
	 * ports outside the IOMMU. An address takes 36 bits.
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;

	ret = dpu_kms_init(dpu);
	if (ret)
		return ret;

	dpu_ovl_irq_enable(dpu->ovl[0], false);
	dpu->irq = platform_get_irq_byname(pdev, "ovl0");
	if (dpu->irq < 0)
		return dpu->irq;
	ret = devm_request_irq(dev, dpu->irq, dpu_irq, 0, dev_name(dev), dpu);
	if (ret)
		return ret;

	ret = aperture_remove_all_conflicting_devices(dpu_driver.name);
	if (ret)
		return ret;

	ret = drm_dev_register(&dpu->drm, 0);
	if (ret)
		return ret;

	drm_client_setup(&dpu->drm, NULL);

	return 0;
}

static void dpu_remove(struct platform_device *pdev)
{
	struct dpu *dpu = platform_get_drvdata(pdev);

	drm_dev_unplug(&dpu->drm);
	drm_atomic_helper_shutdown(&dpu->drm);
}

static void dpu_shutdown(struct platform_device *pdev)
{
	struct dpu *dpu = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&dpu->drm);
}

static const struct dpu_soc mt6897_dpu = {
	.num_pipes = 2,
	.overlap = 11,
};

static const struct of_device_id dpu_of_match[] = {
	{ .compatible = "mediatek,mt6897-dpu", .data = &mt6897_dpu },
	{ }
};
MODULE_DEVICE_TABLE(of, dpu_of_match);

static struct platform_driver dpu_platform_driver = {
	.probe = dpu_probe,
	.remove = dpu_remove,
	.shutdown = dpu_shutdown,
	.driver = {
		.name = "mediatek-dpu",
		.of_match_table = dpu_of_match,
	},
};
module_platform_driver(dpu_platform_driver);

MODULE_DESCRIPTION("MediaTek Dimensity display processor");
MODULE_LICENSE("GPL");
