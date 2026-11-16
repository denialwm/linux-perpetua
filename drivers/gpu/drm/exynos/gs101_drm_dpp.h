/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google gs101 DPP (Display Pre-Processor) channel, as used by the gs101
 * DECON driver.
 */

#ifndef _GS101_DRM_DPP_H_
#define _GS101_DRM_DPP_H_

#include <linux/types.h>

struct device;
struct device_node;
struct drm_framebuffer;
struct gs101_dpp;

struct gs101_dpp *gs101_dpp_get(struct device *consumer,
				struct device_node *np);

int gs101_dpp_check(struct gs101_dpp *dpp, struct drm_framebuffer *fb,
		    unsigned int w, unsigned int h);
void gs101_dpp_update(struct gs101_dpp *dpp, struct drm_framebuffer *fb,
		      unsigned int x, unsigned int y,
		      unsigned int w, unsigned int h);
void gs101_dpp_disable(struct gs101_dpp *dpp);

#endif
