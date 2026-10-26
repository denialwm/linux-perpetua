// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek Dimensity display processor: routing and frame sync
 *
 * Each pipe runs through two subsystems. In the OVL subsystem, OVL0 blends
 * the stripe and hands it over an asynchronous bridge to the display
 * subsystem, where it skips the picture quality engines on its way to the
 * DSC encoder and the DSI link. Crossbars in each subsystem's config block
 * make these connections, and a mutex in each subsystem starts its engines
 * together on every frame of the link.
 */

#include <linux/bits.h>
#include <linux/io.h>

#include <drm/drm_modes.h>

#include "dpu.h"

#define DPU_MUTEX			0x1000

/* OVL subsystem: OVL0 straight to its outgoing bridge. */
#define OVLSYS_BYPASS_MUX_SHADOW	0xf00
#define  OVLSYS_BYPASS_MUX_SHADOW_ALL	0x00ff0000
#define OVLSYS_OVL_CON			0xf08
#define  OVL0_TO_BLEND_CROSSBAR0	BIT(0)
#define OVL_BG_CROSSBAR0_MOUT_EN	0xf10
#define OVL_BG_CROSSBAR1_MOUT_EN	0xf14
#define OVL_BLEND_CROSSBAR0_MOUT_EN	0xf38
#define  OVL0_TO_DLO_RELAY3		BIT(0)
#define OVL_BLEND_CROSSBAR2_MOUT_EN	0xf40

/* Display subsystem: incoming bridge, past the PQ engines, to DSC0 and DSI0. */
#define DISP_BYPASS_MUX_SHADOW		0xc30
#define  DISP_BYPASS_MUX_SHADOW_ALL	0x00ff0001
#define COMP_OUT_CROSSBAR1_MOUT_EN	0xea4
#define  DSC0_TO_MERGE_OUT_CROSSBAR0	BIT(0)
#define MERGE_OUT_CROSSBAR0_MOUT_EN	0xf0c
#define  COMP_OUT_CROSSBAR0_TO_DSI0	BIT(0)
#define PANEL_COMP_OUT_CROSSBAR0_MOUT_EN 0xf48
#define PANEL_COMP_OUT_CROSSBAR1_MOUT_EN 0xf4c
#define  PQ_OUT_CROSSBAR1_TO_DSC0	BIT(1)
#define DSC0_SEL_IN			0xf60
#define  DSC0_FROM_PANEL_COMP_OUT_CROSSBAR1 1
#define PQ_IN_CROSSBAR0_MOUT_EN		0xf74
#define  DLI_RELAY0_TO_PQ_OUT_CROSSBAR3	BIT(2)
#define PQ_OUT_CROSSBAR0_MOUT_EN	0xf98
#define PQ_OUT_CROSSBAR3_MOUT_EN	0xfa4
#define  PQ_IN_CROSSBAR2_TO_PANEL_COMP_OUT_CROSSBAR1 BIT(1)

/* Mutexes: mutex 0 of each subsystem. */
#define MUTEX_EN			(DPU_MUTEX + 0x020)
#define MUTEX_SOF			(DPU_MUTEX + 0x02c)
#define MUTEX_MOD0			(DPU_MUTEX + 0x030)
#define MUTEX_MOD1			(DPU_MUTEX + 0x034)
#define  OVLSYS_MOD_OVL0		BIT(0)
#define  OVLSYS_MOD_DLO_ASYNC3		BIT(15)
#define  DISP_MOD_DLI_ASYNC0		BIT(9)
#define  DISP_MOD_DSC0			BIT(18)
#define  DISP_MOD_DSI0			BIT(20)

/* Start and end of frame come from the link: source 1 in its own subsystem. */
#define MUTEX_SOF_LINK(src)		((src) | (src) << 6)

/*
 * The OVL subsystem of the second pipe takes its frame sync from source 2,
 * as the bootloader sets it up.
 */
static const u32 dpu_ovlsys_sof[DPU_MAX_PIPES] = {
	MUTEX_SOF_LINK(1), MUTEX_SOF_LINK(2),
};

void dpu_route_enable(struct dpu *dpu, const struct drm_display_mode *mode)
{
	unsigned int width = mode->hdisplay / dpu->soc->num_pipes;
	unsigned int pipe;

	for (pipe = 0; pipe < dpu->soc->num_pipes; pipe++) {
		void __iomem *ovlsys = dpu->ovlsys[pipe];
		void __iomem *disp = dpu->dispsys[pipe];

		dpu_ovl_config(dpu->ovl[pipe], width, mode->vdisplay);

		writel(OVLSYS_BYPASS_MUX_SHADOW_ALL,
		       ovlsys + OVLSYS_BYPASS_MUX_SHADOW);
		writel(OVL0_TO_BLEND_CROSSBAR0, ovlsys + OVLSYS_OVL_CON);
		writel(0, ovlsys + OVL_BG_CROSSBAR0_MOUT_EN);
		writel(0, ovlsys + OVL_BG_CROSSBAR1_MOUT_EN);
		writel(OVL0_TO_DLO_RELAY3, ovlsys + OVL_BLEND_CROSSBAR0_MOUT_EN);
		writel(0, ovlsys + OVL_BLEND_CROSSBAR2_MOUT_EN);

		writel(DISP_BYPASS_MUX_SHADOW_ALL, disp + DISP_BYPASS_MUX_SHADOW);
		writel(DLI_RELAY0_TO_PQ_OUT_CROSSBAR3,
		       disp + PQ_IN_CROSSBAR0_MOUT_EN);
		writel(0, disp + PQ_OUT_CROSSBAR0_MOUT_EN);
		writel(PQ_IN_CROSSBAR2_TO_PANEL_COMP_OUT_CROSSBAR1,
		       disp + PQ_OUT_CROSSBAR3_MOUT_EN);
		writel(0, disp + PANEL_COMP_OUT_CROSSBAR0_MOUT_EN);
		writel(PQ_OUT_CROSSBAR1_TO_DSC0,
		       disp + PANEL_COMP_OUT_CROSSBAR1_MOUT_EN);
		writel(DSC0_FROM_PANEL_COMP_OUT_CROSSBAR1, disp + DSC0_SEL_IN);
		writel(DSC0_TO_MERGE_OUT_CROSSBAR0,
		       disp + COMP_OUT_CROSSBAR1_MOUT_EN);
		writel(COMP_OUT_CROSSBAR0_TO_DSI0,
		       disp + MERGE_OUT_CROSSBAR0_MOUT_EN);

		writel(OVLSYS_MOD_OVL0 | OVLSYS_MOD_DLO_ASYNC3,
		       ovlsys + MUTEX_MOD0);
		writel(0, ovlsys + MUTEX_MOD1);
		writel(dpu_ovlsys_sof[pipe], ovlsys + MUTEX_SOF);
		writel(1, ovlsys + MUTEX_EN);

		writel(DISP_MOD_DLI_ASYNC0 | DISP_MOD_DSC0 | DISP_MOD_DSI0,
		       disp + MUTEX_MOD0);
		writel(0, disp + MUTEX_MOD1);
		writel(MUTEX_SOF_LINK(1), disp + MUTEX_SOF);
		writel(1, disp + MUTEX_EN);
	}
}

/* The links are stopped: release the engines from the frame sync. */
void dpu_route_disable(struct dpu *dpu)
{
	unsigned int pipe;

	for (pipe = 0; pipe < dpu->soc->num_pipes; pipe++) {
		writel(0, dpu->dispsys[pipe] + MUTEX_EN);
		writel(0, dpu->ovlsys[pipe] + MUTEX_EN);
	}
}
