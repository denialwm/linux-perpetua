// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek Dimensity display processor: overlay engine (OVL)
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/kernel.h>

#include "dpu.h"

#define OVL_INTEN			0x004
#define OVL_INTSTA			0x008
#define  OVL_INT_FRAME_DONE		BIT(1)
#define  OVL_INT_FRAME_START		BIT(14)
#define OVL_ROI_SIZE			0x020
#define  OVL_ROI_WIDTH			GENMASK(12, 0)
#define  OVL_ROI_HEIGHT			GENMASK(28, 16)
#define OVL_SRC_CON			0x02c
#define  OVL_SRC_CON_LAYER_EN(n)	BIT(n)
#define OVL_L_PITCH(n)			(0x044 + 0x20 * (n))
#define  OVL_L_PITCH_SRC		GENMASK(15, 0)
#define  OVL_L_PITCH_CONST_BLEND	BIT(28)
#define OVL_L_ADDR(n)			(0xf40 + 0x20 * (n))
#define OVL_L_ADDR_MSB(n)		(0xf4c + 0x20 * (n))
#define  OVL_L_ADDR_MSB_BODY		GENMASK(3, 0)

static void ovl_update_bits(void __iomem *ovl, u32 reg, u32 mask, u32 val)
{
	writel((readl(ovl + reg) & ~mask) | (val & mask), ovl + reg);
}

/*
 * Point layer 0 at this pipe's stripe of the frame, or turn the layers
 * off. Layer 0 keeps the format that the bootloader set up and blends with
 * its constant alpha, ignoring the unused byte of each pixel.
 */
void dpu_ovl_scanout(void __iomem *ovl, const struct dpu_scanout *scanout,
		     unsigned int pipe)
{
	if (!scanout->enabled) {
		writel(0, ovl + OVL_SRC_CON);
		return;
	}

	writel(lower_32_bits(scanout->addr[pipe]), ovl + OVL_L_ADDR(0));
	ovl_update_bits(ovl, OVL_L_ADDR_MSB(0), OVL_L_ADDR_MSB_BODY,
			upper_32_bits(scanout->addr[pipe]));
	ovl_update_bits(ovl, OVL_L_PITCH(0),
			OVL_L_PITCH_SRC | OVL_L_PITCH_CONST_BLEND,
			FIELD_PREP(OVL_L_PITCH_SRC, scanout->pitch) |
			OVL_L_PITCH_CONST_BLEND);
	writel(OVL_SRC_CON_LAYER_EN(0), ovl + OVL_SRC_CON);
}

void dpu_ovl_irq_enable(void __iomem *ovl, bool enable)
{
	writel(enable ? OVL_INT_FRAME_DONE | OVL_INT_FRAME_START : 0,
	       ovl + OVL_INTEN);
}

/* Read and acknowledge the pending interrupts. */
u32 dpu_ovl_irq_status(void __iomem *ovl)
{
	u32 status = readl(ovl + OVL_INTSTA);

	/* Writing 0 clears a bit. */
	if (status)
		writel(~status, ovl + OVL_INTSTA);

	return status & readl(ovl + OVL_INTEN);
}

bool dpu_ovl_frame_done(u32 status)
{
	return status & OVL_INT_FRAME_DONE;
}

bool dpu_ovl_frame_start(u32 status)
{
	return status & OVL_INT_FRAME_START;
}

u32 dpu_ovl_width(void __iomem *ovl)
{
	return FIELD_GET(OVL_ROI_WIDTH, readl(ovl + OVL_ROI_SIZE));
}

u32 dpu_ovl_height(void __iomem *ovl)
{
	return FIELD_GET(OVL_ROI_HEIGHT, readl(ovl + OVL_ROI_SIZE));
}

dma_addr_t dpu_ovl_addr(void __iomem *ovl)
{
	return (u64)FIELD_GET(OVL_L_ADDR_MSB_BODY,
			      readl(ovl + OVL_L_ADDR_MSB(0))) << 32 |
	       readl(ovl + OVL_L_ADDR(0));
}
