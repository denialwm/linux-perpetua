// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek Dimensity display processor: DSC encoders
 *
 * Each pipe compresses its stripe of the frame on its own, in front of its
 * DSI link, so every encoder takes the same configuration.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/kernel.h>

#include <drm/display/drm_dsc.h>

#include "dpu.h"

#define DSC_CON				0x000
#define  DSC_CON_EN			BIT(0)
#define  DSC_CON_V1			BIT(7)
#define  DSC_CON_UFOE_SEL		BIT(16)
#define DSC_SPR				0x014
#define DSC_PIC_W			0x018
#define  DSC_PIC_W_WIDTH		GENMASK(15, 0)
#define  DSC_PIC_W_GROUPS_M1		GENMASK(31, 16)
#define DSC_PIC_H			0x01c
#define  DSC_PIC_H_HEIGHT_M1		GENMASK(15, 0)
#define  DSC_PIC_H_EXT_HEIGHT_M1	GENMASK(31, 16)
#define DSC_SLICE_W			0x020
#define  DSC_SLICE_W_WIDTH		GENMASK(15, 0)
#define  DSC_SLICE_W_GROUPS_M1		GENMASK(31, 16)
#define DSC_SLICE_H			0x024
#define  DSC_SLICE_H_HEIGHT_M1		GENMASK(15, 0)
#define  DSC_SLICE_H_COUNT_M1		GENMASK(29, 16)
#define  DSC_SLICE_H_WIDTH_MOD3		GENMASK(31, 30)
#define DSC_CHUNK_SIZE			0x028
#define DSC_BUF_SIZE			0x02c
#define DSC_MODE			0x030
#define  DSC_MODE_SLICE_MODE		BIT(0)
#define  DSC_MODE_MASK			GENMASK(15, 0)
#define DSC_CFG				0x034
#define  DSC_CFG_DEFAULT		0x22
#define DSC_PAD				0x038
#define DSC_ENC_WIDTH			0x03c
#define  DSC_ENC_WIDTH_SLICE		GENMASK(15, 0)
#define  DSC_ENC_WIDTH_PIC		GENMASK(31, 16)
#define DSC_PPS(n)			(0x080 + 4 * (n))
#define DSC_SHADOW			0x200
#define  DSC_SHADOW_VERSION		GENMASK(6, 5)
#define  DSC_SHADOW_VERSION_1		1

static void dsc_update_bits(void __iomem *dsc, u32 reg, u32 mask, u32 val)
{
	writel((readl(dsc + reg) & ~mask) | (val & mask), dsc + reg);
}

static u32 dsc_range(const struct drm_dsc_rc_range_parameters *range)
{
	return range->range_min_qp | range->range_max_qp << 5 |
	       (range->range_bpg_offset & 0x3f) << 10;
}

/* The picture parameter set, as the encoder takes it. */
static void dsc_pps(void __iomem *dsc, const struct drm_dsc_config *cfg)
{
	unsigned int i;
	u32 val;

	writel(cfg->line_buf_depth | cfg->bits_per_component << 4 |
	       cfg->bits_per_pixel << 8 | cfg->convert_rgb << 18 |
	       cfg->block_pred_enable << 19, dsc + DSC_PPS(0));
	writel(cfg->initial_xmit_delay | cfg->initial_dec_delay << 16,
	       dsc + DSC_PPS(1));
	writel(cfg->initial_scale_value | cfg->scale_increment_interval << 16,
	       dsc + DSC_PPS(2));
	writel(cfg->scale_decrement_interval |
	       cfg->first_line_bpg_offset << 16, dsc + DSC_PPS(3));
	writel(cfg->nfl_bpg_offset | cfg->slice_bpg_offset << 16,
	       dsc + DSC_PPS(4));
	writel(cfg->initial_offset | cfg->final_offset << 16,
	       dsc + DSC_PPS(5));
	writel(cfg->flatness_min_qp | cfg->flatness_max_qp << 8 |
	       cfg->rc_model_size << 16, dsc + DSC_PPS(6));
	writel(cfg->rc_edge_factor | cfg->rc_quant_incr_limit0 << 8 |
	       cfg->rc_quant_incr_limit1 << 16 |
	       cfg->rc_tgt_offset_high << 24 | cfg->rc_tgt_offset_low << 28,
	       dsc + DSC_PPS(7));

	/* Four buffer thresholds per register, in units of 64 bits. */
	for (i = 0; i < DSC_NUM_BUF_RANGES - 1; i += 4) {
		unsigned int j;

		val = 0;
		for (j = 0; j < 4 && i + j < DSC_NUM_BUF_RANGES - 1; j++)
			val |= cfg->rc_buf_thresh[i + j] << (8 * j);
		writel(val, dsc + DSC_PPS(8 + i / 4));
	}

	/* Two ranges per register. */
	for (i = 0; i < DSC_NUM_BUF_RANGES; i += 2) {
		val = dsc_range(&cfg->rc_range_params[i]);
		if (i + 1 < DSC_NUM_BUF_RANGES)
			val |= dsc_range(&cfg->rc_range_params[i + 1]) << 16;
		writel(val, dsc + DSC_PPS(12 + i / 2));
	}
}

/*
 * Set the encoders up for the panel's DSC stream and turn them on. The
 * engines have no shadow registers in use: writes take effect at once, so
 * this runs while the links are stopped.
 */
void dpu_dsc_enable(struct dpu *dpu, const struct drm_dsc_config *cfg)
{
	unsigned int groups = DIV_ROUND_UP(cfg->slice_width, 3);
	unsigned int slices = DIV_ROUND_UP(cfg->pic_height, cfg->slice_height);
	unsigned int chunks = cfg->slice_chunk_size * cfg->slice_count;
	unsigned int pipe;

	for (pipe = 0; pipe < dpu->soc->num_pipes; pipe++) {
		void __iomem *dsc = dpu->dsc[pipe];

		writel(DSC_CON_V1 | DSC_CON_UFOE_SEL, dsc + DSC_CON);
		writel(0, dsc + DSC_SPR);
		writel(FIELD_PREP(DSC_ENC_WIDTH_PIC, cfg->pic_width) |
		       FIELD_PREP(DSC_ENC_WIDTH_SLICE, cfg->slice_width),
		       dsc + DSC_ENC_WIDTH);
		writel(FIELD_PREP(DSC_PIC_W_GROUPS_M1,
				  DIV_ROUND_UP(cfg->pic_width, 3) - 1) |
		       FIELD_PREP(DSC_PIC_W_WIDTH, cfg->pic_width),
		       dsc + DSC_PIC_W);
		writel(FIELD_PREP(DSC_PIC_H_EXT_HEIGHT_M1,
				  slices * cfg->slice_height - 1) |
		       FIELD_PREP(DSC_PIC_H_HEIGHT_M1, cfg->pic_height - 1),
		       dsc + DSC_PIC_H);
		writel(FIELD_PREP(DSC_SLICE_W_GROUPS_M1, groups - 1) |
		       FIELD_PREP(DSC_SLICE_W_WIDTH, cfg->slice_width),
		       dsc + DSC_SLICE_W);
		writel(FIELD_PREP(DSC_SLICE_H_WIDTH_MOD3, cfg->slice_width % 3) |
		       FIELD_PREP(DSC_SLICE_H_COUNT_M1, slices - 1) |
		       FIELD_PREP(DSC_SLICE_H_HEIGHT_M1, cfg->slice_height - 1),
		       dsc + DSC_SLICE_H);
		writel(cfg->slice_chunk_size, dsc + DSC_CHUNK_SIZE);
		/* The slices of a line go out in whole 3-byte groups. */
		writel(roundup(chunks, 3) - chunks, dsc + DSC_PAD);
		writel(cfg->slice_chunk_size * cfg->slice_height,
		       dsc + DSC_BUF_SIZE);
		/* The DSI link buffers the stream: no initial delay lines. */
		dsc_update_bits(dsc, DSC_MODE, DSC_MODE_MASK,
				cfg->slice_count > 1 ? DSC_MODE_SLICE_MODE : 0);
		writel(DSC_CFG_DEFAULT, dsc + DSC_CFG);
		dsc_update_bits(dsc, DSC_SHADOW, DSC_SHADOW_VERSION,
				FIELD_PREP(DSC_SHADOW_VERSION,
					   DSC_SHADOW_VERSION_1));

		dsc_pps(dsc, cfg);

		dsc_update_bits(dsc, DSC_CON, DSC_CON_EN, DSC_CON_EN);
	}
}

void dpu_dsc_disable(struct dpu *dpu)
{
	unsigned int pipe;

	for (pipe = 0; pipe < dpu->soc->num_pipes; pipe++)
		dsc_update_bits(dpu->dsc[pipe], DSC_CON, DSC_CON_EN, 0);
}
