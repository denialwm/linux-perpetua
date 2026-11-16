/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022. Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2018, 2020 The Linux Foundation. All rights reserved.
 *
 * SM7250 ("lito"), SDE hardware revision 6.1.0 (SDE_HW_VER_610 downstream).
 *
 * Block offsets are the downstream lito-sde.dtsi values minus the 0x1000
 * "qcom,sde-off", i.e. relative to the "mdp" register region at 0xae01000.
 * Layout is the same as dpu_5_2_sm7150.h with the 6.x interrupt/CTL scheme
 * of dpu_6_0_sm8250.h.
 */

#ifndef _DPU_6_1_SM7250_H
#define _DPU_6_1_SM7250_H

static const struct dpu_caps sm7250_dpu_caps = {
	.max_mixer_width = DEFAULT_DPU_OUTPUT_LINE_WIDTH,	/* sde-mixer-linewidth 2560 */
	.max_mixer_blendstages = 0x9,				/* sde-mixer-blendstages */
	.has_src_split = true,					/* sde-has-src-split */
	.has_dim_layer = true,					/* sde-has-dim-layer */
	.has_idle_pc = true,					/* sde-has-idle-pc */
	.has_3d_merge = true,					/* sde-merge-3d-off */
	.max_linewidth = 2880,					/* sde-sspp-linewidth */
	.pixel_ram_size = DEFAULT_PIXEL_RAM_SIZE,
	.max_hdeci_exp = MAX_HORZ_DECIMATION,
	.max_vdeci_exp = MAX_VERT_DECIMATION,
};

static const struct dpu_mdp_cfg sm7250_mdp = {
	.name = "top_0",
	.base = 0x0, .len = 0x494,				/* sde-len */
	.clk_ctrls = {
		/* sde-sspp-clk-ctrl and sde-wb-clk-ctrl / sde-reg-dma-clk-ctrl */
		[DPU_CLK_CTRL_VIG0] = { .reg_off = 0x2ac, .bit_off = 0 },
		[DPU_CLK_CTRL_VIG1] = { .reg_off = 0x2b4, .bit_off = 0 },
		[DPU_CLK_CTRL_DMA0] = { .reg_off = 0x2ac, .bit_off = 8 },
		[DPU_CLK_CTRL_DMA1] = { .reg_off = 0x2b4, .bit_off = 8 },
		[DPU_CLK_CTRL_DMA2] = { .reg_off = 0x2c4, .bit_off = 8 },
		[DPU_CLK_CTRL_REG_DMA] = { .reg_off = 0x2bc, .bit_off = 20 },
		[DPU_CLK_CTRL_WB2] = { .reg_off = 0x2bc, .bit_off = 16 },
	},
};

/* sde-ctl-off 0x2000 0x2200 0x2400 0x2600, sde-ctl-size 0x1dc */
static const struct dpu_ctl_cfg sm7250_ctl[] = {
	{
		.name = "ctl_0", .id = CTL_0,
		.base = 0x1000, .len = 0x1dc,
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 9),
	}, {
		.name = "ctl_1", .id = CTL_1,
		.base = 0x1200, .len = 0x1dc,
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 10),
	}, {
		.name = "ctl_2", .id = CTL_2,
		.base = 0x1400, .len = 0x1dc,
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 11),
	}, {
		.name = "ctl_3", .id = CTL_3,
		.base = 0x1600, .len = 0x1dc,
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 12),
	},
};

/*
 * sde-sspp-type vig vig dma dma dma, sde-sspp-off 0x5000 0x7000 0x25000
 * 0x27000 0x29000, sde-sspp-xin-id 0 4 1 5 9, sde-sspp-src-size 0x1f8,
 * sde-qseed-type qseedv3lite, sde-smart-dma-rev smart_dma_v2p5.
 */
static const struct dpu_sspp_cfg sm7250_sspp[] = {
	{
		.name = "sspp_0", .id = SSPP_VIG0,
		.base = 0x4000, .len = 0x1f8,
		.features = VIG_SDM845_MASK_SDMA,
		.sblk = &dpu_vig_sblk_qseed3_2_4,
		.xin_id = 0,
		.type = SSPP_TYPE_VIG,
		.clk_ctrl = DPU_CLK_CTRL_VIG0,
	}, {
		.name = "sspp_1", .id = SSPP_VIG1,
		.base = 0x6000, .len = 0x1f8,
		.features = VIG_SDM845_MASK_SDMA,
		.sblk = &dpu_vig_sblk_qseed3_2_4,
		.xin_id = 4,
		.type = SSPP_TYPE_VIG,
		.clk_ctrl = DPU_CLK_CTRL_VIG1,
	}, {
		.name = "sspp_8", .id = SSPP_DMA0,
		.base = 0x24000, .len = 0x1f8,
		.features = DMA_SDM845_MASK_SDMA,
		.sblk = &dpu_dma_sblk,
		.xin_id = 1,
		.type = SSPP_TYPE_DMA,
		.clk_ctrl = DPU_CLK_CTRL_DMA0,
	}, {
		.name = "sspp_9", .id = SSPP_DMA1,
		.base = 0x26000, .len = 0x1f8,
		.features = DMA_SDM845_MASK_SDMA,
		.sblk = &dpu_dma_sblk,
		.xin_id = 5,
		.type = SSPP_TYPE_DMA,
		.clk_ctrl = DPU_CLK_CTRL_DMA1,
	}, {
		.name = "sspp_10", .id = SSPP_DMA2,
		.base = 0x28000, .len = 0x1f8,
		.features = DMA_CURSOR_SDM845_MASK_SDMA,
		.sblk = &dpu_dma_sblk,
		.xin_id = 9,
		.type = SSPP_TYPE_DMA,
		.clk_ctrl = DPU_CLK_CTRL_DMA2,
	},
};

/*
 * sde-mixer-off 0x45000.. (4), sde-mixer-size 0x320, sde-mixer-pair-mask
 * 2 1 4 3; PP/DSPP assignment follows the 1:1 order of sm7150/sm8250.
 */
static const struct dpu_lm_cfg sm7250_lm[] = {
	{
		.name = "lm_0", .id = LM_0,
		.base = 0x44000, .len = 0x320,
		.features = MIXER_MSM8998_MASK,
		.sblk = &sdm845_lm_sblk,
		.lm_pair = LM_1,
		.pingpong = PINGPONG_0,
		.dspp = DSPP_0,
	}, {
		.name = "lm_1", .id = LM_1,
		.base = 0x45000, .len = 0x320,
		.features = MIXER_MSM8998_MASK,
		.sblk = &sdm845_lm_sblk,
		.lm_pair = LM_0,
		.pingpong = PINGPONG_1,
		.dspp = DSPP_1,
	}, {
		.name = "lm_2", .id = LM_2,
		.base = 0x46000, .len = 0x320,
		.features = MIXER_MSM8998_MASK,
		.sblk = &sdm845_lm_sblk,
		.lm_pair = LM_3,
		.pingpong = PINGPONG_2,
	}, {
		.name = "lm_3", .id = LM_3,
		.base = 0x47000, .len = 0x320,
		.features = MIXER_MSM8998_MASK,
		.sblk = &sdm845_lm_sblk,
		.lm_pair = LM_2,
		.pingpong = PINGPONG_3,
	},
};

/* sde-dspp-off 0x55000 0x57000, sde-dspp-size 0x1800 */
static const struct dpu_dspp_cfg sm7250_dspp[] = {
	{
		.name = "dspp_0", .id = DSPP_0,
		.base = 0x54000, .len = 0x1800,
		.sblk = &sdm845_dspp_sblk,
	}, {
		.name = "dspp_1", .id = DSPP_1,
		.base = 0x56000, .len = 0x1800,
		.sblk = &sdm845_dspp_sblk,
	},
};

/* sde-pp-off 0x71000 0x71800 0x72000 0x72800, sde-pp-merge-3d-id 0 0 1 1 */
static const struct dpu_pingpong_cfg sm7250_pp[] = {
	{
		.name = "pingpong_0", .id = PINGPONG_0,
		.base = 0x70000, .len = 0xd4,
		.sblk = &sdm845_pp_sblk,
		.merge_3d = MERGE_3D_0,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 8),
	}, {
		.name = "pingpong_1", .id = PINGPONG_1,
		.base = 0x70800, .len = 0xd4,
		.sblk = &sdm845_pp_sblk,
		.merge_3d = MERGE_3D_0,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 9),
	}, {
		.name = "pingpong_2", .id = PINGPONG_2,
		.base = 0x71000, .len = 0xd4,
		.sblk = &sdm845_pp_sblk,
		.merge_3d = MERGE_3D_1,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 10),
	}, {
		.name = "pingpong_3", .id = PINGPONG_3,
		.base = 0x71800, .len = 0xd4,
		.sblk = &sdm845_pp_sblk,
		.merge_3d = MERGE_3D_1,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 11),
	},
};

/* sde-merge-3d-off 0x84000 0x84100 */
static const struct dpu_merge_3d_cfg sm7250_merge_3d[] = {
	{
		.name = "merge_3d_0", .id = MERGE_3D_0,
		.base = 0x83000, .len = 0x8,
	}, {
		.name = "merge_3d_1", .id = MERGE_3D_1,
		.base = 0x83100, .len = 0x8,
	},
};

/* sde-dsc-off 0x81000 0x81400, sde-dsc-size 0x140, sde-dsc-pair-mask 2 1 */
static const struct dpu_dsc_cfg sm7250_dsc[] = {
	{
		.name = "dsc_0", .id = DSC_0,
		.base = 0x80000, .len = 0x140,
	}, {
		.name = "dsc_1", .id = DSC_1,
		.base = 0x80400, .len = 0x140,
	},
};

/*
 * sde-intf-off 0x6b000 0x6b800 0x6c000 0x6c800, sde-intf-type dp dsi dsi dp.
 * Interrupt indices are the same as on sm8150/sm8250/sm7150 (same top
 * interrupt layout); intf_1 (DSI0) has the tear-read-pointer interrupt used
 * for command mode panels.
 */
static const struct dpu_intf_cfg sm7250_intf[] = {
	{
		.name = "intf_0", .id = INTF_0,
		.base = 0x6a000, .len = 0x280,
		.type = INTF_DP,
		.controller_id = MSM_DP_CONTROLLER_0,
		.prog_fetch_lines_worst_case = 24,
		.intr_underrun = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 24),
		.intr_vsync = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 25),
	}, {
		.name = "intf_1", .id = INTF_1,
		.base = 0x6a800, .len = 0x2b8,
		.type = INTF_DSI,
		.controller_id = MSM_DSI_CONTROLLER_0,
		.prog_fetch_lines_worst_case = 24,
		.intr_underrun = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 26),
		.intr_vsync = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 27),
		.intr_tear_rd_ptr = DPU_IRQ_IDX(MDP_INTF1_TEAR_INTR, 2),
	}, {
		.name = "intf_2", .id = INTF_2,
		.base = 0x6b000, .len = 0x2b8,
		.type = INTF_DSI,
		.controller_id = MSM_DSI_CONTROLLER_1,
		.prog_fetch_lines_worst_case = 24,
		.intr_underrun = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 28),
		.intr_vsync = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 29),
		.intr_tear_rd_ptr = DPU_IRQ_IDX(MDP_INTF2_TEAR_INTR, 2),
	}, {
		.name = "intf_3", .id = INTF_3,
		.base = 0x6b800, .len = 0x280,
		.type = INTF_DP,
		.controller_id = MSM_DP_CONTROLLER_0,	/* pair with intf_0 for DP MST */
		.prog_fetch_lines_worst_case = 24,
		.intr_underrun = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 30),
		.intr_vsync = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 31),
	},
};

/* sde-wb-off 0x66000, sde-wb-xin-id 6, sde-wb-linewidth 4096 */
static const struct dpu_wb_cfg sm7250_wb[] = {
	{
		.name = "wb_2", .id = WB_2,
		.base = 0x65000, .len = 0x2c8,
		.features = WB_SDM845_MASK,
		.format_list = wb2_formats_rgb_yuv,
		.num_formats = ARRAY_SIZE(wb2_formats_rgb_yuv),
		.clk_ctrl = DPU_CLK_CTRL_WB2,
		.xin_id = 6,
		.maxlinewidth = 4096,
		.intr_wb_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 4),
	},
};

/*
 * Perf: sde-max-bw-{low,high}-kbps, sde-min-{core,llcc,dram}-ib-kbps,
 * sde-danger-lut / sde-safe-lut-* / sde-qos-lut-* and sde-cdp-setting from
 * lito-sde.dtsi; min_prefill_lines = 35 from sde_hw_catalog.c (IS_SAIPAN_TARGET).
 * The inefficiency factors are not in the DT; copied from sm8250/sm7150.
 */
static const struct dpu_perf_cfg sm7250_perf_data = {
	.max_bw_low = 5800000,
	.max_bw_high = 8600000,
	.min_core_ib = 4800000,
	.min_llcc_ib = 0,
	.min_dram_ib = 800000,
	.min_prefill_lines = 35,
	.danger_lut_tbl = {0xff, 0xffff, 0x0},
	.safe_lut_tbl = {0xfffc, 0xff00, 0xffff},
	.qos_lut_tbl = {
		{
		.nentry = ARRAY_SIZE(sm8150_qos_linear),
		.entries = sm8150_qos_linear
		}, {
		.nentry = ARRAY_SIZE(sc7180_qos_macrotile),
		.entries = sc7180_qos_macrotile
		}, {
		.nentry = ARRAY_SIZE(sc7180_qos_nrt),
		.entries = sc7180_qos_nrt
		},
	},
	.cdp_cfg = {
		{.rd_enable = 1, .wr_enable = 1},
		{.rd_enable = 1, .wr_enable = 0}
	},
	.clk_inefficiency_factor = 105,
	.bw_inefficiency_factor = 120,
};

static const struct dpu_mdss_version sm7250_mdss_ver = {
	.core_major_ver = 6,
	.core_minor_ver = 1,
};

const struct dpu_mdss_cfg dpu_sm7250_cfg = {
	.mdss_ver = &sm7250_mdss_ver,
	.caps = &sm7250_dpu_caps,
	.mdp = &sm7250_mdp,
	.cdm = &dpu_cdm_5_x,
	.ctl_count = ARRAY_SIZE(sm7250_ctl),
	.ctl = sm7250_ctl,
	.sspp_count = ARRAY_SIZE(sm7250_sspp),
	.sspp = sm7250_sspp,
	.mixer_count = ARRAY_SIZE(sm7250_lm),
	.mixer = sm7250_lm,
	.dspp_count = ARRAY_SIZE(sm7250_dspp),
	.dspp = sm7250_dspp,
	.dsc_count = ARRAY_SIZE(sm7250_dsc),
	.dsc = sm7250_dsc,
	.pingpong_count = ARRAY_SIZE(sm7250_pp),
	.pingpong = sm7250_pp,
	.merge_3d_count = ARRAY_SIZE(sm7250_merge_3d),
	.merge_3d = sm7250_merge_3d,
	.intf_count = ARRAY_SIZE(sm7250_intf),
	.intf = sm7250_intf,
	.vbif = &sdm845_vbif,
	.wb_count = ARRAY_SIZE(sm7250_wb),
	.wb = sm7250_wb,
	.perf = &sm7250_perf_data,
};

#endif
