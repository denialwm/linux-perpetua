/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 */

#ifndef _DT_BINDINGS_CLK_QCOM_VIDEOCC_SM7250_H
#define _DT_BINDINGS_CLK_QCOM_VIDEOCC_SM7250_H

/* VIDEOCC clocks */
#define VIDEOCC_PLL0				0
#define VIDEOCC_APB_CLK				1
#define VIDEOCC_IRIS_CLK_SRC			2
#define VIDEOCC_MVS0_AXI_CLK			3
#define VIDEOCC_MVS0_CORE_CLK			4
#define VIDEOCC_MVS1_AXI_CLK			5
#define VIDEOCC_MVS1_CORE_CLK			6
#define VIDEOCC_MVSC_CORE_CLK			7
#define VIDEOCC_MVSC_CTL_AXI_CLK		8
#define VIDEOCC_SLEEP_CLK			9
#define VIDEOCC_SLEEP_CLK_SRC			10
#define VIDEOCC_VENUS_AHB_CLK			11
#define VIDEOCC_XO_CLK_SRC			12
#define VIDEOCC_XO_CLK				13

/* VIDEOCC resets */
#define VIDEOCC_INTERFACE_BCR			0
#define VIDEOCC_MVS0_BCR			1
#define VIDEOCC_MVS1_BCR			2
#define VIDEOCC_MVSC_BCR			3

/* VIDEOCC GDSCs */
#define MVSC_GDSC				0
#define MVS0_GDSC				1
#define MVS1_GDSC				2

#endif
