/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __PMDOMAIN_MEDIATEK_MT6897_PM_DOMAINS_H
#define __PMDOMAIN_MEDIATEK_MT6897_PM_DOMAINS_H

#include "mtk-pm-domains.h"
#include <dt-bindings/power/mediatek,mt6897-power.h>

/*
 * MT6897 power domain support
 *
 * Only the connectivity domain so far: the bootloader leaves the display
 * and multimedia domains on, and the GPU has its own controller.
 */

/* INFRA bus protection registers */
#define MT6897_TOP_AXI_PROT_EN_SET		0x004
#define MT6897_TOP_AXI_PROT_EN_CLR		0x008
#define MT6897_TOP_AXI_PROT_EN_STA		0x00c
 #define MT6897_TOP_AXI_PROT_EN_CONN		BIT(25)
 #define MT6897_TOP_AXI_PROT_EN_CONN_2ND	BIT(26)

#define MT6897_TOP_AXI_PROT_EN_CONNSYS_SET	0x1c4
#define MT6897_TOP_AXI_PROT_EN_CONNSYS_CLR	0x1c8
#define MT6897_TOP_AXI_PROT_EN_CONNSYS_STA	0x1cc
 #define MT6897_TOP_AXI_PROT_EN_CONNSYS_CONN	BIT(1)
 #define MT6897_TOP_AXI_PROT_EN_CONNSYS_CONN_2ND	BIT(0)

/* PWR_CON registers */
#define MT6897_PWR_ACK				BIT(30)
#define MT6897_PWR_ACK_2ND			BIT(31)

static enum scpsys_bus_prot_block scpsys_bus_prot_blocks_mt6897[] = {
	BUS_PROT_BLOCK_INFRA,
};

static const struct scpsys_domain_data scpsys_domain_data_mt6897[] = {
	[MT6897_POWER_DOMAIN_CONN] = {
		.name = "conn",
		.sta_mask = MT6897_PWR_ACK,
		.sta2nd_mask = MT6897_PWR_ACK_2ND,
		.ctl_offs = 0xe04,
		.pwr_sta_offs = 0xe04,
		.pwr_sta2nd_offs = 0xe04,
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6897_TOP_AXI_PROT_EN_CONN,
					MT6897_TOP_AXI_PROT_EN_SET,
					MT6897_TOP_AXI_PROT_EN_CLR,
					MT6897_TOP_AXI_PROT_EN_STA),
			BUS_PROT_WR_IGN(INFRA, MT6897_TOP_AXI_PROT_EN_CONNSYS_CONN,
					MT6897_TOP_AXI_PROT_EN_CONNSYS_SET,
					MT6897_TOP_AXI_PROT_EN_CONNSYS_CLR,
					MT6897_TOP_AXI_PROT_EN_CONNSYS_STA),
			BUS_PROT_WR_IGN(INFRA, MT6897_TOP_AXI_PROT_EN_CONN_2ND,
					MT6897_TOP_AXI_PROT_EN_SET,
					MT6897_TOP_AXI_PROT_EN_CLR,
					MT6897_TOP_AXI_PROT_EN_STA),
			BUS_PROT_WR_IGN(INFRA, MT6897_TOP_AXI_PROT_EN_CONNSYS_CONN_2ND,
					MT6897_TOP_AXI_PROT_EN_CONNSYS_SET,
					MT6897_TOP_AXI_PROT_EN_CONNSYS_CLR,
					MT6897_TOP_AXI_PROT_EN_CONNSYS_STA),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
};

static const struct scpsys_soc_data mt6897_scpsys_data = {
	.domains_data = scpsys_domain_data_mt6897,
	.num_domains = ARRAY_SIZE(scpsys_domain_data_mt6897),
	.bus_prot_blocks = scpsys_bus_prot_blocks_mt6897,
	.num_bus_prot_blocks = ARRAY_SIZE(scpsys_bus_prot_blocks_mt6897),
	.type = SCPSYS_MTCMOS_TYPE_DIRECT_CTL,
};

#endif /* __PMDOMAIN_MEDIATEK_MT6897_PM_DOMAINS_H */
