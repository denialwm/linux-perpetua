/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek connectivity subsystem power control
 */

#ifndef __SOC_MEDIATEK_MTK_CONSYS_H
#define __SOC_MEDIATEK_MTK_CONSYS_H

#include <linux/err.h>

struct device;
struct mtk_consys;

/* The radios of the connectivity subsystem, as the secure world numbers them */
enum mtk_consys_radio {
	MTK_CONSYS_BT = 0,
	MTK_CONSYS_FM = 1,
	MTK_CONSYS_GPS = 2,
	MTK_CONSYS_WIFI = 3,
};

#if IS_ENABLED(CONFIG_MTK_CONSYS)
struct mtk_consys *mtk_consys_get(struct device *dev);
int mtk_consys_power_on(struct mtk_consys *consys, enum mtk_consys_radio radio);
void mtk_consys_power_off(struct mtk_consys *consys,
			  enum mtk_consys_radio radio);
#else
static inline struct mtk_consys *mtk_consys_get(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

static inline int mtk_consys_power_on(struct mtk_consys *consys,
				      enum mtk_consys_radio radio)
{
	return -ENODEV;
}

static inline void mtk_consys_power_off(struct mtk_consys *consys,
					enum mtk_consys_radio radio)
{
}
#endif

#endif /* __SOC_MEDIATEK_MTK_CONSYS_H */
