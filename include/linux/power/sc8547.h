/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_POWER_SC8547_H
#define _LINUX_POWER_SC8547_H

#include <linux/types.h>

struct device;
struct fwnode_handle;
struct regmap;
struct sc8547;

/**
 * struct sc8547_sample - one frozen ADC snapshot of a charge pump
 * @ibus_ua: input current.
 * @vbus_uv: input voltage behind the input switch.
 * @vac_uv: voltage at the connector side of the input switch.
 * @vout_uv: output voltage.
 * @vbat_uv: battery voltage at the pump's sense pins.
 * @temp: die temperature in tenths of a degree Celsius.
 * @health: POWER_SUPPLY_HEALTH_* for the pump's latched and live faults.
 * @switching: the converter is switching.
 */
struct sc8547_sample {
	int ibus_ua;
	int vbus_uv;
	int vac_uv;
	int vout_uv;
	int vbat_uv;
	int temp;
	int health;
	bool switching;
};

struct sc8547 *sc8547_get(struct device *consumer, struct fwnode_handle *fwnode);
struct regmap *sc8547_regmap(struct sc8547 *cp);
int sc8547_prepare(struct sc8547 *cp);
int sc8547_enable(struct sc8547 *cp);
int sc8547_disable(struct sc8547 *cp);
int sc8547_sample(struct sc8547 *cp, struct sc8547_sample *sample);
void sc8547_set_phy_handler(struct sc8547 *cp, void (*handler)(void *data),
			    void *data);

#endif
