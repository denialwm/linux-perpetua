// SPDX-License-Identifier: GPL-2.0
/*
 * A thermal sensor that reports a weighted average of other thermal zones.
 *
 * Phones estimate the temperature of surfaces that carry no thermistor,
 * such as the back cover, from fixed weights of board and battery sensors,
 * and throttle charging and performance on that estimate.
 *
 * The inputs are looked up by zone name on each read, so a zone that its
 * driver registers again, for example after a module reload, is followed.
 */

#include <linux/err.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

struct weighted_thermal_input {
	const char *zone;
	u32 weight;
};

struct weighted_thermal_sensor {
	struct weighted_thermal_input *inputs;
	unsigned int num_inputs;
	u32 total_weight;
};

static int weighted_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct weighted_thermal_sensor *sensor = thermal_zone_device_priv(tz);
	s64 sum = 0;
	unsigned int i;

	for (i = 0; i < sensor->num_inputs; i++) {
		const struct weighted_thermal_input *input = &sensor->inputs[i];
		struct thermal_zone_device *zone;
		int input_temp, ret;

		zone = thermal_zone_get_zone_by_name(input->zone);
		if (IS_ERR(zone))
			return PTR_ERR(zone);

		/* A missing input must not lower the estimate. */
		ret = thermal_zone_get_temp(zone, &input_temp);
		if (ret)
			return ret;

		sum += (s64)input_temp * input->weight;
	}

	*temp = div_s64(sum, sensor->total_weight);

	return 0;
}

static const struct thermal_zone_device_ops weighted_thermal_ops = {
	.get_temp = weighted_thermal_get_temp,
};

static int weighted_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct weighted_thermal_sensor *sensor;
	struct thermal_zone_device *tzd;
	int count, i, ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	count = of_property_count_strings(np, "zone-names");
	if (count <= 0)
		return dev_err_probe(dev, -EINVAL, "no input zones\n");

	if (of_property_count_u32_elems(np, "weights") != count)
		return dev_err_probe(dev, -EINVAL,
				     "need one weight for each input zone\n");

	sensor->inputs = devm_kcalloc(dev, count, sizeof(*sensor->inputs), GFP_KERNEL);
	if (!sensor->inputs)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		struct weighted_thermal_input *input = &sensor->inputs[i];
		struct thermal_zone_device *zone;

		ret = of_property_read_string_index(np, "zone-names", i, &input->zone);
		if (ret)
			return dev_err_probe(dev, ret, "failed to read zone %d\n", i);

		ret = of_property_read_u32_index(np, "weights", i, &input->weight);
		if (ret)
			return dev_err_probe(dev, ret, "failed to read weight %d\n", i);

		/* Report nothing until every input can be read. */
		zone = thermal_zone_get_zone_by_name(input->zone);
		if (PTR_ERR_OR_ZERO(zone) == -ENODEV)
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "waiting for zone %s\n", input->zone);
		if (IS_ERR(zone))
			return dev_err_probe(dev, PTR_ERR(zone),
					     "cannot use zone %s\n", input->zone);

		sensor->total_weight += input->weight;
	}

	if (!sensor->total_weight)
		return dev_err_probe(dev, -EINVAL, "weights add up to zero\n");

	sensor->num_inputs = count;

	tzd = devm_thermal_of_zone_register(dev, 0, sensor, &weighted_thermal_ops);
	if (IS_ERR(tzd))
		return dev_err_probe(dev, PTR_ERR(tzd), "failed to register the zone\n");

	return 0;
}

static const struct of_device_id weighted_thermal_of_match[] = {
	{ .compatible = "weighted-thermal-sensor" },
	{ }
};
MODULE_DEVICE_TABLE(of, weighted_thermal_of_match);

static struct platform_driver weighted_thermal_driver = {
	.probe = weighted_thermal_probe,
	.driver = {
		.name = "weighted-thermal-sensor",
		.of_match_table = weighted_thermal_of_match,
	},
};
module_platform_driver(weighted_thermal_driver);

MODULE_DESCRIPTION("Weighted average of thermal zones as a thermal sensor");
MODULE_LICENSE("GPL");
