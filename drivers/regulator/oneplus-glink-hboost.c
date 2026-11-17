// SPDX-License-Identifier: GPL-2.0-only
/*
 * The boost converter OnePlus' charger firmware drives for the stylus charger
 * coil, set over PMIC GLINK.
 *
 * The firmware takes a voltage selector (2 V + 50 mV steps) and acknowledges
 * it. It offers no way to read the voltage back, so the regulator reports no
 * voltage until it has set one, and nothing is set before the consumer asks:
 * the charger chip programs the boost as a step of its own power-up sequence.
 */

#include <linux/auxiliary_bus.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>

#define HBOOST_OWNER			32785
#define HBOOST_OPCODE_SET_VOUT		0x10007

#define HBOOST_MIN_UV			2000000
#define HBOOST_STEP_UV			50000
#define HBOOST_N_VOLTAGES		256

#define HBOOST_TIMEOUT			msecs_to_jiffies(2500)

struct hboost_set_vout_req {
	struct pmic_glink_hdr hdr;
	u8 sel;
	u8 reserved[3];
} __packed;

struct hboost_set_vout_resp {
	struct pmic_glink_hdr hdr;
	__le32 ret_code;
} __packed;

struct hboost {
	struct device *dev;
	struct pmic_glink_client *client;
	struct completion ack;
	/* serializes requests, as the firmware answers one at a time */
	struct mutex lock;
	int ret_code;
	bool up;
	int sel;
};

static int hboost_set_voltage_sel(struct regulator_dev *rdev, unsigned int sel)
{
	struct hboost *hb = rdev_get_drvdata(rdev);
	struct hboost_set_vout_req req = {
		.hdr.owner = cpu_to_le32(HBOOST_OWNER),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(HBOOST_OPCODE_SET_VOUT),
		.sel = sel,
	};
	int ret;

	guard(mutex)(&hb->lock);

	if (!READ_ONCE(hb->up))
		return -ENOTCONN;

	reinit_completion(&hb->ack);
	ret = pmic_glink_send(hb->client, &req, sizeof(req));
	if (ret)
		return ret;

	if (!wait_for_completion_timeout(&hb->ack, HBOOST_TIMEOUT))
		return -ETIMEDOUT;

	if (hb->ret_code) {
		dev_err(hb->dev, "firmware refused selector %u: %d\n", sel, hb->ret_code);
		return -EIO;
	}

	hb->sel = sel;

	return 0;
}

static int hboost_get_voltage_sel(struct regulator_dev *rdev)
{
	struct hboost *hb = rdev_get_drvdata(rdev);

	guard(mutex)(&hb->lock);

	return hb->sel;
}

static const struct regulator_ops hboost_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.set_voltage_sel = hboost_set_voltage_sel,
	.get_voltage_sel = hboost_get_voltage_sel,
};

static const struct regulator_desc hboost_desc = {
	.name = "pen-boost",
	.ops = &hboost_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.min_uV = HBOOST_MIN_UV,
	.uV_step = HBOOST_STEP_UV,
	.n_voltages = HBOOST_N_VOLTAGES,
};

static void hboost_callback(const void *data, size_t len, void *priv)
{
	const struct hboost_set_vout_resp *resp = data;
	struct hboost *hb = priv;

	if (le32_to_cpu(resp->hdr.opcode) != HBOOST_OPCODE_SET_VOUT)
		return;

	if (len < sizeof(*resp)) {
		dev_warn(hb->dev, "short response: %zu bytes\n", len);
		return;
	}

	hb->ret_code = le32_to_cpu(resp->ret_code);
	complete(&hb->ack);
}

static void hboost_pdr_notify(void *priv, int state)
{
	struct hboost *hb = priv;

	WRITE_ONCE(hb->up, state == SERVREG_SERVICE_STATE_UP);
}

static int hboost_probe(struct auxiliary_device *adev,
			const struct auxiliary_device_id *id)
{
	struct regulator_config config = {};
	struct regulator_init_data *init_data;
	struct device *dev = &adev->dev;
	struct device_node *np;
	struct regulator_dev *rdev;
	struct hboost *hb;
	int ret;

	hb = devm_kzalloc(dev, sizeof(*hb), GFP_KERNEL);
	if (!hb)
		return -ENOMEM;

	hb->dev = dev;
	hb->sel = -ENOTRECOVERABLE;
	init_completion(&hb->ack);
	ret = devm_mutex_init(dev, &hb->lock);
	if (ret)
		return ret;

	hb->client = devm_pmic_glink_client_alloc(dev, HBOOST_OWNER,
						  hboost_callback,
						  hboost_pdr_notify, hb);
	if (IS_ERR(hb->client))
		return dev_err_probe(dev, PTR_ERR(hb->client),
				     "failed to register with pmic_glink\n");

	np = of_get_child_by_name(dev->of_node, "regulator-pen-boost");
	if (!np)
		return dev_err_probe(dev, -ENODEV, "no regulator-pen-boost node\n");

	init_data = of_get_regulator_init_data(dev, np, &hboost_desc);
	if (!init_data) {
		of_node_put(np);
		return -ENOMEM;
	}

	/*
	 * A voltage range makes the core apply it at registration, which
	 * needs a read-back and the firmware to be up. Leave the boost alone
	 * until the consumer sets it.
	 */
	init_data->constraints.apply_uV = false;

	config.dev = dev;
	config.driver_data = hb;
	config.init_data = init_data;
	config.of_node = np;

	rdev = devm_regulator_register(dev, &hboost_desc, &config);
	of_node_put(np);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev), "failed to register regulator\n");

	pmic_glink_client_register(hb->client);

	return 0;
}

static const struct auxiliary_device_id hboost_id_table[] = {
	{ .name = "pmic_glink.pen-boost", },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, hboost_id_table);

static struct auxiliary_driver hboost_driver = {
	.name = "oneplus_glink_hboost",
	.probe = hboost_probe,
	.id_table = hboost_id_table,
};
module_auxiliary_driver(hboost_driver);

MODULE_DESCRIPTION("OnePlus PMIC GLINK stylus charger boost regulator");
MODULE_LICENSE("GPL");
