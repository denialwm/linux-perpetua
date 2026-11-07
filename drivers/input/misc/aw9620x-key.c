// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW9620x capacitive sensor as a key
 *
 * The AW9620x family (AW96203, AW96205, AW96208) are capacitive proximity
 * ("SAR") sensors with a small MCU running a program from the chip's own
 * flash. Phones also use them as touch keys: the Nubia NX809J (RedMagic 11
 * Pro) has one AW96205 under each shoulder trigger. This driver reports
 * channel 0's first threshold as one key.
 *
 * Register protocol from Awinic's GPL aw_sar driver: 16-bit register
 * addresses with 32-bit values, both big endian on the wire. The chip's
 * configuration comes from the vendor's register file (firmware-name), a
 * 60-byte header, a record count and 6-byte records (address, value; little
 * endian), checksummed by the byte sum from offset 4.
 *
 * The driver never writes the chip's flash.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#define AW9620X_REG_FWVER	0x1a0c
#define AW9620X_REG_STAT	0x1a18
#define AW9620X_REG_CMD		0x4408
#define AW9620X_REG_IRQSRC	0x4410
#define AW9620X_REG_MCFG	0x4444
#define AW9620X_REG_FLASH_WAKE	0x4700
#define AW9620X_REG_CHIPID	0xff00
#define AW9620X_REG_HOSTCTRL	0xff10
#define AW9620X_REG_RSTNALL	0xff18
#define AW9620X_REG_ACCESSEN	0xff20

#define AW9620X_CHIPID_MASK	GENMASK(31, 4)
#define AW9620X_CHIPID		0x20226200
#define AW9620X_ACCESSEN_OPEN	0x3c00f091
#define AW9620X_FLASH_WAKE_ON	0x110
#define AW9620X_FLASH_WAKE_OFF	0x102
#define AW9620X_RSTNALL_RESET	0x3c
#define AW9620X_HOSTCTRL_EN	BIT(0)
#define AW9620X_CMD_ACTIVE	1
#define AW9620X_CMD_SLEEP	2
#define AW9620X_IRQ_INIT_OVER	BIT(0)
/* channel 0 past its first threshold, where the vendor driver reads it */
#define AW9620X_STAT_CH0_TH0	BIT(0)

#define AW9620X_POWER_ON_MS	25
#define AW9620X_MCFG_MS		20
#define AW9620X_INIT_WAIT_MS	100

#define AW9620X_BIN_CHIP	0x18
#define AW9620X_BIN_COUNT	0x3c
#define AW9620X_BIN_RECORDS	0x40
#define AW9620X_BIN_RECORD_LEN	6

struct aw9620x {
	struct device *dev;
	struct regmap *regmap;
	struct input_dev *input;
	unsigned int code;
};

static const struct regmap_config aw9620x_regmap = {
	.reg_bits = 16,
	.val_bits = 32,
	.max_register = 0xffff,
};

static int aw9620x_reset(struct aw9620x *aw)
{
	struct regmap *rm = aw->regmap;
	unsigned int irqsrc;
	int ret;

	ret = regmap_write(rm, AW9620X_REG_ACCESSEN, AW9620X_ACCESSEN_OPEN);
	ret = ret ?: regmap_write(rm, AW9620X_REG_FLASH_WAKE, AW9620X_FLASH_WAKE_ON);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	ret = regmap_write(rm, AW9620X_REG_MCFG, 0);
	if (ret)
		return ret;
	msleep(AW9620X_MCFG_MS);

	ret = regmap_write(rm, AW9620X_REG_RSTNALL, AW9620X_RSTNALL_RESET);
	if (ret)
		return ret;
	msleep(AW9620X_POWER_ON_MS);

	ret = regmap_write(rm, AW9620X_REG_FLASH_WAKE, AW9620X_FLASH_WAKE_OFF);
	if (ret)
		return ret;

	/* The program flags its start in IRQSRC */
	return regmap_read_poll_timeout(rm, AW9620X_REG_IRQSRC, irqsrc,
					irqsrc & AW9620X_IRQ_INIT_OVER, 1000,
					AW9620X_INIT_WAIT_MS * 1000);
}

static int aw9620x_load_config(struct aw9620x *aw, const char *name)
{
	const struct firmware *fw;
	const u8 *rec;
	u32 count, sum = 0;
	size_t i;
	int ret;

	ret = request_firmware(&fw, name, aw->dev);
	if (ret)
		return dev_err_probe(aw->dev, ret, "loading %s\n", name);

	ret = -EINVAL;
	if (fw->size < AW9620X_BIN_RECORDS ||
	    memcmp(fw->data + AW9620X_BIN_CHIP, "AW9620", 6)) {
		dev_err(aw->dev, "%s: not an AW9620x register file\n", name);
		goto out;
	}
	count = get_unaligned_le32(fw->data + AW9620X_BIN_COUNT);
	if (count > (fw->size - AW9620X_BIN_RECORDS) / AW9620X_BIN_RECORD_LEN) {
		dev_err(aw->dev, "%s: %u records don't fit\n", name, count);
		goto out;
	}
	for (i = 4; i < fw->size; i++)
		sum += fw->data[i];
	if (sum != get_unaligned_le32(fw->data)) {
		dev_err(aw->dev, "%s: bad checksum\n", name);
		goto out;
	}

	rec = fw->data + AW9620X_BIN_RECORDS;
	for (i = 0; i < count; i++, rec += AW9620X_BIN_RECORD_LEN) {
		ret = regmap_write(aw->regmap, get_unaligned_le16(rec),
				   get_unaligned_le32(rec + 2));
		if (ret) {
			dev_err(aw->dev, "%s: writing record %zu: %d\n", name, i, ret);
			goto out;
		}
	}
	ret = 0;
out:
	release_firmware(fw);
	return ret;
}

static void aw9620x_report(struct aw9620x *aw)
{
	unsigned int stat;

	if (regmap_read(aw->regmap, AW9620X_REG_STAT, &stat))
		return;
	input_report_key(aw->input, aw->code, stat & AW9620X_STAT_CH0_TH0);
	input_sync(aw->input);
}

static irqreturn_t aw9620x_irq(int irq, void *data)
{
	struct aw9620x *aw = data;
	unsigned int irqsrc;

	/* Reading IRQSRC clears it */
	if (regmap_read(aw->regmap, AW9620X_REG_IRQSRC, &irqsrc))
		return IRQ_NONE;
	aw9620x_report(aw);

	return IRQ_HANDLED;
}

static void aw9620x_sleep(void *data)
{
	struct aw9620x *aw = data;

	regmap_write(aw->regmap, AW9620X_REG_CMD, AW9620X_CMD_SLEEP);
}

static int aw9620x_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	const char *fw_name, *label;
	unsigned int id, ver, irqsrc;
	struct aw9620x *aw;
	int ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;
	aw->dev = dev;

	ret = device_property_read_u32(dev, "linux,code", &aw->code);
	if (ret)
		return dev_err_probe(dev, ret, "linux,code\n");
	ret = device_property_read_string(dev, "firmware-name", &fw_name);
	if (ret)
		return dev_err_probe(dev, ret, "firmware-name\n");
	if (device_property_read_string(dev, "label", &label))
		label = "AW9620x key";

	aw->regmap = devm_regmap_init_i2c(client, &aw9620x_regmap);
	if (IS_ERR(aw->regmap))
		return PTR_ERR(aw->regmap);

	ret = devm_regulator_get_enable(dev, "vcc");
	if (ret)
		return dev_err_probe(dev, ret, "vcc\n");
	msleep(AW9620X_POWER_ON_MS);

	ret = regmap_read(aw->regmap, AW9620X_REG_CHIPID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "reading chip ID\n");
	if ((id & AW9620X_CHIPID_MASK) != AW9620X_CHIPID)
		return dev_err_probe(dev, -ENODEV, "chip ID 0x%08x, not an AW9620x\n", id);

	ret = aw9620x_reset(aw);
	if (ret)
		return dev_err_probe(dev, ret, "starting the chip\n");

	ret = aw9620x_load_config(aw, fw_name);
	if (ret)
		return ret;

	ret = regmap_write(aw->regmap, AW9620X_REG_HOSTCTRL, AW9620X_HOSTCTRL_EN);
	ret = ret ?: regmap_write(aw->regmap, AW9620X_REG_CMD, AW9620X_CMD_ACTIVE);
	ret = ret ?: regmap_read(aw->regmap, AW9620X_REG_IRQSRC, &irqsrc);
	ret = ret ?: regmap_read(aw->regmap, AW9620X_REG_FWVER, &ver);
	if (ret)
		return dev_err_probe(dev, ret, "activating the chip\n");
	ret = devm_add_action_or_reset(dev, aw9620x_sleep, aw);
	if (ret)
		return ret;

	aw->input = devm_input_allocate_device(dev);
	if (!aw->input)
		return -ENOMEM;
	aw->input->name = label;
	aw->input->id.bustype = BUS_I2C;
	input_set_capability(aw->input, EV_KEY, aw->code);

	ret = input_register_device(aw->input);
	if (ret)
		return dev_err_probe(dev, ret, "registering input\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, aw9620x_irq,
					IRQF_ONESHOT, dev_name(dev), aw);
	if (ret)
		return dev_err_probe(dev, ret, "requesting IRQ\n");

	aw9620x_report(aw);
	dev_info(dev, "chip 0x%08x, program 0x%08x\n", id, ver);

	return 0;
}

static const struct of_device_id aw9620x_of_match[] = {
	{ .compatible = "awinic,aw96205" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw9620x_of_match);

static struct i2c_driver aw9620x_driver = {
	.driver = {
		.name = "aw9620x-key",
		.of_match_table = aw9620x_of_match,
	},
	.probe = aw9620x_probe,
};
module_i2c_driver(aw9620x_driver);

MODULE_DESCRIPTION("Awinic AW9620x capacitive sensor as a key");
MODULE_LICENSE("GPL");
