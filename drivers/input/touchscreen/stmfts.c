// SPDX-License-Identifier: GPL-2.0
/*
 * STMicroelectronics FTS Touchscreen device driver
 *
 * Copyright 2017 Samsung Electronics Co., Ltd.
 * Copyright 2017 Andi Shyti <andi@etezian.org>
 * Copyright David Heidelberg <david@ixit.cz>
 * Copyright Petr Hodina <petr.hodina@protonmail.com>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

/* commands (I2C opcodes; see stmfts_spi_read() for SPI) */
#define STMFTS_READ_INFO			0x80
#define STMFTS_READ_STATUS			0x84
#define STMFTS_READ_ONE_EVENT			0x85
#define STMFTS_READ_ALL_EVENT			0x86
#define STMFTS_LATEST_EVENT			0x87
#define STMFTS_SLEEP_IN				0x90
#define STMFTS_SLEEP_OUT			0x91
#define STMFTS_MS_MT_SENSE_OFF			0x92
#define STMFTS_MS_MT_SENSE_ON			0x93
#define STMFTS_SS_HOVER_SENSE_OFF		0x94
#define STMFTS_SS_HOVER_SENSE_ON		0x95
#define STMFTS_MS_KEY_SENSE_OFF			0x9a
#define STMFTS_MS_KEY_SENSE_ON			0x9b
#define STMFTS_SYSTEM_RESET			0xa0
#define STMFTS_CLEAR_EVENT_STACK		0xa1
#define STMFTS_FULL_FORCE_CALIBRATION		0xa2
#define STMFTS_MS_CX_TUNING			0xa3
#define STMFTS_SS_CX_TUNING			0xa4
#define STMFTS5_SET_SCAN_MODE			0xa0

/* FTS5 scan mode commands */
#define STMFTS5_SCAN_MODE_ACTIVE		0x00
#define STMFTS5_SCAN_ACTIVE_MULTI_TOUCH		BIT(0)

/* events */
#define STMFTS_EV_NO_EVENT			0x00
#define STMFTS_EV_MULTI_TOUCH_DETECTED		0x02
#define STMFTS_EV_MULTI_TOUCH_ENTER		0x03
#define STMFTS_EV_MULTI_TOUCH_LEAVE		0x04
#define STMFTS_EV_MULTI_TOUCH_MOTION		0x05
#define STMFTS_EV_HOVER_ENTER			0x07
#define STMFTS_EV_HOVER_LEAVE			0x08
#define STMFTS_EV_HOVER_MOTION			0x09
#define STMFTS_EV_KEY_STATUS			0x0e
#define STMFTS_EV_ERROR				0x0f
#define STMFTS_EV_CONTROLLER_READY		0x10
#define STMFTS_EV_SLEEP_OUT_CONTROLLER_READY	0x11
#define STMFTS_EV_STATUS			0x16
#define STMFTS_EV_DEBUG				0xdb

/*
 * FTS5 events: the event ID is in the high nibble of event[0], the low
 * nibble carries additional payload (bits 5:4 of the touch major axis for
 * touch events), so it must be masked before comparing.
 */
#define STMFTS5_EV_CONTROLLER_READY		0x03
#define STMFTS5_EV_MULTI_TOUCH_ENTER		0x10
#define STMFTS5_EV_MULTI_TOUCH_MOTION		0x20
#define STMFTS5_EV_MULTI_TOUCH_LEAVE		0x30
#define STMFTS5_EV_STATUS_UPDATE		0x40
#define STMFTS5_EV_USER_REPORT			0x50
#define STMFTS5_EV_DEBUG			0xe0
#define STMFTS5_EV_ERROR			0xf0

/* multi touch related event masks */
#define STMFTS_MASK_EVENT_ID			0x0f
#define STMFTS_MASK_TOUCH_ID			0xf0
#define STMFTS_MASK_LEFT_EVENT			0x0f
#define STMFTS_MASK_X_MSB			0x0f
#define STMFTS_MASK_Y_LSB			0xf0
#define STMFTS5_MASK_EVENT_ID			0xf0
#define STMFTS5_MASK_TOUCH_TYPE			0x0f
#define STMFTS5_MASK_MAJOR_MSB			0x0c
#define STMFTS5_MASK_MINOR_MSB			0xc0

/* touch type classifications */
#define STMFTS_TOUCH_TYPE_INVALID		0x00
#define STMFTS_TOUCH_TYPE_FINGER		0x01
#define STMFTS_TOUCH_TYPE_GLOVE			0x02
#define STMFTS_TOUCH_TYPE_STYLUS		0x03
#define STMFTS_TOUCH_TYPE_PALM			0x04
#define STMFTS_TOUCH_TYPE_HOVER			0x05

/* key related event masks */
#define STMFTS_MASK_KEY_NO_TOUCH		0x00
#define STMFTS_MASK_KEY_MENU			0x01
#define STMFTS_MASK_KEY_BACK			0x02

#define STMFTS_RESET_TIMEOUT_MS	300

#define STMFTS_EVENT_SIZE	8
#define STMFTS_STACK_DEPTH	32
#define STMFTS_DATA_MAX_SIZE	(STMFTS_EVENT_SIZE * STMFTS_STACK_DEPTH)
#define STMFTS_MAX_FINGERS	10
#define STMFTS_DEV_NAME		"stmfts"

static const struct regulator_bulk_data stmfts_supplies[] = {
	{ .supply = "vdd" },
	{ .supply = "avdd" },
};

struct stmfts_data {
	const struct stmfts_chip_ops *ops;
	const struct stmfts_bus_ops *bus;

	struct device *dev;
	int irq;
	struct i2c_client *client;
	struct spi_device *spi;
	/* DMA-safe SPI buffers; rx holds the dummy byte + data */
	u8 *spi_tx;
	u8 *spi_rx;
	/* serialises the transfers through spi_tx and spi_rx */
	struct mutex spi_lock;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *mode_switch_gpio;
	struct led_classdev led_cdev;
	struct mutex mutex;

	struct touchscreen_properties prop;

	struct regulator_bulk_data *supplies;

	/*
	 * Presence of ledvdd will be used also to check
	 * whether the LED is supported.
	 */
	struct regulator *ledvdd;

	u16 chip_id;
	u8 chip_ver;
	u16 fw_ver;
	u8 config_id;
	u8 config_ver;

	u8 data[STMFTS_DATA_MAX_SIZE];

	struct completion cmd_done;

	bool use_key;
	bool led_status;
	bool hover_enabled;
	bool running;
};

struct stmfts_bus_ops {
	int (*write)(struct stmfts_data *sdata, const u8 *buf, size_t len);
	int (*read)(struct stmfts_data *sdata, u8 cmd, u8 *buf, size_t len);
};

struct stmfts_chip_ops {
	int  (*configure)(struct stmfts_data *sdata);
	int  (*setup_input)(struct stmfts_data *sdata);
	void (*parse_events)(struct stmfts_data *sdata);
	int  (*set_scan)(struct stmfts_data *sdata, bool enable);
	int  (*set_hover)(struct stmfts_data *sdata, bool enable);
	int  (*runtime_resume)(struct stmfts_data *sdata);
};

static int stmfts_brightness_set(struct led_classdev *led_cdev,
				 enum led_brightness value)
{
	struct stmfts_data *sdata = container_of(led_cdev,
					struct stmfts_data, led_cdev);
	int err;

	if (value != sdata->led_status && sdata->ledvdd) {
		if (!value) {
			regulator_disable(sdata->ledvdd);
		} else {
			err = regulator_enable(sdata->ledvdd);
			if (err) {
				dev_warn(sdata->dev,
					 "failed to enable ledvdd regulator: %d\n",
					 err);
				return err;
			}
		}
		sdata->led_status = value;
	}

	return 0;
}

static enum led_brightness stmfts_brightness_get(struct led_classdev *led_cdev)
{
	struct stmfts_data *sdata = container_of(led_cdev,
						struct stmfts_data, led_cdev);

	return !!regulator_is_enabled(sdata->ledvdd);
}

static int stmfts_write(struct stmfts_data *sdata, const u8 *buf, size_t len)
{
	return sdata->bus->write(sdata, buf, len);
}

static int stmfts_write_byte(struct stmfts_data *sdata, u8 cmd)
{
	return stmfts_write(sdata, &cmd, 1);
}

static int stmfts_read(struct stmfts_data *sdata, u8 cmd, u8 *buf, size_t len)
{
	return sdata->bus->read(sdata, cmd, buf, len);
}

static int stmfts_read_events(struct stmfts_data *sdata)
{
	return stmfts_read(sdata, STMFTS_READ_ALL_EVENT, sdata->data,
			   STMFTS_DATA_MAX_SIZE);
}

/* FTS4 event handling functions */
static void stmfts_report_contact_event(struct stmfts_data *sdata,
					const u8 event[])
{
	u8 slot_id = (event[0] & STMFTS_MASK_TOUCH_ID) >> 4;
	u16 x = event[1] | ((event[2] & STMFTS_MASK_X_MSB) << 8);
	u16 y = (event[2] >> 4) | (event[3] << 4);
	u8 maj = event[4];
	u8 min = event[5];
	u8 orientation = event[6];
	u8 area = event[7];

	if (slot_id >= STMFTS_MAX_FINGERS) {
		dev_warn(sdata->dev,
			 "invalid slot ID %u\n", slot_id);
		return;
	}

	input_mt_slot(sdata->input, slot_id);

	input_mt_report_slot_state(sdata->input, MT_TOOL_FINGER, true);
	touchscreen_report_pos(sdata->input, &sdata->prop, x, y, true);
	input_report_abs(sdata->input, ABS_MT_TOUCH_MAJOR, maj);
	input_report_abs(sdata->input, ABS_MT_TOUCH_MINOR, min);
	input_report_abs(sdata->input, ABS_MT_PRESSURE, area);
	input_report_abs(sdata->input, ABS_MT_ORIENTATION, orientation);

	input_sync(sdata->input);
}

static void stmfts_report_contact_release(struct stmfts_data *sdata,
					  const u8 event[])
{
	u8 slot_id = (event[0] & STMFTS_MASK_TOUCH_ID) >> 4;

	if (slot_id >= STMFTS_MAX_FINGERS) {
		dev_warn(sdata->dev,
			 "invalid slot ID %u\n", slot_id);
		return;
	}

	input_mt_slot(sdata->input, slot_id);
	input_mt_report_slot_inactive(sdata->input);

	input_sync(sdata->input);
}

/* FTS5 event handling functions */
static void stmfts5_report_contact_event(struct stmfts_data *sdata,
					 const u8 event[])
{
	/*
	 * FTS5 touch event format:
	 * event[0] = event ID (high nibble) | major bits 5:4 (bits 3:2)
	 * event[1] = touch ID (high nibble) | touch type (low nibble)
	 * event[2] = X bits 7:0
	 * event[3] = Y bits 3:0 (high nibble) | X bits 11:8 (low nibble)
	 * event[4] = Y bits 11:4
	 * event[5] = pressure
	 * event[6] = major bits 3:0 (high nibble) | minor bits 3:0 (low nibble)
	 * event[7] = minor bits 5:4 (bits 7:6)
	 */
	u8 touch_id = (event[1] & STMFTS_MASK_TOUCH_ID) >> 4;
	u8 touch_type = event[1] & STMFTS5_MASK_TOUCH_TYPE;
	u16 x = ((event[3] & STMFTS_MASK_X_MSB) << 8) | event[2];
	u16 y = (event[4] << 4) | ((event[3] & STMFTS_MASK_Y_LSB) >> 4);
	u8 area = event[5];
	u8 maj = ((event[0] & STMFTS5_MASK_MAJOR_MSB) << 2) | (event[6] >> 4);
	u8 min = ((event[7] & STMFTS5_MASK_MINOR_MSB) >> 2) | (event[6] & 0x0f);
	/* Distance is 0 for touching, max for hovering */
	int distance = 0;
	unsigned int tool;

	if (touch_id >= STMFTS_MAX_FINGERS) {
		dev_warn(sdata->dev,
			 "invalid slot ID %u\n", touch_id);
		return;
	}

	switch (touch_type) {
	case STMFTS_TOUCH_TYPE_STYLUS:
		tool = MT_TOOL_PEN;
		break;

	case STMFTS_TOUCH_TYPE_FINGER:
	case STMFTS_TOUCH_TYPE_GLOVE:
		tool = MT_TOOL_FINGER;
		break;

	case STMFTS_TOUCH_TYPE_PALM:
		/* Palm touch - report but can be filtered by userspace */
		tool = MT_TOOL_PALM;
		break;

	case STMFTS_TOUCH_TYPE_HOVER:
		tool = MT_TOOL_FINGER;
		distance = 255;
		break;

	case STMFTS_TOUCH_TYPE_INVALID:
	default:
		dev_warn(sdata->dev,
			 "invalid touch type %d for slot %d\n",
			 touch_type, touch_id);
		return;
	}

	if (touch_type == STMFTS_TOUCH_TYPE_HOVER) {
		area = 0;
	} else if (!area) {
		/*
		 * Should not happen for contact events. Set minimum pressure
		 * to prevent touch from being dropped.
		 */
		dev_warn_once(sdata->dev,
			      "zero pressure on contact event, slot %d\n",
			      touch_id);
		area = 1;
	}

	input_mt_slot(sdata->input, touch_id);
	input_mt_report_slot_state(sdata->input, tool, true);

	touchscreen_report_pos(sdata->input, &sdata->prop, x, y, true);
	input_report_abs(sdata->input, ABS_MT_TOUCH_MAJOR, maj);
	input_report_abs(sdata->input, ABS_MT_TOUCH_MINOR, min);
	input_report_abs(sdata->input, ABS_MT_PRESSURE, area);
	input_report_abs(sdata->input, ABS_MT_DISTANCE, distance);
}

static void stmfts5_report_contact_release(struct stmfts_data *sdata,
					   const u8 event[])
{
	/* format: touch ID is in high 4 bits of event[1] */
	u8 touch_id = (event[1] & STMFTS_MASK_TOUCH_ID) >> 4;

	if (touch_id >= STMFTS_MAX_FINGERS) {
		dev_warn(sdata->dev,
			 "invalid slot ID %u\n", touch_id);
		return;
	}

	input_mt_slot(sdata->input, touch_id);
	input_mt_report_slot_inactive(sdata->input);
}

static void stmfts_report_hover_event(struct stmfts_data *sdata,
				      const u8 event[])
{
	u16 x = (event[2] << 4) | (event[4] >> 4);
	u16 y = (event[3] << 4) | (event[4] & STMFTS_MASK_Y_LSB);
	u8 z = event[5];

	touchscreen_report_pos(sdata->input, &sdata->prop, x, y, false);
	input_report_abs(sdata->input, ABS_DISTANCE, z);

	input_sync(sdata->input);
}

static void stmfts_report_key_event(struct stmfts_data *sdata, const u8 event[])
{
	switch (event[2]) {
	case 0:
		input_report_key(sdata->input, KEY_BACK, 0);
		input_report_key(sdata->input, KEY_MENU, 0);
		break;

	case STMFTS_MASK_KEY_BACK:
		input_report_key(sdata->input, KEY_BACK, 1);
		break;

	case STMFTS_MASK_KEY_MENU:
		input_report_key(sdata->input, KEY_MENU, 1);
		break;

	default:
		dev_warn(sdata->dev,
			 "unknown key event: %#02x\n", event[2]);
		break;
	}

	input_sync(sdata->input);
}

static void stmfts_parse_events(struct stmfts_data *sdata)
{
	int i;

	for (i = 0; i < STMFTS_STACK_DEPTH; i++) {
		u8 *event = &sdata->data[i * STMFTS_EVENT_SIZE];

		switch (event[0]) {
		case STMFTS_EV_NO_EVENT:
			return;

		case STMFTS_EV_CONTROLLER_READY:
		case STMFTS_EV_SLEEP_OUT_CONTROLLER_READY:
		case STMFTS_EV_STATUS:
			complete(&sdata->cmd_done);
			fallthrough;

		case STMFTS_EV_DEBUG:
			continue;
		}

		switch (event[0] & STMFTS_MASK_EVENT_ID) {
		case STMFTS_EV_MULTI_TOUCH_ENTER:
		case STMFTS_EV_MULTI_TOUCH_MOTION:
			stmfts_report_contact_event(sdata, event);
			break;

		case STMFTS_EV_MULTI_TOUCH_LEAVE:
			stmfts_report_contact_release(sdata, event);
			break;

		case STMFTS_EV_HOVER_ENTER:
		case STMFTS_EV_HOVER_LEAVE:
		case STMFTS_EV_HOVER_MOTION:
			stmfts_report_hover_event(sdata, event);
			break;

		case STMFTS_EV_KEY_STATUS:
			stmfts_report_key_event(sdata, event);
			break;

		case STMFTS_EV_ERROR:
			dev_warn(sdata->dev,
				 "error code: 0x%x%x%x%x%x%x",
				 event[6], event[5], event[4],
				 event[3], event[2], event[1]);
			break;

		default:
			dev_err(sdata->dev,
				"unknown event %#02x\n", event[0]);
		}
	}
}

static void stmfts5_parse_events(struct stmfts_data *sdata)
{
	for (int i = 0; i < STMFTS_STACK_DEPTH; i++) {
		u8 *event = &sdata->data[i * STMFTS_EVENT_SIZE];

		switch (event[0]) {
		case STMFTS_EV_NO_EVENT:
			goto sync;

		case STMFTS5_EV_CONTROLLER_READY:
			complete(&sdata->cmd_done);
			continue;
		}

		switch (event[0] & STMFTS5_MASK_EVENT_ID) {
		case STMFTS5_EV_STATUS_UPDATE:
		case STMFTS5_EV_USER_REPORT:
		case STMFTS5_EV_DEBUG:
			continue;

		case STMFTS5_EV_MULTI_TOUCH_ENTER:
		case STMFTS5_EV_MULTI_TOUCH_MOTION:
			stmfts5_report_contact_event(sdata, event);
			break;

		case STMFTS5_EV_MULTI_TOUCH_LEAVE:
			stmfts5_report_contact_release(sdata, event);
			break;

		case STMFTS5_EV_ERROR:
			dev_warn(sdata->dev,
				 "error code: 0x%02x%02x%02x%02x%02x%02x\n",
				 event[6], event[5], event[4],
				 event[3], event[2], event[1]);
			break;

		default:
			dev_err(sdata->dev,
				"unknown FTS5 event %#02x\n", event[0]);
		}
	}

sync:
	input_mt_sync_frame(sdata->input);
	input_sync(sdata->input);
}

static irqreturn_t stmfts_irq_handler(int irq, void *dev)
{
	struct stmfts_data *sdata = dev;
	int err;

	guard(mutex)(&sdata->mutex);

	err = stmfts_read_events(sdata);
	if (unlikely(err))
		dev_err(sdata->dev,
			"failed to read events: %d\n", err);
	else
		sdata->ops->parse_events(sdata);

	return IRQ_HANDLED;
}

static int stmfts_command(struct stmfts_data *sdata, const u8 cmd)
{
	int err;

	reinit_completion(&sdata->cmd_done);

	err = stmfts_write_byte(sdata, cmd);
	if (err)
		return err;

	if (!wait_for_completion_timeout(&sdata->cmd_done,
					 msecs_to_jiffies(1000)))
		return -ETIMEDOUT;

	return 0;
}

static int stmfts_set_scan(struct stmfts_data *sdata, bool enable)
{
	return stmfts_write_byte(sdata, enable ? STMFTS_MS_MT_SENSE_ON :
					     STMFTS_MS_MT_SENSE_OFF);
}

static int stmfts_set_hover(struct stmfts_data *sdata, bool enable)
{
	return stmfts_write_byte(sdata, enable ? STMFTS_SS_HOVER_SENSE_ON :
					     STMFTS_SS_HOVER_SENSE_OFF);
}

static int stmfts5_set_scan(struct stmfts_data *sdata, bool enable)
{
	u8 scan_mode_cmd[3] = { STMFTS5_SET_SCAN_MODE, STMFTS5_SCAN_MODE_ACTIVE,
				enable ? STMFTS5_SCAN_ACTIVE_MULTI_TOUCH : 0 };

	return stmfts_write(sdata, scan_mode_cmd, sizeof(scan_mode_cmd));
}

static int stmfts_input_open(struct input_dev *dev)
{
	struct stmfts_data *sdata = input_get_drvdata(dev);
	int err;

	err = pm_runtime_resume_and_get(sdata->dev);
	if (err)
		return err;

	err = sdata->ops->set_scan(sdata, true);
	if (err) {
		pm_runtime_put_sync(sdata->dev);
		return err;
	}

	scoped_guard(mutex, &sdata->mutex) {
		sdata->running = true;

		if (sdata->hover_enabled && sdata->ops->set_hover) {
			err = sdata->ops->set_hover(sdata, true);
			if (err)
				dev_warn(sdata->dev,
					 "failed to enable hover\n");
		}
	}

	if (sdata->use_key) {
		err = stmfts_write_byte(sdata, STMFTS_MS_KEY_SENSE_ON);
		if (err)
			/* I can still use only the touch screen */
			dev_warn(sdata->dev,
				 "failed to enable touchkey\n");
	}

	return 0;
}

static void stmfts_input_close(struct input_dev *dev)
{
	struct stmfts_data *sdata = input_get_drvdata(dev);
	int err;

	err = sdata->ops->set_scan(sdata, false);
	if (err)
		dev_warn(sdata->dev,
			 "failed to disable touchscreen: %d\n", err);

	scoped_guard(mutex, &sdata->mutex) {
		sdata->running = false;

		if (sdata->hover_enabled && sdata->ops->set_hover) {
			err = sdata->ops->set_hover(sdata, false);
			if (err)
				dev_warn(sdata->dev,
					 "failed to disable hover: %d\n", err);
		}
	}

	if (sdata->use_key) {
		err = stmfts_write_byte(sdata, STMFTS_MS_KEY_SENSE_OFF);
		if (err)
			dev_warn(sdata->dev,
				 "failed to disable touchkey: %d\n", err);
	}

	pm_runtime_put_sync(sdata->dev);
}

static ssize_t stmfts_sysfs_chip_id(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%#x\n", sdata->chip_id);
}

static ssize_t stmfts_sysfs_chip_version(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", sdata->chip_ver);
}

static ssize_t stmfts_sysfs_fw_ver(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", sdata->fw_ver);
}

static ssize_t stmfts_sysfs_config_id(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%#x\n", sdata->config_id);
}

static ssize_t stmfts_sysfs_config_version(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", sdata->config_ver);
}

static ssize_t stmfts_sysfs_read_status(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);
	u8 status[4];
	int err;

	err = stmfts_read(sdata, STMFTS_READ_STATUS, status, sizeof(status));
	if (err)
		return err;

	return sysfs_emit(buf, "%#02x\n", status[0]);
}

static ssize_t stmfts_sysfs_hover_enable_read(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", sdata->hover_enabled);
}

static ssize_t stmfts_sysfs_hover_enable_write(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t len)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);
	unsigned long value;
	bool hover;
	int err;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	hover = !!value;

	guard(mutex)(&sdata->mutex);

	if (hover != sdata->hover_enabled) {
		if (sdata->running && sdata->ops->set_hover) {
			err = sdata->ops->set_hover(sdata, hover);
			if (err)
				return err;
		}

		sdata->hover_enabled = hover;
	}

	return len;
}

static DEVICE_ATTR(chip_id, 0444, stmfts_sysfs_chip_id, NULL);
static DEVICE_ATTR(chip_version, 0444, stmfts_sysfs_chip_version, NULL);
static DEVICE_ATTR(fw_ver, 0444, stmfts_sysfs_fw_ver, NULL);
static DEVICE_ATTR(config_id, 0444, stmfts_sysfs_config_id, NULL);
static DEVICE_ATTR(config_version, 0444, stmfts_sysfs_config_version, NULL);
static DEVICE_ATTR(status, 0444, stmfts_sysfs_read_status, NULL);
static DEVICE_ATTR(hover_enable, 0644, stmfts_sysfs_hover_enable_read,
					stmfts_sysfs_hover_enable_write);

static struct attribute *stmfts_sysfs_attrs[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_chip_version.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_config_id.attr,
	&dev_attr_config_version.attr,
	&dev_attr_status.attr,
	&dev_attr_hover_enable.attr,
	NULL
};
ATTRIBUTE_GROUPS(stmfts_sysfs);

static int stmfts_read_system_info(struct stmfts_data *sdata)
{
	int err;
	u8 reg[8];

	err = stmfts_read(sdata, STMFTS_READ_INFO, reg, sizeof(reg));
	if (err)
		return err;

	sdata->chip_id = be16_to_cpup((__be16 *)&reg[6]);
	sdata->chip_ver = reg[0];
	sdata->fw_ver = be16_to_cpup((__be16 *)&reg[2]);
	sdata->config_id = reg[4];
	sdata->config_ver = reg[5];

	return 0;
}

static int stmfts_configure(struct stmfts_data *sdata)
{
	int err;

	err = stmfts_read_system_info(sdata);
	if (err)
		return err;

	err = stmfts_command(sdata, STMFTS_SYSTEM_RESET);
	if (err)
		return err;

	err = stmfts_command(sdata, STMFTS_SLEEP_OUT);
	if (err)
		return err;

	/* optional tuning */
	err = stmfts_command(sdata, STMFTS_MS_CX_TUNING);
	if (err)
		dev_warn(sdata->dev,
			 "failed to perform mutual auto tune: %d\n", err);

	/* optional tuning */
	err = stmfts_command(sdata, STMFTS_SS_CX_TUNING);
	if (err)
		dev_warn(sdata->dev,
			 "failed to perform self auto tune: %d\n", err);

	err = stmfts_command(sdata, STMFTS_FULL_FORCE_CALIBRATION);
	if (err)
		return err;

	return 0;
}

static int stmfts_power_on(struct stmfts_data *sdata)
{
	unsigned int timeout;
	int err;

	if (sdata->reset_gpio) {
		gpiod_set_value_cansleep(sdata->reset_gpio, 1);
		/* a short delay before powering up */
		usleep_range(1000, 1500);
	}

	err = regulator_bulk_enable(ARRAY_SIZE(stmfts_supplies),
				    sdata->supplies);
	if (err)
		return err;

	reinit_completion(&sdata->cmd_done);

	if (sdata->reset_gpio) {
		/*
		 * The datasheet does not specify the power on time, but
		 * considering that the reset time is < 10ms, sleep for 20ms
		 * to be sure before releasing reset line.
		 */
		msleep(20);
		gpiod_set_value_cansleep(sdata->reset_gpio, 0);

		timeout = STMFTS_RESET_TIMEOUT_MS;
	} else {
		/*
		 * We do not know the real controller state (was it powered
		 * off or reset). Let's hope that this is enough time to
		 * initialize.
		 */
		timeout = 70;
	}

	enable_irq(sdata->irq);

	if (!wait_for_completion_timeout(&sdata->cmd_done,
					 msecs_to_jiffies(timeout))) {
		if (sdata->reset_gpio) {
			dev_err(sdata->dev, "controller not ready after reset\n");
			err = -ETIMEDOUT;
			goto err_disable_irq;
		}
		/*
		 * If we did not reset the controller ourselves continue
		 * even if we did not receive "ready" message.
		 */
	}

	err = sdata->ops->configure(sdata);
	if (err)
		goto err_disable_irq;

	/*
	 * At this point no one is using the touchscreen
	 * and I don't really care about the return value
	 */
	(void)stmfts_write_byte(sdata, STMFTS_SLEEP_IN);

	return 0;

err_disable_irq:
	disable_irq(sdata->irq);

	regulator_bulk_disable(ARRAY_SIZE(stmfts_supplies), sdata->supplies);
	return err;
}

static int stmfts5_configure(struct stmfts_data *sdata)
{
	u8 event[STMFTS_EVENT_SIZE];

	/* Verify bus communication */
	return stmfts_read(sdata, STMFTS_READ_ALL_EVENT, event, sizeof(event));
}

static void stmfts_power_off(void *data)
{
	struct stmfts_data *sdata = data;

	disable_irq(sdata->irq);

	if (sdata->reset_gpio)
		gpiod_set_value_cansleep(sdata->reset_gpio, 1);

	regulator_bulk_disable(ARRAY_SIZE(stmfts_supplies), sdata->supplies);
}

static int stmfts_setup_input(struct stmfts_data *sdata)
{
	struct device *dev = sdata->dev;

	input_set_abs_params(sdata->input, ABS_MT_ORIENTATION, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_DISTANCE, 0, 255, 0, 0);

	sdata->use_key = device_property_read_bool(dev, "touch-key-connected");
	if (sdata->use_key) {
		input_set_capability(sdata->input, EV_KEY, KEY_MENU);
		input_set_capability(sdata->input, EV_KEY, KEY_BACK);
	}

	return input_mt_init_slots(sdata->input, STMFTS_MAX_FINGERS,
				   INPUT_MT_DIRECT);
}

static int stmfts5_setup_input(struct stmfts_data *sdata)
{
	struct device *dev = sdata->dev;

	sdata->mode_switch_gpio = devm_gpiod_get_optional(dev, "mode-switch",
							  GPIOD_OUT_HIGH);
	if (IS_ERR(sdata->mode_switch_gpio))
		return dev_err_probe(dev, PTR_ERR(sdata->mode_switch_gpio),
				     "Failed to get GPIO 'mode-switch'\n");

	/* Set resolution for accurate calibration */
	if (!input_abs_get_res(sdata->input, ABS_MT_POSITION_X))
		input_abs_set_res(sdata->input, ABS_MT_POSITION_X, 10);
	if (!input_abs_get_res(sdata->input, ABS_MT_POSITION_Y))
		input_abs_set_res(sdata->input, ABS_MT_POSITION_Y, 10);

	input_set_abs_params(sdata->input, ABS_MT_DISTANCE, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOOL_TYPE, 0,
			     MT_TOOL_PALM, 0, 0);

	return input_mt_init_slots(sdata->input, STMFTS_MAX_FINGERS,
				   INPUT_MT_DIRECT);
}

static int stmfts_enable_led(struct stmfts_data *sdata)
{
	int err;

	/* get the regulator for powering the leds on */
	sdata->ledvdd = devm_regulator_get(sdata->dev, "ledvdd");
	if (IS_ERR(sdata->ledvdd))
		return PTR_ERR(sdata->ledvdd);

	sdata->led_cdev.name = STMFTS_DEV_NAME;
	sdata->led_cdev.max_brightness = LED_ON;
	sdata->led_cdev.brightness = LED_OFF;
	sdata->led_cdev.brightness_set_blocking = stmfts_brightness_set;
	sdata->led_cdev.brightness_get = stmfts_brightness_get;

	err = devm_led_classdev_register(sdata->dev, &sdata->led_cdev);
	if (err) {
		devm_regulator_put(sdata->ledvdd);
		return err;
	}

	return 0;
}

static int stmfts_probe(struct stmfts_data *sdata, u16 bustype)
{
	struct device *dev = sdata->dev;
	int err;

	dev_set_drvdata(dev, sdata);
	mutex_init(&sdata->mutex);
	init_completion(&sdata->cmd_done);

	if (!sdata->ops)
		return -EINVAL;

	err = devm_regulator_bulk_get_const(dev,
					    ARRAY_SIZE(stmfts_supplies),
					    stmfts_supplies,
					    &sdata->supplies);
	if (err)
		return err;

	sdata->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sdata->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sdata->reset_gpio),
				     "Failed to get GPIO 'reset'\n");

	sdata->input = devm_input_allocate_device(dev);
	if (!sdata->input)
		return -ENOMEM;

	sdata->input->name = STMFTS_DEV_NAME;
	sdata->input->id.bustype = bustype;
	sdata->input->open = stmfts_input_open;
	sdata->input->close = stmfts_input_close;

	input_set_capability(sdata->input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(sdata->input, EV_ABS, ABS_MT_POSITION_Y);
	touchscreen_parse_properties(sdata->input, true, &sdata->prop);

	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	err = sdata->ops->setup_input(sdata);
	if (err)
		return err;

	input_set_drvdata(sdata->input, sdata);

	/*
	 * stmfts_power_on expects interrupt to be disabled, but
	 * at this point the device is still off and I do not trust
	 * the status of the irq line that can generate some spurious
	 * interrupts. To be on the safe side it's better to not enable
	 * the interrupts during their request.
	 */
	err = devm_request_threaded_irq(dev, sdata->irq,
					NULL, stmfts_irq_handler,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"stmfts_irq", sdata);
	if (err)
		return err;

	dev_dbg(dev, "initializing ST-Microelectronics FTS...\n");

	err = stmfts_power_on(sdata);
	if (err)
		return err;

	err = devm_add_action_or_reset(dev, stmfts_power_off, sdata);
	if (err)
		return err;

	err = input_register_device(sdata->input);
	if (err)
		return err;

	if (sdata->use_key) {
		err = stmfts_enable_led(sdata);
		if (err) {
			/*
			 * Even if the LEDs have failed to be initialized and
			 * used in the driver, I can still use the device even
			 * without LEDs. The ledvdd regulator pointer will be
			 * used as a flag.
			 */
			dev_warn(dev, "unable to use touchkey leds\n");
			sdata->ledvdd = NULL;
		}
	}

	pm_runtime_enable(dev);
	device_enable_async_suspend(dev);

	return 0;
}

static void stmfts_remove(struct device *dev)
{
	pm_runtime_disable(dev);
}

static int stmfts_runtime_suspend(struct device *dev)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);
	int ret;

	ret = stmfts_write_byte(sdata, STMFTS_SLEEP_IN);
	if (ret)
		dev_warn(dev, "failed to suspend device: %d\n", ret);

	return ret;
}

static int stmfts_chip_runtime_resume(struct stmfts_data *sdata)
{
	return stmfts_write_byte(sdata, STMFTS_SLEEP_OUT);
}

static int stmfts5_chip_runtime_resume(struct stmfts_data *sdata)
{
	struct device *dev = sdata->dev;
	u8 int_enable_cmd[4] = { 0xB6, 0x00, 0x2C, 0x01 };
	int err;

	err = stmfts_write_byte(sdata, STMFTS_SLEEP_OUT);
	if (err)
		return err;

	msleep(20);

	/* Perform capacitance tuning after wakeup */
	err = stmfts_write_byte(sdata, STMFTS_MS_CX_TUNING);
	if (err)
		dev_warn(dev, "MS_CX_TUNING failed: %d\n", err);
	msleep(20);

	err = stmfts_write_byte(sdata, STMFTS_SS_CX_TUNING);
	if (err)
		dev_warn(dev, "SS_CX_TUNING failed: %d\n", err);
	msleep(20);

	/* Force calibration */
	err = stmfts_write_byte(sdata, STMFTS_FULL_FORCE_CALIBRATION);
	if (err)
		dev_warn(dev, "FORCE_CALIBRATION failed: %d\n", err);
	msleep(50);

	/* Enable controller interrupts */
	err = stmfts_write(sdata, int_enable_cmd, sizeof(int_enable_cmd));
	if (err)
		return err;

	msleep(20);

	return 0;
}

static int stmfts_runtime_resume(struct device *dev)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);
	int err;

	err = sdata->ops->runtime_resume(sdata);
	if (err)
		dev_err(dev, "failed to resume device: %d\n", err);

	return err;
}

static int stmfts_suspend(struct device *dev)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	stmfts_power_off(sdata);

	return 0;
}

static int stmfts_resume(struct device *dev)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return stmfts_power_on(sdata);
}

static const struct dev_pm_ops stmfts_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(stmfts_suspend, stmfts_resume)
	RUNTIME_PM_OPS(stmfts_runtime_suspend, stmfts_runtime_resume, NULL)
};

static const struct stmfts_chip_ops stmfts4_ops = {
	.configure	= stmfts_configure,
	.setup_input	= stmfts_setup_input,
	.parse_events	= stmfts_parse_events,
	.set_scan	= stmfts_set_scan,
	.set_hover	= stmfts_set_hover,
	.runtime_resume	= stmfts_chip_runtime_resume,
};

static const struct stmfts_chip_ops stmfts5_ops = {
	.configure	= stmfts5_configure,
	.setup_input	= stmfts5_setup_input,
	.parse_events	= stmfts5_parse_events,
	.set_scan	= stmfts5_set_scan,
	.runtime_resume	= stmfts5_chip_runtime_resume,
};

#if IS_ENABLED(CONFIG_I2C)
static int stmfts_i2c_write(struct stmfts_data *sdata, const u8 *buf,
			    size_t len)
{
	int ret;

	ret = i2c_master_send(sdata->client, buf, len);
	if (ret < 0)
		return ret;

	return ret == len ? 0 : -EIO;
}

/*
 * Write the command and read the reply with a repeated start. A plain
 * i2c_smbus_read_i2c_block_data() cannot be used for the event stack,
 * which is 256 bytes and exceeds the 255-byte SMBus block limit.
 */
static int stmfts_i2c_read(struct stmfts_data *sdata, u8 cmd, u8 *buf,
			   size_t len)
{
	struct i2c_msg msgs[2] = {
		{
			.addr	= sdata->client->addr,
			.len	= 1,
			.buf	= &cmd,
		},
		{
			.addr	= sdata->client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		},
	};
	int ret;

	ret = i2c_transfer(sdata->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

static const struct stmfts_bus_ops stmfts_i2c_bus_ops = {
	.write	= stmfts_i2c_write,
	.read	= stmfts_i2c_read,
};

static int stmfts_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct stmfts_data *sdata;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	sdata = devm_kzalloc(dev, sizeof(*sdata), GFP_KERNEL);
	if (!sdata)
		return -ENOMEM;

	sdata->dev = dev;
	sdata->irq = client->irq;
	sdata->client = client;
	sdata->bus = &stmfts_i2c_bus_ops;
	sdata->ops = i2c_get_match_data(client);

	return stmfts_probe(sdata, BUS_I2C);
}

static void stmfts_i2c_remove(struct i2c_client *client)
{
	stmfts_remove(&client->dev);
}

#ifdef CONFIG_OF
static const struct of_device_id stmfts_of_match[] = {
	{ .compatible = "st,stmfts",	.data = &stmfts4_ops },
	{ .compatible = "st,stmfts5",	.data = &stmfts5_ops },
	{ },
};
MODULE_DEVICE_TABLE(of, stmfts_of_match);
#endif

static const struct i2c_device_id stmfts_id[] = {
	{ .name = "stmfts",	.driver_data = (kernel_ulong_t)&stmfts4_ops },
	{ .name = "stmfts5",	.driver_data = (kernel_ulong_t)&stmfts5_ops },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stmfts_id);

static struct i2c_driver stmfts_driver = {
	.driver = {
		.name = STMFTS_DEV_NAME,
		.dev_groups = stmfts_sysfs_groups,
		.of_match_table = of_match_ptr(stmfts_of_match),
		.pm = pm_ptr(&stmfts_pm_ops),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = stmfts_i2c_probe,
	.remove = stmfts_i2c_remove,
	.id_table = stmfts_id,
};

#endif /* CONFIG_I2C */

#if IS_ENABLED(CONFIG_SPI_MASTER)
/*
 * On SPI, FTS5 (FTM5) takes the same write commands as on I2C, but each
 * read opcode is the I2C one plus one (event stack 0x86 -> 0x87) and the
 * reply starts with a dummy byte. The command and the reply share one
 * chip select, held for 10 us after the last byte.
 */
#define STMFTS_SPI_CS_DELAY_US	10
#define STMFTS_SPI_TX_SIZE	8

static int stmfts_spi_write(struct stmfts_data *sdata, const u8 *buf,
			    size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = sdata->spi_tx,
		.len = len,
		.delay = {
			.value = STMFTS_SPI_CS_DELAY_US,
			.unit = SPI_DELAY_UNIT_USECS,
		},
	};

	if (len > STMFTS_SPI_TX_SIZE)
		return -EINVAL;

	guard(mutex)(&sdata->spi_lock);

	memcpy(sdata->spi_tx, buf, len);

	return spi_sync_transfer(sdata->spi, &xfer, 1);
}

static int stmfts_spi_read(struct stmfts_data *sdata, u8 cmd, u8 *buf,
			   size_t len)
{
	struct spi_transfer xfers[2] = {
		{
			.tx_buf = sdata->spi_tx,
			.len = 1,
		},
		{
			.rx_buf = sdata->spi_rx,
			.len = len + 1,
			.delay = {
				.value = STMFTS_SPI_CS_DELAY_US,
				.unit = SPI_DELAY_UNIT_USECS,
			},
		},
	};
	int err;

	if (len > STMFTS_DATA_MAX_SIZE)
		return -EINVAL;

	guard(mutex)(&sdata->spi_lock);

	sdata->spi_tx[0] = cmd + 1;

	err = spi_sync_transfer(sdata->spi, xfers, ARRAY_SIZE(xfers));
	if (err)
		return err;

	memcpy(buf, sdata->spi_rx + 1, len);

	return 0;
}

static const struct stmfts_bus_ops stmfts_spi_bus_ops = {
	.write	= stmfts_spi_write,
	.read	= stmfts_spi_read,
};

static int stmfts_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct stmfts_data *sdata;
	int err;

	spi->bits_per_word = 8;
	err = spi_setup(spi);
	if (err)
		return err;

	sdata = devm_kzalloc(dev, sizeof(*sdata), GFP_KERNEL);
	if (!sdata)
		return -ENOMEM;

	sdata->spi_tx = devm_kzalloc(dev, STMFTS_SPI_TX_SIZE, GFP_KERNEL);
	sdata->spi_rx = devm_kzalloc(dev, STMFTS_DATA_MAX_SIZE + 1, GFP_KERNEL);
	if (!sdata->spi_tx || !sdata->spi_rx)
		return -ENOMEM;

	err = devm_mutex_init(dev, &sdata->spi_lock);
	if (err)
		return err;

	sdata->dev = dev;
	sdata->irq = spi->irq;
	sdata->spi = spi;
	sdata->bus = &stmfts_spi_bus_ops;
	sdata->ops = spi_get_device_match_data(spi);

	return stmfts_probe(sdata, BUS_SPI);
}

static void stmfts_spi_remove(struct spi_device *spi)
{
	stmfts_remove(&spi->dev);
}

/* Only FTS5 has a known SPI protocol */
static const struct of_device_id stmfts_spi_of_match[] = {
	{ .compatible = "st,stmfts5",	.data = &stmfts5_ops },
	{ },
};
MODULE_DEVICE_TABLE(of, stmfts_spi_of_match);

static const struct spi_device_id stmfts_spi_id[] = {
	{ .name = "stmfts5",	.driver_data = (kernel_ulong_t)&stmfts5_ops },
	{ }
};
MODULE_DEVICE_TABLE(spi, stmfts_spi_id);

static struct spi_driver stmfts_spi_driver = {
	.driver = {
		.name = STMFTS_DEV_NAME,
		.dev_groups = stmfts_sysfs_groups,
		.of_match_table = stmfts_spi_of_match,
		.pm = pm_ptr(&stmfts_pm_ops),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = stmfts_spi_probe,
	.remove = stmfts_spi_remove,
	.id_table = stmfts_spi_id,
};
#endif /* CONFIG_SPI_MASTER */

static int __init stmfts_init(void)
{
	int err = 0;

#if IS_ENABLED(CONFIG_I2C)
	err = i2c_add_driver(&stmfts_driver);
	if (err)
		return err;
#endif
#if IS_ENABLED(CONFIG_SPI_MASTER)
	err = spi_register_driver(&stmfts_spi_driver);
#if IS_ENABLED(CONFIG_I2C)
	if (err)
		i2c_del_driver(&stmfts_driver);
#endif
#endif
	return err;
}
module_init(stmfts_init);

static void __exit stmfts_exit(void)
{
#if IS_ENABLED(CONFIG_SPI_MASTER)
	spi_unregister_driver(&stmfts_spi_driver);
#endif
#if IS_ENABLED(CONFIG_I2C)
	i2c_del_driver(&stmfts_driver);
#endif
}
module_exit(stmfts_exit);

MODULE_AUTHOR("Andi Shyti <andi.shyti@samsung.com>");
MODULE_AUTHOR("David Heidelberg <david@ixit.cz>");
MODULE_AUTHOR("Petr Hodina <petr.hodina@protonmail.com>");
MODULE_DESCRIPTION("STMicroelectronics FTS Touch Screen");
MODULE_LICENSE("GPL");
