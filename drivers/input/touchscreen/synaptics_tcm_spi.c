// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Synaptics TouchComm SPI touchscreen driver
 *
 * Copyright (C) 2017-2019 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2017-2019 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/*
 * The controller speaks TouchComm v1. A command is written as
 *
 *	[ command | length (le16) | payload ]
 *
 * and continued in CONTINUE_WRITE chunks when it exceeds the maximum write
 * size that the controller announces in its identify report. A message,
 * either the response to a command or a report, is read as
 *
 *	[ 0xa5 | code | length (le16) | payload | 0x5a ]
 *
 * where codes below 0x10 are command statuses and the others report ids.
 * When a read was too short for the message, the rest is read with a header
 * of [ 0xa5 | 0x03 ] in front of it. Each read is as long as the previous
 * message, so that a stream of equal reports needs one transfer each. On
 * SPI the controller is sent 0xff while it is read.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <drm/drm_panel.h>

#include "synaptics_tcm_gesture.h"
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define SYNA_TCM_INPUT_NAME			"synaptics_tcm_touch"

#define SYNA_TCM_MESSAGE_MARKER			0xa5
#define SYNA_TCM_MESSAGE_PADDING		0x5a
#define SYNA_TCM_HEADER_SIZE			4
#define SYNA_TCM_MIN_READ_LENGTH		9
#define SYNA_TCM_SPI_FILL			0xff

#define SYNA_TCM_CMD_NONE			0x00
#define SYNA_TCM_CMD_CONTINUE_WRITE		0x01
#define SYNA_TCM_CMD_RESET			0x04
#define SYNA_TCM_CMD_RUN_APPLICATION_FIRMWARE	0x14
#define SYNA_TCM_CMD_GET_APPLICATION_INFO	0x20
#define SYNA_TCM_CMD_GET_DYNAMIC_CONFIG		0x23
#define SYNA_TCM_CMD_SET_DYNAMIC_CONFIG		0x24
#define SYNA_TCM_CMD_GET_TOUCH_REPORT_CONFIG	0x25
#define SYNA_TCM_CMD_SET_TOUCH_REPORT_CONFIG	0x26
#define SYNA_TCM_CMD_ENTER_DEEP_SLEEP		0x2c
#define SYNA_TCM_DC_GESTURE			0x09
#define SYNA_TCM_DC_TOUCH_HOLD			0xd4
#define SYNA_TCM_DC_REPORT_RATE			0xe6
#define SYNA_TCM_DC_GESTURE_MASK			0xfe

#define SYNA_TCM_STATUS_IDLE			0x00
#define SYNA_TCM_STATUS_OK			0x01
#define SYNA_TCM_STATUS_BUSY			0x02
#define SYNA_TCM_STATUS_CONTINUED_READ		0x03
#define SYNA_TCM_STATUS_ERROR			0x0f
#define SYNA_TCM_STATUS_INVALID			0xff

#define SYNA_TCM_REPORT_IDENTIFY		0x10
#define SYNA_TCM_REPORT_TOUCH			0x11

#define SYNA_TCM_MODE_APPLICATION		0x01
#define SYNA_TCM_MODE_HOST_DOWNLOAD		0x02

#define SYNA_TCM_APP_STATUS_OK			0x00
#define SYNA_TCM_APP_STATUS_BOOTING		0x01
#define SYNA_TCM_APP_STATUS_UPDATING		0x02

#define SYNA_TCM_OBJECT_FINGER			1
#define SYNA_TCM_OBJECT_GLOVED_FINGER		2

/* The smallest report layout that the firmware has to accept. */
#define SYNA_TCM_TOUCH_REPORT_CONFIG_SIZE	128

#define SYNA_TCM_POWER_DELAY_MS			200
#define SYNA_TCM_RESET_ACTIVE_US		10000
#define SYNA_TCM_RESET_DELAY_MS			80
#define SYNA_TCM_IRQ_ENABLE_DELAY_MS		20
#define SYNA_TCM_RESPONSE_TIMEOUT_MS		3000
#define SYNA_TCM_APP_STATUS_POLL_MS		100
#define SYNA_TCM_APP_STATUS_TIMEOUT_MS		1000
#define SYNA_TCM_RESUME_DELAY_MS		100
#define SYNA_TCM_READ_RETRY_US			5000
#define SYNA_TCM_WRITE_DELAY_US			500

/* Opcodes of a touch report layout. */
enum syna_tcm_touch_code {
	SYNA_TCM_TOUCH_END = 0x00,
	SYNA_TCM_TOUCH_FOREACH_ACTIVE_OBJECT = 0x01,
	SYNA_TCM_TOUCH_FOREACH_OBJECT = 0x02,
	SYNA_TCM_TOUCH_FOREACH_END = 0x03,
	SYNA_TCM_TOUCH_PAD_TO_NEXT_BYTE = 0x04,
	SYNA_TCM_TOUCH_TIMESTAMP = 0x05,
	SYNA_TCM_TOUCH_OBJECT_N_INDEX = 0x06,
	SYNA_TCM_TOUCH_OBJECT_N_CLASSIFICATION = 0x07,
	SYNA_TCM_TOUCH_OBJECT_N_X_POSITION = 0x08,
	SYNA_TCM_TOUCH_OBJECT_N_Y_POSITION = 0x09,
	SYNA_TCM_TOUCH_NUM_OF_ACTIVE_OBJECTS = 0x18,
	SYNA_TCM_TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME = 0x19,
	SYNA_TCM_TOUCH_TUNING_GAUSSIAN_WIDTHS = 0x80,
	SYNA_TCM_TOUCH_TUNING_0D_BUTTONS_VARIANCE = 0x82,
};

enum syna_tcm_command_state {
	SYNA_TCM_CMD_IDLE,
	SYNA_TCM_CMD_BUSY,
	SYNA_TCM_CMD_ERROR,
};

struct syna_tcm_identification {
	u8 version;
	u8 mode;
	char part_number[16];
	__le32 build_id;
	__le16 max_write_size;
} __packed;

struct syna_tcm_app_info {
	__le16 version;
	__le16 status;
	__le16 static_config_size;
	__le16 dynamic_config_size;
	__le16 app_config_start_write_block;
	__le16 app_config_size;
	__le16 max_touch_report_config_size;
	__le16 max_touch_report_payload_size;
	u8 customer_config_id[16];
	__le16 max_x;
	__le16 max_y;
	__le16 max_objects;
	__le16 num_of_buttons;
	__le16 num_of_image_rows;
	__le16 num_of_image_cols;
	__le16 has_hybrid_data;
} __packed;

struct syna_tcm_object {
	u8 status;
	unsigned int x;
	unsigned int y;
};

struct syna_tcm {
	struct spi_device *spi;
	struct input_dev *input;
	struct input_dev *gesture_input;
	struct drm_panel_follower panel_follower;
	bool has_panel, panel_off, gesture_mode, fod_contact, wake_suspend;
	u32 gesture_entries, gesture_exits, double_taps, fod_downs, fod_ups;
	u32 gesture_errors;
	u64 mode_us;
	/* Report rate code set with the panel on, 0 = firmware default.
	 * Protected by reset_lock.
	 */
	u16 report_rate;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;
	struct regulator *avdd;

	struct work_struct reinit_work;
	struct delayed_work resume_work;

	/* Serialises the (re)initialisation of the controller. */
	struct mutex reset_lock;

	/* One command at a time; its response stays in @resp until the next. */
	struct mutex cmd_lock;
	struct completion cmd_done;

	/* Serialises bus transfers and protects all the state below. */
	struct mutex io_lock;
	struct completion reset_done;
	bool ready;
	bool suspended;
	bool resetting;
	bool report_touch;

	u8 command;
	enum syna_tcm_command_state command_state;
	u8 response_code;

	u8 *in;			/* the current message */
	unsigned int in_size;
	u8 *temp;		/* continued reads */
	unsigned int temp_size;
	u8 *out;		/* command writes */
	unsigned int out_size;
	u8 *fill;		/* SYNA_TCM_SPI_FILL, sent while reading */
	unsigned int fill_size;
	u8 *resp;
	unsigned int resp_size;
	unsigned int resp_length;
	u8 *config;		/* touch report layout in use */
	unsigned int config_size;
	unsigned int config_length;

	unsigned int read_length;
	unsigned int payload_length;
	u8 code;

	struct syna_tcm_identification id_info;
	struct syna_tcm_app_info app_info;
	unsigned int wr_chunk_size;

	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
	struct syna_tcm_object *objects;
};

/*
 * The report layout installed after every reset: for each active object
 * its index, classification and position.
 */
static const u8 syna_tcm_touch_config[] = {
	SYNA_TCM_TOUCH_FOREACH_ACTIVE_OBJECT,
	SYNA_TCM_TOUCH_OBJECT_N_INDEX, 4,
	SYNA_TCM_TOUCH_OBJECT_N_CLASSIFICATION, 4,
	SYNA_TCM_TOUCH_OBJECT_N_X_POSITION, 16,
	SYNA_TCM_TOUCH_OBJECT_N_Y_POSITION, 16,
	SYNA_TCM_TOUCH_FOREACH_END,
	SYNA_TCM_TOUCH_END,
};

/* Factory S3910 version-2 normal / gesture prefix followed by MT objects.
 * Keeping one verified layout across modes avoids a contact discontinuity
 * while the fingerprint presentation wakes the display.
 */
static const u8 syna_tcm_gesture_config[] = {
	0x10, 8, 0xc6, 48, 0xc7, 192,
	SYNA_TCM_TOUCH_FOREACH_ACTIVE_OBJECT,
	SYNA_TCM_TOUCH_OBJECT_N_INDEX, 4,
	SYNA_TCM_TOUCH_OBJECT_N_CLASSIFICATION, 4,
	SYNA_TCM_TOUCH_OBJECT_N_X_POSITION, 16,
	SYNA_TCM_TOUCH_OBJECT_N_Y_POSITION, 16,
	SYNA_TCM_TOUCH_FOREACH_END, SYNA_TCM_TOUCH_END,
};

static int syna_tcm_apply_mode(struct syna_tcm *ts, bool screen_off);

static int syna_tcm_grow(u8 **buf, unsigned int *size, unsigned int need)
{
	u8 *p;

	if (need <= *size)
		return 0;

	p = krealloc(*buf, need, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	*buf = p;
	*size = need;

	return 0;
}

static int syna_tcm_spi_read(struct syna_tcm *ts, u8 *data,
			     unsigned int length)
{
	struct spi_transfer xfer = { };
	int error;

	if (length > ts->fill_size) {
		error = syna_tcm_grow(&ts->fill, &ts->fill_size, length);
		if (error)
			return error;

		memset(ts->fill, SYNA_TCM_SPI_FILL, ts->fill_size);
	}

	xfer.tx_buf = ts->fill;
	xfer.rx_buf = data;
	xfer.len = length;

	error = spi_sync_transfer(ts->spi, &xfer, 1);
	if (error)
		dev_err_ratelimited(&ts->spi->dev, "SPI read failed: %d\n",
				    error);

	return error;
}

static int syna_tcm_spi_write(struct syna_tcm *ts, const u8 *data,
			      unsigned int length)
{
	struct spi_transfer xfer = {
		.tx_buf = data,
		.len = length,
	};
	int error;

	error = spi_sync_transfer(ts->spi, &xfer, 1);
	if (error)
		dev_err_ratelimited(&ts->spi->dev, "SPI write failed: %d\n",
				    error);

	return error;
}

/* Reads the rest of a message that did not fit into the first read. */
static int syna_tcm_continued_read(struct syna_tcm *ts,
				   unsigned int total_length)
{
	struct device *dev = &ts->spi->dev;
	unsigned int remaining = total_length - ts->read_length;
	int error;

	error = syna_tcm_grow(&ts->in, &ts->in_size, total_length + 1);
	if (error)
		return error;

	error = syna_tcm_grow(&ts->temp, &ts->temp_size, remaining + 2);
	if (error)
		return error;

	error = syna_tcm_spi_read(ts, ts->temp, remaining + 2);
	if (error)
		return error;

	if (ts->temp[0] != SYNA_TCM_MESSAGE_MARKER) {
		dev_err_ratelimited(dev, "incorrect continued read marker 0x%02x\n",
				    ts->temp[0]);
		return -EIO;
	}

	if (ts->temp[1] != SYNA_TCM_STATUS_CONTINUED_READ) {
		dev_err_ratelimited(dev, "incorrect continued read code 0x%02x\n",
				    ts->temp[1]);
		return -EIO;
	}

	memcpy(&ts->in[ts->read_length], &ts->temp[2], remaining);

	return 0;
}

/*
 * Reads one message into @in. Returns 0 when there is a message to
 * dispatch, 1 when the controller had nothing to send, or an error.
 */
static int syna_tcm_read_message(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	unsigned int total_length;
	bool retry = true;
	int error;

	lockdep_assert_held(&ts->io_lock);

	/* A padding byte may be stored just past the predicted length. */
	error = syna_tcm_grow(&ts->in, &ts->in_size, ts->read_length + 1);
	if (error)
		return error;

again:
	error = syna_tcm_spi_read(ts, ts->in, ts->read_length);
	if (error)
		goto retry_read;

	if (ts->in[0] != SYNA_TCM_MESSAGE_MARKER) {
		dev_err_ratelimited(dev, "incorrect header marker 0x%02x\n",
				    ts->in[0]);
		error = -ENXIO;
		goto retry_read;
	}

	ts->code = ts->in[1];
	ts->payload_length = get_unaligned_le16(&ts->in[2]);

	if (ts->code <= SYNA_TCM_STATUS_ERROR ||
	    ts->code == SYNA_TCM_STATUS_INVALID) {
		switch (ts->code) {
		case SYNA_TCM_STATUS_OK:
			break;
		case SYNA_TCM_STATUS_CONTINUED_READ:
			dev_dbg(dev, "out-of-sync continued read\n");
			fallthrough;
		case SYNA_TCM_STATUS_IDLE:
		case SYNA_TCM_STATUS_BUSY:
			ts->payload_length = 0;
			return 1;
		default:
			dev_err_ratelimited(dev, "incorrect header code 0x%02x\n",
					    ts->code);
			if (ts->code == SYNA_TCM_STATUS_INVALID) {
				if (retry) {
					error = -EIO;
					goto retry_read;
				}
				ts->payload_length = 0;
			}
			break;
		}
	}

	total_length = SYNA_TCM_HEADER_SIZE + ts->payload_length + 1;

	if (total_length - 1 == ts->read_length) {
		/* Only the padding byte is missing. */
		ts->in[total_length - 1] = SYNA_TCM_MESSAGE_PADDING;
	} else if (total_length > ts->read_length) {
		error = syna_tcm_continued_read(ts, total_length);
		if (error)
			return error;
	}

	if (ts->in[total_length - 1] != SYNA_TCM_MESSAGE_PADDING) {
		dev_err_ratelimited(dev, "incorrect message padding byte 0x%02x\n",
				    ts->in[total_length - 1]);
		return -EIO;
	}

	ts->read_length = max(total_length, SYNA_TCM_MIN_READ_LENGTH);

	return 0;

retry_read:
	if (!retry)
		return error;

	retry = false;
	usleep_range(SYNA_TCM_READ_RETRY_US, 2 * SYNA_TCM_READ_RETRY_US);
	goto again;
}

static void syna_tcm_release_contacts(struct syna_tcm *ts)
{
	lockdep_assert_held(&ts->io_lock);

	if (!ts->ready)
		return;

	/* Nothing reported in this frame: all contacts are dropped. */
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static bool syna_tcm_code_has_width(u8 code)
{
	return (code >= SYNA_TCM_TOUCH_TIMESTAMP &&
		code <= SYNA_TCM_TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME) ||
	       (code >= SYNA_TCM_TOUCH_TUNING_GAUSSIAN_WIDTHS &&
		code <= SYNA_TCM_TOUCH_TUNING_0D_BUTTONS_VARIANCE) ||
	       code == 0xc6 || code == 0xc7;
}

/* Index just past the end of the object loop at or after @idx. */
static unsigned int syna_tcm_skip_foreach(const u8 *config,
					  unsigned int length,
					  unsigned int idx)
{
	while (idx < length) {
		u8 code = config[idx++];

		if (code == SYNA_TCM_TOUCH_END)
			return idx - 1;
		if (code == SYNA_TCM_TOUCH_FOREACH_END)
			return idx;
		if (syna_tcm_code_has_width(code))
			idx++;
	}

	return idx;
}

/* Extracts @bits bits at bit @offset of the report, LSB first. */
static int syna_tcm_get_report_data(struct syna_tcm *ts, unsigned int offset,
				    unsigned int bits, unsigned int *data)
{
	const u8 *report = &ts->in[SYNA_TCM_HEADER_SIZE];
	unsigned int byte_offset = offset / 8;
	unsigned int bit_offset = offset % 8;
	unsigned int remaining = bits;
	unsigned int output = 0;
	unsigned int count;
	u8 value;

	if (bits == 0 || bits > 32)
		return -EINVAL;

	if (offset + bits > ts->payload_length * 8) {
		*data = 0;
		return 0;
	}

	while (remaining) {
		count = min(8 - bit_offset, remaining);
		value = report[byte_offset] >> bit_offset;
		value &= 0xff >> (8 - count);
		output |= (unsigned int)value << (bits - remaining);

		bit_offset = 0;
		byte_offset++;
		remaining -= count;
	}

	*data = output;

	return 0;
}

/* Decodes a touch report with the layout that the firmware reported. */
static int syna_tcm_parse_touch_report(struct syna_tcm *ts)
{
	const u8 *config = ts->config;
	unsigned int length = ts->config_length;
	unsigned int report_bits = ts->payload_length * 8;
	unsigned int idx = 0, next = 0, obj = 0, objects = 0;
	unsigned int offset = 0, loop_offset = 0;
	unsigned int active_objects = 0;
	bool active_only = false, have_count = false;
	struct syna_tcm_object *object;
	unsigned int bits, data;
	int error;
	u8 code;

	memset(ts->objects, 0, ts->max_objects * sizeof(*ts->objects));

	while (idx < length) {
		code = config[idx++];

		switch (code) {
		case SYNA_TCM_TOUCH_END:
			return 0;

		case SYNA_TCM_TOUCH_FOREACH_ACTIVE_OBJECT:
		case SYNA_TCM_TOUCH_FOREACH_OBJECT:
			active_only = code == SYNA_TCM_TOUCH_FOREACH_ACTIVE_OBJECT;
			obj = 0;
			next = idx;
			loop_offset = offset;
			continue;

		case SYNA_TCM_TOUCH_FOREACH_END:
			if (!active_only) {
				if (++obj < ts->max_objects)
					idx = next;
			} else if (have_count) {
				if (++objects < active_objects &&
				    objects < ts->max_objects)
					idx = next;
			} else if (offset < report_bits && offset != loop_offset) {
				/* Without a count, objects fill the report. */
				idx = next;
			}
			loop_offset = offset;
			continue;

		case SYNA_TCM_TOUCH_PAD_TO_NEXT_BYTE:
			offset = round_up(offset, 8);
			continue;
		}

		/* Unknown opcodes have no operand. */
		if (!syna_tcm_code_has_width(code))
			continue;

		if (idx >= length)
			return -EINVAL;

		bits = config[idx++];

		/* Tuning data is skipped unchecked. */
		if (code >= SYNA_TCM_TOUCH_TUNING_GAUSSIAN_WIDTHS) {
			offset += bits;
			continue;
		}

		error = syna_tcm_get_report_data(ts, offset, bits, &data);
		if (error)
			return error;

		offset += bits;

		object = obj < ts->max_objects ? &ts->objects[obj] : NULL;

		switch (code) {
		case SYNA_TCM_TOUCH_OBJECT_N_INDEX:
			obj = data;
			break;
		case SYNA_TCM_TOUCH_OBJECT_N_CLASSIFICATION:
			if (object)
				object->status = data;
			break;
		case SYNA_TCM_TOUCH_OBJECT_N_X_POSITION:
			if (object)
				object->x = data;
			break;
		case SYNA_TCM_TOUCH_OBJECT_N_Y_POSITION:
			if (object)
				object->y = data;
			break;
		case SYNA_TCM_TOUCH_NUM_OF_ACTIVE_OBJECTS:
			active_objects = data;
			have_count = true;
			if (!active_objects)
				idx = syna_tcm_skip_foreach(config, length, idx);
			break;
		default:
			/* Decoded, but not reported. */
			break;
		}
	}

	return 0;
}

/* Firmware FOD down/up is a physical contact, mapped to slot zero with the
 * firmware's own coordinates. Ordinary MT reports are suppressed in gesture
 * mode. Never generate a contact from a timer, wake key, or cached location.
 */
static bool syna_tcm_report_gesture(struct syna_tcm *ts)
{
	struct syna_tcm_gesture gesture;
	int error;

	error = syna_tcm_decode_gesture(&ts->in[SYNA_TCM_HEADER_SIZE],
				       ts->payload_length, ts->max_x, ts->max_y,
				       &gesture);
	if (error) {
		ts->gesture_errors++;
		if (ts->fod_contact) {
			syna_tcm_release_contacts(ts);
			ts->fod_contact = false;
		}
		return true;
	}

	if (gesture.code == SYNA_TCM_GESTURE_DTAP && ts->gesture_mode) {
		ts->double_taps++;
		pm_wakeup_event(&ts->spi->dev, 500);
		input_report_key(ts->gesture_input, KEY_WAKEUP, 1);
		input_sync(ts->gesture_input);
		input_report_key(ts->gesture_input, KEY_WAKEUP, 0);
		input_sync(ts->gesture_input);
	} else if (gesture.code == SYNA_TCM_GESTURE_FOD_DOWN &&
		   (ts->gesture_mode || ts->fod_contact)) {
		if (!ts->fod_contact) {
			ts->fod_downs++;
			pm_wakeup_event(&ts->spi->dev, 1000);
			input_mt_slot(ts->input, 0);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
			touchscreen_report_pos(ts->input, &ts->prop,
					       gesture.x, gesture.y, true);
			input_mt_sync_frame(ts->input);
			input_sync(ts->input);
			ts->fod_contact = true;
		}
		return true;
	} else if (gesture.code == SYNA_TCM_GESTURE_FOD_UP && ts->fod_contact) {
		ts->fod_ups++;
		syna_tcm_release_contacts(ts);
		ts->fod_contact = false;
		return true;
	}

	return ts->gesture_mode;
}

static void syna_tcm_report_touch(struct syna_tcm *ts)
{
	struct syna_tcm_object *object;
	unsigned int i;
	int error;

	if (!ts->report_touch)
		return;
	if (ts->has_panel && syna_tcm_report_gesture(ts))
		return;

	error = syna_tcm_parse_touch_report(ts);
	if (error) {
		dev_err_ratelimited(&ts->spi->dev,
				    "failed to parse touch report: %d\n", error);
		return;
	}

	ts->fod_contact = false;
	for (i = 0; i < ts->max_objects; i++) {
		object = &ts->objects[i];

		if (object->status != SYNA_TCM_OBJECT_FINGER &&
		    object->status != SYNA_TCM_OBJECT_GLOVED_FINGER)
			continue;

		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop,
				       object->x, object->y, true);
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static void syna_tcm_handle_identify(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	bool expected = ts->resetting;

	memcpy(&ts->id_info, &ts->in[SYNA_TCM_HEADER_SIZE],
	       min_t(unsigned int, sizeof(ts->id_info), ts->payload_length));
	ts->wr_chunk_size = le16_to_cpu(ts->id_info.max_write_size);

	/* The controller answers these commands by resetting. */
	if (ts->command_state == SYNA_TCM_CMD_BUSY) {
		if (ts->command == SYNA_TCM_CMD_RESET ||
		    ts->command == SYNA_TCM_CMD_RUN_APPLICATION_FIRMWARE) {
			ts->response_code = SYNA_TCM_STATUS_OK;
			ts->command_state = SYNA_TCM_CMD_IDLE;
			expected = true;
		} else {
			dev_notice(dev, "controller was reset\n");
			ts->command_state = SYNA_TCM_CMD_ERROR;
		}
		complete(&ts->cmd_done);
	}

	if (ts->resetting)
		complete(&ts->reset_done);

	if (ts->id_info.mode == SYNA_TCM_MODE_HOST_DOWNLOAD) {
		dev_err(dev, "controller waits for a firmware download, which is not supported\n");
		return;
	}

	syna_tcm_release_contacts(ts);

	/*
	 * After a reset that the driver did not ask for, the report layout
	 * has to be set again. Otherwise the requester does it.
	 */
	if (!expected && ts->ready && !ts->suspended &&
	    ts->id_info.mode == SYNA_TCM_MODE_APPLICATION) {
		ts->report_touch = false;
		schedule_work(&ts->reinit_work);
	}
}

static void syna_tcm_handle_response(struct syna_tcm *ts)
{
	int error;

	if (ts->command_state != SYNA_TCM_CMD_BUSY)
		return;

	ts->response_code = ts->code;
	ts->resp_length = 0;

	if (ts->payload_length) {
		error = syna_tcm_grow(&ts->resp, &ts->resp_size,
				      ts->payload_length);
		if (error) {
			ts->command_state = SYNA_TCM_CMD_ERROR;
			complete(&ts->cmd_done);
			return;
		}

		memcpy(ts->resp, &ts->in[SYNA_TCM_HEADER_SIZE],
		       ts->payload_length);
		ts->resp_length = ts->payload_length;
	}

	ts->command_state = SYNA_TCM_CMD_IDLE;
	complete(&ts->cmd_done);
}

static irqreturn_t syna_tcm_irq(int irq, void *data)
{
	struct syna_tcm *ts = data;
	int ret;

	guard(mutex)(&ts->io_lock);

	ret = syna_tcm_read_message(ts);
	if (ret < 0) {
		dev_err_ratelimited(&ts->spi->dev,
				    "failed to read message: %d\n", ret);
		if (ts->command_state == SYNA_TCM_CMD_BUSY) {
			ts->command_state = SYNA_TCM_CMD_ERROR;
			complete(&ts->cmd_done);
		}
		return IRQ_HANDLED;
	}

	if (ret > 0)
		return IRQ_HANDLED;

	if (ts->code < SYNA_TCM_REPORT_IDENTIFY)
		syna_tcm_handle_response(ts);
	else if (ts->code == SYNA_TCM_REPORT_IDENTIFY)
		syna_tcm_handle_identify(ts);
	else if (ts->code == SYNA_TCM_REPORT_TOUCH)
		syna_tcm_report_touch(ts);

	return IRQ_HANDLED;
}

static int syna_tcm_write_command(struct syna_tcm *ts, u8 command,
				  const u8 *payload, unsigned int length)
{
	unsigned int remaining = length + 2;
	unsigned int chunk_space, chunks, xfer_length, idx;
	int error;

	lockdep_assert_held(&ts->io_lock);

	/*
	 * A write carries at most the maximum write size of the controller,
	 * command byte included. The length field counts as payload here.
	 */
	chunk_space = ts->wr_chunk_size > 2 ? ts->wr_chunk_size - 1 : remaining;
	chunks = DIV_ROUND_UP(remaining, chunk_space);

	for (idx = 0; idx < chunks; idx++) {
		xfer_length = min(remaining, chunk_space);

		error = syna_tcm_grow(&ts->out, &ts->out_size, xfer_length + 1);
		if (error)
			return error;

		if (idx == 0) {
			ts->out[0] = command;
			put_unaligned_le16(length, &ts->out[1]);
			if (xfer_length > 2)
				memcpy(&ts->out[3], payload, xfer_length - 2);
		} else {
			ts->out[0] = SYNA_TCM_CMD_CONTINUE_WRITE;
			memcpy(&ts->out[1], &payload[idx * chunk_space - 2],
			       xfer_length);
		}

		error = syna_tcm_spi_write(ts, ts->out, xfer_length + 1);
		if (error)
			return error;

		remaining -= xfer_length;

		if (chunks > 1)
			usleep_range(SYNA_TCM_WRITE_DELAY_US,
				     2 * SYNA_TCM_WRITE_DELAY_US);
	}

	return 0;
}

/* Sends a command and waits for its response, which is left in @resp. */
static int __syna_tcm_command(struct syna_tcm *ts, u8 command,
			      const u8 *payload, unsigned int length)
{
	struct device *dev = &ts->spi->dev;
	unsigned long timeout = msecs_to_jiffies(SYNA_TCM_RESPONSE_TIMEOUT_MS);
	int error;

	lockdep_assert_held(&ts->cmd_lock);

	scoped_guard(mutex, &ts->io_lock) {
		ts->command = command;
		ts->command_state = SYNA_TCM_CMD_BUSY;
		ts->resp_length = 0;
		reinit_completion(&ts->cmd_done);

		error = syna_tcm_write_command(ts, command, payload, length);
		if (error)
			dev_err(dev, "failed to write command 0x%02x: %d\n",
				command, error);
	}

	if (!error && !wait_for_completion_timeout(&ts->cmd_done, timeout)) {
		dev_err(dev, "timed out waiting for response to command 0x%02x\n",
			command);
		error = -ETIMEDOUT;
	}

	guard(mutex)(&ts->io_lock);

	if (!error && ts->command_state != SYNA_TCM_CMD_IDLE) {
		dev_err(dev, "no valid response to command 0x%02x\n", command);
		error = -EIO;
	} else if (!error && ts->response_code != SYNA_TCM_STATUS_OK) {
		if (ts->resp_length)
			dev_err(dev, "command 0x%02x failed: status 0x%02x, error 0x%02x\n",
				command, ts->response_code, ts->resp[0]);
		else
			dev_err(dev, "command 0x%02x failed: status 0x%02x\n",
				command, ts->response_code);
		error = -EIO;
	}

	ts->command = SYNA_TCM_CMD_NONE;
	ts->command_state = SYNA_TCM_CMD_IDLE;

	return error;
}

static int syna_tcm_command(struct syna_tcm *ts, u8 command,
			    const u8 *payload, unsigned int length)
{
	guard(mutex)(&ts->cmd_lock);

	return __syna_tcm_command(ts, command, payload, length);
}

static u8 syna_tcm_mode(struct syna_tcm *ts)
{
	guard(mutex)(&ts->io_lock);

	return ts->id_info.mode;
}

static int syna_tcm_get_app_info(struct syna_tcm *ts)
{
	unsigned int timeout = SYNA_TCM_APP_STATUS_TIMEOUT_MS;
	unsigned int status;
	int error;

	guard(mutex)(&ts->cmd_lock);

	for (;;) {
		error = __syna_tcm_command(ts, SYNA_TCM_CMD_GET_APPLICATION_INFO,
					   NULL, 0);
		if (error)
			return error;

		if (ts->resp_length)
			memcpy(&ts->app_info, ts->resp,
			       min_t(unsigned int, sizeof(ts->app_info),
				     ts->resp_length));

		status = le16_to_cpu(ts->app_info.status);
		if ((status != SYNA_TCM_APP_STATUS_BOOTING &&
		     status != SYNA_TCM_APP_STATUS_UPDATING) || !timeout)
			return 0;

		msleep(SYNA_TCM_APP_STATUS_POLL_MS);
		timeout -= SYNA_TCM_APP_STATUS_POLL_MS;
	}
}

static int syna_tcm_check_application(struct syna_tcm *ts)
{
	unsigned int status = le16_to_cpu(ts->app_info.status);

	if (status != SYNA_TCM_APP_STATUS_OK) {
		dev_err(&ts->spi->dev, "application firmware status 0x%02x\n",
			status);
		return -ENODEV;
	}

	return 0;
}

static int syna_tcm_start_application(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	u8 mode;
	int error;

	if (syna_tcm_mode(ts) != SYNA_TCM_MODE_APPLICATION) {
		error = syna_tcm_command(ts, SYNA_TCM_CMD_RUN_APPLICATION_FIRMWARE,
					 NULL, 0);
		if (error)
			dev_notice(dev, "failed to run application firmware: %d\n",
				   error);
	}

	mode = syna_tcm_mode(ts);
	if (mode != SYNA_TCM_MODE_APPLICATION) {
		dev_err(dev, "controller is in firmware mode 0x%02x\n", mode);
		return -ENODEV;
	}

	error = syna_tcm_get_app_info(ts);
	if (error)
		return error;

	return syna_tcm_check_application(ts);
}

static int syna_tcm_set_report_config(struct syna_tcm *ts)
{
	unsigned int length =
		le16_to_cpu(ts->app_info.max_touch_report_config_size);
	u8 *config;
	int error;

	if (length < SYNA_TCM_TOUCH_REPORT_CONFIG_SIZE) {
		dev_err(&ts->spi->dev,
			"invalid maximum touch report config size %u\n", length);
		return -EINVAL;
	}

	config = kzalloc(length, GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	if (ts->has_panel)
		memcpy(config, syna_tcm_gesture_config, sizeof(syna_tcm_gesture_config));
	else
		memcpy(config, syna_tcm_touch_config, sizeof(syna_tcm_touch_config));

	error = syna_tcm_command(ts, SYNA_TCM_CMD_SET_TOUCH_REPORT_CONFIG,
				 config, length);
	kfree(config);

	return error;
}

static int syna_tcm_get_report_config(struct syna_tcm *ts)
{
	int error;

	guard(mutex)(&ts->cmd_lock);

	error = __syna_tcm_command(ts, SYNA_TCM_CMD_GET_TOUCH_REPORT_CONFIG,
				   NULL, 0);
	if (error)
		return error;

	guard(mutex)(&ts->io_lock);

	error = syna_tcm_grow(&ts->config, &ts->config_size, ts->resp_length);
	if (error)
		return error;

	if (ts->resp_length)
		memcpy(ts->config, ts->resp, ts->resp_length);
	ts->config_length = ts->resp_length;
	if (ts->has_panel &&
	    (ts->config_length < sizeof(syna_tcm_gesture_config) ||
	     memcmp(ts->config, syna_tcm_gesture_config,
		    sizeof(syna_tcm_gesture_config))))
		return -EPROTO;

	return 0;
}

/*
 * The report layout does not survive a reset: install ours and use the
 * one that the firmware then reports.
 */
static int syna_tcm_configure(struct syna_tcm *ts)
{
	int error;

	scoped_guard(mutex, &ts->io_lock) {
		ts->report_touch = false;
		syna_tcm_release_contacts(ts);
	}

	error = syna_tcm_set_report_config(ts);
	if (error)
		return error;

	return syna_tcm_get_report_config(ts);
}

static void syna_tcm_start_reporting(struct syna_tcm *ts)
{
	unsigned int max_x = le16_to_cpu(ts->app_info.max_x);
	unsigned int max_y = le16_to_cpu(ts->app_info.max_y);
	unsigned int max_objects = le16_to_cpu(ts->app_info.max_objects);

	if (max_x != ts->max_x || max_y != ts->max_y ||
	    max_objects != ts->max_objects)
		dev_warn(&ts->spi->dev,
			 "firmware now reports %ux%u, %u objects; keeping %ux%u, %u objects\n",
			 max_x, max_y, max_objects,
			 ts->max_x, ts->max_y, ts->max_objects);

	guard(mutex)(&ts->io_lock);

	if (!ts->suspended)
		ts->report_touch = true;
}

static void syna_tcm_reset_pulse(struct syna_tcm *ts)
{
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(SYNA_TCM_RESET_ACTIVE_US, 2 * SYNA_TCM_RESET_ACTIVE_US);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(SYNA_TCM_RESET_DELAY_MS);
}

static int syna_tcm_hw_reset(struct syna_tcm *ts)
{
	unsigned long timeout = msecs_to_jiffies(SYNA_TCM_RESPONSE_TIMEOUT_MS);
	int error = 0;

	scoped_guard(mutex, &ts->io_lock) {
		ts->resetting = true;
		reinit_completion(&ts->reset_done);
	}

	syna_tcm_reset_pulse(ts);

	/* The identify report of the reset is normally in by now. */
	if (!wait_for_completion_timeout(&ts->reset_done, timeout)) {
		dev_err(&ts->spi->dev, "no identify report after reset\n");
		error = -ETIMEDOUT;
	}

	scoped_guard(mutex, &ts->io_lock)
		ts->resetting = false;

	return error;
}

static int syna_tcm_reinit(struct syna_tcm *ts)
{
	int error;

	guard(mutex)(&ts->reset_lock);

	scoped_guard(mutex, &ts->io_lock) {
		if (ts->suspended)
			return 0;
	}

	if (syna_tcm_mode(ts) != SYNA_TCM_MODE_APPLICATION)
		return 0;

	error = syna_tcm_get_app_info(ts);
	if (error)
		return error;

	error = syna_tcm_check_application(ts);
	if (error)
		return error;

	error = syna_tcm_configure(ts);
	if (error)
		return error;

	syna_tcm_start_reporting(ts);
	if (ts->has_panel)
		return syna_tcm_apply_mode(ts, ts->panel_off);

	return 0;
}

static void syna_tcm_reinit_work(struct work_struct *work)
{
	struct syna_tcm *ts = container_of(work, struct syna_tcm, reinit_work);
	int error;

	error = syna_tcm_reinit(ts);
	if (error)
		dev_err(&ts->spi->dev,
			"failed to reconfigure after controller reset: %d\n",
			error);
}

static int syna_tcm_wake(struct syna_tcm *ts)
{
	int error;

	guard(mutex)(&ts->reset_lock);

	enable_irq(ts->spi->irq);
	msleep(SYNA_TCM_IRQ_ENABLE_DELAY_MS);

	scoped_guard(mutex, &ts->io_lock)
		ts->suspended = false;

	/* The reset also takes the controller out of deep sleep. */
	error = syna_tcm_hw_reset(ts);
	if (error)
		return error;

	error = syna_tcm_start_application(ts);
	if (error)
		return error;

	error = syna_tcm_configure(ts);
	if (error)
		return error;

	syna_tcm_start_reporting(ts);
	if (ts->has_panel)
		return syna_tcm_apply_mode(ts, ts->panel_off);

	return 0;
}

static void syna_tcm_resume_work(struct work_struct *work)
{
	struct syna_tcm *ts = container_of(to_delayed_work(work),
					   struct syna_tcm, resume_work);
	int error;

	if (ts->has_panel && !READ_ONCE(ts->suspended)) {
		guard(mutex)(&ts->reset_lock);
		error = syna_tcm_apply_mode(ts, ts->panel_off);
	} else {
		error = syna_tcm_wake(ts);
	}
	if (error)
		dev_err(&ts->spi->dev, "failed to reinitialize after resume: %d\n",
			error);
}

static int syna_tcm_power_on(struct syna_tcm *ts)
{
	int error;

	error = regulator_enable(ts->vdd);
	if (error)
		return error;

	error = regulator_enable(ts->avdd);
	if (error) {
		regulator_disable(ts->vdd);
		return error;
	}

	msleep(SYNA_TCM_POWER_DELAY_MS);
	syna_tcm_reset_pulse(ts);

	return 0;
}

static void syna_tcm_power_off(void *data)
{
	struct syna_tcm *ts = data;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	regulator_disable(ts->avdd);
	regulator_disable(ts->vdd);
}

static void syna_tcm_free_buffers(void *data)
{
	struct syna_tcm *ts = data;

	kfree(ts->in);
	kfree(ts->temp);
	kfree(ts->out);
	kfree(ts->fill);
	kfree(ts->resp);
	kfree(ts->config);
}

/* Runs before the interrupt is freed, so that no work is queued again. */
static void syna_tcm_stop(void *data)
{
	struct syna_tcm *ts = data;

	scoped_guard(mutex, &ts->io_lock) {
		ts->ready = false;
		ts->report_touch = false;
	}

	cancel_work_sync(&ts->reinit_work);
	cancel_delayed_work_sync(&ts->resume_work);
}

static int syna_tcm_setup_input(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	struct input_dev *input = ts->input;
	int error;

	ts->max_x = le16_to_cpu(ts->app_info.max_x);
	ts->max_y = le16_to_cpu(ts->app_info.max_y);
	ts->max_objects = le16_to_cpu(ts->app_info.max_objects);

	if (!ts->max_x || !ts->max_y || !ts->max_objects)
		return dev_err_probe(dev, -EINVAL,
				     "invalid touch geometry %ux%u, %u objects\n",
				     ts->max_x, ts->max_y, ts->max_objects);

	ts->objects = devm_kcalloc(dev, ts->max_objects, sizeof(*ts->objects),
				   GFP_KERNEL);
	if (!ts->objects)
		return -ENOMEM;

	input->name = SYNA_TCM_INPUT_NAME;
	input->id.bustype = BUS_SPI;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);

	error = input_mt_init_slots(input, ts->max_objects,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return dev_err_probe(dev, error, "failed to init MT slots\n");

	error = input_register_device(input);
	if (error)
		return dev_err_probe(dev, error,
				     "failed to register input device\n");

	return 0;
}

/* reset_lock serialises panel changes, PM and controller recovery. */
static int syna_tcm_config_value(struct syna_tcm *ts, u8 id, u16 value)
{
	u8 payload[3] = { id, value, value >> 8 };
	int error;

	guard(mutex)(&ts->cmd_lock);
	error = __syna_tcm_command(ts, SYNA_TCM_CMD_SET_DYNAMIC_CONFIG,
				   payload, sizeof(payload));
	if (error)
		return error;
	error = __syna_tcm_command(ts, SYNA_TCM_CMD_GET_DYNAMIC_CONFIG, &id, 1);
	if (error)
		return error;
	if (ts->resp_length < 2 || get_unaligned_le16(ts->resp) != value)
		return -EIO;
	return 0;
}

static int syna_tcm_read_config_value(struct syna_tcm *ts, u8 id, u16 *value)
{
	int error;

	guard(mutex)(&ts->cmd_lock);
	error = __syna_tcm_command(ts, SYNA_TCM_CMD_GET_DYNAMIC_CONFIG, &id, 1);
	if (error)
		return error;
	if (ts->resp_length < 2)
		return -EIO;
	*value = get_unaligned_le16(ts->resp);
	return 0;
}

/* Called with the panel on and the touch mode applied. A failure keeps the
 * firmware's own rate; touch reporting stays on.
 */
static void syna_tcm_apply_report_rate(struct syna_tcm *ts)
{
	int error;

	lockdep_assert_held(&ts->reset_lock);
	if (!ts->report_rate)
		return;
	error = syna_tcm_config_value(ts, SYNA_TCM_DC_REPORT_RATE, ts->report_rate);
	if (error)
		dev_err(&ts->spi->dev, "report rate %u not applied: %d\n",
			ts->report_rate, error);
}

static int syna_tcm_apply_mode(struct syna_tcm *ts, bool screen_off)
{
	u64 start = ktime_get_ns();
	int error;

	lockdep_assert_held(&ts->reset_lock);
	scoped_guard(mutex, &ts->io_lock) {
		ts->report_touch = false;
		if (screen_off && !ts->fod_contact)
			syna_tcm_release_contacts(ts);
	}
	/* Factory mask bit zero enables double tap; TOUCH_HOLD is independent.
	 * These are volatile dynamic settings, never firmware/flash writes.
	 */
	if (screen_off) {
		error = syna_tcm_config_value(ts, SYNA_TCM_DC_GESTURE, 1);
		if (!error)
			error = syna_tcm_config_value(ts, SYNA_TCM_DC_GESTURE_MASK, 1);
		if (!error)
			error = syna_tcm_config_value(ts, SYNA_TCM_DC_TOUCH_HOLD, 1);
	} else {
		error = syna_tcm_config_value(ts, SYNA_TCM_DC_GESTURE, 0);
		if (!error)
			error = syna_tcm_config_value(ts, SYNA_TCM_DC_TOUCH_HOLD, 2);
	}
	scoped_guard(mutex, &ts->io_lock) {
		ts->mode_us = div_u64(ktime_get_ns() - start, 1000);
		if (!error) {
			ts->gesture_mode = screen_off;
			ts->report_touch = true;
			if (screen_off)
				ts->gesture_entries++;
			else
				ts->gesture_exits++;
		} else {
			ts->gesture_errors++;
			syna_tcm_release_contacts(ts);
			ts->fod_contact = false;
		}
	}
	if (!error && !screen_off)
		syna_tcm_apply_report_rate(ts);
	dev_info(&ts->spi->dev, "screen=%s gesture=%u verified=%u transition=%llu us\n",
		 screen_off ? "off" : "on", screen_off, !error, ts->mode_us);
	return error;
}

static int syna_tcm_panel_prepared(struct drm_panel_follower *follower)
{
	struct syna_tcm *ts = container_of(follower, struct syna_tcm, panel_follower);

	guard(mutex)(&ts->reset_lock);
	WRITE_ONCE(ts->panel_off, false);
	/* A DRM supplier may resume before this SPI consumer. Defer commands
	 * until our PM resume, after the bus and pending wake IRQ are ready.
	 */
	if (READ_ONCE(ts->suspended))
		return 0;
	return syna_tcm_apply_mode(ts, false);
}

static int syna_tcm_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct syna_tcm *ts = container_of(follower, struct syna_tcm, panel_follower);

	guard(mutex)(&ts->reset_lock);
	WRITE_ONCE(ts->panel_off, true);
	/* A DRM supplier may resume before this SPI consumer. Defer commands
	 * until our PM resume, after the bus and pending wake IRQ are ready.
	 */
	if (READ_ONCE(ts->suspended))
		return 0;
	return syna_tcm_apply_mode(ts, true);
}

static const struct drm_panel_follower_funcs syna_tcm_panel_funcs = {
	.panel_prepared = syna_tcm_panel_prepared,
	.panel_unpreparing = syna_tcm_panel_unpreparing,
};

static void syna_tcm_clear_wake_irq(void *data)
{
	dev_pm_clear_wake_irq(data);
}

static ssize_t gesture_state_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct syna_tcm *ts = dev_get_drvdata(dev);

	guard(mutex)(&ts->io_lock);
	return sysfs_emit(buf, "panel_off=%u gesture=%u reporting=%u suspended=%u wake_suspend=%u fod_contact=%u entries=%u exits=%u double_taps=%u fod_downs=%u fod_ups=%u errors=%u transition_us=%llu\n",
		 ts->panel_off, ts->gesture_mode, ts->report_touch, ts->suspended,
		 ts->wake_suspend, ts->fod_contact, ts->gesture_entries,
		 ts->gesture_exits, ts->double_taps, ts->fod_downs, ts->fod_ups,
		 ts->gesture_errors, ts->mode_us);
}
static DEVICE_ATTR_RO(gesture_state);

/* The controller's report rate code, read back. Writing a code sets it now
 * and after every reset while the panel is on; 0 leaves the firmware's own
 * rate from the next reset. The codes are the firmware's.
 */
static ssize_t report_rate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct syna_tcm *ts = dev_get_drvdata(dev);
	u16 value;
	int error;

	guard(mutex)(&ts->reset_lock);
	if (READ_ONCE(ts->suspended))
		return -EAGAIN;
	error = syna_tcm_read_config_value(ts, SYNA_TCM_DC_REPORT_RATE, &value);
	if (error)
		return error;
	return sysfs_emit(buf, "%u\n", value);
}

static ssize_t report_rate_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct syna_tcm *ts = dev_get_drvdata(dev);
	u16 value;
	int error;

	error = kstrtou16(buf, 0, &value);
	if (error)
		return error;

	guard(mutex)(&ts->reset_lock);
	ts->report_rate = value;
	if (!value || READ_ONCE(ts->suspended) || READ_ONCE(ts->panel_off))
		return count;
	error = syna_tcm_config_value(ts, SYNA_TCM_DC_REPORT_RATE, value);
	if (error) {
		ts->report_rate = 0;
		return error;
	}
	return count;
}
static DEVICE_ATTR_RW(report_rate);

static struct attribute *syna_tcm_attrs[] = {
	&dev_attr_gesture_state.attr, &dev_attr_report_rate.attr, NULL,
};
ATTRIBUTE_GROUPS(syna_tcm);

static int syna_tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct syna_tcm *ts;
	int error;

	if (spi->controller->flags & SPI_CONTROLLER_HALF_DUPLEX)
		return dev_err_probe(dev, -EINVAL,
				     "full duplex SPI is required\n");

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	ts->has_panel = drm_is_panel_follower(dev);
	ts->panel_off = ts->has_panel;
	spi_set_drvdata(spi, ts);

	error = devm_mutex_init(dev, &ts->reset_lock);
	if (error)
		return error;

	error = devm_mutex_init(dev, &ts->cmd_lock);
	if (error)
		return error;

	error = devm_mutex_init(dev, &ts->io_lock);
	if (error)
		return error;

	init_completion(&ts->cmd_done);
	init_completion(&ts->reset_done);
	INIT_WORK(&ts->reinit_work, syna_tcm_reinit_work);
	INIT_DELAYED_WORK(&ts->resume_work, syna_tcm_resume_work);

	error = devm_add_action_or_reset(dev, syna_tcm_free_buffers, ts);
	if (error)
		return error;

	ts->read_length = SYNA_TCM_MIN_READ_LENGTH;

	spi->bits_per_word = 8;
	error = spi_setup(spi);
	if (error)
		return dev_err_probe(dev, error, "failed to set up SPI\n");

	ts->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ts->vdd))
		return dev_err_probe(dev, PTR_ERR(ts->vdd),
				     "failed to get vdd supply\n");

	ts->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(ts->avdd))
		return dev_err_probe(dev, PTR_ERR(ts->avdd),
				     "failed to get avdd supply\n");

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	error = syna_tcm_power_on(ts);
	if (error)
		return dev_err_probe(dev, error, "failed to power on\n");

	error = devm_add_action_or_reset(dev, syna_tcm_power_off, ts);
	if (error)
		return error;

	/* The identify report of the power-on reset is read right away. */
	error = devm_request_threaded_irq(dev, spi->irq, NULL, syna_tcm_irq,
					  IRQF_ONESHOT, dev_name(dev), ts);
	if (error)
		return dev_err_probe(dev, error, "failed to request IRQ\n");

	msleep(SYNA_TCM_IRQ_ENABLE_DELAY_MS);

	error = syna_tcm_command(ts, SYNA_TCM_CMD_RESET, NULL, 0);
	if (error)
		return dev_err_probe(dev, error, "failed to reset controller\n");

	msleep(SYNA_TCM_RESET_DELAY_MS);

	error = syna_tcm_start_application(ts);
	if (error)
		return error;

	error = syna_tcm_configure(ts);
	if (error)
		return dev_err_probe(dev, error,
				     "failed to set up touch reports\n");

	error = syna_tcm_setup_input(ts);
	if (error)
		return error;

	error = devm_add_action_or_reset(dev, syna_tcm_stop, ts);
	if (error)
		return error;

	scoped_guard(mutex, &ts->io_lock) {
		ts->ready = true;
		ts->report_touch = true;
	}

	if (ts->has_panel) {
		ts->gesture_input = devm_input_allocate_device(dev);
		if (!ts->gesture_input)
			return -ENOMEM;
		ts->gesture_input->name = "synaptics_tcm_gesture";
		ts->gesture_input->id.bustype = BUS_SPI;
		input_set_capability(ts->gesture_input, EV_KEY, KEY_WAKEUP);
		error = input_register_device(ts->gesture_input);
		if (error)
			return error;
		error = devm_device_init_wakeup(dev);
		if (error)
			return error;
		error = dev_pm_set_wake_irq(dev, spi->irq);
		if (error)
			return error;
		error = devm_add_action_or_reset(dev, syna_tcm_clear_wake_irq, dev);
		if (error)
			return error;
		scoped_guard(mutex, &ts->reset_lock)
			error = syna_tcm_apply_mode(ts, true);
		if (error)
			return dev_err_probe(dev, error, "failed to initialise gesture mode\n");
		ts->panel_follower.funcs = &syna_tcm_panel_funcs;
		error = devm_drm_panel_add_follower(dev, &ts->panel_follower);
		if (error)
			return dev_err_probe(dev, error, "failed to follow display panel\n");
	}

	dev_info(dev, "TouchComm v%u, %.*s, build %u: %ux%u, %u objects\n",
		 ts->id_info.version,
		 (int)strnlen(ts->id_info.part_number,
			      sizeof(ts->id_info.part_number)),
		 ts->id_info.part_number, le32_to_cpu(ts->id_info.build_id),
		 ts->max_x, ts->max_y, ts->max_objects);

	return 0;
}

static int syna_tcm_suspend(struct device *dev)
{
	struct syna_tcm *ts = dev_get_drvdata(dev);
	int error;

	cancel_delayed_work_sync(&ts->resume_work);
	cancel_work_sync(&ts->reinit_work);

	if (ts->has_panel && device_may_wakeup(dev)) {
		guard(mutex)(&ts->reset_lock);
		/* Do not re-arm a sleeping controller on every system suspend. */
		if (!ts->gesture_mode || !ts->report_touch) {
			error = syna_tcm_apply_mode(ts, true);
			if (error)
				return error;
		}
		scoped_guard(mutex, &ts->io_lock) {
			ts->suspended = true;
			ts->wake_suspend = true;
		}
		/* PM wakeirq masks delivery while SPI is asleep, then replays
		 * the pending level IRQ after the bus resumes. No reset needed.
		 */
		return 0;
	}

	scoped_guard(mutex, &ts->io_lock) {
		/* Still suspended: the resume work did not run. */
		if (ts->suspended)
			return 0;

		ts->suspended = true;
		ts->report_touch = false;
		syna_tcm_release_contacts(ts);
	}

	cancel_work_sync(&ts->reinit_work);

	if (syna_tcm_mode(ts) == SYNA_TCM_MODE_APPLICATION &&
	    le16_to_cpu(ts->app_info.status) == SYNA_TCM_APP_STATUS_OK) {
		error = syna_tcm_command(ts, SYNA_TCM_CMD_ENTER_DEEP_SLEEP,
					 NULL, 0);
		if (error)
			dev_err(dev, "failed to enter deep sleep: %d\n", error);
	}

	disable_irq(ts->spi->irq);

	return 0;
}

static int syna_tcm_resume(struct device *dev)
{
	struct syna_tcm *ts = dev_get_drvdata(dev);

	if (ts->wake_suspend) {
		/* Keep gesture mode until the DRM panel prepares. Process the
		 * wake-causing report before changing firmware mode.
		 */
		guard(mutex)(&ts->io_lock);
		ts->suspended = false;
		ts->wake_suspend = false;
		if (!ts->panel_off)
			mod_delayed_work(system_dfl_wq, &ts->resume_work,
				msecs_to_jiffies(SYNA_TCM_RESUME_DELAY_MS));
		return 0;
	}

	scoped_guard(mutex, &ts->io_lock) {
		if (!ts->suspended)
			return 0;
	}

	if (ts->has_panel)
		return syna_tcm_wake(ts);

	/* Bring the controller back once the SPI controller is up again. */
	mod_delayed_work(system_dfl_wq, &ts->resume_work,
			 msecs_to_jiffies(SYNA_TCM_RESUME_DELAY_MS));

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(syna_tcm_pm_ops,
				syna_tcm_suspend, syna_tcm_resume);

static const struct spi_device_id syna_tcm_spi_ids[] = {
	{ "s3910" },
	{ }
};
MODULE_DEVICE_TABLE(spi, syna_tcm_spi_ids);

static const struct of_device_id syna_tcm_of_match[] = {
	{ .compatible = "syna,s3910" },
	{ }
};
MODULE_DEVICE_TABLE(of, syna_tcm_of_match);

static struct spi_driver syna_tcm_spi_driver = {
	.driver = {
		.name = "synaptics_tcm_spi",
		.of_match_table = syna_tcm_of_match,
		.pm = pm_sleep_ptr(&syna_tcm_pm_ops),
		.dev_groups = syna_tcm_groups,
	},
	.probe = syna_tcm_probe,
	.id_table = syna_tcm_spi_ids,
};
module_spi_driver(syna_tcm_spi_driver);

MODULE_AUTHOR("Doctor Logix <doctor.logix@gmail.com>");
MODULE_DESCRIPTION("Synaptics TouchComm SPI touchscreen driver");
MODULE_LICENSE("GPL");
