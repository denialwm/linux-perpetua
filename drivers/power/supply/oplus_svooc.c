// SPDX-License-Identifier: GPL-2.0-only
/*
 * Oplus SuperVOOC adapter protocol.
 *
 * Adapted from the Oplus AP VOOC PHY driver,
 * Copyright (C) 2018-2021 Oplus. All rights reserved.
 * The adapter asks and the phone answers. This library answers the
 * identification exchanges; the charger driver answers power commands.
 */
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/power/oplus_svooc.h>

/**
 * oplus_svooc_decode() - decode an adapter frame
 * @rx: the frame's last eight bits
 * @req: the command and whether the PHY latched it one bit late
 *
 * Return: 0, or -EPROTO for a frame of another protocol.
 */
int oplus_svooc_decode(u8 rx, struct oplus_svooc_request *req)
{
	u8 head = rx >> 5;

	req->shifted = false;
	if (head != OPLUS_SVOOC_HEAD && head != OPLUS_VOOC2_HEAD) {
		head = (rx >> 4) & 7;
		req->shifted = true;
	}
	if (head != OPLUS_SVOOC_HEAD && head != OPLUS_VOOC2_HEAD)
		return -EPROTO;
	req->vooc2 = head == OPLUS_VOOC2_HEAD;
	req->command = req->shifted ? rx & 0x0f : (rx >> 1) & 0x0f;
	return 0;
}
EXPORT_SYMBOL_GPL(oplus_svooc_decode);

/** oplus_svooc_frame() - the ten-bit reply frame for a payload */
u16 oplus_svooc_frame(u8 payload, bool shifted)
{
	u16 frame = (OPLUS_SVOOC_HEAD << 7) | (payload & 0x7f);

	return shifted ? frame >> 1 : frame;
}
EXPORT_SYMBOL_GPL(oplus_svooc_frame);

/** oplus_svooc_reset() - forget the adapter, keeping the board data */
void oplus_svooc_reset(struct oplus_svooc *svooc)
{
	svooc->permit = false;
	svooc->identified = false;
	svooc->model = 0;
	svooc->curve_sent = false;
	svooc->exchange = OPLUS_SVOOC_COMMAND;
	svooc->offered = false;
	svooc->attempts = 0;
}
EXPORT_SYMBOL_GPL(oplus_svooc_reset);

/* The adapter must return both random halves; its reply scrambles them. */
static u8 oplus_svooc_challenge(struct oplus_svooc *svooc, u16 random)
{
	u8 lo = random & 0x7f, hi = (random >> 8) & 0x7f;
	u8 first, second;

	svooc->challenge = (hi << 9) | (lo << 1);
	svooc->response = 0;
	first = ((lo >> 2) & 1) | (((hi >> 4) & 1) << 1) |
		((hi & 1) << 2) | ((lo & 1) << 3) |
		(((hi >> 6) & 1) << 4) | (((lo >> 4) & 1) << 5) |
		(((lo >> 6) & 1) << 6);
	second = ((hi >> 3) & 1) | (((lo >> 1) & 1) << 1) |
		 (((hi >> 2) & 1) << 2) | (((hi >> 1) & 1) << 3) |
		 (((lo >> 5) & 1) << 4) | (((hi >> 5) & 1) << 5) |
		 (((lo >> 3) & 1) << 6);
	svooc->challenge_reply = second;
	svooc->exchange = OPLUS_SVOOC_IDENTIFICATION;
	svooc->remaining = 5;
	return first;
}

/**
 * oplus_svooc_handle() - answer the identification exchanges
 * @svooc: the adapter state
 * @req: the decoded adapter frame
 * @random: random bits for a new challenge
 * @payload: the reply
 *
 * Return: 1 with @payload set when the frame was answered here; 0 for a
 * power command (GET_VBAT, VBUS_OK, GET_CURRENT, AP_STATUS) the caller must
 * answer; a negative error when the adapter broke the protocol.
 */
int oplus_svooc_handle(struct oplus_svooc *svooc,
		       const struct oplus_svooc_request *req, u16 random,
		       u8 *payload)
{
	u8 cmd = req->command;
	int index;

	/* VOOC2 adapters open with a FASTCHG offer; answer it as SuperVOOC. */
	if (req->vooc2) {
		if (cmd != OPLUS_SVOOC_FASTCHG || svooc->permit ||
		    svooc->exchange != OPLUS_SVOOC_COMMAND)
			return -EPROTO;
		svooc->offered = true;
		*payload = svooc->circuit;
		return 1;
	}

	switch (svooc->exchange) {
	case OPLUS_SVOOC_IDENTIFICATION:
		if (svooc->remaining == 5) {
			*payload = svooc->challenge_reply;
		} else {
			svooc->response = (svooc->response << 4) | cmd;
			*payload = 0x10;
		}
		if (!--svooc->remaining) {
			if ((svooc->response & 0xfefe) != svooc->challenge)
				return -EKEYREJECTED;
			svooc->identified = true;
			svooc->exchange = OPLUS_SVOOC_COMMAND;
		}
		return 1;
	case OPLUS_SVOOC_ADAPTER:
		svooc->model = (svooc->model << 4) | cmd;
		*payload = 0x10;
		if (!--svooc->remaining) {
			svooc->model &= 0x7f;
			if (!svooc->model)
				return -EPROTO;
			svooc->exchange = OPLUS_SVOOC_COMMAND;
		}
		return 1;
	case OPLUS_SVOOC_CURVE:
		/* Three pairs, each asked twice, in the order of the curve. */
		index = OPLUS_SVOOC_CURVE_LEN - svooc->remaining;
		if (cmd != index / 2 + 1)
			return -EPROTO;
		*payload = svooc->curve[index];
		if (!--svooc->remaining) {
			svooc->curve_sent = true;
			svooc->exchange = OPLUS_SVOOC_COMMAND;
		}
		return 1;
	case OPLUS_SVOOC_COMMAND:
		break;
	}

	switch (cmd) {
	case OPLUS_SVOOC_FASTCHG:
		svooc->offered = true;
		*payload = svooc->circuit | (svooc->permit ? 0x40 : 0);
		return 1;
	case OPLUS_SVOOC_IDENTIFY:
		if (!svooc->offered || svooc->permit || ++svooc->attempts > 3)
			return -EPROTO;
		svooc->identified = false;
		svooc->curve_sent = false;
		*payload = oplus_svooc_challenge(svooc, random);
		return 1;
	case OPLUS_SVOOC_MODEL:
		if (!svooc->offered || svooc->permit)
			return -EPROTO;
		svooc->model = 0;
		svooc->curve_sent = false;
		svooc->exchange = OPLUS_SVOOC_ADAPTER;
		svooc->remaining = 2;
		*payload = 0x28;
		return 1;
	case OPLUS_SVOOC_BATTERY:
		if (!svooc->identified || !svooc->model || svooc->permit)
			return -EPROTO;
		svooc->curve_sent = false;
		svooc->exchange = OPLUS_SVOOC_CURVE;
		svooc->remaining = OPLUS_SVOOC_CURVE_LEN;
		*payload = 0x18;
		return 1;
	case OPLUS_SVOOC_GET_VBAT:
	case OPLUS_SVOOC_VBUS_OK:
	case OPLUS_SVOOC_GET_CURRENT:
	case OPLUS_SVOOC_AP_STATUS:
		return svooc->permit ? 0 : -EACCES;
	case OPLUS_SVOOC_BAD_CABLE:
		return -ENOLINK;
	default:
		return -EOPNOTSUPP;
	}
}
EXPORT_SYMBOL_GPL(oplus_svooc_handle);

/** oplus_svooc_current() - the GET_CURRENT reply, in 100 mA steps */
int oplus_svooc_current(int ua, u8 *payload)
{
	if (ua < 100000 || ua > 12700000 || ua % 100000)
		return -ERANGE;
	*payload = ua / 100000;
	return 0;
}
EXPORT_SYMBOL_GPL(oplus_svooc_current);

/** oplus_svooc_vbat() - the GET_VBAT reply, 10 mV steps from 3.404 V */
u8 oplus_svooc_vbat(int uv)
{
	if (uv < 3404000)
		return 0;
	if (uv > 4654000)
		return 125;
	return (uv - 3404000) / 10000;
}
EXPORT_SYMBOL_GPL(oplus_svooc_vbat);

/** oplus_svooc_vbus() - the VBUS_OK reply for a 2:1 pump */
u8 oplus_svooc_vbus(int vbus_uv, int vbat_uv)
{
	int delta = vbus_uv - 2 * vbat_uv;

	if (delta < 100000)
		return OPLUS_SVOOC_VBUS_LOW;
	if (delta > 200000)
		return OPLUS_SVOOC_VBUS_HIGH;
	return OPLUS_SVOOC_VBUS_NORMAL;
}
EXPORT_SYMBOL_GPL(oplus_svooc_vbus);

MODULE_DESCRIPTION("Oplus SuperVOOC adapter protocol");
MODULE_LICENSE("GPL");
