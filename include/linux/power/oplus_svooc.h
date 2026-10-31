/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_POWER_OPLUS_SVOOC_H
#define _LINUX_POWER_OPLUS_SVOOC_H

#include <linux/types.h>

/*
 * SuperVOOC frames are ten bits on D+/D-, sent MSB first: a three-bit head
 * and a seven-bit payload. Some PHYs latch the adapter's frames one bit
 * late; the reply then goes out shifted by one bit as well.
 */
#define OPLUS_SVOOC_HEAD		4
#define OPLUS_VOOC2_HEAD		5

#define OPLUS_SVOOC_GET_VBAT		1
#define OPLUS_SVOOC_VBUS_OK		2
#define OPLUS_SVOOC_GET_CURRENT		3
#define OPLUS_SVOOC_FASTCHG		4
#define OPLUS_SVOOC_MODEL		5
#define OPLUS_SVOOC_BAD_CABLE		9
#define OPLUS_SVOOC_AP_STATUS		10
#define OPLUS_SVOOC_IDENTIFY		12
#define OPLUS_SVOOC_BATTERY		13

/* Replies to OPLUS_SVOOC_VBUS_OK, for VBUS against twice the battery. */
#define OPLUS_SVOOC_VBUS_LOW		0x10
#define OPLUS_SVOOC_VBUS_HIGH		0x20
#define OPLUS_SVOOC_VBUS_NORMAL		0x30
/* A GET_VBAT reply asking the adapter to request a new current. */
#define OPLUS_SVOOC_CHANGE_CURRENT	0x7f

#define OPLUS_SVOOC_CURVE_LEN		6

struct oplus_svooc_request {
	u8 command;
	bool shifted;
	bool vooc2;
};

enum oplus_svooc_exchange {
	OPLUS_SVOOC_COMMAND,
	OPLUS_SVOOC_IDENTIFICATION,
	OPLUS_SVOOC_ADAPTER,
	OPLUS_SVOOC_CURVE,
};

/**
 * struct oplus_svooc - adapter identification state
 * @circuit: FASTCHG reply describing the charging circuit, without the
 *	permission bit.
 * @curve: battery curve payloads, in transmission order.
 * @permit: grant fast charging in the next FASTCHG reply.
 * @identified: the adapter answered the challenge.
 * @model: the adapter model, or 0.
 * @curve_sent: the adapter has read the battery curve.
 */
struct oplus_svooc {
	u8 circuit;
	u8 curve[OPLUS_SVOOC_CURVE_LEN];
	bool permit;
	bool identified;
	u8 model;
	bool curve_sent;
	/* private: */
	enum oplus_svooc_exchange exchange;
	bool offered;
	u8 remaining;
	u8 attempts;
	u8 challenge_reply;
	u16 challenge;
	u16 response;
};

int oplus_svooc_decode(u8 rx, struct oplus_svooc_request *req);
u16 oplus_svooc_frame(u8 payload, bool shifted);
void oplus_svooc_reset(struct oplus_svooc *svooc);
int oplus_svooc_handle(struct oplus_svooc *svooc,
		       const struct oplus_svooc_request *req, u16 random,
		       u8 *payload);
int oplus_svooc_current(int ua, u8 *payload);
u8 oplus_svooc_vbat(int uv);
u8 oplus_svooc_vbus(int vbus_uv, int vbat_uv);

#endif
