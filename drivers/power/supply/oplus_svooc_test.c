// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the Oplus SuperVOOC adapter protocol. The expected frames
 * are those of the earlier OnePlus Pad 4 driver, which charged from the
 * stock SuperVOOC adapter.
 */
#include <kunit/test.h>
#include <linux/power/oplus_svooc.h>

static const struct oplus_svooc oplus_svooc_pad4 = {
	.circuit = 0x30,
	.curve = { 0x7e, 0x0b, 0x7e, 0x1a, 0x01, 0x49 },
};

static u16 oplus_svooc_reply(struct kunit *test, struct oplus_svooc *svooc,
			     u8 rx, u16 random)
{
	struct oplus_svooc_request req;
	u8 payload = 0;

	KUNIT_ASSERT_EQ(test, oplus_svooc_decode(rx, &req), 0);
	KUNIT_ASSERT_EQ(test, oplus_svooc_handle(svooc, &req, random, &payload), 1);
	return oplus_svooc_frame(payload, req.shifted);
}

static u8 oplus_svooc_rx(u8 command)
{
	return (OPLUS_SVOOC_HEAD << 5) | (command << 1);
}

static void oplus_svooc_test_decode(struct kunit *test)
{
	struct oplus_svooc_request req;

	KUNIT_EXPECT_EQ(test, oplus_svooc_decode(0xa8, &req), 0);
	KUNIT_EXPECT_TRUE(test, req.vooc2);
	KUNIT_EXPECT_FALSE(test, req.shifted);
	KUNIT_EXPECT_EQ(test, req.command, OPLUS_SVOOC_FASTCHG);

	KUNIT_EXPECT_EQ(test, oplus_svooc_decode(0x44, &req), 0);
	KUNIT_EXPECT_FALSE(test, req.vooc2);
	KUNIT_EXPECT_TRUE(test, req.shifted);
	KUNIT_EXPECT_EQ(test, req.command, OPLUS_SVOOC_FASTCHG);

	KUNIT_EXPECT_EQ(test, oplus_svooc_decode(0x00, &req), -EPROTO);
	KUNIT_EXPECT_EQ(test, oplus_svooc_decode(0xe0, &req), -EPROTO);
}

static void oplus_svooc_test_session(struct kunit *test)
{
	struct oplus_svooc svooc = oplus_svooc_pad4;
	struct oplus_svooc_request req;
	static const u8 curve[] = { 1, 1, 2, 2, 3, 3 };
	static const u16 curve_frames[] = { 0x27e, 0x20b, 0x27e, 0x21a, 0x201, 0x249 };
	u16 response = 0xb478;
	u8 payload;
	int i;

	oplus_svooc_reset(&svooc);
	KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc, 0xa8, 0), 0x230);
	KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc,
			oplus_svooc_rx(OPLUS_SVOOC_FASTCHG), 0), 0x230);

	/* The challenge for 0x5a3c, the adapter's echo, then its response. */
	KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc,
			oplus_svooc_rx(OPLUS_SVOOC_IDENTIFY), 0x5a3c), 0x233);
	KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc,
			oplus_svooc_rx(OPLUS_SVOOC_FASTCHG), 0), 0x259);
	for (i = 3; i >= 0; i--)
		KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc,
				oplus_svooc_rx((response >> (4 * i)) & 0xf), 0), 0x210);
	KUNIT_EXPECT_TRUE(test, svooc.identified);

	KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc,
			oplus_svooc_rx(OPLUS_SVOOC_MODEL), 0), 0x228);
	KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc, oplus_svooc_rx(0x4), 0), 0x210);
	KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc, oplus_svooc_rx(0xb), 0), 0x210);
	KUNIT_EXPECT_EQ(test, svooc.model, 0x4b);

	KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc,
			oplus_svooc_rx(OPLUS_SVOOC_BATTERY), 0), 0x218);
	for (i = 0; i < ARRAY_SIZE(curve); i++)
		KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc,
				oplus_svooc_rx(curve[i]), 0), curve_frames[i]);
	KUNIT_EXPECT_TRUE(test, svooc.curve_sent);

	/* Power commands wait for the grant, then go to the caller. */
	KUNIT_ASSERT_EQ(test, oplus_svooc_decode(oplus_svooc_rx(OPLUS_SVOOC_VBUS_OK), &req), 0);
	KUNIT_EXPECT_EQ(test, oplus_svooc_handle(&svooc, &req, 0, &payload), -EACCES);
	svooc.permit = true;
	KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc, 0x44, 0), 0x138);
	KUNIT_EXPECT_EQ(test, oplus_svooc_reply(test, &svooc,
			oplus_svooc_rx(OPLUS_SVOOC_FASTCHG), 0), 0x270);
	KUNIT_EXPECT_EQ(test, oplus_svooc_handle(&svooc, &req, 0, &payload), 0);
	KUNIT_ASSERT_EQ(test, oplus_svooc_decode(oplus_svooc_rx(OPLUS_SVOOC_IDENTIFY), &req), 0);
	KUNIT_EXPECT_EQ(test, oplus_svooc_handle(&svooc, &req, 0, &payload), -EPROTO);
}

static void oplus_svooc_test_rejects(struct kunit *test)
{
	struct oplus_svooc svooc = oplus_svooc_pad4;
	struct oplus_svooc_request req;
	u8 payload;

	/* No identification before an offer, no curve before identification. */
	oplus_svooc_reset(&svooc);
	KUNIT_ASSERT_EQ(test, oplus_svooc_decode(oplus_svooc_rx(OPLUS_SVOOC_IDENTIFY), &req), 0);
	KUNIT_EXPECT_EQ(test, oplus_svooc_handle(&svooc, &req, 0, &payload), -EPROTO);
	oplus_svooc_reply(test, &svooc, oplus_svooc_rx(OPLUS_SVOOC_FASTCHG), 0);
	KUNIT_ASSERT_EQ(test, oplus_svooc_decode(oplus_svooc_rx(OPLUS_SVOOC_BATTERY), &req), 0);
	KUNIT_EXPECT_EQ(test, oplus_svooc_handle(&svooc, &req, 0, &payload), -EPROTO);

	/* A wrong response fails the identification; its bit 0 is ignored. */
	oplus_svooc_reply(test, &svooc, oplus_svooc_rx(OPLUS_SVOOC_IDENTIFY), 0x5a3c);
	oplus_svooc_reply(test, &svooc, oplus_svooc_rx(OPLUS_SVOOC_FASTCHG), 0);
	oplus_svooc_reply(test, &svooc, oplus_svooc_rx(0xb), 0);
	oplus_svooc_reply(test, &svooc, oplus_svooc_rx(0x4), 0);
	oplus_svooc_reply(test, &svooc, oplus_svooc_rx(0x7), 0);
	KUNIT_ASSERT_EQ(test, oplus_svooc_decode(oplus_svooc_rx(0x0), &req), 0);
	KUNIT_EXPECT_EQ(test, oplus_svooc_handle(&svooc, &req, 0, &payload), -EKEYREJECTED);
	KUNIT_EXPECT_FALSE(test, svooc.identified);

	KUNIT_ASSERT_EQ(test, oplus_svooc_decode(oplus_svooc_rx(OPLUS_SVOOC_BAD_CABLE), &req), 0);
	oplus_svooc_reset(&svooc);
	KUNIT_EXPECT_EQ(test, oplus_svooc_handle(&svooc, &req, 0, &payload), -ENOLINK);
}

static void oplus_svooc_test_replies(struct kunit *test)
{
	u8 payload;

	KUNIT_EXPECT_EQ(test, oplus_svooc_current(6200000, &payload), 0);
	KUNIT_EXPECT_EQ(test, oplus_svooc_frame(payload, false), 0x23e);
	KUNIT_EXPECT_EQ(test, oplus_svooc_current(1500000, &payload), 0);
	KUNIT_EXPECT_EQ(test, oplus_svooc_frame(payload, true), 0x107);
	KUNIT_EXPECT_EQ(test, oplus_svooc_current(1550000, &payload), -ERANGE);
	KUNIT_EXPECT_EQ(test, oplus_svooc_current(0, &payload), -ERANGE);

	KUNIT_EXPECT_EQ(test, oplus_svooc_frame(oplus_svooc_vbat(3820000), false), 0x229);
	KUNIT_EXPECT_EQ(test, oplus_svooc_vbat(3000000), 0);
	KUNIT_EXPECT_EQ(test, oplus_svooc_vbat(4700000), 125);
	KUNIT_EXPECT_EQ(test, oplus_svooc_frame(OPLUS_SVOOC_CHANGE_CURRENT, false), 0x27f);

	KUNIT_EXPECT_EQ(test, oplus_svooc_frame(oplus_svooc_vbus(8050000, 4000000), false), 0x210);
	KUNIT_EXPECT_EQ(test, oplus_svooc_frame(oplus_svooc_vbus(8150000, 4000000), false), 0x230);
	KUNIT_EXPECT_EQ(test, oplus_svooc_frame(oplus_svooc_vbus(8250000, 4000000), false), 0x220);
}

static struct kunit_case oplus_svooc_test_cases[] = {
	KUNIT_CASE(oplus_svooc_test_decode),
	KUNIT_CASE(oplus_svooc_test_session),
	KUNIT_CASE(oplus_svooc_test_rejects),
	KUNIT_CASE(oplus_svooc_test_replies),
	{}
};

static struct kunit_suite oplus_svooc_test_suite = {
	.name = "oplus-svooc",
	.test_cases = oplus_svooc_test_cases,
};
kunit_test_suite(oplus_svooc_test_suite);

MODULE_DESCRIPTION("KUnit tests for the Oplus SuperVOOC protocol");
MODULE_LICENSE("GPL");
