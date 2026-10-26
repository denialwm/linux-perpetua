// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek Dimensity display processor: DSI links
 *
 * Each pipe drives one DSI link to the same panel. The first link is the
 * master: in dual mode the second link follows its start, so a command
 * queued on both goes out on both at once.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/unaligned.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_print.h>

#include <video/mipi_display.h>

#include "dpu.h"

#define DSI_START			0x000
#define  DSI_START_START		BIT(0)
#define DSI_INTSTA			0x00c
#define  DSI_INT_LPRX_RD_RDY		BIT(0)
#define  DSI_INT_CMD_DONE		BIT(1)
#define  DSI_BUSY			BIT(31)
#define DSI_CON_CTRL			0x010
#define  DSI_DUAL_EN			BIT(4)
#define DSI_MODE_CTRL			0x014
#define  DSI_MODE			GENMASK(1, 0)
#define  DSI_MODE_CMD			0
#define  DSI_MODE_SYNC_PULSE		1
#define  DSI_MODE_SYNC_EVENT		2
#define  DSI_MODE_BURST			3
#define DSI_CMDQ_SIZE			0x060
#define  DSI_CMDQ_SIZE_SIZE		GENMASK(7, 0)
#define  DSI_CMDQ_SIZE_SEL		BIT(15)
#define DSI_RX_DATA(n)			(0x074 + 4 * (n))
#define DSI_RACK			0x084
#define  DSI_RACK_RACK			BIT(0)
#define DSI_PHY_LCCON			0x104
#define  DSI_LC_HS_TX_EN		BIT(0)
#define DSI_CMDQ(n)			(0xd00 + 4 * (n))
#define  DSI_CMDQ_TYPE			GENMASK(7, 0)
#define  DSI_CMDQ_SHORT			0
#define  DSI_CMDQ_LONG			2
#define  DSI_CMDQ_BTA			BIT(2)
#define  DSI_CMDQ_HEADER		GENMASK(31, 8)

#define DSI_CMDQ_WORDS			32
#define DSI_IDLE_TIMEOUT_US		50000

#define MASTER				0

static void dsi_update_bits(void __iomem *dsi, u32 reg, u32 mask, u32 val)
{
	writel((readl(dsi + reg) & ~mask) | (val & mask), dsi + reg);
}

static int dsi_wait_idle(struct dpu *dpu, unsigned int link)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(dpu->dsi[link] + DSI_INTSTA, status,
				 !(status & DSI_BUSY), 10, DSI_IDLE_TIMEOUT_US);
	if (ret)
		drm_err(&dpu->drm, "dsi%u stays busy: %08x\n", link, status);

	return ret;
}

/* The second link joins the master's start only in dual mode. */
static void dsi_dual(struct dpu *dpu, bool enable)
{
	unsigned int link;

	for (link = 1; link < dpu->soc->num_pipes; link++)
		dsi_update_bits(dpu->dsi[link], DSI_CON_CTRL, DSI_DUAL_EN,
				enable ? DSI_DUAL_EN : 0);
}

static void dsi_start(struct dpu *dpu)
{
	void __iomem *dsi = dpu->dsi[MASTER];

	writel(0, dsi + DSI_START);
	writel(DSI_START_START, dsi + DSI_START);
}

/* Stop the video stream at the end of its frame; the links stay in LP. */
int dpu_dsi_video_stop(struct dpu *dpu)
{
	unsigned int link;
	int ret = 0;

	for (link = 0; link < dpu->soc->num_pipes; link++) {
		dsi_update_bits(dpu->dsi[link], DSI_MODE_CTRL, DSI_MODE,
				DSI_MODE_CMD);
		writel(0, dpu->dsi[link] + DSI_START);
	}

	for (link = 0; link < dpu->soc->num_pipes; link++) {
		ret = ret ?: dsi_wait_idle(dpu, link);
		dsi_update_bits(dpu->dsi[link], DSI_PHY_LCCON, DSI_LC_HS_TX_EN, 0);
	}

	return ret;
}

void dpu_dsi_video_start(struct dpu *dpu)
{
	unsigned long flags = dpu->dsi_device->mode_flags;
	u32 mode = DSI_MODE_SYNC_EVENT;
	unsigned int link;

	if (flags & MIPI_DSI_MODE_VIDEO_BURST)
		mode = DSI_MODE_BURST;
	else if (flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		mode = DSI_MODE_SYNC_PULSE;

	for (link = 0; link < dpu->soc->num_pipes; link++) {
		dsi_update_bits(dpu->dsi[link], DSI_PHY_LCCON, DSI_LC_HS_TX_EN,
				DSI_LC_HS_TX_EN);
		dsi_update_bits(dpu->dsi[link], DSI_MODE_CTRL, DSI_MODE, mode);
	}

	dsi_dual(dpu, true);
	dsi_start(dpu);
}

static void dsi_queue(void __iomem *dsi, const struct mipi_dsi_packet *packet,
		      u32 type)
{
	unsigned int words = 1, i;

	writel(FIELD_PREP(DSI_CMDQ_TYPE, type) |
	       FIELD_PREP(DSI_CMDQ_HEADER, packet->header[0] |
			  packet->header[1] << 8 | packet->header[2] << 16),
	       dsi + DSI_CMDQ(0));

	for (i = 0; i < packet->payload_length; i += 4) {
		u32 word = 0;
		unsigned int j;

		for (j = 0; j < 4 && i + j < packet->payload_length; j++)
			word |= packet->payload[i + j] << (8 * j);
		writel(word, dsi + DSI_CMDQ(words++));
	}

	dsi_update_bits(dsi, DSI_CMDQ_SIZE,
			DSI_CMDQ_SIZE_SIZE | DSI_CMDQ_SIZE_SEL,
			FIELD_PREP(DSI_CMDQ_SIZE_SIZE, words) | DSI_CMDQ_SIZE_SEL);
}

/* Send one packet on @links links, started together by the master. */
static int dsi_send(struct dpu *dpu, const struct mipi_dsi_msg *msg,
		    unsigned int links, u32 type)
{
	struct mipi_dsi_packet packet;
	unsigned int link;
	int ret;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		return ret;
	if (DIV_ROUND_UP(packet.payload_length, 4) >= DSI_CMDQ_WORDS)
		return -EMSGSIZE;

	for (link = 0; link < links; link++) {
		ret = dsi_wait_idle(dpu, link);
		if (ret)
			return ret;
		dsi_update_bits(dpu->dsi[link], DSI_INTSTA,
				DSI_INT_CMD_DONE | DSI_INT_LPRX_RD_RDY, 0);
		dsi_queue(dpu->dsi[link], &packet, type);
	}

	dsi_dual(dpu, links > 1);
	dsi_start(dpu);
	writel(0, dpu->dsi[MASTER] + DSI_START);

	return 0;
}

static ssize_t dsi_read(struct dpu *dpu, const struct mipi_dsi_msg *msg)
{
	void __iomem *dsi = dpu->dsi[MASTER];
	u8 size[2] = { min_t(size_t, msg->rx_len, 10), 0 };
	struct mipi_dsi_msg max_size = {
		.channel = msg->channel,
		.type = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
		.tx_len = sizeof(size),
		.tx_buf = size,
	};
	u8 rx[16], *data = rx + 1;
	size_t len;
	u32 status;
	int i, ret;

	/* Only the master answers. */
	ret = dsi_send(dpu, &max_size, 1, DSI_CMDQ_SHORT);
	ret = ret ?: dsi_wait_idle(dpu, MASTER);
	ret = ret ?: dsi_send(dpu, msg, 1, DSI_CMDQ_BTA);
	if (ret)
		goto out;

	ret = readl_poll_timeout(dsi + DSI_INTSTA, status,
				 status & DSI_INT_LPRX_RD_RDY, 10,
				 DSI_IDLE_TIMEOUT_US);
	if (ret) {
		drm_err(&dpu->drm, "no read response: %08x\n", status);
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(rx) / 4; i++)
		put_unaligned_le32(readl(dsi + DSI_RX_DATA(i)), rx + 4 * i);
	dsi_update_bits(dsi, DSI_INTSTA, DSI_INT_LPRX_RD_RDY, 0);
	dsi_update_bits(dsi, DSI_RACK, DSI_RACK_RACK, DSI_RACK_RACK);
	ret = dsi_wait_idle(dpu, MASTER);
	if (ret)
		goto out;

	switch (rx[0]) {
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		len = 1;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		len = 2;
		break;
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		len = min_t(size_t, get_unaligned_le16(rx + 1), 10);
		data = rx + 4;
		break;
	default:
		drm_err(&dpu->drm, "read response %*ph\n", 4, rx);
		ret = -EIO;
		goto out;
	}

	len = min(len, msg->rx_len);
	memcpy(msg->rx_buf, data, len);
	ret = len;

out:
	dsi_dual(dpu, true);
	return ret;
}

static ssize_t dpu_dsi_transfer(struct mipi_dsi_host *host,
				const struct mipi_dsi_msg *msg)
{
	struct dpu *dpu = container_of(host, struct dpu, dsi_host);
	unsigned int link;
	int ret;

	/* Commands go out between frames only in command mode. */
	if (FIELD_GET(DSI_MODE, readl(dpu->dsi[MASTER] + DSI_MODE_CTRL)) !=
	    DSI_MODE_CMD)
		return -EBUSY;

	if (msg->rx_len)
		return dsi_read(dpu, msg);

	ret = dsi_send(dpu, msg, dpu->soc->num_pipes,
		       mipi_dsi_packet_format_is_long(msg->type) ?
		       DSI_CMDQ_LONG : DSI_CMDQ_SHORT);
	for (link = 0; !ret && link < dpu->soc->num_pipes; link++)
		ret = dsi_wait_idle(dpu, link);

	return ret ?: msg->tx_len;
}

static int dpu_dsi_attach(struct mipi_dsi_host *host,
			  struct mipi_dsi_device *device)
{
	struct dpu *dpu = container_of(host, struct dpu, dsi_host);

	return dpu_bind(dpu, device);
}

static int dpu_dsi_detach(struct mipi_dsi_host *host,
			  struct mipi_dsi_device *device)
{
	struct dpu *dpu = container_of(host, struct dpu, dsi_host);

	dpu_unbind(dpu);
	return 0;
}

static const struct mipi_dsi_host_ops dpu_dsi_host_ops = {
	.attach = dpu_dsi_attach,
	.detach = dpu_dsi_detach,
	.transfer = dpu_dsi_transfer,
};

int dpu_dsi_host_register(struct dpu *dpu)
{
	dpu->dsi_host.dev = dpu->drm.dev;
	dpu->dsi_host.ops = &dpu_dsi_host_ops;

	return mipi_dsi_host_register(&dpu->dsi_host);
}
