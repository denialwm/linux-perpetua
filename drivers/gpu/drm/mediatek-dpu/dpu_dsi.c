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

#include <drm/display/drm_dsc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>

#include <video/mipi_display.h>

#include "dpu.h"

#define DSI_START			0x000
#define  DSI_START_START		BIT(0)
#define DSI_INTSTA			0x00c
#define  DSI_INT_LPRX_RD_RDY		BIT(0)
#define  DSI_INT_CMD_DONE		BIT(1)
#define  DSI_BUSY			BIT(31)
#define DSI_INTEN			0x008
#define DSI_CON_CTRL			0x010
#define  DSI_RESET			BIT(0)
#define  DSI_DUAL_EN			BIT(4)
#define DSI_MODE_CTRL			0x014
#define  DSI_MODE			GENMASK(1, 0)
#define  DSI_MODE_CMD			0
#define  DSI_MODE_SYNC_PULSE		1
#define  DSI_MODE_SYNC_EVENT		2
#define  DSI_MODE_BURST			3
#define DSI_TXRX_CTRL			0x018
#define  DSI_LANE_EN			GENMASK(5, 2)
#define  DSI_CLK_NON_CONTINUOUS		BIT(6)
#define DSI_PSCTRL			0x01c
#define  DSI_PS_WC			GENMASK(14, 0)
#define  DSI_PS_SEL			GENMASK(19, 16)
#define  DSI_PS_SEL_COMPRESSED		5
#define  DSI_PS_30BIT			BIT(20)
#define DSI_VSA_NL			0x020
#define DSI_VBP_NL			0x024
#define DSI_VFP_NL			0x028
#define DSI_VACT_NL			0x02c
#define DSI_SIZE_CON			0x038
#define  DSI_SIZE_WIDTH			GENMASK(14, 0)
#define  DSI_SIZE_HEIGHT		GENMASK(30, 16)
#define DSI_HSA_WC			0x050
#define DSI_HBP_WC			0x054
#define DSI_HFP_WC			0x058
#define  DSI_HFP_WC_WC			GENMASK(14, 0)
#define  DSI_HFP_HS_VB_PS_WC		GENMASK(30, 16)
#define  DSI_HFP_HS_EN			BIT(31)
#define DSI_CMDQ_SIZE			0x060
#define  DSI_CMDQ_SIZE_SIZE		GENMASK(7, 0)
#define  DSI_CMDQ_SIZE_SEL		BIT(15)
#define DSI_RX_DATA(n)			(0x074 + 4 * (n))
#define DSI_RACK			0x084
#define  DSI_RACK_RACK			BIT(0)
#define DSI_MEM_CONTI			0x090
#define  DSI_WMEM_CONTI			0x3c
#define DSI_RESERVED			0x0f0
#define  DSI_VDE_BLOCK_ULTRA		BIT(29)
#define DSI_PHY_LCCON			0x104
#define  DSI_LC_HS_TX_EN		BIT(0)
#define DSI_PHY_TIMECON0		0x110
#define  DSI_LPX			GENMASK(7, 0)
#define  DSI_HS_PREP			GENMASK(15, 8)
#define  DSI_HS_ZERO			GENMASK(23, 16)
#define  DSI_HS_TRAIL			GENMASK(31, 24)
#define DSI_PHY_TIMECON1		0x114
#define  DSI_TA_GO			GENMASK(7, 0)
#define  DSI_TA_SURE			GENMASK(15, 8)
#define  DSI_TA_GET			GENMASK(23, 16)
#define  DSI_DA_HS_EXIT			GENMASK(31, 24)
#define DSI_PHY_TIMECON2		0x118
#define  DSI_CONT_DET			GENMASK(7, 0)
#define  DSI_DA_HS_SYNC			GENMASK(15, 8)
#define  DSI_CLK_HS_ZERO		GENMASK(23, 16)
#define  DSI_CLK_HS_TRAIL		GENMASK(31, 24)
#define DSI_PHY_TIMECON3		0x11c
#define  DSI_CLK_HS_PREP		GENMASK(7, 0)
#define  DSI_CLK_HS_POST		GENMASK(15, 8)
#define  DSI_CLK_HS_EXIT		GENMASK(23, 16)
#define DSI_DEBUG_SEL			0x170
#define  DSI_MM_RST_SEL			BIT(10)
#define DSI_VM_CMD_CON			0x200
#define  DSI_VM_CMD_EN			BIT(0)
#define  DSI_TS_VFP_EN			BIT(5)
#define DSI_BUF_CON0			0x400
#define  DSI_BUF_EN			BIT(0)
#define DSI_BUF_CON1			0x404
#define  DSI_BUF_VALID_THRESHOLD	GENMASK(14, 0)
#define DSI_TX_BUF_RW_TIMES		0x410
#define DSI_BUF_SODI_HIGH		0x414
#define DSI_BUF_SODI_LOW		0x418
#define DSI_BUF_PREULTRA_HIGH		0x424
#define DSI_BUF_PREULTRA_LOW		0x428
#define DSI_BUF_ULTRA_HIGH		0x42c
#define DSI_BUF_ULTRA_LOW		0x430
#define DSI_BUF_URGENT_HIGH		0x434
#define DSI_BUF_URGENT_LOW		0x438
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

/* D-PHY timings in byte clock cycles, from the lane rate in Mbps. */
static void dsi_phy_timing(void __iomem *dsi, unsigned int rate,
			   unsigned int *hs_overhead)
{
	unsigned int lpx = round_up(rate * 80 / 8000 + 1, 2);
	unsigned int hs_prep = round_up((rate * 59 + 4000) / 8000 + 1, 2);
	unsigned int hs_zero = (rate * 163 + 11000) / 8000 + 1 - hs_prep;
	unsigned int hs_trail = (rate * 66 + (rate < 740 ? 44300 : 42000)) /
				8000 + 1;
	unsigned int hs_exit = (rate * 118) / 8000 + 1;
	unsigned int clk_prep = round_up(rate * 57 / 8000 + 1, 2);
	unsigned int clk_zero = rate * 330 / 8000 + 1 - clk_prep;
	unsigned int clk_trail = (rate * 65 + 52000) / 8000 + 1;
	unsigned int clk_post = (rate * 65 + 53000) / 8000 + 1;
	unsigned int clk_exit = rate * 118 / 8000 + 1;

	/* The exit time must be odd. */
	hs_exit += hs_exit % 2 ? 2 : 1;

	writel(FIELD_PREP(DSI_LPX, lpx) | FIELD_PREP(DSI_HS_PREP, hs_prep) |
	       FIELD_PREP(DSI_HS_ZERO, hs_zero) |
	       FIELD_PREP(DSI_HS_TRAIL, hs_trail), dsi + DSI_PHY_TIMECON0);
	writel(FIELD_PREP(DSI_TA_GO, 4 * lpx) |
	       FIELD_PREP(DSI_TA_SURE, 3 * lpx / 2) |
	       FIELD_PREP(DSI_TA_GET, 5 * lpx) |
	       FIELD_PREP(DSI_DA_HS_EXIT, hs_exit), dsi + DSI_PHY_TIMECON1);
	writel(FIELD_PREP(DSI_DA_HS_SYNC, 1) |
	       FIELD_PREP(DSI_CLK_HS_ZERO, clk_zero) |
	       FIELD_PREP(DSI_CLK_HS_TRAIL, clk_trail), dsi + DSI_PHY_TIMECON2);
	writel(FIELD_PREP(DSI_CLK_HS_PREP, clk_prep) |
	       FIELD_PREP(DSI_CLK_HS_POST, clk_post) |
	       FIELD_PREP(DSI_CLK_HS_EXIT, clk_exit), dsi + DSI_PHY_TIMECON3);

	/* Byte cycles per lane for a return from LP to HS. */
	*hs_overhead = lpx + hs_exit + hs_prep + hs_zero + 2;
}

/*
 * The thresholds of the buffer between the display pipe and the link, in
 * buffer units: how many microseconds of link traffic it holds before it
 * asks the memory system for more, with more urgency the emptier it gets.
 */
static void dsi_buffer(struct dpu *dpu, void __iomem *dsi, unsigned int rate,
		       unsigned int lanes, unsigned int words,
		       unsigned int height)
{
	const struct dpu_soc *soc = dpu->soc;
	unsigned int unit = soc->dsi_buf_unit;
	unsigned int drain = rate * lanes / 8 / unit;
	unsigned int fill = soc->mmsys_mhz * 3 / unit;
	unsigned int size = soc->dsi_buf_size * soc->dsi_sram_unit / unit;

#define DSI_BUF_US(us)	((us) * rate * lanes / 8 / unit)

	writel(size - 12 * (fill - drain) / 10, dsi + DSI_BUF_SODI_HIGH);
	writel(DSI_BUF_US(28), dsi + DSI_BUF_SODI_LOW);
	writel(DSI_BUF_US(26), dsi + DSI_BUF_PREULTRA_HIGH);
	writel(DSI_BUF_US(25), dsi + DSI_BUF_PREULTRA_LOW);
	writel(DSI_BUF_US(25), dsi + DSI_BUF_ULTRA_HIGH);
	writel(DSI_BUF_US(23), dsi + DSI_BUF_ULTRA_LOW);
	writel(DSI_BUF_US(soc->dsi_urgent_high_us), dsi + DSI_BUF_URGENT_HIGH);
	writel(DSI_BUF_US(soc->dsi_urgent_low_us), dsi + DSI_BUF_URGENT_LOW);

#undef DSI_BUF_US

	/* The buffer is read in words of 9 bytes. */
	writel(DIV_ROUND_UP(words * height, 9), dsi + DSI_TX_BUF_RW_TIMES);
	dsi_update_bits(dsi, DSI_BUF_CON1, DSI_BUF_VALID_THRESHOLD, 0);
	dsi_update_bits(dsi, DSI_DEBUG_SEL, DSI_MM_RST_SEL, DSI_MM_RST_SEL);
	dsi_update_bits(dsi, DSI_RESERVED, DSI_VDE_BLOCK_ULTRA, 0);
	dsi_update_bits(dsi, DSI_BUF_CON0, DSI_BUF_EN, DSI_BUF_EN);
}

/*
 * Program both links for @mode while they are stopped: lanes, D-PHY
 * timings, the compressed stream and the video timings. Each link carries
 * the whole vertical timing and its own stripe of every line.
 */
void dpu_dsi_config(struct dpu *dpu, const struct drm_display_mode *mode)
{
	const struct mipi_dsi_device *device = dpu->dsi_device;
	const struct drm_dsc_config *dsc = device->dsc;
	unsigned int rate = DIV_ROUND_UP(device->hs_rate, 1000000);
	unsigned int hsa = mode->hsync_end - mode->hsync_start;
	unsigned int hbp = mode->htotal - mode->hsync_end;
	unsigned int hfp = mode->hsync_start - mode->hdisplay;
	unsigned int height = mode->vdisplay;
	unsigned int words = dsc->slice_chunk_size * dsc->slice_count;
	u32 hsa_wc, hbp_wc, hfp_wc;
	unsigned int link;

	/* Packet headers and footers take their bytes from the porches. */
	if (device->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) {
		hsa_wc = ALIGN(hsa * 3 - 10, 4);
		hbp_wc = ALIGN(hbp * 3 - 10, 4);
	} else {
		hsa_wc = ALIGN(hsa * 3 - 4, 4);
		hbp_wc = ALIGN((hbp + hsa) * 3 - 10, 4);
	}
	hfp_wc = ALIGN(hfp * 3 - 12, 4);

	for (link = 0; link < dpu->soc->num_pipes; link++) {
		void __iomem *dsi = dpu->dsi[link];
		unsigned int hs_overhead;

		dsi_update_bits(dsi, DSI_CON_CTRL, DSI_RESET, DSI_RESET);
		dsi_update_bits(dsi, DSI_CON_CTRL, DSI_RESET, 0);
		writel(0, dsi + DSI_INTEN);
		writel(0, dsi + DSI_INTSTA);

		dsi_phy_timing(dsi, rate, &hs_overhead);
		writel(FIELD_PREP(DSI_LANE_EN, GENMASK(device->lanes - 1, 0)) |
		       (device->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS ?
			DSI_CLK_NON_CONTINUOUS : 0), dsi + DSI_TXRX_CTRL);
		writel(DSI_WMEM_CONTI, dsi + DSI_MEM_CONTI);
		dsi_buffer(dpu, dsi, rate, device->lanes, words, height);

		dsi_update_bits(dsi, DSI_PSCTRL,
				DSI_PS_WC | DSI_PS_SEL | DSI_PS_30BIT,
				FIELD_PREP(DSI_PS_WC, words) |
				FIELD_PREP(DSI_PS_SEL, DSI_PS_SEL_COMPRESSED));
		writel(FIELD_PREP(DSI_SIZE_WIDTH, DIV_ROUND_UP(words, 3)) |
		       FIELD_PREP(DSI_SIZE_HEIGHT, height), dsi + DSI_SIZE_CON);

		writel(mode->vsync_end - mode->vsync_start, dsi + DSI_VSA_NL);
		writel(mode->vtotal - mode->vsync_end, dsi + DSI_VBP_NL);
		writel(mode->vsync_start - mode->vdisplay, dsi + DSI_VFP_NL);
		writel(height, dsi + DSI_VACT_NL);
		writel(hsa_wc, dsi + DSI_HSA_WC);
		writel(hbp_wc, dsi + DSI_HBP_WC);
		/* Return to LP in the vertical blanking, once per frame. */
		writel(FIELD_PREP(DSI_HFP_WC_WC, hfp_wc) | DSI_HFP_HS_EN |
		       FIELD_PREP(DSI_HFP_HS_VB_PS_WC,
				  words - hs_overhead * device->lanes),
		       dsi + DSI_HFP_WC);

		dsi_update_bits(dsi, DSI_VM_CMD_CON,
				DSI_VM_CMD_EN | DSI_TS_VFP_EN,
				DSI_VM_CMD_EN | DSI_TS_VFP_EN);
		dsi_update_bits(dsi, DSI_CMDQ_SIZE, DSI_CMDQ_SIZE_SEL,
				DSI_CMDQ_SIZE_SEL);
	}
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
