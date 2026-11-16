// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs101 DPP (Display Pre-Processor) channel.
 *
 * A DPP channel fetches one layer from memory (its RDMA, in the "dma"
 * bank) and feeds it to a DECON window. This driver programs a channel for
 * plain RGB scan-out on behalf of the gs101 DECON driver: no scaling,
 * rotation, compression or colour conversion.
 *
 * The bootloader leaves channel 0 scanning out its splash screen. Only
 * what differs per framebuffer is written: format, sizes, crop offset and
 * base address. The rest of the channel configuration is kept as found.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_print.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "gs101_drm_dpp.h"

/* DMA bank */
#define RDMA_ENABLE			0x0000
#define RDMA_IN_CTRL_0			0x0008
#define  IDMA_IMG_FORMAT_MASK		GENMASK(13, 8)
#define  IDMA_IMG_FORMAT(v)		((v) << 8)
#define RDMA_SRC_SIZE			0x0010
#define RDMA_SRC_OFFSET			0x0014
#define RDMA_IMG_SIZE			0x0018
#define RDMA_BASEADDR_Y8		0x0040
#define GLB_DPU_DMA_VERSION		0x0f00

/* DPP bank */
#define DPP_COM_VERSION			0x0000

/*
 * Image formats are named after the byte order in memory: B, G, R, A is
 * DRM_FORMAT_ARGB8888 on a little-endian bus.
 */
#define IMG_FORMAT_BGRA8888		0
#define IMG_FORMAT_BGRX8888		4

/* Height in the upper half, width in the lower, for all size registers. */
#define RDMA_HW(h, w)			((((h) & 0xffff) << 16) | ((w) & 0xffff))
/* SRC_OFFSET and IMG_SIZE fields are 14 bits wide. */
#define RDMA_MAX_IMG			0x3fff

struct gs101_dpp {
	struct device *dev;
	struct drm_device *drm;
	void *dma_priv;
	void __iomem *dma;
	void __iomem *regs;
	/* The bootloader's image size, for the black buffer. */
	u32 boot_img_size;
	void *black;
	dma_addr_t black_dma;
};

/**
 * gs101_dpp_get - look up the DPP channel of a device tree node
 * @consumer: the device that will use the channel
 * @np: the DPP node
 *
 * Links @consumer to the channel's device, so that unbinding the channel
 * first unbinds @consumer.
 *
 * Returns the channel, or an ERR_PTR: -EPROBE_DEFER while the channel has
 * not probed yet.
 */
struct gs101_dpp *gs101_dpp_get(struct device *consumer,
				struct device_node *np)
{
	struct platform_device *pdev;
	struct gs101_dpp *dpp;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	dpp = platform_get_drvdata(pdev);
	if (dpp && !device_link_add(consumer, &pdev->dev,
				    DL_FLAG_AUTOREMOVE_CONSUMER))
		dpp = ERR_PTR(-EINVAL);
	put_device(&pdev->dev);

	return dpp ?: ERR_PTR(-EPROBE_DEFER);
}

static u32 gs101_dpp_img_format(const struct drm_format_info *format)
{
	return format->has_alpha ? IMG_FORMAT_BGRA8888 : IMG_FORMAT_BGRX8888;
}

/**
 * gs101_dpp_check - check that the channel can scan out a framebuffer
 * @dpp: the channel
 * @fb: the framebuffer
 * @w: width of the source rectangle
 * @h: height of the source rectangle
 */
int gs101_dpp_check(struct gs101_dpp *dpp, struct drm_framebuffer *fb,
		    unsigned int w, unsigned int h)
{
	dma_addr_t addr = exynos_drm_fb_dma_addr(fb, 0);

	/*
	 * There is no stride register: the row length is the width in
	 * SRC_SIZE. A pitch that is not a whole number of pixels cannot be
	 * described.
	 */
	if (fb->pitches[0] % fb->format->cpp[0])
		return -EINVAL;

	if (w > RDMA_MAX_IMG || h > RDMA_MAX_IMG)
		return -EINVAL;

	/* RDMA_BASEADDR_Y8 is the whole address. */
	if (upper_32_bits(addr))
		return -EINVAL;

	return 0;
}

/**
 * gs101_dpp_update - scan out a framebuffer
 * @dpp: the channel
 * @fb: the framebuffer, accepted by gs101_dpp_check()
 * @x: left of the source rectangle
 * @y: top of the source rectangle
 * @w: width of the source rectangle
 * @h: height of the source rectangle
 *
 * The registers are latched by the DECON shadow update that follows.
 */
void gs101_dpp_update(struct gs101_dpp *dpp, struct drm_framebuffer *fb,
		      unsigned int x, unsigned int y,
		      unsigned int w, unsigned int h)
{
	dma_addr_t addr = exynos_drm_fb_dma_addr(fb, 0);
	u32 val;

	val = readl(dpp->dma + RDMA_IN_CTRL_0);
	val &= ~IDMA_IMG_FORMAT_MASK;
	val |= IDMA_IMG_FORMAT(gs101_dpp_img_format(fb->format));
	writel(val, dpp->dma + RDMA_IN_CTRL_0);

	/* The buffer, whose row length is the pitch, and the part shown. */
	writel(RDMA_HW(fb->height, fb->pitches[0] / fb->format->cpp[0]),
	       dpp->dma + RDMA_SRC_SIZE);
	writel(RDMA_HW(y, x), dpp->dma + RDMA_SRC_OFFSET);
	writel(RDMA_HW(h, w), dpp->dma + RDMA_IMG_SIZE);
	writel(lower_32_bits(addr), dpp->dma + RDMA_BASEADDR_Y8);
}

/**
 * gs101_dpp_disable - stop fetching from the last framebuffer
 * @dpp: the channel
 *
 * The DECON window stays enabled, so the channel fetches a black buffer of
 * the bootloader's image size instead of memory that may be freed.
 */
void gs101_dpp_disable(struct gs101_dpp *dpp)
{
	u32 val;

	if (!dpp->black)
		return;

	val = readl(dpp->dma + RDMA_IN_CTRL_0);
	val &= ~IDMA_IMG_FORMAT_MASK;
	val |= IDMA_IMG_FORMAT(IMG_FORMAT_BGRX8888);
	writel(val, dpp->dma + RDMA_IN_CTRL_0);

	writel(dpp->boot_img_size, dpp->dma + RDMA_SRC_SIZE);
	writel(0, dpp->dma + RDMA_SRC_OFFSET);
	writel(dpp->boot_img_size, dpp->dma + RDMA_IMG_SIZE);
	writel(lower_32_bits(dpp->black_dma), dpp->dma + RDMA_BASEADDR_Y8);
}

static int gs101_dpp_bind(struct device *dev, struct device *master,
			  void *data)
{
	struct gs101_dpp *dpp = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	size_t size;
	int ret;

	dpp->drm = drm;

	/*
	 * Allocate framebuffers against the channel that fetches them, with
	 * its 32-bit mask, rather than against the virtual DRM device.
	 */
	ret = exynos_drm_register_dma(drm, dev, &dpp->dma_priv);
	if (ret)
		return ret;

	size = (size_t)(dpp->boot_img_size & 0xffff) *
	       (dpp->boot_img_size >> 16) * 4;
	if (size && !dpp->black)
		dpp->black = dmam_alloc_coherent(dev, size, &dpp->black_dma,
						 GFP_KERNEL);
	if (dpp->black && upper_32_bits(dpp->black_dma)) {
		dmam_free_coherent(dev, size, dpp->black, dpp->black_dma);
		dpp->black = NULL;
	}
	if (!dpp->black)
		dev_warn(dev, "no black buffer for a disabled plane\n");

	return 0;
}

static void gs101_dpp_unbind(struct device *dev, struct device *master,
			     void *data)
{
	struct gs101_dpp *dpp = dev_get_drvdata(dev);

	exynos_drm_unregister_dma(dpp->drm, dev, &dpp->dma_priv);
}

static const struct component_ops gs101_dpp_component_ops = {
	.bind	= gs101_dpp_bind,
	.unbind	= gs101_dpp_unbind,
};

static int gs101_dpp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_bulk_data *clks;
	struct gs101_dpp *dpp;
	int ret;

	dpp = devm_kzalloc(dev, sizeof(*dpp), GFP_KERNEL);
	if (!dpp)
		return -ENOMEM;

	dpp->dev = dev;

	dpp->dma = devm_platform_ioremap_resource_byname(pdev, "dma");
	if (IS_ERR(dpp->dma))
		return PTR_ERR(dpp->dma);

	dpp->regs = devm_platform_ioremap_resource_byname(pdev, "dpp");
	if (IS_ERR(dpp->regs))
		return PTR_ERR(dpp->regs);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA mask\n");

	/* The registers only answer with the DPU clocks running. */
	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	dpp->boot_img_size = readl(dpp->dma + RDMA_IMG_SIZE);

	dev_info(dev,
		 "DPP %#010x, DMA %#010x; RDMA enable %#010x ctrl %#010x size %#010x base %#010x\n",
		 readl(dpp->regs + DPP_COM_VERSION),
		 readl(dpp->dma + GLB_DPU_DMA_VERSION),
		 readl(dpp->dma + RDMA_ENABLE),
		 readl(dpp->dma + RDMA_IN_CTRL_0),
		 dpp->boot_img_size,
		 readl(dpp->dma + RDMA_BASEADDR_Y8));

	platform_set_drvdata(pdev, dpp);

	return component_add(dev, &gs101_dpp_component_ops);
}

static void gs101_dpp_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &gs101_dpp_component_ops);
}

static const struct of_device_id gs101_dpp_of_match[] = {
	{ .compatible = "google,gs101-dpp" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs101_dpp_of_match);

struct platform_driver gs101_dpp_driver = {
	.probe		= gs101_dpp_probe,
	.remove		= gs101_dpp_remove,
	.driver		= {
		.name		= "gs101-dpp",
		.of_match_table	= gs101_dpp_of_match,
	},
};
