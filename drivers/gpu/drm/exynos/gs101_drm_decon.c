// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs101 DECON (Display and Enhancement Controller).
 *
 * DECON blends windows fed by DPP channels and sends the result to the DSI
 * host. On the Pixel 6 the panel runs in command mode: DECON transfers a
 * frame on each tearing effect (TE) pulse of the panel, through its
 * hardware trigger, and raises frame done when the transfer ends.
 *
 * The bootloader leaves DECON0 fully configured, with one window scanning
 * out its splash screen from DPP channel 0, the DSC encoder and the DSI
 * interface running, and the hardware trigger masked. This driver takes
 * that configuration over as it is: it points the window's DPP channel at
 * the framebuffer, requests the shadow update and unmasks the trigger. It
 * does not initialise DECON from reset, so it refuses a DECON the
 * bootloader has not set up.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_plane.h"
#include "gs101_drm_dpp.h"

/* main bank */
#define DECON_VERSION			0x0000
#define DECON_GLOBAL_CON		0x0020
#define DECON_TRIG_CON			0x0030
#define  HW_TRIG_MASK_DECON		BIT(4)
#define  HW_TRIG_EN			BIT(0)
#define DECON_SHD_REG_UP_REQ		0x0050
#define  SHD_REG_UP_REQ_GLOBAL		BIT(31)
#define  SHD_REG_UP_REQ_CMP		BIT(20)
#define  SHD_REG_UP_REQ_WIN_ALL		GENMASK(5, 0)
#define DECON_INT_EN			0x0060
#define  INT_EN_FRAME_DONE		BIT(13)
#define  INT_EN				BIT(0)
#define DECON_INT_PEND			0x0070
#define  INT_PEND_FRAME_DONE		BIT(13)

/* win bank, one 0x1000 block per window */
#define WIN_START_POSITION(n)		(0x1000 * (n) + 0x000c)
#define WIN_END_POSITION(n)		(0x1000 * (n) + 0x0010)
#define  WIN_POS_Y(v)			(((v) >> 16) & 0x3fff)
#define  WIN_POS_X(v)			((v) & 0x3fff)

/* wincon bank, one 0x1000 block per window */
#define DECON_CON_WIN(n)		(0x1000 * (n))
#define  WIN_CHMAP(v)			(((v) >> 4) & 0x7)
#define  WIN_EN				BIT(0)

#define GS101_DECON_WINDOWS		6

/*
 * Window blending is not programmed yet, so formats with alpha are not
 * offered: the plane core would expose an alpha it cannot honour.
 */
static const uint32_t gs101_decon_formats[] = {
	DRM_FORMAT_XRGB8888,
};

struct gs101_decon {
	struct device *dev;
	void __iomem *main;
	void __iomem *win;
	void __iomem *wincon;
	int irq;
	struct gs101_dpp *dpp;
	struct exynos_drm_crtc *crtc;
	struct exynos_drm_plane plane;
	struct exynos_drm_plane_config config;
	/* The window the bootloader left scanning out, and its size. */
	unsigned int window;
	unsigned int width;
	unsigned int height;
	bool active;
};

static int gs101_decon_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct gs101_decon *decon = crtc->ctx;

	writel(INT_EN | INT_EN_FRAME_DONE, decon->main + DECON_INT_EN);

	return 0;
}

static void gs101_decon_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct gs101_decon *decon = crtc->ctx;

	writel(0, decon->main + DECON_INT_EN);
}

static void gs101_decon_atomic_enable(struct exynos_drm_crtc *crtc)
{
	struct gs101_decon *decon = crtc->ctx;

	/* The trigger is unmasked by the flush that follows. */
	decon->active = true;
}

static void gs101_decon_atomic_disable(struct exynos_drm_crtc *crtc)
{
	struct gs101_decon *decon = crtc->ctx;
	u32 val;

	/* Stop transferring; the panel keeps showing the last frame. */
	val = readl(decon->main + DECON_TRIG_CON);
	writel(val | HW_TRIG_MASK_DECON, decon->main + DECON_TRIG_CON);

	decon->active = false;
}

static enum drm_mode_status
gs101_decon_mode_valid(struct exynos_drm_crtc *crtc,
		       const struct drm_display_mode *mode)
{
	struct gs101_decon *decon = crtc->ctx;

	/* The timing, DSC and DSI configuration are the bootloader's. */
	if (mode->hdisplay != decon->width || mode->vdisplay != decon->height)
		return MODE_BAD;

	return MODE_OK;
}

static int gs101_decon_atomic_check(struct exynos_drm_crtc *crtc,
				    struct drm_crtc_state *crtc_state)
{
	struct gs101_decon *decon = crtc->ctx;
	struct drm_plane_state *plane_state;
	struct exynos_drm_plane_state *state;

	plane_state = drm_atomic_get_new_plane_state(crtc_state->state,
						     &decon->plane.base);
	if (!plane_state || !plane_state->crtc || !plane_state->fb)
		return 0;

	/*
	 * The window keeps the bootloader's geometry and DPP does not scale:
	 * the plane covers the whole CRTC with an unscaled source.
	 */
	state = to_exynos_plane_state(plane_state);
	if (state->crtc.x || state->crtc.y ||
	    state->crtc.w != decon->width || state->crtc.h != decon->height ||
	    state->src.w != state->crtc.w || state->src.h != state->crtc.h)
		return -EINVAL;

	return gs101_dpp_check(decon->dpp, plane_state->fb,
			       state->src.w, state->src.h);
}

static void gs101_decon_update_plane(struct exynos_drm_crtc *crtc,
				     struct exynos_drm_plane *plane)
{
	struct exynos_drm_plane_state *state =
		to_exynos_plane_state(plane->base.state);
	struct gs101_decon *decon = crtc->ctx;

	gs101_dpp_update(decon->dpp, state->base.fb, state->src.x,
			 state->src.y, state->src.w, state->src.h);
}

static void gs101_decon_disable_plane(struct exynos_drm_crtc *crtc,
				      struct exynos_drm_plane *plane)
{
	struct gs101_decon *decon = crtc->ctx;

	gs101_dpp_disable(decon->dpp);
}

static void gs101_decon_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct gs101_decon *decon = crtc->ctx;
	u32 val;

	if (decon->active) {
		/*
		 * The windows latch separately from the global request; ask
		 * for all of them, or the window keeps what the bootloader
		 * latched.
		 */
		writel(SHD_REG_UP_REQ_GLOBAL | SHD_REG_UP_REQ_CMP |
		       SHD_REG_UP_REQ_WIN_ALL,
		       decon->main + DECON_SHD_REG_UP_REQ);

		val = readl(decon->main + DECON_TRIG_CON);
		val &= ~HW_TRIG_MASK_DECON;
		val |= HW_TRIG_EN;
		writel(val, decon->main + DECON_TRIG_CON);
	}

	exynos_crtc_handle_event(crtc);
}

static const struct exynos_drm_crtc_ops gs101_decon_crtc_ops = {
	.atomic_enable		= gs101_decon_atomic_enable,
	.atomic_disable		= gs101_decon_atomic_disable,
	.enable_vblank		= gs101_decon_enable_vblank,
	.disable_vblank		= gs101_decon_disable_vblank,
	.mode_valid		= gs101_decon_mode_valid,
	.atomic_check		= gs101_decon_atomic_check,
	.update_plane		= gs101_decon_update_plane,
	.disable_plane		= gs101_decon_disable_plane,
	.atomic_flush		= gs101_decon_atomic_flush,
};

static irqreturn_t gs101_decon_irq(int irq, void *data)
{
	struct gs101_decon *decon = data;
	u32 pend;

	pend = readl(decon->main + DECON_INT_PEND);
	if (!pend)
		return IRQ_NONE;

	/* Write one to clear, including sources nothing handles. */
	writel(pend, decon->main + DECON_INT_PEND);

	if ((pend & INT_PEND_FRAME_DONE) && decon->crtc)
		drm_crtc_handle_vblank(&decon->crtc->base);

	return IRQ_HANDLED;
}

static int gs101_decon_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct gs101_decon *decon = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	int ret;

	decon->config.pixel_formats = gs101_decon_formats;
	decon->config.num_pixel_formats = ARRAY_SIZE(gs101_decon_formats);
	decon->config.type = DRM_PLANE_TYPE_PRIMARY;

	ret = exynos_plane_init(drm, &decon->plane, decon->window,
				&decon->config);
	if (ret)
		return ret;

	decon->crtc = exynos_drm_crtc_create(drm, &decon->plane.base,
					     EXYNOS_DISPLAY_TYPE_LCD,
					     &gs101_decon_crtc_ops, decon);
	if (IS_ERR(decon->crtc)) {
		ret = PTR_ERR(decon->crtc);
		decon->crtc = NULL;
		return ret;
	}

	return 0;
}

static void gs101_decon_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct gs101_decon *decon = dev_get_drvdata(dev);

	decon->crtc = NULL;
}

static const struct component_ops gs101_decon_component_ops = {
	.bind	= gs101_decon_bind,
	.unbind	= gs101_decon_unbind,
};

/* Find the window the bootloader scans out from DPP channel 0. */
static int gs101_decon_find_window(struct gs101_decon *decon)
{
	unsigned int n;
	int window = -ENODEV;

	for (n = 0; n < GS101_DECON_WINDOWS; n++) {
		u32 con = readl(decon->wincon + DECON_CON_WIN(n));
		u32 start = readl(decon->win + WIN_START_POSITION(n));
		u32 end = readl(decon->win + WIN_END_POSITION(n));

		dev_info(decon->dev,
			 "window %u: con %#010x start %#010x end %#010x\n",
			 n, con, start, end);

		if (window < 0 && (con & WIN_EN) && WIN_CHMAP(con) == 0 &&
		    WIN_POS_X(end) >= WIN_POS_X(start) &&
		    WIN_POS_Y(end) >= WIN_POS_Y(start)) {
			window = n;
			decon->width = WIN_POS_X(end) - WIN_POS_X(start) + 1;
			decon->height = WIN_POS_Y(end) - WIN_POS_Y(start) + 1;
		}
	}

	return window;
}

static int gs101_decon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_bulk_data *clks;
	struct device_node *np;
	struct gs101_decon *decon;
	int ret;

	decon = devm_kzalloc(dev, sizeof(*decon), GFP_KERNEL);
	if (!decon)
		return -ENOMEM;

	decon->dev = dev;

	decon->main = devm_platform_ioremap_resource_byname(pdev, "main");
	if (IS_ERR(decon->main))
		return PTR_ERR(decon->main);

	decon->win = devm_platform_ioremap_resource_byname(pdev, "win");
	if (IS_ERR(decon->win))
		return PTR_ERR(decon->win);

	decon->wincon = devm_platform_ioremap_resource_byname(pdev, "wincon");
	if (IS_ERR(decon->wincon))
		return PTR_ERR(decon->wincon);

	decon->irq = platform_get_irq_byname(pdev, "frame-done");
	if (decon->irq < 0)
		return decon->irq;

	np = of_parse_phandle(dev->of_node, "google,dpps", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no DPP channel\n");
	decon->dpp = gs101_dpp_get(dev, np);
	of_node_put(np);
	if (IS_ERR(decon->dpp))
		return dev_err_probe(dev, PTR_ERR(decon->dpp),
				     "DPP channel 0 not available\n");

	/* The registers only answer with the DPU clocks running. */
	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	dev_info(dev, "version %#010x global %#010x trigger %#010x int %#010x\n",
		 readl(decon->main + DECON_VERSION),
		 readl(decon->main + DECON_GLOBAL_CON),
		 readl(decon->main + DECON_TRIG_CON),
		 readl(decon->main + DECON_INT_EN));

	ret = gs101_decon_find_window(decon);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "no window set up by the bootloader\n");
	decon->window = ret;

	dev_info(dev, "taking over window %u, %ux%u\n", decon->window,
		 decon->width, decon->height);

	/*
	 * The bootloader leaves the interrupt master enable off, so nothing
	 * is pending until vblank is enabled.
	 */
	ret = devm_request_irq(dev, decon->irq, gs101_decon_irq, 0,
			       dev_name(dev), decon);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request frame-done\n");

	platform_set_drvdata(pdev, decon);

	return component_add(dev, &gs101_decon_component_ops);
}

static void gs101_decon_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &gs101_decon_component_ops);
}

static const struct of_device_id gs101_decon_of_match[] = {
	{ .compatible = "google,gs101-decon" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs101_decon_of_match);

struct platform_driver gs101_decon_driver = {
	.probe		= gs101_decon_probe,
	.remove		= gs101_decon_remove,
	.driver		= {
		.name		= "gs101-decon",
		.of_match_table	= gs101_decon_of_match,
	},
};
