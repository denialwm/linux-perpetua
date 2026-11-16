// SPDX-License-Identifier: GPL-2.0-only
/*
 * rt5514-spi.c  --  RT5514 SPI driver
 *
 * Copyright 2015 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 */

#include <linux/cleanup.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "rt5514-spi.h"
#include "rt5514.h"

#define DRV_NAME "rt5514-spi"

static struct spi_device *rt5514_spi;

struct rt5514_dsp {
	struct device *dev;
	struct delayed_work copy_work;
	struct mutex dma_lock;
	struct snd_pcm_substream *substream;
	unsigned int buf_base, buf_limit, buf_rp;
	size_t buf_size, get_size, dma_offset;
};

static const struct snd_pcm_hardware rt5514_spi_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= 0x20000 / 8,
	.periods_min		= 8,
	.periods_max		= 8,
	.channels_min		= 1,
	.channels_max		= 1,
	.buffer_bytes_max	= 0x20000,
};

static struct snd_soc_dai_driver rt5514_spi_dai = {
	.name = "rt5514-dsp-cpu-dai",
	.id = 0,
	.capture = {
		.stream_name = "DSP Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static void rt5514_spi_copy_work(struct work_struct *work)
{
	struct rt5514_dsp *rt5514_dsp =
		container_of(work, struct rt5514_dsp, copy_work.work);
	struct snd_pcm_runtime *runtime;
	size_t period_bytes, truncated_bytes = 0;
	unsigned int cur_wp, remain_data;
	u8 buf[8];

	guard(mutex)(&rt5514_dsp->dma_lock);
	if (!rt5514_dsp->substream) {
		dev_err(rt5514_dsp->dev, "No pcm substream\n");
		return;
	}

	runtime = rt5514_dsp->substream->runtime;
	period_bytes = snd_pcm_lib_period_bytes(rt5514_dsp->substream);
	if (!period_bytes) {
		schedule_delayed_work(&rt5514_dsp->copy_work, 5);
		return;
	}

	if (rt5514_dsp->buf_size % period_bytes)
		rt5514_dsp->buf_size = (rt5514_dsp->buf_size / period_bytes) *
			period_bytes;

	if (rt5514_dsp->get_size >= rt5514_dsp->buf_size) {
		rt5514_spi_burst_read(RT5514_BUFFER_VOICE_WP, (u8 *)&buf,
			sizeof(buf));
		cur_wp = buf[0] | buf[1] << 8 | buf[2] << 16 |
					buf[3] << 24;

		if (cur_wp >= rt5514_dsp->buf_rp)
			remain_data = (cur_wp - rt5514_dsp->buf_rp);
		else
			remain_data =
				(rt5514_dsp->buf_limit - rt5514_dsp->buf_rp) +
				(cur_wp - rt5514_dsp->buf_base);

		if (remain_data < period_bytes) {
			schedule_delayed_work(&rt5514_dsp->copy_work, 5);
			return;
		}
	}

	if (rt5514_dsp->buf_rp + period_bytes <= rt5514_dsp->buf_limit) {
		rt5514_spi_burst_read(rt5514_dsp->buf_rp,
			runtime->dma_area + rt5514_dsp->dma_offset,
			period_bytes);

		if (rt5514_dsp->buf_rp + period_bytes == rt5514_dsp->buf_limit)
			rt5514_dsp->buf_rp = rt5514_dsp->buf_base;
		else
			rt5514_dsp->buf_rp += period_bytes;
	} else {
		truncated_bytes = rt5514_dsp->buf_limit - rt5514_dsp->buf_rp;
		rt5514_spi_burst_read(rt5514_dsp->buf_rp,
			runtime->dma_area + rt5514_dsp->dma_offset,
			truncated_bytes);

		rt5514_spi_burst_read(rt5514_dsp->buf_base,
			runtime->dma_area + rt5514_dsp->dma_offset +
			truncated_bytes, period_bytes - truncated_bytes);

		rt5514_dsp->buf_rp = rt5514_dsp->buf_base + period_bytes -
			truncated_bytes;
	}

	rt5514_dsp->get_size += period_bytes;
	rt5514_dsp->dma_offset += period_bytes;
	if (rt5514_dsp->dma_offset >= runtime->dma_bytes)
		rt5514_dsp->dma_offset = 0;

	snd_pcm_period_elapsed(rt5514_dsp->substream);

	schedule_delayed_work(&rt5514_dsp->copy_work, 5);
}

static void rt5514_schedule_copy(struct rt5514_dsp *rt5514_dsp)
{
	u8 buf[8];

	if (!rt5514_dsp->substream)
		return;

	rt5514_dsp->get_size = 0;

	/**
	 * The address area x1800XXXX is the register address, and it cannot
	 * support spi burst read perfectly. So we use the spi burst read
	 * individually to make sure the data correctly.
	 */
	rt5514_spi_burst_read(RT5514_BUFFER_VOICE_BASE, (u8 *)&buf,
		sizeof(buf));
	rt5514_dsp->buf_base = buf[0] | buf[1] << 8 | buf[2] << 16 |
				buf[3] << 24;

	rt5514_spi_burst_read(RT5514_BUFFER_VOICE_LIMIT, (u8 *)&buf,
		sizeof(buf));
	rt5514_dsp->buf_limit = buf[0] | buf[1] << 8 | buf[2] << 16 |
				buf[3] << 24;

	rt5514_spi_burst_read(RT5514_BUFFER_VOICE_WP, (u8 *)&buf,
		sizeof(buf));
	rt5514_dsp->buf_rp = buf[0] | buf[1] << 8 | buf[2] << 16 |
				buf[3] << 24;

	if (rt5514_dsp->buf_rp % 8)
		rt5514_dsp->buf_rp = (rt5514_dsp->buf_rp / 8) * 8;

	rt5514_dsp->buf_size = rt5514_dsp->buf_limit - rt5514_dsp->buf_base;

	if (rt5514_dsp->buf_base && rt5514_dsp->buf_limit &&
		rt5514_dsp->buf_rp && rt5514_dsp->buf_size)
		schedule_delayed_work(&rt5514_dsp->copy_work, 0);
}

static irqreturn_t rt5514_spi_irq(int irq, void *data)
{
	struct rt5514_dsp *rt5514_dsp = data;

	rt5514_schedule_copy(rt5514_dsp);

	return IRQ_HANDLED;
}

/* PCM for streaming audio from the DSP buffer */
static int rt5514_spi_pcm_open(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &rt5514_spi_pcm_hardware);

	return 0;
}

static int rt5514_spi_hw_params(struct snd_soc_component *component,
				struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *hw_params)
{
	struct rt5514_dsp *rt5514_dsp =
		snd_soc_component_get_drvdata(component);
	u8 buf[8];

	guard(mutex)(&rt5514_dsp->dma_lock);
	rt5514_dsp->substream = substream;
	rt5514_dsp->dma_offset = 0;

	/* Read IRQ status and schedule copy accordingly. */
	rt5514_spi_burst_read(RT5514_IRQ_CTRL, (u8 *)&buf, sizeof(buf));
	if (buf[0] & RT5514_IRQ_STATUS_BIT)
		rt5514_schedule_copy(rt5514_dsp);

	return 0;
}

static int rt5514_spi_hw_free(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream)
{
	struct rt5514_dsp *rt5514_dsp =
		snd_soc_component_get_drvdata(component);

	scoped_guard(mutex, &rt5514_dsp->dma_lock)
		rt5514_dsp->substream = NULL;

	cancel_delayed_work_sync(&rt5514_dsp->copy_work);

	return 0;
}

static snd_pcm_uframes_t rt5514_spi_pcm_pointer(
		struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct rt5514_dsp *rt5514_dsp =
		snd_soc_component_get_drvdata(component);

	return bytes_to_frames(runtime, rt5514_dsp->dma_offset);
}


static int rt5514_spi_pcm_probe(struct snd_soc_component *component)
{
	struct rt5514_dsp *rt5514_dsp;
	int ret;

	rt5514_dsp = devm_kzalloc(component->dev, sizeof(*rt5514_dsp),
			GFP_KERNEL);
	if (!rt5514_dsp)
		return -ENOMEM;

	rt5514_dsp->dev = &rt5514_spi->dev;
	mutex_init(&rt5514_dsp->dma_lock);
	INIT_DELAYED_WORK(&rt5514_dsp->copy_work, rt5514_spi_copy_work);
	snd_soc_component_set_drvdata(component, rt5514_dsp);

	if (rt5514_spi->irq) {
		ret = devm_request_threaded_irq(&rt5514_spi->dev,
			rt5514_spi->irq, NULL, rt5514_spi_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "rt5514-spi",
			rt5514_dsp);
		if (ret)
			dev_err(&rt5514_spi->dev,
				"%s Failed to request IRQ: %d\n", __func__,
				ret);
		else
			device_init_wakeup(rt5514_dsp->dev, true);
	}

	return 0;
}

static int rt5514_spi_pcm_new(struct snd_soc_component *component,
			      struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC,
				       NULL, 0, 0);
	return 0;
}

static const struct snd_soc_component_driver rt5514_spi_component = {
	.name			= DRV_NAME,
	.probe			= rt5514_spi_pcm_probe,
	.open			= rt5514_spi_pcm_open,
	.hw_params		= rt5514_spi_hw_params,
	.hw_free		= rt5514_spi_hw_free,
	.pointer		= rt5514_spi_pcm_pointer,
	.pcm_new		= rt5514_spi_pcm_new,
	.legacy_dai_naming	= 1,
};

/**
 * rt5514_spi_burst_read - Read data from SPI by rt5514 address.
 * @addr: Start address.
 * @rxbuf: Data Buffer for reading.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5514_spi_burst_read(unsigned int addr, u8 *rxbuf, size_t len)
{
	u8 spi_cmd = RT5514_SPI_CMD_BURST_READ;
	u8 *tx, *rx;
	unsigned int i, j, end, offset = 0, total;
	int status;

	/*
	 * Same framing the verified userspace loader used: command + 32-bit
	 * address + four dummy clocks + receive window, all in one
	 * full-duplex transfer so chip-select stays asserted. The reply is
	 * the tail of the RX window, with each 8-byte word reversed.
	 */
	tx = kzalloc(9 + RT5514_SPI_BUF_LEN, GFP_KERNEL | GFP_DMA);
	rx = kzalloc(9 + RT5514_SPI_BUF_LEN, GFP_KERNEL | GFP_DMA);
	if (!tx || !rx) {
		kfree(tx);
		kfree(rx);
		return false;
	}

	while (offset < len) {
		struct spi_transfer x = { 0 };

		end = min_t(size_t, RT5514_SPI_BUF_LEN, len - offset);
		total = (end + 7) & ~7;

		memset(tx, 0, 9 + RT5514_SPI_BUF_LEN);
		memset(rx, 0, 9 + RT5514_SPI_BUF_LEN);
		tx[0] = spi_cmd;
		tx[1] = ((addr + offset) >> 24) & 0xff;
		tx[2] = ((addr + offset) >> 16) & 0xff;
		tx[3] = ((addr + offset) >> 8) & 0xff;
		tx[4] = (addr + offset) & 0xff;
		tx[5] = tx[0];
		tx[6] = tx[1];
		tx[7] = tx[2];
		tx[8] = tx[3];

		x.tx_buf = tx;
		x.rx_buf = rx;
		x.len = 9 + total;

		status = spi_sync_transfer(rt5514_spi, &x, 1);
		if (status) {
			kfree(tx);
			kfree(rx);
			return false;
		}

		for (i = 0; i < total; i += 8) {
			for (j = 0; j < 8 && (offset + i + j) < len; j++)
				rxbuf[offset + i + j] = rx[9 + i + 7 - j];
		}

		offset += RT5514_SPI_BUF_LEN;
	}

	kfree(tx);
	kfree(rx);

	return true;
}
EXPORT_SYMBOL_GPL(rt5514_spi_burst_read);

/**
 * rt5514_spi_burst_write - Write data to SPI by rt5514 address.
 * @addr: Start address.
 * @txbuf: Data Buffer for writng.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5514_spi_burst_write(u32 addr, const u8 *txbuf, size_t len)
{
	u8 spi_cmd = RT5514_SPI_CMD_BURST_WRITE;
	u8 *buf;
	unsigned int i, j, end, offset = 0;
	int ret;

	/*
	 * One contiguous buffer per chunk, matching the userspace transfer
	 * that read back byte-for-byte at 1 MHz: command + 32-bit address +
	 * 8-byte-word-reversed data + command, 240 bytes per chunk.
	 */
	buf = kzalloc(RT5514_SPI_BUF_LEN + 6, GFP_KERNEL | GFP_DMA);
	if (!buf)
		return -ENOMEM;

	while (offset < len) {
		end = min_t(size_t, RT5514_SPI_BUF_LEN, len - offset);
		memset(buf, 0, RT5514_SPI_BUF_LEN + 6);

		buf[0] = spi_cmd;
		buf[1] = ((addr + offset) >> 24) & 0xff;
		buf[2] = ((addr + offset) >> 16) & 0xff;
		buf[3] = ((addr + offset) >> 8) & 0xff;
		buf[4] = (addr + offset) & 0xff;

		for (i = 0; i < end; i += 8) {
			for (j = 0; j < 8 && (i + j) < end; j++)
				buf[5 + i + 7 - j] = txbuf[offset + i + j];
		}

		end = (end + 7) & ~7;
		buf[5 + end] = spi_cmd;

		ret = spi_write(rt5514_spi, buf, end + 6);
		if (ret) {
			kfree(buf);
			return ret;
		}

		offset += RT5514_SPI_BUF_LEN;
	}

	kfree(buf);

	return 0;
}
EXPORT_SYMBOL_GPL(rt5514_spi_burst_write);

static int rt5514_spi_probe(struct spi_device *spi)
{
	int ret;

	rt5514_spi = spi;

	ret = devm_snd_soc_register_component(&spi->dev,
					      &rt5514_spi_component,
					      &rt5514_spi_dai, 1);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register component.\n");
		return ret;
	}

	return 0;
}

static int rt5514_suspend(struct device *dev)
{
	int irq = to_spi_device(dev)->irq;

	if (device_may_wakeup(dev))
		enable_irq_wake(irq);

	return 0;
}

static int rt5514_resume(struct device *dev)
{
	struct rt5514_dsp *rt5514_dsp = dev_get_drvdata(dev);
	int irq = to_spi_device(dev)->irq;
	u8 buf[8];

	if (device_may_wakeup(dev))
		disable_irq_wake(irq);

	if (rt5514_dsp) {
		if (rt5514_dsp->substream) {
			rt5514_spi_burst_read(RT5514_IRQ_CTRL, (u8 *)&buf,
				sizeof(buf));
			if (buf[0] & RT5514_IRQ_STATUS_BIT)
				rt5514_schedule_copy(rt5514_dsp);
		}
	}

	return 0;
}

static const struct dev_pm_ops rt5514_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(rt5514_suspend, rt5514_resume)
};

static const struct of_device_id rt5514_of_match[] = {
	{ .compatible = "realtek,rt5514", },
	{},
};
MODULE_DEVICE_TABLE(of, rt5514_of_match);

static struct spi_driver rt5514_spi_driver = {
	.driver = {
		.name = "rt5514",
		.pm = pm_ptr(&rt5514_pm_ops),
		.of_match_table = of_match_ptr(rt5514_of_match),
	},
	.probe = rt5514_spi_probe,
};
module_spi_driver(rt5514_spi_driver);

MODULE_DESCRIPTION("RT5514 SPI driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");


static u32 rt5514_spi_switch_mask;

/*
 * Vendor-compatible arbitration: the codec's shared pins belong to the host
 * while any SPI user holds a mask, and to the DSP otherwise.
 */
void rt5514_spi_request_switch(u32 mask, bool is_require)
{
	if (is_require)
		rt5514_spi_switch_mask |= mask;
	else
		rt5514_spi_switch_mask &= ~mask;

	rt5514_spi_bus_switch(rt5514_spi_switch_mask != 0);
}
EXPORT_SYMBOL_GPL(rt5514_spi_request_switch);
