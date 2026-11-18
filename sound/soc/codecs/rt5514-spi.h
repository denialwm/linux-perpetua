/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * rt5514-spi.h  --  RT5514 driver
 *
 * Copyright 2015 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 */

#ifndef __RT5514_SPI_H__
#define __RT5514_SPI_H__

/**
 * RT5514_SPI_BUF_LEN is the buffer size of SPI master controller.
*/
#define RT5514_SPI_BUF_LEN		240

#define RT5514_BUFFER_VOICE_BASE	0x18000200
#define RT5514_BUFFER_VOICE_LIMIT	0x18000204
#define RT5514_BUFFER_VOICE_WP		0x1800020c
#define RT5514_IRQ_CTRL			0x18002094

#define RT5514_IRQ_STATUS_BIT		(0x1 << 5)

/* SPI Command */
enum {
	RT5514_SPI_CMD_16_READ = 0,
	RT5514_SPI_CMD_16_WRITE,
	RT5514_SPI_CMD_32_READ,
	RT5514_SPI_CMD_32_WRITE,
	RT5514_SPI_CMD_BURST_READ,
	RT5514_SPI_CMD_BURST_WRITE,
};

int rt5514_spi_burst_read(unsigned int addr, u8 *rxbuf, size_t len);
int rt5514_spi_burst_write(u32 addr, const u8 *txbuf, size_t len);


/* Vendor additions used by the ported RT5514P codec driver. */
#define RT5514_BUFFER_VOICE_BASE_V	0x18002fb4
#define RT5514_BUFFER_VOICE_LIMIT_V	0x18002fb8
#define RT5514_BUFFER_VOICE_WP_V	0x18002fbc
#define RT5514_BUFFER_MUSIC_BASE_V	0x1800103c
#define RT5514_BUFFER_MUSIC_LIMIT_V	0x18001040
#define RT5514_BUFFER_MUSIC_WP_V	0x18001044
#define RT5514_BUFFER_ADC_BASE		0x18002fc0
#define RT5514_BUFFER_ADC_LIMIT		0x18002fc4
#define RT5514_BUFFER_ADC_WP		0x18002fc8
#define RT5514_HOTWORD_FLAG		0x18001034
#define RT5514_MUSDET_FLAG		0x18001038
#define RT5514_DSP_WOV_TYPE		0x18002fac
#define RT5514_DSP_FUNC			0x18002fb0

#define SPI_SWITCH_MASK_WORK_0		(1 << 0)
#define SPI_SWITCH_MASK_WORK_1		(1 << 1)
#define SPI_SWITCH_MASK_LOAD		(1 << 2)
#define SPI_SWITCH_MASK_CMD		(1 << 3)
#define SPI_SWITCH_MASK_RESET		(1 << 4)
#define SPI_SWITCH_MASK_WATCHDOG	(1 << 5)
#define SPI_SWITCH_MASK_CHRE_QMI	(1 << 6)
#define SPI_SWITCH_MASK_NO_IRQ		(1 << 7)
#define SPI_SWITCH_MASK_NO_CHRE		(1 << 8)

void rt5514_spi_request_switch(u32 mask, bool is_require);

#endif /* __RT5514_SPI_H__ */
