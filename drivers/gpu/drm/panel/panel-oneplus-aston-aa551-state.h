/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef AA551_POWER_STATE_H
#define AA551_POWER_STATE_H
/* DCS GET_POWER_MODE: bits 2/3/4 are display-on/normal/sleep-out;
 * bit 5 is partial mode. Factory states 0x9c/0xdc satisfy these bits.
 * Bits 0/1 are reserved and are not an ESD/power-state failure criterion.
 * The panel has returned 0x9d on real retained wake.
 */
static inline int aa551_power_mode_ready(unsigned int mode)
{
	return (mode & 0x3c) == 0x1c;
}
#endif
