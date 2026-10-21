/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef SYNA_TCM_GESTURE_H
#define SYNA_TCM_GESTURE_H
/* Fixed factory prefix: gesture byte, six info bytes, 24 coordinate bytes. */
#define SYNA_TCM_GESTURE_BYTES 31
#define SYNA_TCM_GESTURE_DTAP 0x01
#define SYNA_TCM_GESTURE_FOD_DOWN 0x80
#define SYNA_TCM_GESTURE_FOD_UP 0x81
struct syna_tcm_gesture { unsigned char code; unsigned int x, y; };
static inline int syna_tcm_decode_gesture(const unsigned char *p, unsigned int n,
	unsigned int max_x, unsigned int max_y, struct syna_tcm_gesture *g)
{
	if (n < SYNA_TCM_GESTURE_BYTES)
		return -EINVAL;
	g->code = p[0];
	g->x = p[7] | (unsigned int)p[8] << 8;
	g->y = p[9] | (unsigned int)p[10] << 8;
	if (g->code == SYNA_TCM_GESTURE_FOD_DOWN &&
	    (g->x > max_x || g->y > max_y))
		return -ERANGE;
	return 0;
}
#endif
