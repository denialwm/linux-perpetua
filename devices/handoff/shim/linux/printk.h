/* SPDX-License-Identifier: GPL-2.0-only */
/* Host stand-in for <linux/printk.h>, as arch/arm64/kernel/fdt-handoff.c uses it. */
#include <stdio.h>
#define pr_emerg(...) fprintf(stderr, __VA_ARGS__)
