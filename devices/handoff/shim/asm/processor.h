/* SPDX-License-Identifier: GPL-2.0-only */
/* Host stand-in for <asm/processor.h>, as arch/arm64/kernel/fdt-handoff.c uses it. */
#include <stdlib.h>
#define cpu_relax() abort()
