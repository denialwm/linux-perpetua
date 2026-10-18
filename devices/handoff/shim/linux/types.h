/* SPDX-License-Identifier: GPL-2.0-only */
/* Host stand-in for <linux/types.h>, as arch/arm64/kernel/fdt-handoff.c uses it. */
#include <stdbool.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint64_t u64;
typedef uint64_t phys_addr_t;
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define __aligned(n) __attribute__((aligned(n)))
