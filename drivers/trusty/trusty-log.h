/* SPDX-License-Identifier: MIT */
/*
 * Copyright (c) 2015 Google, Inc.
 *
 * Trusty also has a copy of this header.  Please keep the copies in sync.
 */
#ifndef _TRUSTY_LOG_H_
#define _TRUSTY_LOG_H_

#include <linux/build_bug.h>
#include <linux/stddef.h>
#include <linux/types.h>

/*
 * Ring buffer that supports one secure producer thread and one
 * linux side consumer thread.
 */
struct log_rb {
	u32 alloc;
	u32 put;
	u32 sz;
	char data[];
};

/* The layout Trusty writes: no padding between the fields. */
static_assert(offsetof(struct log_rb, data) == 12);

#define SMC_SC_SHARED_LOG_VERSION	SMC_STDCALL_NR(SMC_ENTITY_LOGGING, 0)
#define SMC_SC_SHARED_LOG_ADD		SMC_STDCALL_NR(SMC_ENTITY_LOGGING, 1)
#define SMC_SC_SHARED_LOG_RM		SMC_STDCALL_NR(SMC_ENTITY_LOGGING, 2)

#define TRUSTY_LOG_API_VERSION	1

#endif

