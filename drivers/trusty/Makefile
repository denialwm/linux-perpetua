# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for trusty components
#

# Needed for the trace points
ccflags-y += -I$(src)

obj-$(CONFIG_TRUSTY)		+= trusty-core.o
trusty-core-objs		+= trusty.o trusty-mem.o trusty-sched-share.o
trusty-core-$(CONFIG_ARM)	+= trusty-smc-arm.o
trusty-core-$(CONFIG_ARM64)	+= trusty-smc-arm64.o
trusty-core-$(CONFIG_TRUSTY_IRQ)+= trusty-irq.o
