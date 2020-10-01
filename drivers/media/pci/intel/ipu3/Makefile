# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_VIDEO_IPU3_CIO2) += ipu3-cio2-driver.o

ipu3-cio2-driver-objs += ipu3-cio2.o

ifeq ($(CONFIG_CIO2_BRIDGE), y)
ipu3-cio2-driver-objs += cio2-bridge.o
endif
