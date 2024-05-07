# SPDX-License-Identifier: GPL-2.0

# Enables support for device-tree overlays
DTC_FLAGS := -@

dtb-$(CONFIG_ARCH_BCM2835) += bcm2711-rpi-400.dtb \
			      bcm2711-rpi-4-b.dtb \
			      bcm2711-rpi-cm4-io.dtb \
			      bcm2837-rpi-3-a-plus.dtb \
			      bcm2837-rpi-3-b.dtb \
			      bcm2837-rpi-3-b-plus.dtb \
			      bcm2837-rpi-cm3-io3.dtb \
			      bcm2837-rpi-zero-2-w.dtb

bcm2711-rpi-4-b-imx219-dtbs    := bcm2711-rpi-4-b.dtb    bcm2711-rpi-4-b-imx219.dtbo
bcm2711-cm4-imx219-dtbs		:= bcm2711-rpi-cm4-io.dtb bcm2711-cm4-csi0-imx219.dtbo
bcm2711-cm4-imx708-dtbs		:= bcm2711-rpi-cm4-io.dtb bcm2711-cm4-csi0-imx708.dtbo
dtb-$(CONFIG_ARCH_BCM2835) += bcm2711-rpi-4-b-imx219.dtb
dtb-$(CONFIG_ARCH_BCM2835) += bcm2711-rpi-cm4.dtb

dtb-$(CONFIG_ARCH_BCM2835) += bcm2711-rpi-4-b-imx219.dtb
dtb-$(CONFIG_ARCH_BCM2835) += bcm2711-cm4-imx219.dtb
dtb-$(CONFIG_ARCH_BCM2835) += bcm2711-cm4-imx708.dtb

subdir-y	+= bcmbca
subdir-y	+= northstar2
subdir-y	+= stingray
