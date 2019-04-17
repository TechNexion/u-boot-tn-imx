#
# Copyright 2016 Freescale Semiconductor
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += pico-imx8mq.o

ifdef CONFIG_SPL_BUILD
obj-y += pico-imx8mq_spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o lpddr4_timing_b0.o
endif
