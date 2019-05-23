#
# Copyright 2016 Freescale Semiconductor
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += edm-imx8mq.o

ifdef CONFIG_SPL_BUILD
obj-y += edm-imx8mq_spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_1gb.o lpddr4_timing_2gb.o lpddr4_timing_3gb.o lpddr4_timing_4gb.o
endif
