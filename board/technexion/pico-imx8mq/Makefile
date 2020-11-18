#
# Copyright 2017 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += pico-imx8mq.o

ifdef CONFIG_SPL_BUILD
obj-y += pico-imx8mq_spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_4gb.o lpddr4_timing_base_b1.o lpddr4_timing_4gb_b1.o \
								lpddr4_timing_2gb.o lpddr4_timing_2gb_b1.o
endif
