#
# Copyright 2024 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += edm-imx91.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_IMX91_EDM_LPDDR4) += lpddr4_timing_2400mts.o lpddr4_timing_1600mts.o
endif
