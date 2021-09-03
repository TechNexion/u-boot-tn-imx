#
# Copyright 2021 TechNexion Ltd.
#
# SPDX-License-Identifier:      GPL-2.0+
# Author: Ray Chang <ray.chang@technexion.com>
#

obj-y += flex-imx8mm.o

ifdef CONFIG_SPL_BUILD
obj-y += flex-imx8mm_spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_1gb.o lpddr4_timing_2gb.o lpddr4_timing_3gb.o lpddr4_timing_4gb.o
endif