#
# Copyright 2018 Technexion Ltd.
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += xore-imx8mm.o

ifdef CONFIG_SPL_BUILD
obj-y += xore-imx8mm_spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_1gb.o lpddr4_timing_2gb.o lpddr4_timing_4gb.o
obj-$(CONFIG_IMX8M_DDR4) += ddr4_timing.o
endif
