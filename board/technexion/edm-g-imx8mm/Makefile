#
# Copyright 2020 Technexion Ltd.
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += edm-g-imx8mm.o

ifdef CONFIG_SPL_BUILD
obj-y += edm-g-imx8mm_spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_1gb.o lpddr4_timing_2gb.o lpddr4_timing_4gb.o lpddr4_timing_8gb.o
endif
