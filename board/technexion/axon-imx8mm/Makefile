#
# Copyright 2018 Technexion Ltd.
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += axon-imx8mm.o

ifdef CONFIG_SPL_BUILD
obj-y += axon-imx8mm_spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
obj-$(CONFIG_IMX8M_DDR4) += ddr4_timing.o
endif
