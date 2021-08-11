#
# Copyright 2021 TechNexion Ltd.
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y += edm-g-imx8mn.o

ifdef CONFIG_SPL_BUILD
obj-y += edm-g-imx8mn_spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
endif
