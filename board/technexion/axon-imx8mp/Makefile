#
# Copyright 2020 Technexion Ltd.
#
# SPDX-License-Identifier:      GPL-2.0+
# Author: Richard Hu <richard.hu@technexion.com>
#

obj-y += axon-imx8mp.o

ifdef CONFIG_SPL_BUILD
obj-y += axon-imx8mp_spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
endif
