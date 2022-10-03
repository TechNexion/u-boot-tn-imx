#
# Copyright 2017 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += pico-imx8mq.o

ifdef CONFIG_SPL_BUILD
obj-y += pico-imx8mq_spl.o
# PICO-IMX8MQ supports 6 types of DDR timing settings:
# For silicon rev.B0: LPDDR4 1/2/4 GB (667 MTS DDR train)
# For silicon rev.B1: LPDDR4 1/2/4 GB (100/400 MTS DDR train)
# The LPDDR timing tweak for 1/4 GB is in 'board/technexion/pico-imx8mq/pico-imx8mq_spl.c'
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_2gb.o lpddr4_timing_2gb_b1.o
endif
