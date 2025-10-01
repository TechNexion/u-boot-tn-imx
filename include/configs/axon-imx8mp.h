/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 NXP
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 */

#ifndef __AXON_IMX8MP_H
#define __AXON_IMX8MP_H

#include <asm/arch/imx-regs.h>
#include <env/nxp/imx_env.h>

/* ENET Config */
#if defined(CONFIG_CMD_NET)
#define CFG_FEC_MXC_PHYADDR		1
#endif

#define CFG_SYS_INIT_RAM_ADDR		0x40000000
#define CFG_SYS_INIT_RAM_SIZE		0x80000

/* Totally 8GB DDR */
#define CFG_SYS_SDRAM_BASE		0x40000000
#define PHYS_SDRAM			0x40000000
#define PHYS_SDRAM_SIZE			0xC0000000	/* 3 GB */
#define PHYS_SDRAM_2			0x100000000
#define PHYS_SDRAM_2_SIZE		0x140000000	/* 5 GB */

#define CFG_MXC_UART_BASE		UART2_BASE_ADDR

#ifdef CONFIG_ANDROID_SUPPORT
#include "imx8mp_evk_android.h"
#endif

#endif
