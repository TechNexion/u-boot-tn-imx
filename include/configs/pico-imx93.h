/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2022 NXP
 */

#ifndef __PICO_IMX93_H
#define __PICO_IMX93_H

#include <asm/arch/imx-regs.h>
#include <env/nxp/imx_env.h>

#define CFG_SYS_UBOOT_BASE	\
	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

/* Link Definitions */

#define CFG_SYS_INIT_RAM_ADDR        0x80000000
#define CFG_SYS_INIT_RAM_SIZE        0x200000

#define CFG_SYS_SDRAM_BASE           0x80000000
#define PHYS_SDRAM                      0x80000000
#define PHYS_SDRAM_SIZE			0x80000000 /* 2GB DDR */

#define CFG_SYS_FSL_USDHC_NUM	2

/* Using ULP WDOG for reset */
#define WDOG_BASE_ADDR          WDG3_BASE_ADDR

#ifdef CONFIG_IMX_MATTER_TRUSTY
#define NS_ARCH_ARM64 1
#endif

#ifdef CONFIG_ANDROID_SUPPORT
#include "imx93_pico_android.h"
#endif

#endif
