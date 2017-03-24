
/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6UL_EVK_BRILLO_H
#define __MX6UL_EVK_BRILLO_H

/* For NAND we don't support lock/unlock */
#ifndef CONFIG_SYS_BOOT_NAND
#define CONFIG_FASTBOOT_LOCK
#endif

#define FSL_FASTBOOT_FB_DEV "mmc"

#define CONFIG_FSL_CAAM_KB
#define CONFIG_CMD_FSL_CAAM_KB
#define CONFIG_SHA1
#define CONFIG_SHA256

#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

#define CONFIG_SYS_BOOTM_LEN 0x1000000

#define CONFIG_CMD_READ

#endif
