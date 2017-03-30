/*
 * Copyright (C) 2013-2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef MX6_SABRE_ANDROID_COMMON_H
#define MX6_SABRE_ANDROID_COMMON_H
#include "mx_android_common.h"
/* For NAND we don't support lock/unlock */
#ifndef CONFIG_SYS_BOOT_NAND
#define CONFIG_FASTBOOT_LOCK
#define FSL_FASTBOOT_FB_DEV "mmc"
#define FASTBOOT_ENCRYPT_LOCK
#endif

#define CONFIG_FSL_CAAM_KB
#define CONFIG_CMD_FSL_CAAM_KB
#define CONFIG_SHA1
#define CONFIG_SHA256

#if defined(CONFIG_SYS_BOOT_NAND)
#define ANDROID_FASTBOOT_NAND_PARTS "16m@64m(boot) 16m@80m(recovery) 810m@96m(android_root)ubifs"
#endif

#endif /* MX6_SABRE_ANDROID_COMMON_H */
