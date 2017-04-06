
/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX7ULP_EVK_ANDROID_H
#define __MX7ULP_EVK_ANDROID_H
#include "mx_android_common.h"

#define CONFIG_FLASH_MCUFIRMWARE_SUPPORT
#define CONFIG_CMD_FS_GENERIC
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE

#ifdef CONFIG_FLASH_MCUFIRMWARE_SUPPORT
#define ANDROID_FIRMWARE_START 0
#define ANDROID_FIRMWARE_SIZE  0x20000
#endif
#endif
