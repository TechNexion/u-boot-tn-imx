/*
 * Copyright 2020 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef PICO_IMX8MQ_ANDROID_H
#define PICO_IMX8MQ_ANDROID_H

#define CONFIG_ANDROID_AB_SUPPORT
#ifdef CONFIG_ANDROID_AB_SUPPORT
#define CONFIG_SYSTEM_RAMDISK_SUPPORT
#endif
#define FSL_FASTBOOT_FB_DEV "mmc"

#ifdef CONFIG_SYS_MALLOC_LEN
#undef CONFIG_SYS_MALLOC_LEN
#define CONFIG_SYS_MALLOC_LEN           (64 * SZ_1M)
#endif

#undef CONFIG_EXTRA_ENV_SETTINGS
#undef CONFIG_BOOTCOMMAND

#define CONFIG_EXTRA_ENV_SETTINGS			\
	"splashpos=m,m\0"				\
	"splashimage=0x50000000\0"			\
	"fdt_high=0xffffffffffffffff\0"			\
	"initrd_high=0xffffffffffffffff\0"		\
	"overlay_name=cam_ov5640\0"	\
	"bootargs="					\
	"console=ttymxc0,115200 "			\
	"init=/init "					\
	"androidboot.console=ttymxc0 "			\
	"androidboot.hardware=nxp "			\
	"androidboot.swrotation=0 "			\
	"firmware_class.path=/vendor/firmware "		\
	"loop.max_part=7 "				\
	"androidboot.fbTileSupport=enable "		\
	"transparent_hugepage=never "			\
	"androidboot.vendor.sysrq=1 "			\
	"androidboot.lcd_density=240 "			\
	"androidboot.primary_display=imx-dcss "		\
	"androidboot.displaymode=1080p "		\
	"androidboot.wificountrycode=TW "		\
	"androidboot.usb.debugging=1 "			\
	"cma=736M "					\
	"quiet\0"




/* Enable mcu firmware flash */
#ifdef CONFIG_FLASH_MCUFIRMWARE_SUPPORT
#define ANDROID_MCU_FRIMWARE_DEV_TYPE DEV_MMC
#define ANDROID_MCU_FIRMWARE_START 0x500000
#define ANDROID_MCU_FIRMWARE_SIZE  0x40000
#define ANDROID_MCU_FIRMWARE_HEADER_STACK 0x20020000
#endif

#if !defined(CONFIG_IMX_TRUSTY_OS) || !defined(CONFIG_DUAL_BOOTLOADER)
#undef CONFIG_FSL_CAAM_KB
#endif

#ifdef CONFIG_DUAL_BOOTLOADER
#define CONFIG_SYS_SPL_PTE_RAM_BASE    0x41580000

#ifdef CONFIG_IMX_TRUSTY_OS
#define BOOTLOADER_RBIDX_OFFSET  0x1E000
#define BOOTLOADER_RBIDX_START   0x1F000
#define BOOTLOADER_RBIDX_LEN     0x08
#define BOOTLOADER_RBIDX_INITVAL 0
#endif

#endif

#ifdef CONFIG_IMX_TRUSTY_OS
#define AVB_RPMB
#define KEYSLOT_HWPARTITION_ID 2
#define KEYSLOT_BLKS             0x1FFF
#define NS_ARCH_ARM64 1

#ifdef CONFIG_ID_ATTESTATION
#define ATTESTATION_ID_BRAND "Android"
#define ATTESTATION_ID_DEVICE "pico_imx8mq"
#define ATTESTATION_ID_MANUFACTURER "nxp"
#define ATTESTATION_ID_MODEL "PICO_IMX8MQ"
#ifdef CONFIG_ATTESTATION_ID_PRODUCT
#undef CONFIG_ATTESTATION_ID_PRODUCT
#endif
#define CONFIG_ATTESTATION_ID_PRODUCT "pico_imx8mq"
#endif

#endif

#endif /* IMX8MQ_EVK_ANDROID_H */
