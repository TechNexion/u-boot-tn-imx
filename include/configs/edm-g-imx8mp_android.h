/*
 * Copyright 2020 Technexion
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef EDM_G_IMX8MP_ANDROID_H
#define EDM_G_IMX8MP_ANDROID_H

#define CONFIG_ANDROID_AB_SUPPORT
#ifdef CONFIG_ANDROID_AB_SUPPORT
#define CONFIG_SYSTEM_RAMDISK_SUPPORT
#endif
#define FSL_FASTBOOT_FB_DEV "mmc"

#ifdef CONFIG_SYS_MALLOC_LEN
#undef CONFIG_SYS_MALLOC_LEN
#define CONFIG_SYS_MALLOC_LEN           (32 * SZ_1M)
#endif

#undef CONFIG_EXTRA_ENV_SETTINGS
#undef CONFIG_BOOTCOMMAND

#define CONFIG_EXTRA_ENV_SETTINGS		\
	"splashpos=m,m\0"			\
	"splashimage=0x50000000\0"		\
	"fdt_high=0xffffffffffffffff\0"		\
	"initrd_high=0xffffffffffffffff\0"	\
	"overlay_name=cam_ov5640\0"		\
	"bootargs="				\
	"console=ttymxc1,115200 "		\
	"init=/init "				\
	"androidboot.console=ttymxc1 "		\
	"androidboot.hardware=nxp "		\
	"androidboot.hwrotation=0 "		\
	"firmware_class.path=/vendor/firmware "	\
	"loop.max_part=7 "			\
	"transparent_hugepage=never "		\
	"swiotlb=65536 "			\
	"androidboot.vendor.sysrq=1 "		\
	"androidboot.lcd_density=240 "		\
	"androidboot.primary_display=imx-drm "	\
	"androidboot.wificountrycode=TW "	\
	"androidboot.usb.debugging=1 "		\
	"cma=800M@0x400M-0xb80M "		\
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
#define BOOTLOADER_RBIDX_OFFSET  0x3FE000
#define BOOTLOADER_RBIDX_START   0x3FF000
#define BOOTLOADER_RBIDX_LEN     0x08
#define BOOTLOADER_RBIDX_INITVAL 0
#endif

#endif

#ifdef CONFIG_IMX_TRUSTY_OS
#define AVB_RPMB
#define KEYSLOT_HWPARTITION_ID 2
#define KEYSLOT_BLKS             0x1FFF
#define NS_ARCH_ARM64 1
#endif

#define AVB_AB_I_UNDERSTAND_LIBAVB_AB_IS_DEPRECATED

#endif /* EDM_G_IMX8MP_ANDROID_H */
