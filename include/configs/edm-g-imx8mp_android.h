/*
 * Copyright 2021 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef EDM_G_IMX8MP_ANDROID_H
#define EDM_G_IMX8MP_ANDROID_H

#define FSL_FASTBOOT_FB_DEV "mmc"

#undef CONFIG_EXTRA_ENV_SETTINGS
#undef CONFIG_BOOTCOMMAND

#define CONFIG_EXTRA_ENV_SETTINGS	\
	"splashpos=m,m\0"\
	"splashimage=0x50000000\0"\
	"fdt_high=0xffffffffffffffff\0"\
	"initrd_high=0xffffffffffffffff\0"\
	"bootargs="\
	"stack_depot_disable=on "\
	"kasan.stacktrace=off "\
	"console=ttymxc1,115200 "\
	"earlycon=ec_imx6q,0x30890000,115200 "\
	"init=/init "\
	"firmware_class.path=/vendor/firmware "\
	"loop.max_part=7 "\
	"transparent_hugepage=never "\
	"swiotlb=65536 "\
	"pci=nomsi "\
	"cma=800M@0x400M-0x1000M "\
	"buildvariant=userdebug "\
	"androidboot.hwrotation=0 "\
	"androidboot.usb.debugging=1 "\
	"bootconfig "\
	"\0"


/*
	"dtoverlay=vizionlink-tevi-ap1302\0"\

	"pci=nomsi "\
	"cma=800M@0x400M-0x1000M "\
	"buildvariant=userdebug "\
	"androidboot.console=ttymxc1 "\
	"androidboot.hardware=nxp "\
	"androidboot.hwrotation=0 "\
	"androidboot.vendor.sysrq=1 "\
	"androidboot.lcd_density=240 "\
	"androidboot.wificountrycode=TW "\
	"androidboot.usb.debugging=1 "\

	"androidboot.selinux=permissive " \
	"loglevel=8 "\
	"quiet"
 *
 */

/* Enable mcu firmware flash */
#ifdef CONFIG_FLASH_MCUFIRMWARE_SUPPORT
#define ANDROID_MCU_FRIMWARE_DEV_TYPE DEV_MMC
#define ANDROID_MCU_FIRMWARE_START 0x500000
#define ANDROID_MCU_OS_PARTITION_SIZE 0x40000
#define ANDROID_MCU_FIRMWARE_SIZE  0x20000
#define ANDROID_MCU_FIRMWARE_HEADER_STACK 0x20020000
#endif

#define CONFIG_SYS_SPL_PTE_RAM_BASE    0x41580000

#ifdef CONFIG_IMX_TRUSTY_OS
#define BOOTLOADER_RBIDX_OFFSET  0x3FE000
#define BOOTLOADER_RBIDX_START   0x3FF000
#define BOOTLOADER_RBIDX_LEN     0x08
#define BOOTLOADER_RBIDX_INITVAL 0
#endif

#ifdef CONFIG_IMX_TRUSTY_OS
#define AVB_RPMB
#define KEYSLOT_HWPARTITION_ID 2
#define KEYSLOT_BLKS             0x1FFF
#define NS_ARCH_ARM64 1
#endif

#endif
