/*
 * Copyright 2021 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef EDM_G_IMX8MM_ANDROID_H
#define EDM_G_IMX8MM_ANDROID_H

#define FSL_FASTBOOT_FB_DEV "mmc"

#undef CONFIG_EXTRA_ENV_SETTINGS
#undef CONFIG_BOOTCOMMAND

#define CONFIG_EXTRA_ENV_SETTINGS	\
	"splashpos=m,m\0"\
	"splashimage=0x50000000\0"\
	"fdt_high=0xffffffffffffffff\0"\
	"initrd_high=0xffffffffffffffff\0"\
	"dtoverlay=sn65dsi84-vl10112880 tevi-ov5640\0"\
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
	"androidboot.console=ttymxc1 "\
	"androidboot.hardware=nxp "\
	"androidboot.hwrotation=0 "\
	"androidboot.usb.debugging=1 "\
	"bootconfig "\
	"\0"

/*
	"androidboot.primary_display=imx-drm "\
	"androidboot.vendor.sysrq=1 "\
	"androidboot.lcd_density=240 "\
	"androidboot.wificountrycode=TW "\
	"loglevel=8 "\
	//"quiet"
	//"androidboot.selinux=permissive "
	//"androidboot.camera.layout=${tn_cam}
*/

/* Enable mcu firmware flash */
#ifdef CONFIG_FLASH_MCUFIRMWARE_SUPPORT
#define ANDROID_MCU_FRIMWARE_DEV_TYPE DEV_MMC
#define ANDROID_MCU_FIRMWARE_START 0x500000
#define ANDROID_MCU_OS_PARTITION_SIZE 0x40000
#define ANDROID_MCU_FIRMWARE_SIZE  0x20000
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

#ifdef CONFIG_ID_ATTESTATION
#define ATTESTATION_ID_BRAND "Android"
#define ATTESTATION_ID_DEVICE "edm_g_imx8mm"
#define ATTESTATION_ID_MANUFACTURER "TECHNEXION"
#define ATTESTATION_ID_MODEL "EDM_G_IMX8MM"
#ifdef CONFIG_ATTESTATION_ID_PRODUCT
#undef CONFIG_ATTESTATION_ID_PRODUCT
#endif
#define CONFIG_ATTESTATION_ID_PRODUCT "edm_g_imx8mm"
#endif

/* Enable CONFIG_IMX8M_1G_MEMORY  to config 1GB ddr */
#ifdef CONFIG_IMX8M_1G_MEMORY
#undef  PHYS_SDRAM_SIZE
#define PHYS_SDRAM_SIZE 0x40000000 /* 1GB DDR */
#endif

#ifdef CONFIG_IMX8M_4G_LPDDR4
#undef PHYS_SDRAM_SIZE
#define PHYS_SDRAM_SIZE          0xC0000000 /* 3GB */
#define PHYS_SDRAM_2             0x100000000
#define PHYS_SDRAM_2_SIZE        0x40000000 /* 1GB */
#endif

#endif /* EDM_G_IMX8MM_ANDROID_H */
