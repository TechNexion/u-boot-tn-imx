/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2020 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *         Ray Chang <ray.chang@technexion.com>
 *
 */

#ifndef __PICO_IMX6_CONFIG_H
#define __PICO_IMX6_CONFIG_H

#include "mx6_common.h"

#ifdef CONFIG_SPL_OS_BOOT
/* Falcon Mode */
#define CONFIG_SYS_SPL_ARGS_ADDR   0x18000000

/* Falcon Mode - MMC support: args@1MB kernel@2MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR  0x800   /* 1MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS (CONFIG_CMD_SPL_WRITE_SIZE / 512)
#define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR        0x1000  /* 2MB */
#endif

#define CFG_MXC_UART_BASE		UART1_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)
//#define CONFIG_LOADADDR			0x12000000

/* MMC Configuration */
#define CFG_SYS_FSL_USDHC_NUM	2
#define CFG_SYS_FSL_ESDHC_ADDR	USDHC3_BASE_ADDR

/* USB Configs */
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0

#define DFU_DEFAULT_POLL_TIMEOUT 300

#define CFG_DFU_ENV_SETTINGS \
	"dfu_alt_info=" \
		"spl raw 0x2 0x400;" \
		"u-boot raw 0x8a 0x1000;" \
		"/boot/zImage ext4 0 1;" \
		"rootfs part 0 1\0" \

#define CFG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"console=ttymxc0\0" \
	"splashimage=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
	"splashpos=m,m\0" \
	"splashsource=mmc_fs\0" \
	"som=autodetect\0" \
	"form=pico\0" \
	"baseboard=pi\0" \
	"wifi_module=qca\0" \
	"default_baseboard=pi\0" \
	"fdtfile=undefined\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_addr_r=0x18000000\0" \
	"fdt_addr=0x18000000\0" \
	"ip_dyn=no\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	CFG_DFU_ENV_SETTINGS \
	"mmcpart=1\0" \
	"mmcautodetect=yes\0" \
	"searchbootdev=" \
		"if test ${bootdev} = SD0; then " \
			"setenv mmcrootdev /dev/mmcblk2; " \
			"setenv mmcroot /dev/mmcblk2p2 rootwait rw; " \
		"else " \
			"setenv mmcrootdev /dev/mmcblk0; " \
			"setenv mmcroot /dev/mmcblk0p2 rootwait rw; " \
		"fi\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}; run videoargs\0" \
	"fdtfile_autodetect=on\0" \
	"bootdev_autodetect=on\0" \
	"display_autodetect=on\0" \
	"videoargs=" \
		"if test ${display_autodetect} = off; then " \
			"echo Applying custom display setting...;" \
			"setenv bootargs ${bootargs} ${displayinfo} ${fbmem};" \
		"else " \
			"echo Detecting monitor...;" \
			"setenv nextcon 0; " \
			"i2c dev 1; " \
			"if i2c probe 0x38; then " \
				"setenv bootargs ${bootargs} " \
					"video=mxcfb${nextcon}:dev=lcd,800x480@60," \
						"if=RGB24; " \
				"if test 0 -eq ${nextcon}; then " \
					"setenv fbmem fbmem=10M; " \
				"else " \
					"setenv fbmem ${fbmem},10M; " \
				"fi; " \
				"setexpr nextcon ${nextcon} + 1; " \
			"else " \
				"echo '- no FT5x06 touch display';" \
			"fi; " \
			"if hdmidet; then " \
				"setenv bootargs ${bootargs} " \
					"video=mxcfb${nextcon}:dev=hdmi,1280x720M@60," \
						"if=RGB24; " \
				"setenv fbmem fbmem=28M; " \
				"setexpr nextcon ${nextcon} + 1; " \
			"else " \
				"echo - no HDMI monitor;" \
			"fi; " \
			"setenv bootargs ${bootargs} ${fbmem};" \
		"fi;\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"setfdt=" \
		"if test -n ${wifi_module} && test ${wifi_module} = qca; then " \
			"setenv fdtfile ${som}-${form}-${baseboard}-${wifi_module}.dtb; " \
		"else " \
			"setenv fdtfile ${som}-${form}-${baseboard}.dtb;" \
		"fi\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdtfile}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run searchbootdev; " \
		"run mmcargs; " \
		"echo baseboard is ${baseboard}; " \
		"run setfdt; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"echo WARN: Cannot load the DT; " \
					"echo fall back to load the default DT; " \
					"setenv baseboard ${default_baseboard}; " \
					"run setfdt; " \
					"run loadfdt; " \
					"bootz ${loadaddr} - ${fdt_addr}; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"bootenv=uEnv.txt\0" \
	"loadbootenv=fatload mmc ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from mmc ...; " \
		"env import -t -r $loadaddr $filesize\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
		"ip=${ipaddr} nfsroot=${serverip}:${nfsroot},v3,tcp rw; run videoargs\0" \
	"netboot=echo Booting from net ...; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"run loadbootenv; " \
		"run importbootenv; " \
		"run setfdt; " \
		"run netargs; " \
		"${get_cmd} ${loadaddr} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdtfile}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"fitov=\"\"\0" \
	"fit_addr=0x21100000\0" \
	"fit_high=0xffffffff\0" \
	"fit_overlay=for ov in ${dtoverlay}; do " \
			"echo Overlaying ${ov}...; setenv fitov \"${fitov}#${ov}\"; " \
		"done; echo fit conf: ${fdtfile}${fitov};\0" \
	"fitargs=setenv bootargs console=${console},${baudrate} root=/dev/ram0 rootwait rw " \
		"modules-load=g_acm_ms g_acm_ms.stall=0 g_acm_ms.removable=1 g_acm_ms.file=${mmcrootdev} " \
		"g_acm_ms.iSerialNumber=00:00:00:00:00:00 g_acm_ms.iManufacturer=TechNexion; run videoargs\0" \
	"loadfit=fatload mmc ${mmcdev} ${fit_addr} tnrescue.itb\0" \
	"fitboot=echo Booting from FIT image...; " \
		"run searchbootdev; run setfdt; run fit_overlay; run fitargs; " \
		"bootm ${fit_addr}#conf@${fdtfile}${fitov};\0"

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev}; if mmc rescan; then " \
		   "if run loadbootenv; then " \
			   "echo Loaded environment from ${bootenv};" \
			   "run importbootenv;" \
		   "fi;" \
		   "if test -n $uenvcmd; then " \
			   "echo Running uenvcmd ...;" \
			   "run uenvcmd;" \
		   "fi;" \
		   "if run loadbootscript; then " \
			   "run bootscript; " \
		   "fi; " \
		   "if run loadfit; then " \
			   "run fitboot; " \
		   "fi; " \
		   "if run loadimage; then " \
			   "run mmcboot; " \
		   "else " \
			   "echo WARN: Cannot load kernel from boot media; " \
		   "fi; " \
	   "else run netboot; fi"

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0) \
	func(USB, usb, 0)

#include <config_distro_bootcmd.h>

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CFG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CFG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CFG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CFG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CFG_SYS_INIT_RAM_ADDR + CFG_SYS_INIT_SP_OFFSET)

/* Environment organization */

/* Environment starts at 768k = 768 * 1024 = 786432 */
/*
 * Detect overlap between U-Boot image and environment area in build-time
 *
 * CONFIG_BOARD_SIZE_LIMIT = CONFIG_ENV_OFFSET - u-boot.img offset
 * CONFIG_BOARD_SIZE_LIMIT = 768k - 69k = 699k = 715776
 *
 * Currently CONFIG_BOARD_SIZE_LIMIT does not handle expressions, so
 * write the direct value here
 */
//#define CONFIG_BOARD_SIZE_LIMIT		715776
//+CONFIG_HAS_BOARD_SIZE_LIMIT=y
//+CONFIG_BOARD_SIZE_LIMIT=715776

/* Ethernet Configuration */
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_FEC_MXC_PHYADDR		1

/* Framebuffer */
#define CONFIG_VIDEO_BMP_LOGO

#if defined(CONFIG_ANDROID_SUPPORT)
#include "pico-imx6_android_common.h"
#else
#define CONFIG_USBD_HS
#endif /* CONFIG_ANDROID_SUPPORT */
#endif			       /* __CONFIG_H * */
