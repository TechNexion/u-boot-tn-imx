/*
 * Copyright (C) 2018 Technexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *         Po Cheng <po.cheng@technexion.com>
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#ifndef __PICO_IMX6_CONFIG_H
#define __PICO_IMX6_CONFIG_H

#include "mx6_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/mach-imx/gpio.h>
#include <linux/sizes.h>

#ifdef CONFIG_SPL
#include "imx6_spl.h"
#endif

#define CONFIG_DISPLAY_BOARDINFO_LATE  /* display board info (after reloc) */

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#undef CONFIG_LDO_BYPASS_CHECK

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

/* #define CONFIG_BOARD_LATE_INIT -- redefined */
/* #define CONFIG_MXC_GPIO -- redefined */

#define CONFIG_MXC_UART_BASE		UART1_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#undef CONFIG_CMD_IMLS

/* #define CONFIG_CMD_BMODE -- redefined */

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)
#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1         /* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2         /* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3         /* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000
#define I2C_PMIC_BUS			1

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* MMC Configuration */
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

/* USB Configs */
/* #define CONFIG_USB_EHCI -- not used */
/* #define CONFIG_USB_EHCI_MX6 -- redefined */
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0

/* Ethernet Configuration */
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS
#define CONFIG_PHY_REALTEK

/* Framebuffer */
#define CONFIG_VIDEO_IPUV3
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_SPLASH_SOURCE
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
/* #define CONFIG_IPUV3_CLK 260000000 -- not used, IPUV3_CLK_MX6Q is 26400000, IPUV3_CLK_MX6DL is 19800000 */
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

/* #define CONFIG_CMD_FUSE -- redefined */
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"console=ttymxc0\0" \
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
	"fdt_addr=0x18000000\0" \
	"ip_dyn=no\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
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

/* Miscellaneous configurable options */
/* #define CONFIG_SYS_LONGHELP -- redefined */
/* #define CONFIG_AUTO_COMPLETE -- redefined */
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

/* #define CONFIG_CMDLINE_EDITING -- redefined */

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#if defined(CONFIG_ANDROID_SUPPORT)
#include "pico-imx6_android_common.h"
#else
#define CONFIG_USBD_HS
#endif /* CONFIG_ANDROID_SUPPORT */

#endif  /* __PICO_IMX6_CONFIG_H */
