/*
 * Copyright (C) 2018 Technexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

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

/* #define CONFIG_BOARD_LATE_INIT -- removed due to redefinition */
/* #define CONFIG_MXC_GPIO -- removed due to redefinition */

#define CONFIG_MXC_UART_BASE		UART1_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

#ifndef CONFIG_SILENT_CONSOLE
#define SILENT_ENABLE			0
#define DEBUG_TTY			ttymxc0
#else
#define SILENT_ENABLE			1
#define DEBUG_TTY			null
#endif


/* Command definition */
#undef CONFIG_CMD_IMLS

/* #define CONFIG_CMD_BMODE -- removed due to redefinition */

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
#define CONFIG_SYS_I2C_SPEED            100000
#define I2C_PMIC_BUS			2

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* MMC Configuration */
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

/* SATA Configuration */
/* #define CONFIG_CMD_SATA -- removed due to redefinition */
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

/* USB Configs */
/* #define CONFIG_USB_EHCI -- unused */
/* #define CONFIG_USB_EHCI_MX6 -- removed due to redefinition */
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

/* Framebuffer */
#define CONFIG_VIDEO_IPUV3
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
/* #define CONFIG_IPUV3_CLK 260000000 -- unused */
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

/* #define CONFIG_CMD_FUSE -- removed due to redefinition */
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"console=" __stringify(DEBUG_TTY)"\0" \
	"splashpos=m,m\0" \
	"som=autodetect\0" \
	"form=edm1\0" \
	"baseboard=fairy\0" \
	"wifi_module=qca\0" \
	"default_baseboard=fairy\0" \
	"fdtfile=undefined\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"silent=" __stringify(SILENT_ENABLE) "\0" \
	"searchbootdev=" \
		"if test ${bootdev} = MMC3; then " \
			"setenv mmcroot /dev/mmcblk2p2 rootwait rw; " \
		"elif test ${bootdev} = SD1; then; " \
			"setenv mmcroot /dev/mmcblk0p2 rootwait rw; " \
		"elif test ${bootdev} = SATA; then; " \
			"setenv mmcroot /dev/sda2 rootwait rw; " \
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
		"fatload ${bootmedia} ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from ${bootmedia} ...; " \
		"source\0" \
	"loadimage=fatload ${bootmedia} ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"setfdt=" \
		"if test ${wifi_module} = qca; then " \
			"setenv fdtfile ${som}-${form}-${wifi_module}_${baseboard}.dtb; " \
		"else " \
			"setenv fdtfile ${som}-${form}_${baseboard}.dtb;" \
		"fi\0" \
	"loadfdt=fatload ${bootmedia} ${mmcdev}:${mmcpart} ${fdt_addr} ${fdtfile}\0" \
	"mmcboot=echo Booting from ${bootmedia} ...; " \
		"run searchbootdev; " \
		"run mmcargs; " \
		"echo baseboard is ${baseboard}; " \
		"echo ${bootargs}; " \
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
	"loadbootenv=fatload ${bootmedia} ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from ${bootmedia} ...; " \
		"env import -t -r $loadaddr $filesize\0" \
	"loadfit=fatload mmc ${mmcdev}:${mmcpart} 0x17880000 tnrescue.itb\0" \
	"fit_args=setenv bootargs console=${console},${baudrate} root=/dev/ram0 rootwait rw " \
		"modules-load=g_acm_ms g_acm_ms.stall=0 g_acm_ms.removable=1 g_acm_ms.file=/dev/mmcblk${mmcdev} " \
		"g_acm_ms.iSerialNumber=${ethaddr} g_acm_ms.iManufacturer=TechNexion; " \
		"run videoargs\0" \
	"fitboot=run fit_args; echo ${bootargs}; bootm 17880000#config@${som}-${form}_${baseboard}\0" \

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
/* #define CONFIG_SYS_LONGHELP -- removed due to redefinition */
/* #define CONFIG_AUTO_COMPLETE -- removed due to redefinition */
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

/* #define CONFIG_CMDLINE_EDITING -- removed due to redefinition */

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

#endif			       /* __CONFIG_H * */
