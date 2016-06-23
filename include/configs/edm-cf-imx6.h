/*
 * Copyright (C) 2015 Technexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <linux/sizes.h>

#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#define CONFIG_SPL_SATA_SUPPORT
#define CONFIG_SPL_FAT_SUPPORT
#include "imx6_spl.h"

#ifdef CONFIG_CMD_SATA
#include <asm/imx-common/sata.h>
#endif

#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO_LATE  /* display board info (after reloc) */

#define MACH_TYPE_EDM_CF_IMX6		4257
#define CONFIG_MACH_TYPE		MACH_TYPE_EDM_CF_IMX6

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_SYS_GENERIC_BOARD
#undef CONFIG_LDO_BYPASS_CHECK

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE

#ifdef CONFIG_NO_DEBUG_CONSOLE
#define CONFIG_MXC_UART_BASE		UART5_BASE
#define CONFIG_CONS_INDEX		5
#define CONFIG_DEBUG_TTY               ttyUSB0
#else
#define CONFIG_MXC_UART_BASE		UART1_BASE
#define CONFIG_CONS_INDEX		1
#define CONFIG_DEBUG_TTY               ttymxc0
#endif

#define CONFIG_BAUDRATE			115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_SETEXPR

#define CONFIG_BOOTDELAY		1

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)
#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_I2C_PMIC			2

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* MMC Configuration */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

/* SATA Configuration */
#define CONFIG_CMD_SATA

#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0

/* Ethernet Configuration */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

/* Framebuffer */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"console=" __stringify(CONFIG_DEBUG_TTY)"\0" \
	"splashpos=m,m\0" \
	"som=autodetect\0" \
	"baseboard=fairy\0" \
	"default_baseboard=fairy\0" \
	"fdtfile=undefined\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
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
	"setfdt=setenv fdtfile ${som}_${baseboard}.dtb\0" \
	"loadfdt=fatload ${bootmedia} ${mmcdev}:${mmcpart} ${fdt_addr} ${fdtfile}\0" \
	"mmcboot=echo Booting from ${bootmedia} ...; " \
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
	"loadbootenv=fatload ${bootmedia} ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from ${bootmedia} ...; " \
		"env import -t -r $loadaddr $filesize\0" \

#define CONFIG_BOOTCOMMAND \
		   "if test ${bootmedia} = mmc; then " \
			   "mmc dev ${mmcdev}; mmc rescan; " \
		   "fi;" \
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
		   "else " \
			   "if run loadimage; then " \
				   "run mmcboot; " \
			   "else " \
				   "echo WARN: Cannot load kernel from boot media; " \
			   "fi; " \
		   "fi; "

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256
#define CONFIG_SYS_MAXARGS	       16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

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

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_IS_IN_BOOT_DEVICE
#ifdef CONFIG_ENV_IS_IN_BOOT_DEVICE
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_SATA_ENV_DEV		0
#define CONFIG_SYS_DCACHE_OFF // remove when sata driver support cache
#endif

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#endif			       /* __CONFIG_H * */
