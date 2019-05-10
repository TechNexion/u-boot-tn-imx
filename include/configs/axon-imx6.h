/*
 * Copyright (C) 2018 Technexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __AXON_IMX6_CONFIG_H
#define __AXON_IMX6_CONFIG_H

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

/*#define CONFIG_BOARD_LATE_INIT */

#define CONFIG_MXC_UART_BASE		UART1_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#undef CONFIG_CMD_IMLS

/* #define CONFIG_CMD_BMODE */

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
/* #define CONFIG_I2C_PMIC			1  Should be done only in Kconfig*/

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* MMC Configuration */
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

/* USB Configs */
/* #define CONFIG_USB_EHCI */
/* #define CONFIG_USB_EHCI_MX6 */
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
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_SPLASH_SOURCE
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
/* #define CONFIG_IPUV3_CLK 260000000 */
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP
#endif

/* #define CONFIG_CMD_FUSE -- redefined */
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
	"image=zImage\0" \
	"console=ttymxc0\0" \
	"splashpos=m,m\0" \
	"splashsource=mmc_fs\0" \
	"splashimage=" __stringify(CONFIG_LOADADDR) "\0" \
	"som=autodetect\0" \
	"baseboard=pi\0" \
	"default_baseboard=pi\0" \
	"fdtfile=undefined\0" \
	"fdt_high=0xffffffff\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"nfsroot=/srv/rootfs\0" \
	"serverip=192.168.0.254\0" \
	"ip_dyn=no\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"mmcroot=/dev/mmcblk2p2\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}; run videoargs\0" \
	"enable_uEnv=yes\0" \
	"enable_som_detection=yes\0" \
	"enable_mmc_detection=yes\0" \
	"videoargs=" \
		"setenv bootargs ${bootargs} ${displayinfo};\0" \
	"loadimage=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"setfdt=setenv fdtfile ${som}_${baseboard}.dtb\0" \
	"loadfdt=load mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdtfile}\0" \
	"mmcboot=echo Booting from mmc ...; " \
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
	"loadbootenv=load mmc ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from mmc ...; " \
		"env import -t -r $loadaddr $filesize\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
	"ip=${get_ip} nfsroot=${serverip}:${nfsroot},v3,tcp rw ${displayinfo} \0" \
		"netboot=echo Booting from net ...; " \
		"if test -n ${ipaddr}; then " \
			"setenv get_cmd tftp; " \
			"setenv get_ip ${ipaddr}; " \
		"else " \
			"setenv get_cmd dhcp; " \
			"setenv get_ip dhcp; " \
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
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev}; if mmc rescan; then " \
		   "if test -n $uenvcmd; then " \
			   "echo Running uenvcmd ...;" \
			   "run uenvcmd;" \
		   "fi;" \
		   "if run loadimage; then " \
			   "run mmcboot; " \
		   "fi; " \
	   "fi"

/* Miscellaneous configurable options */
/* #define CONFIG_SYS_LONGHELP  redefined */
/* #define CONFIG_AUTO_COMPLETE redefined */
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

/* #define CONFIG_CMDLINE_EDITING redefined */

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

#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#define CONFIG_PREBOOT \
	"mmc dev ${mmcdev}; if mmc rescan; then " \
		"if test ${enable_uEnv} = yes; then " \
			"if run loadbootenv; then " \
				"echo Loaded environment from ${bootenv};" \
				"run importbootenv;" \
			"fi;" \
		"fi;" \
	"fi;" \
	"echo baseboard is ${baseboard}; " \
	"run setfdt; " \
	"run mmcargs; " \

#endif			       /* __AXON_IMX6_CONFIG_H */
