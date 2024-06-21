/*
 * Copyright (C) 2018 TechNexion Ltd.
 *
 * Author: Tapani Utriainen <tapani@technexion.com>
 *         Richard Hu <richard.hu@technexion.com>
 *         Ray Chang <ray.chang@technexion.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __TEP1_IMX7D_CONFIG_H
#define __TEP1_IMX7D_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include <asm/mach-imx/gpio.h>
#include "mx7_common.h"

#ifdef CONFIG_SECURE_BOOT
#ifndef CONFIG_CSF_SIZE
#define CONFIG_CSF_SIZE 0x4000
#endif
#endif

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CFG_MXC_UART_BASE            uart_base_reg_addr
#ifndef __ASSEMBLY__
extern size_t uart_base_reg_addr;
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* MMC Configs */
#define CFG_SYS_FSL_ESDHC_ADDR	USDHC3_BASE_ADDR

/* FEC Configs */
#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_ETHPRIME                 "FEC"

#define CONFIG_FEC_ENET_DEV 0

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_IPS_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x1
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_IPS_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x2
#endif

#define CONFIG_FEC_MXC_MDIO_BASE	IMX_FEC_BASE

/* PMIC */
#define CONFIG_POWER_PFUZE3000
#define CFG_POWER_PFUZE3000_I2C_ADDR	0x08

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

/* Command definition */

#define CONFIG_SYS_AUXCORE_BOOTDATA 0x60000000 /* Set to QSPI1 A flash at default */

#ifdef CONFIG_IMX_BOOTAUX
#define UPDATE_M4_ENV \
	"tcm_addr=0x7F8000\0" \
	"m4image=m4_tcm.bin\0" \
	"loadm4image=fatload mmc ${mmcdev}:${mmcpart} ${tcm_addr} ${m4image}\0" \
	"update_m4_from_sd=" \
		"if sf probe 0:0; then " \
			"if run loadm4image; then " \
				"setexpr fw_sz ${filesize} + 0xffff; " \
				"setexpr fw_sz ${fw_sz} / 0x10000; "	\
				"setexpr fw_sz ${fw_sz} * 0x10000; "	\
				"sf erase 0x0 ${fw_sz}; " \
				"sf write ${loadaddr} 0x0 ${filesize}; " \
			"fi; " \
		"fi\0" \
	"m4boot=" \
		"if run loadm4image; then " \
			"setenv mcu -m4; " \
			"dcache flush; " \
			"bootaux ${tcm_addr}; " \
		"fi\0"
#else
#define UPDATE_M4_ENV ""
#endif

#define SYS_MMC_IMG_LOAD_PART	1
#ifdef CONFIG_SYS_BOOT_NAND
#define MFG_NAND_PARTITION "mtdparts=gpmi-nand:64m(boot),16m(kernel),16m(dtb),-(rootfs) "
#else
#define MFG_NAND_PARTITION ""
#endif

#ifdef CONFIG_VIDEO
#define VIDEO_MODE "panel=MCIMX28LCD\0"
#else
#define VIDEO_MODE ""
#endif

#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"rdinit=/linuxrc " \
		"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
		"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
		"g_mass_storage.iSerialNumber=\"\" "\
		MFG_NAND_PARTITION \
		"clk_ignore_unused "\
		"\0" \
	"initrd_addr=0x83800000\0" \
	"initrd_high=0xffffffff\0" \
	"bootcmd_mfg=run mfgtool_args;bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0" \

#if defined(CONFIG_SYS_BOOT_NAND)
#define CFG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	VIDEO_MODE \
	"fdt_addr=0x83000000\0" \
	"fdt_high=0xffffffff\0"	  \
	"console=ttymxc2\0" \
	"bootargs=console=ttymxc2,115200 ubi.mtd=3 "  \
		"root=ubi0:rootfs rootfstype=ubifs "		     \
		"mtdparts=gpmi-nand:64m(boot),16m(kernel),16m(dtb),-(rootfs)\0"\
	"bootcmd=nand read ${loadaddr} 0x4000000 0x800000;"\
		"nand read ${fdt_addr} 0x5000000 0x100000;"\
		"bootz ${loadaddr} - ${fdt_addr}\0"

#else
#define CFG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	UPDATE_M4_ENV \
	VIDEO_MODE \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"stdout=serial\0" \
	"console=ttymxc1\0" \
	"splashimage=0x8c000000\0" \
	"splashpos=m,m\0" \
	"splashsource=mmc_fs\0" \
	"som=imx7d\0" \
	"form=tep1\0" \
	"baseboard=tep1\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_addr=0x83000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=" __stringify(SYS_MMC_IMG_LOAD_PART) "\0" \
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
		"root=${mmcroot} \0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"setfdt=setenv fdtfile ${som}-${baseboard}.dtb;\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdtfile}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run searchbootdev; " \
		"run m4boot; " \
		"run mmcargs; " \
		"echo ${bootargs}; " \
		"echo baseboard is ${baseboard}; " \
		"run setfdt; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
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
	"bootenv=uEnv.txt\0" \
	"loadbootenv=fatload mmc ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from mmc ...; " \
		"env import -t -r $loadaddr $filesize\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
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
		"echo ${bootargs}; " \
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
	"fit_addr=0x87880000\0" \
	"fit_high=0xffffffff\0" \
	"fit_overlay=for ov in ${dtoverlay}; do " \
			"echo Overlaying ${ov}...; setenv fitov \"${fitov}#${ov}\"; " \
		"done; echo fit conf: ${fdtfile}${fitov};\0" \
	"fitargs=setenv bootargs console=${console},${baudrate} root=/dev/ram0 rootwait rw " \
		"modules-load=g_acm_ms g_acm_ms.stall=0 g_acm_ms.removable=1 g_acm_ms.file=${mmcrootdev} " \
		"g_acm_ms.iSerialNumber=00:00:00:00:00:00 g_acm_ms.iManufacturer=TechNexion\0" \
	"loadfit=fatload mmc ${mmcdev}:${mmcpart} ${fit_addr} tnrescue.itb\0" \
	"fitboot=echo Booting from FIT image...; " \
		"run setfdt; run fit_overlay; run fitargs; " \
		"bootm ${fit_addr}#conf@${fdtfile}${fitov};\0"

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev}; if mmc rescan; then " \
		   "run searchbootdev; " \
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
#endif

/* Miscellaneous configurable options */
#define CONFIG_SYS_PROMPT		"=> "

/* Print Buffer Size */
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x20000000)

#define CONFIG_SYS_HZ			1000

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CFG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CFG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CFG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CFG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CFG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#ifdef CONFIG_SYS_BOOT_QSPI
#define CONFIG_SYS_USE_QSPI
#define CONFIG_ENV_IS_IN_SPI_FLASH
#elif defined CONFIG_SYS_BOOT_NAND
#define CONFIG_SYS_USE_NAND
#define CONFIG_ENV_IS_IN_NAND
#endif

#ifdef CONFIG_SYS_USE_NAND
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS

/* NAND stuff */
#define CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8
#endif

#ifdef CONFIG_SYS_USE_QSPI
#define CONFIG_FSL_QSPI    /* enable the QUADSPI driver */
#define CONFIG_QSPI_BASE		QSPI1_IPS_BASE_ADDR
#define CONFIG_QSPI_MEMMAP_BASE		QSPI0_ARB_BASE_ADDR

#define CONFIG_CMD_SF
#define	CONFIG_SPI_FLASH
#define	CONFIG_SPI_FLASH_MACRONIX
#define	CONFIG_SPI_FLASH_BAR
#define	CONFIG_SF_DEFAULT_BUS		0
#define	CONFIG_SF_DEFAULT_CS		0
#define	CONFIG_SF_DEFAULT_SPEED		40000000
#define	CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#endif

#if defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET		(3 << 18)
#define CONFIG_ENV_SECT_SIZE		(128 << 10)
#define CONFIG_ENV_SIZE			CONFIG_ENV_SECT_SIZE
#endif

#ifdef CONFIG_SYS_USE_NAND
#define CONFIG_SYS_FSL_USDHC_NUM	1
#else
#define CONFIG_SYS_FSL_USDHC_NUM	2
#endif

/* USB Configs */
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0

#if defined(CONFIG_MXC_EPDC) && defined(CONFIG_SYS_USE_QSPI)
#error "EPDC Pins conflicts QSPI, Either EPDC or QSPI can be enabled!"
#endif

#if defined(CONFIG_ANDROID_SUPPORT)
#include "tep1-imx7d_android_common.h"
#endif

#endif	/* __CONFIG_H */
