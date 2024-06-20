/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2017 NXP Semiconductors
 *
 * Configuration settings for the i.MX7D Pico board.
 */

#ifndef __PICO_IMX7D_CONFIG_H
#define __PICO_IMX7D_CONFIG_H

#include "mx7_common.h"

#ifdef CONFIG_SPL_OS_BOOT
/* Falcon Mode */
#define CONFIG_SPL_FS_LOAD_ARGS_NAME	"args"
#define CONFIG_SPL_FS_LOAD_KERNEL_NAME	"uImage"
#define CONFIG_SYS_SPL_ARGS_ADDR	0x88000000

/* Falcon Mode - MMC support: args@1MB kernel@2MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR       0x800   /* 1MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS      (CONFIG_CMD_SPL_WRITE_SIZE / 512)
#define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR     0x1000  /* 2MB */
#endif

#define CFG_MXC_UART_BASE		UART5_IPS_BASE_ADDR

/* MMC Config */
#define CFG_SYS_FSL_ESDHC_ADDR	USDHC3_BASE_ADDR

#define SYS_MMC_IMG_LOAD_PART	1

#define CFG_EXTRA_ENV_SETTINGS \
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
		"fi\0" \
    "script=boot.scr\0" \
    "image=zImage\0" \
    "stdout=serial\0" \
    "console=ttymxc4\0" \
    "splashimage=0x8c000000\0" \
    "splashpos=m,m\0" \
    "splashsource=mmc_fs\0" \
    "som=imx7d\0" \
    "baseboard=pi\0" \
    "form=pico\0" \
    "wifi_module=qca\0" \
    "default_baseboard=pi\0" \
    "fdt_high=0xffffffff\0" \
    "initrd_high=0xffffffff\0" \
    "fdt_addr=0x83000000\0" \
    "boot_fdt=try\0" \
    "ip_dyn=yes\0" \
    "fbcmd=fastboot 0\0" \
    "mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
    "mmcpart=" __stringify(SYS_MMC_IMG_LOAD_PART) "\0" \
    "searchbootdev=" \
        "if test ${bootdev} = SD0; then " \
            "setenv mmcrootdev /dev/mmcblk2; " \
            "setenv mmcroot /dev/mmcblk2p2 rootwait rw; " \
        "else " \
            "setenv mmcrootdev /dev/mmcblk0; " \
            "setenv mmcroot /dev/mmcblk0p2 rootwait rw; " \
        "fi\0" \
    "mmcautodetect=yes\0" \
    "mmcargs=setenv bootargs console=${console},${baudrate} " \
        "root=${mmcroot} \0" \
    "loadbootscript=" \
        "fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
    "bootscript=echo Running bootscript from mmc ...; " \
        "source\0" \
    "loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
    "setfdt=" \
        "if test -n ${wifi_module} && test ${wifi_module} = qca; then " \
            "setenv fdtfile ${som}-${form}-${baseboard}-${wifi_module}${mcu}.dtb; " \
        "else " \
            "setenv fdtfile ${som}-${form}-${baseboard}${mcu}.dtb;" \
        "fi\0" \
    "loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdtfile}\0" \
    "mmcboot=echo Booting from mmc ...; " \
        "run m4boot; " \
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
#endif


#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x20000000)

//#define CONFIG_SYS_HZ			1000

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CFG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CFG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CFG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CFG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CFG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* PMIC */
#define CONFIG_POWER_PFUZE3000
#define CFG_POWER_PFUZE3000_I2C_ADDR	0x08

/* FLASH and environment organization */

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
//#define CONFIG_BOARD_SIZE_LIMIT               715776
//+CONFIG_HAS_BOARD_SIZE_LIMIT=y
//+CONFIG_BOARD_SIZE_LIMIT=715776

#define CFG_SYS_FSL_USDHC_NUM		2

/* USB Configs */
#define CONFIG_MXC_USB_PORTSC			(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS			0
