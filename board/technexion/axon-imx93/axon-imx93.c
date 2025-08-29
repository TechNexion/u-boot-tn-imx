// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#include <common.h>
#include <env.h>
#include <efi_loader.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/global_data.h>
#include <asm/arch-imx9/ccm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch-imx9/imx93_pins.h>
#include <asm/arch/clock.h>
#include <power/pmic.h>
#include <dm/device.h>
#include <dm/uclass.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <asm/gpio.h>
#include <mmc.h>
#include "../common/periph_detect.h"
#include <splash.h>
#include <imx_sip.h>
#include <linux/arm-smccc.h>
#include <cli.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE(6) | PAD_CTL_FSEL2)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE(6) | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	MX93_PAD_UART1_RXD__LPUART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX93_PAD_UART1_TXD__LPUART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
#define IMX_BOOT_IMAGE_GUID \
	EFI_GUID(0xbc550d86, 0xda26, 0x4b70, 0xac, 0x05, \
		 0x2a, 0x44, 0x8e, 0xda, 0x6f, 0x21)

struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"AXON_IMX93-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 0=flash-bin raw 0 0x2000 mmcpart 1",
	.images = fw_images,
};

u8 num_image_type_guids = ARRAY_SIZE(fw_images);
#endif /* EFI_HAVE_CAPSULE_SUPPORT */

struct tn_display const displays[]= {
/*      bus, addr, id_reg, id, detect */
	{ 2, 0x2a, 0, 101,  "lvds-vl10112880", detect_exc3000_i2c },
	{ 2, 0x2a, 0, 156,  "lvds-vl15613676", detect_exc3000_i2c },
	{ 2, 0x2a, 0,  80,  "vxt-vl0808060nt", detect_exc3000_i2c },
	{ 2, 0x38, 0xa6, 0x01, "vxt-vl050-070-8048nt", detect_i2c },
	{ 2, 0x38, 0xa6, 0x02, "vxt-vl050-070-8048nt", detect_i2c },
};
size_t tn_display_count = ARRAY_SIZE(displays);

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(LPUART1_CLK_ROOT);

	return 0;
}

static int setup_fec(void)
{
	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

static int setup_eqos(void)
{
	return 0;
}

#ifdef CONFIG_SPLASH_SCREEN
static struct splash_location imx_splash_locations[] = {
	{
		.name = "sf",
		.storage = SPLASH_STORAGE_SF,
		.flags = SPLASH_STORAGE_RAW,
		.offset = 0x100000,
	},
	{
		.name = "mmc_fs",
		.storage = SPLASH_STORAGE_MMC,
		.flags = SPLASH_STORAGE_FS,
		.devpart = "0:1",
	},
	{
		.name = "usb_fs",
		.storage = SPLASH_STORAGE_USB,
		.flags = SPLASH_STORAGE_FS,
		.devpart = "0:1",
	},
	{
		.name = "sata_fs",
		.storage = SPLASH_STORAGE_SATA,
		.flags = SPLASH_STORAGE_FS,
		.devpart = "0:1",
	},
};

/*This function is defined in common/splash.c.
  Declare here to remove warning. */
int splash_video_logo_load(void);

int splash_screen_prepare(void)
{
	imx_splash_locations[1].devpart[0] = mmc_get_env_dev() + '0';
	int ret;
	ret = splash_source_load(imx_splash_locations, ARRAY_SIZE(imx_splash_locations));
	if (!ret)
		return 0;
	else {
		printf("\nNo splash.bmp in boot partition!!\n");
		printf("Using default logo!!\n\n");
		return splash_video_logo_load();
	}
}
#endif /* CONFIG_SPLASH_SCREEN */

static int check_mmc_autodetect(void)
{
	char *autodetect_str = env_get("mmcautodetect");

	if (autodetect_str && !strcmp(autodetect_str, "yes"))
		return 1;

	return 0;
}

/* This should be defined for each board */
__weak int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

void board_late_mmc_env_init(void)
{
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_dev();

	if (!check_mmc_autodetect())
		return;

	env_set_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw", mmc_map_to_kernel_blk(dev_no));
	env_set("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}

#define EXC3000_I2C_ADDR 0x2A
#define TOUCH_I2C_BUS 2
void board_modify_fdt(void)
{
	struct udevice *bus = NULL;
	struct udevice *i2c_dev = NULL;
	int nodeoff;
	int ret;
	uint32_t new_value;
	void *fdt = (void *)gd->fdt_blob;

	if (uclass_get_device_by_seq(UCLASS_I2C, TOUCH_I2C_BUS, &bus)) {
		printf("%s: Can't find bus\n", __func__);
		return;
	}

	dm_i2c_probe(bus, EXC3000_I2C_ADDR, 0, &i2c_dev);

	// change splash screen resolution to 8-inch
	if (i2c_dev) {
		printf("%s: detect exc3000 panel, change resolution to 8-inch\n", __func__);
		// Find the node /panel/display-timings/timing0
		nodeoff = fdt_path_offset(fdt, "/panel/display-timings/timing0");
		if (nodeoff < 0) {
			printf("Node not found: %s\n", fdt_strerror(nodeoff));
			return;
		}

		new_value = cpu_to_fdt32(0x258);
		ret = fdt_setprop(fdt, nodeoff, "vactive", &new_value, sizeof(new_value));
		if (ret) {
			printf("Failed to set property 'vactive': %s\n", fdt_strerror(ret));
			return;
		}

		new_value = cpu_to_fdt32(0x3);
		ret = fdt_setprop(fdt, nodeoff, "vfront-porch", &new_value, sizeof(new_value));
		if (ret) {
			printf("Failed to set property 'vfront-porch': %s\n", fdt_strerror(ret));
			return;
		}

		new_value = cpu_to_fdt32(0x2625A00);
		ret = fdt_setprop(fdt, nodeoff, "clock-frequency", &new_value, sizeof(new_value));
		if (ret) {
			printf("Failed to set property 'clock-frequency': %s\n", fdt_strerror(ret));
			return;
		}
	}
}

int board_init(void)
{
	board_modify_fdt();

	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	if (IS_ENABLED(CONFIG_DWC_ETH_QOS))
		setup_eqos();

	return 0;
}

int detect_baseboard(void)
{
	char *baseboard, str_fdtfile[64];


	env_set("baseboard", "wb");
	baseboard = env_get("baseboard");

	strcpy(str_fdtfile, "imx93-axon-");
	strcat(str_fdtfile, baseboard);
	strcat(str_fdtfile, ".dtb");
	env_set("fdtfile", str_fdtfile);
	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "AXON");
	env_set("board_rev", "iMX93");
#endif

#ifndef CONFIG_AVB_SUPPORT
	detect_baseboard();
	detect_display_panel();
#endif
	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0;
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
