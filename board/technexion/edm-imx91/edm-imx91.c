// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 NXP
 */

#include <env.h>
#include <efi_loader.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/global_data.h>
#include <asm/arch-imx9/ccm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch-imx9/imx91_pins.h>
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
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE(6) | PAD_CTL_FSEL2)
#define LCDIF_GPIO_PAD_CTRL	(PAD_CTL_DSE(0xf) | PAD_CTL_FSEL2 | PAD_CTL_PUE)

static iomux_v3_cfg_t const uart_pads[] = {
	MX91_PAD_UART1_RXD__LPUART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX91_PAD_UART1_TXD__LPUART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const lcdif_gpio_pads[] = {
	MX91_PAD_GPIO_IO00__GPIO2_IO0| MUX_PAD_CTRL(LCDIF_GPIO_PAD_CTRL),
	MX91_PAD_GPIO_IO01__GPIO2_IO1 | MUX_PAD_CTRL(LCDIF_GPIO_PAD_CTRL),
	MX91_PAD_GPIO_IO02__GPIO2_IO2 | MUX_PAD_CTRL(LCDIF_GPIO_PAD_CTRL),
	MX91_PAD_GPIO_IO03__GPIO2_IO3 | MUX_PAD_CTRL(LCDIF_GPIO_PAD_CTRL),
};

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
#define IMX_BOOT_IMAGE_GUID \
	EFI_GUID(0xbc550d86, 0xda26, 0x4b70, 0xac, 0x05, \
		 0x2a, 0x44, 0x8e, 0xda, 0x6f, 0x21)

struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"IMX91-EDM-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 0=flash-bin raw 0 0x2000 mmcpart 1",
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};

#endif /* EFI_HAVE_CAPSULE_SUPPORT */

struct tn_display const displays[]= {
/*      bus, addr, id_reg, id, detect */
	{ 2, 0x2a, 0,  80,  "vxt-vl0808060nt", detect_exc3000_i2c },
	{ 2, 0x38, 0xa6, 0x01, "vxt-vl050-070-8048nt", detect_i2c },
	{ 2, 0x38, 0xa6, 0x02, "vxt-vl050-070-8048nt", detect_i2c },
};
size_t tn_display_count = ARRAY_SIZE(displays);

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));
	imx_iomux_v3_setup_multiple_pads(lcdif_gpio_pads, ARRAY_SIZE(lcdif_gpio_pads));

	/* Workaround LCD panel leakage, output low of CLK/DE/VSYNC/HSYNC as early as possible */
	struct gpio_regs *gpio2 = (struct gpio_regs *)(GPIO2_BASE_ADDR + 0x40);
	setbits_le32(&gpio2->gpio_pcor, 0xf);
	setbits_le32(&gpio2->gpio_pddr, 0xf);
	/* Set GPIO2_26 to output high to disable panel backlight at default */
	setbits_le32(&gpio2->gpio_psor, BIT(26));
	setbits_le32(&gpio2->gpio_pddr, BIT(26));

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

#define WL_REG_ON "wl_reg_on"
#define BT_REG_ON "bt_reg_on"
#define TCA9554 "gpio@22"

void setup_wifi(void)
{
	int node, ret;
	struct udevice *udev;
	struct gpio_desc *wl_gpio, *bt_gpio;

	ret = uclass_get_device_by_name(UCLASS_GPIO, TCA9554, &udev);
	if (ret != 0) {
		printf("%s: get %s udev failed\n", __func__, TCA9554);
		return;
	}

	/* WL_REG_ON */
	node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(udev), WL_REG_ON);
	if (node < 0) {
		printf("%s: Can't find node name '%s'\n", __func__, WL_REG_ON);
		return;
	}

	ret = gpio_request_by_name_nodev(offset_to_ofnode(node), "gpio",
			 0, wl_gpio, GPIOD_IS_OUT);
	if ( (ret != 0) || (!dm_gpio_is_valid(wl_gpio)) ) {
		printf("%s: request wl_reg_on failed\n", __func__);
		return;
	}
	dm_gpio_set_value(wl_gpio, 0);
	mdelay(50);
	dm_gpio_free(udev, wl_gpio);

	/* BT_REG_ON */
	node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(udev), BT_REG_ON);
	if (node < 0) {
		printf("%s: Can't find node name '%s'\n", __func__, BT_REG_ON);
		return;
	}

	ret = gpio_request_by_name_nodev(offset_to_ofnode(node), "gpio",
			 0, bt_gpio, GPIOD_IS_OUT);
	if ( (ret != 0) || (!dm_gpio_is_valid(bt_gpio)) ) {
		printf("%s: request bt_reg_on failed\n", __func__);
		return;
	}
	dm_gpio_set_value(bt_gpio, 0);
	mdelay(50);
	dm_gpio_free(udev, bt_gpio);
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
	setup_wifi();

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

	strcpy(str_fdtfile, "imx91-edm-");
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
	env_set("board_name", "EDM");
	env_set("board_rev", "iMX91");
#endif

#ifndef CONFIG_AVB_SUPPORT
	detect_baseboard();
	detect_display_panel();
#endif
	return 0;
}

void board_quiesce_devices(void)
{
}
