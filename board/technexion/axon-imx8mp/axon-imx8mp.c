// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 */

#include <command.h>
#include <env.h>
#include <init.h>
#include <linux/delay.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mp_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <splash.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <mmc.h>
#include <asm/armv8/mmu.h>
#include "axon-imx8mp-ddr.h"
#include "../common/periph_detect.h"
#include <dm/uclass.h>
#include <dm/device.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	MX8MP_PAD_UART2_RXD__UART2_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX8MP_PAD_UART2_TXD__UART2_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	MX8MP_PAD_GPIO1_IO02__WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};


#ifndef CONFIG_SPL_BUILD
#ifdef CONFIG_TN_PHERIPHERAL_DETECT
const tn_camera_chk_t tn_camera_chk[] = {
	{ 1, 0x3c, "tevi-ov5640" },
	{ 4, 0x3c, "tevi-ov5640" },
	{ 1, 0x3d, "tevi-ap1302" },
	{ 4, 0x3d, "tevi-ap1302" },
	{ 1, 0x48, "tevs" },
	{ 4, 0x48, "tevs" },
};
size_t tn_camera_chk_cnt = ARRAY_SIZE(tn_camera_chk);

struct tn_display const displays[]= {
/*      bus, addr, id_reg, id, detect */
	{ 3, 0x2a, 0,    101,  "lvds-vl10112880", detect_exc3000_i2c },
	{ 4, 0x38, 0xA3, 0x54, "ili9881c", detect_i2c },
	{ 4, 0x38, 0xA3, 0x59, "g101uan02", detect_i2c },
	{ 4, 0x3d, 0x98, 0x03, "mipi2hdmi-adv7535", detect_i2c },
	{ 1, 0x2a, 1,     101, "vizionpanel-vl10112880", detect_vizionpanel_i2c },
	{ 1, 0x2a, 1,     150, "vizionpanel-vl15010276", detect_vizionpanel_i2c },
	{ 1, 0x2a, 1,     156, "vizionpanel-vl15613676", detect_vizionpanel_i2c },
	{ 1, 0x2a, 4,      80, "vizionpanel-vl808060",   detect_vizionpanel_i2c },
	{ 1, 0x38, 0xa6, 0x02, "vizionpanel-vl708048",   detect_vizionpanel_i2c },
	{ 1, 0x38, 0xa6, 0x01, "vizionpanel-vl508048",   detect_vizionpanel_i2c },
};
size_t tn_display_count = ARRAY_SIZE(displays);
#endif

static u8 ddr_code __section("data");

static u8 board_get_ddr_code(void)
{
	return (readl(OCRAM_BASE_ADDR));
}

int board_phys_sdram_size(phys_size_t *size)
{
	if (!size)
		return -EINVAL;

	switch (board_get_ddr_code()) {
	case LPDDR4_8GB:
		*size = SZ_8G;
		break;
	case LPDDR4_6GB:
		*size = SZ_6G;
		break;
	case LPDDR4_4GB:
		*size = SZ_4G;
		break;
	case LPDDR4_2GB:
		*size = SZ_2G;
		break;
	case LPDDR4_1GB:
		*size = SZ_1G;
		break;
	default:
		puts("Unknown DDR type!!!\n");
	}

	return 0;
}
#endif

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	const int *cell;
	int offs;
	uint32_t cma_size;
	char *cmasize;
#ifdef CONFIG_IMX8M_DRAM_INLINE_ECC
	int rc;
	phys_addr_t ecc0_start = 0xb0000000;
	phys_addr_t ecc1_start = 0x130000000;
	phys_addr_t ecc2_start = 0x1b0000000;
	size_t ecc_size = 0x10000000;

	rc = add_res_mem_dt_node(blob, "ecc", ecc0_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc0 reserved-memory node.\n");
		return rc;
	}

	rc = add_res_mem_dt_node(blob, "ecc", ecc1_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc1 reserved-memory node.\n");
		return rc;
	}

	rc = add_res_mem_dt_node(blob, "ecc", ecc2_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc2 reserved-memory node.\n");
		return rc;
	}
#endif

	offs = fdt_path_offset(blob, "/reserved-memory/linux,cma");
	cell = fdt_getprop(blob, offs, "size", NULL);
	cma_size = fdt32_to_cpu(cell[1]);
	cmasize = env_get("cma_size");
	if(cmasize || ((u64)(gd->ram_size >> 2) < cma_size)) {
		cma_size = env_get_ulong("cma_size", 10, 256 * 1024 * 1024);
		cma_size = max((u64)(gd->ram_size >> 2), (u64)cma_size);
		fdt_setprop_u64(blob, offs, "size", (uint64_t)cma_size);
	}

	return 0;
}
#endif

#ifdef CONFIG_USB_DWC3

#define USB_PHY_CTRL0			0xF0040
#define USB_PHY_CTRL0_REF_SSP_EN	BIT(2)

#define USB_PHY_CTRL1			0xF0044
#define USB_PHY_CTRL1_RESET		BIT(0)
#define USB_PHY_CTRL1_COMMONONN		BIT(1)
#define USB_PHY_CTRL1_ATERESET		BIT(3)
#define USB_PHY_CTRL1_VDATSRCENB0	BIT(19)
#define USB_PHY_CTRL1_VDATDETENB0	BIT(20)

#define USB_PHY_CTRL2			0xF0048
#define USB_PHY_CTRL2_TXENABLEN0	BIT(8)

#define USB_PHY_CTRL6			0xF0058

#define HSIO_GPR_BASE                               (0x32F10000U)
#define HSIO_GPR_REG_0                              (HSIO_GPR_BASE)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT    (1)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN          (0x1U << HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT)


static struct dwc3_device dwc3_device_data = {
#ifdef CONFIG_SPL_BUILD
	.maximum_speed = USB_SPEED_HIGH,
#else
	.maximum_speed = USB_SPEED_SUPER,
#endif
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
	.power_down_scale = 2,
};

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 RegData;

	/* enable usb clock via hsio gpr */
	RegData = readl(HSIO_GPR_REG_0);
	RegData |= HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN;
	writel(RegData, HSIO_GPR_REG_0);

	/* USB3.0 PHY signal fsel for 100M ref */
	RegData = readl(dwc3->base + USB_PHY_CTRL0);
	RegData = (RegData & 0xfffff81f) | (0x2a<<5);
	writel(RegData, dwc3->base + USB_PHY_CTRL0);

	RegData = readl(dwc3->base + USB_PHY_CTRL6);
	RegData &=~0x1;
	writel(RegData, dwc3->base + USB_PHY_CTRL6);

	RegData = readl(dwc3->base + USB_PHY_CTRL1);
	RegData &= ~(USB_PHY_CTRL1_VDATSRCENB0 | USB_PHY_CTRL1_VDATDETENB0 |
			USB_PHY_CTRL1_COMMONONN);
	RegData |= USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET;
	writel(RegData, dwc3->base + USB_PHY_CTRL1);

	RegData = readl(dwc3->base + USB_PHY_CTRL0);
	RegData |= USB_PHY_CTRL0_REF_SSP_EN;
	writel(RegData, dwc3->base + USB_PHY_CTRL0);

	RegData = readl(dwc3->base + USB_PHY_CTRL2);
	RegData |= USB_PHY_CTRL2_TXENABLEN0;
	writel(RegData, dwc3->base + USB_PHY_CTRL2);

	RegData = readl(dwc3->base + USB_PHY_CTRL1);
	RegData &= ~(USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET);
	writel(RegData, dwc3->base + USB_PHY_CTRL1);
}
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
int board_usb_init(int index, enum usb_init_type init)
{

	if (index == 0 && init == USB_INIT_DEVICE) {
		imx8m_usb_power(index, true);
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_uboot_exit(index);
		imx8m_usb_power(index, false);
	}

	return 0;
}

#define USB_HUB_RST_PAD IMX_GPIO_NR(4, 22)
static iomux_v3_cfg_t const usb_hub_rst_pads[] = {
	MX8MP_PAD_SAI2_RXC__GPIO4_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void setup_usb_rst(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_hub_rst_pads, ARRAY_SIZE(usb_hub_rst_pads));

	gpio_request(USB_HUB_RST_PAD, "usb_hub_rst");
	gpio_direction_output(USB_HUB_RST_PAD, 0);
	gpio_set_value(USB_HUB_RST_PAD, 0);
	mdelay(20);
	gpio_set_value(USB_HUB_RST_PAD, 1);
}
#endif

#define WL_REG_ON_PAD IMX_GPIO_NR(1, 0)
static iomux_v3_cfg_t const wl_reg_on_pads[] = {
	MX8MP_PAD_GPIO1_IO00__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define BT_ON_PAD IMX_GPIO_NR(1, 5)
static iomux_v3_cfg_t const bt_on_pads[] = {
	MX8MP_PAD_GPIO1_IO05__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void setup_wifi(void)
{
	imx_iomux_v3_setup_multiple_pads(wl_reg_on_pads, ARRAY_SIZE(wl_reg_on_pads));
	imx_iomux_v3_setup_multiple_pads(bt_on_pads, ARRAY_SIZE(bt_on_pads));

	gpio_request(WL_REG_ON_PAD, "wl_reg_on");
	gpio_direction_output(WL_REG_ON_PAD, 0);
	gpio_set_value(WL_REG_ON_PAD, 0);

	gpio_request(BT_ON_PAD, "bt_on");
	gpio_direction_output(BT_ON_PAD, 0);
	gpio_set_value(BT_ON_PAD, 0);
}


static iomux_v3_cfg_t const touch_rst_pads[] = {
	MX8MP_PAD_SAI1_TXD1__GPIO4_IO13 | MUX_PAD_CTRL(PAD_CTL_PUE),
};

void setup_touch(void)
{
	imx_iomux_v3_setup_multiple_pads(touch_rst_pads, ARRAY_SIZE(touch_rst_pads));
}

#define CSI1_GPIO_RST IMX_GPIO_NR(1, 8)
#define CSI2_GPIO_RST IMX_GPIO_NR(4, 4)

void setup_camera(void)
{
	gpio_request(CSI1_GPIO_RST, "csi1_rst");
	gpio_direction_output(CSI1_GPIO_RST, 0);
	mdelay(100);
	gpio_direction_output(CSI1_GPIO_RST, 1);

	gpio_request(CSI2_GPIO_RST, "csi2_rst");
	gpio_direction_output(CSI2_GPIO_RST, 0);
	mdelay(100);
	gpio_direction_output(CSI2_GPIO_RST, 1);
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

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
int board_init(void)
{

	setup_wifi();
	setup_touch();
	setup_camera();

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
	setup_usb_rst();
	init_usb_clk();
#endif

	return 0;
}

static int check_mmc_autodetect(void)
{
	char *autodetect_str = env_get("mmcautodetect");

	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

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
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
		mmc_map_to_kernel_blk(dev_no));
	env_set("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}

#define DSI1_RST_NODE "vizionpanel"
#define PCA95554BS "pca9554bs@23"
void reset_dsi(void)
{
	int node, ret;
	struct udevice *udev;
	struct gpio_desc *dsi1_rst_gpio;

	ret = uclass_get_device_by_name(UCLASS_GPIO, PCA95554BS, &udev);
	if (ret != 0) {
		printf("%s: get %s udev failed\n", __func__, PCA95554BS);
		return;
	}

	node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(udev), DSI1_RST_NODE);
	if (node < 0) {
		printf("%s: Can't find node name '%s'\n", __func__, DSI1_RST_NODE);
		return;
	}

	ret = gpio_request_by_name_nodev(offset_to_ofnode(node), "reset-gpios",
			 0, dsi1_rst_gpio, GPIOD_IS_OUT);
	if ( (ret != 0) || (!dm_gpio_is_valid(dsi1_rst_gpio)) ) {
		printf("%s: request reset-gpios failed\n", __func__);
		return;
	}

	dm_gpio_set_value(dsi1_rst_gpio, 1);
	dm_gpio_set_value(dsi1_rst_gpio, 0);
	mdelay(50);
	dm_gpio_set_value(dsi1_rst_gpio, 1);

	dm_gpio_free(udev, dsi1_rst_gpio);
}

int detect_baseboard(void)
{
	char *fdtfile, *baseboard, str_fdtfile[64];

	fdtfile = env_get("fdtfile");
	if (fdtfile && !strcmp(fdtfile, "undefined")) {
		env_set("baseboard", "wizard");
		baseboard = env_get("baseboard");

		strcpy(str_fdtfile, "imx8mp-axon-");
		strcat(str_fdtfile, baseboard);
		strcat(str_fdtfile, ".dtb");
		env_set("fdtfile", str_fdtfile);
	}
	return 0;

}

int board_late_init(void)
{
#ifndef CONFIG_AVB_SUPPORT
	reset_dsi();
	detect_baseboard();
#ifdef CONFIG_TN_PHERIPHERAL_DETECT
	detect_display_panel();
	detect_camera();
#endif
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "AXON");
	env_set("board_rev", "iMX8MP");
#endif
	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif

#ifdef CONFIG_SPL_MMC

#define UBOOT_RAW_SECTOR_OFFSET 0x40
unsigned long spl_mmc_get_uboot_raw_sector(struct mmc *mmc, unsigned long raw_sect)
{
	u32 boot_dev = spl_boot_device();
	switch (boot_dev) {
		case BOOT_DEVICE_MMC2:
			return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR - UBOOT_RAW_SECTOR_OFFSET;
		default:
			return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;
	}
}
#endif
