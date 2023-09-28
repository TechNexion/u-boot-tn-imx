/*
 * Copyright 2019 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <env.h>
#include <init.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <asm/armv8/mmu.h>
#include <spl.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../../freescale/common/tcpc.h"
#include "../../freescale/common/pfuze.h"
#include <usb.h>
#include <dwc3-uboot.h>
#include <splash.h>

DECLARE_GLOBAL_DATA_PTR;

#define QSPI_PAD_CTRL	(PAD_CTL_DSE2 | PAD_CTL_HYS)

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MQ_PAD_GPIO1_IO02__WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MQ_PAD_UART1_RXD__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_UART1_TXD__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	return 0;
}

#ifdef CONFIG_BOARD_POSTCLK_INIT
int board_postclk_init(void)
{
	/* TODO */
	return 0;
}
#endif

int board_phys_sdram_size(phys_size_t *size)
{
	/*************************************************
	ToDo: It's a dirty workaround to store the
	information of DDR size into start address of TCM.
	It'd be better to detect DDR size from DDR controller.
	**************************************************/
    if (!size)
        return -EINVAL;

    u32 ddr_size = readl(MCU_BOOTROM_BASE_ADDR);

    switch (ddr_size) {
	case 0x4: /* DRAM size: 4GB */
		*size = SZ_4G;
		break;
	case 0x3: /* DRAM size: 3GB */
		*size = SZ_3G;
		break;
	case 0x2: /* DRAM size: 2GB */
		*size = SZ_2G;
		break;
	case 0x1: /* DRAM size: 1GB */
		*size = SZ_1G;
		break;
	default:
		puts("Unknown DDR type!!!\n");
	}

    return 0;

}

/* Get the top of usable RAM */
ulong board_get_usable_ram_top(ulong total_size)
{
        if(gd->ram_top > 0x100000000)
            gd->ram_top = 0x100000000;

        return gd->ram_top;
}

#ifdef CONFIG_FEC_MXC
#define FEC_RST_PAD IMX_GPIO_NR(1, 9)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
	IMX8MQ_PAD_GPIO1_IO09__GPIO1_IO9 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define FEC_PWR_PAD IMX_GPIO_NR(1, 13)
static iomux_v3_cfg_t const fec1_pwr_pads[] = {
	IMX8MQ_PAD_GPIO1_IO13__GPIO1_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define WL_REG_ON_PAD IMX_GPIO_NR(3, 24)
static iomux_v3_cfg_t const wl_reg_on_pads[] = {
	IMX8MQ_PAD_SAI5_RXD3__GPIO3_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define BT_ON_PAD IMX_GPIO_NR(3, 21)
static iomux_v3_cfg_t const bt_on_pads[] = {
	IMX8MQ_PAD_SAI5_RXD0__GPIO3_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_rst_pads, ARRAY_SIZE(fec1_rst_pads));
	imx_iomux_v3_setup_multiple_pads(fec1_pwr_pads, ARRAY_SIZE(fec1_pwr_pads));

	gpio_request(IMX_GPIO_NR(1, 13), "fec1_pwr");
	gpio_direction_output(IMX_GPIO_NR(1, 13), 1);
	udelay(500);

	gpio_request(IMX_GPIO_NR(1, 9), "fec1_rst");
	gpio_direction_output(IMX_GPIO_NR(1, 9), 0);
	udelay(35000);
	gpio_direction_output(IMX_GPIO_NR(1, 9), 1);
}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	setup_iomux_fec();

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_SHIFT, 0);
	return set_clk_enet(ENET_125MHZ);
}


int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
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

static struct dwc3_device dwc3_device_data = {
	.maximum_speed = USB_SPEED_HIGH,
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
	.power_down_scale = 2,
};

int usb_gadget_handle_interrupts(void)
{
	dwc3_uboot_handle_interrupt(0);
	return 0;
}

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 RegData;

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
	int ret = 0;
	imx8m_usb_power(index, true);

	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	} else if (index == 0 && init == USB_INIT_HOST) {
		return ret;
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;
	if (index == 0 && init == USB_INIT_DEVICE)
			dwc3_uboot_exit(index);

	imx8m_usb_power(index, false);

	return ret;
}
#endif

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

int splash_screen_prepare(void)
{
	imx_splash_locations[1].devpart[0] = mmc_get_env_dev() + '0';
	return splash_source_load(imx_splash_locations, ARRAY_SIZE(imx_splash_locations));
}
#endif /* CONFIG_SPLASH_SCREEN */

#define MIPI_RESET_PAD IMX_GPIO_NR(3, 4)
static iomux_v3_cfg_t const mipi_reset_pads[] = {
	IMX8MQ_PAD_NAND_CE3_B__GPIO3_IO4 | MUX_PAD_CTRL(NO_PAD_CTRL),  /* DSI RST, GPIO_P257 */
};

void reset_mipi_panel(void)
{
	imx_iomux_v3_setup_multiple_pads(mipi_reset_pads, ARRAY_SIZE(mipi_reset_pads));

	gpio_request(MIPI_RESET_PAD, "mipi_reset");
	gpio_direction_output(MIPI_RESET_PAD, 1);
	udelay(500);
	gpio_direction_output(MIPI_RESET_PAD, 0);
	udelay(500);
	gpio_direction_output(MIPI_RESET_PAD, 1);

}

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

int board_init(void)
{
	setup_wifi();

	reset_mipi_panel();

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
	init_usb_clk();
#endif

	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
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

int add_dtoverlay(char *ov_name)
{
	char *dtoverlay, arr_dtov[64];

	dtoverlay = env_get("dtoverlay");
	if (dtoverlay) {
		strcpy(arr_dtov, dtoverlay);
		if (!strstr(arr_dtov, ov_name)) {
			strcat(arr_dtov, " ");
			strcat(arr_dtov, ov_name);
			env_set("dtoverlay", arr_dtov);
		}
	} else
		env_set("dtoverlay", ov_name);

	return 0;
}

#define FT5336_TOUCH_I2C_BUS 3
#define FT5336_TOUCH_I2C_ADDR 0x38
#define ADV7535_HDMI_I2C_ADDR 0x3d

int detect_display_panel(void)
{
	struct udevice *bus = NULL;
	struct udevice *i2c_dev = NULL;
	int ret, touch_id;

	ret = uclass_get_device_by_seq(UCLASS_I2C, FT5336_TOUCH_I2C_BUS, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return -EINVAL;
	}
	/* detect different MIPI panel by touch controller */
	ret = dm_i2c_probe(bus, FT5336_TOUCH_I2C_ADDR, 0, &i2c_dev);
	if (! ret) {
		touch_id = dm_i2c_reg_read(i2c_dev, 0xA3);
		switch (touch_id) {
		case 0x54:
			add_dtoverlay("ili9881c");
			break;
		case 0x58:
			add_dtoverlay("g080uan01");
			break;
		case 0x59:
			add_dtoverlay("g101uan02");
			break;
		default:
			printf("Unknown panel ID!\r\n");
		}
	}

	/* detect MIPI2HDMI controller */
	ret = dm_i2c_probe(bus, ADV7535_HDMI_I2C_ADDR, 0, &i2c_dev);
	if (! ret) {
		add_dtoverlay("mipi2hdmi-adv7535");
	}

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	const int *cell;
	int offs;
	uint32_t cma_size;
	char *cmasize;

	offs = fdt_path_offset(blob, "/reserved-memory/linux,cma");
	cell = fdt_getprop(blob, offs, "size", NULL);
	cma_size = fdt32_to_cpu(cell[1]);
	cmasize = env_get("cma_size");
	if(cmasize || ((u64)(gd->ram_size >> 1) < cma_size)) {
		cma_size = env_get_ulong("cma_size", 10, 320 * 1024 * 1024);
		cma_size = min((uint32_t)(mem_map[DRAM1_INDEX].size >> 1), cma_size);
		fdt_setprop_u64(blob, offs, "size", (uint64_t)cma_size);
	}

	return 0;
}
#endif

int board_late_init(void)
{
	char *fdt_file, str_fdtfile[64];

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "EDM-IMX8MQ");
	env_set("board_rev", "iMX8MQ");
	detect_display_panel();
#endif

	fdt_file = env_get("fdt_file");
	if (fdt_file && !strcmp(fdt_file, "undefined")) {
		strcpy(str_fdtfile, "imx8mq-edm-wizard");
		strcat(str_fdtfile, ".dtb");
		env_set("fdt_file", str_fdtfile);
	}

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
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

/* return hard code board id for imx8m_ref */
#if defined(CONFIG_ANDROID_THINGS_SUPPORT) && defined(CONFIG_ARCH_IMX8M)
int get_imx8m_baseboard_id(void)
{
	return IMX8M_REF_3G;
}
#endif
