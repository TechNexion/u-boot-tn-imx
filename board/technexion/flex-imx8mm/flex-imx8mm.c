/*
 * Copyright 2018 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *         Ray Chang <ray.chang@technexion.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include <usb.h>
#include <sec_mipi_dsim.h>
#include <imx_mipi_dsi_bridge.h>
#include <mipi_dsi_panel.h>
#include <asm/mach-imx/video.h>
#include <dm/uclass.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART2_RXD_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART2_TXD_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
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

static int ddr_size;

int dram_init(void)
{
	/*************************************************
	ToDo: It's a dirty workaround to store the
	information of DDR size into start address of TCM.
	It'd be better to detect DDR size from DDR controller.
	**************************************************/
	ddr_size = readl(M4_BOOTROM_BASE_ADDR);

	if (ddr_size == 0x4) {
		/* rom_pointer[1] contains the size of TEE occupies */
		if (rom_pointer[1])
			gd->ram_size = PHYS_SDRAM_SIZE_4GB - rom_pointer[1];
		else
			gd->ram_size = PHYS_SDRAM_SIZE_4GB;
	}
	else if (ddr_size == 0x3) {
		if (rom_pointer[1])
			gd->ram_size = PHYS_SDRAM_SIZE_3GB - rom_pointer[1];
		else
			gd->ram_size = PHYS_SDRAM_SIZE_3GB;
	}
	else if (ddr_size == 0x2) {
		if (rom_pointer[1])
			gd->ram_size = PHYS_SDRAM_SIZE_2GB - rom_pointer[1];
		else
			gd->ram_size = PHYS_SDRAM_SIZE_2GB;
	}
	else if (ddr_size == 0x1) {
		if (rom_pointer[1])
			gd->ram_size = PHYS_SDRAM_SIZE_1GB - rom_pointer[1];
		else
			gd->ram_size = PHYS_SDRAM_SIZE_1GB;
	}
	else
		puts("Unknown DDR type!!!\n");
	return 0;
}

/* Get the top of usable RAM */
ulong board_get_usable_ram_top(ulong total_size)
{
	if(gd->ram_top > 0x100000000)
		gd->ram_top = 0x100000000;

	return gd->ram_top;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	return 0;
}
#endif

#ifdef CONFIG_FEC_MXC
#define FEC_RST_PAD IMX_GPIO_NR(2, 10)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
	IMX8MM_PAD_SD1_RESET_B_GPIO2_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_rst_pads,
					 ARRAY_SIZE(fec1_rst_pads));

	gpio_request(FEC_RST_PAD, "fec1_rst");
	gpio_direction_output(FEC_RST_PAD, 0);
	udelay(500);
	gpio_direction_output(FEC_RST_PAD, 1);
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

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	debug("board_usb_init %d, type %d\n", index, init);

	imx8m_usb_power(index, true);

	return ret;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;

	debug("board_usb_cleanup %d, type %d\n", index, init);

	imx8m_usb_power(index, false);
	return ret;
}

#define WL_REG_ON_PAD IMX_GPIO_NR(1, 0)
static iomux_v3_cfg_t const wl_reg_on_pads[] = {
	IMX8MM_PAD_GPIO1_IO00_GPIO1_IO0 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define BT_ON_PAD IMX_GPIO_NR(1, 3)
static iomux_v3_cfg_t const bt_on_pads[] = {
	IMX8MM_PAD_GPIO1_IO03_GPIO1_IO3 | MUX_PAD_CTRL(NO_PAD_CTRL),
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

#ifndef CONFIG_VIDEO_MXS
#define DSI1_RST_PAD IMX_GPIO_NR(1, 11)
#define DSI1_VDDEN_PAD IMX_GPIO_NR(1, 12)
static iomux_v3_cfg_t const dsi1_ctrl_pads[] = {
	IMX8MM_PAD_GPIO1_IO11_GPIO1_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
	IMX8MM_PAD_GPIO1_IO12_GPIO1_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void setup_mipi_dsi(void)
{
	imx_iomux_v3_setup_multiple_pads(dsi1_ctrl_pads, ARRAY_SIZE(dsi1_ctrl_pads));

	gpio_request(DSI1_RST_PAD, "dsi1_rst");
	gpio_direction_output(DSI1_RST_PAD, 0);

	gpio_request(DSI1_VDDEN_PAD, "dsi1_vdden");
	gpio_direction_output(DSI1_VDDEN_PAD, 0);
}
#endif // #ifndef CONFIG_VIDEO_MXS

int board_init(void)
{
	setup_wifi();
#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif
#ifndef CONFIG_VIDEO_MXS
	setup_mipi_dsi();
#endif

	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}

int mmc_map_to_kernel_blk(int devno)
{
	return devno + 1;
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

#ifdef CONFIG_VIDEO_MXS

#define DISPLAY_MIX_SFT_RSTN_CSR		0x00
#define DISPLAY_MIX_CLK_EN_CSR		0x04

   /* 'DISP_MIX_SFT_RSTN_CSR' bit fields */
#define BUS_RSTN_BLK_SYNC_SFT_EN	BIT(6)

   /* 'DISP_MIX_CLK_EN_CSR' bit fields */
#define LCDIF_PIXEL_CLK_SFT_EN		BIT(7)
#define LCDIF_APB_CLK_SFT_EN		BIT(6)

void disp_mix_bus_rstn_reset(ulong gpr_base, bool reset)
{
	if (!reset)
		/* release reset */
		setbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
	else
		/* hold reset */
		clrbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
}

void disp_mix_lcdif_clks_enable(ulong gpr_base, bool enable)
{
	if (enable)
		/* enable lcdif clks */
		setbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
	else
		/* disable lcdif clks */
		clrbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
}

struct mipi_dsi_client_dev adv7535_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
	.name = "ADV7535",
};

struct mipi_dsi_client_dev rm67191_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
};

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				9
#define MIPI				10

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.reg_base = MIPI_DSI_BASE_ADDR,
	.gpr_base = CSI_BASE_ADDR + 0x8000,
};

void do_enable_mipi_led(struct display_info_t const *dev)
{
	gpio_request(IMX_GPIO_NR(1, 12), "DSI VDDEN");
	gpio_direction_output(IMX_GPIO_NR(1, 12), 0);
	mdelay(100);
	gpio_direction_output(IMX_GPIO_NR(1, 12), 1);

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);

	rm67191_init();
	rm67191_dev.name = displays[1].mode.name;
	imx_mipi_dsi_bridge_attach(&rm67191_dev); /* attach rm67191 device */
}

void board_quiesce_devices(void)
{
	gpio_request(IMX_GPIO_NR(1, 12), "DSI VDDEN");
	gpio_direction_output(IMX_GPIO_NR(1, 12), 0);
}

struct display_info_t const displays[] = {{
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
	.enable	= do_enable_mipi_led,
	.mode	= {
		.name			= "RM67191_OLED",
		.refresh		= 60,
		.xres			= 1080,
		.yres			= 1920,
		.pixclock		= 7575, /* 132000000 */
		.left_margin	= 34,
		.right_margin	= 20,
		.upper_margin	= 4,
		.lower_margin	= 10,
		.hsync_len		= 2,
		.vsync_len		= 2,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} } };
size_t display_count = ARRAY_SIZE(displays);
#endif

#define FT5336_TOUCH_I2C_BUS 0
#define FT5336_TOUCH_I2C_ADDR 0x38

int board_late_init(void)
{
	struct udevice *bus;
	struct udevice *i2c_dev = NULL;
	int ret;
	char *fdt_file;

	fdt_file = env_get("fdt_file");
	if (fdt_file && !strcmp(fdt_file, "undefined")) {
		ret = uclass_get_device_by_seq(UCLASS_I2C, FT5336_TOUCH_I2C_BUS, &bus);
		if (ret) {
			printf("%s: Can't find bus\n", __func__);
			return -EINVAL;
		}

		ret = dm_i2c_probe(bus, FT5336_TOUCH_I2C_ADDR, 0, &i2c_dev);
		if (ret)
			env_set("fdt_file", "imx8mm-flex-pi.dtb");
		else
			env_set("fdt_file", "imx8mm-flex-pi-ili9881c.dtb");
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
