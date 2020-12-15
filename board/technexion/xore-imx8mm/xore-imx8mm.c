/*
 * Copyright 2018 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *         Ray Chang <ray.chang@technexion.com>
 *         Andy Lin <andy.lin@technexion.com>
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
#include <sec_mipi_pll_1432x.h>
#include <sec_mipi_dphy_ln14lpp.h>

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

int board_init(void)
{
	setup_wifi();
#ifdef CONFIG_FEC_MXC
	setup_fec();
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

#define MIPI_DSI_I2C_BUS 2
#define MIPI_DSI_LVDS_I2C_BUS 1
#define ADV7535_MAIN_I2C_ADDR 0x3d
#define FT5336_TOUCH_I2C_ADDR 0x38
#define SN65DSI84_I2C_ADDR 0x2d

#define DISPLAY_NAME_MIPI2HDMI   "MIPI2HDMI"
#define DISPLAY_NAME_MIPI5       "ILI9881C_LCD"
#define DISPLAY_NAME_MIPI8       "G080UAN01_LCD"
#define DISPLAY_NAME_MIPI10      "G101UAN02_LCD"
#define DISPLAY_NAME_MIPI2LVDS10 "M101NWWB_LCD"
#define DISPLAY_NAME_MIPI2LVDS15 "G156XW01_LCD"
#define DISPLAY_NAME_MIPI2LVDS21 "G215HVN01_LCD"

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.reg_base = MIPI_DSI_BASE_ADDR,
	.gpr_base = CSI_BASE_ADDR + 0x8000,
	.dphy_pll = &pll_1432x,
	.dphy_timing = dphy_timing_ln14lpp_v1p2,
	.num_dphy_timing = ARRAY_SIZE(dphy_timing_ln14lpp_v1p2),
	.dphy_timing_cmp = dphy_timing_default_cmp,
};

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

struct mipi_panel_id {
	const char *panel_name;
	int id;
	const char *suffix;
};

static const struct mipi_panel_id mipi_panel_mapping[] = {
	{DISPLAY_NAME_MIPI2HDMI, 0, "-adv7535"},
	{DISPLAY_NAME_MIPI5, 0x54, "-ili9881c"},
	{DISPLAY_NAME_MIPI8, 0x58, "-g080uan01"},
	{DISPLAY_NAME_MIPI10, 0x59, "-g101uan02"},
	{DISPLAY_NAME_MIPI2LVDS10, 0, "-sn65dsi84-m101nwwb"},
	{DISPLAY_NAME_MIPI2LVDS15, 0, "-sn65dsi84-g156xw01"},
	{DISPLAY_NAME_MIPI2LVDS21, 0, "-sn65dsi84-g215hvn01"},
};

static int detect_i2c(struct display_info_t const *dev)
{
	struct udevice *bus, *i2c_dev = NULL;
	int ret = 0, val, i;

	if ((0 == uclass_get_device_by_seq(UCLASS_I2C, MIPI_DSI_I2C_BUS, &bus)) &&
			(0 == dm_i2c_probe(bus, dev->addr, 0, &i2c_dev))) {
		if (dev->addr == FT5336_TOUCH_I2C_ADDR) {
			val = dm_i2c_reg_read(i2c_dev, 0xA3);
			for (i = 1; i < ARRAY_SIZE(mipi_panel_mapping); i++) {
				const struct mipi_panel_id *instr = &mipi_panel_mapping[i];
				if((strcmp(instr->panel_name, dev->mode.name) == 0) &&
						(instr->id == val)) {
					ret = 1;
					break;
				}
			}
		} else {
			ret = 1;
		}
	} else 	if ((0 == uclass_get_device_by_seq(UCLASS_I2C, MIPI_DSI_LVDS_I2C_BUS, &bus)) &&
				(0 == dm_i2c_probe(bus, dev->addr, 0, &i2c_dev))) {
		if (dev->addr == SN65DSI84_I2C_ADDR) {
			ret = 1;
		}
	}

	return ret;
}

struct mipi_dsi_client_dev adv7535_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
	.name = "ADV7535",
};

struct mipi_dsi_client_dev ili9881c_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_VIDEO_HSE,
};

struct mipi_dsi_client_dev gxxxuan_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE,
};

struct mipi_dsi_client_dev sn65dsi84_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_VIDEO_BURST,
};

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				9
#define MIPI				10

void do_enable_mipi2hdmi(struct display_info_t const *dev)
{
#ifdef CONFIG_ADV75353
	/* ADV7353 initialization */
	adv7535_init(MIPI_DSI_I2C_BUS);
#endif

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);
	imx_mipi_dsi_bridge_attach(&adv7535_dev); /* attach adv7535 device */
}

#define DSI1_RST_PAD IMX_GPIO_NR(1, 11)
#define DSI1_VDDEN_PAD IMX_GPIO_NR(1, 12)
#define DSI1_BL_EN_PAD IMX_GPIO_NR(1, 10)
#define DSI1_BL_PWM_PAD IMX_GPIO_NR(1, 1)
static iomux_v3_cfg_t const dsi1_ctrl_pads[] = {
	IMX8MM_PAD_GPIO1_IO10_GPIO1_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
	IMX8MM_PAD_GPIO1_IO11_GPIO1_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
	IMX8MM_PAD_GPIO1_IO12_GPIO1_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),
	IMX8MM_PAD_SPDIF_EXT_CLK_GPIO5_IO5 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void do_enable_mipi_lcd(struct display_info_t const *dev)
{
	imx_iomux_v3_setup_multiple_pads(dsi1_ctrl_pads, ARRAY_SIZE(dsi1_ctrl_pads));
	gpio_request(DSI1_VDDEN_PAD, "DSI VDDEN");
	gpio_direction_output(DSI1_VDDEN_PAD, 1);
	mdelay(5);
	gpio_request(DSI1_RST_PAD, "DSI RST");
	gpio_direction_output(DSI1_RST_PAD, 0);
	mdelay(20);
	gpio_direction_output(DSI1_RST_PAD, 1);
	gpio_request(DSI1_BL_PWM_PAD, "DSI BL");
	gpio_direction_output(DSI1_BL_PWM_PAD, 1);
	gpio_request(DSI1_BL_EN_PAD, "DSI BLEN");
	gpio_direction_output(DSI1_BL_EN_PAD, 0);
	mdelay(10);
	gpio_direction_output(DSI1_BL_EN_PAD, 1);

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);

	if (!strcmp(dev->mode.name, DISPLAY_NAME_MIPI5)) {
#ifdef CONFIG_ILI9881C
		ili9881c_init();
		ili9881c_dev.name = dev->mode.name;
		imx_mipi_dsi_bridge_attach(&ili9881c_dev); /* attach ili9881c device */
#endif
	}else if (!strcmp(dev->mode.name, DISPLAY_NAME_MIPI8) ||
				!strcmp(dev->mode.name, DISPLAY_NAME_MIPI10)) {
		gxxxuan_dev.name = dev->mode.name;
		imx_mipi_dsi_bridge_attach(&gxxxuan_dev);
	}else if (!strcmp(dev->mode.name, DISPLAY_NAME_MIPI2LVDS10) ||
				!strcmp(dev->mode.name, DISPLAY_NAME_MIPI2LVDS15) ||
				!strcmp(dev->mode.name, DISPLAY_NAME_MIPI2LVDS21)) {
#ifdef CONFIG_SN65DSI84
		sn65dsi84_init(MIPI_DSI_LVDS_I2C_BUS, dev->mode.name);
		sn65dsi84_dev.name = dev->mode.name;
		imx_mipi_dsi_bridge_attach(&sn65dsi84_dev);
#endif
	}
}

void board_quiesce_devices(void)
{
	gpio_request(DSI1_VDDEN_PAD, "DSI VDDEN");
	gpio_direction_output(DSI1_VDDEN_PAD, 0);
}

struct display_info_t const displays[] = {{
	.bus = LCDIF_BASE_ADDR,
	.addr = ADV7535_MAIN_I2C_ADDR,
	.pixfmt = 24,
	.detect = detect_i2c,
	.enable	= do_enable_mipi2hdmi,
	.mode	= {
		.name			= DISPLAY_NAME_MIPI2HDMI,
		.refresh		= 60,
		.xres			= 1920,
		.yres			= 1080,
		.pixclock		= 6734, /* 148500 kHz */
		.left_margin	= 148,
		.right_margin	= 88,
		.upper_margin	= 36,
		.lower_margin	= 4,
		.hsync_len		= 44,
		.vsync_len		= 5,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = FT5336_TOUCH_I2C_ADDR,
	.pixfmt = 24,
	.detect = detect_i2c,
	.enable	= do_enable_mipi_lcd,
	.mode	= {
		.name			= DISPLAY_NAME_MIPI5,
		.refresh		= 60,
		.xres			= 720,
		.yres			= 1280,
		.pixclock		= 16129, /* 62000  kHz */
		.left_margin	= 30,
		.right_margin	= 10,
		.upper_margin	= 20,
		.lower_margin	= 10,
		.hsync_len		= 20,
		.vsync_len		= 10,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = FT5336_TOUCH_I2C_ADDR,
	.pixfmt = 24,
	.detect = detect_i2c,
	.enable	= do_enable_mipi_lcd,
	.mode	= {
		.name			= DISPLAY_NAME_MIPI8,
		.refresh		= 60,
		.xres			= 1200,
		.yres			= 1920,
		.pixclock		= 6273, /* 956400  kHz */
		.left_margin	= 60,
		.right_margin	= 60,
		.upper_margin	= 25,
		.lower_margin	= 35,
		.hsync_len		= 2,
		.vsync_len		= 1,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = FT5336_TOUCH_I2C_ADDR,
	.pixfmt = 24,
	.detect = detect_i2c,
	.enable	= do_enable_mipi_lcd,
	.mode	= {
		.name			= DISPLAY_NAME_MIPI10,
		.refresh		= 60,
		.xres			= 1920,
		.yres			= 1200,
		.pixclock		= 6671, /* 899400  kHz */
		.left_margin	= 60,
		.right_margin	= 60,
		.upper_margin	= 5,
		.lower_margin	= 5,
		.hsync_len		= 18,
		.vsync_len		= 2,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = SN65DSI84_I2C_ADDR,
	.pixfmt = 24,
	.detect = detect_i2c,
	.enable	= do_enable_mipi_lcd,
	.mode	= {
		.name			= DISPLAY_NAME_MIPI2LVDS10,
		.refresh		= 60,
		.xres			= 1280,
		.yres			= 800,
		.pixclock		= 14513, /* 413400  kHz */
		.left_margin	= 40,
		.right_margin	= 40,
		.upper_margin	= 10,
		.lower_margin	= 3,
		.hsync_len		= 80,
		.vsync_len		= 10,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = SN65DSI84_I2C_ADDR,
	.pixfmt = 24,
	.detect = detect_i2c,
	.enable	= do_enable_mipi_lcd,
	.mode	= {
		.name			= DISPLAY_NAME_MIPI2LVDS15,
		.refresh		= 60,
		.xres			= 1368,
		.yres			= 768,
		.pixclock		= 13157, /* 456000  kHz */
		.left_margin	= 90,
		.right_margin	= 90,
		.upper_margin	= 17,
		.lower_margin	= 17,
		.hsync_len		= 20,
		.vsync_len		= 4,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = SN65DSI84_I2C_ADDR,
	.pixfmt = 24,
	.detect = detect_i2c,
	.enable	= do_enable_mipi_lcd,
	.mode	= {
		.name			= DISPLAY_NAME_MIPI2LVDS21,
		.refresh		= 60,
		.xres			= 1920,
		.yres			= 1080,
		.pixclock		= 14285, /* 420000  kHz */
		.left_margin	= 70,
		.right_margin	= 70,
		.upper_margin	= 17,
		.lower_margin	= 17,
		.hsync_len		= 20,
		.vsync_len		= 4,
		.sync			= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode			= FB_VMODE_NONINTERLACED

} } };
size_t display_count = ARRAY_SIZE(displays);
#endif

int board_late_init(void)
{
	char *fdt_file, str_fdtfile[64];
	char const *panel = env_get("panel");
	int i;

	fdt_file = env_get("fdt_file");
	if (fdt_file && !strcmp(fdt_file, "undefined")) {
		strcpy(str_fdtfile, "imx8mm-xore-wizard");

		for (i = 0; i < display_count; i++) {
			struct display_info_t const *dev = displays+i;
			if ((!panel && dev->detect && dev->detect(dev)) || !strcmp(panel, dev->mode.name)) {
				strcat(str_fdtfile, mipi_panel_mapping[i].suffix);
				break;
			}
		}
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
