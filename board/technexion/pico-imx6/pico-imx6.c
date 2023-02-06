// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2014 O.S. Systems Software LTDA.
 * Copyright (C) 2020 TechNexion Ltd.
 *
 * Author: Fabio Estevam <festevam@gmail.com>
 *         Richard Hu <richard.hu@technexion.com>
 *         Ray Chang <ray.chang@technexion.com>
 */

#include <common.h>
#include <env.h>
#include <init.h>
#include <net.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/sizes.h>
#include <i2c.h>
#include <miiphy.h>
#include <netdev.h>
#include <phy.h>
#include <splash.h>
#include <mmc.h>
#include <command.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define ETH_PHY_RESET		IMX_GPIO_NR(1, 26)
#define WL_REG_ON		IMX_GPIO_NR(1, 7)
#define BT_NRST			IMX_GPIO_NR(7, 12)
#define LVDS0_EN		IMX_GPIO_NR(2, 8)
#define LVDS0_BL_EN		IMX_GPIO_NR(2, 9)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}

static iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC    | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL |
		   MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK |
		   MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* AR8035 PHY Reset */
        IOMUX_PADS(PAD_ENET_RXD1__GPIO1_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const wifi_pads[] = {
	/* wifi wl-reg-on */
	IOMUX_PADS(PAD_GPIO_7__GPIO1_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const bt_pads[] = {
	/* bluetooth BT_nRST */
	IOMUX_PADS(PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);

	/* Reset AR8031 PHY */
	gpio_request(ETH_PHY_RESET, "enet_phy_reset");
	gpio_direction_output(ETH_PHY_RESET, 0);
	mdelay(35);
	gpio_set_value(ETH_PHY_RESET, 1);
	mdelay(75);
}

static bool cpu_is_pop(void)
{
	u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	u32 ddr_map;

	/* BOOT_CFG3[4] and BOOT_CFG3[5] */
	ddr_map = (soc_sbmr >> 20) & 0x3;

	return((ddr_map == 0x2)?(true):(false));
}

struct i2c_pads_info mx6q_i2c2_pad_info = {
	.scl = {
		.i2c_mode = MX6Q_PAD_KEY_COL3__I2C2_SCL
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO4_IO12
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_KEY_ROW3__I2C2_SDA
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO4_IO13
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

struct i2c_pads_info mx6dl_i2c2_pad_info = {
	.scl = {
		.i2c_mode = MX6DL_PAD_KEY_COL3__I2C2_SCL
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6DL_PAD_KEY_COL3__GPIO4_IO12
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6DL_PAD_KEY_ROW3__I2C2_SDA
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6DL_PAD_KEY_ROW3__GPIO4_IO13
			| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

struct i2c_pads_info mx6q_i2c3_pad_info = {
	.scl = {
		.i2c_mode = MX6Q_PAD_EIM_D17__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_EIM_D17__GPIO3_IO17 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_EIM_D18__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_EIM_D18__GPIO3_IO18 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 18)
	}
};

struct i2c_pads_info mx6dl_i2c3_pad_info = {
	.scl = {
		.i2c_mode = MX6DL_PAD_EIM_D17__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6DL_PAD_EIM_D17__GPIO3_IO17 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6DL_PAD_EIM_D18__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6DL_PAD_EIM_D18__GPIO3_IO18 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 18)
	}
};

#ifndef CONFIG_SPL_BUILD
#if defined(CONFIG_VIDEO_IPUV3)
static iomux_v3_cfg_t const lvds_pads[] = {
	/* lvds */
	IOMUX_PADS(PAD_SD4_DAT0__GPIO2_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__GPIO2_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const ft5x06_wvga_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK),
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02), /* HSync */
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03), /* VSync */
	IOMUX_PADS(PAD_DI0_PIN4__IPU1_DI0_PIN04	| MUX_PAD_CTRL(PAD_CTL_DSE_120ohm)), /* Contrast */
	IOMUX_PADS(PAD_DI0_PIN15__IPU1_DI0_PIN15), /* DISP0_DRDY */
	IOMUX_PADS(PAD_DISP0_DAT0__IPU1_DISP0_DATA00),
	IOMUX_PADS(PAD_DISP0_DAT1__IPU1_DISP0_DATA01),
	IOMUX_PADS(PAD_DISP0_DAT2__IPU1_DISP0_DATA02),
	IOMUX_PADS(PAD_DISP0_DAT3__IPU1_DISP0_DATA03),
	IOMUX_PADS(PAD_DISP0_DAT4__IPU1_DISP0_DATA04),
	IOMUX_PADS(PAD_DISP0_DAT5__IPU1_DISP0_DATA05),
	IOMUX_PADS(PAD_DISP0_DAT6__IPU1_DISP0_DATA06),
	IOMUX_PADS(PAD_DISP0_DAT7__IPU1_DISP0_DATA07),
	IOMUX_PADS(PAD_DISP0_DAT8__IPU1_DISP0_DATA08),
	IOMUX_PADS(PAD_DISP0_DAT9__IPU1_DISP0_DATA09),
	IOMUX_PADS(PAD_DISP0_DAT10__IPU1_DISP0_DATA10),
	IOMUX_PADS(PAD_DISP0_DAT11__IPU1_DISP0_DATA11),
	IOMUX_PADS(PAD_DISP0_DAT12__IPU1_DISP0_DATA12),
	IOMUX_PADS(PAD_DISP0_DAT13__IPU1_DISP0_DATA13),
	IOMUX_PADS(PAD_DISP0_DAT14__IPU1_DISP0_DATA14),
	IOMUX_PADS(PAD_DISP0_DAT15__IPU1_DISP0_DATA15),
	IOMUX_PADS(PAD_DISP0_DAT16__IPU1_DISP0_DATA16),
	IOMUX_PADS(PAD_DISP0_DAT17__IPU1_DISP0_DATA17),
	IOMUX_PADS(PAD_DISP0_DAT18__IPU1_DISP0_DATA18),
	IOMUX_PADS(PAD_DISP0_DAT19__IPU1_DISP0_DATA19),
	IOMUX_PADS(PAD_DISP0_DAT20__IPU1_DISP0_DATA20),
	IOMUX_PADS(PAD_DISP0_DAT21__IPU1_DISP0_DATA21),
	IOMUX_PADS(PAD_DISP0_DAT22__IPU1_DISP0_DATA22),
	IOMUX_PADS(PAD_DISP0_DAT23__IPU1_DISP0_DATA23),
	IOMUX_PADS(PAD_SD4_DAT2__GPIO2_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL)), /* DISP0_BKLEN */
	IOMUX_PADS(PAD_SD4_DAT3__GPIO2_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL)), /* DISP0_VDDEN */
};

static void do_enable_hdmi(struct display_info_t const *dev)
{
	imx_enable_hdmi_phy();
}

static int detect_i2c(struct display_info_t const *dev)
{
	return (0 == i2c_set_bus_num(dev->bus)) &&
			(0 == i2c_probe(dev->addr));
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;

	/* set CH0 data width to 24bit (IOMUXC_GPR2:5 0=18bit, 1=24bit) */
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);

	/* Enable Backlight - use GPIO for Brightness adjustment */
	SETUP_IOMUX_PAD(PAD_SD4_DAT1__GPIO2_IO09);
	gpio_request(IMX_GPIO_NR(2, 9), "backlight_enable");
	gpio_direction_output(IMX_GPIO_NR(2, 9), 1);

	gpio_request(IMX_GPIO_NR(2, 8), "brightness");
	SETUP_IOMUX_PAD(PAD_SD4_DAT0__GPIO2_IO08);
	gpio_direction_output(IMX_GPIO_NR(2, 8), 1);
}

static void enable_ft5x06_wvga(struct display_info_t const *dev)
{
	SETUP_IOMUX_PADS(ft5x06_wvga_pads);

	gpio_request(IMX_GPIO_NR(2, 10), "parallel_enable");
	gpio_request(IMX_GPIO_NR(2, 11), "parallel_brightness");
	gpio_direction_output(IMX_GPIO_NR(2, 10), 1);
	gpio_direction_output(IMX_GPIO_NR(2, 11), 1);
}

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt = IPU_PIX_FMT_RGB24,
	.detect = NULL,
	.enable = enable_lvds,
	.mode	= {
		.name		= "hj070na",
		.refresh	= 60,
		.xres		= 1024,
		.yres		= 600,
		.pixclock	= 15385,
		.left_margin	= 220,
		.right_margin	= 40,
		.upper_margin	= 21,
		.lower_margin	= 7,
		.hsync_len	= 60,
		.vsync_len	= 10,
		.sync		= FB_SYNC_EXT,
		.vmode		= FB_VMODE_NONINTERLACED
	} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
	} }, {
	.bus	= 1,
	.addr	= 0x38,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_i2c,
	.enable	= enable_ft5x06_wvga,
	.mode	= {
		.name           = "FT5x06-WVGA",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 30303,
		.left_margin    = 45,
		.right_margin   = 210,
		.upper_margin   = 22,
		.lower_margin   = 22,
		.hsync_len      = 1,
		.vsync_len      = 1,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
	} }
};
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	SETUP_IOMUX_PADS(lvds_pads);
	gpio_request(LVDS0_EN, "lvds0_enable");
	gpio_request(LVDS0_BL_EN, "lvds0_bl_enable");
	gpio_direction_output(LVDS0_EN, 1);
	gpio_direction_output(LVDS0_BL_EN, 1);

	enable_ipu_clock();
	imx_setup_hdmi();

	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		| MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
		 | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	 reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
		| IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
		| IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
		| IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
		| IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT
		| IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
		| IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT
		| IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0
		| IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);
	reg = readl(&iomux->gpr[3]);

	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
		| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
		| (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
		<< IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);

	writel(reg, &iomux->gpr[3]);
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

int splash_screen_prepare(void)
{
	imx_splash_locations[1].devpart[0] = mmc_get_env_dev() + '0';
	return splash_source_load(imx_splash_locations, ARRAY_SIZE(imx_splash_locations));
}
#endif /* CONFIG_SPLASH_SCREEN */
#endif /* CONFIG_VIDEO_IPUV3 */
#endif /* CONFIG_SPL_BUILD */

int board_early_init_f(void)
{
	setup_iomux_uart();
#ifndef CONFIG_SPL_BUILD
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
	//enable_ft5x06_wvga(NULL);
#endif
#endif
	return 0;
}

int board_eth_init(struct bd_info *bis)
{
	setup_iomux_enet();

	return cpu_eth_init(bis);
}

int board_phy_config(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8035 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe7;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#ifdef CONFIG_ENV_IS_IN_MMC
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
#endif

int overwrite_console(void)
{
	return 1;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	char *s;
#endif

	if (is_mx6dq())
		env_set("board_rev", "MX6Q");
	else
		env_set("board_rev", "MX6DL");

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
	/* reset wifi chip */
	SETUP_IOMUX_PADS(wifi_pads);
	gpio_request(WL_REG_ON, "wl_reg_on");
	gpio_direction_output(WL_REG_ON, 0);
	mdelay(10);
	gpio_set_value(WL_REG_ON, 1);

	/* reset bluetooth chip */
	SETUP_IOMUX_PADS(bt_pads);
	gpio_request(BT_NRST, "bt_nrst");
	gpio_direction_output(BT_NRST, 0);
	mdelay(10);
	gpio_set_value(BT_NRST, 1);

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if ((s = env_get ("fdtfile_autodetect")) != NULL) {
		if (strncmp (s, "off", 3) != 0) {
			if (is_mx6dqp())
				env_set("som", "imx6qp");
			else if (is_mx6dq())
				env_set("som", "imx6q");
			else if (is_mx6sdl())
				env_set("som", "imx6dl");
			else
				printf("CPU type is not supported!!!\r\n");
		}
	}

	if ((s = env_get ("bootdev_autodetect")) != NULL) {
		if (strncmp (s, "off", 3) != 0) {
			switch (get_boot_device()) {
			case MMC3_BOOT:
			case SD3_BOOT:
				env_set("bootdev", "SD0");
				break;
			case SD1_BOOT:
				env_set("bootdev", "SD1");
				break;
			default:
				printf("Wrong boot device!");
			}
		}
	}
#endif

	return 0;
}

int board_init(void)
{
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	if (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D) ||
		is_cpu_type(MXC_CPU_MX6QP) || is_cpu_type(MXC_CPU_MX6DP)) {
		//setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c1_pad_info);
		setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c2_pad_info);
		setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c3_pad_info);
	} else {
		//setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c1_pad_info);
		setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c2_pad_info);
		setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c3_pad_info);
	}

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bis)
{
	const int *cell;
	int offs;
	uint32_t cma_size;
	unsigned int dram_size;

	dram_size = imx_ddr_size() / 1024 / 1024;
	offs = fdt_path_offset(blob, "/reserved-memory/linux,cma");
	cell = fdt_getprop(blob, offs, "size", NULL);
	cma_size = fdt32_to_cpu(cell[0]);
	if (dram_size == 512) {
		/* CMA is aligned by 32MB on i.mx8mq,
		   so CMA size can only be multiple of 32MB */
		cma_size = env_get_ulong("cma_size", 10, (6 * 32) * 1024 * 1024);
		fdt_setprop_u32(blob, offs, "size", (uint64_t)cma_size);
	}

	return 0;
}
#endif

int checkboard(void)
{
	if (cpu_is_pop())
		printf("Board: pico-imx6-pop\n");
	else
		printf("Board: pico-imx6\n");

	printf("Available baseboard: dwarf, hobbit, nymph, pi\n");

	return 0;
}
