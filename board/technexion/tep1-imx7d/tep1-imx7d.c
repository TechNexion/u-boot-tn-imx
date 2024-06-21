/*
 * Copyright (C) 2018 Technexion Ltd.
 *
 * Author: Tapani Utriainen <tapani@technexion.com>
 *         Richard Hu <richard.hu@technexion.com>
 *         Alvin Chen <alvin.chen@technexion.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <init.h>
#include <linux/delay.h>
#include <command.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include <asm/arch/clock_slice.h>
#ifdef CONFIG_SYS_I2C_MXC
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#endif
#if defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#include <asm/arch/crm_regs.h>

#ifdef CONFIG_VIDEO_MXS
#include <linux/fb.h>
#include <splash.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_DSE_3P3V_49OHM | \
	PAD_CTL_PUS_PU100KOHM | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)
#define ENET_PAD_CTRL_MII  (PAD_CTL_DSE_3P3V_32OHM)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)

#define I2C_PAD_CTRL    (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU100KOHM)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_PU100KOHM | \
	PAD_CTL_DSE_3P3V_49OHM)

#define LCD_SYNC_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_PU100KOHM | \
	PAD_CTL_DSE_3P3V_196OHM)

#define QSPI_PAD_CTRL	\
	(PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

#define SPI_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)

#define BUTTON_PAD_CTRL    (PAD_CTL_PUS_PU5KOHM | PAD_CTL_DSE_3P3V_98OHM)

#define NAND_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)

#define NAND_PAD_READY0_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUS_PU5KOHM)

#define EPDC_PAD_CTRL	0x0

#define DDR_TYPE_DET	IMX_GPIO_NR(4, 21)

#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC*/
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C1_SCL__GPIO4_IO8 | PC,
		.gp = IMX_GPIO_NR(4, 8),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C1_SDA__GPIO4_IO9 | PC,
		.gp = IMX_GPIO_NR(4, 9),
	},
};

/* I2C2 */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C2_SCL__I2C2_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C2_SCL__GPIO4_IO10 | PC,
		.gp = IMX_GPIO_NR(4, 10),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C2_SDA__I2C2_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C2_SDA__GPIO4_IO11 | PC,
		.gp = IMX_GPIO_NR(4, 11),
	},
};

struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C3_SCL__I2C3_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C3_SCL__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C3_SDA__I2C3_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C3_SDA__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13),
	},
};
#endif


#define USB_HUB_REV_GPIO    IMX_GPIO_NR(4, 14)

static iomux_v3_cfg_t const usb_hub_rev_pads[] = {
	MX7D_PAD_I2C4_SCL__GPIO4_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* Revision Detect
		USB2 HOST:R68 10K-R2 ON,R65 10K-R2 OFF.
		USB2 OTG :R65 10K-R2 ON,R68 10K-R2 OFF.
	*/
};

static u8 board_rev __section("data");
static void detect_board_rev(void)
{
	// USB HUB Revision Detect
	imx_iomux_v3_setup_multiple_pads(usb_hub_rev_pads, ARRAY_SIZE(usb_hub_rev_pads));

	gpio_request(USB_HUB_REV_GPIO, "board_rev");
	gpio_direction_input(USB_HUB_REV_GPIO);
	if (gpio_get_value(USB_HUB_REV_GPIO) != 0) {
		/* TEP1-IMX7D rev.A2 */
		board_rev = 1;
	} else {
		/* TEP1-IMX7D rev.A1 */
		board_rev = 0;
	}
}

static iomux_v3_cfg_t const ddr_type_detection_pads[] = {
	/* ddr type detection */
	MX7D_PAD_ECSPI2_MOSI__GPIO4_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_ddr_type_detection(void)
{
	imx_iomux_v3_setup_multiple_pads(ddr_type_detection_pads, ARRAY_SIZE(ddr_type_detection_pads));
}

int dram_init(void)
{
	unsigned int ddr_size;

	setup_iomux_ddr_type_detection();
	gpio_direction_input(DDR_TYPE_DET);

	if (gpio_get_value(DDR_TYPE_DET))
		ddr_size = 1024;
	else
		ddr_size = 512;

	gd->ram_size = ((ulong)ddr_size * 1024 * 1024);

	return 0;
}

static iomux_v3_cfg_t const wdog_pads[] = {
	MX7D_PAD_GPIO1_IO00__WDOG1_WDOG_B | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/* Console pinout for rev A1 of TEP1-IMX7 hardware */
static iomux_v3_cfg_t const uart2_pads[] = {
	MX7D_PAD_UART2_TX_DATA__UART2_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_UART2_RX_DATA__UART2_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* Console pinout for rev A2 of TEP1-IMX7 hardware */
static iomux_v3_cfg_t const uart3_pads[] = {
	MX7D_PAD_UART3_TX_DATA__UART3_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_UART3_RX_DATA__UART3_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX7D_PAD_SD1_CLK__SD1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_CMD__SD1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA0__SD1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA1__SD1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA2__SD1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA3__SD1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* Carrier MicroSD Card Detect */
	MX7D_PAD_SD1_CD_B__GPIO5_IO0  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

/* EMMC/SD */
static iomux_v3_cfg_t const usdhc3_emmc_pads[] = {
	MX7D_PAD_SD3_CLK__SD3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_CMD__SD3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

#ifdef CONFIG_SYS_USE_NAND
static iomux_v3_cfg_t const gpmi_pads[] = {
	MX7D_PAD_SD3_DATA0__NAND_DATA00 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA1__NAND_DATA01 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA2__NAND_DATA02 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA3__NAND_DATA03 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA4__NAND_DATA04 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA5__NAND_DATA05 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA6__NAND_DATA06 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_DATA7__NAND_DATA07 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_CLK__NAND_CLE	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_CMD__NAND_ALE	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_STROBE__NAND_RE_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SD3_RESET_B__NAND_WE_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_MCLK__NAND_WP_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	//MX7D_PAD_SAI1_RX_BCLK__NAND_CE3_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	//MX7D_PAD_SAI1_RX_SYNC__NAND_CE2_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_RX_DATA__NAND_CE1_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_TX_BCLK__NAND_CE0_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_TX_SYNC__NAND_DQS	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX7D_PAD_SAI1_TX_DATA__NAND_READY_B	| MUX_PAD_CTRL(NAND_PAD_READY0_CTRL),
};

static void setup_gpmi_nand(void)
{
	imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));

	/*
	 * NAND_USDHC_BUS_CLK is set in rom
	 */

	set_clk_nand();

	/*
	 * APBH clock root is set in init_esdhc, USDHC3_CLK.
	 * There is no clk gate for APBHDMA.
	 * No touch here.
	 */
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

static iomux_v3_cfg_t const ccm_clko_pads[] = {
	MX7D_PAD_GPIO1_IO03__CCM_CLKO2 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX7D_PAD_GPIO1_IO02__CCM_CLKO1 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

size_t uart_base_reg_addr = UART2_IPS_BASE_ADDR;

static void setup_iomux_uart(void)
{
	// USB HUB Revision Detect
	imx_iomux_v3_setup_multiple_pads(usb_hub_rev_pads, ARRAY_SIZE(usb_hub_rev_pads));

	gpio_request(USB_HUB_REV_GPIO, "board_rev");
	gpio_direction_input(USB_HUB_REV_GPIO);
	if (gpio_get_value(USB_HUB_REV_GPIO) != 0) {
		uart_base_reg_addr = UART3_IPS_BASE_ADDR;
		imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
	} else {
		uart_base_reg_addr = UART2_IPS_BASE_ADDR;
		imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	}
}

#define USDHC1_CD_GPIO	IMX_GPIO_NR(5, 0)

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC1_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO); /* Assume uSDHC1 sd is always present */
		break;
	case USDHC3_BASE_ADDR:
		ret = 1; /* Assume uSDHC3 emmc is always present */
		break;
	}

	return ret;
}

int board_mmc_init(struct bd_info *bis)
{
	int ret;
	u32 index = 0;

	/*
	 * Following map is done:
	 * (USDHC)	(Physical Port)
	 * usdhc3	SOM MicroSD/MMC
	 * usdhc1	Carrier board MicroSD
	 * Always set boot USDHC as mmc0
	 */

	imx_iomux_v3_setup_multiple_pads(
				usdhc3_emmc_pads, ARRAY_SIZE(usdhc3_emmc_pads));

	imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
	gpio_direction_input(USDHC1_CD_GPIO);

	switch (get_boot_device()) {
	case SD1_BOOT:
		usdhc_cfg[0].esdhc_base = USDHC1_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
		usdhc_cfg[0].max_bus_width = 4;
		usdhc_cfg[1].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		usdhc_cfg[1].max_bus_width = 8;
		break;
	case MMC3_BOOT:
	case SD3_BOOT:
	default:
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		usdhc_cfg[0].max_bus_width = 1;
		usdhc_cfg[1].esdhc_base = USDHC1_BASE_ADDR;
		usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
		usdhc_cfg[1].max_bus_width = 4;
		break;
	}

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (ret)
			return ret;
	}

	return 0;
}

void board_late_mmc_init(void)
{
	switch (get_boot_device()) {
	case SD3_BOOT:
	case MMC3_BOOT:
		env_set("bootdev", "SD0");
		break;
	case SD1_BOOT:
		env_set("bootdev", "SD1");
		break;
	default:
		printf("Wrong boot device!");
	}
}

#ifdef CONFIG_FEC_MXC
static int setup_fec(int fec_id)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	if (0 == fec_id) {
		/* Use 125M anatop REF_CLK1 for ENET1, clear gpr1[13], gpr1[17]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
		(IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK |
			IOMUXC_GPR_GPR1_GPR_ENET1_CLK_DIR_MASK), 0);
	} else {
		/* Use 125M anatop REF_CLK2 for ENET2, clear gpr1[14], gpr1[18]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
		(IOMUXC_GPR_GPR1_GPR_ENET2_TX_CLK_SEL_MASK |
			IOMUXC_GPR_GPR1_GPR_ENET2_CLK_DIR_MASK), 0);
	}

	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8035 output a 125MHz clk from CLK_25M */
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
#endif

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

int board_early_init_f(void)
{
#ifdef CONFIG_SUPPORT_SPL
	setup_iomux_uart();
#endif

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);
#endif

	return 0;
}

#ifdef CONFIG_VIDEO
void setup_lcd(void)
{
	/* Set Brightness to high */
	gpio_request(IMX_GPIO_NR(1, 10), "Brightness");
	gpio_direction_output(IMX_GPIO_NR(1, 10), 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 10), 1);

	/* Set LCD enable to high */
	gpio_request(IMX_GPIO_NR(3, 4), "LCD enable pin");
	gpio_direction_output(IMX_GPIO_NR(3, 4), 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(3, 4), 1);
}
#endif

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	detect_board_rev();

	if (board_rev == 1) {
		uart_base_reg_addr = UART3_IPS_BASE_ADDR;
		imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
	} else {
		uart_base_reg_addr = UART2_IPS_BASE_ADDR;
		imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	}

#ifdef CONFIG_VIDEO
	setup_lcd();
#endif
#ifdef CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif

	//edm1-imx7 custom initialize
	imx_iomux_v3_setup_multiple_pads(ccm_clko_pads, ARRAY_SIZE(ccm_clko_pads));

	clock_set_src(IPP_DO_CLKO2, OSC_32K_CLK);
	udelay(500);
	clock_set_src(IPP_DO_CLKO1, OSC_24M_CLK);

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1", MAKE_CFGVAL(0x10, 0x10, 0x00, 0x00)},
	{"emmc", MAKE_CFGVAL(0x10, 0x2a, 0x00, 0x00)},
	/* TODO: Nand */
	/*{"qspi", MAKE_CFGVAL(0x00, 0x40, 0x00, 0x00)},*/
	{NULL,   0},
};
#endif

#ifdef CONFIG_POWER
#define I2C_PMIC	0
int power_init_board(void)
{
	struct pmic *p;
	int ret;
	unsigned int reg, rev_id;

	ret = power_pfuze3000_init(I2C_PMIC);
	if (ret)
		return ret;

	p = pmic_get("PFUZE3000");
	ret = pmic_probe(p);
	if (ret)
		return ret;

	pmic_reg_read(p, PFUZE3000_DEVICEID, &reg);
	pmic_reg_read(p, PFUZE3000_REVID, &rev_id);
	printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", reg, rev_id);

	/* disable Low Power Mode during standby mode */
	pmic_reg_read(p, PFUZE3000_LDOGCTL, &reg);
	reg |= 0x1;
	pmic_reg_write(p, PFUZE3000_LDOGCTL, reg);

	/* SW1A/1B mode set to APS/APS */
	reg = 0x8;
	pmic_reg_write(p, PFUZE3000_SW1AMODE, reg);
	pmic_reg_write(p, PFUZE3000_SW1BMODE, reg);

	/* SW1A/1B standby voltage set to 1.025V */
	reg = 0xd;
	pmic_reg_write(p, PFUZE3000_SW1ASTBY, reg);
	pmic_reg_write(p, PFUZE3000_SW1BSTBY, reg);

	/* decrease SW1B normal voltage to 0.975V */
	pmic_reg_read(p, PFUZE3000_SW1BVOLT, &reg);
	reg &= ~0x1f;
	reg |= PFUZE3000_SW1AB_SETP(9750);
	pmic_reg_write(p, PFUZE3000_SW1BVOLT, reg);

	return 0;
}
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_init();
	board_late_mmc_env_init();
#endif

	char *s;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	env_set("som", get_som_type());

	if (board_rev == 1) {
		/* TEP1-IMX7D rev.A2 */
		env_set("baseboard", "tep1-a2");
		env_set("console", "ttymxc2");
	};

	s = env_get ("bootdev_autodetect");
	if (s != NULL) {
		if (strncmp (s, "off", 3) != 0) {
			switch (get_boot_device()) {
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
	return 0;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

int checkboard(void)
{
	puts("Board: TEP1-IMX7D\n");

	return 0;
}

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, struct bd_info *bis)
{
	return 0;
}
#endif

#ifdef CONFIG_USB_EHCI_MX7
iomux_v3_cfg_t const usb_otg1_pads[] = {
	MX7D_PAD_GPIO1_IO05__USB_OTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const usb_otg2_pads[] = {
	MX7D_PAD_UART3_CTS_B__USB_OTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int board_ehci_hcd_init(int port)
{
	switch (port) {
	case 0:
		imx_iomux_v3_setup_multiple_pads(usb_otg1_pads,
						 ARRAY_SIZE(usb_otg1_pads));
		break;
	case 1:
		imx_iomux_v3_setup_multiple_pads(usb_otg2_pads,
						 ARRAY_SIZE(usb_otg2_pads));
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}
	return 0;
}
#endif

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	int button_pressed = 0;
	/* TODO recovery mode trigger function */

	return  button_pressed;
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
