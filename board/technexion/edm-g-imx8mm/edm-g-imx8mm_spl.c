/*
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright 2020 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 */

#include <common.h>
#include <command.h>
#include <cpu_func.h>
#include <hang.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <spl.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/arch/ddr.h>

#include <power/pmic.h>
#include <power/pca9450.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc_imx.h>
#include <fsl_sec.h>
#include <mmc.h>
#include <linux/delay.h>
#include <asm/sections.h>

DECLARE_GLOBAL_DATA_PTR;

int spl_board_boot_device(enum boot_device boot_dev_spl)
{
	switch (boot_dev_spl) {
	case SD2_BOOT:
	case MMC2_BOOT:
		return BOOT_DEVICE_MMC1;
	case SD3_BOOT:
	case MMC3_BOOT:
		return BOOT_DEVICE_MMC2;
	case QSPI_BOOT:
		return BOOT_DEVICE_NOR;
	case NAND_BOOT:
		return BOOT_DEVICE_NAND;
	case USB_BOOT:
		return BOOT_DEVICE_BOARD;
	default:
		return BOOT_DEVICE_NONE;
	}
}

#define VER_DET_GPIO_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_DSE1)

static iomux_v3_cfg_t const ver_det_pads[] = {
	IMX8MM_PAD_NAND_DATA00_GPIO3_IO6 | MUX_PAD_CTRL(VER_DET_GPIO_PAD_CTRL), /* BOARD ID0 */
	IMX8MM_PAD_NAND_DATA02_GPIO3_IO8 | MUX_PAD_CTRL(VER_DET_GPIO_PAD_CTRL), /* BOARD ID1 */
};

#define BOARD_ID0		IMX_GPIO_NR(3, 6)
#define BOARD_ID1		IMX_GPIO_NR(3, 8)

static void setup_iomux_ver_det(void)
{
	imx_iomux_v3_setup_multiple_pads(ver_det_pads, ARRAY_SIZE(ver_det_pads));

	gpio_request(BOARD_ID0, "board_id0");
	gpio_direction_input(BOARD_ID0);
	gpio_request(BOARD_ID1, "board_id1");
	gpio_direction_input(BOARD_ID1);
}

/***********************************************
BOARD_ID0    BOARD_ID1
   1            0       8G LPDDR4
   1            1       4G LPDDR4
   0            1       2G LPDDR4
   0            0       1G LPDDR4
************************************************/

void spl_dram_init(void)
{
	setup_iomux_ver_det();

	/*************************************************
	ToDo: It's a dirty workaround to store the
	information of DDR size into start address of TCM.
	U-boot would extract this information in dram_init().
	**************************************************/

	if (gpio_get_value(BOARD_ID0) && !gpio_get_value(BOARD_ID1)) {
		puts("dram_init: LPDDR4 8GB\n");
		ddr_init(&dram_timing_8gb);
		writel(0x5, MCU_BOOTROM_BASE_ADDR);
	}
	else if (gpio_get_value(BOARD_ID0) && gpio_get_value(BOARD_ID1)) {
		puts("dram_init: LPDDR4 4GB\n");
		ddr_init(&dram_timing_4gb);
		writel(0x4, MCU_BOOTROM_BASE_ADDR);
	}
	else if (!gpio_get_value(BOARD_ID0) && gpio_get_value(BOARD_ID1)) {
		puts("dram_init: LPDDR4: 2GB\n");
		ddr_init(&dram_timing_2gb);
		writel(0x2, MCU_BOOTROM_BASE_ADDR);
	}
	else if (!gpio_get_value(BOARD_ID0) && !gpio_get_value(BOARD_ID1)) {
		puts("dram_init: LPDDR4: 1GB\n");
		ddr_init(&dram_timing_1gb);
		writel(0x1, MCU_BOOTROM_BASE_ADDR);
	}
	else
		puts("Unknown DDR type!!!\n");
}

#define I2C_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)
#define PC	MUX_PAD_CTRL(I2C_PAD_CTRL)

struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C1_SCL_I2C1_SCL | PC,
		.gpio_mode = IMX8MM_PAD_I2C1_SCL_GPIO5_IO14 | PC,
		.gp = IMX_GPIO_NR(5, 14),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C1_SDA_I2C1_SDA | PC,
		.gpio_mode = IMX8MM_PAD_I2C1_SDA_GPIO5_IO15 | PC,
		.gp = IMX_GPIO_NR(5, 15),
	},
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 12)
#define USDHC2_PWR_GPIO IMX_GPIO_NR(2, 19)

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE |PAD_CTL_PE | \
			 PAD_CTL_FSEL2)
#define USDHC_GPIO_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_DSE1)

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IMX8MM_PAD_NAND_WE_B_USDHC3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_WP_B_USDHC3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA04_USDHC3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA05_USDHC3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA06_USDHC3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA07_USDHC3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_RE_B_USDHC3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_CE2_B_USDHC3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_CE3_B_USDHC3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_CLE_USDHC3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	IMX8MM_PAD_SD2_CLK_USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_CMD_USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA0_USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA1_USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA2_USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA3_USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_RESET_B_GPIO2_IO19 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_cd_pad =
	IMX8MM_PAD_SD2_CD_B_GPIO2_IO12 | MUX_PAD_CTRL(USDHC_GPIO_PAD_CTRL);

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR, 0, 4},
	{USDHC3_BASE_ADDR, 0, 8},
};

int board_mmc_init(struct bd_info *bis)
{
	int i, ret;
	/*
		* According to the board_mmc_init() the following map is done:
		* (U-Boot device node)    (Physical Port)
		* mmc0                    USDHC2
		* mmc1                    USDHC3
		*/
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			gpio_request(USDHC2_PWR_GPIO, "usdhc2_reset");
			gpio_direction_output(USDHC2_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC2_PWR_GPIO, 1);
			break;
		case 1:
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = 1;
		break;
	case USDHC2_BASE_ADDR:
		imx_iomux_v3_setup_pad(usdhc2_cd_pad);
		gpio_request(USDHC2_CD_GPIO, "usdhc2 cd");
		gpio_direction_input(USDHC2_CD_GPIO);

		ret = !gpio_get_value(USDHC2_CD_GPIO);

		return ret;
	}

	return 1;
}

#ifdef CONFIG_POWER
#define I2C_PMIC	0
int power_init_board(void)
{
	struct pmic *p;
	int ret;

	ret = power_pca9450_init(I2C_PMIC, 0x35);
	if (ret)
		printf("power init failed");
	p = pmic_get("PCA9450");
	pmic_probe(p);

	/* BUCKxOUT_DVS0/1 control BUCK123 output */
	pmic_reg_write(p, PCA9450_BUCK123_DVS, 0x29);
	/* Buck 1 DVS control through PMIC_STBY_REQ */
	pmic_reg_write(p, PCA9450_BUCK1CTRL, 0x59);

	/* decrease RESET key long push time from the default 10s to 10ms */
	/* Ton_Deb of PCA9450 is 20ms and don't change */
	/* increase VDD_SOC to typical value 0.85v before first DRAM access */
	/* pmic_reg_write(p, PCA9450_BUCK1OUT_DVS0, 0x14); */
	pmic_reg_write(p, PCA9450_BUCK1OUT_DVS0, 0x14);
	pmic_reg_write(p, PCA9450_BUCK1OUT_DVS1, 0x10);

	/* increase VDD_DRAM to 0.975v for 3Ghz DDR -> 0.95V instead of 0.975V, */
	/* because PCA9450 Buck3 can set 0.95V */
	/* Also, set B3_ENMODE=2 (ON by PMIC_ON_REQ=H & PMIC_STBY_REQ=L) */
	pmic_reg_write(p, PCA9450_BUCK3OUT_DVS0, 0x1C);
	pmic_reg_write(p, PCA9450_BUCK3CTRL, 0x4A);

	/* set VDD_SNVS_0V8 from default 0.85V */
	pmic_reg_write(p, PCA9450_LDO2CTRL, 0xC0);

	/* set WDOG_B_CFG to 10b=Cold Reset, except LDO1/2 */
	pmic_reg_write(p, PCA9450_RESET_CTRL, 0xA1);

	/* enable level translator, forced enable */
	pmic_reg_write(p, PCA9450_CONFIG2, 0x3);

	return 0;
}
#endif

void spl_board_init(void)
{
#ifndef CONFIG_SPL_USB_SDP_SUPPORT
	/* Serial download mode */
	if (is_usb_boot()) {
		puts("Back to ROM, SDP\n");
		restore_boot_params();
	}
#endif
	puts("Normal Boot\n");
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	int ret;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	board_early_init_f();

	timer_init();

	preloader_console_init();

	ret = spl_init();
	if (ret) {
		debug("spl_init() failed: %d\n", ret);
		hang();
	}

	enable_tzc380();

#ifdef CONFIG_POWER
	/* Adjust pmic voltage to 1.0V for 800M */
	setup_i2c(I2C_PMIC, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

	power_init_board();
#endif

	/* DDR initialization */
	spl_dram_init();

	board_init_r(NULL, 0);
}
