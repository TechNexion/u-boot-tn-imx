/*
 * Copyright 2020 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <cpu_func.h>
#include <hang.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/ddr.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include <spl.h>
#include <linux/delay.h>
#include <power/pmic.h>
#include <init.h>
#include <fsl_sec.h>

DECLARE_GLOBAL_DATA_PTR;

extern struct dram_timing_info dram_timing_2gb_b1;
extern struct dram_timing_info dram_timing_2gb;

#define DDR_DET_1       IMX_GPIO_NR(3, 11)
#define DDR_DET_2       IMX_GPIO_NR(3, 12)
#define DDR_DET_3       IMX_GPIO_NR(3, 13)

static iomux_v3_cfg_t const ver_det_pads[] = {
    IMX8MQ_PAD_NAND_DATA01__GPIO3_IO7 | MUX_PAD_CTRL(NO_PAD_CTRL),
    IMX8MQ_PAD_NAND_DATA02__GPIO3_IO8 | MUX_PAD_CTRL(NO_PAD_CTRL),
    IMX8MQ_PAD_NAND_DATA03__GPIO3_IO9 | MUX_PAD_CTRL(NO_PAD_CTRL),
    IMX8MQ_PAD_NAND_DATA04__GPIO3_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
    IMX8MQ_PAD_NAND_DATA05__GPIO3_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
    IMX8MQ_PAD_NAND_DATA06__GPIO3_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),
    IMX8MQ_PAD_NAND_DATA07__GPIO3_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
void setup_iomux_ver_det(void)
{
    imx_iomux_v3_setup_multiple_pads(ver_det_pads, ARRAY_SIZE(ver_det_pads));

    gpio_request(DDR_DET_1, "ddr_det_1");
    gpio_direction_input(DDR_DET_1);
    gpio_request(DDR_DET_2, "ddr_det_2");
    gpio_direction_input(DDR_DET_2);
    gpio_request(DDR_DET_3, "ddr_det_3");
    gpio_direction_input(DDR_DET_3);
}

void spl_dram_init(void)
{
	setup_iomux_ver_det();

	/*************************************************
	ToDo: It's a dirty workaround to store the
	information of DDR size into start address of TCM.
	U-boot would extract this information in dram_init().
	**************************************************/

	if (!gpio_get_value(DDR_DET_1) && !gpio_get_value(DDR_DET_2) && gpio_get_value(DDR_DET_3)) {
		puts("dram_init: LPDDR4 4GB\n");
		if (soc_rev() >= CHIP_REV_2_1) {
			/* Silicon rev B1. Overwrite DDRC_CFG for 4GB LPDDR4*/
			dram_timing_2gb_b1.ddrc_cfg[2].val = 0xa3080020;
			dram_timing_2gb_b1.ddrc_cfg[38].val = 0x17;
			/* ddr_fsp0_cfg */
			dram_timing_2gb_b1.fsp_msg[0].fsp_cfg[8].val = 0x310;
			dram_timing_2gb_b1.fsp_msg[0].fsp_cfg[20].val = 0x3;
			/* ddr_fsp1_cfg */
			dram_timing_2gb_b1.fsp_msg[1].fsp_cfg[9].val = 0x310;
			dram_timing_2gb_b1.fsp_msg[1].fsp_cfg[21].val = 0x3;
			/* ddr_fsp2_cfg */
			dram_timing_2gb_b1.fsp_msg[2].fsp_cfg[9].val = 0x310;
			dram_timing_2gb_b1.fsp_msg[2].fsp_cfg[21].val = 0x3;
			/* ddr_fsp0_2d_cfg */
			dram_timing_2gb_b1.fsp_msg[3].fsp_cfg[11].val = 0x310;
			dram_timing_2gb_b1.fsp_msg[3].fsp_cfg[23].val = 0x3;
			ddr_init(&dram_timing_2gb_b1);
		} else {
			/* Silicon rev B0. Overwrite DDRC_CFG for 4GB LPDDR4*/
			dram_timing_2gb.ddrc_cfg[2].val = 0xa3080020;
			dram_timing_2gb.ddrc_cfg[38].val = 0x17;
			/* ddr_fsp0_cfg */
			dram_timing_2gb.fsp_msg[0].fsp_cfg[9].val = 0x310;
			dram_timing_2gb.fsp_msg[0].fsp_cfg[21].val = 0x3;
			/* ddr_fsp1_cfg */
			dram_timing_2gb.fsp_msg[1].fsp_cfg[10].val = 0x310;
			dram_timing_2gb.fsp_msg[1].fsp_cfg[22].val = 0x3;
			/* ddr_fsp0_2d_cfg */
			dram_timing_2gb.fsp_msg[2].fsp_cfg[10].val = 0x310;
			dram_timing_2gb.fsp_msg[2].fsp_cfg[22].val = 0x3;
			ddr_init(&dram_timing_2gb);
		}
		writel(0x4, MCU_BOOTROM_BASE_ADDR);
	} else if (gpio_get_value(DDR_DET_1) && gpio_get_value(DDR_DET_2) && !gpio_get_value(DDR_DET_3)) {
		puts("dram_init: LPDDR4 2GB\n");
		if (soc_rev() >= CHIP_REV_2_1) {
			/* Silicon rev B1 */
			ddr_init(&dram_timing_2gb_b1);
		} else {
			/* Silicon rev B0 */
			ddr_init(&dram_timing_2gb);
		}
		writel(0x2, MCU_BOOTROM_BASE_ADDR);
	} else if (gpio_get_value(DDR_DET_1) && !gpio_get_value(DDR_DET_2) && gpio_get_value(DDR_DET_3)) {
		puts("dram_init: LPDDR4 1GB\n");
		if (soc_rev() >= CHIP_REV_2_1) {
			/* Silicon rev B1. Overwrite DDRC_CFG for 1GB LPDDR4*/
			dram_timing_2gb_b1.ddrc_cfg[6].val = 0x300090;
			dram_timing_2gb_b1.ddrc_cfg[22].val = 0x96;
			dram_timing_2gb_b1.ddrc_cfg[43].val = 0xf070707;
			dram_timing_2gb_b1.ddrc_cfg[47].val = 0x60012;
			dram_timing_2gb_b1.ddrc_cfg[62].val = 0x13;
			dram_timing_2gb_b1.ddrc_cfg[71].val = 0x30005;
			dram_timing_2gb_b1.ddrc_cfg[86].val = 0x5;
			ddr_init(&dram_timing_2gb_b1);
		} else {
			/* Silicon rev B0. Overwrite DDRC_CFG for 1GB LPDDR4*/
			dram_timing_2gb.ddrc_cfg[6].val = 0x610090;
			dram_timing_2gb.ddrc_cfg[22].val = 0x96;
			dram_timing_2gb.ddrc_cfg[43].val = 0xf070707;
			dram_timing_2gb.ddrc_cfg[47].val = 0x14001f;
			dram_timing_2gb.ddrc_cfg[62].val = 0x20;
			ddr_init(&dram_timing_2gb);
		}
		writel(0x1, MCU_BOOTROM_BASE_ADDR);
	} else
		puts("Unknown DDR type!!!\n");
}

#define I2C_PAD_CTRL    (PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
struct i2c_pads_info i2c_pad_info1 = {
    .scl = {
        .i2c_mode = IMX8MQ_PAD_I2C1_SCL__I2C1_SCL | PC,
        .gpio_mode = IMX8MQ_PAD_I2C1_SCL__GPIO5_IO14 | PC,
        .gp = IMX_GPIO_NR(5, 14),
    },
    .sda = {
        .i2c_mode = IMX8MQ_PAD_I2C1_SDA__I2C1_SDA | PC,
        .gpio_mode = IMX8MQ_PAD_I2C1_SDA__GPIO5_IO15 | PC,
        .gp = IMX_GPIO_NR(5, 15),
    },
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 12)
#define USDHC1_PWR_GPIO IMX_GPIO_NR(2, 10)
#define USDHC2_PWR_GPIO IMX_GPIO_NR(2, 19)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = 1;
		break;
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		return ret;
	}

	return 1;
}

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | \
			 PAD_CTL_FSEL2)
#define USDHC_GPIO_PAD_CTRL (PAD_CTL_PUE | PAD_CTL_DSE1)

static iomux_v3_cfg_t const usdhc1_pads[] = {
	IMX8MQ_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA4__USDHC1_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA5__USDHC1_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA6__USDHC1_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA7__USDHC1_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_RESET_B__GPIO2_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	IMX8MQ_PAD_SD2_CLK__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* 0xd6 */
	IMX8MQ_PAD_SD2_CMD__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* 0xd6 */
	IMX8MQ_PAD_SD2_DATA0__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* 0xd6 */
	IMX8MQ_PAD_SD2_DATA1__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* 0xd6 */
	IMX8MQ_PAD_SD2_DATA2__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* 0x16 */
	IMX8MQ_PAD_SD2_DATA3__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* 0xd6 */
	IMX8MQ_PAD_SD2_CD_B__GPIO2_IO12 | MUX_PAD_CTRL(USDHC_GPIO_PAD_CTRL),
	IMX8MQ_PAD_SD2_RESET_B__GPIO2_IO19 | MUX_PAD_CTRL(USDHC_GPIO_PAD_CTRL),
};

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR, 0, 8},
	{USDHC2_BASE_ADDR, 0, 4},
};

int board_mmc_init(struct bd_info *bis)
{
	int i, ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc1                    USDHC2
	 */
	for (i = 0; i < CFG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			init_clk_usdhc(0);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			imx_iomux_v3_setup_multiple_pads(usdhc1_pads,
							 ARRAY_SIZE(usdhc1_pads));
			gpio_request(USDHC1_PWR_GPIO, "usdhc1_reset");
			gpio_direction_output(USDHC1_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC1_PWR_GPIO, 1);
			break;
		case 1:
			init_clk_usdhc(1);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			imx_iomux_v3_setup_multiple_pads(usdhc2_pads,
							 ARRAY_SIZE(usdhc2_pads));
			gpio_request(USDHC2_PWR_GPIO, "usdhc2_reset");
			gpio_direction_output(USDHC2_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC2_PWR_GPIO, 1);
			break;
		default:
			printf("Warning: you configured more USDHC controllers(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}

#ifdef CONFIG_POWER
#define I2C_PMIC	0
int power_init_board(void)
{
    struct pmic *p;
    int ret;

    ret = power_bd71837_init(I2C_PMIC);
    if (ret)
        printf("power init failed");

    p = pmic_get("BD71837");
    pmic_probe(p);

    /* decrease RESET key long push time from the default 10s to 10ms */
    pmic_reg_write(p, BD718XX_PWRONCONFIG1, 0x0);

    /* unlock the PMIC regs */
    pmic_reg_write(p, BD718XX_REGLOCK, 0x1);

#ifndef CONFIG_IMX8M_LPDDR4
    /* increase NVCC_DRAM_1V2 to 1.2v for DDR4 */
    pmic_reg_write(p, BD718XX_4TH_NODVS_BUCK_VOLT, 0x28);
#endif

	/* disable GPU and VPU power in uboot */
	pmic_reg_write(p, BD71837_BUCK3_CTRL, 0x46);
	pmic_reg_write(p, BD71837_BUCK4_CTRL, 0x46);

    /* lock the PMIC regs */
    pmic_reg_write(p, BD718XX_REGLOCK, 0x11);

	puts("Found PMIC\n");
    return 0;
}
#endif

void spl_board_init(void)
{

#ifdef CONFIG_FSL_CAAM
	if (sec_init()) {
		printf("\nsec_init failed!\n");
	}
#endif


#ifndef CONFIG_SPL_USB_SDP_SUPPORT
	/* Serial download mode */
	if (is_usb_boot()) {
		puts("Back to ROM, SDP\n");
		restore_boot_params();
	}
#endif

	init_usb_clk();

	puts("Normal Boot\n");
}

#ifdef CONFIG_SPL_LOAD_FIT
#define PCA9555_23_I2C_ADDR 0x23
#define PCA9555_26_I2C_ADDR 0x26
#define EXPANSION_IC_I2C_BUS 2
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = IMX8MQ_PAD_I2C3_SCL__I2C3_SCL | PC,
		.gpio_mode = IMX8MQ_PAD_I2C3_SCL__GPIO5_IO18 | PC,
		.gp = IMX_GPIO_NR(5, 18),
	},
	.sda = {
		.i2c_mode = IMX8MQ_PAD_I2C3_SDA__I2C3_SDA | PC,
		.gpio_mode = IMX8MQ_PAD_I2C3_SDA__GPIO5_IO19 | PC,
		.gp = IMX_GPIO_NR(5, 19),
	},
};

int board_fit_config_name_match(const char *name)
{
	i2c_set_bus_num(EXPANSION_IC_I2C_BUS);

	if (!i2c_probe(PCA9555_23_I2C_ADDR) && !i2c_probe(PCA9555_26_I2C_ADDR)) {
		if (!strcmp(name, "imx8mq-pico-wizard"))
			return 0;
	} else {
		if (!strcmp(name, "imx8mq-pico-pi"))
			return 0;
	}

	return -EINVAL;
}
#endif

void board_init_f(ulong dummy)
{
	int ret;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	init_uart_clk(0);

	board_early_init_f();

	timer_init();

	preloader_console_init();

	ret = spl_init();
	if (ret) {
		debug("spl_init() failed: %d\n", ret);
		hang();
	}

	enable_tzc380();

	/* Adjust pmic voltage to 1.0V for 800M */
	setup_i2c(I2C_PMIC, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);


	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	power_init_board();

	/* DDR initialization */
	spl_dram_init();

	board_init_r(NULL, 0);
}
