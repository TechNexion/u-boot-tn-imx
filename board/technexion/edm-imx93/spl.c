// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#include <init.h>
#include <power/pmic.h>
#include <power/pca9450.h>
#include <power/pf0900.h>
#include <spl.h>
#include <asm/global_data.h>
#include <asm/sections.h>
#include <asm/arch/clock.h>
#include <asm/arch/ddr.h>
#include <asm/arch/mu.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/trdc.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/arch-imx9/bbsm.h>
#include <asm/mach-imx/ele_api.h>

DECLARE_GLOBAL_DATA_PTR;

int spl_board_boot_device(enum boot_device boot_dev_spl)
{
#ifdef CONFIG_SPL_BOOTROM_SUPPORT
	return BOOT_DEVICE_BOOTROM;
#else
	switch (boot_dev_spl) {
	case SD1_BOOT:
	case MMC1_BOOT:
		return BOOT_DEVICE_MMC1;
	case SD2_BOOT:
	case MMC2_BOOT:
		return BOOT_DEVICE_MMC2;
	default:
		return BOOT_DEVICE_NONE;
	}
#endif
}

void spl_board_init(void)
{
	int ret;

	ret = ele_start_rng();
	if (ret)
		printf("Fail to start RNG: %d\n", ret);

#ifdef CONFIG_SPL_IMX_BBSM
	ret = bbsm_tamper_detect_enable();
	if (ret)
		printf("Failed to enable BBSM Tamper Detection: %d\n", ret);
#endif
	puts("Normal Boot\n");
}

extern struct dram_timing_info dram_timing_1866mts;
void spl_dram_init(void)
{
	struct dram_timing_info *ptiming = &dram_timing;

	if (is_voltage_mode(VOLT_LOW_DRIVE))
		ptiming = &dram_timing_1866mts;

	printf("DDR: %uMTS\n", ptiming->fsp_msg[0].drate);
	ddr_init(ptiming);
}

#if CONFIG_IS_ENABLED(DM_PMIC_PCA9450)
int power_init_board(void)
{
	struct udevice *dev;
	int ret;
	unsigned int buck_val;

	ret = pmic_get("pmic@25", &dev);
	if (ret != 0) {
		puts("ERROR: Get PMIC PCA9451A failed!\n");
		return ret;
	}
	puts("PMIC: PCA9451A\n");
	/* BUCKxOUT_DVS0/1 control BUCK123 output */
	pmic_reg_write(dev, PCA9450_BUCK123_DVS, 0x29);

	/* enable DVS control through PMIC_STBY_REQ */
	pmic_reg_write(dev, PCA9450_BUCK1CTRL, 0x59);

	if (is_voltage_mode(VOLT_LOW_DRIVE)) {
		buck_val = 0x0c; /* 0.8v for Low drive mode */
		printf("PMIC: Low Drive Voltage Mode\n");
	} else if (is_voltage_mode(VOLT_NOMINAL_DRIVE)) {
		buck_val = 0x10; /* 0.85v for Nominal drive mode */
		printf("PMIC: Nominal Voltage Mode\n");
	} else {
		buck_val = 0x14; /* 0.9v for Over drive mode */
		printf("PMIC: Over Drive Voltage Mode\n");
	}

	ele_volt_change_start_req();

	pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS0, buck_val);
	pmic_reg_write(dev, PCA9450_BUCK3OUT_DVS0, buck_val);

	ele_volt_change_finish_req();

	/* set standby voltage to 0.65v */
	pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS1, 0x0);

	/* I2C_LT_EN*/
	pmic_reg_write(dev, 0xa, 0x3);
	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	int ret;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	timer_init();

	arch_cpu_init();

	spl_early_init();

	preloader_console_init();

	ret = imx9_probe_mu();
	if (ret) {
		printf("Fail to init ELE API\n");
	} else {
		debug("SOC: 0x%x\n", gd->arch.soc_rev);
		debug("LC: 0x%x\n", gd->arch.lifecycle);
	}

	clock_init_late();

	power_init_board();

	if (!is_voltage_mode(VOLT_LOW_DRIVE))
		set_arm_core_max_clk();

	/* Init power of mix */
	soc_power_init();

	/* Setup TRDC for DDR access */
	trdc_init();

	/* DDR initialization */
	spl_dram_init();

	/* Put M33 into CPUWAIT for following kick */
	ret = m33_prepare();
	if (!ret)
		printf("M33 prepare ok\n");

	board_init_r(NULL, 0);
}

#ifdef CONFIG_ANDROID_SUPPORT
int board_get_emmc_id(void) {
	return 0;
}
#endif
