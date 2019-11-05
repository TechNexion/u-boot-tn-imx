/*
 * Copyright (C) 2018 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *         Alvin chen <alvin.chen@technexion.com>
 *         Ray Chang <ray.chang@technexion.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/video.h>
#include <asm/mach-imx/iomux-v3.h>
#include <mxc_epdc_fb.h>
#include <mmc.h>
#include <spl.h>
#include <fsl_esdhc.h>
#include <linux/libfdt.h>

DECLARE_GLOBAL_DATA_PTR;

#define DDR_TYPE_DET			IMX_GPIO_NR(5, 1)

#ifdef CONFIG_SPL_BUILD

#ifdef CONFIG_MX6

/*
 * Driving strength:
 *   0x30 == 40 Ohm
 *   0x28 == 48 Ohm
 */
#define IMX6UL_DRIVE_STRENGTH		0x30
/* configure MX6UltraLite/UltraLiteLite mmdc DDR io registers */
static struct mx6ul_iomux_ddr_regs mx6ul_ddr_ioregs = {
	.dram_dqm0 = IMX6UL_DRIVE_STRENGTH,
	.dram_dqm1 = IMX6UL_DRIVE_STRENGTH,
	.dram_ras = IMX6UL_DRIVE_STRENGTH,
	.dram_cas = IMX6UL_DRIVE_STRENGTH,
	.dram_odt0 = IMX6UL_DRIVE_STRENGTH,
	.dram_odt1 = IMX6UL_DRIVE_STRENGTH,
	.dram_sdba2 = 0x00000000,
	.dram_sdclk_0 = 0x00000008,
	.dram_sdqs0 = 0x00000038,
	.dram_sdqs1 = IMX6UL_DRIVE_STRENGTH,
	.dram_reset = IMX6UL_DRIVE_STRENGTH,
};

static struct mx6ul_iomux_grp_regs mx6ul_grp_ioregs = {
	.grp_addds = IMX6UL_DRIVE_STRENGTH,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_b0ds = IMX6UL_DRIVE_STRENGTH,
	.grp_ctlds = IMX6UL_DRIVE_STRENGTH,
	.grp_b1ds = IMX6UL_DRIVE_STRENGTH,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ddr_type = 0x000c0000,
};

/*
 * calibration - these are the various CPU/DDR3 combinations we support
 */

static struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpdgctrl0 = 0x01400140,
	.p0_mprddlctl = 0x40404044,
	.p0_mpwrdlctl = 0x40405450,
};

struct mx6_ddr_sysinfo mx6ul_ddr_sysinfo = {
	.dsize = 0,
	.cs_density = 20,
	.ncs = 1,
	.cs1_mirror = 0,
	.rtt_wr = 2,
	.rtt_nom = 1,		/* RTT_Nom = RZQ/2 */
	.walat = 1,		/* Write additional latency */
	.ralat = 5,		/* Read additional latency */
	.mif3_mode = 3,		/* Command prediction working mode */
	.bi_on = 1,		/* Bank interleaving enabled */
	.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
	.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	.ddr_type = DDR_TYPE_DDR3,
};

/* 2Gb DDR3 256MB at 400MHz */
static struct mx6_ddr3_cfg ddr_2gb_800mhz = {
	.mem_speed = 800,
	.density = 2,
	.width = 16,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1500,
	.trcmin = 5250,
	.trasmin = 3750,
};

/* 4Gb DDR3 512MB at 400MHz */
static struct mx6_ddr3_cfg ddr_4gb_800mhz = {
	.mem_speed = 800,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1500,
	.trcmin = 5250,
	.trasmin = 3750,
};
#endif

static iomux_v3_cfg_t const ddr_type_detection_pads[] = {
	MX6_PAD_SNVS_TAMPER1__GPIO5_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_ddr_type_detection(void)
{
	SETUP_IOMUX_PADS(ddr_type_detection_pads);
}

/* clock control general registers */
static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	/* imx6ul */
	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

static void spl_dram_init(void)
{
	setup_iomux_ddr_type_detection();
	gpio_direction_input(DDR_TYPE_DET);

	if (gpio_get_value(DDR_TYPE_DET)) {
		mx6ul_dram_iocfg(ddr_4gb_800mhz.width, &mx6ul_ddr_ioregs, &mx6ul_grp_ioregs);
		mx6_dram_cfg(&mx6ul_ddr_sysinfo, &mx6_mmcd_calib, &ddr_4gb_800mhz);
	} else {
		mx6ul_dram_iocfg(ddr_2gb_800mhz.width, &mx6ul_ddr_ioregs, &mx6ul_grp_ioregs);
		mx6_dram_cfg(&mx6ul_ddr_sysinfo, &mx6_mmcd_calib, &ddr_2gb_800mhz);
	}
}

#ifdef MX6UL
/* General Purpose Registers for imx6ul */
void imx6ul_gpr_init(void)
{
	struct iomuxc_gpr_base_regs *gpr_regs = (struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;
	writel(0x4F400005, &gpr_regs->gpr[1]);
}
#endif

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();

#ifdef MX6UL
	imx6ul_gpr_init();
#else
	gpr_init();
#endif

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif /* CONFIG_SPL_BUILD */
