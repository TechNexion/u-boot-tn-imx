// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *         Ray Chang <ray.chang@technexion.com>
 *
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <spl.h>
#include <asm/mach-imx/sata.h>
#include <init.h>
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_SPL_BUILD)
#include <asm/arch/mx6-ddr.h>
/*
 * Driving strength:
 *   0x30 == 40 Ohm
 *   0x28 == 48 Ohm
 */

#define IMX6DQ_DRIVE_STRENGTH		0x30
#define IMX6SDL_DRIVE_STRENGTH		0x28
#define IMX6QP_DRIVE_STRENGTH		0x28

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* Break into full U-Boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

	return 0;
}
#endif

/* configure MX6Q/DUAL mmdc DDR io registers */
static struct mx6dq_iomux_ddr_regs mx6dq_ddr_ioregs = {
	.dram_sdclk_0 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdclk_1 = IMX6DQ_DRIVE_STRENGTH,
	.dram_cas = IMX6DQ_DRIVE_STRENGTH,
	.dram_ras = IMX6DQ_DRIVE_STRENGTH,
	.dram_reset = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdcke0 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdcke1 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdba2 = 0x00000000,
	.dram_sdodt0 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdodt1 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdqs0 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdqs1 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdqs2 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdqs3 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdqs4 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdqs5 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdqs6 = IMX6DQ_DRIVE_STRENGTH,
	.dram_sdqs7 = IMX6DQ_DRIVE_STRENGTH,
	.dram_dqm0 = IMX6DQ_DRIVE_STRENGTH,
	.dram_dqm1 = IMX6DQ_DRIVE_STRENGTH,
	.dram_dqm2 = IMX6DQ_DRIVE_STRENGTH,
	.dram_dqm3 = IMX6DQ_DRIVE_STRENGTH,
	.dram_dqm4 = IMX6DQ_DRIVE_STRENGTH,
	.dram_dqm5 = IMX6DQ_DRIVE_STRENGTH,
	.dram_dqm6 = IMX6DQ_DRIVE_STRENGTH,
	.dram_dqm7 = IMX6DQ_DRIVE_STRENGTH,
};

/* configure MX6Q/DUAL mmdc GRP io registers */
static struct mx6dq_iomux_grp_regs mx6dq_grp_ioregs = {
	.grp_ddr_type = 0x000c0000,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke = 0x00000000,
	.grp_addds = IMX6DQ_DRIVE_STRENGTH,
	.grp_ctlds = IMX6DQ_DRIVE_STRENGTH,
	.grp_ddrmode = 0x00020000,
	.grp_b0ds = IMX6DQ_DRIVE_STRENGTH,
	.grp_b1ds = IMX6DQ_DRIVE_STRENGTH,
	.grp_b2ds = IMX6DQ_DRIVE_STRENGTH,
	.grp_b3ds = IMX6DQ_DRIVE_STRENGTH,
	.grp_b4ds = IMX6DQ_DRIVE_STRENGTH,
	.grp_b5ds = IMX6DQ_DRIVE_STRENGTH,
	.grp_b6ds = IMX6DQ_DRIVE_STRENGTH,
	.grp_b7ds = IMX6DQ_DRIVE_STRENGTH,
};

/* configure MX6SOLO/DUALLITE mmdc DDR io registers */
struct mx6sdl_iomux_ddr_regs mx6sdl_ddr_ioregs = {
	.dram_sdclk_0 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdclk_1 = IMX6SDL_DRIVE_STRENGTH,
	.dram_cas = IMX6SDL_DRIVE_STRENGTH,
	.dram_ras = IMX6SDL_DRIVE_STRENGTH,
	.dram_reset = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdcke0 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdcke1 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdba2 = 0x00000000,
	.dram_sdodt0 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdodt1 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdqs0 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdqs1 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdqs2 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdqs3 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdqs4 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdqs5 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdqs6 = IMX6SDL_DRIVE_STRENGTH,
	.dram_sdqs7 = IMX6SDL_DRIVE_STRENGTH,
	.dram_dqm0 = IMX6SDL_DRIVE_STRENGTH,
	.dram_dqm1 = IMX6SDL_DRIVE_STRENGTH,
	.dram_dqm2 = IMX6SDL_DRIVE_STRENGTH,
	.dram_dqm3 = IMX6SDL_DRIVE_STRENGTH,
	.dram_dqm4 = IMX6SDL_DRIVE_STRENGTH,
	.dram_dqm5 = IMX6SDL_DRIVE_STRENGTH,
	.dram_dqm6 = IMX6SDL_DRIVE_STRENGTH,
	.dram_dqm7 = IMX6SDL_DRIVE_STRENGTH,
};

/* configure MX6SOLO/DUALLITE mmdc GRP io registers */
struct mx6sdl_iomux_grp_regs mx6sdl_grp_ioregs = {
	.grp_ddr_type = 0x000c0000,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke = 0x00000000,
	.grp_addds = IMX6SDL_DRIVE_STRENGTH,
	.grp_ctlds = IMX6SDL_DRIVE_STRENGTH,
	.grp_ddrmode = 0x00020000,
	.grp_b0ds = IMX6SDL_DRIVE_STRENGTH,
	.grp_b1ds = IMX6SDL_DRIVE_STRENGTH,
	.grp_b2ds = IMX6SDL_DRIVE_STRENGTH,
	.grp_b3ds = IMX6SDL_DRIVE_STRENGTH,
	.grp_b4ds = IMX6SDL_DRIVE_STRENGTH,
	.grp_b5ds = IMX6SDL_DRIVE_STRENGTH,
	.grp_b6ds = IMX6SDL_DRIVE_STRENGTH,
	.grp_b7ds = IMX6SDL_DRIVE_STRENGTH,
};

/* H5T04G63AFR-PB */
static struct mx6_ddr3_cfg h5t04g63afr = {
	.mem_speed = 1600,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

/* H5TQ2G63DFR-H9 */
static struct mx6_ddr3_cfg h5tq2g63dfr = {
	.mem_speed = 1333,
	.density = 2,
	.width = 16,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1350,
	.trcmin = 4950,
	.trasmin = 3600,
};

/* H5TQ2G63FFR-H9 for i.mx6Solo/DL operating DDR at 400MHz */
static struct mx6_ddr3_cfg h5tq2g63ffr_800mhz = {
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

/*
 * calibration - these are the various CPU/DDR3 combinations we support
 */

static struct mx6_mmdc_calibration mx6q_2g_mmdc_calib = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpwldectrl1 = 0x00000000,
	.p1_mpwldectrl0 = 0x00000000,
	.p1_mpwldectrl1 = 0x00000000,
	.p0_mpdgctrl0 = 0x032C0340,
	.p0_mpdgctrl1 = 0x03300324,
	.p1_mpdgctrl0 = 0x032C0338,
	.p1_mpdgctrl1 = 0x03300274,
	.p0_mprddlctl = 0x423A383E,
	.p1_mprddlctl = 0x3638323E,
	.p0_mpwrdlctl = 0x363C4640,
	.p1_mpwrdlctl = 0x4034423C,
};

/* DDR 64bit 2GB */
static struct mx6_ddr_sysinfo mem_q = {
	.dsize		= 2,
	.cs1_mirror	= 0,
	/* config for full 4GB range so that get_mem_size() works */
	.cs_density	= 32,
	.ncs		= 1,
	.bi_on		= 1,
	.rtt_nom	= 2,
	.rtt_wr		= 0,
	.ralat		= 5,
	.walat		= 0,
	.mif3_mode	= 3,
	.rst_to_cke	= 0x23,
	.sde_to_rst	= 0x10,
};

static struct mx6_mmdc_calibration mx6d_1g_mmdc_calib = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpwldectrl1 = 0x00000000,
	.p1_mpwldectrl0 = 0x00000000,
	.p1_mpwldectrl1 = 0x00000000,
	.p0_mpdgctrl0 = 0x03280334,
	.p0_mpdgctrl1 = 0x03280320,
	.p1_mpdgctrl0 = 0x0330033C,
	.p1_mpdgctrl1 = 0x033C0278,
	.p0_mprddlctl = 0x423A3A3E,
	.p1_mprddlctl = 0x3C3E3842,
	.p0_mpwrdlctl = 0x36384240,
	.p1_mpwrdlctl = 0x4A384440,
};

static struct mx6_mmdc_calibration mx6dl_1g_mmdc_calib = {
	.p0_mpwldectrl0 = 0x003E0047,
	.p0_mpwldectrl1 = 0x00300036,
	.p1_mpwldectrl0 = 0x0018001B,
	.p1_mpwldectrl1 = 0x0010002C,
	.p0_mpdgctrl0 = 0x02380234,
	.p0_mpdgctrl1 = 0x02300230,
	.p1_mpdgctrl0 = 0x0218021C,
	.p1_mpdgctrl1 = 0x0210020C,
	.p0_mprddlctl = 0x40464846,
	.p1_mprddlctl = 0x40444640,
	.p0_mpwrdlctl = 0x3A302E30,
	.p1_mpwrdlctl = 0x3630302C,
};

static struct mx6_mmdc_calibration mx6s_512m_mmdc_calib = {
	.p0_mpwldectrl0 = 0x0033003C,
	.p0_mpwldectrl1 = 0x002E0032,
	.p0_mpdgctrl0 = 0x02300224,
	.p0_mpdgctrl1 = 0x02140218,
	.p0_mprddlctl = 0x40444A48,
	.p0_mpwrdlctl = 0x38362E2E,
};

/* DDR 64bit 1GB */
static struct mx6_ddr_sysinfo mem_dl = {
	.dsize		= 2,
	.cs1_mirror	= 0,
	/* config for full 4GB range so that get_mem_size() works */
	.cs_density	= 32,
	.ncs		= 1,
	.bi_on		= 1,
	.rtt_nom	= 2,
	.rtt_wr		= 0,
	.ralat		= 5,
	.walat		= 0,
	.mif3_mode	= 3,
	.rst_to_cke	= 0x23,
	.sde_to_rst	= 0x10,
};

/* DDR 32bit 512MB */
static struct mx6_ddr_sysinfo mem_s = {
	.dsize		= 1,
	.cs1_mirror	= 0,
	/* config for full 4GB range so that get_mem_size() works */
	.cs_density	= 32,
	.ncs		= 1,
	.bi_on		= 1,
	.rtt_nom	= 2,
	.rtt_wr		= 0,
	.ralat		= 5,
	.walat		= 0,
	.mif3_mode	= 3,
	.rst_to_cke	= 0x23,
	.sde_to_rst	= 0x10,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000F3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static void spl_dram_init_imx6qp_lpddr3(void)
{
	 /* i.MX6QP */
	/* DDR IO TYPE */
	writel(0x000C0000, IOMUXC_BASE_ADDR + 0x798);
	writel(0x00000000, IOMUXC_BASE_ADDR + 0x758);
	/* Clock */
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x588);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x594);
	/* Address */
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x56c);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x578);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x74c);
	/* Control */
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x57c);//fix to apply for LPDDR2

	writel(0x00000000, IOMUXC_BASE_ADDR + 0x58c);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x59c);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x5a0);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x78c);
	/* Data Strobe */
	writel(0x00020000, IOMUXC_BASE_ADDR + 0x750);

	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x5a8);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x5b0);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x524);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x51c);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x518);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x50c);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x5b8);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x5c0);
	/* Data */
	writel(0x00020000, IOMUXC_BASE_ADDR + 0x774);

	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x784);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x788);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x794);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x79c);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x7a0);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x7a4);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x7a8);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x748);

	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x5ac);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x5b4);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x528);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x520);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x514);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x510);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x5bc);
	writel(IMX6QP_DRIVE_STRENGTH, IOMUXC_BASE_ADDR + 0x5c4);

	/* MMDC0_MDSCR
	   set the Configuration request bit during MMDC set up */
	writel(0x00008000, MMDC_P0_BASE_ADDR + 0x01c);

	/* Calibrations */
	/* ZQ */
	writel(0xa1390003, MMDC_P0_BASE_ADDR + 0x800);
	/* write leveling */
	writel(0x00060004, MMDC_P0_BASE_ADDR + 0x80c);
	writel(0x000B0004, MMDC_P0_BASE_ADDR + 0x810);
	writel(0x00000004, MMDC_P1_BASE_ADDR + 0x80c);
	writel(0x00000000, MMDC_P1_BASE_ADDR + 0x810);
	/* DQS gating, read delay, write delay calibration values
	 based on calibration compare of 0x00ffff00  */
	writel(0x03040314, MMDC_P0_BASE_ADDR + 0x83c);
	writel(0x03080300, MMDC_P0_BASE_ADDR + 0x840);
	writel(0x03000310, MMDC_P1_BASE_ADDR + 0x83c);
	writel(0x0268023C, MMDC_P1_BASE_ADDR + 0x840);

	writel(0x4034363A, MMDC_P0_BASE_ADDR + 0x848);
	writel(0x36302C3C, MMDC_P1_BASE_ADDR + 0x848);

	writel(0x3E3E4046, MMDC_P0_BASE_ADDR + 0x850);
	writel(0x483A4844, MMDC_P1_BASE_ADDR + 0x850);

	writel(0x33333333, MMDC_P0_BASE_ADDR + 0x81c);
	writel(0x33333333, MMDC_P0_BASE_ADDR + 0x820);
	writel(0x33333333, MMDC_P0_BASE_ADDR + 0x824);
	writel(0x33333333, MMDC_P0_BASE_ADDR + 0x828);
	writel(0x33333333, MMDC_P1_BASE_ADDR + 0x81c);
	writel(0x33333333, MMDC_P1_BASE_ADDR + 0x820);
	writel(0x33333333, MMDC_P1_BASE_ADDR + 0x824);
	writel(0x33333333, MMDC_P1_BASE_ADDR + 0x828);

	writel(0x24912489, MMDC_P0_BASE_ADDR + 0x8c0);
	writel(0x24914452, MMDC_P1_BASE_ADDR + 0x8c0);

	writel(0x00000800, MMDC_P0_BASE_ADDR + 0x8b8);
	writel(0x00000800, MMDC_P1_BASE_ADDR + 0x8b8);
	/* MMDC init:
	  in DDR3, 64-bit mode, only MMDC0 is initiated: */
	writel(0x00020036, MMDC_P0_BASE_ADDR + 0x004);
	writel(0x09444040, MMDC_P0_BASE_ADDR + 0x008);
	writel(0x898E79A4, MMDC_P0_BASE_ADDR + 0x00c);
	writel(0xDB538F64, MMDC_P0_BASE_ADDR + 0x010);
	writel(0x01FF00DD, MMDC_P0_BASE_ADDR + 0x014);

	writel(0x00011740, MMDC_P0_BASE_ADDR + 0x018);
	writel(0x00008000, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x000026D2, MMDC_P0_BASE_ADDR + 0x02c);
	writel(0x008E1023, MMDC_P0_BASE_ADDR + 0x030);
	writel(0x00000047, MMDC_P0_BASE_ADDR + 0x040);

	writel(0x14420000, MMDC_P0_BASE_ADDR + 0x400);
	writel(0x841A0000, MMDC_P0_BASE_ADDR + 0x000);
	writel(0x00400c58, MMDC_P0_BASE_ADDR + 0x890);

	/* add noc DDR configuration */
	writel(0x00000000, NOC_DDR_BASE_ADDR + 0x008);
	writel(0x2871C39B, NOC_DDR_BASE_ADDR + 0x00c);
	writel(0x000005B4, NOC_DDR_BASE_ADDR + 0x038);
	writel(0x00000040, NOC_DDR_BASE_ADDR + 0x014);
	writel(0x00000020, NOC_DDR_BASE_ADDR + 0x028);
	writel(0x00000020, NOC_DDR_BASE_ADDR + 0x02c);

	writel(0x02088032, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x00008033, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x00048031, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x19308030, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x04008040, MMDC_P0_BASE_ADDR + 0x01c);

	writel(0x00007800, MMDC_P0_BASE_ADDR + 0x020);
	writel(0x00022227, MMDC_P0_BASE_ADDR + 0x818);
	writel(0x00022227, MMDC_P1_BASE_ADDR + 0x818);
	writel(0x00025576, MMDC_P0_BASE_ADDR + 0x004);
	writel(0x00011006, MMDC_P0_BASE_ADDR + 0x404);
	writel(0x00000000, MMDC_P0_BASE_ADDR + 0x01c);
}

#ifdef CONFIG_IMX6_SPREAD_SPECTRUM
static void enable_spread_spectrum(void)
{
	asm volatile (
		"ldr r0, =#0x20C4000\n"			// base register CCM_BASE_ADDR
		"ldr r1, [r0,#0x18]\n"			// get the CCM_CMCMR
		"ldr r3, [r0,#0x18]\n"			// restore it later
		"and r1, r1, #0xffffcfff\n"		// set r1 to set periph_clk2_sel to "01 derive clock from osc_clk (pll1_ref_clk)"
		"orr r1, r1, #0x00001000\n"		//
		"str r1, [r0,#0x18]\n"			// the set it
		"ldr r1, [r0,#0x14]\n"			// load the CCM_CBCDR
		"ldr r2, [r0,#0x14]\n"			// restore it later
		"orr r1, r1, #0x02000000\n"		// set r1 to set periph_clk_sel to "1 derive clock from periph_clk2_clk clock source."
		"str r1, [r0,#0x14]\n"			// then set it

		"ldr r0, =0x020c8000\n"			//
		"ldr r1, [r0,#0x30]\n"			// get the CCM_ANALOG_PLL_SYS
		"orr r1, r1, #0x00010000\n"		// set to bypass
		"str r1, [r0,#0x30]\n"

		"orr r1, r1, #0x00011000\n"		// set to power down (the docs don't say if there needs to be a delay from the above)
		"str r1, [r0,#0x30]\n"

		"ldr r1, =0x00001770\n"			// set the denominator to 6000 dec
		"str r1, [r0,#0x60]\n"
		"ldr r1, =0x0bb88006\n"			// set the STOP t0 3000 (xbb8) and the STEP to 6 (x006)
										// giving: range=24M * 3000/6000 = 12M, step=24M * 6 / 6000 = 24khz
		"str r1, [r0,#0x40]\n"

		"ldr r1, [r0,#0x30]\n"

		"and r1, r1, #0xFFFFEFFF\n"		// power up
		"str r1, [r0,#0x30]\n"

		"and r1, r1, #0xFFFEFFFF\n"		// enable (the docs don't say if there needs to be a delay from the above)
		"str r1, [r0,#0x30]\n"

		"ldr r4, =0x0\n"
	);

	//udelay(10);
	asm volatile (
		"pu_delay:\n"				// wait until the pll is locked
		"ldr r1, [r0,#0xD0]\n"
		"tst r1, #0x80000000\n"
		"bne pu_delay\n"
	);


	asm volatile (
		"ldr r0, =#0x20C4000\n"			// restore and done
		"str r2, [r0,#0x14]\n"
		"str r3, [r0,#0x18]\n"
	);
}
#endif

static void spl_dram_init(void)
{
#ifdef CONFIG_IMX6_SPREAD_SPECTRUM
	enable_spread_spectrum();
#endif
	if (is_mx6dqp()) {
		spl_dram_init_imx6qp_lpddr3();
	} else {
		switch (get_cpu_type()) {
		case MXC_CPU_MX6SOLO:
			mx6sdl_dram_iocfg(32, &mx6sdl_ddr_ioregs, &mx6sdl_grp_ioregs);
			mx6_dram_cfg(&mem_s, &mx6s_512m_mmdc_calib, &h5tq2g63ffr_800mhz);
			break;
		case MXC_CPU_MX6DL:
			mx6sdl_dram_iocfg(64, &mx6sdl_ddr_ioregs, &mx6sdl_grp_ioregs);
			mx6_dram_cfg(&mem_dl, &mx6dl_1g_mmdc_calib, &h5tq2g63dfr);
			break;
		case MXC_CPU_MX6D:
			mx6sdl_dram_iocfg(64, &mx6sdl_ddr_ioregs, &mx6sdl_grp_ioregs);
			mx6_dram_cfg(&mem_dl, &mx6d_1g_mmdc_calib, &h5tq2g63dfr);
			break;
		case MXC_CPU_MX6Q:
			mx6dq_dram_iocfg(64, &mx6dq_ddr_ioregs, &mx6dq_grp_ioregs);
			mx6_dram_cfg(&mem_q, &mx6q_2g_mmdc_calib, &h5t04g63afr);
			break;
		}
	}
	udelay(100);
}

void board_init_f(ulong dummy)
{
	ccgr_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	gpr_init();

	/* iomux */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
#ifndef CONFIG_SILENT_CONSOLE
	preloader_console_init();
#endif
	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
