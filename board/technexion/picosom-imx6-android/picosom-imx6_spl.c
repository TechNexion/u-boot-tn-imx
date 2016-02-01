/*
 * Copyright (C) 2015 Technexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <spl.h>

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

/* H5T04G63AFR-PB for i.mx6Solo/DL operating DDR at 400MHz */
static struct mx6_ddr3_cfg h5t04g63afr_800mhz = {
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
	.rtt_nom	= 1,
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
	.rtt_nom	= 1,
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
	.rtt_nom	= 1,
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
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static void gpr_init(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* enable AXI cache for VDOA/VPU/IPU */
	writel(0xF00000CF, &iomux->gpr[4]);
	/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
	writel(0x007F007F, &iomux->gpr[6]);
	writel(0x007F007F, &iomux->gpr[7]);
}

/* Set up LPDDR2 timing by ourself */
static void spl_dram_init_lpddr2(void)
{
	/* Apollos dirty hack for setting 400MHz clocks */
	writel(0x60324, 0x020C4018);
	 /* i.MX6Q */
	/* DDR IO TYPE */
	writel(0x00080000, IOMUXC_BASE_ADDR + 0x798);
	writel(0x00000000, IOMUXC_BASE_ADDR + 0x758);
	/* Clock */
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x588);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x594);
	/* Address */
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x56c);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x578);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x74c);
	/* Control */
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x57c);//fix to apply for LPDDR2

	writel(0x00000000, IOMUXC_BASE_ADDR + 0x58c);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x59c);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x5a0);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x78c);
	/* Data Strobe */
	writel(0x00020000, IOMUXC_BASE_ADDR + 0x750);

	writel(0x00003028, IOMUXC_BASE_ADDR + 0x5a8);
	writel(0x00003028, IOMUXC_BASE_ADDR + 0x5b0);
	writel(0x00003028, IOMUXC_BASE_ADDR + 0x524);
	writel(0x00003028, IOMUXC_BASE_ADDR + 0x51c);
	writel(0x00003028, IOMUXC_BASE_ADDR + 0x518);
	writel(0x00003028, IOMUXC_BASE_ADDR + 0x50c);
	writel(0x00003028, IOMUXC_BASE_ADDR + 0x5b8);
	writel(0x00003028, IOMUXC_BASE_ADDR + 0x5c0);
	/* Data */
	writel(0x00020000, IOMUXC_BASE_ADDR + 0x774);

	writel(0x00000028, IOMUXC_BASE_ADDR + 0x784);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x788);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x794);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x79c);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x7a0);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x7a4);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x7a8);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x748);

	writel(0x00000028, IOMUXC_BASE_ADDR + 0x5ac);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x5b4);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x528);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x520);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x514);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x510);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x5bc);
	writel(0x00000028, IOMUXC_BASE_ADDR + 0x5c4);

	//=============================================================================
	// DDR Controller Registers
	//=============================================================================
	// Manufacturer:	Micron
	// Device Part Number:	MT42L64M64D2KH-18
	// Clock Freq.: 	400MHz
	// MMDC channels: 	MMDC0 & MMDC1
	// Density per CS in Gb:4
	// Chip Selects used:	1
	// Number of Banks:	8
	// Row address:    	14
	// Column address: 	10
	// Data bus width	2x32
	//=============================================================================

	writel(0x00008000, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x00008000, MMDC_P1_BASE_ADDR + 0x01c);
	writel(0x1B4700C7, MMDC_P0_BASE_ADDR + 0x85c);
	writel(0x1B4700C7, MMDC_P1_BASE_ADDR + 0x85c);

	/* Calibrations */
	/* ZQ */
	writel(0xA1390003, MMDC_P0_BASE_ADDR + 0x800);

	/* ca bus abs delay */
	writel(0x00400000, MMDC_P0_BASE_ADDR + 0x890);
	writel(0x00400000, MMDC_P1_BASE_ADDR + 0x890);

	/* Read calibration */
	writel(0x42424044, MMDC_P0_BASE_ADDR + 0x848);
	writel(0x4242404A, MMDC_P1_BASE_ADDR + 0x848);

	/* Write calibration */
	writel(0x3E383A34, MMDC_P0_BASE_ADDR + 0x850);
	writel(0x40323C3A, MMDC_P1_BASE_ADDR + 0x850);

	/* dqs gating dis */
	writel(0x20000000, MMDC_P0_BASE_ADDR + 0x83C);
	writel(0x00000000, MMDC_P0_BASE_ADDR + 0x840);
	writel(0x20000000, MMDC_P1_BASE_ADDR + 0x83C);
	writel(0x00000000, MMDC_P1_BASE_ADDR + 0x840);

	/* read data bit delay */
	writel(0x33333333, MMDC_P0_BASE_ADDR + 0x81C);
	writel(0x33333333, MMDC_P0_BASE_ADDR + 0x820);
	writel(0x33333333, MMDC_P0_BASE_ADDR + 0x824);
	writel(0x33333333, MMDC_P0_BASE_ADDR + 0x828);
	writel(0x33333333, MMDC_P1_BASE_ADDR + 0x81C);
	writel(0x33333333, MMDC_P1_BASE_ADDR + 0x820);
	writel(0x33333333, MMDC_P1_BASE_ADDR + 0x824);
	writel(0x33333333, MMDC_P1_BASE_ADDR + 0x828);

	/* write data bit delay */
	writel(0xF3333333, MMDC_P0_BASE_ADDR + 0x82C);
	writel(0xF3333333, MMDC_P0_BASE_ADDR + 0x830);
	writel(0xF3333333, MMDC_P0_BASE_ADDR + 0x834);
	writel(0xF3333333, MMDC_P0_BASE_ADDR + 0x838);
	writel(0xF3333333, MMDC_P1_BASE_ADDR + 0x82C);
	writel(0xF3333333, MMDC_P1_BASE_ADDR + 0x830);
	writel(0xF3333333, MMDC_P1_BASE_ADDR + 0x834);
	writel(0xF3333333, MMDC_P1_BASE_ADDR + 0x838);

	/* Complete calibration by forced measurement */
	writel(0x00000800, MMDC_P0_BASE_ADDR + 0x8b8);
	writel(0x00000800, MMDC_P1_BASE_ADDR + 0x8b8);


	//=============================================================================
	// Calibration setup end
	//=============================================================================
	/* Channel0 - startng address */
	writel(0x00020036, MMDC_P0_BASE_ADDR + 0x004);
	writel(0x00000000, MMDC_P0_BASE_ADDR + 0x008);
	writel(0x33374133, MMDC_P0_BASE_ADDR + 0x00c);
	writel(0x00100A82, MMDC_P0_BASE_ADDR + 0x010);
	writel(0x00000093, MMDC_P0_BASE_ADDR + 0x014);

	//MDMISC: RALAT kept to the high level of 5
	//MDMISC: consider reducing RALAT if your 528MHz board design allow that. Lower RALAT benefits:
	//a. better operation at low frequency, for LPDDR2 freq < 100MHz, change RALAT to 3
	//b. Small performence improvment
	writel(0x0000174C, MMDC_P0_BASE_ADDR + 0x018);
	writel(0x00008000, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x0F9F26D2, MMDC_P0_BASE_ADDR + 0x02c);
	writel(0x00000010, MMDC_P0_BASE_ADDR + 0x030);
	writel(0x00190778, MMDC_P0_BASE_ADDR + 0x038);
	writel(0x00000053, MMDC_P0_BASE_ADDR + 0x040);
	writel(0x11420000, MMDC_P0_BASE_ADDR + 0x400);
	writel(0x83110000, MMDC_P0_BASE_ADDR + 0x000);

	/* Channel1 - starting address */
	writel(0x00020036, MMDC_P1_BASE_ADDR + 0x004);
	writel(0x00000000, MMDC_P1_BASE_ADDR + 0x008);
	writel(0x33374133, MMDC_P1_BASE_ADDR + 0x00c);
	writel(0x00100A82, MMDC_P1_BASE_ADDR + 0x010);
	writel(0x00000093, MMDC_P1_BASE_ADDR + 0x014);

	//MDMISC: RALAT kept to the high level of 5
	//MDMISC: consider reducing RALAT if your 528MHz board design allow that. Lower RALAT benefits:
	//a. better operation at low frequency, for LPDDR2 freq < 100MHz, change RALAT to 3
	//b. Small performence improvment
	writel(0x0000174C, MMDC_P1_BASE_ADDR + 0x018);
	writel(0x00008000, MMDC_P1_BASE_ADDR + 0x01c);
	writel(0x0F9F26D2, MMDC_P1_BASE_ADDR + 0x02c);
	writel(0x00000010, MMDC_P1_BASE_ADDR + 0x030);
	writel(0x00190778, MMDC_P1_BASE_ADDR + 0x038);
	writel(0x00000013, MMDC_P1_BASE_ADDR + 0x040);
	writel(0x11420000, MMDC_P1_BASE_ADDR + 0x400);
	writel(0x83110000, MMDC_P1_BASE_ADDR + 0x000);

	// Channel0 : Configure DDR device:
	//CS0
	writel(0x003F8030, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0xFF0A8030, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x82018030, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x04028030, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x03038030, MMDC_P0_BASE_ADDR + 0x01c);

	// Channel1 : Configure DDR device:
	//CS0
	writel(0x003F8030, MMDC_P1_BASE_ADDR + 0x01c);
	writel(0xFF0A8030, MMDC_P1_BASE_ADDR + 0x01c);
	writel(0x82018030, MMDC_P1_BASE_ADDR + 0x01c);
	writel(0x04028030, MMDC_P1_BASE_ADDR + 0x01c);
	writel(0x03038030, MMDC_P1_BASE_ADDR + 0x01c);

	// DDR_PHY_P0_MPZQHWCTRL, enable both one-time & periodic HW ZQ calibration.
	writel(0xA1390003, MMDC_P0_BASE_ADDR + 0x800);

	writel(0x00001800, MMDC_P0_BASE_ADDR + 0x020);
	writel(0x00001800, MMDC_P1_BASE_ADDR + 0x020);

	writel(0x00000000, MMDC_P0_BASE_ADDR + 0x818);
	writel(0x00000000, MMDC_P1_BASE_ADDR + 0x818);

	writel(0x00025576, MMDC_P0_BASE_ADDR + 0x004);
	writel(0x00025576, MMDC_P1_BASE_ADDR + 0x004);

	writel(0x00011006, MMDC_P0_BASE_ADDR + 0x404);
	writel(0x00011006, MMDC_P1_BASE_ADDR + 0x404);

	writel(0x00000000, MMDC_P0_BASE_ADDR + 0x01c);
	writel(0x00000000, MMDC_P1_BASE_ADDR + 0x01c);
}



static void spl_dram_init(void)
{
	switch (get_cpu_type()) {
	case MXC_CPU_MX6SOLO:
		mx6sdl_dram_iocfg(32, &mx6sdl_ddr_ioregs, &mx6sdl_grp_ioregs);
		mx6_dram_cfg(&mem_s, &mx6s_512m_mmdc_calib, &h5tq2g63ffr_800mhz);
		break;
	case MXC_CPU_MX6DL:
		mx6sdl_dram_iocfg(32, &mx6sdl_ddr_ioregs, &mx6sdl_grp_ioregs);
		mx6_dram_cfg(&mem_s, &mx6dl_1g_mmdc_calib, &h5t04g63afr_800mhz);
		break;
	case MXC_CPU_MX6D:
	case MXC_CPU_MX6Q:
		spl_dram_init_lpddr2();
		break;
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
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

void reset_cpu(ulong addr)
{
}
#endif
