// SPDX-License-Identifier: GPL-2.0+

#include <cpu_func.h>
#include <hang.h>
#include <init.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6ul_pins.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/sections.h>
#include <fsl_esdhc_imx.h>
#include <linux/libfdt.h>
#include <spl.h>

#if defined(CONFIG_XPL_BUILD)

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* Break into full U-Boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

	return 0;
}
#endif

#include <asm/arch/mx6-ddr.h>

#define DDR_TYPE_DET	IMX_GPIO_NR(5, 1)

static iomux_v3_cfg_t const ddr_type_detection_pads[] = {
	MX6_PAD_SNVS_TAMPER1__GPIO5_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static struct mx6ul_iomux_grp_regs mx6_grp_ioregs = {
	.grp_addds = 0x00000030,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_ctlds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ddr_type = 0x00080000,
};

static struct mx6ul_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000030,
	.dram_dqm1 = 0x00000030,
	.dram_ras = 0x00000030,
	.dram_cas = 0x00000030,
	.dram_odt0 = 0x00000030,
	.dram_odt1 = 0x00000030,
	.dram_sdba2 = 0x00000000,
	.dram_sdclk_0 = 0x00000030,
	.dram_sdqs0 = 0x00000030,
	.dram_sdqs1 = 0x00000030,
	.dram_reset = 0x00000030,
};

static struct mx6_mmdc_calibration mx6_mmcd_calib_256mb = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpdgctrl0 = 0x01380134,
	.p0_mprddlctl = 0x40404244,
	.p0_mpwrdlctl = 0x40405050,
};

static struct mx6_mmdc_calibration mx6_mmcd_calib_512mb = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpdgctrl0 = 0x41440140,
	.p0_mprddlctl = 0x40404246,
	.p0_mpwrdlctl = 0x40405048,
};

static struct mx6_ddr_sysinfo ddr_sysinfo = {
	.dsize		= 0,
	.cs1_mirror	= 0,
	.ncs		= 1,
	.bi_on		= 1,
	.rtt_nom	= 1,
	.rtt_wr		= 0,
	.ralat		= 5,
	.walat		= 0,
	.mif3_mode	= 3,
	.rst_to_cke	= 0x23,
	.sde_to_rst	= 0x10,
	.pd_fast_exit	= 1,
	.refsel = 0,
	.refr = 1,
};

static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 1333,
	.density = 4,
	.width = 16,
	.banks = 8,
	.coladdr = 10,
	.pagesz = 1,
	.trcd = 1350,
	.trcmin = 4950,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

static void imx6ul_spl_dram_cfg_size(u32 ram_size)
{
	const struct mx6_mmdc_calibration *calib;
	struct mmdc_p_regs *mmdc0 = (struct mmdc_p_regs *)MMDC_P0_BASE_ADDR;

	if (ram_size == SZ_256M) {
		mem_ddr.rowaddr = 14;
		ddr_sysinfo.cs_density = 18;
		calib = &mx6_mmcd_calib_256mb;
	} else {
		mem_ddr.rowaddr = 15;
		ddr_sysinfo.cs_density = 20;
		calib = &mx6_mmcd_calib_512mb;
	}

	mx6ul_dram_iocfg(mem_ddr.width, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&ddr_sysinfo, calib, &mem_ddr);

	/* Preserve the board-specific values that the DDR API cannot express. */
	writel(1 << 15, &mmdc0->mdscr);
	if (ram_size == SZ_256M) {
		writel(0x00333030, &mmdc0->mdotc);
		writel(0x0002552D, &mmdc0->mdpdc);
	} else {
		writel(0x1B333030, &mmdc0->mdotc);
		writel(0xB66D0B63, &mmdc0->mdcfg1);
	}
	writel(0x00201740, &mmdc0->mdmisc);
	writel(0x000026D2, &mmdc0->mdrwd);
	writel(0x00000227, &mmdc0->mpodtctrl);
	writel(0x00011006, &mmdc0->mapsr);
	writel(0, &mmdc0->mdscr);
}

static void imx6ul_spl_dram_cfg(void)
{
	ulong ram_size, ram_size_test;

	imx_iomux_v3_setup_multiple_pads(ddr_type_detection_pads,
					 ARRAY_SIZE(ddr_type_detection_pads));
	gpio_direction_input(DDR_TYPE_DET);
	ram_size = gpio_get_value(DDR_TYPE_DET) ? SZ_256M : SZ_512M;

	imx6ul_spl_dram_cfg_size(ram_size);
	ram_size_test = get_ram_size((long int *)PHYS_SDRAM, ram_size);
	if (ram_size_test != ram_size) {
		puts("ERROR: DRAM size detection failed\n");
		hang();
	}
}

void board_init_f(ulong dummy)
{
	ccgr_init();
	arch_cpu_init();
	board_early_init_f();
	timer_init();
	preloader_console_init();
	imx6ul_spl_dram_cfg();
	memset(__bss_start, 0, __bss_end - __bss_start);
	board_init_r(NULL, 0);
}

void reset_cpu(void)
{
}

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_READY_B__USDHC1_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CE0_B__USDHC1_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CE1_B__USDHC1_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CLE__USDHC1_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC1_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}

int board_mmc_init(struct bd_info *bis)
{
	imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
}
#endif
