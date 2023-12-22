/*
 * Copyright (C) 2018 Technexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <mxc_epdc_fb.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <spl.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_SPL_BUILD)

/**********************************************
* Revision Detection
*
* DDR_TYPE_DET_1   DDR_TYPE_DET_2
*   GPIO_1           GPIO_2
*     0                1           2GB DDR3
*     0                0           1GB DDR3
*     1                0           512MB DDR3
***********************************************/
#define DDR_TYPE_DET_1   IMX_GPIO_NR(1, 12)
#define DDR_TYPE_DET_2   IMX_GPIO_NR(1, 13)

static iomux_v3_cfg_t const ddr_type_detection_pads[] = {
	/* ddr type detection: 512MB or 1GB */
	MX7D_PAD_GPIO1_IO12__GPIO1_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* ddr type detection: 2GB */
	MX7D_PAD_GPIO1_IO13__GPIO1_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_ddr_type_detection(void)
{
	imx_iomux_v3_setup_multiple_pads(ddr_type_detection_pads, ARRAY_SIZE(ddr_type_detection_pads));
}

static void ddr3_512mb_init(void)
{
	writel(0x4F400005, 0x30391000);
	/* Clear then set bit30 to ensure exit from DDR retention */
	writel(0x40000000, 0x30360388);
	writel(0x40000000, 0x30360384);

	writel(0x00000002, 0x30391000);
	writel(0x01040001, 0x307a0000);
	writel(0x00400046, 0x307a0064);
	writel(0x00000001, 0x307a0490);
	writel(0x00020083, 0x307a00d0);
	writel(0x00690000, 0x307a00d4);
	writel(0x09300004, 0x307a00dc);
	writel(0x04080000, 0x307a00e0);
	writel(0x00100004, 0x307a00e4);
	writel(0x0000033f, 0x307a00f4);
	writel(0x09081109, 0x307a0100);
	writel(0x0007020D, 0x307a0104);
	writel(0x03040407, 0x307a0108);
	writel(0x00002006, 0x307a010c);
	writel(0x04020205, 0x307a0110);
	writel(0x03030202, 0x307a0114);
	writel(0x00000803, 0x307a0120);
	writel(0x00800020, 0x307a0180);
	writel(0x02098204, 0x307a0190);
	writel(0x00030303, 0x307a0194);
	writel(0x80400003, 0x307a01a0);
	writel(0x00100020, 0x307a01a4);
	writel(0x80100004, 0x307a01a8);
	writel(0x00000015, 0x307a0200);
	writel(0x00161616, 0x307a0204);
	writel(0x00000F0F, 0x307A0210);
	writel(0x04040404, 0x307a0214);
	writel(0x0F0F0404, 0x307a0218);
	writel(0x06000604, 0x307a0240);
	writel(0x00000001, 0x307a0244);
	writel(0x00000000, 0x30391000);
	writel(0x17420f40, 0x30790000);
	writel(0x10210100, 0x30790004);
	writel(0x00060807, 0x30790010);
	writel(0x1010007e, 0x307900b0);
	writel(0x00000d6e, 0x3079009c);
	writel(0x08080808, 0x30790020);
	writel(0x08080808, 0x30790030);
	writel(0x01000010, 0x30790050);
	writel(0x00000010, 0x30790050);
	writel(0x0e207304, 0x307900c0);
	writel(0x0e247304, 0x307900c0);
	writel(0x0e247306, 0x307900c0);

	while ((readl(0x307900c4) & 0x1) != 0x1);

	writel(0x0e247304, 0x307900c0);
	writel(0x0e207304, 0x307900c0);

	writel(0x00000000, 0x30384130);
	writel(0x00000178, 0x30340020);
	writel(0x00000002, 0x30384130);
	writel(0x0000000f, 0x30790018);

	while ((readl(0x307a0004) & 0x1) != 0x1);
}

static void ddr3_1gb_init(void)
{
	writel(0x4F400005, 0x30391000);
	/* Clear then set bit30 to ensure exit from DDR retention */
	writel(0x40000000, 0x30360388);
	writel(0x40000000, 0x30360384);

	writel(0x00000002, 0x30391000);
	writel(0x01040001, 0x307a0000);
	writel(0x80400003, 0x307a01a0);
	writel(0x00100020, 0x307a01a4);
	writel(0x80100004, 0x307a01a8);
	writel(0x00400046, 0x307a0064);
	writel(0x00000001, 0x307a0490);
	writel(0x00020083, 0x307a00d0);
	writel(0x00690000, 0x307a00d4);
	writel(0x09300004, 0x307a00dc);
	writel(0x04080000, 0x307a00e0);
	writel(0x00100004, 0x307a00e4);
	writel(0x0000033f, 0x307a00f4);
	writel(0x09081109, 0x307a0100);
	writel(0x0007020d, 0x307a0104);
	writel(0x03040407, 0x307a0108);
	writel(0x00002006, 0x307a010c);
	writel(0x04020205, 0x307a0110);
	writel(0x03030202, 0x307a0114);
	writel(0x00000803, 0x307a0120);
	writel(0x00800020, 0x307a0180);
	writel(0x02000100, 0x307a0184);
	writel(0x02098204, 0x307a0190);
	writel(0x00030303, 0x307a0194);
	writel(0x00000016, 0x307a0200);
	writel(0x00171717, 0x307a0204);
	writel(0x04040404, 0x307a0214);
	writel(0x0f040404, 0x307a0218);
	writel(0x06000604, 0x307a0240);
	writel(0x00000001, 0x307a0244);
	writel(0x00000000, 0x30391000);
	writel(0x17420f40, 0x30790000);
	writel(0x10210100, 0x30790004);
	writel(0x00060807, 0x30790010);
	writel(0x1010007e, 0x307900b0);
	writel(0x00000d6e, 0x3079009c);
	writel(0x08080808, 0x30790020);
	writel(0x08080808, 0x30790030);
	writel(0x01000010, 0x30790050);
	writel(0x00000010, 0x30790050);

	writel(0x0e207304, 0x307900c0);
	writel(0x0e247304, 0x307900c0);
	writel(0x0e247306, 0x307900c0);

	while ((readl(0x307900c4) & 0x1) != 0x1);

	writel(0x0e247304, 0x307900c0);
	writel(0x0e207304, 0x307900c0);

	writel(0x00000000, 0x30384130);
	writel(0x00000178, 0x30340020);
	writel(0x00000002, 0x30384130);
	writel(0x0000000f, 0x30790018);

	while ((readl(0x307a0004) & 0x1) != 0x1);
}

static void ddr3_2gb_init(void)
{
	writel(0x4F400005, 0x30391000);
	/* Clear then set bit30 to ensure exit from DDR retention */
	writel(0x40000000, 0x30360388);
	writel(0x40000000, 0x30360384);

	writel(0x00000002, 0x30391000);
	writel(0x01040001, 0x307A0000);
	writel(0x00400046, 0x307A0064);
	writel(0x00000001, 0x307a0490);
	writel(0x00690000, 0x307A00D4);
	writel(0x00020083, 0x307A00D0);
	writel(0x00000000, 0x307A00D8);
	writel(0x09300004, 0x307A00DC);
	writel(0x04080000, 0x307A00E0);
	writel(0x00100004, 0x307A00E4);
	writel(0x0000033F, 0x307A00F4);
	writel(0x09081109, 0x307A0100);
	writel(0x0007020D, 0x307A0104);
	writel(0x03040407, 0x307A0108);
	writel(0x00002006, 0x307A010C);
	writel(0x04020205, 0x307A0110);
	writel(0x03030202, 0x307A0114);
	writel(0x00000803, 0x307A0120);
	writel(0x00800020, 0x307A0180);
	writel(0x02098204, 0x307A0190);
	writel(0x00030303, 0x307A0194);
	writel(0x80400003, 0x307A01A0);
	writel(0x00100020, 0x307A01A4);
	writel(0x80100004, 0x307A01A8);
	writel(0x0000001F, 0x307A0200);
	writel(0x00181818, 0x307A0204);
	writel(0x00000000, 0x307A020C);
	writel(0x00000F0F, 0x307A0210);
	writel(0x04040404, 0x307A0214);
	writel(0x04040404, 0x307A0218);
	writel(0x06000604, 0x307A0240);
	writel(0x00000001, 0x307A0244);

	writel(0x00000000, 0x30391000);
	writel(0x17420F40, 0x30790000);
	writel(0x10210100, 0x30790004);
	writel(0x00060807, 0x30790010);
	writel(0x1010007E, 0x307900B0);
	writel(0x00000D6E, 0x3079009C);

	writel(0x04040404, 0x30790030);
	writel(0x0A0A0A0A, 0x30790020);
	writel(0x01000010, 0x30790050);
	writel(0x00000010, 0x30790050);
	writel(0x0E207304, 0x307900C0);
	writel(0x0E247304, 0x307900C0);
	writel(0x0E247306, 0x307900C0);

	while ((readl(0x307900c4) & 0x1) != 0x1);

	writel(0x0e247304, 0x307900c0);
	writel(0x0e207304, 0x307900c0);

	writel(0x00000000, 0x30384130);
	writel(0x00000178, 0x30340020);
	writel(0x00000002, 0x30384130);
	writel(0x0000000f, 0x30790018);

	while ((readl(0x307a0004) & 0x1) != 0x1);
}

static void spl_dram_init(void)
{
	setup_iomux_ddr_type_detection();
	gpio_direction_input(DDR_TYPE_DET_1);
	gpio_direction_input(DDR_TYPE_DET_2);

	if (gpio_get_value(DDR_TYPE_DET_1)) {
		ddr3_512mb_init();
	} else if (gpio_get_value(DDR_TYPE_DET_2)) {
		ddr3_2gb_init();
	} else {
		ddr3_1gb_init();
	}
}

static void gpr_init(void)
{
	struct iomuxc_gpr_base_regs *gpr_regs =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;
	writel(0x4F400005, &gpr_regs->gpr[1]);
}

#ifdef CONFIG_SPL_OS_BOOT
/* optional: Do board specific preparation before SPL does the Linux kernel image boot */
void spl_board_prepare_for_linux(void)
{
	/* maybe update the uEnv.txt? */
	/* maybe setup the kernel ATAGS? */
	printf("prepare to boot linux, timestamp: %lu\n", timer_get_us());
}

/* required: SPL falcon mode:  Returns "0": for booting kernel, "1" for booting uboot. */
int spl_start_uboot (void)
{
	/* break into falcon u-boot on 'f' */
	if (serial_tstc() && serial_getc() == 'f') {
		printf("Falcon Mode: timestamp: %lu\n", timer_get_us());
		return 0;
	}
	printf("Uboot Mode: timestamp: %lu\n", timer_get_us());
	return 1;
}
#endif

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

        ret = spl_init();
        if (ret) {
                debug("spl_init() failed: %d\n", ret);
                hang();
        }

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

#endif
