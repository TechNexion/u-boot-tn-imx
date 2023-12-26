/*
 * Copyright (C) 2018 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <init.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <mxc_epdc_fb.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <spl.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_SPL_BUILD)


#define FEC_RST_GPIO	IMX_GPIO_NR(5, 11)
#define FEC_PWR_GPIO    IMX_GPIO_NR(4, 15)

static iomux_v3_cfg_t const enet_pwr_en_pads[] = {
	MX7D_PAD_I2C4_SDA__GPIO4_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL), // power pin
};

static void enable_eth_pwr(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pwr_en_pads, ARRAY_SIZE(enet_pwr_en_pads));
	gpio_direction_output(FEC_PWR_GPIO, 0);
	gpio_set_value(FEC_PWR_GPIO, 0);
}

#define DDR_TYPE_DET   IMX_GPIO_NR(4, 21)

static iomux_v3_cfg_t const ddr_type_detection_pads[] = {
	/* ddr type detection: 512MB or 1GB */
	MX7D_PAD_EPDC_BDR0__GPIO2_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
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
	while ((readl(0x307900c4) & 0x1) != 0x1)
		;

	writel(0x0e247304, 0x307900c0);
	writel(0x0e207304, 0x307900c0);

	writel(0x00000000, 0x30384130);
	writel(0x00000178, 0x30340020);
	writel(0x00000002, 0x30384130);
	writel(0x0000000f, 0x30790018);
	while ((readl(0x307a0004) & 0x1) != 0x1)
		;
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
	while ((readl(0x307900c4) & 0x1) != 0x1)
		;

	writel(0x0e247304, 0x307900c0);
	writel(0x0e207304, 0x307900c0);

	writel(0x00000000, 0x30384130);
	writel(0x00000178, 0x30340020);
	writel(0x00000002, 0x30384130);
	writel(0x0000000f, 0x30790018);
	while ((readl(0x307a0004) & 0x1) != 0x1)
		;
}

static void spl_dram_init(void)
{
	setup_iomux_ddr_type_detection();
	gpio_direction_input(DDR_TYPE_DET);

	if (gpio_get_value(DDR_TYPE_DET))
		ddr3_1gb_init();
	else
		ddr3_512mb_init();
}

static void gpr_init(void)
{
	struct iomuxc_gpr_base_regs *gpr_regs =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;
	writel(0x4F400005, &gpr_regs->gpr[1]);
}

void board_init_f(ulong dummy)
{
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

	/* Turn on power for ETH PHY */
	enable_eth_pwr();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

#endif
