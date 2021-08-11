/*
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright 2020 TechNexion Ltd.
 *
 * Author: Andy Lin <andy.lin@technexion.com>
 *
 */
#include <common.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <i2c.h>
#include <asm/io.h>
#include <usb.h>
#include <asm/armv8/mmu.h>
#include <linux/errno.h>
#include <stdbool.h>
#include <mmc.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_SAI2_RXC_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_SAI2_RXFS_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(0);

	return 0;
}

static int ddr_size;
extern struct mm_region *mem_map;
#define DRAM1_INDEX 5 /* Correspond to the index of DRAM1 of imx8m_mem_map
                         in arch/arm/mach-imx/imx8m/soc.c */

int board_phys_sdram_size(phys_size_t *size)
{
	if (!size)
		return -EINVAL;

	/*************************************************
	ToDo: It's a dirty workaround to store the
	information of DDR size into start address of TCM.
	It'd be better to detect DDR size from DDR controller.
	**************************************************/
	ddr_size = readl(MCU_BOOTROM_BASE_ADDR);

	if (ddr_size == 0x4) {
		*size = SZ_4G;
		mem_map[DRAM1_INDEX].size=SZ_4G;
	}
	else if (ddr_size == 0x3) {
		*size = SZ_3G;
		mem_map[DRAM1_INDEX].size=SZ_3G;
	}
	else if (ddr_size == 0x2) {
		*size = SZ_2G;
		mem_map[DRAM1_INDEX].size=SZ_2G;
	}
	else if (ddr_size == 0x1) {
		*size = SZ_1G;
		mem_map[DRAM1_INDEX].size=SZ_1G;
	}
	else
		puts("Unknown DDR type!!!\n");
	return 0;
}

/* Get the top of usable RAM */
ulong board_get_usable_ram_top(ulong total_size)
{
	if(gd->ram_top > 0x100000000)
		gd->ram_top = 0x100000000;

	return gd->ram_top;
}

#if IS_ENABLED(CONFIG_FEC_MXC)
#define PHY_PWR_PAD IMX_GPIO_NR(3, 1)
#define FEC_RST_PAD IMX_GPIO_NR(3, 2)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
	IMX8MM_PAD_NAND_CE1_B_GPIO3_IO2 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const phy_pwr_pads[] = {
	IMX8MM_PAD_NAND_CE0_B_GPIO3_IO1 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_rst_pads,
					 ARRAY_SIZE(fec1_rst_pads));

	imx_iomux_v3_setup_multiple_pads(phy_pwr_pads,
					 ARRAY_SIZE(phy_pwr_pads));

	gpio_request(PHY_PWR_PAD, "phy_pwr");
	gpio_direction_output(PHY_PWR_PAD, 1);
	mdelay(100);

	gpio_request(FEC_RST_PAD, "fec1_rst");
	gpio_direction_output(FEC_RST_PAD, 0);
	udelay(500);
	gpio_direction_output(FEC_RST_PAD, 1);
}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	setup_iomux_fec();
	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1], 0x2000, 0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	debug("board_usb_init %d, type %d\n", index, init);

	imx8m_usb_power(index, true);

	return ret;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;

	debug("board_usb_cleanup %d, type %d\n", index, init);

	imx8m_usb_power(index, false);
	return ret;
}

#define WL_REG_ON_PAD IMX_GPIO_NR(3, 6)
static iomux_v3_cfg_t const wl_reg_on_pads[] = {
	IMX8MM_PAD_NAND_DATA00_GPIO3_IO6 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define BT_ON_PAD IMX_GPIO_NR(3, 7)
static iomux_v3_cfg_t const bt_on_pads[] = {
	IMX8MM_PAD_NAND_DATA01_GPIO3_IO7 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void setup_wifi(void)
{
	imx_iomux_v3_setup_multiple_pads(wl_reg_on_pads, ARRAY_SIZE(wl_reg_on_pads));
	imx_iomux_v3_setup_multiple_pads(bt_on_pads, ARRAY_SIZE(bt_on_pads));

	gpio_request(WL_REG_ON_PAD, "wl_reg_on");
	gpio_direction_output(WL_REG_ON_PAD, 0);
	gpio_set_value(WL_REG_ON_PAD, 1);

	gpio_request(BT_ON_PAD, "bt_on");
	gpio_direction_output(BT_ON_PAD, 0);
	gpio_set_value(BT_ON_PAD, 0);
}

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				9
#define MIPI				10

int board_init(void)
{
	setup_wifi();
	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	return 0;
}

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

#define EXPANSION_IC_I2C_BUS 2
#define EXPANSION_IC_I2C_ADDR 0x27

int board_late_init(void)
{
	char *fdt_file = NULL;
	struct udevice *bus = NULL;
	struct udevice *i2c_dev = NULL;
	int ret;

	fdt_file = env_get("fdt_file");
	if (fdt_file && !strcmp(fdt_file, "undefined")) {
		ret = uclass_get_device_by_seq(UCLASS_I2C, EXPANSION_IC_I2C_BUS, &bus);
		if (ret) {
			printf("%s: Can't find bus\n", __func__);
			return -EINVAL;
		}

		ret = dm_i2c_probe(bus, EXPANSION_IC_I2C_ADDR, 0, &i2c_dev);
		if (ret) {
			env_set("fdt_file", "imx8mm-axon-pi.dtb");
			env_set("baseboard", "pi");
		}
		else {
			env_set("fdt_file", "imx8mm-axon-wizard.dtb");
			env_set("baseboard", "wizard");
		}
	}
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "AXON");
	env_set("board_rev", "iMX8MM");
#endif
	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif
