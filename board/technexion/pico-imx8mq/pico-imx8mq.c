// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */

#include <common.h>
#include <env.h>
#include <init.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../../freescale/common/tcpc.h"
#include "../../freescale/common/pfuze.h"
#include <usb.h>
#include <asm/armv8/mmu.h>
#include <dwc3-uboot.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MQ_PAD_GPIO1_IO02__WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MQ_PAD_UART1_RXD__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_UART1_TXD__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));
	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

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

#ifdef CONFIG_FSL_QSPI
int board_qspi_init(void)
{
	set_clk_qspi();

	return 0;
}
#endif

#ifdef CONFIG_FEC_MXC
#define FEC_RST_PAD IMX_GPIO_NR(1, 9)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
    IMX8MQ_PAD_GPIO1_IO09__GPIO1_IO9 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
    imx_iomux_v3_setup_multiple_pads(fec1_rst_pads,
                     ARRAY_SIZE(fec1_rst_pads));

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
	clrsetbits_le32(&gpr->gpr[1],
		IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK, 0);
	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#ifdef CONFIG_USB_DWC3

#define USB_PHY_CTRL0			0xF0040
#define USB_PHY_CTRL0_REF_SSP_EN	BIT(2)

#define USB_PHY_CTRL1			0xF0044
#define USB_PHY_CTRL1_RESET		BIT(0)
#define USB_PHY_CTRL1_COMMONONN		BIT(1)
#define USB_PHY_CTRL1_ATERESET		BIT(3)
#define USB_PHY_CTRL1_VDATSRCENB0	BIT(19)
#define USB_PHY_CTRL1_VDATDETENB0	BIT(20)

#define USB_PHY_CTRL2			0xF0048
#define USB_PHY_CTRL2_TXENABLEN0	BIT(8)

static struct dwc3_device dwc3_device_data = {
#ifdef CONFIG_SPL_BUILD
	.maximum_speed = USB_SPEED_HIGH,
#else
	.maximum_speed = USB_SPEED_SUPER,
#endif
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
	.power_down_scale = 2,
};

int usb_gadget_handle_interrupts(void)
{
	dwc3_uboot_handle_interrupt(0);
	return 0;
}

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 RegData;

	RegData = readl(dwc3->base + USB_PHY_CTRL1);
	RegData &= ~(USB_PHY_CTRL1_VDATSRCENB0 | USB_PHY_CTRL1_VDATDETENB0 |
			USB_PHY_CTRL1_COMMONONN);
	RegData |= USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET;
	writel(RegData, dwc3->base + USB_PHY_CTRL1);

	RegData = readl(dwc3->base + USB_PHY_CTRL0);
	RegData |= USB_PHY_CTRL0_REF_SSP_EN;
	writel(RegData, dwc3->base + USB_PHY_CTRL0);

	RegData = readl(dwc3->base + USB_PHY_CTRL2);
	RegData |= USB_PHY_CTRL2_TXENABLEN0;
	writel(RegData, dwc3->base + USB_PHY_CTRL2);

	RegData = readl(dwc3->base + USB_PHY_CTRL1);
	RegData &= ~(USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET);
	writel(RegData, dwc3->base + USB_PHY_CTRL1);
}
#endif

#ifdef CONFIG_USB_TCPC
struct tcpc_port port;
struct tcpc_port_config port_config = {
	.i2c_bus = 0,
	.addr = 0x50,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 20000,
	.max_snk_ma = 3000,
	.max_snk_mw = 15000,
	.op_snk_mv = 9000,
};

struct gpio_desc type_sel_desc;
static iomux_v3_cfg_t ss_mux_gpio[] = {
	IMX8MQ_PAD_NAND_RE_B__GPIO3_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void ss_mux_select(enum typec_cc_polarity pol)
{
	if (pol == TYPEC_POLARITY_CC1)
		dm_gpio_set_value(&type_sel_desc, 1);
	else
		dm_gpio_set_value(&type_sel_desc, 0);
}

static int setup_typec(void)
{
	int ret;

	imx_iomux_v3_setup_multiple_pads(ss_mux_gpio, ARRAY_SIZE(ss_mux_gpio));

	ret = dm_gpio_lookup_name("GPIO3_15", &type_sel_desc);
	if (ret) {
		printf("%s lookup GPIO3_15 failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	ret = dm_gpio_request(&type_sel_desc, "typec_sel");
	if (ret) {
		printf("%s request typec_sel failed ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	dm_gpio_set_dir_flags(&type_sel_desc, GPIOD_IS_OUT);

	ret = tcpc_init(&port, port_config, &ss_mux_select);
	if (ret) {
		printf("%s: tcpc init failed, err=%d\n",
		       __func__, ret);
	}

	return ret;
}
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	if (index == 0 && init == USB_INIT_DEVICE) {
		imx8m_usb_power(index, true);
#ifdef CONFIG_USB_TCPC
		ret = tcpc_setup_ufp_mode(&port);
#endif
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	} else if (index == 0 && init == USB_INIT_HOST) {
#ifdef CONFIG_USB_TCPC
		ret = tcpc_setup_dfp_mode(&port);
#endif
		return ret;
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;
	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_uboot_exit(index);
		imx8m_usb_power(index, false);
	} else if (index == 0 && init == USB_INIT_HOST) {
#ifdef CONFIG_USB_TCPC
		ret = tcpc_disable_src_vbus(&port);
#endif
	}

	return ret;
}
#endif

int board_init(void)
{
#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
	init_usb_clk();
#endif

#ifdef CONFIG_USB_TCPC
	setup_typec();
#endif
	return 0;
}

int board_mmc_get_env_dev(int devno)
{
    return devno;
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

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "EVK");
	env_set("board_rev", "iMX8MQ");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
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

