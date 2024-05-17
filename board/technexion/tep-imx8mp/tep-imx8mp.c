// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 */

#include <common.h>
#include <env.h>
#include <errno.h>
#include <init.h>
#include <errno.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/delay.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mp_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <imx_sip.h>
#include <linux/arm-smccc.h>
#include <mmc.h>
#include <asm/armv8/mmu.h>
#include <fdt_support.h>
#include <jffs2/load_kernel.h>
#include <mtd_node.h>
#include <command.h>
#include "../common/periph_detect.h"
#include <asm/mach-imx/boot_mode.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)
#define OTG_PWR_EN_PAD IMX_GPIO_NR(4, 0)

const tn_camera_chk_t tn_camera_chk[] = {
	{ 1, 1, 0x0f, "hdmi2mipi-tc358743" },
};
size_t tn_camera_chk_cnt = ARRAY_SIZE(tn_camera_chk);

struct tn_display const displays[]= {
/*      bus, addr, id_reg, id, detect */
	{  0,  0, 0x0eef, 150, "lvds-vl15010276",  detect_exc3000_usb },
	{  0,  0, 0x0eef, 156, "lvds-vl156192108", detect_exc3000_usb },
	{  0,  0, 0x0eef, 215, "lvds-vl215192108", detect_exc3000_usb },
	{ 3, 0x2a, 0,     101, "vizionpanel-vl10112880", detect_vizionpanel_i2c },
	{ 3, 0x2a, 0,     150, "vizionpanel-vl15010276", detect_vizionpanel_i2c },
	{ 3, 0x2a, 0,     156, "vizionpanel-vl15613676", detect_vizionpanel_i2c },
};
size_t tn_display_count = ARRAY_SIZE(displays);

static iomux_v3_cfg_t const uart_pads[] = {
	MX8MP_PAD_UART2_RXD__UART2_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX8MP_PAD_UART2_TXD__UART2_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	MX8MP_PAD_GPIO1_IO02__WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

static iomux_v3_cfg_t const otg_pwr_en_pads[] = {
	MX8MP_PAD_SAI1_RXFS__GPIO4_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);

	imx_iomux_v3_setup_multiple_pads(otg_pwr_en_pads, ARRAY_SIZE(otg_pwr_en_pads));
	gpio_request(OTG_PWR_EN_PAD, "otg_pwr_en");
        gpio_direction_output(OTG_PWR_EN_PAD, 1);

	return 0;
}

int board_phys_sdram_size(phys_size_t *size)
{
	if (!size)
		return -EINVAL;

	/*************************************************
	ToDo: It's a dirty workaround to store the
	information of DDR size into start address of OCRAM.
	It'd be better to detect DDR size from DDR controller.
	**************************************************/
	u32 ddr_size = readl(OCRAM_BASE_ADDR);

	switch (ddr_size) {
	case 0x5: /* DRAM size: 8GB */
		*size = SZ_8G;
		break;
	case 0x4: /* DRAM size: 6GB */
		*size = SZ_6G;
		break;
	case 0x3: /* DRAM size: 4GB */
		*size = SZ_4G;
		break;
	case 0x1: /* DRAM size: 2GB */
		*size = SZ_2G;
		break;
	case 0x2: /* DRAM size: 1GB */
		*size = SZ_1G;
		break;
	default:
		puts("Unknown DDR type!!!\n");
	}

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	const int *cell;
	int offs;
	uint32_t cma_size;
	char *cmasize;
#ifdef CONFIG_IMX8M_DRAM_INLINE_ECC
	int rc;
	phys_addr_t ecc0_start = 0xb0000000;
	phys_addr_t ecc1_start = 0x130000000;
	phys_addr_t ecc2_start = 0x1b0000000;
	size_t ecc_size = 0x10000000;

	rc = add_res_mem_dt_node(blob, "ecc", ecc0_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc0 reserved-memory node.\n");
		return rc;
	}

	rc = add_res_mem_dt_node(blob, "ecc", ecc1_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc1 reserved-memory node.\n");
		return rc;
	}

	rc = add_res_mem_dt_node(blob, "ecc", ecc2_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc2 reserved-memory node.\n");
		return rc;
	}
#endif

	offs = fdt_path_offset(blob, "/reserved-memory/linux,cma");
	cell = fdt_getprop(blob, offs, "size", NULL);
	cma_size = fdt32_to_cpu(cell[1]);
	cmasize = env_get("cma_size");
	if(cmasize || ((u64)(gd->ram_size >> 1) < cma_size)) {
		cma_size = env_get_ulong("cma_size", 10, 320 * 1024 * 1024);
		cma_size = min((u64)(gd->ram_size >> 1), (u64)cma_size);
		fdt_setprop_u64(blob, offs, "size", (uint64_t)cma_size);
	}

	static const struct node_info nodes[] = {
		{ "jedec,spi-nor", MTD_DEV_TYPE_NOR, },
	};

	fdt_fixup_mtdparts(blob, nodes, ARRAY_SIZE(nodes));

	return 0;
}
#endif

#ifdef CONFIG_FEC_MXC
#define FEC_RST_PAD IMX_GPIO_NR(1, 9)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
	MX8MP_PAD_GPIO1_IO09__GPIO1_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_rst_pads,
		ARRAY_SIZE(fec1_rst_pads));

	gpio_request(FEC_RST_PAD, "fec1_rst");
	gpio_direction_output(FEC_RST_PAD, 0);
	mdelay(500);
	gpio_direction_output(FEC_RST_PAD, 1);
}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	setup_iomux_fec();
	/* Enable RGMII TX clk output */
	setbits_le32(&gpr->gpr[1], BIT(22));

	return 0;
}
#endif

#ifdef CONFIG_DWC_ETH_QOS
#define EQOS_RST_PAD IMX_GPIO_NR(1, 10)
static iomux_v3_cfg_t const eqos_rst_pads[] = {
	MX8MP_PAD_GPIO1_IO10__GPIO1_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_eqos(void)
{
	imx_iomux_v3_setup_multiple_pads(eqos_rst_pads,
					 ARRAY_SIZE(eqos_rst_pads));

	gpio_request(EQOS_RST_PAD, "eqos_rst");
	gpio_direction_output(EQOS_RST_PAD, 0);
	mdelay(15);
	gpio_direction_output(EQOS_RST_PAD, 1);
	mdelay(100);
}

static int setup_eqos(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	setup_iomux_eqos();

	/* set INTF as RGMII, enable RGMII TXC clock */
	clrsetbits_le32(&gpr->gpr[1],
			IOMUXC_GPR_GPR1_GPR_ENET_QOS_INTF_SEL_MASK, BIT(16));
	setbits_le32(&gpr->gpr[1], BIT(19) | BIT(21));

	return set_clk_eqos(ENET_125MHZ);
}
#endif

#if defined(CONFIG_FEC_MXC) || defined(CONFIG_DWC_ETH_QOS)
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

#define USB_PHY_CTRL6			0xF0058

#define HSIO_GPR_BASE                               (0x32F10000U)
#define HSIO_GPR_REG_0                              (HSIO_GPR_BASE)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT    (1)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN          (0x1U << HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT)


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

	/* enable usb clock via hsio gpr */
	RegData = readl(HSIO_GPR_REG_0);
	RegData |= HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN;
	writel(RegData, HSIO_GPR_REG_0);

	/* USB3.0 PHY signal fsel for 100M ref */
	RegData = readl(dwc3->base + USB_PHY_CTRL0);
	RegData = (RegData & 0xfffff81f) | (0x2a<<5);
	writel(RegData, dwc3->base + USB_PHY_CTRL0);

	RegData = readl(dwc3->base + USB_PHY_CTRL6);
	RegData &=~0x1;
	writel(RegData, dwc3->base + USB_PHY_CTRL6);

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

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
int board_usb_init(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, true);

	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_uboot_exit(index);
	}

	imx8m_usb_power(index, false);

	return 0;
}

#define USB_HUB_RST_PAD IMX_GPIO_NR(4, 22)
static iomux_v3_cfg_t const usb_hub_rst_pads[] = {
	MX8MP_PAD_SAI2_RXC__GPIO4_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void setup_usb_rst(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_hub_rst_pads, ARRAY_SIZE(usb_hub_rst_pads));

	gpio_request(USB_HUB_RST_PAD, "usb_hub_rst");
	gpio_direction_output(USB_HUB_RST_PAD, 0);
	gpio_set_value(USB_HUB_RST_PAD, 0);
	mdelay(20);
	gpio_set_value(USB_HUB_RST_PAD, 1);
}
#endif

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				13
#define MIPI				15

int board_init(void)
{
	struct arm_smccc_res res;

#ifdef CONFIG_DWC_ETH_QOS
	/* clock, pin, gpr */
	setup_eqos();
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
	setup_usb_rst();
	init_usb_clk();
#endif

	 /* enable the dispmix & mipi phy power domain */
	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN,
		DISPMIX, true, 0, 0, 0, 0, &res);
	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN,
		MIPI, true, 0, 0, 0, 0, &res);

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

#define DSI1_RST_NODE "vizionpanel"
#define PCA95554BS "pca9554bs@23"
void reset_dsi(void)
{
	int node, ret;
	struct udevice *udev;
	struct gpio_desc *dsi1_rst_gpio;

	ret = uclass_get_device_by_name(UCLASS_GPIO, PCA95554BS, &udev);
	if (ret != 0) {
		printf("%s: get %s udev failed\n", __func__, PCA95554BS);
		return;
	}

	node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(udev), DSI1_RST_NODE);
	if (node < 0) {
		printf("%s: Can't find node name '%s'\n", __func__, DSI1_RST_NODE);
		return;
	}

	ret = gpio_request_by_name_nodev(offset_to_ofnode(node), "reset-gpios",
			 0, dsi1_rst_gpio, GPIOD_IS_OUT);
	if ( (ret != 0) || (!dm_gpio_is_valid(dsi1_rst_gpio)) ) {
		printf("%s: request reset-gpios failed\n", __func__);
		return;
	}

	dm_gpio_set_value(dsi1_rst_gpio, 1);
	dm_gpio_set_value(dsi1_rst_gpio, 0);
	mdelay(50);
	dm_gpio_set_value(dsi1_rst_gpio, 1);

	dm_gpio_free(udev, dsi1_rst_gpio);
}

void check_if_boot_from_spi(void)
{
	enum boot_device bt_dev = get_boot_device();
	if (bt_dev == QSPI_BOOT)
		env_set("qspi_boot", "yes");
	else
		env_set("qspi_boot", "no");
}

void add_default_camera_overlay(void)
{
#define ENV_DTOVERLAY           "dtoverlay"
#define SIZE_DTOVERLAY          (256)
	int cam_ov_list_number = 3;
	int i;
	char* ov_list;
	char* default_cam_ov = "vls";
	char* cam_ov_list[] = { "hdmi2mipi-tc358743", "vls" };

	ov_list = env_get("ENV_DTOVERLAY");
	if (ov_list) {
		for (i=0; i< cam_ov_list_number ; i++)
		{
			if (strstr(ov_list, cam_ov_list[i]))
				return;
		}
		/* No any camera overlay found, add default overlay to it. */
		snprintf(ov_list, SIZE_DTOVERLAY, "%s %s", ov_list, default_cam_ov);
		env_set("ENV_DTOVERLAY", ov_list);
	} else {
		env_set("ENV_DTOVERLAY", default_cam_ov);
	}
}

int board_late_init(void)
{
	reset_dsi();
	check_if_boot_from_spi();
	detect_display_panel();
	detect_camera();

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	add_default_camera_overlay();
	setup_fec();

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "TEP");
	env_set("board_rev", "iMX8MP");
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

#ifdef CONFIG_SPL_MMC_SUPPORT

#define UBOOT_RAW_SECTOR_OFFSET 0x40
unsigned long spl_mmc_get_uboot_raw_sector(struct mmc *mmc)
{
	u32 boot_dev = spl_boot_device();
	switch (boot_dev) {
		case BOOT_DEVICE_MMC2:
			return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR - UBOOT_RAW_SECTOR_OFFSET;
		default:
			return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;
	}
}
#endif
