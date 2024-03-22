 // SPDX-License-Identifier: GPL-2.0+
 /*
  * Copyright 2020 TechNexion Ltd.
  *
  * Author: Richard Hu <richard.hu@technexion.com>
  *
  */
#include <common.h>
#include <efi_loader.h>
#include <env.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/global_data.h>
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
#include <splash.h>
#include <imx_sip.h>
#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <command.h>
#include "../common/periph_detect.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART2_RXD_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART2_TXD_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

const tn_camera_chk_t tn_camera_chk[] = {
	{ 1, 1, 0x3c, "tevi-ov5640" },
	{ 1, 1, 0x3d, "tevi-ap1302" },
};
size_t tn_camera_chk_cnt = ARRAY_SIZE(tn_camera_chk);

struct tn_display const displays[]= {
/*      bus, addr, id_reg, id, detect */
	{ 2, 0x38, 0xA3, 0x54, "ili9881c", detect_i2c },
	{ 2, 0x38, 0xA3, 0x58, "g080uan01", detect_i2c },
	{ 2, 0x38, 0xA3, 0x59, "g101uan02", detect_i2c },
	{ 2, 0x3d, 0x98, 0x3d, "mipi2hdmi-adv7535", detect_i2c },
	{ 2, 0x3d, 0x98, 0x03, "mipi2hdmi-adv7535", detect_i2c },
};
size_t tn_display_count = ARRAY_SIZE(displays);


#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"IMX8MM-EVK-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 2=flash-bin raw 0x42 0x2000 mmcpart 1",
	.images = fw_images,
};

u8 num_image_type_guids = ARRAY_SIZE(fw_images);
#endif /* EFI_HAVE_CAPSULE_SUPPORT */

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);

	return 0;
}

int board_phys_sdram_size(phys_size_t *size)
{
	if (!size)
		return -EINVAL;

	/*************************************************
	ToDo: It's a dirty workaround to store the
	information of DDR size into start address of TCM.
	It'd be better to detect DDR size from DDR controller.
	**************************************************/
	u32 ddr_size = readl(MCU_BOOTROM_BASE_ADDR);

	switch (ddr_size) {
	case 0x4: /* DRAM size: 4GB */
		*size = SZ_4G;
		break;
	case 0x3: /* DRAM size: 3GB */
		*size = SZ_3G;
		break;
	case 0x2: /* DRAM size: 2GB */
		*size = SZ_2G;
		break;
	case 0x1: /* DRAM size: 1GB */
		*size = SZ_1G;
		break;
	default:
		puts("Unknown DDR type!!!\n");
	}

	return 0;
}

/* Get the top of usable RAM */
phys_size_t board_get_usable_ram_top(phys_size_t total_size)
{
	if(gd->ram_top > 0x100000000)
		gd->ram_top = 0x100000000;

	return gd->ram_top;
}

#if IS_ENABLED(CONFIG_FEC_MXC)
#define FEC_RST_PAD IMX_GPIO_NR(2, 10)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
	IMX8MM_PAD_SD1_RESET_B_GPIO2_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
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
	clrsetbits_le32(&gpr->gpr[1], 0x2000, 0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
#ifndef CONFIG_DM_ETH
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);
#endif
	return 0;
}
#endif

#define CSI_PDN_PAD IMX_GPIO_NR(1, 6)
#define CSI_RST_PAD IMX_GPIO_NR(1, 5)
static iomux_v3_cfg_t const csi_pads[] = {
	IMX8MM_PAD_GPIO1_IO05_GPIO1_IO5 | MUX_PAD_CTRL(NO_PAD_CTRL),
	IMX8MM_PAD_GPIO1_IO06_GPIO1_IO6 | MUX_PAD_CTRL(NO_PAD_CTRL),
	IMX8MM_PAD_GPIO1_IO14_CCM_CLKO1 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_csi(void)
{
	imx_iomux_v3_setup_multiple_pads(csi_pads,
					 ARRAY_SIZE(csi_pads));

	gpio_request(CSI_PDN_PAD, "csi_pwdn");
	gpio_direction_output(CSI_PDN_PAD, 0);
	udelay(500);
	gpio_direction_output(CSI_PDN_PAD, 1);

	gpio_request(CSI_RST_PAD, "csi_rst");
	gpio_direction_output(CSI_RST_PAD, 0);
	udelay(500);
	gpio_direction_output(CSI_RST_PAD, 1);
}

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

#define WL_REG_ON_PAD IMX_GPIO_NR(1, 11)
static iomux_v3_cfg_t const wl_reg_on_pads[] = {
	IMX8MM_PAD_GPIO1_IO11_GPIO1_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define BT_ON_PAD IMX_GPIO_NR(1, 12)
static iomux_v3_cfg_t const bt_on_pads[] = {
	IMX8MM_PAD_GPIO1_IO12_GPIO1_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void setup_wifi(void)
{
	imx_iomux_v3_setup_multiple_pads(wl_reg_on_pads, ARRAY_SIZE(wl_reg_on_pads));
	imx_iomux_v3_setup_multiple_pads(bt_on_pads, ARRAY_SIZE(bt_on_pads));

	gpio_request(WL_REG_ON_PAD, "wl_reg_on");
	gpio_direction_output(WL_REG_ON_PAD, 0);
	gpio_set_value(WL_REG_ON_PAD, 0);

	gpio_request(BT_ON_PAD, "bt_on");
	gpio_direction_output(BT_ON_PAD, 0);
	gpio_set_value(BT_ON_PAD, 0);
}

#ifdef CONFIG_SPLASH_SCREEN
static struct splash_location imx_splash_locations[] = {
	{
		.name = "sf",
		.storage = SPLASH_STORAGE_SF,
		.flags = SPLASH_STORAGE_RAW,
		.offset = 0x100000,
	},
	{
		.name = "mmc_fs",
		.storage = SPLASH_STORAGE_MMC,
		.flags = SPLASH_STORAGE_FS,
		.devpart = "0:1",
	},
	{
		.name = "usb_fs",
		.storage = SPLASH_STORAGE_USB,
		.flags = SPLASH_STORAGE_FS,
		.devpart = "0:1",
	},
	{
		.name = "sata_fs",
		.storage = SPLASH_STORAGE_SATA,
		.flags = SPLASH_STORAGE_FS,
		.devpart = "0:1",
	},
};

/*This function is defined in common/splash.c.
  Declare here to remove warning. */
int splash_video_logo_load(void);

int splash_screen_prepare(void)
{
	imx_splash_locations[1].devpart[0] = mmc_get_env_dev() + '0';
	int ret;
	ret = splash_source_load(imx_splash_locations, ARRAY_SIZE(imx_splash_locations));
	if (!ret)
		return 0;
	else {
		printf("\nNo splash.bmp in boot partition!!\n");
		printf("Using default logo!!\n\n");
		return splash_video_logo_load();
	}
}
#endif /* CONFIG_SPLASH_SCREEN */

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				9
#define MIPI				10

int board_init(void)
{
	struct arm_smccc_res res;

	setup_wifi();
	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	setup_csi();

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

#define FT5336_TOUCH_I2C_BUS 2
#define FT5336_TOUCH_I2C_ADDR 0x38
#define ADV7535_HDMI_I2C_ADDR 0x3d
#define PCA9555_23_I2C_ADDR 0x23
#define PCA9555_26_I2C_ADDR 0x26
#define EXPANSION_IC_I2C_BUS 2

int detect_baseboard(void)
{
	struct udevice *bus = NULL;
	struct udevice *i2c_dev = NULL;
	int ret;
	char *fdtfile, *baseboard, str_fdtfile[64];

	fdtfile = env_get("fdtfile");
	if (fdtfile && !strcmp(fdtfile, "undefined")) {
		ret = uclass_get_device_by_seq(UCLASS_I2C, EXPANSION_IC_I2C_BUS, &bus);
		if (ret) {
			printf("%s: Can't find bus\n", __func__);
			return -EINVAL;
		}

		baseboard = env_get("baseboard");
		if (!dm_i2c_probe(bus, PCA9555_23_I2C_ADDR, 0, &i2c_dev) && \
		!dm_i2c_probe(bus, PCA9555_26_I2C_ADDR, 0, &i2c_dev) )
			env_set("baseboard", "wizard");
		else
			env_set("baseboard", "pi");
		baseboard = env_get("baseboard");

		strcpy(str_fdtfile, "imx8mm-pico-");
		strcat(str_fdtfile, baseboard);
		strcat(str_fdtfile, ".dtb");
		env_set("fdtfile", str_fdtfile);
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

	offs = fdt_path_offset(blob, "/reserved-memory/linux,cma");
	cell = fdt_getprop(blob, offs, "size", NULL);
	cma_size = fdt32_to_cpu(cell[1]);
	cmasize = env_get("cma_size");
	if(cmasize || ((u64)(gd->ram_size >> 1) < cma_size)) {
		cma_size = env_get_ulong("cma_size", 10, 320 * 1024 * 1024);
		cma_size = min((u64)(gd->ram_size >> 1), (u64)cma_size);
		fdt_setprop_u64(blob, offs, "size", (uint64_t)cma_size);
	}

	return 0;
}
#endif

int board_late_init(void)
{
#ifndef CONFIG_AVB_SUPPORT
	detect_baseboard();
	detect_display_panel();
	detect_camera();
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "PICO");
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
