/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 NXP
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 */

#include <env.h>
#include <efi_loader.h>
#include <init.h>
#include <fdt_support.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <mmc.h>
#include <power/regulator.h>
#include <scmi_agent.h>
#include "../dts/upstream/src/arm64/freescale/imx95-power.h"
#include <i2c.h>
#include <asm/arch/sys_proto.h>
#include <dm/uclass.h>
#include <dm/uclass-internal.h>
#include <command.h>
#include "edm-imx95-ddr.h"
#include "../common/periph_detect.h"
#include <miiphy.h>
#include <phy.h>

extern int board_fix_fdt_fuse(void *fdt);

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
#define IMX_BOOT_IMAGE_GUID \
	EFI_GUID(0x2c4db6b3, 0x0b15, 0x4a36, 0xbe, 0xae, \
		 0x1e, 0xa1, 0x35, 0x46, 0x4f, 0x5b)

struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"EDM-IMX95-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 0=flash-bin raw 0 0x2000 mmcpart 1",
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};
#endif /* EFI_HAVE_CAPSULE_SUPPORT */

#ifdef CONFIG_SPLASH_SCREEN
#include <splash.h>

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
	int ret;
	pr_err("splash_screen_prepare\n");
	imx_splash_locations[1].devpart[0] = mmc_get_env_dev() + '0';
	pr_err("splash_screen_prepare, devpart:%s\n", imx_splash_locations[1].devpart);
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

#ifdef CONFIG_TN_PHERIPHERAL_DETECT
const tn_camera_chk_t tn_camera_chk[] = {
	{ 4, 0x48, 0x2e, 0x00, 0x3020, 0x00, "tevs-csi0" },
	{ 1, 0x48, 0x2e, 0x00, 0x3020, 0x00, "tevs-csi1" },
	{ 4, 0x48, 0x2e, 0x00, 0x3020, 0x01, "tevm-csi0" },
	{ 1, 0x48, 0x2e, 0x00, 0x3020, 0x01, "tevm-csi1" },
	{ 4, 0x48, 0x00, 0x25, 0x3020, 0x00, "vls-gm2-csi0" },
	{ 1, 0x48, 0x00, 0x25, 0x3020, 0x00, "vls-gm2-csi1" },
	{ 4, 0x48, 0x00, 0x25, 0x3020, 0x01, "vlm-gm2-csi0" },
	{ 1, 0x48, 0x00, 0x25, 0x3020, 0x01, "vlm-gm2-csi1" },
};
size_t tn_camera_chk_cnt = ARRAY_SIZE(tn_camera_chk);
struct tn_display const displays[]= {
/*      bus, addr, id_reg, id, detect */
	{ 0, 0x2a, 0,    101,  "lvds-vl10112880", detect_exc3000_i2c },
	{ 0, 0x2a, 0,    156,  "lvds-vl156192108", detect_exc3000_i2c },
	{ 0, 0x3d, 0x98, 0x03, "mipi2hdmi-adv7535", detect_i2c },
	{ 0, 0x3d, 0x98, 0x3b, "mipi2hdmi-adv7535", detect_i2c }
};
size_t tn_display_count = ARRAY_SIZE(displays);
#endif

enum typec_cc_polarity {
	TYPEC_POLARITY_CC1,
	TYPEC_POLARITY_CC2,
};

ulong tca_base;

void tca_mux_select(enum typec_cc_polarity pol)
{
	u32 val;

	if (!tca_base)
		return;

	/* Set OP mode to System configure Mode */
	clrbits_le32(tca_base + 0x10, 0x3);

	val = readl(tca_base + 0x30);

	setbits_le32(tca_base + 0x18, BIT(3));
	udelay(1);

	if (pol == TYPEC_POLARITY_CC1)
		clrbits_le32(tca_base + 0x18, BIT(2));
	else
		setbits_le32(tca_base + 0x18, BIT(2));

	udelay(1);

	clrbits_le32(tca_base + 0x18, BIT(3));
}

static void setup_typec(void)
{
	tca_base = USB1_BASE_ADDR + 0xfc000;

	tca_mux_select(TYPEC_POLARITY_CC1);
}

static int imx9_scmi_power_domain_enable(u32 domain, bool enable)
{
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_name(UCLASS_CLK, "protocol@14", &dev);
	if (ret)
		return ret;

	return scmi_pwd_state_set(dev, 0, domain, enable ? 0 : BIT(30));
}

int board_usb_init(int index, enum usb_init_type init)
{
	if (index == 0 && init == USB_INIT_DEVICE) {
		setup_typec();
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	return 0;
}

void camera_init(void)
{
	int ret;
	struct gpio_desc csi1_pdb, csi2_pdb;

	/* CSI1_PDB - gpio@21 pin 12 */
	ret = dm_gpio_lookup_name("gpio@21_12", &csi1_pdb);
	if (ret) {
		printf("%s: lookup gpio@21_12 failed ret = %d\n", __func__, ret);
	} else {
		ret = dm_gpio_request(&csi1_pdb, "CSI1_PDB");
		if (ret) {
			printf("%s: request CSI1_PDB failed ret = %d\n", __func__, ret);
		} else {
			dm_gpio_set_dir_flags(&csi1_pdb, GPIOD_IS_OUT);
			dm_gpio_set_value(&csi1_pdb, 0); /* Set to LOW */
			dm_gpio_set_value(&csi1_pdb, 1); /* Set to HIGH */
		}
	}

	/* CSI2_PDB - gpio@21 pin 13 */
	ret = dm_gpio_lookup_name("gpio@21_13", &csi2_pdb);
	if (ret) {
		printf("%s: lookup gpio@21_13 failed ret = %d\n", __func__, ret);
	} else {
		ret = dm_gpio_request(&csi2_pdb, "CSI2_PDB");
		if (ret) {
			printf("%s: request CSI2_PDB failed ret = %d\n", __func__, ret);
		} else {
			dm_gpio_set_dir_flags(&csi2_pdb, GPIOD_IS_OUT);
			dm_gpio_set_value(&csi2_pdb, 0); /* Set to LOW */
			dm_gpio_set_value(&csi2_pdb, 1); /* Set to HIGH */
		}
	}
}

static void netc_phy_rst(const char *gpio_name, const char *label)
{
	int ret;
	struct gpio_desc desc;

	/* ENET_RST_B */
	ret = dm_gpio_lookup_name(gpio_name, &desc);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, gpio_name, ret);
		return;
	}

	ret = dm_gpio_request(&desc, label);
	if (ret) {
		printf("%s request %s failed ret = %d\n", __func__, label, ret);
		return;
	}

	/* assert the ENET_RST_B */
	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);
	udelay(10000);
	dm_gpio_set_value(&desc, 0); /* deassert the ENET_RST_B */
	udelay(80000);

}

static void __maybe_unused netc_regulator_enable(const char *devname, bool enable)
{
	int ret;
	struct udevice *dev;

	ret = regulator_get_by_devname(devname, &dev);
	if (ret) {
		printf("Get %s regulator failed %d\n", devname, ret);
		return;
	}

	ret = regulator_set_enable_if_allowed(dev, enable);
	if (ret) {
		printf("%s %s regulator %d\n",
			enable ? "Enable": "Disable", devname, ret);
		return;
	}
}

void netc_init(void)
{
	int ret;

	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, false);
	udelay(10000);

	/* Power up the NETC MIX. */
	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, true);
	if (ret) {
		printf("SCMI_POWWER_STATE_SET Failed for NETC MIX\n");
		return;
	}

	netc_phy_rst("gpio@22_1", "ENET1_RST_B");
	netc_phy_rst("gpio@22_2", "ENET2_RST_B");

	/* Enable in SW count */
	netc_regulator_enable("regulator-m2-m2-pwr", true);
	netc_regulator_enable("regulator-aqr-stby", true);
	netc_regulator_enable("regulator-mac-stby", true);
	netc_regulator_enable("regulator-aqr-en", true);
	netc_regulator_enable("regulator-mac-en", true);

	/* Disable regulator to have explicit reset to AQR PHY and clock generator */
	udelay(10000);
	netc_regulator_enable("regulator-aqr-stby", false);
	netc_regulator_enable("regulator-mac-stby", false);
	netc_regulator_enable("regulator-aqr-en", false);
	netc_regulator_enable("regulator-mac-en", false);

	udelay(10000);
	netc_regulator_enable("regulator-aqr-stby", true);
	netc_regulator_enable("regulator-mac-stby", true);
}

void lvds_backlight_on(void)
{
	struct udevice *dev;
	int ret;
	u8 reg;

	if (!IS_ENABLED(CONFIG_TARGET_IMX95_15X15_EVK))
		return;

	ret = i2c_get_chip_for_busnum(2, 0x62, 1, &dev);
	if (ret) {
		printf("%s: Cannot find pca9632 led dev\n",
		       __func__);
		return;
	}

	reg = 1;
	dm_i2c_write(dev, 0x1, &reg, 1);

	reg = 5;
	dm_i2c_write(dev, 0x8, &reg, 1);
}

int board_init(void)
{
	int ret;
	ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, true);
	if (ret) {
		printf("SCMI_POWWER_STATE_SET Failed for USB\n");
		return ret;
	}

	imx9_scmi_power_domain_enable(IMX95_PD_DISPLAY, false);
	imx9_scmi_power_domain_enable(IMX95_PD_CAMERA, false);

	netc_init();

	power_on_m7("mx95evkrpmsg");

	lvds_backlight_on();

	camera_init();

	return 0;
}

static int check_mmc_autodetect(void)
{
	char *autodetect_str = env_get("mmcautodetect");

	if (autodetect_str && !strcmp(autodetect_str, "yes"))
		return 1;

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
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw", mmc_map_to_kernel_blk(dev_no));
	env_set("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}

int board_late_init(void)
{
#ifdef CONFIG_TN_PHERIPHERAL_DETECT
	detect_display_panel();
	detect_camera();
#endif
	if (IS_ENABLED(CONFIG_ENV_IS_IN_MMC))
		board_late_mmc_env_init();

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	char *p, *b, *s;
	char *token = NULL;
	int i, ret = 0;
	u64 base[CONFIG_NR_DRAM_BANKS] = {0};
	u64 size[CONFIG_NR_DRAM_BANKS] = {0};

	p = env_get("jh_root_mem");
	if (!p)
		return 0;

	i = 0;
	token = strtok(p, ",");
	while (token) {
		if (i >= CONFIG_NR_DRAM_BANKS) {
			printf("Error: The number of size@base exceeds CONFIG_NR_DRAM_BANKS.\n");
			return -EINVAL;
		}

		b = token;
		s = strsep(&b, "@");
		if (!s) {
			printf("The format of jh_root_mem is size@base[,size@base...].\n");
			return -EINVAL;
		}
		base[i] = simple_strtoull(b, NULL, 16);
		size[i] = simple_strtoull(s, NULL, 16);
		token = strtok(NULL, ",");
		i++;
	}

	ret = fdt_fixup_memory_banks(blob, base, size, CONFIG_NR_DRAM_BANKS);
	if (ret)
		return ret;

	return 0;
}
#endif

static u8 board_get_ddr_code(void)
{
	return (readl(OCRAM_NON_SECURE_BASE_ADDR));
}

int board_phys_sdram_size(phys_size_t *size)
{
	u8 code = board_get_ddr_code();
	pr_info("DDR type code:%d\n", code);
	switch (code) {
		case LPDDR5_4GB:
			*size = PHYS_SDRAM_SIZE + SZ_2G;
			break;
		case LPDDR5_8GB:
			*size = PHYS_SDRAM_SIZE + SZ_6G;
			break;
		case LPDDR5_16GB:
			*size = PHYS_SDRAM_SIZE + PHYS_SDRAM_2_SIZE;
			break;

		case LPDDR4X_2GB:
			*size = PHYS_SDRAM_SIZE;
			break;
		case LPDDR4X_4GB:
			*size = PHYS_SDRAM_SIZE + SZ_2G;
			break;
		case LPDDR4X_8GB:
			*size = PHYS_SDRAM_SIZE + SZ_6G;
			break;

		default:
			puts("Unknown DDR type!!!\n");
			break;
	}

	return 0;
}

void board_quiesce_devices(void)
{
	int ret;
	struct uclass *uc_dev;

	ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, false);
	if (ret) {
		printf("%s: Failed for HSIO MIX: %d\n", __func__, ret);
		return;
	}

	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, false);
	if (ret) {
		printf("%s: Failed for NETC MIX: %d\n", __func__, ret);
		return;
	}

	ret = uclass_get(UCLASS_SPI_FLASH, &uc_dev);
	if (uc_dev)
		ret = uclass_destroy(uc_dev);
	if (ret)
		printf("couldn't remove SPI FLASH devices\n");
}

#if IS_ENABLED(CONFIG_OF_BOARD_FIXUP)
static void disable_fdt_resources(void *fdt)
{
	int i = 0;
	int nodeoff, ret;
	const char *status = "disabled";
	static const char * const dsi_nodes[] = {
		"/soc/bus@42000000/i2c@426b0000",
		"/soc/bus@42000000/i2c@426d0000",
		"/soc/system-controller@4cde0000"
	};

	for (i = 0; i < ARRAY_SIZE(dsi_nodes); i++) {
		nodeoff = fdt_path_offset(fdt, dsi_nodes[i]);
		if (nodeoff > 0) {
set_status:
			ret = fdt_setprop(fdt, nodeoff, "status", status,
					  strlen(status) + 1);
			if (ret == -FDT_ERR_NOSPACE) {
				ret = fdt_increase_size(fdt, 512);
				if (!ret)
					goto set_status;
			}
		}
	}
}

static int board_fix_19x19_evk(void *fdt)
{
	char cfgname[SCMI_MISC_MAX_CFGNAME];
	u32 msel;
	int ret;
	const char *netcfg = "mx95netc";

	ret = scmi_misc_cfginfo(&msel, cfgname);
	if (!ret) {
		debug("SM: %s\n", cfgname);
		if (!strcmp(netcfg, cfgname))
			disable_fdt_resources(fdt);
	}

	return 0;
}

int board_fix_fdt(void *fdt)
{
	/* Remove nodes based on fuses. */
	board_fix_fdt_fuse(fdt);

	return board_fix_19x19_evk(fdt);
}
#endif
#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0;
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

const tn_m2_mdio_device_check_t tn_m2_mdio_device_chk[] = {
	{ "mdio", 0x01, 0x001C, 0xC916, "usxgmii-net10g"},
};
size_t tn_m2_mdio_device_cnt = ARRAY_SIZE(tn_m2_mdio_device_chk);

