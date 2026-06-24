/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 NXP
 *
 * Author: Ray Chang <ray.chang@technexion.com>
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
#include "edge-ai-imx95-ddr.h"

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

	/* Enable in SW count */
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
	switch (board_get_ddr_code()) {
	case LPDDR5_4GB:
		*size = PHYS_SDRAM_SIZE + SZ_2G;
		break;
	case LPDDR5_8GB:
		*size = PHYS_SDRAM_SIZE + SZ_6G;
		break;
	case LPDDR5_16GB:
		*size = PHYS_SDRAM_SIZE + PHYS_SDRAM_2_SIZE;
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
