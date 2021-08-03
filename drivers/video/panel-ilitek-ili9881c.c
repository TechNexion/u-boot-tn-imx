// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017-2018, Bootlin
 */

#include <common.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <linux/err.h>
#include <power/regulator.h>
#include <linux/delay.h>

#include <malloc.h>
#include <asm/io.h>
#include <errno.h>



struct ili9881c_panel_priv {
	struct udevice *power;
	struct gpio_desc backlight;
	struct gpio_desc reset;
	unsigned int timing_mode;		/* 0 */
	unsigned int lanes;			/* 4 */
	enum mipi_dsi_pixel_format format;	/* MIPI_DSI_FMT_RGB888 */
	unsigned long mode_flags;		/* MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE | MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_VIDEO_HSE */
};

static const struct display_timing default_timing = {
	.pixelclock.typ	= 62000000,
	.hactive.typ	= 720,
	.hfront_porch.typ	= 10,
	.hback_porch.typ	= 30,
	.hsync_len.typ	= 20,
	.vactive.typ	= 1280,
	.vfront_porch.typ	= 10,
	.vback_porch.typ	= 20,
	.vsync_len.typ	= 10,
};

enum ili9881c_op {
	ILI9881C_SWITCH_PAGE,
	ILI9881C_COMMAND,
};

struct ili9881c_instr {
	enum ili9881c_op	op;

	union arg {
		struct cmd {
			u8	cmd;
			u8	data;
		} cmd;
		u8	page;
	} arg;
};

#define ILI9881C_SWITCH_PAGE_INSTR(_page)	\
	{					\
		.op = ILI9881C_SWITCH_PAGE,	\
		.arg = {			\
			.page = (_page),	\
		},				\
	}

#define ILI9881C_COMMAND_INSTR(_cmd, _data)		\
	{						\
		.op = ILI9881C_COMMAND,		\
		.arg = {				\
			.cmd = {			\
				.cmd = (_cmd),		\
				.data = (_data),	\
			},				\
		},					\
	}

static const struct ili9881c_instr ili9881c_init[] = {
	ILI9881C_SWITCH_PAGE_INSTR(3),
	ILI9881C_COMMAND_INSTR(0x01, 0x00),
	ILI9881C_COMMAND_INSTR(0x02, 0x00),
	ILI9881C_COMMAND_INSTR(0x03, 0x73),
	ILI9881C_COMMAND_INSTR(0x04, 0x00),
	ILI9881C_COMMAND_INSTR(0x05, 0x00),
	ILI9881C_COMMAND_INSTR(0x06, 0x0a),
	ILI9881C_COMMAND_INSTR(0x07, 0x00),
	ILI9881C_COMMAND_INSTR(0x08, 0x00),
	ILI9881C_COMMAND_INSTR(0x09, 0x01),
	ILI9881C_COMMAND_INSTR(0x0a, 0x00),
	ILI9881C_COMMAND_INSTR(0x0b, 0x00),
	ILI9881C_COMMAND_INSTR(0x0c, 0x01),
	ILI9881C_COMMAND_INSTR(0x0d, 0x00),
	ILI9881C_COMMAND_INSTR(0x0e, 0x00),
	ILI9881C_COMMAND_INSTR(0x0f, 0x14),
	ILI9881C_COMMAND_INSTR(0x10, 0x14),
	ILI9881C_COMMAND_INSTR(0x11, 0x00),
	ILI9881C_COMMAND_INSTR(0x12, 0x00),
	ILI9881C_COMMAND_INSTR(0x13, 0x00),
	ILI9881C_COMMAND_INSTR(0x14, 0x00),
	ILI9881C_COMMAND_INSTR(0x15, 0x00),
	ILI9881C_COMMAND_INSTR(0x16, 0x00),
	ILI9881C_COMMAND_INSTR(0x17, 0x00),
	ILI9881C_COMMAND_INSTR(0x18, 0x00),
	ILI9881C_COMMAND_INSTR(0x19, 0x00),
	ILI9881C_COMMAND_INSTR(0x1a, 0x00),
	ILI9881C_COMMAND_INSTR(0x1b, 0x00),
	ILI9881C_COMMAND_INSTR(0x1c, 0x00),
	ILI9881C_COMMAND_INSTR(0x1d, 0x00),
	ILI9881C_COMMAND_INSTR(0x1e, 0x40),
	ILI9881C_COMMAND_INSTR(0x1f, 0x80),
	ILI9881C_COMMAND_INSTR(0x20, 0x06),
	ILI9881C_COMMAND_INSTR(0x21, 0x01),
	ILI9881C_COMMAND_INSTR(0x22, 0x00),
	ILI9881C_COMMAND_INSTR(0x23, 0x00),
	ILI9881C_COMMAND_INSTR(0x24, 0x00),
	ILI9881C_COMMAND_INSTR(0x25, 0x00),
	ILI9881C_COMMAND_INSTR(0x26, 0x00),
	ILI9881C_COMMAND_INSTR(0x27, 0x00),
	ILI9881C_COMMAND_INSTR(0x28, 0x33),
	ILI9881C_COMMAND_INSTR(0x29, 0x03),
	ILI9881C_COMMAND_INSTR(0x2a, 0x00),
	ILI9881C_COMMAND_INSTR(0x2b, 0x00),
	ILI9881C_COMMAND_INSTR(0x2c, 0x00),
	ILI9881C_COMMAND_INSTR(0x2d, 0x00),
	ILI9881C_COMMAND_INSTR(0x2e, 0x00),
	ILI9881C_COMMAND_INSTR(0x2f, 0x00),
	ILI9881C_COMMAND_INSTR(0x30, 0x00),
	ILI9881C_COMMAND_INSTR(0x31, 0x00),
	ILI9881C_COMMAND_INSTR(0x32, 0x00),
	ILI9881C_COMMAND_INSTR(0x33, 0x00),
	ILI9881C_COMMAND_INSTR(0x34, 0x04),
	ILI9881C_COMMAND_INSTR(0x35, 0x00),
	ILI9881C_COMMAND_INSTR(0x36, 0x00),
	ILI9881C_COMMAND_INSTR(0x37, 0x00),
	ILI9881C_COMMAND_INSTR(0x38, 0x78),
	ILI9881C_COMMAND_INSTR(0x39, 0x00),
	ILI9881C_COMMAND_INSTR(0x3a, 0x40),
	ILI9881C_COMMAND_INSTR(0x3b, 0x40),
	ILI9881C_COMMAND_INSTR(0x3c, 0x00),
	ILI9881C_COMMAND_INSTR(0x3d, 0x00),
	ILI9881C_COMMAND_INSTR(0x3e, 0x00),
	ILI9881C_COMMAND_INSTR(0x3f, 0x00),
	ILI9881C_COMMAND_INSTR(0x40, 0x00),
	ILI9881C_COMMAND_INSTR(0x41, 0x00),
	ILI9881C_COMMAND_INSTR(0x42, 0x00),
	ILI9881C_COMMAND_INSTR(0x43, 0x00),
	ILI9881C_COMMAND_INSTR(0x44, 0x00),
	ILI9881C_COMMAND_INSTR(0x50, 0x01),
	ILI9881C_COMMAND_INSTR(0x51, 0x23),
	ILI9881C_COMMAND_INSTR(0x52, 0x45),
	ILI9881C_COMMAND_INSTR(0x53, 0x67),
	ILI9881C_COMMAND_INSTR(0x54, 0x89),
	ILI9881C_COMMAND_INSTR(0x55, 0xab),
	ILI9881C_COMMAND_INSTR(0x56, 0x01),
	ILI9881C_COMMAND_INSTR(0x57, 0x23),
	ILI9881C_COMMAND_INSTR(0x58, 0x45),
	ILI9881C_COMMAND_INSTR(0x59, 0x67),
	ILI9881C_COMMAND_INSTR(0x5a, 0x89),
	ILI9881C_COMMAND_INSTR(0x5b, 0xab),
	ILI9881C_COMMAND_INSTR(0x5c, 0xcd),
	ILI9881C_COMMAND_INSTR(0x5d, 0xef),
	ILI9881C_COMMAND_INSTR(0x5e, 0x11),
	ILI9881C_COMMAND_INSTR(0x5f, 0x01),
	ILI9881C_COMMAND_INSTR(0x60, 0x00),
	ILI9881C_COMMAND_INSTR(0x61, 0x15),
	ILI9881C_COMMAND_INSTR(0x62, 0x14),
	ILI9881C_COMMAND_INSTR(0x63, 0x0e),
	ILI9881C_COMMAND_INSTR(0x64, 0x0f),
	ILI9881C_COMMAND_INSTR(0x65, 0x0c),
	ILI9881C_COMMAND_INSTR(0x66, 0x0d),
	ILI9881C_COMMAND_INSTR(0x67, 0x06),
	ILI9881C_COMMAND_INSTR(0x68, 0x02),
	ILI9881C_COMMAND_INSTR(0x69, 0x07),
	ILI9881C_COMMAND_INSTR(0x6a, 0x02),
	ILI9881C_COMMAND_INSTR(0x6b, 0x02),
	ILI9881C_COMMAND_INSTR(0x6c, 0x02),
	ILI9881C_COMMAND_INSTR(0x6d, 0x02),
	ILI9881C_COMMAND_INSTR(0x6e, 0x02),
	ILI9881C_COMMAND_INSTR(0x6f, 0x02),
	ILI9881C_COMMAND_INSTR(0x70, 0x02),
	ILI9881C_COMMAND_INSTR(0x71, 0x02),
	ILI9881C_COMMAND_INSTR(0x72, 0x02),
	ILI9881C_COMMAND_INSTR(0x73, 0x02),
	ILI9881C_COMMAND_INSTR(0x74, 0x02),
	ILI9881C_COMMAND_INSTR(0x75, 0x01),
	ILI9881C_COMMAND_INSTR(0x76, 0x00),
	ILI9881C_COMMAND_INSTR(0x77, 0x14),
	ILI9881C_COMMAND_INSTR(0x78, 0x15),
	ILI9881C_COMMAND_INSTR(0x79, 0x0e),
	ILI9881C_COMMAND_INSTR(0x7a, 0x0f),
	ILI9881C_COMMAND_INSTR(0x7b, 0x0c),
	ILI9881C_COMMAND_INSTR(0x7c, 0x0d),
	ILI9881C_COMMAND_INSTR(0x7d, 0x06),
	ILI9881C_COMMAND_INSTR(0x7e, 0x02),
	ILI9881C_COMMAND_INSTR(0x7f, 0x07),
	ILI9881C_COMMAND_INSTR(0x80, 0x02),
	ILI9881C_COMMAND_INSTR(0x81, 0x02),
	ILI9881C_COMMAND_INSTR(0x82, 0x02),
	ILI9881C_COMMAND_INSTR(0x83, 0x02),
	ILI9881C_COMMAND_INSTR(0x84, 0x02),
	ILI9881C_COMMAND_INSTR(0x85, 0x02),
	ILI9881C_COMMAND_INSTR(0x86, 0x02),
	ILI9881C_COMMAND_INSTR(0x87, 0x02),
	ILI9881C_COMMAND_INSTR(0x88, 0x02),
	ILI9881C_COMMAND_INSTR(0x89, 0x02),
	ILI9881C_COMMAND_INSTR(0x8A, 0x02),

	ILI9881C_SWITCH_PAGE_INSTR(4),
	ILI9881C_COMMAND_INSTR(0x00, 0x80),
	ILI9881C_COMMAND_INSTR(0x6C, 0x15),
	ILI9881C_COMMAND_INSTR(0x6E, 0x2a),
	ILI9881C_COMMAND_INSTR(0x6F, 0x33),
	ILI9881C_COMMAND_INSTR(0x3A, 0x94),
	ILI9881C_COMMAND_INSTR(0x8D, 0x1a),
	ILI9881C_COMMAND_INSTR(0x87, 0xba),
	ILI9881C_COMMAND_INSTR(0x26, 0x76),
	ILI9881C_COMMAND_INSTR(0xB2, 0xd1),
	ILI9881C_COMMAND_INSTR(0xB5, 0x06),

	ILI9881C_SWITCH_PAGE_INSTR(1),
	ILI9881C_COMMAND_INSTR(0x22, 0x0A),
	ILI9881C_COMMAND_INSTR(0x31, 0x00),
	ILI9881C_COMMAND_INSTR(0x53, 0x8C),
	ILI9881C_COMMAND_INSTR(0x55, 0x8F),
	ILI9881C_COMMAND_INSTR(0x50, 0xc0),
	ILI9881C_COMMAND_INSTR(0x51, 0xc0),
	ILI9881C_COMMAND_INSTR(0x60, 0x08),
	ILI9881C_COMMAND_INSTR(0xA0, 0x08),
	ILI9881C_COMMAND_INSTR(0xA1, 0x19),
	ILI9881C_COMMAND_INSTR(0xA2, 0x26),
	ILI9881C_COMMAND_INSTR(0xA3, 0x1a),
	ILI9881C_COMMAND_INSTR(0xA4, 0x1d),
	ILI9881C_COMMAND_INSTR(0xA5, 0x2c),
	ILI9881C_COMMAND_INSTR(0xA6, 0x21),
	ILI9881C_COMMAND_INSTR(0xA7, 0x22),
	ILI9881C_COMMAND_INSTR(0xA8, 0x7c),
	ILI9881C_COMMAND_INSTR(0xA9, 0x21),
	ILI9881C_COMMAND_INSTR(0xAA, 0x2e),
	ILI9881C_COMMAND_INSTR(0xAB, 0x66),
	ILI9881C_COMMAND_INSTR(0xAC, 0x1C),
	ILI9881C_COMMAND_INSTR(0xAD, 0x18),
	ILI9881C_COMMAND_INSTR(0xAE, 0x4e),
	ILI9881C_COMMAND_INSTR(0xAF, 0x1a),
	ILI9881C_COMMAND_INSTR(0xB0, 0x22),
	ILI9881C_COMMAND_INSTR(0xB1, 0x49),
	ILI9881C_COMMAND_INSTR(0xB2, 0x56),
	ILI9881C_COMMAND_INSTR(0xB3, 0x39),
	ILI9881C_COMMAND_INSTR(0xC0, 0x08),
	ILI9881C_COMMAND_INSTR(0xC1, 0x1a),
	ILI9881C_COMMAND_INSTR(0xC2, 0x26),
	ILI9881C_COMMAND_INSTR(0xC3, 0x0b),
	ILI9881C_COMMAND_INSTR(0xC4, 0x0e),
	ILI9881C_COMMAND_INSTR(0xC5, 0x24),
	ILI9881C_COMMAND_INSTR(0xC6, 0x18),
	ILI9881C_COMMAND_INSTR(0xC7, 0x1b),
	ILI9881C_COMMAND_INSTR(0xC8, 0x85),
	ILI9881C_COMMAND_INSTR(0xC9, 0x17),
	ILI9881C_COMMAND_INSTR(0xCA, 0x23),
	ILI9881C_COMMAND_INSTR(0xCB, 0x79),
	ILI9881C_COMMAND_INSTR(0xCC, 0x1C),
	ILI9881C_COMMAND_INSTR(0xCD, 0x1f),
	ILI9881C_COMMAND_INSTR(0xCE, 0x50),
	ILI9881C_COMMAND_INSTR(0xCF, 0x2d),
	ILI9881C_COMMAND_INSTR(0xD0, 0x31),
	ILI9881C_COMMAND_INSTR(0xD1, 0x49),
	ILI9881C_COMMAND_INSTR(0xD2, 0x57),
	ILI9881C_COMMAND_INSTR(0xD3, 0x39),

	ILI9881C_SWITCH_PAGE_INSTR(0),
};

/*
 * The panel seems to accept some private DCS commands that map
 * directly to registers.
 *
 * It is organised by page, with each page having its own set of
 * registers, and the first page looks like it's holding the standard
 * DCS commands.
 *
 * So before any attempt at sending a command or data, we have to be
 * sure if we're in the right page or not.
 */
static int ili9881c_switch_page(struct mipi_dsi_device *dsi, u8 page)
{
	u8 buf[4] = { 0xff, 0x98, 0x81, page };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(dsi, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return 0;
}

static int ili9881c_send_cmd_data(struct mipi_dsi_device *dsi, u8 cmd, u8 data)
{
	u8 buf[2] = { cmd, data };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(dsi, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return 0;
}

static int ili9881c_enable(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *dsi = plat->device;
	unsigned int i;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	for (i = 0; i < ARRAY_SIZE(ili9881c_init); i++) {
		const struct ili9881c_instr *instr = &ili9881c_init[i];

		if (instr->op == ILI9881C_SWITCH_PAGE)
			ret = ili9881c_switch_page(dsi, instr->arg.page);
		else if (instr->op == ILI9881C_COMMAND)
			ret = ili9881c_send_cmd_data(dsi, instr->arg.cmd.cmd,
						      instr->arg.cmd.data);

		if (ret)
			return ret;
	}

	ret = ili9881c_switch_page(dsi, 0);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret)
		return ret;

	mdelay(120);

	mipi_dsi_dcs_set_display_on(dsi);

	return 0;
}

static int ili9881c_panel_get_display_timing(struct udevice *dev,
					    struct display_timing *timings)
{
	struct ili9881c_panel_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *device = plat->device;

	memcpy(timings, &default_timing, sizeof(*timings));

	/* fill characteristics of DSI data link */
	if (device) {
		device->lanes = priv->lanes;
		device->format = priv->format;
		device->mode_flags = priv->mode_flags;
	}

	return 0;
}

static int ili9881c_panel_enable_backlight(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *device = plat->device;
	/* struct udevice *bus, *i2c_dev = NULL; */
	/* int val; */
	int ret;

	/* MIPI_BL_EN */
	/*
	ret = uclass_get_device_by_seq(UCLASS_I2C, EXPANSION_IC_I2C_BUS, &bus);
	if (!ret) {
		ret = dm_i2c_probe(bus, EXPANSION_IC_I2C_ADDR, 0, &i2c_dev);
		if (!ret) {
			dm_i2c_read(i2c_dev, 6, &val, 1);
			val &= 0xfd;
			dm_i2c_write(i2c_dev, 6, &val, 1);
		}
	}
	*/

	ret = mipi_dsi_attach(device);
	if (ret < 0)
		return ret;

	return ili9881c_enable(dev);
}

static int ili9881c_panel_ofdata_to_platdata(struct udevice *dev)
{
	struct ili9881c_panel_priv *priv = dev_get_priv(dev);
	u32 video_mode;
	int ret;

	priv->format = MIPI_DSI_FMT_RGB888;
	priv->mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = dev_read_u32(dev, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			priv->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			priv->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			printf("invalid video mode %d\n", video_mode);
			break;
		}
	}

	ret = dev_read_u32(dev, "dsi-lanes", &priv->lanes);
	if (ret) {
		printf("Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	ret =  uclass_get_device_by_phandle(UCLASS_REGULATOR, dev, "power-supply", &priv->power);
	if (ret && ret != -ENOENT) {
		printf("Warning: cannot get power supply (%d)\n", ret);
		return ret;
	}

	ret = gpio_request_by_name(dev, "reset-gpio", 0, &priv->reset, GPIOD_IS_OUT);
	if (ret && ret != -ENOENT) {
		printf("Warning: cannot get reset GPIO (%d)\n", ret);
		return ret;
	}

	ret = gpio_request_by_name(dev, "backlight-gpio", 0, &priv->backlight, GPIOD_IS_OUT);
	if (ret && ret != -ENOENT) {
		printf("Warning: cannot get backlight GPIO (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int ili9881c_panel_probe(struct udevice *dev)
{
	struct ili9881c_panel_priv *priv = dev_get_priv(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DM_REGULATOR) && priv->power) {
		ret = regulator_set_enable(priv->power, true);
		if (ret)
			return ret;
		mdelay(5);
	}

	/* reset panel - reset gpio pin is GPIO_ACTIVE_LOW */
	ret = dm_gpio_set_value(&priv->reset, true);
	if (ret)
		printf("reset gpio fails to set true\n");
	mdelay(20);
	ret = dm_gpio_set_value(&priv->reset, false);
	if (ret)
		printf("reset gpio fails to set false\n");
	mdelay(20);

	ret = dm_gpio_set_value(&priv->backlight, 0);
	if (ret)
		printf("backlight gpio fails to set on\n");

	return 0;
}

static int ili9881c_panel_remove(struct udevice *dev)
{
	struct ili9881c_panel_priv *priv = dev_get_priv(dev);

	dm_gpio_set_value(&priv->reset, true);

	return 0;
}

static const struct panel_ops ili9881c_panel_ops = {
	.enable_backlight = ili9881c_panel_enable_backlight,
	.get_display_timing = ili9881c_panel_get_display_timing,
};

static const struct udevice_id ili9881c_panel_ids[] = {
	{ .compatible = "ilitek,ili9881c" },
	{ }
};

U_BOOT_DRIVER(ili9881c_panel) = {
	.name				= "ili9881c_panel",
	.id				= UCLASS_PANEL,
	.of_match			= ili9881c_panel_ids,
	.ops				= &ili9881c_panel_ops,
	.of_to_plat			= ili9881c_panel_ofdata_to_platdata,
	.probe				= ili9881c_panel_probe,
	.remove				= ili9881c_panel_remove,
	.plat_auto			= sizeof(struct mipi_dsi_panel_plat),
	.priv_auto			= sizeof(struct ili9881c_panel_priv),
};
