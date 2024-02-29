#include <common.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <linux/err.h>
#include <asm/arch/clock.h>
#include <backlight.h>
#include <video.h>

#include <linux/delay.h>

struct sn65dsi84_panel_priv {
	struct udevice *reg;
	struct udevice *backlight;
	struct gpio_desc enable;
	unsigned int lanes;
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
	unsigned int addr;
};

static const struct display_timing default_timing = {
	.pixelclock.typ		= 69500000,
	.hactive.typ		= 1280,
	.hfront_porch.typ	= 40,
	.hback_porch.typ	= 40,
	.hsync_len.typ		= 80,
	.vactive.typ		= 800,
	.vfront_porch.typ	= 3,
	.vback_porch.typ	= 10,
	.vsync_len.typ		= 10,
};

static int sn65dsi84_i2c_reg_write(struct udevice *dev, uint8_t addr, uint8_t data)
{
	int err;

	err = dm_i2c_write(dev,addr, &data,1);

	return err;
}

static int sn65dsi84_i2c_reg_read(struct udevice *dev, uint8_t addr, uint8_t *data)
{
	uint8_t valb;
	int err;

	err = dm_i2c_read(dev, addr,&valb,1);

	if(err) {
		return err;
	}

	*data = (int)valb;

	return 0;
}

static int sn65dsi84_enable(struct udevice *dev)
{
	uint8_t address;
	uint8_t data = 0;
	int err;
	int len;
	int chipid[] = {0x35,0x38,0x49,0x53,0x44,0x20,0x20,0x20,0x01}; /*58isd*/

	uint8_t reg[][2] = {
		{0x09,0x00},
		{0x0A,0x05},
		{0x0B,0x10},
		{0x0D,0x00},
		{0x10,0x26},
		{0x11,0x00},
		{0x12,0x2a},
		{0x13,0x00},
		{0x18,0x78},
		{0x19,0x00},
		{0x1A,0x03},
		{0x1B,0x00},
		{0x20,0x00},
		{0x21,0x05},
		{0x22,0x00},
		{0x23,0x00},
		{0x24,0x00},
		{0x25,0x00},
		{0x26,0x00},
		{0x27,0x00},
		{0x28,0x21},
		{0x29,0x00},
		{0x2A,0x00},
		{0x2B,0x00},
		{0x2C,0x50},
		{0x2D,0x00},
		{0x2E,0x00},
		{0x2F,0x00},
		{0x30,0x0a},
		{0x31,0x00},
		{0x32,0x00},
		{0x33,0x00},
		{0x34,0x28},
		{0x35,0x00},
		{0x36,0x00},
		{0x37,0x00},
		{0x39,0x00},
		{0x3A,0x00},
		{0x3B,0x00},
		{0x3C,0x00},
		{0x3D,0x00},
		{0x3E,0x00},
		{0x0D,0x01},

	};

	len = sizeof(reg) / sizeof(reg[0]);

	for(int i = 0; i < sizeof(chipid)/sizeof(int); i++) {
		address = (char)i;
		err = sn65dsi84_i2c_reg_read(dev,address,&data);
		if(err < 0) {
			printf("Failed to read chip id \n");
			return err;
		}
		if(data != chipid[i]) {
			printf("chip id is not correct \n");
			return err;
		}
	}
	debug("Valid sn65dsi84 chip id \n");


	for(int i = 0; i < len; i++) {
		sn65dsi84_i2c_reg_write(dev, reg[i][0],reg[i][1]);
		if(err < 0) {
			return err;
		}
	}
	mdelay(10);
	sn65dsi84_i2c_reg_write(dev,0x09,1);
	debug("Soft reset to default \n");

	return 0;
}


static int sn65dsi84_panel_enable_backlight(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *device = plat->device;

	int ret;

	ret = mipi_dsi_attach(device);
	if(ret < 0) {
		return ret;
	}

	return 0;
}

static int sn65dsi84_panel_get_display_timing(struct udevice *dev, struct display_timing *timings)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *device = plat->device;
	struct sn65dsi84_panel_priv *priv = dev_get_priv(dev);

	memcpy(timings, &default_timing, sizeof(*timings));

	if(device) {
		device->lanes = priv->lanes;
		device->format = priv->format;
		device->mode_flags = priv->mode_flags;
	}
	return 0;
}

static int sn65dsi84_panel_probe(struct udevice *dev)
{
	struct sn65dsi84_panel_priv *priv = dev_get_priv(dev);

	debug("%s : sn65dsi84 probe start \n\r",__func__);

	/* set low first to shutdown / reset the sn65dsi84 */
	dm_gpio_set_value(&priv->enable, false);
	mdelay(10);
	/* sets high to enable sn65dsi84 */
	dm_gpio_set_value(&priv->enable, true);
	mdelay(10);

	sn65dsi84_enable(dev);

	return 0;
}

static int sn65dsi84_panel_of_to_plat(struct udevice *dev)
{
	struct sn65dsi84_panel_priv *priv = dev_get_priv(dev);
	int ret;

	priv->format = MIPI_DSI_FMT_RGB888;

	priv->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_VIDEO_SYNC_PULSE; /* dsi-flags = <0x0007>*/

	ret = dev_read_u32(dev, "dsi-lanes", &priv->lanes);
	if (ret) {
		printf("Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	ret = gpio_request_by_name(dev, "enable-gpios", 0, &priv->enable, GPIOD_IS_OUT);
	if (ret && ret != -ENOENT) {
		printf("Warning: cannot get enable GPIO (%d)\n", ret);
		return ret;
	}

	return 0;
}

static const struct panel_ops sn65dsi84_panel_ops = {
	.enable_backlight = sn65dsi84_panel_enable_backlight,
	.get_display_timing = sn65dsi84_panel_get_display_timing,
};


static const struct udevice_id sn65dsi84_panel_id[] = {
	{ .compatible = "ti,sn65dsi84"},
	{}
};

U_BOOT_DRIVER(sn65dsi84_panel) = {
	.name = "sn65dsi84_panel",
	.id = UCLASS_PANEL,
	.of_match = sn65dsi84_panel_id,
	.ops = &sn65dsi84_panel_ops,
	.of_to_plat = sn65dsi84_panel_of_to_plat,
	.probe = sn65dsi84_panel_probe,
	.plat_auto = sizeof(struct mipi_dsi_panel_plat),
	.priv_auto = sizeof(struct sn65dsi84_panel_priv),
};
