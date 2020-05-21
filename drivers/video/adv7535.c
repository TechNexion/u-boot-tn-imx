/*
 * Copyright 2020 TechNexion
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <dm/uclass.h>


#define ADV7535_MAIN_I2C_ADDR 0x3d
#define ADV7535_DSI_CEC_I2C_ADDR 0x3c

static int adv7535_i2c_reg_write(struct udevice *dev, uint addr, uint mask, uint data)
{
	uint8_t valb;
	int err;

	if (mask != 0xff) {
		err = dm_i2c_read(dev, addr, &valb, 1);
		if (err)
			return err;

		valb &= ~mask;
		valb |= data;
	} else {
		valb = data;
	}

	err = dm_i2c_write(dev, addr, &valb, 1);
	return err;
}

static int adv7535_i2c_reg_read(struct udevice *dev, uint8_t addr, uint8_t *data)
{
	uint8_t valb;
	int err;

	err = dm_i2c_read(dev, addr, &valb, 1);
	if (err)
		return err;

	*data = (int)valb;
	return 0;
}

void adv7535_init(int i2c_bus)
{
	struct udevice *bus, *main_dev, *cec_dev;
	int ret;
	uint8_t val;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: No bus %d\n", __func__, i2c_bus);
		return;
	}

	ret = dm_i2c_probe(bus, ADV7535_MAIN_I2C_ADDR, 0, &main_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, ADV7535_MAIN_I2C_ADDR, i2c_bus);
		return;
	}

	ret = dm_i2c_probe(bus, ADV7535_DSI_CEC_I2C_ADDR, 0, &cec_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, ADV7535_MAIN_I2C_ADDR, i2c_bus);
		return;
	}

	adv7535_i2c_reg_read(main_dev, 0x00, &val);
	debug("Chip revision: 0x%x (expected: 0x14)\n", val);
	adv7535_i2c_reg_read(cec_dev, 0x00, &val);
	debug("Chip ID MSB: 0x%x (expected: 0x75)\n", val);
	adv7535_i2c_reg_read(cec_dev, 0x01, &val);
	debug("Chip ID LSB: 0x%x (expected: 0x33)\n", val);

	/* Power */
	adv7535_i2c_reg_write(main_dev, 0x41, 0xff, 0x10);
	/* Initialisation (Fixed) Registers */
	adv7535_i2c_reg_write(main_dev, 0x16, 0xff, 0x20);
	adv7535_i2c_reg_write(main_dev, 0x9A, 0xff, 0xE0);
	adv7535_i2c_reg_write(main_dev, 0xBA, 0xff, 0x70);
	adv7535_i2c_reg_write(main_dev, 0xDE, 0xff, 0x82);
	adv7535_i2c_reg_write(main_dev, 0xE4, 0xff, 0x40);
	adv7535_i2c_reg_write(main_dev, 0xE5, 0xff, 0x80);
	adv7535_i2c_reg_write(cec_dev, 0x15, 0xff, 0xD0);
	adv7535_i2c_reg_write(cec_dev, 0x17, 0xff, 0xD0);
	adv7535_i2c_reg_write(cec_dev, 0x24, 0xff, 0x20);
	adv7535_i2c_reg_write(cec_dev, 0x57, 0xff, 0x11);
	/* 4 x DSI Lanes */
	adv7535_i2c_reg_write(cec_dev, 0x1C, 0xff, 0x40);

	/* DSI Pixel Clock Divider */
	adv7535_i2c_reg_write(cec_dev, 0x16, 0xff, 0x18);

	/* Enable Internal Timing Generator */
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);
	/* 1920 x 1080p 60Hz */
	adv7535_i2c_reg_write(cec_dev, 0x28, 0xff, 0x89); /* total width */
	adv7535_i2c_reg_write(cec_dev, 0x29, 0xff, 0x80); /* total width */
	adv7535_i2c_reg_write(cec_dev, 0x2A, 0xff, 0x02); /* hsync */
	adv7535_i2c_reg_write(cec_dev, 0x2B, 0xff, 0xC0); /* hsync */
	adv7535_i2c_reg_write(cec_dev, 0x2C, 0xff, 0x05); /* hfp */
	adv7535_i2c_reg_write(cec_dev, 0x2D, 0xff, 0x80); /* hfp */
	adv7535_i2c_reg_write(cec_dev, 0x2E, 0xff, 0x09); /* hbp */
	adv7535_i2c_reg_write(cec_dev, 0x2F, 0xff, 0x40); /* hbp */

	adv7535_i2c_reg_write(cec_dev, 0x30, 0xff, 0x46); /* total height */
	adv7535_i2c_reg_write(cec_dev, 0x31, 0xff, 0x50); /* total height */
	adv7535_i2c_reg_write(cec_dev, 0x32, 0xff, 0x00); /* vsync */
	adv7535_i2c_reg_write(cec_dev, 0x33, 0xff, 0x50); /* vsync */
	adv7535_i2c_reg_write(cec_dev, 0x34, 0xff, 0x00); /* vfp */
	adv7535_i2c_reg_write(cec_dev, 0x35, 0xff, 0x40); /* vfp */
	adv7535_i2c_reg_write(cec_dev, 0x36, 0xff, 0x02); /* vbp */
	adv7535_i2c_reg_write(cec_dev, 0x37, 0xff, 0x40); /* vbp */

	/* Reset Internal Timing Generator */
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0x8B);
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);

	/* HDMI Output */
	adv7535_i2c_reg_write(main_dev, 0xAF, 0xff, 0x16);
	/* AVI Infoframe - RGB - 16-9 Aspect Ratio */
	adv7535_i2c_reg_write(main_dev, 0x55, 0xff, 0x02);
	adv7535_i2c_reg_write(main_dev, 0x56, 0xff, 0x0);

	/*  GC Packet Enable */
	adv7535_i2c_reg_write(main_dev, 0x40, 0xff, 0x0);
	/*  GC Colour Depth - 24 Bit */
	adv7535_i2c_reg_write(main_dev, 0x4C, 0xff, 0x0);
	/*  Down Dither Output Colour Depth - 8 Bit (default) */
	adv7535_i2c_reg_write(main_dev, 0x49, 0xff, 0x00);

	/* set low refresh 1080p30 */
	adv7535_i2c_reg_write(main_dev, 0x4A, 0xff, 0x80); /*should be 0x80 for 1080p60 and 0x8c for 1080p30*/

	/* HDMI Output Enable */
	adv7535_i2c_reg_write(cec_dev, 0xbe, 0xff, 0x3c);
	adv7535_i2c_reg_write(cec_dev, 0x03, 0xff, 0x89);
}
