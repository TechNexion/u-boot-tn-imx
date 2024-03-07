// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 TechNexion Ltd.
 *
 * Author: Richard Hu <richard.hu@technexion.com>
 *
 */

#ifndef __PERIPH_DETECT_H__
#define __PERIPH_DETECT_H__

struct tn_display {
	int	bus;
	int	addr;
    int id_reg;
    int id;
    const char *ov_name;
    int	(*detect)(struct tn_display const *dev);
};

struct camera_cfg {
       u8 camera_index;
       u8 i2c_bus_index;
       u8 eeprom_i2c_addr;
};

extern struct tn_display const displays[];
extern size_t tn_display_count;

int tn_debug(const char *fmt, ...);
int detect_i2c(struct tn_display const *dev);
int detect_exc3000_usb(struct tn_display const *dev);
int detect_display_panel(void);
#endif
