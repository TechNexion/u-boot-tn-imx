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

typedef struct tn_camera_chk {
	u8 camera_index;
	u8 i2c_bus_index;
	u8 i2c_addr;
	const char *ov_name;
}tn_camera_chk_t;

extern struct tn_display const displays[];
extern size_t tn_display_count;
__weak extern const tn_camera_chk_t tn_camera_chk[];
extern size_t tn_camera_chk_cnt;
extern uint8_t tn_cam_exclusive_i2c_addr[];
extern size_t tn_cam_exclusive_i2c_addr_cnt;

int tn_debug(const char *fmt, ...);
int detect_i2c(struct tn_display const *dev);
int detect_exc3000_i2c(struct tn_display const *dev);
int detect_vizionpanel_i2c(struct tn_display const *dev);
int detect_exc3000_usb(struct tn_display const *dev);
int detect_display_panel(void);
int detect_camera(void);


// FIXME: Obsolete
//   For compatible old codes, I keep these
//   Please remove these after update all old codes
struct camera_cfg {
	u8 camera_index;
	u8 i2c_bus_index;
	u8 eeprom_i2c_addr;
};
extern const struct camera_cfg tevi_camera[];
extern size_t tevi_camera_cnt;
int detect_tevi_camera(void);

#endif	//#ifndef __PERIPH_DETECT_H__
