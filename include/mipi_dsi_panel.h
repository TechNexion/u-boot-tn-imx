/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#ifndef __MIPI_DSI_PANEL_H
#define __MIPI_DSI_PANEL_H

void adv7535_init(int i2c_bus);
void hx8363_init(void);
void ili9881c_init(void);
void rm67191_init(void);
void rm68200_init(void);
void sn65dsi84_init(int i2c_bus, const char *panel);

#endif
