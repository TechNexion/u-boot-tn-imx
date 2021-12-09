 // SPDX-License-Identifier: GPL-2.0+
 /*
  * Copyright 2021 TechNexion Ltd.
  *
  * Author: Richard Hu <richard.hu@technexion.com>
  *
  */

#include <i2c.h>
#include <env.h>
#include <dm/uclass.h>
#include "periph_detect.h"

int add_dtoverlay(const char *ov_name)
{
	char *dtoverlay, arr_dtov[64];

	dtoverlay = env_get("dtoverlay");
	if (dtoverlay) {
		strcpy(arr_dtov, dtoverlay);
		if (!strstr(arr_dtov, ov_name)) {
			strcat(arr_dtov, " ");
			strcat(arr_dtov, ov_name);
			env_set("dtoverlay", arr_dtov);
		}
	} else
		env_set("dtoverlay", ov_name);

	return 0;
}


int detect_i2c(struct tn_display const *dev)
{
#if CONFIG_IS_ENABLED(DM_I2C)
	struct udevice *bus, *udev;
	int rc, read_id;

	rc = uclass_get_device_by_seq(UCLASS_I2C, dev->bus, &bus);
	if (rc)
		return rc;

	if (! dm_i2c_probe(bus, dev->addr, 0, &udev)) {
		if (dev->id_reg) {
			read_id = dm_i2c_reg_read(udev, dev->id_reg);
			if (read_id != dev->id) {
				return 0;
			}
		}
		return 1;
	}
	return 0;
#endif
}


int detect_display_panel(void)
{
	int i=0;

	for (i = 0; i < tn_display_count; i++) {
		struct tn_display const *dev = &displays[i];
		if (dev->detect && dev->detect(dev)) {
			add_dtoverlay(dev->ov_name);
			debug("Detect panel - %s !!!\r\n", dev->ov_name);
			break;
		}
	}
	
	return 0;
}



int detect_tevi_camera(void)
{
	return 0;
}
