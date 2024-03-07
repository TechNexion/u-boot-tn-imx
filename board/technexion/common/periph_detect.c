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
#include <usb.h>
#include "periph_detect.h"
#include <dm/device.h>
#include <dm/uclass-internal.h>

int tn_debug(const char *fmt, ...)
{
#ifdef TN_DEBUG
	va_list args;
	uint i;

	va_start(args, fmt);
	i = vprintf(fmt, args);
	va_end(args);

	return i;
#endif
	return 0;
}

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

#ifdef CONFIG_DM_USB
int usb_inited_dm = -1;
#endif

/* { UNUSED, UNUSED, idVendor, resolution, ov_name, detect_usb } */
__weak int detect_exc3000_usb(struct tn_display const *dev)
{
#ifdef CONFIG_DM_USB
	struct udevice *hub, *child;
	struct uclass *uc_hub;
	struct usb_device *udev;
	int ret, id_num;
	char resolution[4]= "000";

	tn_debug("Detect func: %s, for overlay: %s\n", __func__, dev->ov_name);
	if (usb_inited_dm != 1) {
		usb_init();
		usb_inited_dm = 1;
	}

	id_num = dev->id;
	for (int i = 2 ; i >= 0 ; i-- ) {
		resolution[i] = 0x30 + (id_num % 10);
		id_num /= 10;
	}
	tn_debug("resolution=%s\n", resolution);

	ret = uclass_get(UCLASS_USB_HUB, &uc_hub);
	if (ret) {
		tn_debug("%s: no available hub !!\n", __func__);
		return 0;
	}

	uclass_foreach_dev(hub, uc_hub) {
		if (!device_active(hub))
			continue;
		udev = dev_get_parent_priv(hub);

		for (device_find_first_child(hub, &child);
				child;
				device_find_next_child(&child))
		{
			if (!device_active(hub))
				continue;

			udev = dev_get_parent_priv(child);
			tn_debug("udev->prod=%s\n", udev->prod);
			if (strstr(udev->prod, resolution) != NULL)
				return 1;
		}
	}
#endif /* !define(CONFIG_DM_USB) */
	return 0;
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
