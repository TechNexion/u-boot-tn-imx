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
#include <linux/compat.h>
#include "periph_detect.h"

#define ENV_DTOVERLAY		"dtoverlay"
#define SIZE_DTOVERLAY		(256)

static int _add_dtoverlay(const char *ov_name)
{
	char *dtoverlay = NULL;
	char arr_dtov[SIZE_DTOVERLAY] = { '\0' };

	if (ov_name == NULL) {
		return(-1);
	}

	dtoverlay = env_get(ENV_DTOVERLAY);
	if (dtoverlay == NULL) {
		sprintf(arr_dtov, "%s", ov_name);
	} else {
		sprintf(arr_dtov, "%s %s", dtoverlay, ov_name);
	}

	return(env_set(ENV_DTOVERLAY, arr_dtov));
}

static int _ov_in_dtoverlay(const char *ov) {
	return((ov != NULL) && (strstr(env_get(ENV_DTOVERLAY), ov) != NULL));
}

static struct udevice * _check_i2c_dev(int bus_idx, uint addr) {
	struct udevice *bus = NULL;
	struct udevice *i2c_dev = NULL;

	if (uclass_get_device_by_seq(UCLASS_I2C, bus_idx, &bus)) {
		printf("%s: Can't find bus\n", __func__);
		return(NULL);
	}

	dm_i2c_probe(bus, addr, 0, &i2c_dev);

	return(i2c_dev);
}

__weak int detect_i2c(struct tn_display const *dev)
{
#if CONFIG_IS_ENABLED(DM_I2C)
	struct udevice *udev = NULL;
	int read_id = -1;

	udev = _check_i2c_dev(dev->bus, dev->addr);
	if (udev != NULL) {
		if (dev->id_reg == 0) {
			return(1);
		}

		read_id = dm_i2c_reg_read(udev, dev->id_reg);
		if (read_id == dev->id) {
			return(1);
		}
	}
#endif
	return 0;
}

__weak int detect_display_panel(void)
{
	int i=0;

	for (i = 0; i < tn_display_count; i++) {
		struct tn_display const *dev = &displays[i];
		if (dev->detect && dev->detect(dev)) {
			_add_dtoverlay(dev->ov_name);
			debug("Detect panel - %s !!!\r\n", dev->ov_name);
			break;
		}
	}

	return 0;
}

static int _detect_camera(const tn_camera_chk_t *list, size_t count) {
	int i = 0, ret = -1;
	char *cam_autodetect = env_get("cameraautodetect");

	// Default: Do not auto detection
	if ((cam_autodetect == NULL) || (strcmp(cam_autodetect, "yes") != 0)) {
		printf("Camera auto detection is disabled\n");
		return(0);
	}

	if ((list == NULL) || (count <= 0)) {
		//printf("%s - Invalid camera list\n", __func__);
		return(-1);
	}

	for (i = 0; i < count; ++i) {
		printf("Check %s - i2c#%d 0x%02x\n", list[i].ov_name, list[i].i2c_bus_index, list[i].i2c_addr);
		if (_ov_in_dtoverlay(list[i].ov_name) ||
			(_check_i2c_dev(list[i].i2c_bus_index, list[i].i2c_addr) == NULL)) {
			continue;
		}

		// Check VizionLink
		if((tn_vizionlink_i2c_addr > 0) &&
			(_check_i2c_dev(list[i].i2c_bus_index, tn_vizionlink_i2c_addr)) != NULL) {
			printf("VizionLink detected, skip %s\n", list[i].ov_name);
			continue;
		}

		_add_dtoverlay(list[i].ov_name);
		ret = 0;
	}

	return(ret);
}

__weak const tn_camera_chk_t tn_camera_chk[] = {};
__weak size_t tn_camera_chk_cnt = 0;
__weak int tn_vizionlink_i2c_addr = -1;

__weak int detect_camera(void) {
	return(_detect_camera(tn_camera_chk, tn_camera_chk_cnt));
}


// FIXME: Obsolete
//   For compatible old codes, I convert tevi_camera to tn_camera_chk
//   Please remove this after update all old codes
__weak const struct camera_cfg tevi_camera[] = {};
__weak size_t tevi_camera_cnt = 0;

__weak int detect_tevi_camera(void) {
	int i = 0, ret = -1;
	tn_camera_chk_t *tevi_cam = NULL;

	if ((tevi_camera == NULL) || (tevi_camera_cnt <= 0)) {
		return(-1);
	}

	tevi_cam = kzalloc(sizeof(tn_camera_chk_t) * tevi_camera_cnt, GFP_KERNEL);
	if (tevi_cam == NULL) {
		goto _exit;
	}

	for (i = 0; i < tevi_camera_cnt; i++) {
		tevi_cam[i].camera_index = tevi_camera[i].camera_index;
		tevi_cam[i].i2c_bus_index = tevi_camera[i].i2c_bus_index;
		tevi_cam[i].i2c_addr = tevi_camera[i].eeprom_i2c_addr;
		tevi_cam[i].ov_name = "tevi-ov5640";
	}

	ret = _detect_camera((const tn_camera_chk_t *)tevi_cam, tevi_camera_cnt);

	free(tevi_cam);

_exit:
	return(ret);
}
