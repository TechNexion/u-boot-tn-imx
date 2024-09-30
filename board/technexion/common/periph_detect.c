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
#include <linux/compat.h>
#include "periph_detect.h"
#include <linux/delay.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>

#define ENV_DTOVERLAY		"dtoverlay"
#define SIZE_DTOVERLAY		(256)

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

static int _add_dtoverlay(const char *ov_name)
{
	char *dtoverlay = NULL;
	char arr_dtov[SIZE_DTOVERLAY] = { '\0' };

	if (ov_name == NULL) {
		return(-1);
	}

	dtoverlay = env_get(ENV_DTOVERLAY);
	if (dtoverlay == NULL) {
		printf("%s: adding overlay for %s\n", __FILE__, ov_name);
		snprintf(arr_dtov, SIZE_DTOVERLAY, "%s", ov_name);
	} else if(strstr(dtoverlay, ov_name)) {
		printf("%s: existed overlay name: %s\n", __FILE__, ov_name);
		snprintf(arr_dtov, SIZE_DTOVERLAY, "%s", dtoverlay);
	} else {
		printf("%s: adding overlay for %s\n", __FILE__, ov_name);
		snprintf(arr_dtov, SIZE_DTOVERLAY, "%s %s", dtoverlay, ov_name);
	}

	return(env_set(ENV_DTOVERLAY, arr_dtov));
}

static struct udevice * _check_i2c_dev(int bus_idx, uint addr) {
	struct udevice *bus = NULL;
	struct udevice *i2c_dev = NULL;

	if (uclass_get_device_by_seq(UCLASS_I2C, bus_idx, &bus)) {
		tn_debug("%s: Can't find bus\n", __func__);
		return(NULL);
	}

	dm_i2c_probe(bus, addr, 0, &i2c_dev);

	return(i2c_dev);
}

/* { bus, addr, id_reg, id, ov_name, detect_i2c } */
__weak int detect_i2c(struct tn_display const *dev)
{
#if CONFIG_IS_ENABLED(DM_I2C)
	struct udevice *udev = NULL;
	int read_id = -1;

	tn_debug("Detect func: %s, for overlay: %s\n", __func__, dev->ov_name);
	udev = _check_i2c_dev(dev->bus, dev->addr);
	if (udev != NULL) {
		read_id = dm_i2c_reg_read(udev, dev->id_reg);
		if (read_id == dev->id) {
			return(1);
		}
	}
#endif
	return 0;
}

#define EXC3000_RETRY_TIMES 1
/* { bus, addr, UNUSED, resolution, ov_name, detect_exc3000_i2c } */
__weak int detect_exc3000_i2c(struct tn_display const *dev)
{
#if CONFIG_IS_ENABLED(DM_I2C)
	struct udevice *udev = NULL;
	struct i2c_msg msg_read_frame, msg_vendor_req;
	int vendor_resolution = 0;
	u8 read_vendor_buf[20];
	int retry = 0;

	u8 i2c_buf_read_frame[2] = { 0x27, 0x00 };
	msg_read_frame.addr = 0x2a;
	msg_read_frame.flags = 0;
	msg_read_frame.len = 2;
	msg_read_frame.buf = i2c_buf_read_frame;

	u8 i2c_buf_vendor_req[68] = { 0x67, 0x00, 0x42, 0x00, 0x03, 0x01, 0x45, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	msg_vendor_req.addr = 0x2a;
	msg_vendor_req.flags = 0;
	msg_vendor_req.len = 68;
	msg_vendor_req.buf = i2c_buf_vendor_req;

	tn_debug("Detect func: %s, for overlay: %s\n", __func__, dev->ov_name);
	udev = _check_i2c_dev(dev->bus, dev->addr);
	if (udev == NULL)
		goto skip_detect_exc3000_i2c;
RETRY_EXC3000:
	// clear read_frame
	mdelay(20);
	dm_i2c_xfer(udev, &msg_read_frame, 1);
	mdelay(50);
	dm_i2c_read(udev, 0, read_vendor_buf, 20);
	mdelay(50);

	// send vendor request
	dm_i2c_xfer(udev, &msg_vendor_req, 1);
	mdelay(50);
	dm_i2c_xfer(udev, &msg_read_frame, 1);
	mdelay(50);
	dm_i2c_read(udev, 0, read_vendor_buf, 20);

	// change ascii char to int
	for (int i=16; i< 19; i++) {
		// if not ASCII number, retry or exit.
		if ((read_vendor_buf[i] < 0x30) || (read_vendor_buf[i] > 0x39)){
			if (retry < EXC3000_RETRY_TIMES){
				retry ++;
				tn_debug("%s: i2c transfer error. retry=%d\n",__func__, retry);
				goto RETRY_EXC3000;
			} else {
				tn_debug("%s: i2c transfer error. retry=%d, exit.\n",__func__, retry);
				goto skip_detect_exc3000_i2c;
			}
		}
		tn_debug("detect_exc3000_i2c: read_buf[%d] = %c\n",i , read_vendor_buf[i]);
		vendor_resolution += (read_vendor_buf[i] - 0x30);
		if (i!=18)
			vendor_resolution *= 10;
		tn_debug("detect_exc3000_i2c: vendor_resolution=%u\n", vendor_resolution);
	}
	if (vendor_resolution == dev->id)
		return(1);

#endif
skip_detect_exc3000_i2c:
	return 0;
}

#define VIZIONPANEL_SER_ADDR 0x0C
#define VIZIONPANEL_DES_ADDR 0x2C
#define VIZIONPANEL_SER_INIT_SIZE 14
#define VIZIONPANEL_DES_INIT_SIZE 11
int vizionpanel_init(int vizionpanel_i2c_bus)
{
	struct udevice *udev = NULL;
	int i;

	uint ser_init_offset[VIZIONPANEL_SER_INIT_SIZE] = {
			0x01, 0x03, 0x5B, 0x1E, 0x0F, 0x1E, 0x5B, 0x07, 0x08, 0x70, 0x77, 0x71, 0x78};
	const u8 ser_init_buffer[VIZIONPANEL_SER_INIT_SIZE] = {
			0x0F, 0x9A, 0x20, 0x02, 0x09, 0x01, 0x21, 0x54, 0x54, 0x40, 0x40, 0x70, 0x70};

	uint des_init_offset[VIZIONPANEL_DES_INIT_SIZE] = {
			0x01, 0x34, 0x08, 0x10, 0x1F, 0x09, 0x11, 0x0A, 0x12, 0x26, 0x27};
	const u8 des_init_buffer[VIZIONPANEL_DES_INIT_SIZE] = {
			0x0f, 0x01, 0x54, 0x54, 0x09, 0x40, 0x40, 0x70, 0x70, 0x19, 0x19};

	udev = _check_i2c_dev(vizionpanel_i2c_bus, VIZIONPANEL_SER_ADDR);
	if (udev) {
		// continue write for ser
		for (i = 0 ; i < VIZIONPANEL_SER_INIT_SIZE ; i++) {
			dm_i2c_write(udev, ser_init_offset[i], &ser_init_buffer[i], 1);
			mdelay(50);
		}
	} else {
		tn_debug("%s: Can't find vizionpanel ser device\n", __func__);
		return -1;
	}

	udev = _check_i2c_dev(vizionpanel_i2c_bus, VIZIONPANEL_DES_ADDR);
	if (udev) {
		// continue write for des
		for (i = 0 ; i < VIZIONPANEL_DES_INIT_SIZE ; i++) {
			dm_i2c_write(udev, des_init_offset[i], &des_init_buffer[i], 1);
			mdelay(50);
		}
	} else {
		tn_debug("%s: Can't find vizionpanel des device\n", __func__);
		return -1;
	}

	tn_debug("%s: vizionpanel init success\n", __func__);
	mdelay(100);
	return 0;
}

#ifdef CONFIG_DM_I2C
/**
 * state list of 'vizionpanel_inited_dm':
 * -1 -> not init
 *  0 -> init failed
 *  1 -> init success
 */
int vizionpanel_inited_dm = -1;
#endif

#define VIZIONPANEL_INIT_RETRY 3
/**
 * { bus, addr, UNUSED, resolution, ov_name, detect_vizionpanel_i2c }
 * Please put this detection after all of the 'detect_exc3000_i2c',
 * prevent from the both touches are on the same bus.
 * And reset DSI before all detecion.
 * Do vizionpanel init and retry, and use 'detect_exc3000_i2c' to detect after that.
 */
#define EXC3000_addr 0x2a
#define EDT_FT5X06_addr 0x38
int detect_vizionpanel_i2c(struct tn_display const *dev)
{
#if CONFIG_IS_ENABLED(DM_I2C)
	int i;
	if (vizionpanel_inited_dm == -1) {
		vizionpanel_inited_dm = 0;
		for (i = 0 ; i < VIZIONPANEL_INIT_RETRY ; i++) {
			if (!vizionpanel_init(dev->bus)) {
				vizionpanel_inited_dm = 1;
				break;
			}
			tn_debug("%s: init failed retry %d times\n", __func__, i + 1);
		}
	}

	if (vizionpanel_inited_dm == 0) {
		tn_debug("%s: vizionpanel is not up\n", __func__);
		return 0;
	}

	if ( dev->addr == EXC3000_addr )
		return detect_exc3000_i2c(dev);
	else if ( dev->addr == EDT_FT5X06_addr )
		return detect_i2c(dev);
#endif
	return 0;
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

__weak int detect_display_panel(void)
{
	int i=0;

	tn_debug("%s: Running %s\n", __FILE__, __func__);
	for (i = 0; i < tn_display_count; i++) {
		struct tn_display const *dev = &displays[i];
		if (dev->detect && dev->detect(dev)) {
			_add_dtoverlay(dev->ov_name);
			tn_debug("Detect panel - %s !!!\r\n", dev->ov_name);
			break;
		}
	}

	return 0;
}

static int _detect_camera(const tn_camera_chk_t *list, size_t count) {
	int i = 0, ret = -1;
	char *cam_autodetect = env_get("cameraautodetect");

	tn_debug("%s: Running %s\n", __FILE__, __func__);
	// Default: Do not auto detection
	if ((cam_autodetect == NULL) || (strcmp(cam_autodetect, "yes") != 0)) {
		tn_debug("Camera auto detection is disabled\n");
		return(0);
	}

	if ((list == NULL) || (count <= 0)) {
		tn_debug("%s - Invalid camera list\n", __func__);
		return(-1);
	}

	for (i = 0; i < count; ++i) {
		int j = 0, skip = 0;

		tn_debug("Check %s - i2c#%d 0x%02x\n", list[i].ov_name, list[i].i2c_bus_index, list[i].i2c_addr);
		if (_check_i2c_dev(list[i].i2c_bus_index, list[i].i2c_addr) == NULL) {
			continue;
		}

		// Check exclsive address

		for(j = 0; j < tn_cam_exclusive_i2c_addr_cnt; ++j) {
			if((tn_cam_exclusive_i2c_addr[j] > 0) &&
				(_check_i2c_dev(list[i].i2c_bus_index, tn_cam_exclusive_i2c_addr[j])) != NULL) {
				tn_debug("Exclsived address detected, skip %s\n", list[i].ov_name);
				skip = 1;
				break;
			}
		}

		if(skip) {
			continue;
		}

		_add_dtoverlay(list[i].ov_name);
		ret = 0;
	}

	return(ret);
}

__weak size_t tn_camera_chk_cnt = 0;

// Camera sensor exclusive I2C address
// Since some media devices will be recognized as other media devices,
// adding the i2c address of these special devices to tn_cam_exclusive_i2c_addr
// can avoid this.
//   0x30: VizionLink
//   0x3f: ADV7533/ADV7535
__weak uint8_t tn_cam_exclusive_i2c_addr[] = { 0x30, 0x3f };
__weak size_t tn_cam_exclusive_i2c_addr_cnt = ARRAY_SIZE(tn_cam_exclusive_i2c_addr);

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

	if (tevi_camera_cnt <= 0) {
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
