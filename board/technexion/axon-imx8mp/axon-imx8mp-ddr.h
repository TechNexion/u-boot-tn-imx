 // SPDX-License-Identifier: GPL-2.0+
 /*
  * Copyright 2021 TechNexion Ltd.
  *
  * Author: Richard Hu <richard.hu@technexion.com>
  *
  */
#ifndef __EDM_G_IMX8MP_DDR_H
#define __EDM_G_IMX8MP_DDR_H

/***********************************************
BOARD_ID2    BOARD_ID1   BOARD_ID0
   1            0            1       8G LPDDR4
   0            0            1       6G LPDDR4
   1            1            0       4G LPDDR4
   0            0            0       2G LPDDR4
   1            0            0       1G LPDDR4
************************************************/
enum {
	LPDDR4_8GB = 0x5,
	LPDDR4_6GB = 0x1,
	LPDDR4_4GB = 0x6,
	LPDDR4_2GB = 0x0,
	LPDDR4_1GB = 0x4,
        LPDDR4_UNKNOWN = 0xf,
};

#endif /* __EDM_G_IMX8MP_DDR_H */