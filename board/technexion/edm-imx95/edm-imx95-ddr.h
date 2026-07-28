 // SPDX-License-Identifier: GPL-2.0+
 /*
  * Copyright 2025 TechNexion Ltd.
  *
  * Author: Richard Hu <richard.hu@technexion.com>
  *
  */
#ifndef __EDM_IMX95_DDR_H
#define __EDM_IMX95_DDR_H

/***********************************************
 * BOARD_ID1   BOARD_ID0
 *     0            1       4G LPDDR5
 *     1            0       8G LPDDR5
 *     1            1       16G LPDDR5

 *     0            1       2G LPDDR4X
 *     1            0       4G LPDDR4X
 *     1            1       8G LPDDR4X
************************************************/
enum {
	LPDDR5_4GB = 0x1,
	LPDDR5_8GB = 0x2,
	LPDDR5_16GB = 0x3,
	LPDDR4X_2GB = 0x4,
	LPDDR4X_4GB = 0x5,
	LPDDR4X_8GB = 0x6,

	LPDDR5_UNKNOWN = 0xf,
};

#define OCRAM_NON_SECURE_BASE_ADDR 0x0204C0000

#endif /* __EDM_IMX95_DDR_H */