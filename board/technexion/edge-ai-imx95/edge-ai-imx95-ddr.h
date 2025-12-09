 // SPDX-License-Identifier: GPL-2.0+
 /*
  * Copyright 2025 TechNexion Ltd.
  *
  * Author: Ray Chang <ray.chang@technexion.com>
  *
  */
#ifndef __EDGE_AI_IMX95_DDR_H
#define __EDGE_AI_IMX95_DDR_H

/***********************************************
 * BOARD_ID1   BOARD_ID0
 *     1            1       16G LPDDR5
 *     1            0       8G LPDDR5
 *     0            1       4G LPDDR5 
************************************************/
enum {
	LPDDR5_16GB = 0x3,
	LPDDR5_8GB = 0x2,
	LPDDR5_4GB = 0x1,
	LPDDR5_UNKNOWN = 0xf,
};

#define OCRAM_NON_SECURE_BASE_ADDR 0x0204C0000

#endif /* __EDGE_AI_IMX95_DDR_H */