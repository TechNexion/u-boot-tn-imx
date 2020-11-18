#ifndef __LPDDR4_TIMING_BASE_B1_H__
#define __LPDDR4_TIMING_BASE_B1_H__

#include <asm/arch/ddr.h>
#include "lpddr4_timing_base_b1.h"

#define  ELEMENTS_IN(array)            __elements_in_##array
#define  ELEMENTS_IN_DEF(array)        size_t __elements_in_##array = sizeof(array) / sizeof(array[0]);
#define  ELEMENTS_IN_DECLARE(array)    extern size_t __elements_in_##array;

extern struct dram_cfg_param ddr_ddrphy_cfg_b1[];
ELEMENTS_IN_DECLARE(ddr_ddrphy_cfg_b1)

extern struct dram_cfg_param ddr_ddrphy_trained_csr_b1[];
ELEMENTS_IN_DECLARE(ddr_ddrphy_trained_csr_b1)

extern struct dram_cfg_param ddr_phy_pie_b1[];
ELEMENTS_IN_DECLARE(ddr_phy_pie_b1)

#endif /* __LPDDR4_TIMING_BASE_B1_H__ */