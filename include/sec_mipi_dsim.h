/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __SEC_MIPI_DSIM_H__
#define __SEC_MIPI_DSIM_H__

/* DPHY PLL structure */
struct sec_mipi_dsim_range {
	uint32_t min;
	uint32_t max;
};

struct sec_mipi_dsim_pll {
	struct sec_mipi_dsim_range p;
	struct sec_mipi_dsim_range m;
	struct sec_mipi_dsim_range s;
	struct sec_mipi_dsim_range k;
	struct sec_mipi_dsim_range fin;
	struct sec_mipi_dsim_range fpref;
	struct sec_mipi_dsim_range fvco;
};

/* DPHY timings structure */
struct sec_mipi_dsim_dphy_timing {
	uint32_t bit_clk;	/* MHz */

	uint32_t clk_prepare;
	uint32_t clk_zero;
	uint32_t clk_post;
	uint32_t clk_trail;

	uint32_t hs_prepare;
	uint32_t hs_zero;
	uint32_t hs_trail;

	uint32_t lpx;
	uint32_t hs_exit;
};

#define DSIM_DPHY_TIMING(bclk, cpre, czero, cpost, ctrail,	\
			 hpre, hzero, htrail, lp, hexit)	\
	.bit_clk	= bclk,					\
	.clk_prepare	= cpre,					\
	.clk_zero	= czero,				\
	.clk_post	= cpost,				\
	.clk_trail	= ctrail,				\
	.hs_prepare	= hpre,					\
	.hs_zero	= hzero,				\
	.hs_trail	= htrail,				\
	.lpx		= lp,					\
	.hs_exit	= hexit

static inline int dphy_timing_default_cmp(const void *key, const void *elt)
{
	const struct sec_mipi_dsim_dphy_timing *_key = key;
	const struct sec_mipi_dsim_dphy_timing *_elt = elt;

	/* find an element whose 'bit_clk' is equal to the
	 * the key's 'bit_clk' value or, the difference
	 * between them is less than 5.
	 */
	if (abs((int)(_elt->bit_clk - _key->bit_clk)) <= 5)
		return 0;

	if (_key->bit_clk < _elt->bit_clk)
		/* search bottom half */
		return 1;
	else
		/* search top half */
		return -1;
}

struct sec_mipi_dsim_plat_data {
	uint32_t version;
	uint32_t max_data_lanes;
	uint64_t max_data_rate;
	const struct sec_mipi_dsim_dphy_timing *dphy_timing;
	uint32_t num_dphy_timing;
	ulong reg_base;
	ulong gpr_base;
	const struct sec_mipi_dsim_pll *dphy_pll;
	int (*dphy_timing_cmp)(const void *key, const void *elt);
};

int sec_mipi_dsim_setup(const struct sec_mipi_dsim_plat_data *plat_data);

#endif
