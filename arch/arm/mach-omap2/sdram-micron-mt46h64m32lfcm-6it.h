/*
 * SDRC register values for the  Micron MT46H64M32LFCM-6 IT
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_SDRAM_MICRON_MT46H64M32LFCM_6IT
#define __ARCH_ARM_MACH_OMAP2_SDRAM_MICRON_MT46H64M32LFCM_6IT

#include <plat/sdrc.h>
#define TDAL_165  6
#define TDPL_165  3
#define TRRD_165  2
#define TRCD_165  3
#define TRP_165   3
#define TRAS_165  7
#define TRC_165   10
#define TRFC_165 12
#define V_ACTIMA_165 ((TRFC_165 << 27) | (TRC_165 << 22) | (TRAS_165 << 18) | \
                      (TRP_165 << 15) | (TRCD_165 << 12) | (TRRD_165 << 9) | \
                      (TDPL_165 << 6) | (TDAL_165))

#define TWTR_165  1
#define TCKE_165  1
//#define TXP_165   2
#define TXP_165   1
#define XSR_165 19
#define V_ACTIMB_165 ((TCKE_165 << 12) | (XSR_165 << 0) | \
                      (TXP_165 << 8) | (TWTR_165 << 16))

#define TDAL_200  6
#define TDPL_200  3
#define TRRD_200  2
#define TRCD_200  3
#define TRP_200   3
#define TRAS_200  8
#define TRC_200  11
#define TRFC_200 15
#define V_ACTIMA_200 ((TRFC_200 << 27) | (TRC_200 << 22) | (TRAS_200 << 18) | \
                      (TRP_200 << 15) | (TRCD_200 << 12) | (TRRD_200 << 9) | \
                      (TDPL_200 << 6) | (TDAL_200))

#define TWTR_200  2
#define TCKE_200  1
#define TXP_200   2
#define XSR_200 23
#define V_ACTIMB_200 ((TCKE_200 << 12) | (XSR_200 << 0) | \
                      (TXP_200 << 8) | (TWTR_200 << 16))

/* Micron MT29C8G48MAZAPBJA-5IT */
static struct omap_sdrc_params mt46h64m32lfcm6it_sdrc_params[] = {
	[0] = {
		.rate        = 166000000,
		.actim_ctrla = V_ACTIMA_165,
		.actim_ctrlb = V_ACTIMB_165,
		.rfr_ctrl    = 0x0005e801,
		.mr          = 0x00000032,
	},
};

#endif
