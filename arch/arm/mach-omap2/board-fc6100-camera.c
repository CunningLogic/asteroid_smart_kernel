/*
 * arch/arm/mach-omap2/board-fc6100-camera.c
 *
 * Copyright (C) 2011 MM Solutions
 * Author: Boris Todorov <btodorov@mm-sol.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>

#include <plat/gpio.h>
#include <plat/control.h>
#include <plat/omap-pm.h>
#include <media/tvp5150.h>
#include <mach/board-fc6100.h>

#include "devices.h"

#include "../../../drivers/media/video/isp/isp.h"
#include "../../../drivers/media/video/isp/ispreg.h"

#include "parrot-common.h"

#include "board-fc6100-camera.h"

#define TVP515x_I2C_BUSNUM		3
#define TVP515x_I2C_ADDR_LO		0x5C // (0xB8)
#define TVP515x_I2C_ADDR_HI		0x5D // (0xBA)



#if defined(CONFIG_VIDEO_TVP5150) || defined(CONFIG_VIDEO_TVP5150_MODULE)


static int fc6100_tvp5150_s_power(struct v4l2_subdev *subdev, u32 on)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);

	if (on) {
		/* Enable EXTCLK */
		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 27000000, 1);
		mdelay(50);
#ifdef CONFIG_PM
		/*
		 * Through-put requirement:
		 * Set max OCP freq for 3630 is 200 MHz through-put
		 * is in KByte/s so 200000 KHz * 4 = 800000 KByte/s
		 */
		omap_pm_set_min_bus_tput(isp->dev, OCP_INITIATOR_AGENT,
					 800000);
#endif

	} else {
		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 0, 1);
#ifdef CONFIG_PM
		/* Remove pm constraints */
		omap_pm_set_min_bus_tput(isp->dev, OCP_INITIATOR_AGENT, 0);
#endif
	}

	return 0;
}

/*
 * Note: Mezz HW04  boards must be patched to get the TVP powered up properly,
 * so the resetb gpio has no effect on tvp5151 on these boards.
 * */
static struct tvp5150_platform_data fc6100_tvp5150_platform_data = {
	.s_power	= fc6100_tvp5150_s_power,
};

static struct i2c_board_info fc6100_camera_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tvp5150", TVP515x_I2C_ADDR_LO),
		.platform_data = &fc6100_tvp5150_platform_data,
	},
};

static struct isp_subdev_i2c_board_info fc6100_camera_primary_subdevs[] = {
	{
		.board_info = &fc6100_camera_i2c_devices[0],
		.i2c_adapter_id = TVP515x_I2C_BUSNUM,
	},
	{ NULL, 0, },
};

static struct isp_v4l2_subdevs_group fc6100_camera_subdevs[] = {
	{
		.subdevs = fc6100_camera_primary_subdevs,
		.interface = ISP_INTERFACE_PARALLEL,
		.bus = { .parallel = {
				.data_lane_shift        = 0,
				.clk_pol                = 0,
				.hs_pol                 = 0,
				.vs_pol                 = 0,
				.data_pol		= 0,
				.fldmode                = 1,
				.bt656                  = 1,
		} },
	},
	{ NULL, 0, },
};

static struct isp_platform_data isp_pdata = {
	.subdevs = fc6100_camera_subdevs,
};

void __init fc6100_camera_init(void)
{

	pr_info("FC6100: Camera init\n");

	/* Initial state: */
	gpio_set_value(FC_CAM_EN_N, 1); /* CAM_EN is inverted on Mezz HW04 */
	mdelay (200);

	/* Power On Sequence */
	gpio_set_value(FC_CAM_EN_N, 0);
	mdelay (20); /* t1 (cf. TVP5151 spec. p24) */


if (omap3_init_camera(&isp_pdata) < 0)
		pr_warning("FC6100: Unable to register camera platform \n");

	pr_info("FC6100: Camera init done successfully \n");
}

#else
void __init fc6100_camera_init(void) {}
#endif
