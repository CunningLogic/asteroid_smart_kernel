/*
 * OMAP display LED Driver
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: Dan Murphy <DMurphy@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __OMAP_DISPLAY_LED__
#define __OMAP_DISPLAY_LED__

#define LEDS_CTRL_AS_ONE_DISPLAY 	(1 << 0)
#define LEDS_CTRL_AS_TWO_DISPLAYS 	(1 << 1)

struct omap_disp_led_platform_data {
	int flags;
	void (*primary_display_set)(u8 value);
	void (*secondary_display_set)(u8 value);
};

struct display_led_data {
	struct led_classdev pri_display_class_dev;
	struct led_classdev sec_display_class_dev;
	struct omap_disp_led_platform_data *led_pdata;
	struct mutex pri_disp_lock;
	struct mutex sec_disp_lock;
};

struct display_led_data *get_omap_led_info(void);

#endif  /*__OMAP_DISPLAY_LED__*/
