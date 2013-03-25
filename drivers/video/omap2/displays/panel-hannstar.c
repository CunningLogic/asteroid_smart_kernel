/*
 * LCD panel driver for Samsung LTE430WQ-F0C
 *
 * Author: Steve Sakoman <steve@sakoman.com>
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

#include <linux/module.h>
#include <linux/delay.h>

#include <plat/display.h>

static struct omap_video_timings hannstar_lte_timings = {
	.x_res = 800,
	.y_res = 480,

	.pixel_clock	= 33260,

    .hsw        = 48 /* pulse width */,
    .hfp        = 40 /* front */,
    .hbp        = 928-800-48-40 /* back */,

    .vsw        = 3 /* pulse width */,
    .vfp        = 13 /* front */,
    .vbp        = 525-480-3-13 /* back */,
};

static int hannstar_lte_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT /*| OMAP_DSS_LCD_IPC*/
		/*| OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS*/;
	dssdev->panel.timings = hannstar_lte_timings;

	dssdev->s_mode = OMAP_DSS_DISPLAY_DEEP;

	return 0;
}

static void hannstar_lte_panel_remove(struct omap_dss_device *dssdev)
{
}

static int hannstar_lte_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(200);

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

err0:
	if (r)
		omapdss_dpi_display_disable(dssdev);
	return r;
}

static void hannstar_lte_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(200);
	omapdss_dpi_display_disable(dssdev);
}

static int hannstar_lte_panel_suspend(struct omap_dss_device *dssdev)
{
	hannstar_lte_panel_disable(dssdev);
	return 0;
}

static int hannstar_lte_panel_resume(struct omap_dss_device *dssdev)
{
	return hannstar_lte_panel_enable(dssdev);
}

static void hannstar_lte_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void hannstar_get_dimension(struct omap_dss_device *dssdev,
		u32 *width, u32 *height)
{
	*width = dssdev->panel.width_in_mm;
	*height = dssdev->panel.height_in_mm;
}

static struct omap_dss_driver hannstar_lte_driver = {
	.probe		= hannstar_lte_panel_probe,
	.remove		= hannstar_lte_panel_remove,

	.enable		= hannstar_lte_panel_enable,
	.disable	= hannstar_lte_panel_disable,
	.suspend	= hannstar_lte_panel_suspend,
	.resume		= hannstar_lte_panel_resume,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,
	.get_timings    = hannstar_lte_panel_get_timings,
	.get_dimension = hannstar_get_dimension,

	.driver         = {
		.name   = "hannstar_lte_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init hannstar_lte_panel_drv_init(void)
{
	return omap_dss_register_driver(&hannstar_lte_driver);
}

static void __exit hannstar_lte_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&hannstar_lte_driver);
}

module_init(hannstar_lte_panel_drv_init);
module_exit(hannstar_lte_panel_drv_exit);
MODULE_LICENSE("GPL");
