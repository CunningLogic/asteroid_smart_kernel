/*
 * LCD panel driver for Kyocera TCG101
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

static struct omap_video_timings tcg101wxlp_timings = {
	.x_res = 1280,
	.y_res = 800,

	.pixel_clock	= 66000,

	.hsw		= 32,				// Horizontal synchronization pulse width
	.hbp		= 48,				// Horizontal back porch
	.hfp		= 80,				// Horizontal front porch

	.vsw		= 6,				// Vertical synchronization pulse width
	.vbp		= 3, 				// Vertical back porch
	.vfp		= 14				// Vertical front porch
};

static int tcg101wxlp_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT /*| OMAP_DSS_LCD_IPC */
		| OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;
	dssdev->panel.timings = tcg101wxlp_timings;

	return 0;
}

static void tcg101wxlp_panel_remove(struct omap_dss_device *dssdev)
{
}

static int tcg101wxlp_panel_enable(struct omap_dss_device *dssdev)
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

static void tcg101wxlp_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(200);
	omapdss_dpi_display_disable(dssdev);
}

static int tcg101wxlp_panel_suspend(struct omap_dss_device *dssdev)
{
	tcg101wxlp_panel_disable(dssdev);
	return 0;
}

static int tcg101wxlp_panel_resume(struct omap_dss_device *dssdev)
{
	return tcg101wxlp_panel_enable(dssdev);
}
static void tcg101wxlp_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void tcg101wxlp_get_dimension(struct omap_dss_device *dssdev,
		u32 *width, u32 *height)
{
	*width = dssdev->panel.width_in_mm;
	*height = dssdev->panel.height_in_mm;
}

static struct omap_dss_driver tcg101wxlp_driver = {
	.probe		= tcg101wxlp_panel_probe,
	.remove		= tcg101wxlp_panel_remove,

	.enable		= tcg101wxlp_panel_enable,
	.disable	= tcg101wxlp_panel_disable,
	.suspend	= tcg101wxlp_panel_suspend,
	.resume		= tcg101wxlp_panel_resume,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,
	.get_timings    = tcg101wxlp_panel_get_timings,
	.get_dimension  = tcg101wxlp_get_dimension,

	.driver         = {
		.name   = "kyocera-TCG101WXVLPAANN",
		.owner  = THIS_MODULE,
	},
};

static int __init tcg101wxlp_panel_drv_init(void)
{
	return omap_dss_register_driver(&tcg101wxlp_driver);
}

static void __exit tcg101wxlp_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&tcg101wxlp_driver);
}

module_init(tcg101wxlp_panel_drv_init);
module_exit(tcg101wxlp_panel_drv_exit);
MODULE_LICENSE("GPL");
