/*
 * linux/arch/arm/mach-omap2/usb-musb.c
 *
 * This file will contain the board specific details for the
 * MENTOR USB OTG controller on OMAP3430
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Vikram Pandita
 *
 * Generalization by:
 * Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/usb/android_composite.h>

#include <linux/usb/musb.h>
#include <linux/pm_runtime.h>
#include <asm/sizes.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <plat/omap-pm.h>
#ifdef CONFIG_ARCH_OMAP3
#include "../../../drivers/usb/musb/musb_io.h"
#include "../../../drivers/usb/musb/musb_regs.h"
#include "../../../drivers/usb/musb/omap2430.h"
int musb_module_reset(void);
#endif /* CONFIG_ARCH_OMAP3 */

#define CONTROL_DEV_CONF                0x300
#define PHY_PD				(1 << 0)

#define OTG_SYSCONFIG			0x404
#define OTG_SYSC_SOFTRESET		BIT(1)

#define OTG_FORCESTDBY			0x414

#ifdef CONFIG_ARCH_OMAP4
#define DIE_ID_REG_BASE         (L4_44XX_PHYS + 0x2000)
#define DIE_ID_REG_OFFSET               0x200
#else
#define DIE_ID_REG_BASE         (L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET               0x218
#endif /* CONFIG_ARCH_OMAP4 */

#ifdef CONFIG_USB_MUSB_SOC

static const char name[] = "musb_hdrc";
#define MAX_OMAP_MUSB_HWMOD_NAME_LEN	16

static struct omap_hwmod *oh_p;

static struct musb_hdrc_config musb_config = {
	.multipoint	= 1,
	.dyn_fifo	= 1,
	.num_eps	= 16,
	.ram_bits	= 12,
};

#ifdef CONFIG_ANDROID
#define MAX_USB_SERIAL_NUM		17
#define OMAP_VENDOR_ID			0x0451
#define OMAP_UMS_PRODUCT_ID		0xD100
#define OMAP_ADB_PRODUCT_ID		0xD101
#define OMAP_UMS_ADB_PRODUCT_ID		0xD102
#define OMAP_RNDIS_PRODUCT_ID		0xD103
#define OMAP_RNDIS_ADB_PRODUCT_ID	0xD104
#define OMAP_ACM_PRODUCT_ID		0xD105
#define OMAP_ACM_ADB_PRODUCT_ID		0xD106
#define OMAP_ACM_UMS_ADB_PRODUCT_ID	0xD107
#define OMAP_MTP_PRODUCT_ID		0xD108
#define OMAP_MTP_ADB_PRODUCT_ID 	0xD109
#define OMAP_MTP_UMS_ADB_PRODUCT_ID	0xD10A

static char device_serial[MAX_USB_SERIAL_NUM];

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_acm[] = {
	"acm",
};

static char *usb_functions_acm_adb[] = {
	"acm",
	"adb",
};

static char *usb_functions_acm_ums_adb[] = {
	"acm",
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};
static char *usb_functions_mtp_ums_adb[] = {
	"mtp",
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_MTP
	"mtp",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id     = OMAP_UMS_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_ums),
		.functions      = usb_functions_ums,
	},
	{
		.product_id     = OMAP_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_adb),
		.functions      = usb_functions_adb,
	},
	{
		.product_id     = OMAP_UMS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_ums_adb),
		.functions      = usb_functions_ums_adb,
	},
	{
		.product_id     = OMAP_RNDIS_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis),
		.functions      = usb_functions_rndis,
	},
	{
		.product_id     = OMAP_RNDIS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions      = usb_functions_rndis_adb,
	},
	{
		.product_id     = OMAP_ACM_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_acm),
		.functions      = usb_functions_acm,
	},
	{
		.product_id     = OMAP_ACM_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_acm_adb),
		.functions      = usb_functions_acm_adb,
	},
	{
		.product_id     = OMAP_ACM_UMS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_acm_ums_adb),
		.functions      = usb_functions_acm_ums_adb,
	},
	{
		.product_id     = OMAP_MTP_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp),
		.functions      = usb_functions_mtp,
	},
	{
		.product_id     = OMAP_MTP_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_adb),
		.functions      = usb_functions_mtp_adb,
	},
	{
		.product_id     = OMAP_MTP_UMS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_ums_adb),
		.functions      = usb_functions_mtp_ums_adb,
        },
};

/* standard android USB platform data */
static struct android_usb_platform_data andusb_plat = {
	.vendor_id		= OMAP_VENDOR_ID,
	.product_id		= OMAP_UMS_PRODUCT_ID,
	.manufacturer_name	= "Texas Instruments Inc.",
	.product_name		= "OMAP-3/4",
	.serial_number		= device_serial,
	.num_products		= ARRAY_SIZE(usb_products),
	.products		= usb_products,
	.num_functions		= ARRAY_SIZE(usb_functions_all),
	.functions		= usb_functions_all,
};

static struct platform_device androidusb_device = {
	.name		= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data  = &andusb_plat,
	},
};

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data usbms_plat = {
	.vendor		= "Texas Instruments Inc.",
	.product	= "OMAP4",
	.release	= 1,
	.nluns		= 1,
};

static struct platform_device usb_mass_storage_device = {
	.name		= "usb_mass_storage",
	.id		= -1,
	.dev		= {
		.platform_data = &usbms_plat,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= OMAP_VENDOR_ID,
	.vendorDescr	= "Texas Instruments Inc.",
	};

static struct platform_device rndis_device = {
	.name		= "rndis",
	.id		= -1,
	.dev		= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static void usb_gadget_init(void)
{
	unsigned int val[4] = { 0 };
	unsigned int reg;
#ifdef CONFIG_USB_ANDROID_RNDIS
	int i;
	char *src;
#endif
	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;

	if (cpu_is_omap44xx()) {
		val[0] = omap_readl(reg);
		val[1] = omap_readl(reg + 0x8);
		val[2] = omap_readl(reg + 0xC);
		val[3] = omap_readl(reg + 0x10);
	} else if (cpu_is_omap34xx()) {
		val[0] = omap_readl(reg);
		val[1] = omap_readl(reg + 0x4);
		val[2] = omap_readl(reg + 0x8);
		val[3] = omap_readl(reg + 0xC);
	}

	snprintf(device_serial, MAX_USB_SERIAL_NUM, "%08X%08X%08X%08X",
					val[3], val[2], val[1], val[0]);

#ifdef CONFIG_USB_ANDROID_RNDIS
	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	src = device_serial;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	platform_device_register(&rndis_device);
#endif

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	platform_device_register(&usb_mass_storage_device);
#endif
	platform_device_register(&androidusb_device);
}

#else

static void usb_gadget_init(void)
{
}

#endif /* CONFIG_ANDROID */

static struct musb_hdrc_platform_data musb_plat = {
#ifdef CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode		= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode		= MUSB_PERIPHERAL,
#endif
	/* .clock is set dynamically */
	.config		= &musb_config,

	/* REVISIT charge pump on TWL4030 can supply up to
	 * 100 mA ... but this value is board-specific, like
	 * "mode", and should be passed to usb_musb_init().
	 */
	.power		= 50,			/* up to 100 mA */
};

static u64 musb_dmamask = DMA_BIT_MASK(32);

static int usb_idle_hwmod(struct omap_device *od)
{
	struct omap_hwmod *oh = *od->hwmods;

	if (irqs_disabled())
		_omap_hwmod_idle(oh);
	else
		omap_device_idle_hwmods(od);
	return 0;
}

static int usb_enable_hwmod(struct omap_device *od)
{
	struct omap_hwmod *oh = *od->hwmods;

	if (irqs_disabled())
		_omap_hwmod_enable(oh);
	else
		omap_device_enable_hwmods(od);
	return 0;
}

static struct omap_device_pm_latency omap_musb_latency[] = {
	  {
		.deactivate_func = usb_idle_hwmod,
		.activate_func	 = usb_enable_hwmod,
		.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	  },
};

void __init usb_musb_init(struct omap_musb_board_data *board_data)
{
	char oh_name[MAX_OMAP_MUSB_HWMOD_NAME_LEN];
	struct omap_hwmod *oh;
	struct omap_device *od;
	struct platform_device *pdev = NULL;
	struct device	*dev;
	int l, bus_id = -1;
	struct musb_hdrc_platform_data *pdata;
	void __iomem *otg_base;

	if (!board_data) {
		pr_err("Board data is required for hdrc device register\n");
		return;
	}
	l = snprintf(oh_name, MAX_OMAP_MUSB_HWMOD_NAME_LEN,
						"usb_otg_hs");
	WARN(l >= MAX_OMAP_MUSB_HWMOD_NAME_LEN,
			"String buffer overflow in MUSB device setup\n");

	oh = omap_hwmod_lookup(oh_name);

	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
	} else {
		/*
		 * REVISIT: This line can be removed once all the platforms
		 * using musb_core.c have been converted to use use clkdev.
		 */
		musb_plat.clock = "ick";
		musb_plat.board_data = board_data;
		musb_plat.power = board_data->power >> 1;
		musb_plat.mode = board_data->mode;
		musb_plat.device_enable = omap_device_enable;
		musb_plat.device_idle = omap_device_idle;
		musb_plat.enable_wakeup = omap_device_enable_wakeup;
		musb_plat.disable_wakeup = omap_device_disable_wakeup;
#ifdef CONFIG_PM
		musb_plat.set_min_bus_tput = omap_pm_set_min_bus_tput;
#endif
		/*
		 * Errata 1.166 idle_req/ack is broken in omap3430
		 * workaround is to disable the autodile bit for omap3430.
		 */
		if (cpu_is_omap3430() || cpu_is_omap3630())
			oh->flags |= HWMOD_NO_OCP_AUTOIDLE;

		musb_plat.oh = oh;
		oh_p = oh;
		pdata = &musb_plat;

		od = omap_device_build(name, bus_id, oh, pdata,
				       sizeof(struct musb_hdrc_platform_data),
							omap_musb_latency,
				       ARRAY_SIZE(omap_musb_latency), false);
		if (IS_ERR(od)) {
			pr_err("Could not build omap_device for %s %s\n",
						name, oh_name);
		} else {

			pdev = &od->pdev;
			dev = &pdev->dev;
			get_device(dev);
			dev->dma_mask = &musb_dmamask;
			dev->coherent_dma_mask = musb_dmamask;
			put_device(dev);
		}

		otg_base = ioremap(OMAP34XX_HSUSB_OTG_BASE, SZ_4K);

		if (WARN_ON(!otg_base))
			return;

		/* Reset OTG controller.  After reset, it will be in
		 * force-idle, force-standby mode. */
		__raw_writel(OTG_SYSC_SOFTRESET, otg_base + OTG_SYSCONFIG);

		iounmap(otg_base);

		/* powerdown the phy */
		if (board_data->interface_type == MUSB_INTERFACE_UTMI)
			omap_writel(PHY_PD, DIE_ID_REG_BASE + CONTROL_DEV_CONF);

		usb_gadget_init();
#ifdef CONFIG_ARCH_OMAP3
		if(pdev){
			musb_plat.device_enable(pdev);
			musb_module_reset();
			musb_plat.device_idle(pdev);
		}else{
			pr_err("Could not reset musb module.\n");
		}
#endif
	}
}

#ifdef CONFIG_ARCH_OMAP3
/*
Errata ID: i445

Instead of using the OTG_SYSCONFIG:SOFTRESET bit filed, the user can use the
SOFTRESET register of the USB module at offset 0x7F, which will have the same effect
and will allow the hsusb_stp	pin to go high and the reset command to be sent.
As soon as this is done, LINK and PHY can start negotiating and function normally.
 */
int musb_module_reset(void)
{
	struct omap_hwmod *oh = oh_p;
	struct platform_device *pdev;
	struct resource	*iomem;
	void __iomem	*base;
	int ret = 0;
	u32 val;

	if(oh){
		pdev = &oh->od->pdev;
		iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!iomem){
			pr_err("musb_module_reset: no mem resource\n");
			return -EINVAL;
		}
	}else{
		/* reset called at boot */
		iomem = kmalloc(sizeof(struct resource), GFP_KERNEL);
		if(!iomem){
			pr_err("musb_module_reset: memory alloc failed\n");
			return -ENOMEM;
		}
		iomem->start = OMAP34XX_HSUSB_OTG_BASE;
		iomem->end = OMAP34XX_HSUSB_OTG_BASE + SZ_4K - 1;
	}

	base = ioremap(iomem->start, resource_size(iomem));
	if (!base) {
		pr_err("musb_module_reset: usb otg ioremap failed\n");
		ret = -ENOMEM;
		goto out;
	}

	/* recommended by TI HW team.
	   step1. set USB OTG_SYSCONFIG:SOFTRESET
	   step2. set SOFTRESET of the USB module(0x7F).
	   step3. wait 3ms.
	 */
#if 0
	/* If set the otg sysconfig.softreset here,
	   usb adb does not connected, when the boot without usb cable.
	   So, softreset do not use.
	 */
	/* softreset */
	val = musb_readl(base, OTG_SYSCONFIG);
	val |= SOFTRST;
	musb_writel(base, OTG_SYSCONFIG, val);
#endif
	/* module reset */
	musb_writeb(base, MUSB_SOFT_RST, 0x3);
	mdelay(3); /* recommended by TI HW team */

	iounmap(base);
out:
	if(!oh){
		kfree(iomem);
	}

	return ret;
}
#endif /* CONFIG_ARCH_OMAP3 */

void musb_context_save_restore(enum musb_state state)
{
	struct omap_hwmod *oh = oh_p;
	struct omap_device *od = oh->od;
	struct platform_device *pdev = &od->pdev;
	struct device *dev = &pdev->dev;
	struct device_driver *drv = dev->driver;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	struct clk *phyclk;
	struct clk *clk48m;
	struct resource	*iomem;
	void __iomem	*base;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem)
		return;

	phyclk = clk_get(NULL, "ocp2scp_usb_phy_ick");
	if (!phyclk) {
		pr_warning("cannot clk_get ocp2scp_usb_phy_ick\n");
		return;
	}
	clk48m = clk_get(NULL, "ocp2scp_usb_phy_phy_48m");
	if (!clk48m) {
		pr_warning("cannot clk_get ocp2scp_usb_phy_phy_48m\n");
		return;
	}

	base = ioremap(iomem->start, resource_size(iomem));
	if (!base) {
		dev_err(dev, "ioremap failed\n");
		return;
	}

	if (drv) {
#ifdef CONFIG_PM_RUNTIME
		struct musb_hdrc_platform_data *pdata = dev->platform_data;
		const struct dev_pm_ops *pm = drv->pm;

		if (plat->is_usb_active)
			if (!pdata->is_usb_active(dev)) {

				switch (state) {
				case save_context:
				/*Save the context, set the sysconfig setting
				 *  to force standby force idle during idle and
				 *  disable the clock.
				 */
				pm->suspend(dev);
				pdata->device_idle(pdev);
				break;

				case disable_clk:

				/* set the sysconfig setting to force standby,
				 * force idle during idle and disable
				 * the clock.
				 */
				if (data->interface_type ==
						MUSB_INTERFACE_UTMI) {
					/* Disable the 48Mhz phy clock and
					 * module mode
					 */
					clk_disable(phyclk);
					clk_disable(clk48m);
				}
				/* Enable ENABLEFORCE bit*/
				__raw_writel(0x1, base + OTG_FORCESTDBY);
				pdata->device_idle(pdev);
				break;

				case restore_context:
				/* Enable the clock, set the sysconfig setting
				 * back to no idle and no stndby
				 * after wakeup, restore the context.
				 */
				pdata->device_enable(pdev);
				pm->resume_noirq(dev);
				break;

				case enable_clk:
				/* set the sysconfig setting back to no idle
				 * and no stndby after wakeup and enable
				 * the clock.
				 */
				pdata->device_enable(pdev);

				if (data->interface_type ==
						MUSB_INTERFACE_UTMI) {
					/* Enable phy 48Mhz clock and module
					 * mode bit
					 */
					clk_enable(phyclk);
					clk_enable(clk48m);
				}
				/* Disable ENABLEFORCE bit*/
				__raw_writel(0x0, base + OTG_FORCESTDBY);

				break;

				default:
					break;
				}
			}
#endif

	}
	iounmap(base);
}

#else
void __init usb_musb_init(struct omap_musb_board_data *board_data)
{
	int l;

	if (board_data && board_data->interface_type == MUSB_INTERFACE_UTMI)
		/*powerdown the phy*/
		omap_writel(PHY_PD, DIE_ID_REG_BASE + CONTROL_DEV_CONF);
}
void musb_context_save_restore(enum musb_state state)
{
}
#endif /* CONFIG_USB_MUSB_SOC */
