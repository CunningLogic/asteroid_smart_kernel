/**
********************************************************************************
* @file dummy-smsc-usb43340.c
* @brief smsc USB43340 transceiver
*
* Copyright (C) 2010 Parrot S.A.
*
* @author     Christian ROSALIE <christian.rosalie@parrot.com>
* @date       2010-09-03
********************************************************************************
*/

/**
 * \mainpage
 *
 * This module initializes the SMSC 43340 USB transceiver to manage the OMAP 3630 USB OTGUSB
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/usb/otg.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/workqueue.h>
#include "dummy-smsc-usb43340.h"

struct smsc43340_usb {
	struct otg_transceiver	otg;
	struct device		*dev;

        /* pins that must be configured before using it */
	int			gpio_reset;
	int			gpio_overcurrent;

        struct work_struct      irq_work;

	int			irq;
	u8			asleep;
};

/* internal define on top of container_of */
#define xceiv_to_smsc(x)		container_of((x), struct smsc43340_usb, otg);
#define smsc43340_phy_power_on(smsc)	gpio_set_value(smsc->gpio_reset, 1)
#define smsc43340_phy_power_off(smsc)	gpio_set_value(smsc->gpio_reset, 0)

static void smsc43340_phy_suspend(struct smsc43340_usb *smsc)
{
	if (smsc->asleep)
		return;

	smsc43340_phy_power_off(smsc);
	smsc->asleep = 1;
}

static void smsc43340_phy_resume(struct smsc43340_usb *smsc)
{
	if (!smsc->asleep)
		return;

        smsc43340_phy_power_on(smsc);

	smsc->asleep = 0;
}

static void smsc43340_irq_work_handler(struct work_struct *work)
{
	struct smsc43340_usb *smsc =
		container_of(work, struct smsc43340_usb, irq_work);

	smsc43340_phy_resume(smsc);
	printk(KERN_ERR "smsc43340_usb: USB on\n");
}


static irqreturn_t smsc43340_usb_irq(int irq, void *_smsc)
{
	struct smsc43340_usb *smsc = _smsc;

	if (gpio_is_valid(smsc->gpio_overcurrent)) {
		if (!gpio_get_value(smsc->gpio_overcurrent)) {
			/* Delay the activation of the usb */
			schedule_work(&smsc->irq_work);
		} else {
			smsc43340_phy_suspend(smsc);
			printk(KERN_ERR "smsc43340_usb: USB off\n");
		}
	} else {
		schedule_work(&smsc->irq_work);
	}

	return IRQ_HANDLED;
}

static int smsc43340_set_suspend(struct otg_transceiver *x, int suspend)
{
	struct smsc43340_usb *smsc = xceiv_to_smsc(x);

	if (suspend)
		smsc43340_phy_suspend(smsc);
	else
		smsc43340_phy_resume(smsc);

	return 0;
}

static int smsc43340_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct smsc43340_usb *smsc;

	if (!x)
		return -ENODEV;

	smsc = xceiv_to_smsc(x);
	smsc->otg.gadget = gadget;
	if (!gadget)
		smsc->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int smsc43340_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct smsc43340_usb *smsc;

	if (!x)
		return -ENODEV;

	smsc = xceiv_to_smsc(x);
	smsc->otg.host = host;
	if (!host)
		smsc->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int smsc43340_phy_init(struct otg_transceiver *x)
{
	struct smsc43340_usb *smsc = xceiv_to_smsc(x);
	/* Reset USB Phy */
	smsc43340_phy_power_off(smsc);
	mdelay(20);
	smsc43340_phy_power_on(smsc);

	smsc->asleep = 0;

	return 0;
}

static void smsc43340_phy_shutdown(struct otg_transceiver *x)
{
	struct smsc43340_usb *smsc = xceiv_to_smsc(x);

	/* Reset USB Phy */
	smsc43340_phy_power_off(smsc);
}

static int smsc43340_set_phy_clk(struct otg_transceiver *x, int on)
{
	return 0;
}

static int smsc43340_set_vbus(struct otg_transceiver *x, bool enabled)
{
	return 0;
}

static int __init smsc43340_usb_probe(struct platform_device *pdev)
{
	struct smsc43340_usb_data *pdata = pdev->dev.platform_data;
	struct smsc43340_usb	  *smsc;
	int			status;

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	smsc = kzalloc(sizeof *smsc, GFP_KERNEL);
	if (!smsc)
		return -ENOMEM;

	smsc->dev                 = &pdev->dev;
	smsc->otg.dev             = smsc->dev;
	smsc->otg.label           = "smsc43340";
	smsc->otg.set_host        = smsc43340_set_host;
	smsc->otg.set_peripheral  = smsc43340_set_peripheral;
	smsc->otg.set_suspend     = smsc43340_set_suspend;
	smsc->otg.init		  = smsc43340_phy_init;
	smsc->otg.shutdown	  = smsc43340_phy_shutdown;
	smsc->otg.set_clk	  = smsc43340_set_phy_clk;
	smsc->otg.set_vbus	  = smsc43340_set_vbus;
	smsc->asleep              = 1;
	smsc->gpio_reset          = pdata->gpio_reset;
	smsc->gpio_overcurrent    = pdata->gpio_overcurrent;

	if (!gpio_is_valid(smsc->gpio_reset)) {
		kfree(smsc);
		return -EINVAL;
	}

	// Init IRQ
	if (gpio_is_valid(smsc->gpio_overcurrent)) {
		// Get IRQ
		smsc->irq = OMAP_GPIO_IRQ(smsc->gpio_overcurrent);

		if (gpio_request(smsc->gpio_overcurrent, "smsc43340_usb_irq") < 0) {
			printk(KERN_ERR "Failed to request GPIO%d for smsc43340 IRQ\n",
				smsc->gpio_overcurrent);
			return -EIO;
		}
		gpio_direction_input(smsc->gpio_overcurrent);
	}

	smsc43340_phy_init(&smsc->otg);

	otg_set_transceiver(&smsc->otg);

	platform_set_drvdata(pdev, smsc);

        status = request_irq(smsc->irq, smsc43340_usb_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"smsc43340_usb", smsc);
	if (status < 0) {
		dev_dbg(&pdev->dev, "can't get IRQ %d, err %d\n",
			smsc->irq, status);
		kfree(smsc);
		return status;
	}

        INIT_WORK(&smsc->irq_work, smsc43340_irq_work_handler);

        /* Generate first IRQ */
        smsc43340_usb_irq(smsc->irq, (void *) smsc );

	dev_info(&pdev->dev, "Initialized SMSC 43340 USB module for OMAP 3630 USB OTG\n");
	BLOCKING_INIT_NOTIFIER_HEAD(&smsc->otg.notifier);
	return 0;
}

static int __exit smsc43340_usb_remove(struct platform_device *pdev)
{
	struct smsc43340_usb *smsc = platform_get_drvdata(pdev);

	free_irq(smsc->irq, smsc);

	smsc43340_phy_power_off(smsc);

	kfree(smsc);

	return 0;
}

static struct platform_driver smsc43340_usb_driver = {
	.probe		= smsc43340_usb_probe,
	.remove		= __exit_p(smsc43340_usb_remove),
	.driver		= {
		.name	= "smsc43340_usbotg",
		.owner	= THIS_MODULE,
	},
};

static int __init smsc43340_usb_init(void)
{
	return platform_driver_register(&smsc43340_usb_driver);
}
subsys_initcall(smsc43340_usb_init);

static void __exit smsc43340_usb_exit(void)
{
	platform_driver_unregister(&smsc43340_usb_driver);
}
module_exit(smsc43340_usb_exit);

MODULE_AUTHOR("PARROT SA by Christian ROSALIE <christian.rosalie@parrot.com>");
MODULE_DESCRIPTION("SMSC43340 USB Transceiver");
MODULE_LICENSE("GPL");
