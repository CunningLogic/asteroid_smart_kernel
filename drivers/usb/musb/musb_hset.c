/*
 * HS USB Host-mode HSET driver for Mentor USB Controller
 *
 * Copyright (C) 2005 Mentor Graphics Corporation
 * Copyright (C) 2007 Texas Instruments, Inc.
 *      Nishant Kamat <nskamat at ti.com>
 *      Vikram Pandita <vikram.pandita at ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * This driver is based on the USB Skeleton driver-2.0
 * (drivers/usb/usb-skeleton.c) written by Greg Kroah-Hartman (greg at kroah.com)
 *
 * Notes:
 * There are 2 ways this driver can be used:
 *	1 - By attaching a HS OPT (OTG Protocol Tester) card.
 * 	    The OPT test application contains scripts to test each mode.
 *	    Each script attaches the OPT with a distinct VID/PID. Depending on
 *	    the VID/PID this driver go into a particular test mode.
 *	2 - Through /proc/drivers/musb_HSET interface.
 *	    This is a forceful method, and rather more useful.
 *	    Any USB device can be attached to the Host.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/usb.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include "../core/usb.h"
#include "musbdefs.h"
#include "musb_io.h"
#include "musb_regs.h"
#include "musb_core.h"


/* ULPI pass-through */
#define MGC_O_HDRC_ULPI_VBUSCTL	0x70
#define MGC_O_HDRC_ULPI_REGDATA 0x74
#define MGC_O_HDRC_ULPI_REGADDR 0x75
#define MGC_O_HDRC_ULPI_REGCTL	0x76

/* extended config & PHY control */
#define MGC_O_HDRC_ENDCOUNT	0x78
#define MGC_O_HDRC_DMARAMCFG	0x79
#define MGC_O_HDRC_PHYWAIT	0x7A
#define MGC_O_HDRC_PHYVPLEN	0x7B	/* units of 546.1 us */
#define MGC_O_HDRC_HSEOF1	0x7C	/* units of 133.3 ns */
#define MGC_O_HDRC_FSEOF1	0x7D	/* units of 533.3 ns */
#define MGC_O_HDRC_LSEOF1	0x7E	/* units of 1.067 us */

/* Added in HDRC 1.9(?) & MHDRC 1.4 */
/* ULPI */
#define MGC_M_ULPI_VBUSCTL_USEEXTVBUSIND    0x02
#define MGC_M_ULPI_VBUSCTL_USEEXTVBUS	    0x01
#define MGC_M_ULPI_REGCTL_INT_ENABLE	    0x08
#define MGC_M_ULPI_REGCTL_READNOTWRITE	    0x04
#define MGC_M_ULPI_REGCTL_COMPLETE	    0x02
#define MGC_M_ULPI_REGCTL_REG		    0x01
/* extended config & PHY control */
#define MGC_M_ENDCOUNT_TXENDS	0x0f
#define MGC_S_ENDCOUNT_TXENDS	0
#define MGC_M_ENDCOUNT_RXENDS	0xf0
#define MGC_S_ENDCOUNT_RXENDS	4
#define MGC_M_DMARAMCFG_RAMBITS	0x0f	    /* RAMBITS-1 */
#define MGC_S_DMARAMCFG_RAMBITS	0
#define MGC_M_DMARAMCFG_DMACHS	0xf0
#define MGC_S_DMARAMCFG_DMACHS	4
#define MGC_M_PHYWAIT_WAITID	0x0f	    /* units of 4.369 ms */
#define MGC_S_PHYWAIT_WAITID	0
#define MGC_M_PHYWAIT_WAITCON	0xf0	    /* units of 533.3 ns */
#define MGC_S_PHYWAIT_WAITCON	4

/*---------------------------------------------------------------------------*/
/* This is the list of VID/PID that the HS OPT card will use. */
static struct usb_device_id hset_table [] = {
	{ USB_DEVICE(6666, 0x0101) },	/* TEST_SE0_NAK */
	{ USB_DEVICE(6666, 0x0102) },	/* TEST_J */
	{ USB_DEVICE(6666, 0x0103) },	/* TEST_K */
	{ USB_DEVICE(6666, 0x0104) },	/* TEST_PACKET */
	{ USB_DEVICE(6666, 0x0105) },	/* TEST_FORCE_ENABLE */
	{ USB_DEVICE(6666, 0x0106) },	/* HS_HOST_PORT_SUSPEND_RESUME */
	{ USB_DEVICE(6666, 0x0107) },	/* SINGLE_STEP_GET_DEV_DESC */
	{ USB_DEVICE(6666, 0x0108) },	/* SINGLE_STEP_SET_FEATURE */
	{ }				/* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, hset_table);

/* Structure to hold all of our device specific stuff */
struct usb_hset {
	struct usb_device *	udev;	/* the usb device for this device */
	struct usb_interface *	interface; /* the interface for this device */
	struct kref		kref;
	struct musb *		musb; /* the controller */
	struct proc_dir_entry* pde;
};
#define to_hset_dev(d) container_of(d, struct usb_hset, kref)

static struct usb_hset *the_hset;
static struct usb_driver hset_driver;
static struct proc_dir_entry *
	hset_proc_create(char *name, struct usb_hset *hset);
static void hset_proc_delete(char *name, struct usb_hset *hset);

/*---------------------------------------------------------------------------*/
/* Test routines */

static int musb_write_ulpi_reg(struct musb *musb, u8 address, u8 value);

static void test_musb(struct musb *musb, u8 mode)
{
	void __iomem *pBase = musb->mregs;

	if (mode == MUSB_TEST_J || mode == MUSB_TEST_K)
		/* write 0x50 to Function Control Register (@ 0x04) of the PHY
		 * 0x50 => OpMode = 10b => Disable bit-stuff and NRZI encoding
		 * 	   SuspendM = 1b => Powered
		 */
		musb_write_ulpi_reg(musb, 0x04, 0x50);

	if (mode == MUSB_TEST_PACKET) {
	        musb_ep_select(pBase, 0);
		musb_writew(musb->endpoints[0].regs, MUSB_CSR0, 0);
		musb_write_fifo(&musb->endpoints[0], sizeof(musb_test_packet),
				musb_test_packet);
		/* need to set test mode before enabling FIFO, as otherwise
		 * the packet goes out and the FIFO empties before the test
		 * mode is set
		 */
		musb_writeb(pBase, MUSB_TESTMODE, mode);
		musb_writew(musb->endpoints[0].regs, MUSB_CSR0,
						MUSB_CSR0_TXPKTRDY);
	} else
		musb_writeb(pBase, MUSB_TESTMODE, mode);
}

static inline void test_se0_nak(struct usb_hset *hset)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	test_musb(hset->musb, MUSB_TEST_SE0_NAK);
}

static inline void test_j(struct usb_hset *hset)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	test_musb(hset->musb, MUSB_TEST_J);
}

static inline void test_k(struct usb_hset *hset)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	test_musb(hset->musb, MUSB_TEST_K);
}

static inline void test_packet(struct usb_hset *hset)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	test_musb(hset->musb, MUSB_TEST_PACKET);
}

static inline void test_force_enable(struct usb_hset *hset)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	test_musb(hset->musb, MUSB_TEST_FORCE_HOST);
}

static void suspend(struct usb_hset *hset)
{
	struct musb *musb = hset->musb;
	void __iomem *pBase = musb->mregs;
	u8 power;

	printk(KERN_INFO "%s\n", __FUNCTION__);
	power = musb_readb(pBase, MUSB_POWER);
	musb_writeb(pBase, MUSB_POWER, power | MUSB_POWER_SUSPENDM);
}

static void resume(struct usb_hset *hset)
{
	struct musb *musb = hset->musb;
	void __iomem *pBase = musb->mregs;
	u8 power;

	printk(KERN_INFO "%s\n", __FUNCTION__);
	power = musb_readb(pBase, MUSB_POWER);
	if (power & MUSB_POWER_SUSPENDM) {
		power &= ~(MUSB_POWER_SUSPENDM | MUSB_POWER_RESUME);
		musb_writeb(pBase, MUSB_POWER, power | MUSB_POWER_RESUME);
		msleep(20);
		power = musb_readb(pBase, MUSB_POWER);
		musb_writeb(pBase, MUSB_POWER, power & ~MUSB_POWER_RESUME);
	} else
		printk(KERN_DEBUG "not suspended??\n");
}

static void test_suspend_resume(struct usb_hset *hset)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	suspend(hset);
	msleep(15000);	/* Wait for 15 sec */
	resume(hset);
}

int usb_get_device_descriptor1(struct usb_device *dev, unsigned int size)
{
	struct usb_device_descriptor *desc;
	int ret;

	if (size > sizeof(*desc))
		return -EINVAL;
	desc = kmalloc(sizeof(*desc), GFP_NOIO);
	if (!desc)
		return -ENOMEM;

	ret = usb_get_descriptor(dev, USB_DT_DEVICE, 0, desc, size);
	if (ret >= 0)
		memcpy(&dev->descriptor, desc, size);
	kfree(desc);
	return ret;
}

static void test_single_step_get_dev_desc(struct usb_hset *hset)
{
	struct musb *musb = hset->musb;
	struct usb_device *udev;
	struct usb_bus *bus = hcd_to_bus(musb_to_hcd(musb));

	printk(KERN_INFO "%s\n", __FUNCTION__);
	if (!musb || !bus || !bus->root_hub) {
		printk(KERN_ERR "Host controller not ready!\n");
		return;
	}
	udev = bus->root_hub->children[0];
	if (!udev) {
		printk(KERN_DEBUG "No device connected.\n");
		return;
	}
	usb_get_device_descriptor1(udev, sizeof(struct usb_device_descriptor));
}

static void test_single_step_set_feature(struct usb_hset *hset)
{
	struct musb *musb = hset->musb;
	struct usb_device *udev;
	struct usb_bus *bus = hcd_to_bus(musb_to_hcd(musb));

	printk(KERN_INFO "%s\n", __FUNCTION__);
	if (!musb || !bus || !bus->root_hub) {
		printk(KERN_ERR "Host controller not ready!\n");
		return;
	}

	udev = bus->root_hub->children[0];
	if (!udev) {
		printk(KERN_DEBUG "No device connected.\n");
		return;
	}
	usb_control_msg(udev,
			usb_sndctrlpipe(udev, 0),
			USB_REQ_SET_FEATURE, 0, 0,
			0, NULL, 0, HZ * USB_CTRL_SET_TIMEOUT);
}

static void enumerate_bus(struct work_struct *ignored)
{
	struct usb_hset *hset = the_hset;
	struct musb *musb = hset->musb;
	struct usb_device *udev;
	struct usb_bus *bus = hcd_to_bus(musb_to_hcd(musb));

	printk(KERN_INFO "%s\n", __FUNCTION__);
	if (!musb || !bus || !bus->root_hub) {
		printk(KERN_ERR "Host controller not ready!\n");
		return;
	}
	udev = bus->root_hub->children[0];
	if (udev)
		usb_reset_device(udev);
}

DECLARE_WORK(enumerate, enumerate_bus);

/*---------------------------------------------------------------------------*/

/* This function can be called either by musb_init_hset() or usb_hset_probe().
 * musb_init_hset() is called by the controller driver during its init(),
 * while usb_hset_probe() is called when an OPT is attached. We take care not
 * to allocate the usb_hset structure twice.
 */
static struct usb_hset* init_hset_dev(void *controller)
{
	struct usb_hset *hset = NULL;

	/* if already allocated, just increment use count and return */
	if (the_hset) {
		kref_get(&the_hset->kref);
		return the_hset;
	}

	hset = kmalloc(sizeof(*hset), GFP_KERNEL);
	if (hset == NULL) {
		err("Out of memory");
		return NULL;
	}
	memset(hset, 0x00, sizeof(*hset));
	hset->musb = (struct musb *)(controller);

	kref_init(&hset->kref);
	the_hset = hset;
	return hset;
}

static void hset_delete(struct kref *kref)
{
	struct usb_hset *dev = to_hset_dev(kref);

	kfree (dev);
}
/*---------------------------------------------------------------------------*/
/* Usage of HS OPT */

static inline struct musb *dev_to_musb(struct device *dev)
{
#ifdef CONFIG_USB_MUSB_HDRC_HCD
        /* usbcore insists dev->driver_data is a "struct hcd *" */
        return hcd_to_musb(dev_get_drvdata(dev));
#else
        return dev_get_drvdata(dev);
#endif
}

/* Called when the HS OPT is attached as a device */
static int hset_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	struct usb_hset *hset;
	struct usb_device *udev;
	struct usb_hcd *hcd;
	int retval = -ENOMEM;

	printk("BCERTIF:%s()\n", __FUNCTION__);

	udev = usb_get_dev(interface_to_usbdev(interface));
	// hcd = udev->bus->controller->driver_data;
	hcd = dev_get_drvdata(udev->bus->controller);





	hset = init_hset_dev(hcd->hcd_priv);
	if (!hset)
		return retval;

	hset->udev = udev;
	hset->interface = interface;
	usb_set_intfdata(interface, hset);

	switch(id->idProduct) {
	case 0x0101:
		test_se0_nak(hset);
		break;
	case 0x0102:
		test_j(hset);
		break;
	case 0x0103:
		test_k(hset);
		break;
	case 0x0104:
		test_packet(hset);
		break;
	case 0x0105:
		test_force_enable(hset);
		break;
	case 0x0106:
		test_suspend_resume(hset);
		break;
	case 0x0107:
		msleep(15000);	/* SOFs for 15 sec */
		test_single_step_get_dev_desc(hset);
		break;
	case 0x0108:
		test_single_step_get_dev_desc(hset);
		msleep(15000);	/* SOFs for 15 sec */
		test_single_step_set_feature(hset);
		break;
	};
	return 0;
}

static void hset_disconnect(struct usb_interface *interface)
{
	struct usb_hset *hset;

	/* prevent hset_open() from racing hset_disconnect() */
	lock_kernel();
	hset = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);
	unlock_kernel();

	usb_put_dev(hset->udev);
	kref_put(&hset->kref, hset_delete);
}

static struct usb_driver hset_driver = {
//	.owner =	THIS_MODULE,
	.name =		"hset",
	.probe =	hset_probe,
	.disconnect =	hset_disconnect,
	.id_table =	hset_table,
};

int usb_hset_init(void)
{
	int result;
	printk("BCERTIF:%s()\n", __FUNCTION__);
	/* register this driver with the USB subsystem */
	result = usb_register(&hset_driver);
	if (result)
		err("usb_register failed. Error number %d", result);

	return result;
}

void usb_hset_exit(void)
{
        printk("BCERTIF:%s()\n", __FUNCTION__);
	/* deregister this driver with the USB subsystem */
	usb_deregister(&hset_driver);
}

//module_init (usb_hset_init);
//module_exit (usb_hset_exit);

MODULE_LICENSE("GPL");

/*---------------------------------------------------------------------------*/
/* Usage of /proc/drivers/musb_hset interface */

void musb_init_hset(char *name, struct musb *musb)
{
	struct usb_hset *hset;
	printk("BCERTIF:%s()\n", __FUNCTION__);
	hset = init_hset_dev((void *)musb);
	hset_proc_create(name, hset);
}

void musb_exit_hset(char *name, struct musb *musb)
{
	struct usb_hset *hset = the_hset;
	
	printk("BCERTIF:%s()\n", __FUNCTION__);
	if (!hset) {
		printk(KERN_DEBUG "No hset device!\n");
		return;
	}
	hset_proc_delete(name, hset);
	kref_put(&hset->kref, hset_delete);
}

static int dump_menu(char *buffer)
{
	int count = 0;

	*buffer = 0;
	count = sprintf(buffer, "HOST-side high-speed electrical test modes:\n"
				"J: Test_J\n"
				"K: Test_K\n"
				"S: Test_SE0_NAK\n"
				"P: Test_PACKET\n"
				"H: Test_FORCE_ENABLE\n"
				"U: Test_SUSPEND_RESUME\n"
				"G: Test_SINGLE_STEP_GET_DESC\n"
				"F: Test_SINGLE_STEP_SET_FEATURE\n"
				"E: Enumerate bus\n"
				"D: Suspend bus\n"
				"R: Resume bus\n"
				"?: help menu\n");
	if (count < 0)
		return count;
	buffer += count;
	return count;
}

static int hset_proc_write(struct file *file, const char __user *buffer,
			 unsigned long count, void *data)
{
	char cmd;
	struct usb_hset *hset = (struct usb_hset *)data;
	char help[500];
	/* MOD_INC_USE_COUNT; */
	//if (hset->musb->asleep)
	//	return EBUSY;

	if(copy_from_user(&cmd, buffer, 1))
		return -EFAULT;
	switch (cmd) {
	case 'S':
		test_se0_nak(hset);
		break;
	case 'J':
		test_j(hset);
		break;
	case 'K':
		test_k(hset);
		break;
	case 'P':
		test_packet(hset);
		break;
	case 'H':
		test_force_enable(hset);
		break;
	case 'U':
		test_suspend_resume(hset);
		break;
	case 'G':
		test_single_step_get_dev_desc(hset);
		break;
	case 'F':
		test_single_step_set_feature(hset);
		break;
	case 'E':
		schedule_work(&enumerate);
		break;
	case 'D':
		suspend(hset);
		break;
	case 'R':
		resume(hset);
		break;
	case '?':
		dump_menu(help);
		printk(KERN_INFO "%s", help);
		break;
	default:
		printk(KERN_ERR "Command %c not implemented\n", cmd);
		break;
	}

	return count;
}

static int hset_proc_read(char *page, char **start,
			off_t off, int count, int *eof, void *data)
{
	char *buffer = page;
	int code = 0;

	count -= off;
	count -= 1;		/* for NUL at end */
	if (count < 0)
		return -EINVAL;

	code = dump_menu(buffer);
	if (code > 0) {
		buffer += code;
		count -= code;
	}

	*eof = 1;
	return (buffer - page) - off;
}

void hset_proc_delete(char *name, struct usb_hset *hset)
{
	if (hset->pde)
		remove_proc_entry(name, NULL);
}

struct proc_dir_entry *hset_proc_create(char *name, struct usb_hset *hset)
{
	struct proc_dir_entry	*pde;


	printk("BCERTIF:%s()\n", __FUNCTION__);
	/* FIXME convert everything to seq_file; then later, debugfs */

	if (!name || !hset)
		return NULL;

	hset->pde = pde = create_proc_entry(name,
				     S_IFREG | S_IRUGO | S_IWUSR, NULL);


	printk("BCERTIF:%s(): trying to register /proc/%s\n", __FUNCTION__, name);
	if (pde) {
		pde->data = (void *)hset;
		// pde->owner = THIS_MODULE;

		pde->read_proc = hset_proc_read;
		pde->write_proc = hset_proc_write;

		pde->size = 0;

		pr_debug("Registered /proc/%s\n", name);
	} else {
		pr_debug("Cannot create a valid proc file entry");
	}

	return pde;
}



/* ULPI_REG_CONTROL */
#define MUSB_ULPI_REG_CONTROL	0x76	/* 8 bit */
#define MUSB_ULPI_REG_REQ	(1 << 0)
#define MUSB_ULPI_REG_CMPLT	(1 << 1)


static int musb_write_ulpi_reg(struct musb* musb, u8 address, u8 value)
{
	u8 ctl = 0;
	u8* pBase = musb->mregs;



	/* ensure not powered down */
	if(musb_readb(pBase, MUSB_POWER) & MUSB_POWER_ENSUSPEND) {
		return -ENODEV;
	}

	/* polled */
	musb_writeb(pBase, MGC_O_HDRC_ULPI_REGADDR, address);
	musb_writeb(pBase, MGC_O_HDRC_ULPI_REGDATA, value);
	musb_writeb(pBase, MGC_O_HDRC_ULPI_REGCTL, MUSB_ULPI_REG_REQ);

	while(!(MUSB_ULPI_REG_CMPLT & ctl)) {
		ctl = musb_readb(pBase, MUSB_ULPI_REG_CONTROL);
	}

	musb_writeb(pBase, MUSB_ULPI_REG_CONTROL, 0);

	return 0;
}
