/*
 * linux/arch/arm/mach-omap2/board-fidji.c
 *
 * Copyright (C) 2010 Parrot SA
 * David Guilloteau <david.guilloteau@parrot.com>
 *
 * Modified from mach-omap2/board-zoom2.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/tsc2007.h>
#include <../drivers/parrot/input/touchscreen/rohm_bu21023.h>
#include <../drivers/parrot/input/touchscreen/atmel_mxt_ts.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mmc/host.h>
#include <linux/pwm_backlight.h>

#include <linux/switch.h>
#include <plat/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <linux/reboot.h>

#include <plat/mcspi.h>
#include <plat/gpio.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <plat/timer-gp.h>
#include <plat/pwm.h>
#include <plat/vrfb.h>

#include <plat/usb.h>
#include <plat/mux.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <plat/control.h>

#include <plat/display.h>
//#include <plat/hdq.h>

#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif

#include <asm/mach/flash.h>
#include <plat/board.h>
#include <plat/gpmc.h>
#include <plat/opp_twl_tps.h>
#include <../drivers/parrot/usb/otg/dummy-smsc-usb43340.h>

#include "hsmmc.h"
#include "smartreflex-class3.h"

#include "omap3-opp.h"

#include "parrot-common.h"

#define	FIDJI_EXT_PHY_RESET_GPIO (104)
#define FIDJI_NAND_CS 0

static int board_rev = 0;

#define GPIO_BTWIFI_RESET   24
#define GPIO_GPS_UPDATE     25
#define GPIO_IPOD_LEV_SH    29
#define GPIO_IPOD_RESET    163
#define GPIO_BOARD_REV      94
#define GPIO_LCD_ONOFF     170
#define GPIO_LCD_EN        109
#define GPIO_MMC1_CD       107
#define GPIO_MMC1_WP       108
#define GPIO_TS_PENIRQ     103
#define GPIO_TS_RESET      102
#define GPIO_GPS_ONOFF     171
#define GPIO_GPS_RESET     173
#define GPIO_RF_SDN        172
#define GPIO_RF_IRQ        177
#define GPIO_RF_LSH        174
#define GPIO_RESET_PHY_1   104
#define GPIO_RESET_PHY_0   164

static void omap_fidji_power_off(void)
{
	if(board_rev < 8 && board_rev != 0) // board_rev != 0 if print_board_rev is not called
		gpio_set_value(GPIO_RESET_PHY_0, 0);
	else
		gpio_set_value(GPIO_RESET_PHY_1, 0);
	/* reboot if we are still alive after 1 s */
	local_irq_disable();
	mdelay(1000);
	machine_restart(NULL);
}

static void print_board_rev(void)
{
	int r, i;

	for (i = 0; i < 4; i++) {
		omap_mux_init_gpio(GPIO_BOARD_REV + i, OMAP_PIN_INPUT);

		r = gpio_request(GPIO_BOARD_REV + i, "fidji gpio");
		gpio_direction_input(GPIO_BOARD_REV + i);
		if (r)
			printk(KERN_WARNING "Could not request GPIO %d\n", GPIO_BOARD_REV + i);

		r = gpio_direction_input(GPIO_BOARD_REV + i);
		if (r)
			printk(KERN_WARNING "Could not config GPIO %d\n", GPIO_BOARD_REV + i);

		if (gpio_get_value(GPIO_BOARD_REV + i))
			board_rev |= (1 << i);
	}

	//Save the board revision
	system_rev = board_rev;

	printk(KERN_INFO "FIDJI board revision : %d\n", board_rev);

	return;
}

static int fidji_board_rev(void)
{
	return board_rev;
}

int fidji_pins[] = {
	24, //bt-rst
	172, //rf-sdn
	163, //ipod
	//XXX to be revomed after proper reboot
	164,
	-1,
};


/*** usb *****/
static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {
	.port_mode[0]		= EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1]		= 0,
	.port_mode[2]		= 0,
	.phy_reset  = true,
	.reset_gpio_port[0]  = 104,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static struct smsc43340_usb_data fidji_usb_otg_data = {
	/* pins must be set before being used */
	.gpio_reset		= 164,
	.gpio_overcurrent	= -EINVAL,
};

struct platform_device fidji_usb_otg_device = {
	.name		= "smsc43340_usbotg",
	.id		= -1,
	.dev		= {
		.platform_data = &fidji_usb_otg_data,
	},
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static void usb_init(void)
{
	// reset PHY
	parrot_gpio_out_init(164, 1); // gpio is requested to be used for the product shutdown

	omap_mux_init_gpio(104, OMAP_PIN_OUTPUT);

	//XXX check with hw team
	omap_mux_init_signal("hsusb0_clk.hsusb0_clk", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_stp.hsusb0_stp", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("hsusb0_dir.hsusb0_dir", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_nxt.hsusb0_nxt", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data0.hsusb0_data0", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data1.hsusb0_data1", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data2.hsusb0_data2", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data3.hsusb0_data3", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data4.hsusb0_data4", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data5.hsusb0_data5", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data6.hsusb0_data6", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data7.hsusb0_data7", OMAP_PIN_INPUT);

	usb_ehci_init(&ehci_pdata);
	usb_musb_init(&musb_board_data);
}

/****** spi ******/
static struct omap2_mcspi_device_config fidji_mcspi_config = {
	.turbo_mode		= 0,
	.single_channel		= 1,  /* 0: slave, 1: master */
};


static struct omap2_mcspi_platform_config fidji_mcspi_platform_data = {
	.num_cs = 1,
	.mode = OMAP2_MCSPI_MASTER,
	.dma_mode = 0,
	.force_cs_mode = 1,
	.fifo_depth = 0,
	.regs_data = 0
};


static struct spi_board_info fidji_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "spidev",
		.bus_num		= 2,
		.chip_select		= 0,
		.max_speed_hz		= 5000000,
		.controller_data 	= &fidji_mcspi_config,
		.mode			= SPI_MODE_0,
	},
};


/**** display *******/
/* On HW 8, GPIO_LCD_ONOFF is for touchscreen too */
/* so don't modify it after init */
static int fidji_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	/* reg en */
	if (fidji_board_rev() != 8)
		gpio_set_value(GPIO_LCD_ONOFF, 0);
	/* lcd en */
	if (fidji_board_rev() != 7)
		gpio_set_value(GPIO_LCD_EN, 1);
	return 0;
}


static void fidji_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	/* reg en */
	if (fidji_board_rev() != 8)
		gpio_set_value(GPIO_LCD_ONOFF, 1);
	/* lcd en */
	if (fidji_board_rev() != 7)
		gpio_set_value(GPIO_LCD_EN, 0);
}

static struct omap_dss_device fidji_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	//.driver_name = "chimei_lte_panel",
	.driver_name = "hannstar_lte_panel",
	.phy.dpi.data_lines = 24,
	.platform_enable = fidji_panel_enable_lcd,
	.platform_disable = fidji_panel_disable_lcd,
	.panel.width_in_mm = 108,
	.panel.height_in_mm = 65,
	.channel = OMAP_DSS_CHANNEL_LCD,
};

static struct omap_dss_device *fidji_dss_devices[] = {
	&fidji_lcd_device,
};

static struct omap_dss_board_info fidji_dss_data = {
	.num_devices = ARRAY_SIZE(fidji_dss_devices),
	.devices = fidji_dss_devices,
	.default_device = &fidji_lcd_device,
};

static void __init display_init(void)
{
	int i;

	/* disable hsync & vsync */
	omap_mux_init_signal("dss_hsync.gpio_67",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("dss_vsync.gpio_68",
			OMAP_PIN_INPUT);

	/* need for 3630 */
	for (i = 0; i < 24; i++) {
		char mux_name[25];
		sprintf(mux_name, "dss_data%i.dss_data%i", i, i);
		omap_mux_init_signal(mux_name, OMAP_PIN_OUTPUT);
	}
	omap_mux_init_signal("dss_acbias.dss_acbias", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_pclk.dss_pclk", OMAP_PIN_OUTPUT);

	omap_mux_init_gpio(GPIO_LCD_ONOFF, OMAP_PIN_INPUT);

	omap_mux_init_signal("mcspi2_cs1.gpt8_pwm_evt",
			OMAP_PIN_OUTPUT);

/*      *     GPIO_LC_EN (109)          *    GPIO_LC_ONOFF (170)       *
 ***********************************************************************
 * HW 7 * GPIO NOT USED                 * 1V8 LCD_1V8_nEN              *
 *      *                               *  -> nEN_1V8_LS_LCD (RLS LCD) *
 ***********************************************************************
 * HW 8 * 1V8 HZL LCD_1V8_POWER         * 1V8 LCD_1V8_nEN              *
 *      *  -> EN_1V8_LS_POWER (RLS LCD) *  -> nEN_1V8_LS_LCD (RLS LCD) *
 ***********************************************************************/
	if (fidji_board_rev() == 8) {
		//Ugly Patch for Hardware
	        /* On HW 8, GPIO_LCD_ONOFF is for touchscreen too */
		parrot_gpio_out_init(GPIO_LCD_ONOFF, 0);
		mdelay(10);
		gpio_set_value(GPIO_LCD_ONOFF, 1);
		mdelay(10);
		gpio_set_value(GPIO_LCD_ONOFF, 0);
	} else
		parrot_gpio_out_init(GPIO_LCD_ONOFF, 1);
	
	if (fidji_board_rev() != 7) {
		parrot_gpio_out_init(GPIO_LCD_EN, 1);
	}

	fidji_lcd_device.panel.width_in_mm = fidji_lcd_device.panel.width_in_mm * (800 - sgx_reduce_pixels_left - sgx_reduce_pixels_right) / 800;
	fidji_lcd_device.panel.height_in_mm = fidji_lcd_device.panel.height_in_mm * (480 - sgx_reduce_pixels_down) / 480;
	omap_display_init(&fidji_dss_data);
}

/***** extra device ******/

struct platform_device fidji_gpio = {
	.name           = "gpio",
	.id             = -1,
	.num_resources  = 0,
	.resource       = NULL,
};

struct platform_device fidji_acpower_device = {
	.name		= "acpower",
	.id		= -1,
};

/***** i2c *******/

/* PMU */

/* PMU TPS65023 */
static struct regulator_consumer_supply vdcdc1_supply[] = {
	{
		.supply		= "vdcdc1_1v2_mpu_iva",
	}
};

static struct regulator_consumer_supply vdcdc2_supply[] = {
	{
		.supply		= "vdcdc2_1v8",
	}
};

static struct regulator_consumer_supply vdcdc3_supply[] = {
	{
		.supply		= "vdcdc3_1v2_c",
	}
};

static struct regulator_consumer_supply vldo1_supply[] = {
	{
		.supply		= "ldo2_1v8_pll",
	}
};

static struct regulator_consumer_supply vldo2_supply[] = {
	{
		.supply		= "ldo2_gps",
	},
};

static struct regulator_init_data regulator_init_data[] = {
	/* VDCDC1 /  1V2_MPU_IVA / OMAP 3630 MPU and IVA power supply */
	{
		.constraints = {
			.name			= "VDCDC1 / 1V2_MPU_IVA",
			.min_uV			= 1200000,
			.max_uV			= 1200000,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL,
			.valid_ops_mask		= 0,

			.always_on = 1, /* never turn this off */
		},
		.num_consumer_supplies  = ARRAY_SIZE(vdcdc1_supply),
		.consumer_supplies      = vdcdc1_supply,
        },
	/* VDCDC2 /  1V8 / Board prower supply */
	{
		.constraints = {
			.name			= "VDCDC2 / 1V8",
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL,
			.valid_ops_mask		= 0,

			.always_on = 1, /* never turn this off */
		},
		.num_consumer_supplies  = ARRAY_SIZE(vdcdc2_supply),
		.consumer_supplies      = vdcdc2_supply,
        },
	/* VDCDC3 /  1V2_C / Omap 3630 Core power supply */
	{
		.constraints = {
			.name			= "VDCDC3 / 1V2_C",
			.min_uV			= 1200000,
			.max_uV			= 1200000,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL,
			.valid_ops_mask		= 0,

			.always_on = 1, /* never turn this off */
		},
		.num_consumer_supplies  = ARRAY_SIZE(vdcdc3_supply),
		.consumer_supplies      = vdcdc3_supply,
        },
	/* LDO1 / 1V8_PLL  */
	{
		.constraints = {
			.name			= "LDO1 / 1V8_PLL",
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL,

			.always_on = 1, /* never turn this off */
		},
		.num_consumer_supplies  = ARRAY_SIZE(vldo1_supply),
		.consumer_supplies      = vldo1_supply,
    },
	/* LDO2 / GPS  */
	{
		.constraints = {
			.name			= "LDO2 / GPS",
			.min_uV			= 3300000,
			.max_uV			= 3300000,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL,
			.apply_uV		= true,
			/* if we don't set REGULATOR_CHANGE_VOLTAGE mmc fail to enable it */
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,

			.always_on = 1, /* never turn this off */
		},
		.num_consumer_supplies  = ARRAY_SIZE(vldo2_supply),
		.consumer_supplies      = vldo2_supply,
        }
};

static struct i2c_board_info __initdata fidji_i2c_bus1_info_pmu_tps65023[] = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &regulator_init_data,
	},
};


/**** mmc ****/
static struct omap2_hsmmc_info mmc_rev2[] = {
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= GPIO_MMC1_CD,
		.gpio_wp	= GPIO_MMC1_WP,
	},
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask       = 0x00100000,
		.max_freq	= 24000000,
	},
	{}	/* Terminator */
};

static void fidji_mmc_init(void)
{
	u32 reg;
	/* MMC2 CD and CLKIN */
	omap_mux_init_signal("sdmmc2_clkin",
			OMAP_PIN_INPUT);

	/*
	 * MMC2 has been designed to be used with a level shifter
	 * but we are NOT using anyone.
	 */
	reg = omap_ctrl_readl(OMAP343X_CONTROL_DEVCONF1);
	reg |= OMAP2_MMCSDIO2ADPCLKISEL;
	omap_ctrl_writel(reg, OMAP343X_CONTROL_DEVCONF1);

	/* MMC1 CD and WP */
	omap_mux_init_gpio(GPIO_MMC1_CD, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_MMC1_WP, OMAP_PIN_INPUT);
	omap2_hsmmc_init(mmc_rev2);
}

static u8 mxt224e_read_chg(void)
{
        printk(KERN_ERR "MXT READ_CHANGE\n");
	return mxt224e_read_irq();
}

static const u8 mxt224e_config_1aa[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x1E, 0x00, 0x02, 0x0A, 0x00,
	0x00, 0x02, 0x00, 0x0A, 0x05, 0x8F, 0x00, 0x00,
        0x13, 0x0B, 0x00, 0x00, 0x23, 0x02, 0x06, 0x00,
        0x02, 0x01, 0x40, 0x0A, 0x0A, 0x0A, 0x0A, 0x20,
        0x03, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x40,
        0x00, 0x00, 0x00, 0x0A, 0x09, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xA9, 0x7F, 0x9A, 0x0E, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x34, 0x61,
	0xA0, 0x4F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x32, 0x1E, 0x1E, 0x00, 0x00, 0x0A, 0x00, 0x00,
	0x03, 0x18, 0x18, 0x00, 0x00, 0x01, 0x00, 0x0A,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0xC0, 0x72, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x20,
	0x20, 0x00, 0x00, 0x30, 0x08, 0x10, 0x20, 0x00,
	0x14, 0x04, 0x00, 0x20, 0x00, 0x0F, 0x00, 0x00,
	0x00, 0x00, 0x05, 0x00, 0x00, 0x28, 0x01, 0x02,
	0x01, 0x4F, 0x0A, 0x0A, 0x0A, 0x00, 0x00, 0x00,
	0x00, 0x40, 0x00, 0x00, 0x00, 0x0A, 0x0A, 0x01
};

static const u8 mxt224e_config_2ab[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x00, 0x18, 0x00, 0x01, 0x0A, 0x00,
	0x00, 0x01, 0x00, 0x0A, 0x05, 0x8F, 0x00, 0x00,
	0x13, 0x0B, 0x00, 0x00, 0x3C, 0x02, 0x06, 0x00,
	0x02, 0x01, 0x40, 0x0A, 0x0A, 0x0A, 0x0A, 0x20,
	0x03, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x40,
	0x00, 0x00, 0x00, 0x0A, 0x09, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0x00, 0xC8, 0x32, 0x40, 0x1F, 0x00, 0x32,
	0x1E, 0x1E, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x03,
	0x20, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x14, 0x46, 0x32, 0x00, 0x00, 0x00, 0xBE, 0x00,
	0x00, 0x01, 0xC4, 0x72, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x20, 0x20,
	0x00, 0x00, 0x30, 0x08, 0x10, 0x00, 0x00, 0x14,
	0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00,
	0x00, 0x05, 0x00, 0x00, 0x1E, 0x01, 0x01, 0x01,
	0x30, 0x0A, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00,
	0x40, 0x00, 0x00, 0x00, 0x14, 0x0A, 0x01, 0x00,
	0x08, 0xED, 0x00
};

static struct mxt_platform_data  mxt224e_data = {
	.config_1 = &mxt224e_config_1aa[0],
	.config_1_length = sizeof(mxt224e_config_1aa),
	.config_1_crc = 0xD0AF83,

	.config_2 = &mxt224e_config_2ab[0],
	.config_2_length = sizeof(mxt224e_config_2ab),
	.config_2_crc = 0x3E8CC9,

	.orient = MXT_HORIZONTAL_FLIP,
	.irqflags = IRQF_TRIGGER_FALLING,
	.read_chg = mxt224e_read_chg,
	.x_size = 800,
	.y_size = 480,
	//.x_line = Hook in config
	//.y_line = Hook in config
	//.threshold = Hook in config
};

static struct i2c_board_info __initdata fidji_i2c_tsc2007_rev7[] = {
	{
		I2C_BOARD_INFO("tsc2007", 0x49),
		.flags = I2C_CLIENT_WAKE,
		/*.irq = 0, set by tsc2007_init */
	},
};

static struct i2c_board_info __initdata fidji_i2c_bus3_info_rev7[] = {
	{
		I2C_BOARD_INFO(ROHM_I2C_NAME,  ROHM_I2C_Address),
		//.platform_data = &rohm_bu21018_config,
		/*.irq = OMAP_GPIO_IRQ(OMAP3_DEVKIT_TS_GPIO),*/
	}
};

/* Touchscreen */
static void omap_fidji_rohm_bu21023_init(void)
{
	int ret;

	omap_mux_init_gpio(GPIO_TS_PENIRQ, OMAP_PIN_INPUT_PULLUP);
	ret = gpio_request(GPIO_TS_PENIRQ, "ROHM BU21023_pen_down");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for ""ROHM BU21023 pen down IRQ\n", GPIO_TS_PENIRQ);
		return;
	}
	gpio_direction_input(GPIO_TS_PENIRQ);
}

static struct i2c_board_info __initdata fidji_i2c_mxt224e[] = {
	{
		I2C_BOARD_INFO("tnx_mxt_ts", 0x4a),
		.platform_data = &mxt224e_data,
		/*.irq = 0, set by mxt224e_init */
	}
};

static void __init omap_i2c_init(void)
{
	parrot_omap_i2c_init(1);
	parrot_omap_i2c_init(2);
	parrot_omap_i2c_init(3);
	parrot_omap_i2c_init(4);

	/* PMU */
	if(fidji_board_rev() < 9) {
		regulator_init_data[4].constraints.min_uV =
			regulator_init_data[4].constraints.max_uV = 2500000;
	}
	omap_register_i2c_bus(1, 100, NULL, fidji_i2c_bus1_info_pmu_tps65023,
			ARRAY_SIZE(fidji_i2c_bus1_info_pmu_tps65023));

	omap_mux_init_signal("i2c2_scl", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("i2c2_sda", OMAP_PIN_INPUT_PULLUP);

	/* Ipod chip */
	omap_register_i2c_bus(2, 10, NULL, NULL, 0);

	/* Touchscreen */
	omap_mux_init_signal("i2c3_scl", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("i2c3_sda", OMAP_PIN_INPUT_PULLUP);
	if (fidji_board_rev() == 7) {
		parrot_gpio_out_init(GPIO_TS_RESET, 0);
		mdelay(10);
		gpio_set_value(GPIO_TS_RESET, 1);
		tsc2007_init(GPIO_TS_PENIRQ, &fidji_i2c_tsc2007_rev7[0]);
		omap_register_i2c_bus(3, 2600, NULL, fidji_i2c_tsc2007_rev7,
				ARRAY_SIZE(fidji_i2c_tsc2007_rev7));
	}
	else if (fidji_board_rev() == 8) {
		omap_fidji_rohm_bu21023_init();
		fidji_i2c_bus3_info_rev7[0].irq = gpio_to_irq(GPIO_TS_PENIRQ);
		omap_register_i2c_bus(3, 400, NULL, fidji_i2c_bus3_info_rev7,
				ARRAY_SIZE(fidji_i2c_bus3_info_rev7));
	}
	else if (fidji_board_rev() >= 9) {
		printk(KERN_ERR "MXT BOARD INIT\n");
		mxt224e_init(GPIO_TS_PENIRQ, GPIO_TS_RESET, &fidji_i2c_mxt224e[0], 0);
		omap_register_i2c_bus(3, 400 , NULL, fidji_i2c_mxt224e, ARRAY_SIZE(fidji_i2c_mxt224e));
	}
}

/* NAND chip access: 16 bit */
static struct omap_nand_platform_data fidji_nand_data = {
	.nand_setup	= NULL,
	.dma_channel	= -1,	/* disable DMA in OMAP NAND driver */
	.dev_ready	= (void *)1,
	.devsize	= 1,	/* '0' for 8-bit, '1' for 16-bit device */
	.cs      = FIDJI_NAND_CS,
	.ecc_opt     = 0x2, /* HW ECC in romcode layout */
	.gpmc_baseaddr   = (void *)OMAP34XX_GPMC_VIRT,
	.gpmc_cs_baseaddr    =(void *)(OMAP34XX_GPMC_VIRT+GPMC_CS0_BASE+FIDJI_NAND_CS*GPMC_CS_SIZE),
};

static struct omap2_pwm_platform_config pwm_config = {
  .timer_id           = 8,   // GPT8_PWM_EVT
  .polarity           = 1     // Active-high
};

static struct platform_device pwm_device = {
  .name               = "omap-pwm",
  .id                 = 0,
  .dev                = {
    .platform_data  = &pwm_config
  }
};

static struct platform_pwm_backlight_data pwm_backlight_data = {
	.pwm_id         = 0,
	.max_brightness = 1000,
	.dft_brightness = 300,
	.dft_power      = 1,
	.pwm_period_ns  = 40000,
};

static struct platform_device pwm_backlight = {
  .name               = "pwm-backlight",
  .id                 = 0,
  .dev                = {
    .platform_data  = &pwm_backlight_data
  }
};

static void __init omap_fidji_init_irq(void)
{
	parrot_omap_init_irq(200);
}

static void peripherals_init(void)
{
	fidji_gpio.dev.platform_data = fidji_pins;

	parrot_omap_serial_init();
	fidji_mmc_init();
	omap_i2c_init();

	usb_init();
	spi_register_board_info(fidji_spi_board_info, ARRAY_SIZE(fidji_spi_board_info));
	omap2_init_mcspi(&fidji_mcspi_platform_data, 2);
}


static struct platform_device *fidji_devices[] __initdata = {
	&user_gpio,
	&fidji_gpio,
	&fidji_acpower_device,
	&fidji_usb_otg_device,
	&omap_33,
	&pwm_backlight,
	&pwm_device,
};

static void __init omap_fidji_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);

	print_board_rev();

	if (fidji_board_rev() < 7)
		panic("hardware not supported");

	peripherals_init();

	parrot_omap_gpmc_nand_config(&fidji_nand_data);

	gpmc_nand_init(&fidji_nand_data);

	display_init();

	sr_class3_init();

	parrot_omap_voltage_init();

	platform_add_devices(fidji_devices, ARRAY_SIZE(fidji_devices));



	/* config BT */
	/* PCM */
	omap_mux_init_signal("mcbsp1_dx.mcbsp3_dx",
			OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcbsp1_dr.mcbsp3_dr",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("mcbsp1_fsx.mcbsp3_fsx",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("mcbsp1_clkx.mcbsp3_clkx",
			OMAP_PIN_INPUT);

	/* UART */
	omap_mux_init_signal("uart1_tx.uart1_tx",
			OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart1_rx.uart1_rx",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("uart1_rts.uart1_rts",
			OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart1_cts.uart1_cts",
			OMAP_PIN_INPUT);

	/* reset */
	parrot_gpio_user_out_init(GPIO_BTWIFI_RESET, 1, "bt-rst");

	/* config GPS */
	/* UART */
	omap_mux_init_signal("mcbsp3_clkx.uart2_tx",
			OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcbsp3_fsx.uart2_rx",
			OMAP_PIN_INPUT);

	/* ON/OFF - RESET */
	parrot_gpio_out_init(GPIO_GPS_ONOFF, 0);

	if (fidji_board_rev() < 9) {
		mdelay(20);
		gpio_set_value(GPIO_GPS_ONOFF, 1);
		omap_mux_init_gpio(GPIO_GPS_RESET, OMAP_PIN_INPUT);
		parrot_gpio_out_init(GPIO_GPS_RESET, 0);
	}
	else  {
		// Hardware 09+
		omap_mux_init_gpio(GPIO_GPS_UPDATE, OMAP_PIN_INPUT);
		parrot_gpio_out_init(GPIO_GPS_UPDATE, 0);
	}

	/* config SPI-RF */
	/* SDN_RF*/
	parrot_gpio_user_out_init(GPIO_RF_SDN, 0, "rf-sdn");
	/* IRQ from the rf device */
	omap_mux_init_gpio(GPIO_RF_IRQ, OMAP_PIN_INPUT|OMAP_PIN_OFF_WAKEUPENABLE);

	/* SPI pin configuration */
	omap_mux_init_signal("mcspi2_clk.mcspi2_clk",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("mcspi2_simo.mcspi2_simo",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("mcspi2_somi.mcspi2_somi",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("mcspi2_cs0.mcspi2_cs0",
			OMAP_PIN_INPUT);

	/* level shifter */
	parrot_gpio_out_init(GPIO_RF_LSH, 0);

	/* IPOD chip */

	/* level shifter */
	parrot_gpio_out_init(GPIO_IPOD_LEV_SH, 1);
	/* reset */
	parrot_gpio_user_out_init(GPIO_IPOD_RESET, 1, "ipod-rst");


	/* put in safe mode unused pins */
	omap_mux_init_signal("gpmc_clk.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("gpmc_nbe1.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("gpmc_wait2.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("gpmc_wait3.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c4_scl.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c4_sda.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("sdmmc2_dat7.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcbsp3_dx.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcbsp3_dr.safe_mode", OMAP_PIN_INPUT);

	pm_power_off = omap_fidji_power_off;
}


MACHINE_START(OMAP_FIDJI, "fidji board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= parrot_omap_map_io,
	.init_irq	= omap_fidji_init_irq,
	.init_machine	= omap_fidji_init,
	.timer		= &omap_timer,
MACHINE_END
