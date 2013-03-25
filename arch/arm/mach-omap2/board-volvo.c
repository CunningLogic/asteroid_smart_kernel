/*
 * linux/arch/arm/mach-omap2/board-fc6100.c
 *
 * Copyright (C) 2010 Parrot SA
 * Christian Rosalie <christian.rosalie@parrot.com>
 *
 * Modified from linux/arch/arm/mach-omap2/board-fc6100.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <plat/mux.h>
#include <mach/board-fc6100.h>
#include <linux/i2c/tsc2007.h>
#include <../drivers/parrot/input/touchscreen/atmel_mxt_ts.h>
#include <../drivers/parrot/i2c/smsc-82514-usb-hub.h>
#include "parrot-common.h"

#define LCD_RST_N			FC_LCD_RST_N/* see also LVDS_RST_N on schematic */
#define LCD_BKL_EN_HW03			FC_LCD_BKL_EN
#define IT_TOUCHSCREEN_N		FC_IT_TOUCHSCREEN_N

#define USB_HUB_RESET_N			FC_LCD_BKL_PWM /* see USB_HUB_RESETn on Schematics */
#define USB_HUB_RESET_N_HW05		FC_SDIO_1_D0   /* see USB_HUB_RESETn on Schematics */
#define GPS_ENN				FC_GPIO_0_PWM	/* see GPS_ENn on Schematics */
#define LVDS_BIST_EN			FC_GPIO_1_PWM	/* use only for test purpose */

#define MOTION_INT			FC_UART_0_RTS	/* see MOTION_INT_TO_FC6100 on Schematics */
#define WB_GPIO_02_HW02			FC_GPIO_2_CLK
#define ETH_MAX3100_SPI_DBG_IT_GPIO	FC_GPIO_2_CLK

#define TMS_IT_N			FC_IT_HOST_N	/* see IT_HOST_N or TMS_ITn on schematic */


static int volvo_ver = 0;

static int volvo_get_version(void)
{
	switch (fc6100_mod_get_motherboard_revision()) {
		case 0x10:
			return 0;
			break;
		case 0x11:
			return 1;
			break;
		case 0x12:
			return 2;
			break;
		case 0x13:
			return 3;
			break;
		case 0x14:
			return 4;
			break;
		case 0x15:
			return 5;
			break;
		case 0x16:
			return 6;
			break;
		case 0x17:
			return 7;
			break;
	}
	BUG_ON(1);
	return -1;
}

int volvo_pins[] = {
	/* Output */
	FC_IPOD_RESET_N,	//Ipod : Mandatory for FC6100 family product
	GPS_ENN,		//gpio0
	LVDS_BIST_EN,		//gpio1
	WB_GPIO_02_HW02,	//gpio2
	FC_SPI_0_CS0_N,		//TMS : spi cs0

	/* Input */
	RF_RST_N,		//BT reset : Mandatory for FC6100 family product
	TMS_IT_N,		//TMS IT : spi irq
	-1,
};


static int ironbox_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_RST_N, 1);

	/* PROTO HW03 and followings */
	gpio_set_value(LCD_BKL_EN_HW03, 1);

	return 0;
}


static void ironbox_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_RST_N, 0);

	/* PROTO HW03 and followings */
	gpio_set_value(LCD_BKL_EN_HW03, 0);
}


static struct omap_dss_device fc6100_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "tpo_laj07t001a_panel",
	.phy.dpi.data_lines = 24,
	.platform_enable = ironbox_panel_enable_lcd,
	.platform_disable = ironbox_panel_disable_lcd,
	.panel.width_in_mm = 108,
	.panel.height_in_mm = 65,
	.channel = OMAP_DSS_CHANNEL_LCD,
};

static void __init omap_ironbox_devwb_display_init(void)
{
	// LCD RST, BKL_EN, BKL_PWM
	parrot_gpio_out_init(LCD_RST_N, 0);

	// PROTO HW03 and followings
	parrot_gpio_out_init(LCD_BKL_EN_HW03, 0);

	fc6100_mod_display_init( &fc6100_lcd_device);
}

/* ************************************* SPI definition ********************************** */
#if defined(CONFIG_SERIAL_MAX3100) || defined(CONFIG_SERIAL_MAX3100_MODULE)
static struct plat_max3100 max3100_plat_data = {
		.loopback  = 0,
		.crystal   = 1,	 // 3.6864 Mhz external crystal
		.poll_time = 100,   //TODO must be check according to the CPU load
};
#endif

// fast Ethernet controller on SPI1
static struct spi_board_info fc6100_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "spidev",
		.bus_num		= 3,
		.chip_select		= 0,
		.max_speed_hz		= 5000000,
		.controller_data 	= &fc6100_mcspi_config,
		.mode			= SPI_MODE_0,
	},
#if defined(CONFIG_SERIAL_MAX3100) || defined(CONFIG_SERIAL_MAX3100_MODULE)
	[1] = {
		.modalias		= "max3100",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 3000000,
		.controller_data 	= &fc6100_mcspi_config,
		.mode			= SPI_MODE_0,		  //see diagramm in datasheet p3
		.platform_data		  = &max3100_plat_data,

	}
#else
	[1] = {
		.modalias		= "spidev",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 2400000,
		.controller_data 	= &fc6100_mcspi_config,
		.mode			= SPI_MODE_3,
	},
#endif
};


static struct smsc82514_pdata hub_init = {
	.ds_port_2 = DS_HIGH,
	.reset_pin = USB_HUB_RESET_N_HW05,
};

static struct i2c_board_info __initdata fc6100_ironbox_i2c_bus2_info[] = {
	{
		/* USB HUB SMSC 82514 */
		I2C_BOARD_INFO("smsc82514", 0x2c),
		.platform_data = &hub_init
	}
};

static void __init omap_ironbox_devwb_i2c_init(void)
{
	/* Ipod chip and Nuvoton chip NAU8820 are managed by default */
	omap_register_i2c_bus(2, I2C_IPOD_CHIP_MAX_FREQ, NULL, fc6100_ironbox_i2c_bus2_info,
		ARRAY_SIZE(fc6100_ironbox_i2c_bus2_info));

	omap_register_i2c_bus(3, 100, NULL, NULL, 0 );
}



#define ETH_MAX3100_TEXT "max3100 irq"

#if defined(CONFIG_SERIAL_MAX3100) || defined(CONFIG_SERIAL_MAX3100_MODULE)
static void __init eth_max3100_init(void)
{
	/* PROTO HW03 and followings */

	parrot_gpio_out_init(ETH_MAX3100_SPI_DBG_RST_N_GPIO58_HW02, 1);

	omap_mux_init_gpio(ETH_MAX3100_SPI_DBG_IT_GPIO,
			OMAP_PIN_INPUT); /* SPI DBG IT */

	fc6100_spi_board_info[2].irq = OMAP_GPIO_IRQ(ETH_MAX3100_SPI_DBG_IT_GPIO);

	if (gpio_request(ETH_MAX3100_SPI_DBG_IT_GPIO, ETH_MAX3100_TEXT) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for %s\n",
				 ETH_MAX3100_SPI_DBG_IT_GPIO, ETH_MAX3100_TEXT);
		return;
	}
	gpio_direction_input(ETH_MAX3100_SPI_DBG_IT_GPIO);

}
#endif


/*
 * Miscellaneous (board specific)
 */

static void __init omap_ironbox_devwb_miscellaneous( void )
{
	/* USB hub */
	if( volvo_ver >= 5 ){
		parrot_gpio_out_init(hub_init.reset_pin, 0);
	}else{
		parrot_gpio_out_init(USB_HUB_RESET_N, 0);
		mdelay(20);
		gpio_set_value(USB_HUB_RESET_N, 1);
	}

	/* Enable GPS */
	parrot_gpio_out_init(GPS_ENN, 0);
	mdelay(20);
	gpio_set_value(GPS_ENN, 1);

	/* Used by RaggaSpi Userspace driver (FC6100 <->TMS470 SPI interface) */
	parrot_gpio_user_in_init(TMS_IT_N,OMAP_PIN_OFF_WAKEUPENABLE,"TMS_IT_N");

	/* Used by RaggaSpi to manually manage CS0 */
	parrot_gpio_user_out_init(FC_SPI_0_CS0_N,1,"mcspi1_cs0");

	/* PROTO HW05 and followings */

	/* Init GPIO 0, 1, 2 of the WB (for test purpose) as output */
	parrot_gpio_out_init(LVDS_BIST_EN, 0);

	parrot_gpio_out_init(WB_GPIO_02_HW02, 0);

	if(volvo_ver >= 3)
	{
		/* Reserve for future use */
		parrot_gpio_user_in_init(MOTION_INT, OMAP_PIN_OFF_WAKEUPENABLE, "video-in-irq");
	}
}


#define WB_IRONBOX_CONFIG	FC6100_USE_SPI_ALL		| \
				FC6100_USE_ALL_MMC		| \
				FC6100_USE_ALL_MCBSP		| \
				FC6100_USE_ALL_I2C		| \
				FC6100_USE_LCD_HSYNC_VSYNC



static void __init omap_ironbox_init(void)
{
	fc6100_gpio.dev.platform_data = volvo_pins;
	fc6100_mod_common_init(WB_IRONBOX_CONFIG);

	volvo_ver = volvo_get_version();

	if (fc6100_mod_get_pcb_revision() < FC6100_0_HW05) {
		panic("this module is not supported on Ironbox");
	}

	/* I2c */
	omap_ironbox_devwb_i2c_init();

	/* Display */
	omap_ironbox_devwb_display_init();

	/* Miscellaneaous */
	omap_ironbox_devwb_miscellaneous();

#if defined(CONFIG_SERIAL_MAX3100) || defined(CONFIG_SERIAL_MAX3100_MODULE)
	eth_max3100_init();
#endif
	spi_register_board_info(fc6100_spi_board_info, ARRAY_SIZE(fc6100_spi_board_info));
}


MACHINE_START(OMAP_VOLVO, "Ironbox board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= parrot_omap_map_io,
	.init_irq	= fc6100_mod_init_irq,
	.init_machine	= omap_ironbox_init,
	.timer		= &omap_timer,
MACHINE_END
