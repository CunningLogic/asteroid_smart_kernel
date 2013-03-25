/*
 * linux/arch/arm/mach-omap2/board-ais.c
 *
 * Copyright (C) 2012 Parrot SA
 * Christian Rosalie <christian.rosalie@parrot.com>
 *
 * Modified from linux/arch/arm/mach-omap2/board-volvo.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <plat/common.h>
#include <plat/mux.h>
#include <plat/pwm.h>
#include <mach/board-fc6100.h>
#include "parrot-common.h"
#include <../drivers/parrot/input/touchscreen/atmel_mxt_ts.h>
#include <../drivers/parrot/i2c/smsc-82514-usb-hub.h>
#include <linux/pwm_backlight.h>


#define USB_HUB_RST_N			FC_SDIO_1_D0		/* see USB_HUB_RST_n on Schematics */
#define EMMC_RST_N			FC_SDIO_1_D1		/* see eMMC_RST_N on Schematics */

#define LVDS_SER_RST_N			FC_SDIO_1_D2		/* see LVDS_SER_RST_n on Schematics */
#define LVDS_SER_IT_N			FC_IT_TOUCHSCREEN_N	/* see LVDS_SER_IT_n on Schematics */
#define DISBUT_GLOBAL_RST		FC_GPIO_2_CLK		/* see FC_GPIO_2_CLK on Schematics */
#define I2C_DISBUT_IT_N			FC_SDIO_1_CMD		/* see I2C_DISBUT_IT_N on Schematics */

#define GPS_ENN				FC_GPIO_1_PWM		/* see GPS_ENn on Schematics */
#define CAM_IT_N			FC_UART_0_RTS		/* see CAM_IT_N on Schematics */
#define TMS_IT_N			FC_IT_HOST_N		/* see IT_HOST_N or TMS_ITn on schematics */
#define SPDIF_RST_N			FC_SDIO_1_D3		/* see SPDIF_RST_N on schematics */


/*	FC6100 Module - unused pin
*
*	Set to 3v3 :
*		FC_SDIO_1_WP_N, FC_SDIO_1_CD_N, FC_SDIO_0_CD_N, FC_SDIO_0_WP_N, USB_1_OC_N, USB_0_OC_N
*
*
*	Set to ground :
*		LINEIN2_R, LINEIN2_L, MIC2_P, MIC2_N
*		TV_CHROMA, TV_LUMA
*		SDIO_1_CLKIN, SDIO_1_CLK, I2S_IN0, I2S_IN1, USB_1_VBUS
*
*	Set to high impedance :
*		UART_0_CTS, USB_0_ID, USB_0_CPEN
*		LCD_RST_N, SPI_1_CS0_N, SPI_1_MISO
*		SPI_1_MOSI, SPI_1_CLK, LCD_BKL_EN
*
*/

int ais_pins[] = {
	/* Output */
	FC_IPOD_RESET_N,	//Ipod : Mandatory for FC6100 family product
	GPS_ENN,		// Enable GPS
	TMS_IT_N,		//TMS : spi chip select

	/* Input */
	RF_RST_N,		//BT reset : Mandatory for FC6100 family product
	TMS_IT_N,		//TMS IT
	-1,
};

static int ais_ver = 0;

static int ais_get_version(void)
{
	switch (fc6100_mod_get_motherboard_revision()) {
		case 0x24: //HW00
			return 0;
			break;
		case 0x25: //HW01
			return 1;
			break;
		case 0x26: //HW02
			return 2;
			break;
	}
	BUG_ON(1);
	return 0;
}


static struct omap2_pwm_platform_config ais_lcd_pwm_config = {
	.timer_id           = 9,   // GPT9_PWM_EVT LCD_BKL_PWM
	.polarity           = 1     // Active-high
};

static struct platform_device ais_lcd_pwm_device = {
	.name               = "omap-pwm",
	.id                 = 0,
	.dev                = { .platform_data  = &ais_lcd_pwm_config }
};

static int ais_lcd_backlight_pwm_notify(struct device *dev, int brightness)
{
	return brightness;
}

static struct platform_pwm_backlight_data ais_lcd_pwm_data = {
	.pwm_id         = 0,
	.max_brightness = 1000,
	.dft_brightness = 800,
	.dft_power      = 1,
	.pwm_period_ns  = 500000,  // 500hz
	.notify = ais_lcd_backlight_pwm_notify,
};

static struct platform_device ais_lcd_pwm_backlight = {
	.name               = "pwm-backlight",
	.id                 = 0,
	.dev                = { .platform_data  = &ais_lcd_pwm_data }
};


static struct omap2_pwm_platform_config ais_x86_pwm_config = {
	.timer_id           = 10,   // GPT10_PWM_EVT : see FC_GPIO_0_PWM or PWM_X86LEDS on Schematics
	.polarity           = 1     // Active-high
};

static struct platform_device ais_x86_pwm_device = {
	.name               = "omap-pwm",
	.id                 = 1,
	.dev                = { .platform_data  = &ais_x86_pwm_config }
};

static int ais_x86_pwm_notify(struct device *dev, int brightness)
{
	return brightness;
}

static struct platform_pwm_backlight_data ais_x86_pwm_data = {
	.pwm_id         = 1,
	.max_brightness = 1000,
	.dft_brightness = 800,
	.dft_power      = 1,
	.pwm_period_ns  = 500000,  // 500hz
	.notify = ais_x86_pwm_notify,
};

static struct platform_device ais_x86_pwm_leds = {
	.name               = "pwm-backlight",
	.id                 = 1,
	.dev                = { .platform_data  = &ais_x86_pwm_data }
};


static struct platform_device *ais_devices[] __initdata = {
	/* Related to gpt9_pwm */
	&ais_lcd_pwm_backlight,
	&ais_lcd_pwm_device,

	/* Related to gpt10_pwm */
	&ais_x86_pwm_leds,
	&ais_x86_pwm_device,
};




static int ais_panel_enable_lcd(struct omap_dss_device *dssdev)
{

	return 0;
}


static void ais_panel_disable_lcd(struct omap_dss_device *dssdev)
{
}


static struct omap_dss_device fc6100_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "kyocera-TCG070WVLPAANN",
	.phy.dpi.data_lines = 24,
	.platform_enable = ais_panel_enable_lcd,
	.platform_disable = ais_panel_disable_lcd,
	.panel.width_in_mm = 108,
	.panel.height_in_mm = 65,
	.channel = OMAP_DSS_CHANNEL_LCD,
};

static void __init omap_ais_devwb_display_init(void)
{
	/* Reset DISBUT chips */
	parrot_gpio_out_init(DISBUT_GLOBAL_RST, 0);

	/* Reset LVDS chips */
	parrot_gpio_out_init(LVDS_SER_RST_N, 1);


	fc6100_mod_display_init( &fc6100_lcd_device);
}

static struct spi_board_info fc6100_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "spidev",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 2400000,
		.controller_data 	= &fc6100_mcspi_config,
		.mode			= SPI_MODE_3,
	},
};


/* The panel is shipped with a revision 1.0 of the firmware
   "Family ID: 129 Variant ID: 1 Version: 1.0 Build: 0xAA Object Num: 18"
   The touchpanel is TP1101, the ID sample is APT1101G01-01) */
static const u8 mxt224e_ais_config[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x19, 0x00, 0x02, 0x0A, 0x00,
	0x00, 0x02, 0x00, 0x0A, 0x05, 0x8F, 0x00, 0x00,
	0x12, 0x0B, 0x00, 0x00, 0x14, 0x02, 0x02, 0x00,
	0x02, 0x01, 0x40, 0x0A, 0x0A, 0x0A, 0x0A, 0x20,
	0x03, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x40,
	0x00, 0x00, 0x00, 0x0A, 0x09, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xA9, 0x7F, 0x9A, 0x0E, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x28, 0x63,
	0x38, 0x4A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A,
	0x14, 0x10, 0x14, 0x00, 0x00, 0x0A, 0x00, 0x00,
	0x03, 0x23, 0x23, 0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x84, 0x42, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x10, 0x00, 0x00, 0x40, 0x08, 0x10, 0x20, 0x00,
	0x14, 0x04, 0x00, 0x20, 0x00, 0x05, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x04, 0x0A,
	0x01, 0x20, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F,
	0x0A, 0x40, 0x00, 0x00, 0x00, 0x28, 0x08, 0x04
};

static struct mxt_platform_data mxt224e_data = {
	.config_1 = &mxt224e_ais_config[0],
	.config_1_length = sizeof(mxt224e_ais_config),
	.config_1_crc = 0xA80577,
	.config_2 = &mxt224e_ais_config[0],
	.config_2_length = sizeof(mxt224e_ais_config),
	.config_2_crc = 0xA80577,
	.orient = MXT_HORIZONTAL_FLIP,
	.irqflags = IRQF_TRIGGER_FALLING,
	.x_size = 800,
	.y_size = 480,
};

/* The panel is shipped with a revision 2.0.AB of the firmware
   "Family ID: 129 Variant ID: 1 Version: 2.0.AB Object Num: 16"
   The touchpanel ID is FKJ08042-01
   with this configuration the driver required a firmware called maxtouch.cfg */
static struct mxt_platform_data mxt224e_hw01_data = {
	.orient = MXT_HORIZONTAL_FLIP,
	.irqflags = IRQF_TRIGGER_FALLING,
	.x_size = 800,
	.y_size = 480,
};


static struct smsc82514_pdata hub_init = {
	.ds_port_1 = DS_HIGH,
	.ds_port_2 = DS_HIGH,
	.reset_pin = USB_HUB_RST_N,
};

static struct i2c_board_info __initdata fc6100_ais_i2c_bus2_info[] = {
	{
		/* USB HUB SMSC 82514 */
		I2C_BOARD_INFO("smsc82514", 0x2c),
		.platform_data = &hub_init
	}
};

static struct i2c_board_info __initdata fc6100_ais_i2c_bus3_info[] = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", 0x4b),
		.platform_data = &mxt224e_data
	},
	{
		I2C_BOARD_INFO("lvds", 0xc),
	},
	{
		I2C_BOARD_INFO("mclaren_x86", 0x66),
	}

};

static void __init omap_ais_devwb_i2c_init(void)
{
	/* AIS Board */

	/* USB hub : init reset gpio */
	parrot_gpio_out_init(USB_HUB_RST_N, 0);

	/* Ipod chip and Nuvoton chip NAU8820 are managed by default */
	omap_register_i2c_bus(2, I2C_IPOD_CHIP_MAX_FREQ, NULL, fc6100_ais_i2c_bus2_info,
		ARRAY_SIZE(fc6100_ais_i2c_bus2_info));

	//TODO Check IT parameter for this call
	// IT should be available through i2c LVDS driver
	parrot_gpio_in_init(LVDS_SER_IT_N, OMAP_PIN_OFF_WAKEUPENABLE);
	fc6100_ais_i2c_bus3_info[1].irq = gpio_to_irq(LVDS_SER_IT_N);
	fc6100_ais_i2c_bus3_info[2].irq = -1;
	mxt224e_init(-1 /* LVDS_SER_IT_N */, -1, &fc6100_ais_i2c_bus3_info[0], 1);

	/* Update touchscreen settings for HW01 */
	if( ais_ver ){
		fc6100_ais_i2c_bus3_info[0].platform_data = &mxt224e_hw01_data;
	}

	omap_register_i2c_bus(3, 100, NULL, fc6100_ais_i2c_bus3_info,
		ARRAY_SIZE(fc6100_ais_i2c_bus3_info));
}


/*
 * Miscellaneous (board specific)
 */

static void __init omap_ais_devwb_miscellaneous( void )
{
	/* Enable GPS */
	parrot_gpio_out_init(GPS_ENN, 0);
	mdelay(20);
	gpio_set_value(GPS_ENN, 1);


	/* Reset eMMC */
	parrot_gpio_out_init(EMMC_RST_N, 0);
	mdelay(2);
	/*gpio_set_value(EMMC_RST_N, 1);
	mdelay(2);*/

	/* Reset SPDIF */
	parrot_gpio_out_init(SPDIF_RST_N, 0);
	mdelay(20);
	gpio_set_value(SPDIF_RST_N, 1);

	/* Used by RaggaSpi Userspace driver (FC6100 <->TMS470 SPI interface) */
	parrot_gpio_user_in_init(TMS_IT_N,OMAP_PIN_OFF_WAKEUPENABLE,"TMS_IT_N");

	/* Used by RaggaSpi to manually manage CS0 */
	parrot_gpio_user_out_init(FC_SPI_0_CS0_N,1,"mcspi1_cs0");

	/* PWM for lcd backligth (see LCD_BKL_PWM)
	   and x86 leds (see PWM_X86LEDS) */
	omap_mux_init_signal("gpmc_ncs4.gpt9_pwm_evt", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ncs5.gpt10_pwm_evt", OMAP_PIN_OUTPUT);
}


#define WB_AIS_CONFIG		FC6100_USE_SPI1_CS0		| \
				FC6100_USE_MMC1			| \
				FC6100_DISABLE_MMC1_WP_CD	| \
				FC6100_USE_ALL_MCBSP		| \
				FC6100_USE_ALL_I2C



static void __init omap_ais_init(void)
{
	fc6100_gpio.dev.platform_data = ais_pins;
	fc6100_mod_common_init(WB_AIS_CONFIG);

	if (fc6100_mod_get_pcb_revision() < FC6100_0_HW06) {
		panic("this module is not supported on AIS");
	}

	ais_ver = ais_get_version();

	/* I2c */
	omap_ais_devwb_i2c_init();

	/* Display */
	omap_ais_devwb_display_init();

	/* Miscellaneaous */
	omap_ais_devwb_miscellaneous();

	spi_register_board_info(fc6100_spi_board_info, ARRAY_SIZE(fc6100_spi_board_info));

	/* Init AIS devices */
	platform_add_devices(ais_devices, ARRAY_SIZE(ais_devices));
}


MACHINE_START(OMAP_AIS, "AIS board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= parrot_omap_map_io,
	.init_irq	= fc6100_mod_init_irq,
	.init_machine	= omap_ais_init,
	.timer		= &omap_timer,
MACHINE_END
