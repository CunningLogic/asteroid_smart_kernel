/*
 * linux/arch/arm/mach-omap2/board-hsae.c
 *
 * Copyright (C) 2012 Parrot SA
 * Christian Rosalie <christian.rosalie@parrot.com>
 *
 * Modified from linux/arch/arm/mach-omap2/board-fc6100.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <plat/common.h>
#include <plat/pwm.h>
#include <plat/mux.h>
#include <mach/board-fc6100.h>
#include <linux/i2c/tsc2007.h>
#include <linux/pwm_backlight.h>
#include "parrot-common.h"


int hsae_devwb_pins[] = {
	/* Output */
	FC_IPOD_RESET_N,	//Ipod : Mandatory for FC6100 family product

	/* GPIO 0, 1, 2 of the WB used for test purpose */
	FC_GPIO_0_PWM,
	FC_GPIO_1_PWM,
	FC_GPIO_2_CLK,

	/* Input */
	RF_RST_N,		//BT reset : Mandatory for FC6100 family product
	-1,
};

static struct omap2_pwm_platform_config hsae_pwm_config = {
  .timer_id           = 9,   // GPT9_PWM_EVT LCD_BKL_PWM
  .polarity           = 1     // Active-high
};

static struct platform_device hsae_pwm_device = {
  .name               = "omap-pwm",
  .id                 = 0,
  .dev                = { .platform_data  = &hsae_pwm_config }
};

static int hsae_pwm_lcd_notify(struct device *dev, int brightness)
{
	return brightness;
}

static struct platform_pwm_backlight_data hsae_pwm_backlight_data = {
	.pwm_id         = 0,
	.max_brightness = 1000,
	.dft_brightness = 400,
	.dft_power      = 1,
	.pwm_period_ns  = 1000000,  // 1Khz
	.notify = hsae_pwm_lcd_notify,
};

static struct platform_device hsae_pwm_backlight = {
  .name               = "pwm-backlight",
  .id                 = 0,
  .dev                = { .platform_data  = &hsae_pwm_backlight_data }
};


static struct platform_device *hsae_devices[] __initdata = {
	&hsae_pwm_backlight,
	&hsae_pwm_device,
};

static int hsae_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(FC_LCD_RST_N, 1);
	gpio_set_value(FC_LCD_BKL_EN, 1);

	return 0;
}


static void hsae_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(FC_LCD_RST_N, 0);
	gpio_set_value(FC_LCD_BKL_EN, 0);
}


static struct omap_dss_device hsae_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "tpo_laj07t001a_panel",
	.phy.dpi.data_lines = 24,
	.platform_enable = hsae_panel_enable_lcd,
	.platform_disable = hsae_panel_disable_lcd,
	.panel.width_in_mm = 108,
	.panel.height_in_mm = 65,
	.channel = OMAP_DSS_CHANNEL_LCD,
};

static void __init omap_hsae_devwb_display_init(void)
{
	parrot_gpio_out_init(FC_LCD_RST_N, 0);
	parrot_gpio_out_init(FC_LCD_BKL_EN, 0);

	fc6100_mod_display_init(&hsae_lcd_device);
}

static struct spi_board_info hsae_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "spidev",
		.bus_num		= 3,
		.chip_select		= 0,
		.max_speed_hz		= 5000000,
		.controller_data 	= &fc6100_mcspi_config,
		.mode			= SPI_MODE_0,
	},
	[1] = {
		.modalias		= "spidev",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 2400000,
		.controller_data 	= &fc6100_mcspi_config,
		.mode			= SPI_MODE_3,
	},
};


static struct i2c_board_info __initdata hsae_i2c_bus3_info[] = {
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.flags = I2C_CLIENT_WAKE,
	},
};

static void __init omap_hsae_devwb_i2c_init(void)
{
	/* Ipod chip and Nuvoton chip NAU8820 are managed by default */
	omap_register_i2c_bus(2, I2C_IPOD_CHIP_MAX_FREQ, NULL, NULL, 0);

	tsc2007_init(FC_IT_TOUCHSCREEN_N, &hsae_i2c_bus3_info[0]);
	omap_register_i2c_bus(3, 100, NULL, hsae_i2c_bus3_info,
		ARRAY_SIZE(hsae_i2c_bus3_info));
}


#define TMS_IT_N			FC_IT_HOST_N	/* see IT_HOST_N or TMS_ITn on schematic */
/*
 * Miscellaneous (board specific)
 * Generic GPIO 00/01/02 Outside the module
 */
static void __init omap_hsae_devwb_miscellaneous( void )
{

	/* PROTO HW02 and followings */

	/* Init GPIO 0, 1, 2 of the WB (for test purpose) as output */
	parrot_gpio_user_out_init(FC_GPIO_0_PWM, 0, NULL);

	parrot_gpio_user_out_init(FC_GPIO_1_PWM, 0, NULL);

	parrot_gpio_user_out_init(FC_GPIO_2_CLK, 0, NULL);

	/* Used by RaggaSpi Userspace driver */
	parrot_gpio_user_in_init(TMS_IT_N,OMAP_PIN_OFF_WAKEUPENABLE,"TMS_IT_N");

	/* Used by RaggaSpi to manage manually CS0 */
	omap_mux_init_signal("mcspi1_cs0.gpio_174", OMAP_PIN_INPUT);
	parrot_gpio_user_out_init(174,1,"mcspi1_cs0");
}

#define WB_HSAE_CONFIG		FC6100_USE_SPI_ALL		| \
				FC6100_USE_ALL_MMC		| \
				FC6100_USE_ALL_MCBSP		| \
				FC6100_USE_UART1_RTS_CTS	| \
				FC6100_USE_ALL_I2C		| \
				FC6100_USE_LCD_HSYNC_VSYNC

static void __init omap_hsae_init(void)
{
	fc6100_gpio.dev.platform_data = hsae_devwb_pins;

	fc6100_mod_common_init(WB_HSAE_CONFIG);

	/* PWM for backligth (see LCD_BKL_PWM) */
	omap_mux_init_signal("gpmc_ncs4.gpt9_pwm_evt", OMAP_PIN_OUTPUT);

	/* I2c */
	omap_hsae_devwb_i2c_init();

	/* Display */
	omap_hsae_devwb_display_init();

	/* Miscellaneaous */
	omap_hsae_devwb_miscellaneous();

	spi_register_board_info(hsae_spi_board_info, ARRAY_SIZE(hsae_spi_board_info));

	platform_add_devices(hsae_devices, ARRAY_SIZE(hsae_devices));
}


MACHINE_START(OMAP_HSAE, "hsae board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= parrot_omap_map_io,
	.init_irq	= fc6100_mod_init_irq,
	.init_machine	= omap_hsae_init,
	.timer		= &omap_timer,
MACHINE_END
