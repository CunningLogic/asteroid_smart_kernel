/*
 * linux/arch/arm/mach-omap2/board-fc6100.c
 *
 * Copyright (C) 2010 Parrot SA
 * Florent Bayendrian <florent.bayendrian@parrot.com>
 *
 * Modified from linux/arch/arm/mach-omap2/board-fidji.c
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
#include <../drivers/parrot/input/touchscreen/atmel_mxt_ts.h>
#include <linux/pwm_backlight.h>
#include "parrot-common.h"

#if ( (defined(CONFIG_KSZ8851SNL) || defined(CONFIG_KSZ8851SNL_MODULE)) \
	  && (defined(CONFIG_SERIAL_MAX3100) || defined(CONFIG_SERIAL_MAX3100_MODULE))  )
#error Cannot select KSZ8851SNL Ethernet chip and MAX3100 SPI/UART chip at the same time
#endif

int fc6100_devwb_pins[] = {
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

static struct omap2_pwm_platform_config pwm_config = {
  .timer_id           = 9,   // GPT9_PWM_EVT LCD_BKL_PWM
  .polarity           = 1     // Active-high
};

static struct platform_device pwm_device = {
  .name               = "omap-pwm",
  .id                 = 0,
  .dev                = { .platform_data  = &pwm_config }
};

static int fc6100_pwm_lcd_notify(struct device *dev, int brightness)
{
	return brightness;
}

static struct platform_pwm_backlight_data pwm_backlight_data = {
	.pwm_id         = 0,
	.max_brightness = 1000,
	.dft_brightness = 400,
	.dft_power      = 1,
	.pwm_period_ns  = 1000000,  // 1Khz
	.notify = fc6100_pwm_lcd_notify,
};

static struct platform_device pwm_backlight = {
  .name               = "pwm-backlight",
  .id                 = 0,
  .dev                = { .platform_data  = &pwm_backlight_data }
};


static struct platform_device *fc6100_devices[] __initdata = {
	&pwm_backlight,
	&pwm_device,
};

static int fc6100_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(FC_LCD_RST_N, 1);

	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW03) {
		/* PROTO HW03 and C and D */
		gpio_set_value(FC_LCD_BKL_EN, 1);
		printk(KERN_ERR "%s\n", __func__);
	}
	else if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW02) {
		/* PROTO HW02 */
		gpio_set_value(FC_LCD_BKL_EN_HW02, 1);
	}

	return 0;
}


static void fc6100_panel_disable_lcd(struct omap_dss_device *dssdev)
{
       gpio_set_value(FC_LCD_RST_N, 0);

       if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW03) {
               /* PROTO HW03 and C and D */
               gpio_set_value(FC_LCD_BKL_EN, 0);
       }
       else if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW02) {
               /* PROTO HW02 */
               gpio_set_value(FC_LCD_BKL_EN_HW02, 0);
       }

}


static struct omap_dss_device fc6100_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "tpo_laj07t001a_panel",
	//.driver_name = "toshiba_lte_panel",
	//.driver_name = "hannstar_lte_panel",
	.phy.dpi.data_lines = 24,
	.platform_enable = fc6100_panel_enable_lcd,
	.platform_disable = fc6100_panel_disable_lcd,
	.panel.width_in_mm = 108,
	.panel.height_in_mm = 65,
	.channel = OMAP_DSS_CHANNEL_LCD,
};

static void __init omap_fc6100_devwb_display_init(void)
{
	// LCD RST, BKL_EN, BKL_PWM
	parrot_gpio_out_init(FC_LCD_RST_N, 0);

	// PROTO HW02 and followings
	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW03) {
		parrot_gpio_out_init(FC_LCD_BKL_EN, 0);
	}
	else{
		parrot_gpio_out_init(FC_LCD_BKL_EN_HW02, 0);
	}

	fc6100_mod_display_init(&fc6100_lcd_device);
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
#if defined(CONFIG_KSZ8851SNL) || defined(CONFIG_KSZ8851SNL_MODULE)
	[1] = {
		.modalias		= "ksz8851snl",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 30000000,
		.controller_data 	= &fc6100_mcspi_config,
		.mode			= SPI_MODE_3,
	}
#elif defined(CONFIG_SERIAL_MAX3100) || defined(CONFIG_SERIAL_MAX3100_MODULE)
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


static struct i2c_board_info __initdata fc6100_i2c_bus3_info[] = {
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.flags = I2C_CLIENT_WAKE,
	},
#ifdef MCLAREN_TEST
	{
		I2C_BOARD_INFO("mclaren_x86", 0x66),
	},
#endif
};

static void __init omap_fc6100_devwb_i2c_init(void)
{
	/* Ipod chip and Nuvoton chip NAU8820 are managed by default */
	omap_register_i2c_bus(2, I2C_IPOD_CHIP_MAX_FREQ, NULL, NULL, 0);

#ifdef MCLAREN_TEST
	parrot_gpio_in_init(FC_IT_TOUCHSCREEN_N, OMAP_PIN_INPUT_PULLUP);
	fc6100_i2c_bus3_info[1].irq = gpio_to_irq(FC_IT_TOUCHSCREEN_N);
#else
	tsc2007_init(FC_IT_TOUCHSCREEN_N, &fc6100_i2c_bus3_info[0]);
#endif
	omap_register_i2c_bus(3, 100, NULL, fc6100_i2c_bus3_info,
		ARRAY_SIZE(fc6100_i2c_bus3_info));
}


#define ETH_MAX3100_SPI_DBG_RST_N_GPIO58_HW02	(58) /* PROTO HW02 */

#if defined(CONFIG_KSZ8851SNL) || defined(CONFIG_KSZ8851SNL_MODULE)
# define ETH_MAX3100_SPI_DBG_IT_GPIO (54)
# define ETH_MAX3100_TEXT "ksz8851snl irq"
#elif defined(CONFIG_SERIAL_MAX3100) || defined(CONFIG_SERIAL_MAX3100_MODULE)
# define ETH_MAX3100_SPI_DBG_IT_GPIO (186)
# define ETH_MAX3100_TEXT "max3100 irq"
#endif

#if defined(CONFIG_KSZ8851SNL) || defined(CONFIG_KSZ8851SNL_MODULE) || defined(CONFIG_SERIAL_MAX3100) || defined(CONFIG_SERIAL_MAX3100_MODULE)
static void __init eth_max3100_init(void)
{
	/* PROTO HW02 and followings */

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

#define TMS_IT_N			FC_IT_HOST_N	/* see IT_HOST_N or TMS_ITn on schematic */
/*
 * Miscellaneous (board specific)
 * Generic GPIO 00/01/02 Outside the module 
 */
static void __init omap_fc6100_devwb_miscellaneous( void )
{

	/* PROTO HW02 and followings */

	/* Init GPIO 0, 1, 2 of the WB (for test purpose) as output */
	parrot_gpio_user_out_init(FC_GPIO_0_PWM, 0, NULL);

	parrot_gpio_user_out_init(FC_GPIO_1_PWM, 0, NULL);

	parrot_gpio_user_out_init(FC_GPIO_2_CLK, 0, NULL);
#if !defined(CONFIG_KSZ8851SNL) && !defined(CONFIG_KSZ8851SNL_MODULE) && !defined(CONFIG_SERIAL_MAX3100) && !defined(CONFIG_SERIAL_MAX3100_MODULE)
	/* Used by RaggaSpi Userspace driver */
	parrot_gpio_user_in_init(TMS_IT_N,OMAP_PIN_OFF_WAKEUPENABLE,"TMS_IT_N");

	/* Used by RaggaSpi to manage manually CS0 */
	omap_mux_init_signal("mcspi1_cs0.gpio_174", OMAP_PIN_INPUT);
	parrot_gpio_user_out_init(174,1,"mcspi1_cs0");
#endif
}

#define WB_FC6100_CONFIG	FC6100_USE_SPI_ALL		| \
				FC6100_USE_ALL_MMC		| \
				FC6100_USE_ALL_MCBSP		| \
				FC6100_USE_UART1_RTS_CTS	| \
				FC6100_USE_ALL_I2C		| \
				FC6100_USE_LCD_HSYNC_VSYNC

static void __init omap_fc6100_init(void)
{
	fc6100_gpio.dev.platform_data = fc6100_devwb_pins;

	fc6100_mod_common_init(WB_FC6100_CONFIG);

	/* PWM for backligth (see LCD_BKL_PWM) */
	omap_mux_init_signal("gpmc_ncs4.gpt9_pwm_evt", OMAP_PIN_OUTPUT);

	/* I2c */
	omap_fc6100_devwb_i2c_init();

	/* Display */
	omap_fc6100_devwb_display_init();

	/* Miscellaneaous */
	omap_fc6100_devwb_miscellaneous();

#if defined(CONFIG_KSZ8851SNL) || defined(CONFIG_KSZ8851SNL_MODULE) || defined(CONFIG_SERIAL_MAX3100) || defined(CONFIG_SERIAL_MAX3100_MODULE)
	eth_max3100_init();
#endif
	spi_register_board_info(fc6100_spi_board_info, ARRAY_SIZE(fc6100_spi_board_info));

	platform_add_devices(fc6100_devices, ARRAY_SIZE(fc6100_devices));
}


MACHINE_START(OMAP_FC6100, "fc6100 board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= parrot_omap_map_io,
	.init_irq	= fc6100_mod_init_irq,
#ifdef CONFIG_MACH_OMAP_FACTORY
	.init_machine	= omap_fc6100_factory_init,
#else
	.init_machine	= omap_fc6100_init,
#endif
	.timer		= &omap_timer,
MACHINE_END
