/*
 * linux/arch/arm/mach-omap2/board-rnb5.c
 *
 * Copyright (C) 2011 Parrot SA
 * Florent Bayendrian <florent.bayendrian@parrot.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/timer-gp.h>
#include <plat/mux.h>
#include <plat/pwm.h>
#include <plat/vrfb.h>

#include "prm.h"
#include "prm-regbits-34xx.h"

#include <mach/board-fc6100.h>
#include <asm/mach-types.h>
#include <linux/reboot.h>

#include <../drivers/parrot/input/touchscreen/atmel_mxt_ts.h>
#include "parrot-common.h"
#include <linux/mpu.h>

#include "hsmmc.h"


/* output */
#define TVP_PDN			FC_IT_HOST_N
#define GPIO_TCT_RESET		FC_SDIO_1_D0
#define GPIO_PRDN_CODEC		FC_SDIO_1_D1
#define GPIO_MUTE		FC_SDIO_1_D2
#define GPIO_SWITCH_MICRO_HW1	FC_SDIO_1_D3 /* only hw01*/
#define GPIO_STDBY_AMPLI_HW2	FC_SDIO_1_D3 /* only hw02+ */
#define GPIO_EN_PWR		FC_SDIO_1_CLKIN
#define GPIO_RESET_HUB_HW1		FC_SDIO_1_CLK /* only hw01*/
#define GPIO_TFT_LS_EN		FC_LCD_BKL_EN
#define GPIO_RST_LCDn		FC_LCD_RST_N
#define GPIO_N_IRQ_RF		FC_SPI_0_MOSI
#define GPIO_EN_3V3_FC		FC_UART_0_CTS

/* input */
#define GPIO_INT_TS		FC_IT_TOUCHSCREEN_N
#define GPIO_PARK_BRAKE_HW1		FC_SDIO_1_CMD /* only hw01-02 */
#define GPIO_EN_LS_PS_HW3      FC_SDIO_1_CMD /* only hw03+ */
#define GPIO_ACC_IN		FC_SDIO_1_CD_N
#define GPIO_R_GEAR		FC_SDIO_1_WP_N
//XXX SP_IN gpio 131
#define GPIO_ON_OFF_FC		FC_SPI_0_CLK
#define GPIO_I1_DETECT		FC_SPI_0_MISO
#define RNB5_GPIO_2_CLK_HW1		FC_GPIO_2_CLK /* only hw01 */
#define GPIO_N_IRQ_RVS		FC_UART_0_RTS

/* hw 02 ...*/
#define GPIO_RESET_TS_HW2		FC_SPI_1_CLK
#define GPIO_ACTIVE_ANTENNA_HW2	FC_SPI_1_MISO

/* hw 03 ...*/
#define GPIO_PARK_BRAKE_HW3        FC_SPI_1_MOSI
#define GPIO_UNDERVOLTAGE_HW3  FC_SPI_0_CS0_N
#define GPIO_TILT_HW3          FC_SPI_1_CS0_N
#define GPIO_ILLUMINATION_HW3  USB_1_OC_N
#define GPIO_I1_PWM            FC_GPIO_0_PWM


static int rnb5_ver = 0;

static int rnb5_get_version(void)
{
	switch (fc6100_mod_get_motherboard_revision()) {
		case 0x02:
			return 1;
			break;
		case 0x03:
			return 2;
			break;
		case 0x04:
			return 3;
			break;
		case 0x05:
			return 4;
			break;
		case 0x06:
			return 5;
			break;
		case 0x07:
			return 6;
			break;
	}
	BUG_ON(1);
	return 0;
}

int rnb5_pins[] = {
	/* input */
	GPIO_PARK_BRAKE_HW1,//park-brake hw03 change for hardware version
	171,//on-off
	173,//secure
	15,//ac-key
	157,//rev-gear
	174,//undervolt hw03
	91,//tilt hw03
	46,//illumination hw03
	/* output */
	23,//BT
	111,//ipod
	//XXX for rnb5tb
	135,//standby ampli
	134,//codec mute
	133,//powerdown codec
	132,//codec, ampli, tuner reset
	-1,
};

static u8 mxt224e_read_chg(void)
{
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
	0xFF, 0xFF, 0x32, 0x18, 0x00, 0x01, 0x0A, 0x00,
	0x00, 0x01, 0x00, 0x0A, 0x05, 0x8F, 0x00, 0x00,
	0x13, 0x0B, 0x00, 0x00, 0x1C, 0x01, 0x06, 0x00,
	0x01, 0x01, 0x20, 0x0A, 0x0A, 0x0A, 0x01, 0x20,
	0x03, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x40,
	0x00, 0x00, 0x00, 0x14, 0x0A, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0x00, 0x40, 0x1F, 0x94, 0x11, 0x00, 0x32,
	0x1E, 0x1E, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x03,
	0x20, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x14, 0x46, 0x32, 0x00, 0x00, 0x00, 0xBE, 0x00,
	0x00, 0x1F, 0x84, 0x42, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20,
	0x00, 0x00, 0x30, 0x08, 0x10, 0x00, 0x00, 0x14,
	0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x28, 0x01, 0x01, 0x01,
	0x30, 0x0A, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00,
	0x40, 0x00, 0x00, 0x00, 0x14, 0x0A, 0x01, 0x00,
	0x00, 0x00, 0x00
};

static struct mxt_platform_data  mxt224e_data = {
	.config_1 = &mxt224e_config_1aa[0],
	.config_1_length = sizeof(mxt224e_config_1aa),
	.config_1_crc = 0xD0AF83,
	.config_2 = &mxt224e_config_2ab[0],
	.config_2_length = sizeof(mxt224e_config_2ab),
	.config_2_crc = 0x20715D,
	.orient = MXT_HORIZONTAL_FLIP,
	.irqflags = IRQF_TRIGGER_FALLING,
	.read_chg = mxt224e_read_chg,
	.x_size = 800,
	.y_size = 480,
	//.x_line = Hook in config
	//.y_line = Hook in config
	//.threshold = Hook in config
};

/*
 * Touchscreen controller
 */
static struct i2c_board_info __initdata rnb5_i2c_bus3_info[] = {
	[0] = {
		I2C_BOARD_INFO("tnx_mxt_ts", 0x4a),
		.platform_data = &mxt224e_data,
		/*.irq = 0, set by mxt224e_init */
	},
};

static struct omap2_pwm_platform_config pwm_config = {
  .timer_id           = 9,   // GPT9_PWM_EVT LCD_BKL_PWM
  .polarity           = 1     // Active-high
};

static struct omap2_pwm_platform_config button_pwm_config = {
  .timer_id           = 10,   // GPT10_PWM_EVT FC_GPIO_0_PWM
  .polarity           = 1     // Active-high
};


/*
 * Display settings
 */

static int fc6100_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (rnb5_ver >= 3)
		parrot_gpio_out_init(GPIO_EN_LS_PS_HW3, 1);     // /!\ MUST be after display init and EN_PWR

	return 0;
}

static void fc6100_panel_disable_lcd(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device rnb5_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "hannstar_lte_panel",
	.phy.dpi.data_lines = 24,
	.panel.width_in_mm = 137,
	.panel.height_in_mm = 77,
	.platform_enable = fc6100_panel_enable_lcd,
	.platform_disable = fc6100_panel_disable_lcd,
	.channel = OMAP_DSS_CHANNEL_LCD,
};

static struct platform_device pwm_device = {
  .name               = "omap-pwm",
  .id                 = 0,
  .dev                = { .platform_data  = &pwm_config }
};
static struct platform_device pwm_button_device = {
  .name               = "omap-pwm",
  .id                 = 1,
  .dev                = { .platform_data  = &button_pwm_config }
};

static int rnb5_pwm_lcd_notify(struct device *d, int b)
{
	int ret;
	/* on hw01 backlight is inverted */
	if(rnb5_ver <= 1)
		ret = 1000 - b;
	else
		ret = b;
	return ret;
}

static struct platform_pwm_backlight_data pwm_backlight_data = {
	.pwm_id         = 0,
	.max_brightness = 1000,
	.dft_brightness = 400,
	.dft_power      = 1,
	.pwm_period_ns  = 40000,
	.notify = rnb5_pwm_lcd_notify,
};

static struct platform_pwm_backlight_data pwm_button_data = {
	.pwm_id         = 1,
	.max_brightness = 1000,
	.dft_brightness = 1000,
	.dft_power      = 0,
	.pwm_period_ns  = 4000000,
};

static struct platform_device pwm_backlight = {
  .name               = "pwm-backlight",
  .id                 = 0,
  .dev                = { .platform_data  = &pwm_backlight_data }
};
static struct platform_device pwm_button = {
  .name               = "pwm-backlight",
  .id                 = 1,
  .dev                = { .platform_data  = &pwm_button_data }
};

static struct platform_device *rnb5_devices[] __initdata = {
	&pwm_backlight,
	&pwm_button,
	&pwm_device,
	&pwm_button_device,
};
/*
 * Miscellaneous (board specific)
 */

static void omap_rnb5_shutdown_sequence(void)
{
	if (rnb5_ver >= 3) {
		gpio_set_value(GPIO_EN_LS_PS_HW3, 0);
		mdelay(40);
	}
	gpio_set_value(GPIO_EN_PWR, 0);
	mdelay(40);
	gpio_set_value(GPIO_EN_3V3_FC, 0);
}

static void omap_rnb5_power_off(void)
{
	omap_rnb5_shutdown_sequence();
	local_irq_disable();
	/* reboot if we are still alive after 1 s */
	mdelay(1000);
	machine_restart(NULL);
}

static void __init omap_rnb5_miscellaneous_init( void )
{
	// user Inputs
	parrot_gpio_user_in_init(GPIO_ON_OFF_FC, 0, "on-off");
	parrot_gpio_user_in_init(GPIO_I1_DETECT, OMAP_PIN_INPUT_PULLUP, "secure");
	parrot_gpio_user_in_init(GPIO_ACC_IN, 0, "ac-key"); // 12v key ( 0 == 12v key on)
	parrot_gpio_user_in_init(GPIO_R_GEAR, 0, "rev-gear"); // reverse gear ( 0 == reverse gear on )

	if(rnb5_ver <= 1) {
		parrot_gpio_user_in_init(RNB5_GPIO_2_CLK_HW1, 0, NULL);
	}
	else if (rnb5_ver <= 2) {
		parrot_gpio_user_in_init(GPIO_PARK_BRAKE_HW1, 0, "park-brake"); // parking brake input ( 0 == parking brake on)
	}
	else {
		rnb5_pins[0] = GPIO_PARK_BRAKE_HW3;
		parrot_gpio_user_in_init(GPIO_PARK_BRAKE_HW3, 0, "park-brake"); // parking brake input ( 0 == parking brake on)
		parrot_gpio_user_in_init(GPIO_UNDERVOLTAGE_HW3, 0, "undervolt");   // Undervoltage detection (not yet managed)
		parrot_gpio_user_in_init(GPIO_TILT_HW3, 0, "tilt");    //for test purpose
		parrot_gpio_user_in_init(GPIO_ILLUMINATION_HW3, 0, "illumination");    //for test purpose
	}
}

void omap_rnb5_i2c_init(void)
{
	//Limitated to 50kHz due to chips Ipod
	omap_register_i2c_bus(2, 50, NULL, NULL, 0);

	if(rnb5_ver <= 1)
		/* reset is shared with touchscreen */
		mxt224e_init(GPIO_INT_TS, -1, &rnb5_i2c_bus3_info[0], 0);
	else
		mxt224e_init(GPIO_INT_TS, GPIO_RESET_TS_HW2, &rnb5_i2c_bus3_info[0], 1);

	omap_register_i2c_bus(3, 100, NULL, rnb5_i2c_bus3_info, ARRAY_SIZE(rnb5_i2c_bus3_info));
}

#define WB_RNB5_CONFIG  FC6100_USE_MMC1			| \
			FC6100_USE_ALL_MCBSP		| \
			FC6100_USE_COMPOSITE_TV		| \
			FC6100_USE_ALL_I2C		| \
			FC6100_USE_LCD_HSYNC_VSYNC


static void __init omap_rnb5_init(void)
{
	char test_12v_key = 0;

	parrot_gpio_out_init(GPIO_EN_PWR, 1); // power supply enable (USB PHY, LCD, etc..)

	//Change MMC settings before calling common init
	mmc2_settings.max_freq = 24000000;
	fc6100_gpio.dev.platform_data = rnb5_pins;

	//FC6100 Common init
	fc6100_mod_common_init(WB_RNB5_CONFIG);

	// Init pwm for backlight : LCD_BKL_PWM
	// Need to be here to avoid screen flash on boot
	omap_mux_init_signal("gpmc_ncs4.gpt9_pwm_evt", OMAP_PIN_OUTPUT);

	// enable fc6100
	parrot_gpio_out_init(GPIO_EN_3V3_FC, 1);

	/* reset shared with toucscreen */
	parrot_gpio_out_init(GPIO_RST_LCDn, 1);
	mdelay(10);
	gpio_set_value(GPIO_RST_LCDn, 0);

	parrot_gpio_out_init(GPIO_MUTE, 1);			// codec mute
	parrot_gpio_out_init(GPIO_TCT_RESET, 0);		// RST tuner/codec/amplifier, active low
	parrot_gpio_out_init(GPIO_PRDN_CODEC, 1);		// Power down codec (active low)

	/* on hw01 GPIO_SWITCH_MICRO_HW1 : Switch micro ( 0=external micro / 1=internal ) */
	parrot_gpio_out_init(GPIO_STDBY_AMPLI_HW2, 0); 		// Standby ampli (active = 0)

	// /!\ warning system_rev is only initialized after common_init
	rnb5_ver = rnb5_get_version();
	printk(KERN_INFO "RnB5 PCB revision : %d\n", rnb5_ver);

	omap_rnb5_i2c_init();

	parrot_gpio_user_in_init(GPIO_N_IRQ_RVS, OMAP_PIN_OFF_WAKEUPENABLE, "video-in-irq");

	//Display
	rnb5_lcd_device.panel.width_in_mm = rnb5_lcd_device.panel.width_in_mm * (800 - sgx_reduce_pixels_left - sgx_reduce_pixels_right) / 800;
	rnb5_lcd_device.panel.height_in_mm = rnb5_lcd_device.panel.height_in_mm * (480 - sgx_reduce_pixels_down) / 480;
	fc6100_mod_display_init(&rnb5_lcd_device);

	omap_rnb5_miscellaneous_init();

	if(__raw_readl(OMAP3430_PRM_RSTST) & OMAP3430_MPU_WD_RST_MASK)
	{
		printk(KERN_ERR "RnB5 resetted by watchdog !\n");
		test_12v_key = 1;
	}

	if(__raw_readl(OMAP2_L4_IO_ADDRESS(OMAP343X_SCRATCHPAD + 4)) == 0x424D0000) // 'B' 'M' 0 0 written by omap_prcm_arch_reset
	{
		printk(KERN_INFO "RnB5 was resetted\n");
		test_12v_key = 1;
	}

	if(test_12v_key && gpio_get_value(GPIO_ACC_IN)) // /!\ MUST be after omap_rnb5_miscellaneous_init
	{
		omap_rnb5_shutdown_sequence();
		mdelay(10000); // Wait at least 10 seconds since hardware is armed for this amount of time
		machine_restart(NULL);
	}

	if (rnb5_ver >= 2)
		parrot_gpio_out_init(GPIO_ACTIVE_ANTENNA_HW2, 0); 		// Active antenna, not used, 0 for now
	else
		parrot_gpio_out_init(GPIO_RESET_HUB_HW1, 1);		// RST HUB

	// Init pwm for led
	omap_mux_init_signal("gpmc_ncs5.gpt10_pwm_evt", OMAP_PIN_OUTPUT);
	// Init pwm for subwoofer
	omap_mux_init_signal("gpmc_ncs6.gpt11_pwm_evt", OMAP_PIN_OUTPUT);

	platform_add_devices(rnb5_devices, ARRAY_SIZE(rnb5_devices));

	pm_power_off = omap_rnb5_power_off;
}

MACHINE_START(OMAP_RNB5, "rnb5 board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= parrot_omap_map_io,
	.init_irq	= fc6100_mod_init_irq,
	.init_machine	= omap_rnb5_init,
	.timer		= &omap_timer,
MACHINE_END
