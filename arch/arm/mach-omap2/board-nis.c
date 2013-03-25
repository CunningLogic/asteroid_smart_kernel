

#include <linux/kernel.h>
#include <linux/init.h>
#include <plat/mux.h>
#include <mach/board-fc6100.h>

#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif

#include "parrot-common.h"
#include "board-fc6100.h"
#include "omap3-opp.h"
#include "smartreflex-class3.h"
#include "hsmmc.h"

#include <linux/mmc/host.h>
#include <linux/i2c/rtc-pca8565.h>
#include "board-fc6100-camera.h"
#include <../drivers/parrot/input/touchscreen/atmel_mxt_ts.h>

#define GPS_EN			FC_UART_0_RTS	/* see GPS EN on Schematics */
#define GPS_REFLASH		FC_UART_0_CTS	/* see GPS_REFLASH on Schematics */
#define USB_0_CHEN		FC_GPIO_2_CLK	/* USB 0 Charge Enable, see USB_0_CHEN on Schematics */
#define USB_1_CHEN		FC_GPIO_1_PWM	/* USB 1 Charge Enable, see USB_1_CHEN on Schematics */
#define TMS_IT_N		FC_IT_HOST_N	/* see IT_HOST_N on schematic */
#define USB_HUB_RESET_N		FC_LCD_BKL_EN	/* see USB_HUB_RESETn on schematic */

/*	FC6100 Module - unused pin
*
*	Set to 3v3 :
*		FC_SDIO_1_CMD, FC_SDIO_1_D0, FC_SDIO_1_D1, FC_SDIO_1_D2, FC_SDIO_1_D3
*		FC_SDIO_1_CD_N, FC_SDIO_1_WP_N, FC_SDIO_0_WP_N
*
*
*	Set to 1v8 :
*		SPI_1_CS0_N, FC_IT_TOUCHSCREEN_N
*
*
*	Set to ground :
*		FC_LCD_BKL_PWM, TV_CHROMA, TV_LUMA
*		FC_SDIO_1_CLKIN, SPI_1_CLK, SPI_1_MOSI, SPI_1_MISO
*
*/

int nis_pins[] = {
	/* Output */
	RF_RST_N,		//BT reset
	FC_IPOD_RESET_N,	//Ipod
	USB_1_CHEN,		//USB 1 Charge Enable
	USB_0_CHEN,		//USB 0 Charge Enable
	GPS_REFLASH,		//GPS boot
	GPS_EN,			//GPS enable

	/* Input */
	FC_SDIO_0_CD_N,		//Sdio 0 card detect
	FC_SPI_0_CS0_N,		//TMS : spi chip select
	TMS_IT_N,		//TMS IT
	USB_HUB_RESET_N,	//USB HUB USB82512 reset
	-1,
};

static const u8 mxt224e_config[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xFF, 0xFF, 0x32, 0x08, 0x05, 0x14, 0x14, 0x00,
  0x00, 0x0A, 0x0F, 0x00, 0x00, 0x83, 0x00, 0x01,
  0x11, 0x0C, 0x00, 0x11, 0x37, 0x02, 0x02, 0x00,
  0x01, 0x01, 0x00, 0x05, 0x0A, 0x0A, 0x0A, 0x00,
  0x00, 0x00, 0x00, 0x19, 0x19, 0x2D, 0x3C, 0xC0,
  0x64, 0xC0, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00,
  0x11, 0x01, 0x00, 0x01, 0x1E, 0x02, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,
  0x00, 0x1A, 0x01, 0x01, 0x01, 0x00, 0x07, 0x00,
  0x00, 0x19, 0x00, 0xE7, 0xFF, 0x04, 0x64, 0x00,
  0x00, 0x0A, 0x0F, 0x14, 0x19, 0x1E, 0x04, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A,
  0xFF, 0x03, 0x00, 0x64, 0x64, 0x01, 0x0A, 0x14,
  0x28, 0x4B, 0x00, 0x02, 0x00, 0x64, 0x00, 0x19,
  0x00, 0x00, 0x00, 0xE0, 0x2E, 0x58, 0x1B, 0xB0,
  0x36, 0xF4, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x01, 0x04, 0x08, 0x00
};

static struct mxt_platform_data mxt224e_data = {
       .config_1 = &mxt224e_config[0],
       .config_1_length = sizeof(mxt224e_config),
       .config_1_crc = 0x720E94,
       .config_2 = &mxt224e_config[0],
       .config_2_length = sizeof(mxt224e_config),
       .config_2_crc = 0x720E94,
       .orient = MXT_HORIZONTAL_FLIP,
       .irqflags = IRQF_TRIGGER_FALLING,
       //.read_chg = mxt224e_read_chg
};
static struct i2c_board_info __initdata fc6100_ais_i2c_bus2_info[] = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", 0x4b),
		.platform_data = &mxt224e_data
	},
};

static void __init omap_nis_devwb_i2c_init(void)
{
	/* Ipod chip and Nuvoton chip NAU8820 are managed by default */
	mxt224e_init(-1, -1, &fc6100_ais_i2c_bus2_info[0], 1);
	omap_register_i2c_bus(2, I2C_IPOD_CHIP_MAX_FREQ, NULL, fc6100_ais_i2c_bus2_info,
		ARRAY_SIZE(fc6100_ais_i2c_bus2_info));

	/* TODO check requiremets on frequency 
	 * Video ADC : TVP5151 (@ 0x5B)
	 * LVDS Serializer : DS90UR905Q (@ 0x6B) */
	omap_register_i2c_bus(3, 100, NULL, NULL, 0);
}


static struct spi_board_info fc6100_nis_board_info[] __initdata = {
	/* TMS470 : communication link */
	[0] = {
		.modalias		= "spidev",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 2400000,
		.controller_data 	= &fc6100_mcspi_config,
		.mode			= SPI_MODE_3,
		.irq			= OMAP_GPIO_IRQ(TMS_IT_N),
	},
};

static int nis_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(FC_LCD_RST_N, 1);
	return 0;
}


static void nis_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(FC_LCD_RST_N, 0);
}

static struct omap_dss_device nis_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "tpo_laj07t001a_panel",
	.phy.dpi.data_lines = 24,
	.platform_enable = nis_panel_enable_lcd,
	.platform_disable = nis_panel_disable_lcd,
	.panel.width_in_mm = 108,
	.panel.height_in_mm = 65,
	.channel = OMAP_DSS_CHANNEL_LCD,
};

/*
 * Display
 */
static void __init omap_nis_devwb_display_init(void)
{
	// LCD RST 
	parrot_gpio_out_init(FC_LCD_RST_N, 0);
	fc6100_mod_display_init(&nis_lcd_device);
}


/*
 * Miscellaneous (board specific)
 */
static void __init omap_nis_devwb_miscellaneous( void )
{

	/* TODO IT_HOST_1V8_N used for SPI */
	/* Used by RaggaSpi Userspace driver (FC6100 <->TMS470 SPI interface) */
	parrot_gpio_user_in_init(TMS_IT_N,OMAP_PIN_OFF_WAKEUPENABLE,"TMS_IT_N");
 
	/* Used by RaggaSpi to manually manage CS0 */
	parrot_gpio_user_out_init(FC_SPI_0_CS0_N,1,"mcspi1_cs0");

	spi_register_board_info(fc6100_nis_board_info, ARRAY_SIZE(fc6100_nis_board_info));
 
	/*  Set GPS_REFLASH
	 * Force GPS_REFLASH to one at startup to enter into
	 * firmware upload mode through UART.
	 * NOT use on the NIS board
	 */
	parrot_gpio_out_init(GPS_REFLASH, 1);

	/*  Set GPS_EN */
	parrot_gpio_out_init(GPS_EN, 0);
	mdelay(1);
	gpio_set_value(GPS_EN, 1);

	/* USB charge enable */
	parrot_gpio_out_init(USB_0_CHEN, 1);
	parrot_gpio_out_init(USB_0_CHEN, 1);


	/* USB HUB USB82512 reset */
	parrot_gpio_out_init(USB_HUB_RESET_N, 0);
	mdelay(20);
	gpio_set_value(USB_HUB_RESET_N, 1);

}

#define WB_NIS_CONFIG		FC6100_USE_SPI1_CS0		| \
				FC6100_USE_MMC1			| \
				FC6100_DISABLE_MMC1_WP		| \
				FC6100_DISABLE_MMC2_WP_CD	| \
				FC6100_USE_ALL_MCBSP		| \
				FC6100_USE_ALL_I2C		| \
				FC6100_USE_LCD_HSYNC_VSYNC


static void __init omap_nis_init(void)
{
	fc6100_mod_common_init(WB_NIS_CONFIG);
	fc6100_gpio.dev.platform_data = nis_pins;

	if (fc6100_mod_get_pcb_revision() < FC6100_0_HW07) {
		panic("this module is not supported on NIS");
	}

	/* I2c */
	omap_nis_devwb_i2c_init();

	/* Display */
	omap_nis_devwb_display_init();

	/* Miscellaneaous */
	omap_nis_devwb_miscellaneous();
}


MACHINE_START(OMAP_NIS, "nis board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= parrot_omap_map_io,
	.init_irq	= fc6100_mod_init_irq,
	.init_machine	= omap_nis_init,
	.timer		= &omap_timer,
MACHINE_END
