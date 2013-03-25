#include <plat/mux.h>
#include <mach/board-fc6100.h>
#include "parrot-common.h"


#define DSS_HSYNC			(LCD_HS)
#define DSS_VSYNC			(LCD_VS)

/*
 * Miscellaneous (board specific)
 */
static void __init omap_fc6100_factory_miscellaneous( void )
{
	int loop;

	/* Serial init */
	parrot_gpio_user_out_init(FC_UART_0_RTS, 0, NULL);
	parrot_gpio_user_out_init(FC_UART_0_CTS, 0, NULL);

	/* I2C */
	parrot_gpio_user_out_init(FC_I2C_0_SCL, 0, NULL);
	parrot_gpio_user_out_init(FC_I2C_0_SDA, 0, NULL);

	parrot_gpio_user_out_init(FC_IT_TOUCHSCREEN_N, 0, NULL);
	omap_register_i2c_bus(2, I2C_IPOD_CHIP_MAX_FREQ, NULL, NULL, 0);


	/* Display */
	for (loop = FC_DSS_PCLK; loop <= FC_DSS_DATA17 ; loop++) {
		parrot_gpio_user_out_init(loop, 0, NULL);
	}

	for (loop = FC_DSS_DATA18; loop <= FC_DSS_DATA23; loop++) {
		if ( loop != 4 ) {
			parrot_gpio_user_out_init(loop, 0, NULL);
		}
	}

	/* disable hsync & vsync */
	parrot_gpio_user_out_init(FC_LCD_HS,0, NULL);
	parrot_gpio_user_out_init(FC_LCD_VS,0, NULL);

	/* export additionnal I/O in userland for test purpose */
	parrot_gpio_user_out_init(FC_LCD_RST_N, 0, NULL);
	parrot_gpio_user_out_init(FC_LCD_BKL_PWM, 0, NULL);

	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW03) {
		parrot_gpio_user_out_init(FC_LCD_BKL_EN, 0, NULL);
	}
	else{
		parrot_gpio_user_out_init(FC_LCD_BKL_EN_HW02, 0, NULL);
	}

	/* SPI */
	for (loop = FC_SPI_0_CLK; loop <= FC_SPI_0_CS0_N; loop++) {
		parrot_gpio_user_out_init(loop, 0, NULL);
	}

	for (loop = FC_SPI_1_CLK; loop <= FC_SPI_1_CS0_N; loop++) {
		parrot_gpio_user_out_init(loop, 0, NULL);
	}
	if (fc6100_mod_get_pcb_revision() == FC6100_0_HW05)
		parrot_gpio_user_out_init(FC_SPI_1_CS1_N_HW05, 0, NULL);

	/* Audio */
	parrot_gpio_user_out_init(FC_I2S_IN1, 0, NULL);
	parrot_gpio_user_out_init(FC_I2S_OUT1, 0, NULL);
	parrot_gpio_user_out_init(FC_I2S_OUT2, 0, NULL);

	/* Camera */
	parrot_gpio_user_out_init(FC_CAM_EN_N, 1, NULL);

	for (loop = FC_CAM_HS; loop <= FC_CAM_D9; loop++) {
		/* FC_CAM_FLD is only on hw06 */
		if (loop == FC_CAM_FLD && fc6100_mod_get_pcb_revision() < FC6100_0_HW06)
			continue;

		parrot_gpio_user_out_init(loop, 0, NULL);
	}

	/* USB */
	if(fc6100_mod_get_pcb_revision() >= FC6100_0_HW03) {
		/* PROTO HW03 and following */
		gpio_export(USB_0_ID, 1);
		gpio_export(USB_1_ID, 1);
	}

	gpio_export(ULPI_0_RST_N, 1);
	gpio_export(ULPI_1_RST_N, 1);

	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW06)
		gpio_export(USB_0_OC_N, 1);
	else
		gpio_export(USB_0_OC_N_HW02, 1);

	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW05){
		gpio_export(USB_1_OC_N, 1);
	}else{
		//Gpio 126 for over current
		gpio_export(USB_1_OC_N_HW04, 1);
	}

	/* May be used in the future to communicate with host */
	parrot_gpio_user_in_init(FC_IT_HOST_N, 0, NULL);

	/* PROTO HW02 and followings */

	/* Init GPIO 0, 1, 2 of the WB (for test purpose) as output */
	parrot_gpio_user_out_init(FC_GPIO_0_PWM, 0, NULL);
	parrot_gpio_user_out_init(FC_GPIO_1_PWM, 0, NULL);
	parrot_gpio_user_out_init(FC_GPIO_2_CLK, 0, NULL);
}

#define WB_FACTORY_CONFIG	FC6100_USE_ALL_MMC		| \
				FC6100_USE_UART1_RTS_CTS	| \
				FC6100_USE_TVOUT_ONLY

void __init omap_fc6100_factory_init(void)
{
	fc6100_mod_common_init(WB_FACTORY_CONFIG);

	fc6100_mod_display_init(NULL);

	omap_fc6100_factory_miscellaneous();
}
