#ifndef __ASM_ARCH_OMAP_FC6100_H
#define __ASM_ARCH_OMAP_FC6100_H


#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/dma-mapping.h>

#include <linux/switch.h>
#include <plat/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/gpio.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <plat/timer-gp.h>
#include <plat/mmc.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <plat/control.h>
#include <plat/usb.h>
#include <plat/display.h>


#define	FC6100_PCB_VERSION_MASK		(0x0f)		/* Mask for pcb version */
#define	FC6100_BOARD_VERSION_MASK	(0x7f << 10)	/* Mask for the board version/ID */
#define	FC6100_0_HW00			0
#define FC6100_0_HW01			FC6100_0_HW00
#define	FC6100_0_HW02			(1<<1)  /* FC6100 PROTO B */
#define	FC6100_0_HW03			(0x3)   /* FC6100 PROTO B'*/
#define	FC6100_0_HW04			(0x4)   /* FC6100 PROTO C */
#define	FC6100_0_HW05			(0x5)   /* FC6100 PROTO D */
#define	FC6100_0_HW06			(0x6)   /* FC6100 PROTO E */
#define	FC6100_0_HW07			(0x7)   /* FC6100 PROTO F */

#define fc6100_board_rev()			(board_rev)
#define fc6100_mod_get_pcb_revision()		(board_rev & FC6100_PCB_VERSION_MASK)
#define fc6100_mod_get_motherboard_revision()	((board_rev & FC6100_BOARD_VERSION_MASK) >> 10)
#define machine_is_fc6100_module()		(machine_is_omap_fc6100() \
							|| machine_is_omap_rnb5() \
							|| machine_is_omap_volvo() \
							|| machine_is_omap_nis() \
							|| machine_is_omap_ais() \
							|| machine_is_omap_hsae() )

#define I2C_IPOD_CHIP_MAX_FREQ			(10)

//Board Settings
#define FC6100_USE_SPI1_CS0			(1)
#define FC6100_USE_SPI3_CS0			(1<<1)
#define FC6100_USE_SPI3_CS1			(1<<2)
#define FC6100_USE_SPI_ALL			(FC6100_USE_SPI3_CS1 | FC6100_USE_SPI3_CS0 | FC6100_USE_SPI1_CS0)

#define FC6100_USE_MMC1				(1<<3)
#define FC6100_USE_MMC2				(1<<4)
#define FC6100_USE_ALL_MMC			(FC6100_USE_MMC1 | FC6100_USE_MMC2)

#define FC6100_USE_MCBSP2			(1<<5) //2nd I2S codec (not defined)
#define FC6100_USE_MCBSP5			(1<<6)
#define FC6100_USE_ALL_MCBSP			(FC6100_USE_MCBSP2 | FC6100_USE_MCBSP5)

#define FC6100_USE_COMPOSITE_TV			(1<<7)
#define FC6100_USE_SVIDEO_TV			(1<<8)
#define FC6100_USE_TVOUT_ONLY			(1<<9) //TV_OUT as default screen
#define FC6100_USE_LCD_HSYNC_VSYNC		(1<<10)

#define FC6100_USE_UART1_RTS_CTS		(1<<11)
#define FC6100_UART1_RTS_CTS_NOT_USED		~(1<<11)

#define FC6100_USE_I2C2				(1<<12)
#define FC6100_USE_I2C3				(1<<13)
#define FC6100_USE_ALL_I2C			(FC6100_USE_I2C2 | FC6100_USE_I2C3)

//#define FC6100_USE_USB0_HOST			(1<<14)


#define FC6100_DISABLE_MMC1_WP			(1<<15)
#define FC6100_DISABLE_MMC1_CD			(1<<16)
#define FC6100_DISABLE_MMC1_WP_CD		( FC6100_DISABLE_MMC1_CD | FC6100_DISABLE_MMC1_WP )

#define FC6100_DISABLE_MMC2_WP			(1<<17)
#define FC6100_DISABLE_MMC2_CD			(1<<18)
#define FC6100_DISABLE_MMC2_WP_CD		( FC6100_DISABLE_MMC2_CD | FC6100_DISABLE_MMC2_WP )

#define FC6100_USE_CAM_HSYNC_VSYNC		(1<<19)

#define fc6100_mod_get_config()			(board_config)
#define fc6100_mod_get_audio_config()		(board_config & FC6100_USE_ALL_MCBSP)


// GPIO / Connector

/* odd pins */
/* 1 - 9 are analog audio */
#define FC_IPOD_RESET_N				(111)			// gpio_111
/* 3V3_PERM */
/* IS2_MCLK */
/* GND */
#define FC_I2S_OUT2				(20)			// mcbsp5_dx-gpio_20
#define FC_IT_HOST_N				(16)			// gpio_16
#define FC_I2C_0_SCL				(184)			// i2c3_scl-gpio_184
#define FC_I2C_0_SDA				(185)			// i2c3_sda-gpio_185
#define FC_SDIO_1_D0				(132)			// mmc2_dat0-gpio_132
#define FC_SDIO_1_D1				(133)			// mmc2_dat1-gpio_133
#define FC_SDIO_1_D2				(134)			// mmc2_dat2-gpio_134
#define FC_SDIO_1_D3				(135)			// mmc2_dat3-gpio_135
#define FC_IT_TOUCHSCREEN_N			(170)			// gpio_170
#define FC_I2C_1_SCL				(168)			// i2c2_scl-gpio_168
#define FC_I2C_1_SDA				(183)			// i2c2_sda-gpio_183
#define FC_SDIO_1_CLKIN				(139)			// mmc2_clkin-gpio_139
#define FC_SDIO_1_CMD				(131)			// mmc2_cmd-gpio_131
/* S_RESETOUT_N */
#define FC_SDIO_1_CLK				(130)			// mmc2_clk-gpio_130
/* 49 : XXX change in hw03 */
#define FC_LCD_BKL_EN				(18)			// gpio_18
#define FC_LCD_BKL_EN_HW02			(FC_I2S_OUT2)		// gpio_20

#define FC_LCD_HS				(67)			// dss_hs-gpio_67
#define FC_DSS_DATA7				(77)			// dss_data7-gpio_77
#define FC_DSS_DATA6				(76)			// dss_data6-gpio_76
#define FC_DSS_DATA9				(79)			// dss_data9-gpio_79
#define FC_DSS_DATA8				(78)			// dss_data8-gpio_78
#define FC_LCD_VS				(68)			// dss_vs-gpio_68
#define FC_DSS_DATA17				(87)			// dss_data17-gpio_87
#define FC_DSS_PCLK				(66)			// dss_pclk-gpio66
#define FC_LCD_BKL_PWM				(55)			// gpt9_pwm_evt-gpio_55
/* 69 : GND */
#define FC_LCD_EN				(69)			// dss_acbias-gpio_69
#define FC_DSS_DATA16				(86)			// dss_data16-gpio_86
#define FC_DSS_DATA23				(8)
#define FC_DSS_DATA22				(7)
#define FC_DSS_DATA20				(5)
#define FC_DSS_DATA21				(6)
#define FC_DSS_DATA18				(2)
#define FC_DSS_DATA19				(3)
#define FC_DSS_DATA13				(83)			// dss_data13-gpio_83
#define FC_DSS_DATA15				(85)			// dss_data15-gpio_85
#define FC_DSS_DATA14				(84)			// dss_data14-gpio_84
#define FC_DSS_DATA1				(71)			// dss_data1-gpio_71
#define FC_LCD_RST_N				(156)			// gpio_156
#define FC_DSS_DATA0				(70)			// dss_data0-gpio_70
#define FC_DSS_DATA12				(82)			// dss_data12-gpio_82
#define FC_DSS_DATA10				(80)			// dss_data10-gpio_80
#define FC_DSS_DATA11				(81)			// dss_data11-gpio_81
#define FC_DSS_DATA5				(75)			// dss_data5-gpio_75
#define FC_DSS_DATA4				(74)			// dss_data4-gpio_74
#define FC_DSS_DATA3				(73)			// dss_data3-gpio_73
#define FC_DSS_DATA2				(72)			// dss_data2-gpio_72
/* 3V3 MAIN */
/* 3V3 MAIN */
/* 3V3 MAIN */
#define FC_SDIO_1_CD_N				(15)			// gpio_15
#define FC_SDIO_1_WP_N				(157)			// gpio_157
#define FC_SDIO_0_WP_N				(49)			// gpio_49
#define FC_SDIO_0_WP_N_HW03_HW02		(129)			// gpio_129
#define FC_SDIO_0_CD_N				(93)			// gpio_93
/* 127 : RESET_HOST_N */
/* 129 : BOOTS */
/* 131 : XXX */
#define USB_1_OC_N				(46)			// gpio_46, follow ULPI_1_OC on schematics
#define USB_1_OC_N_HW04				(126)			// gpio_126_j20, follow ULPI_1_OC on schematics
/* 133 : USB_1_VBUS */
/* 135 : XXX */
#define USB_0_OC_N				(92)			// gpio_92,  follow ULPI_0_OC on schematics
#define USB_0_OC_N_HW02				FC_CAM_FLD		// Compliant with module from HW02 to HW04
/* 137 : USB_0_VBUS */
/* 139 : GND */


/* even pin */
/* pin 2 - 16 are analog audio and gnd*/
/* 18 : I2S_IN0 */
/* 20 : I2S_OUT0 */
#define FC_I2S_IN1				(118)			// mcbsp2_dr-gpio_118
/* 24 : I2S_FSYNC */
/* 26 : I2S_BCLK */
#define FC_I2S_OUT1				(119)			// mcbsp2_dx-gpio_119

#define FC_CAM_D5				(104)			// cam_d5-gpio_104
#define FC_CAM_HS				(94)			// cam_hs-gpio_94
#define FC_CAM_VS				(95)			// cam_vs-gpio_95
/* 36 : 3V3 MAIN */
/* 38 : 3V3 MAIN */
#define FC_CAM_D3				(102)			// cam_d3-gpio_102
#define FC_CAM_MCLK				(96)			// cam_xclka-gpio_96
#define FC_CAM_D2				(101)			// cam_d2-gpio_101
#define FC_CAM_D4				(103)			// cam_d4-gpio_103
#define FC_CAM_PCLK				(97)			// cam_pclk-gpio_97
#define FC_CAM_EN_N				(21)			// gpio_21
#define FC_CAM_D1				(100)			// cam_d1-gpio_100
#define FC_CAM_D0				(99)			// cam_d0-gpio_99
#define FC_CAM_D9				(108)			// cam_d9-gpio_108
//#define FC_SLEEP				CAM_D9 /* not clear on module HW06 */
#define FC_CAM_D8				(107)			// cam_d8-gpio_107
//#define FC_SLEEPOUT_N				CAM_D8 /* not clear on module HW06 */
#define FC_CAM_D7				(106)			// cam_d7-gpio_106
#define FC_CAM_D6				(105)			// cam_d6-gpio_105

#define FC_SPI_1_MISO				(90)			// mcspi3_miso-gpio_90
#define FC_SPI_1_MOSI				(89)			// mcspi3_mosi-gpio_89
#define FC_SPI_1_CS0_N				(91)			// mcspi3_cs0-gpio_91
#define FC_SPI_1_CLK				(88)			// mcspi3_clk-gpio_88
#define FC_GPIO_1_PWM				(57)			// gpt11_pwm_evt-gpio_57
/* 74 : 3V3 MAIN */
/* 76 : 3V3 MAIN */
#define FC_GPIO_0_PWM				(56)			// gpt10_pwm_evt-gpio_56
/* 80 : XXX CAM_FLD on HW06, SPI_1_CS1_N on HW 05 */
#define FC_CAM_FLD				(98)			// cam_fld-gpio_98
#define FC_SPI_1_CS1_N_HW05			USB_0_OC_N		// mcspi3_cs1-gpio_92 

#define FC_SDIO_0_CLK				(120)			// mmc1_clk-gpio_120
#define FC_SDIO_0_D0				(122)			// mmc1_dat0-gpio_122
#define FC_SDIO_0_D3				(125)			// mmc1_dat3-gpio_125
#define FC_SDIO_0_CMD				(121)			// mmc1_cmd-gpio_121
#define FC_SDIO_0_D2				(124)			// mmc1_dat2-gpio_124
#define FC_SDIO_0_D1				(123)			// mmc1_dat1-gpio_123
/* 94 : GND */
/* 96 : TV_LUMA */
/* 98 : TV_CHROMA */
#define FC_SPI_0_CS0_N				(174)			// mcspi1_cs0-gpio_174
#define FC_SPI_0_MOSI				(172)			// mcspi1_mosi-gpio_172	
#define FC_SPI_0_CLK				(171)			// mcspi1_clk-gpio_171
#define FC_SPI_0_MISO				(173)			// mcspi1_miso-gpio_173
#define FC_GPIO_2_CLK				(186)			// sys_clkout2-gpio_186	
#define FC_UART_0_RTS				(149)			//uart1_rts-gpio_149
#define FC_UART_0_CTS				(150)			//uart1_cts-gpio_150
/* 114 : GND */
/* 116 : 3V3 MAIN */
/* UART_0_RX */
/* UART_0_TX */
/* H_RESET_OUT_N */
/* 124 : 1V8_OUT */
/* 126 : 1V8_OUT */
/* USB_1_DM */
/* USB_1_DP */
/* USB_1_CPEN */
/* USB_0_CPEN */
/* USB_0_ID */
/* USB_0_DM */
/* USB_0_DP */


// not available on the connector (inside the module)

									// not clear cam_d8 seems to have the same use, SLEEPOUT_N
#define SLEEP_3V3				(0)			// gpio_0, follow SLEEP_REQUEST_N  on schematics


#define USB_0_ID				(110)			// gpio_110, follow ULPI_0_IDCTRL on schematics

#define USB_1_ID				(109)			// gpio_109, follow ULPI_1_IDCTRL on schematics
									// Compliant with module from HW00 to HW04

#define ULPI_0_RST_N				(14)			//gpio_14, not available on the connector (inside the module)
#define ULPI_1_RST_N				(22)			//gpio_22, not available on the connector (inside the module)






#define RF_RST_N				(23)			// gpio_23

#define WIFI_ANT_DET_N				(167)			// gpio_167

// Available on module HW05
#define SLEEPOUT_N				(47)			// gpio_47, follow SLEEPOUT_1V8 on schematics 
#define SLEEP_REQUEST_FLAG_HW05			(0)			// gpio_0

// Available on module HW07
#define BT_EXT_ANT_DET_N			(45)			// gpio_45


extern int board_rev;
extern unsigned int board_config;
extern struct omap2_mcspi_device_config fc6100_mcspi_config;
extern struct omap2_hsmmc_info mmc1_settings;
extern struct omap2_hsmmc_info mmc2_settings;
extern struct platform_device fc6100_gpio;

void __init fc6100_mod_init_irq(void);
void __init fc6100_mod_common_init(unsigned int mod_settings);
int __init fc6100_mod_display_init(struct omap_dss_device *lcd_device);
void __init omap_fc6100_factory_init(void);

#endif /* __ASM_ARCH_OMAP_FC6100_H */

