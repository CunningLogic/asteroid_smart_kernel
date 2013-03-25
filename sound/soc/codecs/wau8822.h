/*
 * wau8822.h  --  WAU8822 Soc Audio driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _WAU8822_H
#define _WAU8822_H


// Configuration Register Defines.
#define CFG_SOFTWARE_RESET    (0x000)
#define CFG_POWER_MANAGEMENT1 (0x001)
#define CFG_POWER_MANAGEMENT2	(0x002)
#define CFG_POWER_MANAGEMENT3 (0x003)
#define CFG_AUDIO_INTERFACE	  (0x004)
#define CFG_COMPANDING_CTRL   (0x005)
#define CFG_CLK_GEN_CTRL	  (0x006)
#define CFG_ADDITIONAL_CTRL	  (0x007)
#define CFG_GPIO 			  (0x008)
#define CFG_JACK_DETECT_CTRL1  (0x009)
#define CFG_DAC_CTRL		  (0x00A)
#define CFG_LEFT_DAC_DIGITAL_VOL  (0x00B)
#define CFG_RIGHT_DAC_DIGITAL_VOL (0x00C)
#define CFG_JACK_DETECT_CTRL2  	  (0x00D)
#define CFG_ADC_CTRL		  	  (0x00E)
#define CFG_LEFT_ADC_DIGITAL_VOL  (0x00F)
#define CFG_RIGHT_ADC_DIGITAL_VOL (0x010)
#define CFG_EQ1_SHELF_LOW   	  (0x012)
#define CFG_EQ2_PEAK1 			  (0x013)
#define CFG_EQ3_PEAK2 			  (0x014)
#define CFG_EQ4_PEAK3			  (0x015)
#define CFG_EQ5_HIGH_SHELF		  (0x016)
#define CFG_DAC_LIMITER1		  (0x018)
#define CFG_DAC_LIMITER2 		  (0x019)
#define CFG_NOTCH_FILTER1		  (0x01B)
#define CFG_NOTCH_FILTER2 		  (0x01C)
#define CFG_NOTCH_FILTER3 		  (0x01D)
#define CFG_NOTCH_FILTER4         (0x01E)
#define CFG_ALC_CTRL1			  (0x020)
#define CFG_ALC_CTRL2 			  (0x021)
#define CFG_ALC_CTRL3 			  (0x022)
#define CFG_NOISE_GATE 	(0x023)
#define CFG_PLLN		(0x024)
#define CFG_PLL_K1		(0x025)
#define CFG_PLL_K2 		(0x026)
#define CFG_PLL_K3 		(0x027)
#define CFG_ATTENUATION_CTRL	  (0x028)
#define CFG_3D_CONTROL   (0x029)
#define CFG_BEEP_CONTROL (0x02B)
#define CFG_INPUT_CTRL   (0x02C)
#define CFG_LEFT_INP_PGA_GAIN_CTRL  (0x02D)
#define CFG_RIGHT_INP_PGA_GAIN_CTRL (0x02E)
#define CFG_LEFT_ADC_BOOST_CTRL     (0x02F)
#define CFG_RIGHT_ADC_BOOST_CTRL 	(0x030)
#define CFG_OUTPUT_CTRL			    (0x031)
#define CFG_LEFT_MIXER_CTRL			(0x032)
#define CFG_RIGHT_MIXER_CTRL 		(0x033)
//LOUT1 --> HP-,ROUT1 --> HP+,LOUT2 --> SPKOUT-,ROUT2 --> SPKOUT+,OUT3 --> AUXOUT2,OUT4 --> AUXOUT1
#define CFG_LOUT1_HP_VOLUME_CTRL		(0x034)
#define CFG_ROUT1_HP_VOLUME_CTRL 		(0x035)
#define CFG_LOUT2_SPKR_VOLUME_CTRL		(0x036)
#define CFG_ROUT2_SPKR_VOLUME_CTRL 		(0x037)
#define CFG_OUT3_MIXER_CTRL (0x038)
#define CFG_OUT4_MIXER_CTRL (0x039)

/* WAU8822 register space */

#define WAU8822_RESET		0x0
#define WAU8822_POWER1		0x1
#define WAU8822_POWER2		0x2
#define WAU8822_POWER3		0x3
#define WAU8822_IFACE		0x4
#define WAU8822_COMP			0x5
#define WAU8822_CLOCK		0x6
#define WAU8822_ADD			0x7
#define WAU8822_GPIO			0x8
#define WAU8822_JACK1        0x9
#define WAU8822_DAC			0xa
#define WAU8822_DACVOLL	    0xb
#define WAU8822_DACVOLR      0xc
#define WAU8822_JACK2        0xd
#define WAU8822_ADC			0xe
#define WAU8822_ADCVOLL		0xf
#define WAU8822_ADCVOLR      0x10
#define WAU8822_EQ1			0x12
#define WAU8822_EQ2			0x13
#define WAU8822_EQ3			0x14
#define WAU8822_EQ4			0x15
#define WAU8822_EQ5			0x16
#define WAU8822_DACLIM1		0x18
#define WAU8822_DACLIM2		0x19
#define WAU8822_NOTCH1		0x1b
#define WAU8822_NOTCH2		0x1c
#define WAU8822_NOTCH3		0x1d
#define WAU8822_NOTCH4		0x1e
#define WAU8822_ALC1			0x20
#define WAU8822_ALC2			0x21
#define WAU8822_ALC3			0x22
#define WAU8822_NGATE		0x23
#define WAU8822_PLLN			0x24
#define WAU8822_PLLK1		0x25
#define WAU8822_PLLK2		0x26
#define WAU8822_PLLK3		0x27
#define WAU8822_VIDEO		0x28
#define WAU8822_3D           0x29
#define WAU8822_BEEP         0x2b
#define WAU8822_INPUT		0x2c
#define WAU8822_INPPGAL  	0x2d
#define WAU8822_INPPGAR      0x2e
#define WAU8822_ADCBOOSTL	0x2f
#define WAU8822_ADCBOOSTR    0x30
#define WAU8822_OUTPUT		0x31
#define WAU8822_MIXL	        0x32
#define WAU8822_MIXR         0x33
#define WAU8822_HPVOLL		0x34
#define WAU8822_HPVOLR       0x35
#define WAU8822_SPKVOLL      0x36
#define WAU8822_SPKVOLR      0x37
#define WAU8822_OUT3MIX		0x38
#define WAU8822_MONOMIX      0x39
#define WAU8822_SPKBOOST	0x31
#define WAU8822_AUX1BOOST    0x31
#define WAU8822_AUX2BOOST	0x31
#define WAU8822_BIASGEN	0x3

#define WAU8822_CACHEREGNUM 	64

/* Clock divider Id's */
#define WAU8822_OPCLKDIV		0
#define WAU8822_MCLKDIV		1
#define WAU8822_ADCCLK		2
#define WAU8822_DACCLK		3
#define WAU8822_BCLKDIV		4

/* DAC clock dividers */
#define WAU8822_DACCLK_F2	(1 << 3)
#define WAU8822_DACCLK_F4	(0 << 3)

/* ADC clock dividers */
#define WAU8822_ADCCLK_F2	(1 << 3)
#define WAU8822_ADCCLK_F4	(0 << 3)

/* PLL Out dividers */
#define WAU8822_OPCLKDIV_1	(0 << 4)
#define WAU8822_OPCLKDIV_2	(1 << 4)
#define WAU8822_OPCLKDIV_3	(2 << 4)
#define WAU8822_OPCLKDIV_4	(3 << 4)

/* BCLK clock dividers */
#define WAU8822_BCLKDIV_1	(0 << 2)
#define WAU8822_BCLKDIV_2	(1 << 2)
#define WAU8822_BCLKDIV_4	(2 << 2)
#define WAU8822_BCLKDIV_8	(3 << 2)
#define WAU8822_BCLKDIV_16	(4 << 2)
#define WAU8822_BCLKDIV_32	(5 << 2)

/* MCLK clock dividers */
#define WAU8822_MCLKDIV_1	(0 << 5)
#define WAU8822_MCLKDIV_1_5	(1 << 5)
#define WAU8822_MCLKDIV_2	(2 << 5)
#define WAU8822_MCLKDIV_3	(3 << 5)
#define WAU8822_MCLKDIV_4	(4 << 5)
#define WAU8822_MCLKDIV_6	(5 << 5)
#define WAU8822_MCLKDIV_8	(6 << 5)
#define WAU8822_MCLKDIV_12	(7 << 5)

#define WAU8822_OPCLK_DIV_1		(0 << 4)
#define WAU8822_OPCLK_DIV_2		(1 << 4)
#define WAU8822_OPCLK_DIV_3		(2 << 4)
#define WAU8822_OPCLK_DIV_4		(3 << 4)
#ifdef SPKVDD_5V
	#define WAU8822_5V_MODE 1
#else
	#define WAU8822_5V_MODE 0
#endif // #if definf SPKVDD_5V


struct wau8822_setup_data {
        unsigned short i2c_address;
        int mic2_input;
        ////unsigned short i2c_bus;
};

#endif
