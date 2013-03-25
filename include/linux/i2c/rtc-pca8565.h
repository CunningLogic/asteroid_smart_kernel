/*
 *  rtc-pca8565.h - header for PCA8565 RTC
 *  include/linux/i2c/rtc-pca8565.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Copyright (C) 2011 Parrot S.A.
 *
 * Author: Christian Rosalie <christian.rosalie@parrot.com>
 *
 * 
 */

/*
 * Registers mapping
 */

/* Control */
#define CONTROL_1_REG		0x00
#define CONTROL_2_REG		0x01

/* Time */
#define SECONDS_REG		0x02
#define MINUTES_REG		0x03
#define HOURS_REG		0x04
#define DAYS_REG		0x05
#define WEEKDAYS_REG		0x06
#define MONTHS_CENTURY_REG	0x07
#define YEARS_REG		0x08

/* Alarm */
#define MINUTE_ALRM_REG		0x09
#define HOUR_ALRM_REG		0x0A
#define DAY_ALRM_REG		0x0B
#define WEEKDAY_ALRM_REG	0x0C

/* Timer function */
#define TIMER_CLOCK_OUT_REG	0x0D
#define TIMER_CONTROL_REG	0x0E
#define TIMER_REG		0x0F


/*
 * CONTROL_1_REG
 */
#define TEST1		(1<<7)
#define STOP            (1<<5)	/* RTC stop */
#define TESTC		(1<<3)	/* power-on reset override may be enabled */


/*
 * CONTROL_2_REG
 */
#define	TI_TP		(1<<4)	/* INT pulses active according to Table 26 (subject to the status of TIE);*/
				/* Remark: note that if AF and AIE are active then INT will be permanently active */
#define	AF		(1<<3)	/* alarm flag active */
#define	TF		(1<<2)	/* timer flag active */
#define	AIE		(1<<1)	/* alarm interrupt enabled */
#define	TIE		0x1	/* timer interrupt enabled */


/*
 * SECONDS_REG, MINUTES_REG 
 */
#define	VL			(1<<7)		/* Integrity of the clock information is not guaranteed */	
#define	SCD_MIN_MASK		(0x7f)


/*
 * HOURS_REG, DAYS_REG
 */
#define	HOURS_DAY_MASK		(0x3f)


/*
 * WEEKDAYS_REG
 */
#define	WEEKDAYS_MASK		(0x7)		/* day of the week */

/*
 * MONTHS_CENTURY_REG
 */
#define	CENTURY			(1<<7)		/* indicates the century is x + 1 */
#define	MONTHS_MASK		(0x1f)

/*
 *  MINUTE_ALRM_REG
 */
#define AE_M			(1<<7)		/* minutes alarm is disabled */
#define	MINUTE_ALRM_TEN_MASK	(0x7<<4)	/* ten's place */
#define	MINUTE_ALRM_UNIT_MASK	(0x0F)		/* unit place */

/*
 *  HOUR_ALRM_REG, DAY_ALRM_REG
 */
#define AE_H			(1<<7)		/* hour alarm is disabled */
#define AE_D			(1<<7)		/* day alarm is disabled */
#define	DAY_HOUR_ALRM_MASK	(0x3f)


/*
 *  WEEKDAY_ALRM_REG
 */
#define AE_W			(1<<7)		/* weekday alarm is disabled */
#define	WEEKDAY_ALARM_MASK	(0x7)

/*
 *  TIMER_CLOCK_OUT_REG
 */

#define FE				(1<<7)		/* the CLKOUT output is activated */
/* frequency output at pin CLKOUT */
#define TIMER_CLOCK_OUT_32_KHZ    	0x00		/* 32.768 kHz */
#define TIMER_CLOCK_OUT_1_KHZ    	0x01		/* 1.024 kHz  */
#define TIMER_CLOCK_OUT_32_HZ    	0x02		/* 32 Hz      */
#define TIMER_CLOCK_OUT_1_HZ	   	0x03 		/*  1         */

/*
 *  TIMER_CONTROL_REG
 */
#define TE	(1<<7) 	/* Timer is enabled */
#define TIMER_SOURCE_4096_HZ    0x00
#define TIMER_SOURCE_64_HZ    	0x01
#define TIMER_SOURCE_1_HZ    	0x02
#define TIMER_SOURCE_1_60_HZ   	0x03 /*  1/60 Hz */


/* Register size */
#define PCA8565_REG_LEN		TIMER_REG + 1	/* 16 Register * 1 byte */
#define PCA8565_TIME_REG_LEN	YEARS_REG + 1	/* 8 Register */

struct pca8565_platform_data {
	u8 clock_output;	/* For more details see CLKOUT_control register at address 0Dh */
	u8 timer_source;	/* For more details see Timer_control register at address 0Eh */
};

