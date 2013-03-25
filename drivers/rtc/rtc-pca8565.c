/*
 *  drivers/rtc/rtc-pca8565.c
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
 * Description:
 *  Driver for PCA8565 RTC
 *  Modified from drivers/rtc/rtc-pcf8583.c
 * 
 * Status: 
 *  Alarm and timer countdown not available
 *  
 * 
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/bcd.h>
#include <linux/delay.h>
#include <linux/i2c/rtc-pca8565.h>



struct pca8565 {
	struct rtc_device *rtc;
	u8 clock_output;	/* For more details see CLKOUT_control register at address 0Dh */
	u8 timer_source;	/* For more details see Register Timer_control register at address 0Eh */
	/* reserved for alarm and timer */
};

#define get_clock_output(x)    ((struct pca8565 *)i2c_get_clientdata(x))->clock_output
#define set_clock_output(x, v) get_clock_output(x) = v
#define get_timer_source(x)    ((struct pca8565 *)i2c_get_clientdata(x))->timer_source
#define set_timer_source(x, v) get_timer_source(x) = v

static struct i2c_driver pca8565_driver;

static int pca8565_i2c_write_clkout_timer_src_regs(struct i2c_client *client){
	u8 i2c_clkout_timer_src_buf[3] = { TIMER_CLOCK_OUT_REG };
	u8 value = 0;

	int err;
	struct i2c_msg clkout_timer_src_msgs[1] = {
		{
		 .addr = client->addr,
		 .flags = 0,	/* write */
		 .len = sizeof(i2c_clkout_timer_src_buf),
		 .buf = i2c_clkout_timer_src_buf}
	};

	/*
	 * Set timer source and clock output
	 */
	value = get_clock_output(client);
	i2c_clkout_timer_src_buf[1] = (value > TIMER_CLOCK_OUT_1_HZ) ? 0 : value ;
	i2c_clkout_timer_src_buf[1] |= FE; /* Activate clock out */

	value = get_timer_source(client);
	i2c_clkout_timer_src_buf[2] = (value > TIMER_SOURCE_1_60_HZ) ? 0 : value ;

	err = i2c_transfer(client->adapter, clkout_timer_src_msgs, ARRAY_SIZE(clkout_timer_src_msgs));
	if (err != ARRAY_SIZE(clkout_timer_src_msgs))
		goto write_failed;

	return 0;

 write_failed:
	dev_err(&client->dev, "%s: register write failed\n", __func__);
	return -EIO;
}


static int pca8565_i2c_write_ctrl_time_regs(struct i2c_client *client, u8 const *buf)
{
	u8 i2c_control_and_time_buf[1 + PCA8565_TIME_REG_LEN ] = { CONTROL_1_REG };
	int err;
	struct i2c_msg control_and_time_msgs[1] = {
		{
		 .addr = client->addr,
		 .flags = 0,	/* write */
		 .len = sizeof(i2c_control_and_time_buf),
		 .buf = i2c_control_and_time_buf}
	};

	memcpy(&i2c_control_and_time_buf[1], buf, PCA8565_TIME_REG_LEN);

	err = i2c_transfer(client->adapter, control_and_time_msgs, ARRAY_SIZE(control_and_time_msgs));
	if (err != ARRAY_SIZE(control_and_time_msgs))
		goto write_failed;

	return 0;


 write_failed:
	dev_err(&client->dev, "%s: register write failed\n", __func__);
	return -EIO;
}


/**
 * Reading procedure for PCA8565 RTC 
 * Recommended method for reading the time:
 *  1. Send a START condition and the slave address for write (A2h).
 *  2. Set the address pointer to registers Seconds (02h).
 *  3. Send a RESTART condition or STOP followed by START.
 *  4. Send the slave address for read (A3h).
 *  5. Read the register Seconds.
 *  6. Read the register Minutes.
 *  7. Read the register Hours.
 *  8. Read the register Days.
 *  9. Read the register Weekdays.
 *  10. Read the register Months_century.
 *  11. Read the register Years.
 *  12. Send a STOP condition.
 */

static int pca8565_i2c_read_ctrl_time_regs(struct i2c_client *client, u8 *buf)
{
	u8 reg_control_1_read[1] = { CONTROL_1_REG };

	struct i2c_msg msgs[2] = {
		{
		 .addr = client->addr,
		 .flags = 0,	/* write */
		 .len = sizeof(reg_control_1_read),
		 .buf = reg_control_1_read}
		,
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = PCA8565_TIME_REG_LEN,
		 .buf = buf}
	};

	int err;

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: register read failed\n", __func__);
		return -EIO;
	}
	return 0;

}


static int pca8565_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 regs[PCA8565_REG_LEN];
	u8 buf[2];
	int err;

	err = pca8565_i2c_read_ctrl_time_regs(client, regs);
	if (err < 0)
		return err;

	/*
	 * Ensure that the RTC is running.
	 */
	if( regs[CONTROL_1_REG] & STOP ){

		dev_warn(&client->dev, "%s: resetting RTC\n", __func__);

		buf[0] = CONTROL_1_REG; /* register control_1 */
		buf[1] = regs[CONTROL_1_REG] & ~STOP;

		if ((err = i2c_master_send(client, (char *)buf, 2)) < 0)
			return err;
	}

	/*
	 * Ensure the time is valid
	 */
	if( regs[SECONDS_REG] & VL ){
		dev_err(&client->dev, "%s: the integrity of the clock information is not guaranteed\n", __func__);
	}

	/*
	 * Retrieve time informations
	 */
	tm->tm_sec = bcd2bin( regs[SECONDS_REG] & SCD_MIN_MASK );
	tm->tm_min = bcd2bin( regs[MINUTES_REG] & SCD_MIN_MASK );
	tm->tm_hour = bcd2bin( regs[HOURS_REG] & HOURS_DAY_MASK );
	tm->tm_mday = bcd2bin( regs[DAYS_REG] & HOURS_DAY_MASK );
	tm->tm_wday = bcd2bin( regs[WEEKDAYS_REG] & WEEKDAYS_MASK ) - 1;

	tm->tm_mon = bcd2bin( regs[MONTHS_CENTURY_REG] & MONTHS_MASK ) - 1;
	tm->tm_year = bcd2bin( regs[YEARS_REG] ) +
		       ((regs[MONTHS_CENTURY_REG] & CENTURY) ? 1 : 0) * 100;

	return 0;
}

static int pca8565_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 regs[PCA8565_REG_LEN];
	u8 val = 0;
	int err;

	/* clean memory */
	memset(regs, 0, sizeof(regs));
	/*
	 * Set control register
	 */
	regs[CONTROL_1_REG] = 0x0; 
	regs[CONTROL_2_REG] = 0x0;

	/*
	 * Set time
	 */
	regs[SECONDS_REG] = bin2bcd( tm->tm_sec ) & SCD_MIN_MASK;
	regs[MINUTES_REG] = bin2bcd( tm->tm_min ) & SCD_MIN_MASK; 

	regs[HOURS_REG]   = bin2bcd( tm->tm_hour ) & HOURS_DAY_MASK;
	regs[DAYS_REG] 	  = bin2bcd( tm->tm_mday ) & HOURS_DAY_MASK;

	regs[WEEKDAYS_REG] 	  = (bin2bcd( tm->tm_wday ) & WEEKDAYS_MASK) + 1;
	regs[MONTHS_CENTURY_REG]  = (bin2bcd( tm->tm_mon  ) & MONTHS_MASK) + 1 
					+ ( (tm->tm_year > 100) ? CENTURY : 0 ) ;

	val = (tm->tm_year > 100) ? (tm->tm_year - 100) : tm->tm_year ;
	regs[YEARS_REG]		  = bin2bcd( val );

	err = pca8565_i2c_write_ctrl_time_regs(client, regs);
	if (err)
		return err;

	return 0;
}


static const struct rtc_class_ops pca8565_rtc_ops = {
	.read_time	= pca8565_rtc_read_time,
	.set_time	= pca8565_rtc_set_time,
	
};


static int pca8565_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct pca8565 *pca8565;
	struct pca8565_platform_data *pdata = client->dev.platform_data;
	u8 buf[3] = {};
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		dev_err(&client->dev, "%s: RTC i2c_check_functionality\n", __func__);
		return -ENODEV;
	}

	pca8565 = kzalloc(sizeof(struct pca8565), GFP_KERNEL);
	if (!pca8565){
		dev_err(&client->dev, "%s: RTC kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	pca8565->rtc = rtc_device_register(pca8565_driver.driver.name,
			&client->dev, &pca8565_rtc_ops, THIS_MODULE);

	if (IS_ERR(pca8565->rtc)) {
		err = PTR_ERR(pca8565->rtc);
		goto exit_kfree;
	}

	i2c_set_clientdata(client, pca8565);

	/*
	 * Ensure that the RTC is running.
	 */
	if( i2c_master_recv(client, buf , 1) != 1 ){
		dev_err(&client->dev, "%s: RTC control register failed\n", __func__);
	}

	if( buf[CONTROL_1_REG] & ( STOP | TEST1 | TESTC ) ){

		dev_warn(&client->dev, "%s: resetting RTC\n", __func__);

		/*
	 	* Clear control register
	 	*/
		buf[0] = CONTROL_1_REG; /* register control_1 */
		buf[1] = 0x0; 
		buf[2] = 0x0;

		if ((err = i2c_master_send(client, (char *)buf, 2)) < 0)
			goto exit_kfree;
	}

	/*
	 * Set timer source and clock output
	 */
	if( pdata ){
		set_clock_output( client, pdata->clock_output ); 
		set_timer_source( client, pdata->timer_source ); 

		if ((err = pca8565_i2c_write_clkout_timer_src_regs(client)) < 0){
			dev_warn(&client->dev, "%s: setting clock source and timer source failed\n", __func__);
			goto exit_kfree;
		}
	}


	return 0;

exit_kfree:
	dev_err(&client->dev, "%s: RTC registration failed\n", __func__);
	kfree(pca8565);
	return err;
}

static int __devexit pca8565_remove(struct i2c_client *client)
{
	struct pca8565 *pca8565 = i2c_get_clientdata(client);

	if (pca8565->rtc)
		rtc_device_unregister(pca8565->rtc);
	kfree(pca8565);
	return 0;
}

static const struct i2c_device_id pca8565_id[] = {
	{ "pca8565", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca8565_id);

static struct i2c_driver pca8565_driver = {
	.driver = {
		.name	= "pca8565",
		.owner	= THIS_MODULE,
	},
	.probe		= pca8565_probe,
	.remove		= __devexit_p(pca8565_remove),
	.id_table	= pca8565_id,
};


static __init int pca8565_init(void)
{
	return i2c_add_driver(&pca8565_driver);
}

static __exit void pca8565_exit(void)
{
	i2c_del_driver(&pca8565_driver);
}

module_init(pca8565_init);
module_exit(pca8565_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Christian Rosalie");
MODULE_DESCRIPTION("PCA8565 I2C RTC driver");
