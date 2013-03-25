/* drivers/input/touchscreen/rohm_bu21023.c
 *
 * ROHM BU21023 touchscreen driver
 *
 * Copyright (C) 2010 ROHM SEMICONDUCTOR Co. Ltd.
 *
 * Author: CT Cheng <ct.cheng@rohm.com.tw>
 *
 * Modify: SPI -> IIC, Polling -> Interrupt, Add Filter, by Tracy Wen <tracy-wen@rohm.com.cn> 
 *
 * Firmware version: ver35h005 with calibration for PARROT 22 July 2011
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <asm/irq.h>
#include <linux/platform_device.h>
#include "rohm_bu21023.h"
#include <linux/slab.h>
	#include "rohm_bu21023_firmware.h"

//ROHM setting
#define TOUCH_SCREEN_SETTING

#ifdef TOUCH_SCREEN_SETTING
#define ROHM_TS_ABS_X_MIN 	50
#define ROHM_TS_ABS_X_MAX 	950
#define ROHM_TS_ABS_Y_MIN 	105
#define ROHM_TS_ABS_Y_MAX 	880
#endif
	
#define DAC_OFS 0x08

/* Print the coordinate of touch point in the debug console */

#define TWO_COUNT 8
#define ONE_COUNT 2 //10
#define KEY_COUNT 10
#define DELTA_X 100
#define DELTA_Y 160
#define DELTA_2_X 100
#define DELTA_2_Y 150
#define DELTA_3_X (2 * DELTA_2_X)
#define DELTA_3_Y (2 * DELTA_2_Y)
#define STEP_DIFF 0x10
#define TIMER_NS  3000000 //10000000   //report rate 1/10000000=100Hz

#define DUAL_X_OFFSET 0
#define DUAL_Y_OFFSET 0

/* ABS "Relative Center Position" */
#define FLAT 0x0000
/* ABS "Kernel Pre-Filtering" */
#define FUZZ 0x0010

/********************************************/

#define SW_AVG
#define FILTER_2


/********************************************/

/* REGISTER DEFINITION */
#define VADOUT_YP_H     0x00
#define VADOUT_YP_L     0x01
#define VADOUT_XP_H     0x02
#define VADOUT_XP_L     0x03
#define VADOUT_YN_H     0x04
#define VADOUT_YN_L     0x05
#define VADOUT_XN_H     0x06
#define VADOUT_XN_L     0x07
#define PRM1_X_H        0x08
#define PRM1_X_L        0x09
#define PRM1_Y_H        0x0a
#define PRM1_Y_L        0x0b
#define PRM2_X_H        0x0c
#define PRM2_X_L        0x0d
#define PRM2_Y_H        0x0e
#define PRM2_Y_L        0x0f
#define MLT_PRM_MONI_X  0x10
#define MLT_PRM_MONI_Y  0x11
#define DEBUG_MONI_1    0x12
#define DEBUG_MONI_2    0x13
#define VADOUT_ZX_H     0x14
#define VADOUT_ZX_L     0x15
#define VADOUT_ZY_H     0x16
#define VADOUT_ZY_L     0x17
#define Z_PARAM_H       0x18
#define Z_PARAM_L       0x19

#define POS_X1_H        0x20
#define POS_X1_L        0x21
#define POS_Y1_H        0x22
#define POS_Y1_L        0x23
#define POS_X2_H        0x24
#define POS_X2_L        0x25
#define POS_Y2_H        0x26
#define POS_Y2_L        0x27

#define TOUCH_GESTURE   0x29
#define INT_STS         0x2a
#define ERR_STS         0x2b
#define CMN_SETUP1      0x30
#define CMN_SETUP2      0x31
#define CMN_SETUP3      0x32
#define INTVL_TIME      0x33
#define STEP_X          0x34
#define STEP_Y          0x35

#define OFS_X           0x38
#define OFS_Y           0x39
#define TH_DET_TOUCH    0x3A
#define TH_DET_GESTURE  0x3B

#define INT_MSK         0x3D
#define INT_CLR         0x3E
#define ERR_MSK         0x3F
#define SYSTEM          0x40

#define FORCE_INT       0x42

#define PRM_CPU_FREQ    0x50
#define EEPROM_ADDR     0x51
#define PRM_B_OFS       0x52
#define TH_SLEEP_IN     0x53

#define EVR_XY          0x56
#define PRM_SWOFF_TIME  0x57

#define P_VERSION       0x5F
#define PRM_AD_CTRL     0x60
#define PRM_WAIT_AD     0x61
#define SWCONT          0x62
#define EVR_X           0x63
#define EVR_Y           0x64
#define TEST1           0x65
#define TEST2           0x66
#define TEST3           0x67
#define CALIB_REG1      0x68
#define CALIB_REG2      0x69
#define CALIB_REG3      0x6A
#define TEST7           0x6B
#define TEST8           0x6C
#define TEST9           0x6D

#define EX_ADDR_H       0x70
#define EX_ADDR_L       0x71
#define EX_WDAT         0x72
#define EX_RDAT         0x73
#define EX_CHK_SUM1     0x74
#define EX_CHK_SUM2     0x75
#define EX_CHK_SUM3     0x76

/***********************/

#ifdef SW_AVG

/* Size of Moving Average buffer array */
#define BUFF_ARRAY      20

#define DISTANCE_X 32
#define DISTANCE_Y 32
#define AVG_BOUND_X 16
#define AVG_BOUND_Y 16
#define S_DISTANCE_X 6
#define S_DISTANCE_Y 6
#define S_AVG_BOUND_X 4
#define S_AVG_BOUND_Y 4

/* skip times of moving average when touch starting */
#define SKIP_AVG_1      TWO_COUNT
#define SKIP_AVG_2 5

/* Blind Area */
#define X_LIM           300
#define Y_LIM           300

#endif

struct touch_point
{
    unsigned int x;
    unsigned int y;

#ifdef SW_AVG
    unsigned int buff_x[BUFF_ARRAY];
    unsigned int buff_y[BUFF_ARRAY];
    unsigned char buff_cnt;
    /* Previous coordinate of touch point after moving average calculating */
    unsigned int old_avg_x;
    unsigned int old_avg_y;
#endif


};

struct touch_point tp1, tp2;

static u8 finger_count_0 = 0;	//from 0 to 1
static u8 finger_count_1 = 0;	//from 2 to 1
static u8 finger_count_2 = 0;	//from 0/1 to 2
static u8 finger_count_3 = 0;	//from 2 to 2VAr lock
static u8 key_count = 0;
static u8 error_flg_1 = 0;
static u8 error_flg_2 = 0;
static u8 variance_lock = 0;
static u8 finger =0;
static unsigned int x1_old = 0;
static unsigned int y1_old = 0;
static unsigned int x2_old = 0;
static unsigned int y2_old = 0;

static u8 hold_data = 0;
static int x_cap_center = 0;
static int y_cap_center = 0;
static int x_diff = 0;
static int y_diff = 0;
static int x_temp = 0;
static int y_temp = 0;
static unsigned int variance = 0;

static unsigned int x1_var = 0;
static unsigned int y1_var = 0;
static unsigned int x2_var = 0;
static unsigned int y2_var = 0;


/********************************************/

static struct workqueue_struct *rohm_wq;

struct rohm_ts_data 
{
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint32_t flags;
	int (*power)(int on);
};



#ifdef SW_AVG

//-----------------------------------------------------------------------------
//
//Coord_Avg
//
//-----------------------------------------------------------------------------
static void Coord_Avg (struct touch_point *tp)
{

	unsigned long temp_x = 0, temp_y = 0, temp_n = 0;
	unsigned int i;
	////////////////////////////////////////
	// X1,Y1 Moving Avg
	////////////////////////////////////////
	if((tp->x != 0) && (tp->y != 0))
	{			               			  			      				
		if(tp->buff_cnt >= SKIP_AVG_1)
		{  
		    if(((abs(tp->buff_x[0] - tp->x) > DISTANCE_X) && (abs(tp->buff_y[0] - tp->y) > DISTANCE_Y)) ||
		       ((abs(tp->buff_x[0] - tp->x) > S_DISTANCE_X) && (abs(tp->buff_y[0] - tp->y) < S_DISTANCE_Y)) ||
			   ((abs(tp->buff_x[0] - tp->x) < S_DISTANCE_X) && (abs(tp->buff_y[0] - tp->y) > S_DISTANCE_Y)) ||			   
			   (((tp->old_avg_x != 0) && (abs(tp->old_avg_x - tp->x) > AVG_BOUND_X)) && 
			   ( (tp->old_avg_y != 0) && (abs(tp->old_avg_y - tp->y) > AVG_BOUND_Y))) ||
			   (((tp->old_avg_x != 0) && (abs(tp->old_avg_x - tp->x) > S_AVG_BOUND_X)) && 			
			   ( (tp->old_avg_y != 0) && (abs(tp->old_avg_y - tp->y) < S_AVG_BOUND_Y)))||
			   (((tp->old_avg_x != 0) && (abs(tp->old_avg_x - tp->x) < S_AVG_BOUND_X)) && 			
			   ( (tp->old_avg_y != 0) && (abs(tp->old_avg_y - tp->y) > S_AVG_BOUND_Y))))
			{
				for (i = 0; i < tp->buff_cnt; i++)
				{
					tp->buff_x[tp->buff_cnt - i] = tp->buff_x[tp->buff_cnt - i - 1];
					tp->buff_y[tp->buff_cnt - i] = tp->buff_y[tp->buff_cnt - i - 1];
				}
				tp->buff_x[0] = tp->x;
				tp->buff_y[0] = tp->y;
 
				temp_x = 0;
				temp_y = 0;
				temp_n = 0;
        
				for (i = 0; i <= tp->buff_cnt; i++)
				{
					temp_x += ((unsigned long) (tp->buff_x[i] * (tp->buff_cnt - i + 1)));
					temp_y += ((unsigned long) (tp->buff_y[i] * (tp->buff_cnt - i + 1)));
					temp_n += (unsigned long) (tp->buff_cnt - i + 1);
				}            
				tp->x = temp_x / temp_n;
				tp->y = temp_y / temp_n;
		
				tp->old_avg_x = tp->x;
				tp->old_avg_y = tp->y;  	
				if(tp->buff_cnt < (BUFF_ARRAY-1))
					tp->buff_cnt++;
			}
			else 
			{	  
				tp->x = tp->old_avg_x;
				tp->y = tp->old_avg_y;	
			} 
		}
		else
		{
			for (i = 0; i < tp->buff_cnt; i++)
			{
				tp->buff_x[tp->buff_cnt - i] = tp->buff_x[tp->buff_cnt - i - 1];
				tp->buff_y[tp->buff_cnt - i] = tp->buff_y[tp->buff_cnt - i - 1];
			}	
			tp->buff_x[0] = tp->x;
			tp->buff_y[0] = tp->y;
			if(tp->buff_cnt < (BUFF_ARRAY-1))
				tp->buff_cnt++;
			tp->old_avg_x = tp->x;
			tp->old_avg_y = tp->y;
	}
    }				//End/ of "if((x1 != 0) && (y1 != 0))"
    else
    {
	
	tp->buff_cnt = 0;
	if ((tp->buff_x[0] != 0) && (tp->buff_y[0] != 0))
	{
	    tp->x = tp->buff_x[0];
	    tp->y = tp->buff_y[0];
	}
	else
	{
			tp->x = 0;
			tp->y = 0;			
	}
	tp->buff_x[0] = 0;
	tp->buff_y[0] = 0;
	tp->old_avg_x = 0;
	tp->old_avg_y = 0;
    }


		}

static void Coord_Avg2(struct touch_point *tp)
{
  
  unsigned long temp_x = 0, temp_y = 0, temp_n = 0;
  unsigned int i;
  ////////////////////////////////////////
  // X1,Y1 Moving Avg
  ////////////////////////////////////////
  if ((tp->x != 0) && (tp->y != 0))
  {
 
	for (i = 0; i < tp->buff_cnt; i++)
	{
	  tp->buff_x[tp->buff_cnt - i] = tp->buff_x[tp->buff_cnt - i - 1];
	  tp->buff_y[tp->buff_cnt - i] = tp->buff_y[tp->buff_cnt - i - 1];
	}
	tp->buff_x[0] = tp->x;
	tp->buff_y[0] = tp->y;
	
	temp_x = 0;
	temp_y = 0;
	temp_n = 0;
	
	for (i = 0; i <= tp->buff_cnt; i++)
	{
	  temp_x += ((unsigned long) (tp->buff_x[i] * (tp->buff_cnt - i + 1)));
	  temp_y += ((unsigned long) (tp->buff_y[i] * (tp->buff_cnt - i + 1)));
	  temp_n += (unsigned long) (tp->buff_cnt - i + 1);
	}
	tp->x = temp_x / temp_n;
	tp->y = temp_y / temp_n;
	
	tp->old_avg_x = tp->x;
	tp->old_avg_y = tp->y;
	if (tp->buff_cnt < (BUFF_ARRAY - 1))
	  tp->buff_cnt++;
      }
   				//End/ of "if((x1 != 0) && (y1 != 0))"
	else 
	{
		tp->buff_cnt = 0;
		if((tp->buff_x[0] != 0) && (tp->buff_y[0] != 0))
		{
			tp->x = tp->buff_x[0];
			tp->y = tp->buff_y[0];
		}
		else
		{
			tp->x = 0;
			tp->y = 0;
		}
		tp->buff_x[0] = 0;
		tp->buff_y[0] = 0;
		tp->old_avg_x = 0;
		tp->old_avg_y = 0;
	}
}

static void Reset_Coord_Avg(struct touch_point *tp)
{
    tp->buff_cnt = 0;

    tp->buff_x[0] = 0;
    tp->buff_y[0] = 0;
    tp->old_avg_x = 0;
    tp->old_avg_y = 0;
    
    

}
#endif

static int BU21020ManualCalibration(struct i2c_client *client)
{

    unsigned char test4;	//Reg0x68
    unsigned char test5;	//Reg0x69
    unsigned char test6;	//Reg0x6A

    unsigned char calib_sts;
    int calib_x;
    int calib_y;
    int calib_reg_x;
    int calib_reg_y;
    int calib_err_x;
    int calib_err_y;
    int i, j, k;
    int ret;
    struct i2c_msg msg[2];
    uint8_t buf[34];

    i2c_smbus_write_byte_data(client, 0x65, 0x01);	//calibration data disable

    for (i = 0; i < 10; i++)
    {
	for (k = 0; k < 20; k++)
	{
	    udelay(1000);
	}


	//1'nd data
	msg[0].addr = client->addr;	//For BUMV21018MWV test only
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	buf[0] = 0x08;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;
	ret = i2c_transfer(client->adapter, msg, 2);
#ifdef SHOW_TOUCH_POINT
	printk("0x%x\t0x%x\t0x%x\t0x%x\t0x%x\n", buf[0], buf[1], buf[2], buf[3], buf[33]);
#endif
	if (!(buf[33] & 0x01))
	    break;

    }

    if (i >= 10)
    {
	printk("reg0x18 = 0x%x\n", i2c_smbus_read_byte_data(client, 0x18));
	printk("reg0x3A = 0x%x\n", i2c_smbus_read_byte_data(client, 0x3A));
	printk("please do not touch panel, and do this step again. 1\n");
	i2c_smbus_write_byte_data(client, 0x65, 0x21);	//calibration data enable
	for (k = 0; k < 220; k++)	//wait 11 sampling periods, almost 11ms
	{
	    udelay(1000);
	}
	return 1;
}


    calib_x = buf[0] * 4 + buf[1] - DAC_OFS;
    calib_y = buf[2] * 4 + buf[3] - DAC_OFS;


    for (j = 0; j < 100; j++)
    {
	calib_reg_x = calib_x + (calib_x & 0x200) * 2;	//calib_reg_x = calib_x + calib[9]*1024
	calib_reg_y = calib_y + (calib_y & 0x200) * 2;	//calib_reg_x = calib_x + calib[9]*1024

	test4 = calib_reg_x >> 3;	//calib_reg_x[10:3]
	test5 = (calib_reg_y & 0x07) * 16 + (calib_reg_x & 0x07);	//calib_reg_y[10:3]*16+ calib_reg_x[2:0]
	test6 = calib_reg_y >> 3;	//calib_reg_y[10:3]

	i2c_smbus_write_byte_data(client, 0x68, test4);
	i2c_smbus_write_byte_data(client, 0x69, test5);
	i2c_smbus_write_byte_data(client, 0x6A, test6);

	i2c_smbus_write_byte_data(client, 0x65, 0x21);	//calibration data enable

	for (k = 0; k < 200; k++)
	{
	    udelay(1000);
	}


	i2c_smbus_write_byte_data(client, 0x42, 0x00);
	i2c_smbus_write_byte_data(client, 0x42, 0x01);	//force calibration
	i2c_smbus_write_byte_data(client, 0x42, 0x00);

	for (k = 0; k < 200; k++)
	{
	    udelay(1000);
	}

	for (i = 0; i <= 200; i++)
	{
	    calib_sts = i2c_smbus_read_byte_data(client, 0x29);
#ifdef SHOW_TOUCH_POINT
	    printk("calib_sts = 0x%x\n", calib_sts);
#endif
	    if (calib_sts & 0x01)
	    {
		printk("please do not touch panel, and do this step again. 1\n");
		return 1;
	    }
	    if (!(calib_sts & 0x08))	//Check calibration Status, calib_status == 10, keep going
		break;
	    udelay(200);
	}

	if (i >= 200)
	{
	    printk(" Manual calibration can not finish. Calib_status = 0x%x\n", calib_sts);
	    return 1;
	}

	if (!(calib_sts & 0x04))	//calib_status == 00
	    break;


	for (i = 0; i < 10; i++)
	{
	    for (k = 0; k < 20; k++)
	    {
		udelay(1000);
	    }


	    //2'nd data
	    msg[0].addr = client->addr;	//For BUMV21018MWV test only
	    msg[0].flags = 0;
	    msg[0].len = 1;
	    msg[0].buf = buf;

	    buf[0] = 0x08;

	    msg[1].addr = client->addr;
	    msg[1].flags = I2C_M_RD;
	    msg[1].len = sizeof(buf);
	    msg[1].buf = buf;
	    ret = i2c_transfer(client->adapter, msg, 2);
#ifdef SHOW_TOUCH_POINT
	    printk("0x%x\t0x%x\t0x%x\t0x%x\t0x%x\n", buf[0], buf[1], buf[2], buf[3], buf[33]);
#endif
	    if (!(buf[33] & 0x01))
		break;
	}

	if (i >= 10)
	{
	    printk("please do not touch panel, and do this step again. 2\n");
	    return 2;
	}

	calib_err_x = buf[0] * 4 + buf[1];
	calib_err_y = buf[2] * 4 + buf[3];

	if (calib_err_x <= 4)
	{
	    calib_x = calib_x - 1;
	}
	else if (calib_err_x >= 60)
	{
	    calib_x = calib_x + 1;
	}


	if (calib_err_y <= 4)
	{
	    calib_y = calib_y - 1;
	}
	else if (calib_err_y <= 60)
	{
	    calib_y = calib_y + 1;
	}
    }


    if (j >= 100)
    {
	printk("Manual calibration error.\n");
	return 3;
    }

    printk(" Manual calibration succeed.\n");
    return 0;
}



static void rohm_ts_work_func(struct work_struct *work)
{
	//define ouput data start
	//unsigned int finger = 0 ;
	unsigned int touch = 0 ;
 	unsigned int gesture = 0 ;
	unsigned int report = 0;

    unsigned int tempX = 0;
    unsigned int tempY = 0;
	
 	//end of define data
	
	int ret;
	int bad_data = 0;
	struct i2c_msg msg[2];
	uint8_t buf[10];
	struct rohm_ts_data *ts = container_of(work, struct rohm_ts_data, work);
	//1'nd data
	msg[0].addr = ts->client->addr;				//For BUMV21018MWV test only
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

    buf[0] = 0x20;  //I2C slave address

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;
	ret = i2c_transfer(ts->client->adapter, msg, 2);


	if (ret < 0) 
	{
		bad_data = 1;
		goto done;
	} 
	else
	{
		bad_data = 0;

        /*XY coordinate */
        tp1.x = buf[1] | ((uint16_t)buf[0] << 2); 
		tp1.y = buf[3] | ((uint16_t)buf[2] << 2);
		tp2.x = buf[5] | ((uint16_t)buf[4] << 2); 
		tp2.y = buf[7] | ((uint16_t)buf[6] << 2);

		touch = buf[8];
		gesture = buf[9];

#ifdef SHOW_TOUCH_POINT		
	printk("\n1(X %d | Y %d) : 2(X %d | Y %d) : touch %d\n", tp1.x, tp1.y, tp2.x, tp2.y, touch);
#endif

#ifdef FILTER_1 //if coordinate and touch_info are asynchronous, ignore information this time and next time

		if( (tp1.y > 0) && (tp1.x > 0) && (tp2.y>0) && (tp2.x>0) )
		{

			if (error_flg_2 || error_flg_1)
			{
				error_flg_1 =0;
				if((touch & 0x03)==0x03)
					error_flg_2 =0;
				else
					error_flg_2 =1;
				hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);			
				return;
			}	
			else if((touch&0x03)!=0x03)
			{
			 		error_flg_2 =1;
				 	hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
		     		return;
			}
			else 
				error_flg_2 = 0;
		
		}
		else if ( ((tp1.y > 0) && (tp1.x > 0)) || ((tp2.y>0) && (tp2.x>0) ))
		{

			if (error_flg_1 || error_flg_2)
			{
				error_flg_2 =0;
				if((touch & 0x03)==0x01)
					error_flg_1 =0;
				else
					error_flg_1 =1;				
				hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);			
				return;
			}	
			else if((touch&0x03)!=0x01)
			{
			 		error_flg_1 =1;
				 	hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
		     		return;
			}
			else 
				error_flg_1 = 0;			
		}
		else if( (tp1.y > 0) || (tp1.x > 0) || (tp2.y>0) || (tp2.x>0) || (touch&0x03)	)
		{

			error_flg_1 =0;
			error_flg_2 =0;
			hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);			
			return;
		}
		else
		{
			error_flg_1 =0;
			error_flg_2 =0;
		}		

#endif


		if( ( (tp1.y > 0) && (tp1.x > 0) ) && ( (tp2.y>0) && (tp2.x>0) ) )
		{


#ifdef SHOW_TOUCH_POINT
	    printk("BEF 1(X %d | Y %d) : 2(X %d | Y %d) : touch %d\n", tp1.x, tp1.y, tp2.x, tp2.y, touch);
#endif			

	    //Filter for Big Contact Surface
	    //if ((abs(tp1.x - tp2.x) < DELTA_2_X) && (abs(tp1.y - tp2.y) < DELTA_2_Y))
	    if ((((abs(tp1.x - tp2.x) < DELTA_2_X) && (abs(tp1.y - tp2.y) < DELTA_2_Y))
		|| ((abs(tp1.x - tp2.x) < DELTA_3_X) && (abs(tp1.y - tp2.y) < (DELTA_2_Y / 2)))
		|| ((abs(tp1.x - tp2.x) < (DELTA_2_X / 2)) && (abs(tp1.y - tp2.y) < DELTA_3_Y))) && (finger != 2))
	    {
		finger = 1;
		tempX = (tp1.x + tp2.x) / 2;
		tempY = (tp1.y + tp2.y) / 2;
		tp1.x = tempX;
		tp1.y = tempY;
#ifdef SHOW_TOUCH_POINT
		printk("reject\n");
#endif
	    }
	    else if ((abs(tp1.x - tp2.x) < DELTA_2_X) && (abs(tp1.y - tp2.y) < DELTA_2_Y))
	    {
	      finger = 1;
	      tempX = (tp1.x + tp2.x) / 2;
	      tempY = (tp1.y + tp2.y) / 2;
	      tp1.x = tempX;
	      tp1.y = tempY;
	      #ifdef SHOW_TOUCH_POINT
	      printk("reject\n");
	      #endif
	    }
	    else
	    {
			finger = 2;	

		//Variance Swapping Correction
		if ((x1_old != 0) && (y1_old != 0) && ((finger_count_3 == 0) || variance_lock == 1))
		{
		    variance = (x1_old * y1_old);

		    x1_var = abs(x1_old - tp1.x);
		    y1_var = abs(y1_old - tp1.y);
		    x2_var = abs(x1_old - tp2.x);
		    y2_var = abs(y1_old - tp2.y);

		    #ifdef SHOW_TOUCH_POINT
		    printk("Variance 1(%d) 2(%d) Old(%d)\n", (y1_var * x1_var) , (y2_var * x2_var), variance);
		    #endif
		    
		    
		    if ((y1_var * x1_var) > (y2_var * x2_var))
		    {

			x_temp = tp1.x;
			y_temp = tp1.y;
			tp1.x = tp2.x;
			tp1.y = tp2.y;
			tp2.x = x_temp;
			tp2.y = y_temp;
			#ifdef SHOW_TOUCH_POINT
			printk("VAR 1(X %d | Y %d) : 2(X %d | Y %d) : touch %d\n", tp1.x, tp1.y, tp2.x, tp2.y, touch);
			#endif
			variance_lock = 1;
		    }
		}

		//Dif Between Value


		if (hold_data == 0 && ((tp1.x != 0) && (tp1.y != 0)))
		{
		    if ((x1_old == 0) || (y1_old == 0))
		    {
			x_diff = 0;
			y_diff = 0;

		    }
		    else
		    {
			x_diff = tp1.x - x1_old;
			y_diff = tp1.y - y1_old;
		    }

		    hold_data = 1;
		    #ifdef SHOW_TOUCH_POINT
		    printk("diff X %d | Y %d\n", x_diff, y_diff);
		    #endif
		    
		    #ifdef SW_AVG
		    Reset_Coord_Avg(&tp1);
		    Reset_Coord_Avg(&tp2);
		    #endif
		}

		if (		/*(tp1.x < (x_cap + X_LIM)) && (tp1.x > (x_cap - X_LIM)) && (tp1.y < (y_cap + Y_LIM)) && (tp1.y > (y_cap - Y_LIM))
				   && */ (hold_data == 1))
		{
		    x_temp = tp1.x - x_diff;
		    y_temp = tp1.y - y_diff;

		    tp1.x = x_temp;
		    tp1.y = y_temp;
		    #ifdef SHOW_TOUCH_POINT
		    printk("blinded\n");
		    #endif
		}


		#ifdef SW_AVG
		Coord_Avg2(&tp1);
		Coord_Avg2(&tp2);
		#endif

		tp2.x += DUAL_X_OFFSET;
		tp2.y += DUAL_Y_OFFSET;

	    }

	    #ifdef SHOW_TOUCH_POINT
	    printk("AFT 1(X %d | Y %d) : 2(X %d | Y %d) : touch %d\n", tp1.x, tp1.y, tp2.x, tp2.y, touch);
	    #endif
	    
#ifdef FILTER_2 //add counter between touch status from 2/0 to 1, or from 1/0 to 2
		    finger_count_1 = 0; 
			finger_count_2++;
			if(finger_count_2 < TWO_COUNT)
			{
				hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);			
				return;
			}
#endif		

	    /*if (finger_count_2 >= TWO_COUNT)
			{
				if (abs(x1-x1_old) < STEP_DIFF)
					x1 = x1_old;

				if (abs(y1-y1_old) < STEP_DIFF)
					y1 = y1_old;

				if (abs(x2-x2_old) < STEP_DIFF)
					x2 = x2_old;

				if (abs(y2-y2_old) < STEP_DIFF)
					y2 = y2_old;
			}
			x1_old = x1;
			y1_old = y1;
			x2_old = x2;
	    y2_old = y2;*/
		}
		else if ( ( (tp1.y>0)&&(tp1.x>0) ) || ( (tp2.y>0)&& (tp2.x>0) ) )
		{
		    if((tp2.x != 0) && (tp2.y != 0))
		   	{
		        tp1.x = tp2.x;
				tp1.y = tp2.y;
				tp2.x = 0;
				tp2.y = 0;
		   	}	
#ifdef FILTER_2 //add counter between touch status from 2/0 to 1, or from 1/0 to 2
			if(finger==0)
			{
		        finger_count_0 =1;
				finger = 1;
				hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);	
		        return;
			}			
		    else if(finger==2)
		    {
		        finger_count_1 = ONE_COUNT;
				finger_count_2 = 0;
				finger = 1;
				hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
		        return;
		    }
			else //if(finger ==1)
			{
		    	if(finger_count_1 > 1)
		        {
		            finger_count_1--;				
					hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);	
		            return;
		       	}
				else if(finger_count_1 == 1)
				{
		            x1_old = tp1.x;
		        	y1_old = tp1.y;
		            finger_count_1--;				
					hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);	
		            return;
				}
				else if(finger_count_0 == 1)
				{
					finger_count_0 =0;
					x1_old = tp1.x;
		        	y1_old = tp1.y;
				}

		    }				
			if( (abs(tp1.x-x1_old)>=DELTA_X) || (abs(tp1.y-y1_old)>=DELTA_Y) )          	
		    {   	
				x1_old = tp1.x;
		        y1_old = tp1.y;
		//printk("%d,%d,%d\n", finger_count_0, finger_count_1, finger_count_2);
				hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);	
		        return;
		    }  

			x1_old = tp1.x;
		    y1_old = tp1.y;
#endif		
			finger = 1;
#ifdef KEY_TOUCH			
			if((finger_count_1 == 0) && (tp1.x >= ROHM_TS_ABS_X_MAX))
			{
				key_count += 1;
			}
			else 
			{
				key_count = 0;
			}	
#endif	
			
		}
		else
		{	
			x1_old = 0;
			y1_old = 0;
			x2_old = 0;
			y2_old = 0;
		    finger = 0;	
		    error_flg_1 = 0;
			error_flg_2 = 0;
		    finger_count_0 = 0;
		    finger_count_1 = 0;
		    finger_count_2 = 0;
		}


	//printk("x1_old = %d,    y1_old = %d,    count = %d\n",x1_old,y1_old,finger_count_0);



		if(finger == 0)
		{
	    hold_data = 0;
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, 0); // 0 touch_width,File system do judge this register
#ifdef KEY_TOUCH	
			if(key_count > KEY_COUNT)
			{
				input_report_key(ts->input_dev, KEY_EXIT, 0);
				report = 5;  			
			}
			else 
#endif
			{
			input_report_key(ts->input_dev, BTN_TOUCH, finger);  // finger  num   0, 1, 2
            input_report_key(ts->input_dev, BTN_2, finger > 1); // finger2 state 0, 0, 1
			report = 0;
			}     

            input_sync(ts->input_dev);   // up load data     
			key_count = 0;
	    finger_count_3 = 0;
	    variance_lock = 0;
		}
		else if (finger == 1)
		{
	    hold_data = 0;
#ifdef KEY_TOUCH	
			if(key_count)
			{
				if(key_count == KEY_COUNT)
				{		
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 3);	
		        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, tp1.x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, tp1.y);
		    x1_old = tp1.x;
		    y1_old = tp1.y;
			    input_mt_sync(ts->input_dev);		
				//input_event(ts->input_dev, EV_KEY, KEY_EXIT, 1);
				input_report_key(ts->input_dev, KEY_EXIT, 1);
				input_sync(ts->input_dev);
				report = 4;
		    finger_count_3 = 0;
		    variance_lock = 0;
				}  
			}

			else
#endif
			{
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 3);	
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, tp1.x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, tp1.y);
		x1_old = tp1.x;
		y1_old = tp1.y;
	        input_mt_sync(ts->input_dev);
			input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, 0); // 0 touch_width,File system do judge this register
			input_report_key(ts->input_dev, BTN_TOUCH, finger);  // finger  num   0, 1, 2
            input_report_key(ts->input_dev, BTN_2, finger > 1); // finger2 state 0, 0, 1
           	input_sync(ts->input_dev);   // up load data  
			report = 1;  
		finger_count_3 = 0;
		variance_lock = 0;
			}

		 
		}
		else if(finger == 2)
		{
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 3);	
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, tp1.x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, tp1.y);
	    x1_old = tp1.x;
	    y1_old = tp1.y;
	        input_mt_sync(ts->input_dev);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 3);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_X, tp2.x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, tp2.y);
	    x2_old = tp2.x;
	    y2_old = tp2.y;
            input_mt_sync(ts->input_dev);
			input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, 0); // 0 touch_width,File system do judge this register
			input_report_key(ts->input_dev, BTN_TOUCH, finger);  // finger  num   0, 1, 2
            input_report_key(ts->input_dev, BTN_2, finger > 1); // finger2 state 0, 0, 1
            input_sync(ts->input_dev);   // up load data   
			report = 2;                       
	    finger_count_3 = 1;
		}

#ifdef SHOW_TOUCH_POINT			
	printk("1(X %d | Y %d) : 21(X %d | Y %d) : finger %d : report %d \n", tp1.x, tp1.y, tp2.x, tp2.y, finger, report);
#endif




#ifdef UPDATE_INT
		i2c_smbus_write_byte_data(ts->client, 0x3E, 0xFF);


		if (ts->use_irq)
			enable_irq(ts->client->irq);

	
#else
		if (finger)
	  	{

			hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);			//report rate 1/10000000=100Hz
		}
		else
		{
		// Clear all Interrupt
			hrtimer_cancel(&ts->timer);
#ifdef SHOW_TOUCH_POINT
	    printk(".\n");
#endif
	    if ((gesture & 0x04) && (!(gesture & 0x01)))
		BU21020ManualCalibration(ts->client);
	    i2c_smbus_write_byte_data(ts->client, 0x3E, 0xFF);
			if (ts->use_irq)
				enable_irq(ts->client->irq);
		}
#endif

	}
done:
	;

}

static enum hrtimer_restart rohm_ts_timer_func(struct hrtimer *timer)
{
	struct rohm_ts_data *ts = container_of(timer, struct rohm_ts_data, timer);
	queue_work(rohm_wq, &ts->work);
	//hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);			//report rate 1/12500000=80Hz
	return HRTIMER_NORESTART;
}

static irqreturn_t rohm_ts_irq_handler(int irq, void *dev_id)
{
	struct rohm_ts_data *ts = dev_id;
	//printk("Rohm_ts_irq_handler\n"); 
	disable_irq_nosync(ts->client->irq);
	queue_work(rohm_wq, &ts->work);
	//printk("Rohm_ts_irq_handler end\n");
	return IRQ_HANDLED;
}


static int rohm_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rohm_ts_data *ts;
	int ret = 0;
	struct ROHM_I2C_platform_data *pdata;
	int i;
	int offset = 0;
	int block;
	int bytes;
	int check_sum_file;
	int check_sum_room;
	int check_sum_room1;
	int check_sum_room2;
	int check_sum_room3;
	printk(KERN_ERR "ROHM BU21023 rohm_ts_probe!!\n");

	rohm_wq = create_singlethread_workqueue("rohm_wq");
	if (!rohm_wq)
	{
		//printk(KERN_ERR "BU21018 create_singlethread_workqueue ERROR!!\n");
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk(KERN_ERR "Rohm_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) 
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, rohm_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;
	if (ts->power) 
	{
		ret = ts->power(1);
		if (ret < 0) 
		{
			printk(KERN_ERR "Rohm_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}

	//end of default setting

    //Download 



    ////////////////////////////////////////////////////////////////////////
    // Init BU21023
    ////////////////////////////////////////////////////////////////////////
	// Wait 200usec for Reset
    udelay(200);
#ifdef TOUCH_SCREEN_SETTING
	//common setting 
	i2c_smbus_write_byte_data(ts->client, 0x30, 0x46);
	i2c_smbus_write_byte_data(ts->client, 0x31, 0x05);	//SWAP XY (from ROHM 0x19) Swap XP and Inv X & Y
	i2c_smbus_write_byte_data(ts->client, 0x32, 0x62);	//0x6e->enable INTVL  
	//timing setting
	i2c_smbus_write_byte_data(ts->client, 0x33, 0x00);	//0x00->sample period 20ms   
	i2c_smbus_write_byte_data(ts->client,  0x50, 0x00);
	i2c_smbus_write_byte_data(ts->client,  0x60, 0x04);
	i2c_smbus_write_byte_data(ts->client, 0x61, 0x06);
	i2c_smbus_write_byte_data(ts->client,  0x57, 0x04);
	//panel setting
	i2c_smbus_write_byte_data(ts->client, 0x63, 0x91);
	i2c_smbus_write_byte_data(ts->client, 0x64, 0xC2);
	i2c_smbus_write_byte_data(ts->client, 0x34, 0x5d);
	i2c_smbus_write_byte_data(ts->client, 0x35, 0xB6);
	i2c_smbus_write_byte_data(ts->client, 0x3A, 0x60);
	i2c_smbus_write_byte_data(ts->client, 0x3B, 0x01);
	i2c_smbus_write_byte_data(ts->client, 0x36, 0x0C);
	i2c_smbus_write_byte_data(ts->client, 0x37, 0x08);
	//fixed value setting
	i2c_smbus_write_byte_data(ts->client, 0x52, 0x08);
	i2c_smbus_write_byte_data(ts->client, 0x56, 0x04);
	i2c_smbus_write_byte_data(ts->client,  0x62, 0x0F);
	i2c_smbus_write_byte_data(ts->client, 0x65, 0x01);
#endif
	//analog power on
	i2c_smbus_write_byte_data(ts->client, 0x40, 0x01);
	// Wait 100usec for Power-on
	udelay(100);
	// Beginning address setting of program memory for host download
	i2c_smbus_write_byte_data(ts->client, 0x70, 0x00);
	i2c_smbus_write_byte_data(ts->client, 0x71, 0x00);

    // Download firmware to BU21020
    printk("BU21023 firmware download starts!\n");
    block = CODESIZE / 32;
    bytes = CODESIZE % 32;
	for(i = 0; i < block; i++)
	{
    	offset = i * 32;
		i2c_smbus_write_i2c_block_data(ts->client, 0x72, 32, &program[offset]);
    }

    offset = block * 32;
	for(i = 0; i < bytes; i++)
	{
        i2c_smbus_write_byte_data(ts->client, 0x72, program[offset + i]);
	}

    printk("Program INT 0x10, reg0x2A = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x2A));

    check_sum_room1 = i2c_smbus_read_byte_data(ts->client, 0x74);
    check_sum_room2 = i2c_smbus_read_byte_data(ts->client, 0x75);
    check_sum_room3 = i2c_smbus_read_byte_data(ts->client, 0x76);
    check_sum_room = (check_sum_room1 << 16) | (check_sum_room2 << 8) | check_sum_room3;

    check_sum_file = (program[8189] << 16) | (program[8190] << 8) | program[8191];
	
    if (check_sum_room == check_sum_file)
    {
	printk("BU21023 firmware download success.\n");

    }
    else
    {
	printk("BU21023 firmware download failed. \n");
	printk("checksum_room = %5d, checksum_file = %5d\n", check_sum_room, check_sum_file);
    }

    i2c_smbus_write_byte_data(ts->client, 0x3E, 0xFF);
    //Step4:  Coordinate Offset enable
    //i2c_smbus_write_byte_data(ts->client,  0x60, 0x04);
    //i2c_smbus_write_byte_data(ts->client,  0x3B, 0x01);
    //Step5:  CPU power on
	  i2c_smbus_write_byte_data(ts->client, 0x40, 0x03);
    for (i = 0; i < 30; i++)
    {
	udelay(1000);
    }

    //Set Blind Area 
    i2c_smbus_write_byte_data(ts->client, 0x60, 0x80);
    i2c_smbus_write_byte_data(ts->client, 0x3B, 0xC6);	//To set blind are of Y axis.
    i2c_smbus_write_byte_data(ts->client, 0x3B, 0x46);	//To set blind are of X axis.
	// Clear all Interrupt
 	  i2c_smbus_write_byte_data(ts->client, 0x3D, 0xFE);
    printk("Power on INT 0x08, reg0x2A = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x2A));
	  i2c_smbus_write_byte_data(ts->client, 0x3E, 0xFF);

    //Manual Calibration
    if (!BU21020ManualCalibration(ts->client))
	printk("BU21023 Manual Calibration success.\n");
    else
	printk("BU21023 Manual Calibration failed.\n");

for (i=0;i<15;i++)
	{udelay(1000);}

    printk("BU21023 reg0x30 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x30));
	printk("BU21023 reg0x31 = 0x%x\n",i2c_smbus_read_byte_data(ts->client, 0x31));
    printk("BU21023 reg0x32 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x32));
    printk("BU21023 reg0x3B = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x3B));
    printk("BU21023 reg0x3D = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x3D));
    printk("BU21023 reg0x3F = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x3F));
    printk("BU21023 reg0x65 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x65));
	printk("BU21023 reg0x34 = 0x%x\n",i2c_smbus_read_byte_data(ts->client, 0x34));
	printk("BU21023 reg0x35 = 0x%x\n",i2c_smbus_read_byte_data(ts->client, 0x35));
    printk("BU21023 reg0x38 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x38));
    printk("BU21023 reg0x39 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x39));
	printk("BU21023 reg0x3A = 0x%x\n",i2c_smbus_read_byte_data(ts->client, 0x3A));
    printk("BU21023 reg0x52 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x52));
    printk("BU21023 reg0x56 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x56));
    printk("BU21023 reg0x62 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x62));
    printk("BU21023 reg0x63 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x63));
    printk("BU21023 reg0x64 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x64));
    printk("BU21023 reg0x33 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x33));
    printk("BU21023 reg0x53 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x53));
    printk("BU21023 reg0x57 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x57));
    printk("BU21023 reg0x61 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x61));

    printk("\nBU21023 reg0x18 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x18));
	printk("BU21023 reg0x65 = 0x%x\n",i2c_smbus_read_byte_data(ts->client, 0x65));
	printk("BU21023 reg0x6B = 0x%x\n",i2c_smbus_read_byte_data(ts->client, 0x6B));
    printk("BU21023 reg0x6C = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x6C));

    i2c_smbus_write_byte_data(ts->client, 0x65, 0x31);

    printk("BU21023 reg0x65 = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x65));
    printk("BU21023 reg0x6B = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x6B));
    printk("BU21023 reg0x6C = 0x%x\n", i2c_smbus_read_byte_data(ts->client, 0x6C));
    i2c_smbus_write_byte_data(ts->client, 0x65, 0x21);
	// Clear all Interrupt
 	printk("BU21023 reg0x2A = 0x%x\n",i2c_smbus_read_byte_data(ts->client, 0x2A));
	i2c_smbus_write_byte_data(ts->client, 0x3E, 0xFF);
    // Init end
    ////////////////////////////////////////////////////////////////////////


//	ret = rohm_init_panel(ts); 
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) 
	{
		ret = -ENOMEM;
		printk(KERN_ERR "Rohm_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "Rohm-CTP-BU21023GUL";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(KEY_EXIT, ts->input_dev->keybit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	printk("Rohm_ts_probe: --------------------setting ready\n");

	////////////////////////////////////////////////////////////////////////////
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, ROHM_TS_ABS_X_MIN, ROHM_TS_ABS_X_MAX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, ROHM_TS_ABS_Y_MIN, ROHM_TS_ABS_Y_MAX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
//	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	////////////////////////////////////////////////////////////////////////////
	input_set_abs_params(ts->input_dev, ABS_X,ROHM_TS_ABS_X_MIN, ROHM_TS_ABS_X_MAX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y,ROHM_TS_ABS_Y_MIN, ROHM_TS_ABS_Y_MAX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 1550, 0, 0);  //1550  two point
    input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0); 	//15   two point
	input_set_abs_params(ts->input_dev, ABS_HAT0X, ROHM_TS_ABS_X_MIN, ROHM_TS_ABS_X_MAX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0Y, ROHM_TS_ABS_Y_MIN, ROHM_TS_ABS_Y_MAX, 0, 0);

	ret = input_register_device(ts->input_dev);
	if(ret) 
	{
		printk(KERN_ERR "==>: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	if(client->irq) 
	{
		//ret = request_irq(client->irq, rohm_ts_irq_handler, IRQF_TRIGGER_LOW, client->name, ts);  	
		ret = request_irq(client->irq, rohm_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);  	
//Trigger status used IRQF_TRIGGER_FALLING ; ref \linux\interrupt.h
// IRQF_TRIGGER_NONE	
// IRQF_TRIGGER_RISING	
// IRQF_TRIGGER_FALLING	
// IRQF_TRIGGER_HIGH
// IRQF_TRIGGER_LOW
// IRQF_TRIGGER_PROBE
/////////////////////////////

		printk("Request IRQ Failed==>ret : %d\n", ret);
		if (ret == 0) 
		{
			ts->use_irq = 1;	//1 : interrupt mode/0 : polling mode
			//free_irq(client->irq, ts);
		}
		else 
		{
			ts->use_irq = 0;	//1 set 1 : interrupt mode/0 : polling mode
			//dev_err(&client->dev, "request_irq failed\n");
		}	
	}
	if (!ts->use_irq) 
	{
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = rohm_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
		//timer init
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = rohm_ts_timer_func;


	printk(KERN_INFO "==>:Touchscreen Up %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	//err_detect_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int rohm_ts_remove(struct i2c_client *client)
{
	struct rohm_ts_data *ts = i2c_get_clientdata(client);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static const struct i2c_device_id rohm_ts_id[] = 
{
	{ ROHM_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver rohm_ts_driver = 
{
	.probe		= rohm_ts_probe,
	.remove		= rohm_ts_remove,
	.id_table	= rohm_ts_id,
	.driver = {
		.name	= ROHM_I2C_NAME,
	},
};

static int __devinit rohm_ts_init(void)
{
	//printk(KERN_ERR "ROHM BU21018 rohm_ts_init \n");
	return i2c_add_driver(&rohm_ts_driver);
}

static void __exit rohm_ts_exit(void)
{
	i2c_del_driver(&rohm_ts_driver);
	if (rohm_wq)
		destroy_workqueue(rohm_wq);
}

module_init(rohm_ts_init);
module_exit(rohm_ts_exit);

MODULE_DESCRIPTION("Rohm Touchscreen Driver");
MODULE_LICENSE("GPL");
