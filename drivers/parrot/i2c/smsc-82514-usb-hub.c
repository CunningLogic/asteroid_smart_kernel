/**
********************************************************************************
* @file smsc-82514-usb-hub.c
* @brief SMSC USB82514 Automotive Grade USB 2.0 Hi-Speed 4-Port Hub
*
* Copyright (C) 2012 Parrot S.A.
*
* @author     Christian ROSALIE <christian.rosalie@parrot.com>
* @date       2012-08-13
********************************************************************************
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <plat/gpio.h>

#include "smsc-82514-usb-hub.h"


#define SMSC_USB82514_DRV_NAME		"smsc82514"
#define SMSC_USB82514_DRV_VERSION	0x1

/* Registers */
#define SMSC82514_VENDOR_ID_LSB			0x0	/* Vendor ID (LSB) */
#define SMSC82514_VENDOR_ID_MSB			0x1	/* Vendor ID (MSB) */
#define SMSC82514_PRODUCT_ID_LSB		0x2	/* Product ID (LSB) */
#define SMSC82514_PRODUCT_ID_MSB		0x3	/* Product ID (MSB) */

#define SMSC82514_DEVICE_ID_LSB			0x4	/* Device ID (LSB) */
#define SMSC82514_DEVICE_ID_MSB			0x5	/* Device ID (MSB) */

#define SMSC82514_CFG_DATA_BYTE1		0x6	/* Configuration Data Byte 1 */
#define SMSC82514_CFG_DATA_BYTE2		0x7	/* Configuration Data Byte 2 */
#define SMSC82514_CFG_DATA_BYTE3		0x8	/* Configuration Data Byte 3 */

#define SMSC82514_N_REMOV_DEV			0x9	/* Non-Removable Device */
#define SMSC82514_PORT_DIS_SELF_PWDED		0xa	/* Port Disable for Self-Powered Operation */
#define SMSC82514_PORT_DIS_BUS_PWDED		0xb	/* Port Disable for Bus-Powered Operation */

#define SMSC82514_MAX_PWR_SELF_PWDED		0x0c	/* Max Power for Self-Powered Operation */
#define SMSC82514_MAX_PWR_BUS_PWDED		0x0d	/* Max Power for Bus-Powered Operation */
#define SMSC82514_HUB_CTRL_MAX_CUR_SELF_PWDED	0x0E	/* Hub Controller Max Current for Self-Powered Operation */
#define SMSC82514_HUB_CTRL_MAX_CUR_BUS_PWDED	0x0f	/* Hub Controller Max Current for Bus-Powered Operation */
#define SMSC82514_POWER_ON_TIME			0x10	/* Power-on Time 32h */

#define SMSC82514_LANGUAGE_ID_HIGH		0x11	/* Language ID High */
#define SMSC82514_LANGUAGE_ID_LOW		0x12	/* Language ID Low */
#define SMSC82514_MANUFACTURER_STR_LENGTH	0x13
#define SMSC82514_PRODUCT_STR_LENGTH		0x14
#define SMSC82514_SERIAL_STR_LENGTH		0x15

#define SMSC82514_MANUFACTURER_STR		0x16	/*	16h-53h Manufacturer String 00h	*/
#define SMSC82514_PRODUCT_STR			0x54	/*	54h-91h Product String 00h	*/
#define SMSC82514_SERIAL_STR			0x92	/*	92h-Cfh Serial String 00h	*/

#define SMSC82514_BOOST_UP			0xf6
#define SMSC82514_BOOST_4_0			0xf8

/* Reserved addresses
	0xf7		Reserved
	0xd0-0xf5	Reserved
	0xf9		Reserved
	0xfd		Reserved
 */

#define SMSC82514_RESERVED1			0xf9
#define SMSC82514_RESERVED2			0xfd

#define SMSC82514_PORTSWAP			0xfa
#define SMSC82514_PORTMAP_12			0xfb
#define SMSC82514_PORTMAP_34			0xfc
#define SMSC82514_STATUS_COMMAND		0xff

struct smsc82514_data {
	struct i2c_client *client;
	struct smsc82514_pdata ds_data;
};

static u8 smsc82514_init_seq[][2] = {
	{SMSC82514_VENDOR_ID_LSB,		0x24},
	{SMSC82514_VENDOR_ID_MSB,		0x04},
	{SMSC82514_PRODUCT_ID_LSB,		0x14},
	{SMSC82514_PRODUCT_ID_MSB,		0x25},
	{SMSC82514_DEVICE_ID_LSB,		0xA0},
	{SMSC82514_DEVICE_ID_MSB,		0x80},
	{SMSC82514_CFG_DATA_BYTE1,		0x9B},
	{SMSC82514_CFG_DATA_BYTE2,		0x20},
	{SMSC82514_CFG_DATA_BYTE3,		0x02},
	{SMSC82514_N_REMOV_DEV,			0x00},
	{SMSC82514_PORT_DIS_SELF_PWDED,		0x00},
	{SMSC82514_PORT_DIS_BUS_PWDED,		0x00},
	{SMSC82514_MAX_PWR_SELF_PWDED,		0x01},
	{SMSC82514_MAX_PWR_BUS_PWDED,		0x32},
	{SMSC82514_HUB_CTRL_MAX_CUR_SELF_PWDED,	0x01},
	{SMSC82514_HUB_CTRL_MAX_CUR_BUS_PWDED,	0x32},
	{SMSC82514_POWER_ON_TIME,		0x32},

};

#define SMSC82514_INIT_SEQ_SIZE  ARRAY_SIZE(smsc82514_init_seq)

static int smsc82514_init_client(struct i2c_client *client)
{
	int i;
	struct i2c_msg xfer[2];
	u8 dout[3];
	struct smsc82514_data *data = i2c_get_clientdata(client);

	//reset hub
	if( data->ds_data.reset_pin ){
		gpio_set_value(data->ds_data.reset_pin, 0);

		mdelay(20);

		gpio_set_value(data->ds_data.reset_pin, 1);
	}

	mdelay(10);

	xfer[0].addr	= client->addr;
	xfer[0].flags	= 0;
	xfer[0].len	= 3;
	xfer[0].buf	= dout;

	dout[1]		= 1;

	for( i = 0 ; i < SMSC82514_INIT_SEQ_SIZE ; i++ ){

		dout[0] = smsc82514_init_seq[i][0];
		dout[2] = smsc82514_init_seq[i][1];

		if (i2c_transfer(client->adapter, xfer, 1) < 0) {
			dev_err(&client->dev, "%s: i2c transfer failed\n\
					Unable to write (0x%x) register\n",
					 __func__, smsc82514_init_seq[i][0] );
			return -EIO;
		}

		mdelay(1);
	}

	// Apply USB husb boost
	//Boost usb hub port if necessary
	dout[0] = SMSC82514_BOOST_4_0;
	dout[2] = data->ds_data.ds_port_1
			| (data->ds_data.ds_port_2 << 2)
			| (data->ds_data.ds_port_3 << 4)
			| (data->ds_data.ds_port_4 << 6);

	if (i2c_transfer(client->adapter, xfer, 1) < 0) {
		dev_err(&client->dev, " i2c transfer failed\n\
				Unable apply hub boost (value == 0x%x)\n",
				dout[2]);
		return -EIO;
	}

	mdelay(1);

	//Start hub
	//Once started, registers are protected
	//The hub must be reset to update configuration
	dout[0] = SMSC82514_STATUS_COMMAND;
	dout[2] = 0x01;

	if (i2c_transfer(client->adapter, xfer, 1) < 0) {
		dev_err(&client->dev, " i2c transfer failed\n\
				Unable to start usb hub\n");
		return -EIO;
	}

	mdelay(1);

	return 0;
}

static int smsc82514_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct smsc82514_data *data = NULL;
	struct smsc82514_pdata *pds_data;
	int ret = 0;

	pr_err(SMSC_USB82514_DRV_NAME ": version %d\n", SMSC_USB82514_DRV_VERSION );

	data = kzalloc(sizeof(struct smsc82514_data), GFP_KERNEL);
	if (!data) {
		pr_err(SMSC_USB82514_DRV_NAME ": Memory allocation failed\n" );
		ret = -ENOMEM;
		goto exit;
	}

	if( client->dev.platform_data ){

		pds_data = (struct smsc82514_pdata *) client->dev.platform_data;

		if( pds_data ) /* copy drive stregth settings */
			memcpy(&data->ds_data, pds_data, sizeof(struct smsc82514_pdata));
		else /* init with the default value */
			memset(&data->ds_data, 0x0, sizeof(struct smsc82514_pdata));
	}

	data->client = client;
	i2c_set_clientdata(client, data);

	/* Initialize chip */
	ret = smsc82514_init_client(client);

exit:
	return ret;
}

static int __devexit smsc82514_remove(struct i2c_client *client)
{
	struct smsc82514_data *data = i2c_get_clientdata(client);

	kfree(data);
	return 0;
}


static const struct i2c_device_id smsc82514_i2c_table[] = {
	{"smsc82514", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, smsc82514_i2c_table);

static struct i2c_driver smsc82514_i2c_driver = {
	.driver = {
	.name  = SMSC_USB82514_DRV_NAME,
	.owner = THIS_MODULE,
	},
	.probe		= smsc82514_probe,
	.remove		= __devexit_p(smsc82514_remove),
	.id_table	= smsc82514_i2c_table,
};


static int __init smsc82514_modinit(void)
{
	int ret = 0;

	pr_err(SMSC_USB82514_DRV_NAME ": module init \n");
	ret = i2c_add_driver(&smsc82514_i2c_driver);
	if (ret != 0) {
	pr_err("Failed to register SMSC USB82514 driver: %d\n", ret);
	}

	return ret;
}

static void __exit smsc82514_modexit(void)
{
	pr_err(SMSC_USB82514_DRV_NAME ": module exit \n");
	i2c_del_driver(&smsc82514_i2c_driver);
}

module_init(smsc82514_modinit);
module_exit(smsc82514_modexit);


MODULE_AUTHOR("PARROT SA by Christian ROSALIE <christian.rosalie@parrot.com>");
MODULE_DESCRIPTION("SMSC USB82514 USB 2.0 Hub");
MODULE_LICENSE("GPL");
