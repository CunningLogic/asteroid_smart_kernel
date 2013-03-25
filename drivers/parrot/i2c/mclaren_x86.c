#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>

#include "ti_ub925_lvds.h"

#define I2C_IRQ_STATUS 1
#define I2C_BUT_STATUS 2

#define DRIVER_NAME "mclaren_x86"
#define DRIVER_VERSION 0x1

#if 0
/* for test without lvds */
#define lvds_request_irq(...) ENOMEM
#define lvds_free_irq(...)
#endif

struct mclaren_x86_key {
	int keycode;
	int bit;
	int rot;
};

/**
  AISCP_VIRTUALPANEL_KEY_APP_SWITCH,
  AISCP_VIRTUALPANEL_KEY_VOLUME_DOWN,
  AISCP_VIRTUALPANEL_KEY_VOLUME_UP,
  AISCP_VIRTUALPANEL_KEY_BACK,
  AISCP_VIRTUALPANEL_KEY_HOME,
  AISCP_VIRTUALPANEL_KEY_MENU,
  AISCP_VIRTUALPANEL_KEY_MUTE,
  AISCP_VIRTUALPANEL_KEY_COUNT
 */
struct mclaren_x86_key default_keys [8] = {
	{1, 0x01, 0},
	{2, 0x02, 1},
	{3, 0x04, 1},
	{4, 0x08, 0},
	{5, 0x10, 0},
	{6, 0x20, 0},
	{7, 0x40, 0},
	{0,0, 0},
};

struct mclaren_x86_data {
	struct i2c_client *client;
	struct input_dev  *input;
	struct mclaren_x86_key *key;
};


/* func */
static int __mclaren_x86_read_reg(struct i2c_client *client,
			       u8 reg)
{
	struct i2c_msg xfer[2];
	u8 buf[1];

	buf[0] = reg & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = buf;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		return -EIO;
	}

	return buf[0];
}

#if 0
static int __mclaren_x86_read_write(struct i2c_client *client,
			       u8 reg, u8 val)
{
	char buff[2] = {reg, val};
	if (i2c_master_send(client, buff, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		return -EIO;
	}
	return 0;
}
#endif

static int mclaren_x86_initialize(struct mclaren_x86_data *data)
{
	struct i2c_client *client = data->client;

	if (__mclaren_x86_read_reg(client, 0x03) != 0x86)
		return -ENODEV;

	// release irq
	__mclaren_x86_read_reg(client, I2C_BUT_STATUS);

	return 0;
}

static irqreturn_t lvds_interrupt(int irq, void *dev_id)
{
	struct mclaren_x86_data *data = dev_id;
	struct i2c_client *client = data->client;
	int status = __mclaren_x86_read_reg(client, I2C_IRQ_STATUS);
	if (status > 0) {
		int i;
		status = __mclaren_x86_read_reg(client, I2C_BUT_STATUS);
		for (i = 0; i < 8; i++) {
			if (data->key[i].keycode)
				input_report_key(data->input, data->key[i].keycode, status & data->key[i].bit);
			if (data->key[i].rot)
				input_report_key(data->input, data->key[i].keycode, 0);
		}
		input_sync(data->input);
		return IRQ_HANDLED;
	}
	else
		return IRQ_NONE;

}

static int __devinit mclaren_x86_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct mclaren_x86_platform_data *pdata = client->dev.platform_data;
	struct mclaren_x86_data *data;
	struct input_dev *input_dev;
	int error;
	int i;

	printk(KERN_INFO"starting mclaren_x86\n");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		error = -ENOMEM;
		goto err_no_mem;
	}

	data->client = client;
	data->key = default_keys;


	error = mclaren_x86_initialize(data);
	if (error)
		goto err_free_object;

	input_dev = input_allocate_device();
	if (!input_dev) {
		error = -ENOMEM;
		goto err_free_object;
	}

	/* input layer init */
	data->input = input_dev;
	input_dev->name = DRIVER_NAME;
	input_dev->phys = DRIVER_NAME"/input0";

	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = DRIVER_VERSION;
	input_dev->evbit[0] = BIT(EV_KEY);
	input_dev->evbit[0] |= BIT(EV_REP);

	for (i = 0; i < 8; i++) {
		if (data->key[i].keycode) {
			set_bit(data->key[i].keycode, input_dev->keybit);
		}
	}

	error = input_register_device(data->input);
	if (error)
		goto err_free_input;

	if (client->irq >= 0)
		error = request_threaded_irq(client->irq, NULL, lvds_interrupt,
				IRQF_TRIGGER_FALLING, client->dev.driver->name, data);
	else
		error = lvds_request_irq(client->irq, lvds_interrupt,
				IRQF_TRIGGER_FALLING, client->dev.driver->name, data);

	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_unreg_input;
	}


	return 0;

err_unreg_input:
	input_unregister_device(data->input);
err_free_input:
	input_free_device(data->input);
err_free_object:
	kfree(data);

err_no_mem:
	return error;
}

static int __devexit mclaren_x86_remove(struct i2c_client *client)
{
	struct mclaren_x86_data *data = i2c_get_clientdata(client);

	if (client->irq >= 0)
		free_irq(client->irq, data);
	else
		lvds_free_irq(client->irq, data);

	input_unregister_device(data->input);
	input_free_device(data->input);

	kfree(data);

	return 0;
}

static const struct i2c_device_id mclaren_x86_id[] = {
	{ "mclaren_x86", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mclaren_x86_id);

static struct i2c_driver mclaren_x86_driver = {
	.driver = {
		.name	= "mclaren_x86",
		.owner	= THIS_MODULE,
	},
	.probe		= mclaren_x86_probe,
	.remove		= __devexit_p(mclaren_x86_remove),
	.id_table	= mclaren_x86_id,
};

static int __init mclaren_x86_init(void)
{
	return i2c_add_driver(&mclaren_x86_driver);
}

static void __exit mclaren_x86_exit(void)
{
	i2c_del_driver(&mclaren_x86_driver);
}

module_init(mclaren_x86_init);
module_exit(mclaren_x86_exit);
