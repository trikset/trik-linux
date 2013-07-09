#include <linux/input.h>	/* BUS_SPI */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/types.h>
#include "l3g42xxd.h"
static int l3g42xxd_i2c_suspend(struct device *dev)
{
	return 0;
}

static int l3g42xxd_i2c_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(l3g42xxd_i2c_pm, l3g42xxd_i2c_suspend, l3g42xxd_i2c_resume);


static int l3g42xxd_i2c_write(struct device *dev,unsigned char reg, unsigned char data)
{
	struct i2c_client *client = to_i2c_client(dev);
	
	return i2c_smbus_write_byte_data(client, reg, data);
}

static int l3g42xxd_i2c_read(struct device *dev,unsigned char reg)
{
	struct i2c_client *client = to_i2c_client(dev);

	return i2c_smbus_read_byte_data(client, reg);
}
static  int l3g42xxd_i2c_read_block (struct device * dev,unsigned char reg,   unsigned char  count, void *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return i2c_smbus_read_i2c_block_data(client, reg, count, buf);
}

static int __devexit l3g42xxd_i2c_remove(struct i2c_client *client)
{
	struct l3g42xxd_chip *chip = i2c_get_clientdata(client);
	
	l3g42xxd_remove(chip);
	i2c_set_clientdata(client, NULL);
	return 0;
}
static int __devinit l3g42xxd_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct l3g42xxd_chip* chip;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		pr_err("%s :client not i2c capable\n",__func__);
		return -ENODEV;
	}
	ret = l3g42xxd_probe(&client->dev,BUS_I2C,client->irq,l3g42xxd_i2c_read,l3g42xxd_i2c_write,l3g42xxd_i2c_read_block,&chip);
	if (ret){
		pr_err("%s: l3g42xxd_probe failed\n",__func__);
		return ret;
	}
	i2c_set_clientdata(client, chip);

	return 0;
}
static const struct i2c_device_id l3g42xxd_id[] = {
	{"l3g42xxd",0 },
};
static struct i2c_driver l3g42xxd_i2c_driver = {
	.driver = {
		.name = "l3g42xxd",
		.pm   = &l3g42xxd_i2c_pm,
	},
	.probe    = l3g42xxd_i2c_probe,
	.remove   = __devexit_p(l3g42xxd_i2c_remove),
	.id_table = l3g42xxd_id,
};

module_i2c_driver(l3g42xxd_i2c_driver);

MODULE_AUTHOR("Roman Meshkevich <romik.momik@trikset.com>");
MODULE_DESCRIPTION("L3gd20 Three-Axis Digital Accelerometer I2C Bus Driver");
MODULE_LICENSE("GPL");

