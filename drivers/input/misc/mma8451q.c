#include <linux/errno.h>
#include <linux/input.h>	/* BUS_SPI */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/interrupt.h>

#include "linux/mma8451q.h"

// struct mma8451q_platform_data
// {
// 	int gpio_int2;
// };
enum {
	MODE_ODR_800HZ = 0,
	MODE_ODR_400HZ,
	MODE_ODR_200HZ,
	MODE_ODR_100HZ,
	MODE_ODR_50HZ,
	MODE_ODR_12HZ,
	MODE_ODR_6HZ,
	MODE_ODR_2HZ,

};
enum {
	MODE_2G = 0,
	MODE_4G,
	MODE_8G,
};

enum {
	MMA_STANDBY = 0,
	MMA_ACTIVED,
};

struct mma8451q_accel_data
{
	short x;
	short y;
	short z;
};
struct mma8451q_driver_data
{
	struct i2c_client * client;
	struct input_dev* input_dev;
	//struct mma8451q_platform_data* pdata;
	u8 client_irq;
	struct work_struct irq_work;
	bool enabled;
};

static short mma8451q_read(struct mma8451q_driver_data* drv_data,u8 reg){
	return i2c_smbus_read_byte_data(drv_data->client, reg);
}
static short mma8451q_write(struct mma8451q_driver_data* drv_data,u8 reg,u8 value){
	return i2c_smbus_write_byte_data(drv_data->client, reg, value);
}
static short mma8451q_read_block(struct mma8451q_driver_data* drv_data,u8 reg,void * buf,size_t size)
{
	return i2c_smbus_read_i2c_block_data(drv_data->client, reg, size, buf);
}

static int  mma8451q_suspend(struct device *dev)
{
	return 0;
}
static int  mma8451q_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(mma8451q_pm, mma8451q_suspend, mma8451q_resume);

static irqreturn_t mma8451q_irq_callback(int irq, void *dev_id){
	struct mma8451q_driver_data* chip = dev_id;
	pr_err("%s: irq get\n",__func__);
    if(chip)
		schedule_work(&chip->irq_work);
	return IRQ_HANDLED;
}
static int mma8451q_get_values(struct mma8451q_driver_data* chip, struct mma8451q_accel_data *data)
{
	//int accel_data[6];
	//mma8451q_read_block(chip,OUT_X_MSB,accel_data,6);

	return 0;	
}
static void mma8451q_irq_worker(struct work_struct *work)
{
	int value;
	struct mma8451q_driver_data *chip = container_of(work, struct mma8451q_driver_data, irq_work);
	pr_err("%s: irq get\n",__func__);
	//struct mma8451q_accel_data data;
	// mma8451q_get_values(chip,&data);
	// input_report_abs(chip->input_dev, ABS_X, data.x);
	// input_report_abs(chip->input_dev, ABS_Y, data.y);
	// input_report_abs(chip->input_dev, ABS_Z, data.z);
	// input_sync(chip->input_dev);
	{
		short x,y,z;
		u8 accel_data[7];
		pr_err("%s: res %d\n",__func__,mma8451q_read_block(chip,OUT_X_MSB,accel_data,7));

		x = ((accel_data[0] << 8) & 0xff00) | accel_data[1];
		y = ((accel_data[2] << 8) & 0xff00) | accel_data[3];
		z = ((accel_data[4] << 8) & 0xff00) | accel_data[5];
		x = (short)(x) >> 2;
		y = (short)(y) >> 2;
		z = (short)(z) >> 2;
		input_report_abs(chip->input_dev, ABS_X, x);
		input_report_abs(chip->input_dev, ABS_Y, y);
		input_report_abs(chip->input_dev, ABS_Z, z);
		input_sync(chip->input_dev);
	}
	// 	pr_err("%s: F_STATUS 0x%02x\n",__func__,mma8451q_read(chip,F_STATUS));
	// 	pr_err("%s: INT_SOURCE 0x%02x\n",__func__,mma8451q_read(chip,INT_SOURCE));
	// 	pr_err("%s: res %d\n",__func__,mma8451q_read(chip,OUT_X_MSB));
	// 	pr_err("%s: F_STATUS 0x%02x\n",__func__,mma8451q_read(chip,F_STATUS));
	// 	pr_err("%s: res %d\n",__func__,mma8451q_read(chip,OUT_X_LSB));
	// 	pr_err("%s: F_STATUS 0x%02x\n",__func__,mma8451q_read(chip,F_STATUS));
	// 	pr_err("%s: res %d\n",__func__,mma8451q_read(chip,OUT_Y_MSB));
	// 	pr_err("%s: F_STATUS 0x%02x\n",__func__,mma8451q_read(chip,F_STATUS));
	// 	pr_err("%s: res %d\n",__func__,mma8451q_read(chip,OUT_Y_LSB));
	// 	pr_err("%s: F_STATUS 0x%02x\n",__func__,mma8451q_read(chip,F_STATUS));
	// 	pr_err("%s: res %d\n",__func__,mma8451q_read(chip,OUT_Z_MSB));
	// 	pr_err("%s: F_STATUS 0x%02x\n",__func__,mma8451q_read(chip,F_STATUS));
	// 	pr_err("%s: res %d\n",__func__,mma8451q_read(chip,OUT_Z_LSB));
	// //mma8451q_read_block(chip,OUT_X_MSB,accel_data,6);
	
}

static void mma8451q_input_dev_remove(struct mma8451q_driver_data* chip)
{
	input_unregister_device(chip->input_dev);
	free_irq(chip->client->irq, chip);
	input_free_device(chip->input_dev);
}
static int mma8451q_input_open(struct input_dev *dev)
{

    return 0;
}

static void mma8451q_input_close(struct input_dev *dev)
{

}

static int mma8451q_init_chip(struct mma8451q_driver_data* chip)
{
	chip->enabled = false;
	mma8451q_write(chip,CTRL_REG1,0x00);
	mma8451q_write(chip,CTRL_REG1,0b00011000);

	mma8451q_write(chip,CTRL_REG2,0b00010000);
	mma8451q_write(chip,CTRL_REG3,0b00000000);
	mma8451q_write(chip,CTRL_REG4,0b00000001);
	mma8451q_write(chip,CTRL_REG5,0b00000000);

	mma8451q_write(chip,CTRL_REG1,0b00000001);
	// mma8451q_read(chip,INT_SOURCE);
	// mma8451q_read(chip,F_STATUS);

	return 0;
}
static int mma8451q_input_dev_init(struct mma8451q_driver_data* chip)
{
	int res;
	u16 device_id = mma8451q_read(chip,WHO_AM_I);
	pr_info("%s: Device Id = 0x%02x\n",__func__,device_id);
	if (device_id != MMA8451_ID){
		pr_err("%s: Unknown device = 0x%02x\n",__func__,device_id);
		res = -ENODEV;
		goto exit_unknown_device;
	}
	chip->input_dev = input_allocate_device();
	if (!chip->input_dev)
	{
		res = -ENOMEM;
		pr_err("%s:input device allocate failed\n",__func__);
		goto exit_input_device_alloc;
	}
	chip->input_dev->name = "mma8451q";
	chip->input_dev->id.bustype = BUS_I2C;
	chip->input_dev->id.version = 1;
	chip->input_dev->id.product = le16_to_cpu(device_id);
	chip->input_dev->id.vendor = 123;

	chip->input_dev->dev.parent = &chip->client->dev;
	chip->input_dev->open = mma8451q_input_open;
	chip->input_dev->close  = mma8451q_input_close;

	set_bit(EV_ABS,chip->input_dev->evbit);
	set_bit(EV_REP,chip->input_dev->evbit);
	input_set_drvdata(chip->input_dev, chip);
	input_set_abs_params(chip->input_dev, ABS_X,
                             -8192, 8191, 0, 0);
 	input_set_abs_params(chip->input_dev, ABS_Y,
                             -8192, 8191, 0, 0);
 	input_set_abs_params(chip->input_dev, ABS_Z,
                             -8192, 8191, 0, 0);

	res = request_irq(chip->client->irq,
				     mma8451q_irq_callback,
                    (IRQF_TRIGGER_FALLING),
					       "mma8451q_irq",
						     chip);

	if (res != 0) {
        pr_err("%s: irq request failed: %d\n", __func__, res);
        goto exit_irq_request;
    }
    res = input_register_device(chip->input_dev);
    if (res) {
        pr_err("%s:unable to register input device %s\n",__func__,chip->input_dev->name);
        goto exit_register_device;
    }
    res = mma8451q_init_chip(chip);
    if (res){
    	pr_err("%s: failed to init chip \n",__func__);
    	goto exit_init_chip;
    }
	return 0;
exit_init_chip:
	input_free_device(chip->input_dev);
exit_register_device:
	free_irq(chip->client->irq, chip);
exit_irq_request:
	input_free_device(chip->input_dev);
exit_input_device_alloc:
exit_unknown_device:
	return res;
}

static int __devexit  mma8451q_remove(struct i2c_client *client)
{
	struct mma8451q_driver_data * drv_data =  i2c_get_clientdata(client);
	mma8451q_input_dev_remove(drv_data);
	flush_work_sync(&drv_data->irq_work);
	kfree(drv_data);

	return 0;
}
static int __devinit  mma8451q_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int res;
	struct mma8451q_driver_data * drv_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		pr_err("%s :client not i2c capable\n",__func__);
		res =  -ENODEV;
		goto exit_nothing_to_free;
	}
	if (client->irq <= 0){
		pr_err("%s: IRQ(%d) not configured\n",__func__,client->irq);
		res =  -EINVAL;
		goto exit_nothing_to_free;
	}
	drv_data = kzalloc(sizeof(struct mma8451q_driver_data),GFP_KERNEL);
	if (!drv_data) {
		res = -ENOMEM;
		goto exit_alloc_drv_data;
	}
	drv_data->client = client;
	i2c_set_clientdata(client, drv_data);
	INIT_WORK(&drv_data->irq_work, mma8451q_irq_worker);

	res = mma8451q_input_dev_init(drv_data);
	if (res){
        pr_err("%s:failed to init input device \n",__func__);
        goto exit_input_dev_init; 
    }

	return 0;
exit_input_dev_init:
	kfree(drv_data);
exit_alloc_drv_data:
exit_nothing_to_free:
	return res;
}

static const struct i2c_device_id mma8451q_id[] = {
	{"mma8451q",0x1c },
	{"mma845xq",0 },
};
static struct i2c_driver mma8451q_driver = {
	.driver = {
		.name = "mma8451q",
		.pm   = &mma8451q_pm,
	},
	.probe    = mma8451q_probe,
	.remove   = __devexit_p(mma8451q_remove),
	.id_table = mma8451q_id,
};

module_i2c_driver(mma8451q_driver);

MODULE_AUTHOR("Roman Meshkevich <romik.momik@trikset.com>");
MODULE_DESCRIPTION("mma8451q Three-Axis Digital Accelerometer I2C Bus Driver");
MODULE_LICENSE("GPL");
