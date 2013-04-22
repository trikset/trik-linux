#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/pm.h>
#include <linux/module.h>

#include <linux/l3g42xxd.h>

/** Register map */
#define L3G4200D_WHO_AM_I               0x0f
#define L3G4200D_CTRL_REG1              0x20
#define L3G4200D_CTRL_REG2              0x21
#define L3G4200D_CTRL_REG3              0x22
#define L3G4200D_CTRL_REG4              0x23
#define L3G4200D_CTRL_REG5              0x24

#define L3G4200D_REF_DATA_CAP           0x25
#define L3G4200D_OUT_TEMP               0x26
#define L3G4200D_STATUS_REG             0x27

#define L3G4200D_OUT_X_L                0x28
#define L3G4200D_OUT_X_H                0x29
#define L3G4200D_OUT_Y_L                0x2a
#define L3G4200D_OUT_Y_H                0x2b
#define L3G4200D_OUT_Z_L                0x2c
#define L3G4200D_OUT_Z_H                0x2d

#define L3G4200D_FIFO_CTRL              0x2e
#define L3G4200D_FIFO_SRC               0x2e

#define L3G4200D_INTERRUPT_CFG          0x30
#define L3G4200D_INTERRUPT_SRC          0x31
#define L3G4200D_INTERRUPT_THRESH_X_H   0x32
#define L3G4200D_INTERRUPT_THRESH_X_L   0x33
#define L3G4200D_INTERRUPT_THRESH_Y_H   0x34
#define L3G4200D_INTERRUPT_THRESH_Y_L   0x35
#define L3G4200D_INTERRUPT_THRESH_Z_H   0x36
#define L3G4200D_INTERRUPT_THRESH_Z_L   0x37
#define L3G4200D_INTERRUPT_DURATION     0x38

#define PM_MASK                         0x08

#define I2C_RETRY_DELAY                 5
#define I2C_RETRIES                     5
#define AUTO_INCREMENT                  0x80

struct l3g42xxd_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
//	struct input_polled_dev *input_polled;
	struct platform_device *platform_device;
	struct work_struct irq_work;
	struct l3g42xxd_platform_data *pdata;
};

struct gyro_val {
        s16 x;
        s16 y;
        s16 z;
};

/*input parametrs 
 *
 *dps = 250;500;2000;
 *hz = 100;200;400;800;
 */
/**Function to work with device**/

//static int l3g42xxd_i2c_read (struct i2c_client* client,u8 reg){

//}


static const struct i2c_device_id l3g42xxd_id[] = {
        {"l3g42xxd", 0},
        {},
};

static int l3g42xxd_misc_open(struct inode *inode, struct file *file){
	pr_info("%s \n",__func__);
	return 0;
}
static int l3g42xxd_misc_release(struct inode *inode, struct file *file){
	pr_info("%s \n",__func__);
	return 0;
}
static long l3g42xxd_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
	pr_info("%s \n",__func__);
	return 0;
}
static const struct file_operations l3g42xxd_misc_fops = {
        .owner = THIS_MODULE,
        .open = l3g42xxd_misc_open,
	.release =l3g42xxd_misc_release,
        .unlocked_ioctl = l3g42xxd_misc_ioctl,
};

static struct miscdevice l3g42xxd_misc_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "l3g42xxd",
        .fops = &l3g42xxd_misc_fops,
};


static int l3g42xxd_i2c_read(struct l3g42xxd_data *gyro, u8 * buf, int len)
{
        int err;
        int tries = 0;
        struct i2c_msg msgs[] = {
                {
                        .addr = gyro->client->addr,
                        .flags = gyro->client->flags & I2C_M_TEN,
                        .len = 1,
                        .buf = buf,
                },
                {
                        .addr = gyro->client->addr,
                        .flags = (gyro->client->flags & I2C_M_TEN) | I2C_M_RD,
                        .len = len,
                        .buf = buf,
                },
        };

        do {
                err = i2c_transfer(gyro->client->adapter, msgs, 2);
                if (err != 2)
                        msleep_interruptible(I2C_RETRY_DELAY);
        } while ((err != 2) && (++tries < I2C_RETRIES));

        if (err != 2) {
                dev_err(&gyro->client->dev, "read transfer error\n");
                err = -EIO;
        } else {
                err = 0;
        }

        return err;
}
static int l3g42xxd_i2c_write(struct l3g42xxd_data *gyro, u8 * buf, int len)
{
        int err;
        int tries = 0;
        struct i2c_msg msgs[] = {
                {
                        .addr = gyro->client->addr,
                        .flags = gyro->client->flags & I2C_M_TEN,
                        .len = len + 1,
                        .buf = buf,
                },
        };

        do {
                err = i2c_transfer(gyro->client->adapter, msgs, 1);
                if (err != 1)
                        msleep_interruptible(I2C_RETRY_DELAY);
        } while ((err != 1) && (++tries < I2C_RETRIES));

        if (err != 1) {
                dev_err(&gyro->client->dev, "write transfer error\n");
                err = -EIO;
        } else {
                err = 0;
        }

        return err;
}

static int l3g42xxd_get_gyro_data(struct l3g42xxd_data *gyro,
                                         struct gyro_val *data)
{
 	u8 gyro_data[6];
        int err = -1;

//  	pr_warning("%s\n",__func__);
        /* Data bytes from hardware xL, xH, yL, yH, zL, zH */

        gyro_data[0] = (AUTO_INCREMENT | L3G4200D_OUT_X_L);
        err = l3g42xxd_i2c_read(gyro, gyro_data, 6);
        if (err < 0)
                return err;

        data->x = (gyro_data[1] << 8) | gyro_data[0];
        data->y = (gyro_data[3] << 8) | gyro_data[2];
        data->z = (gyro_data[5] << 8) | gyro_data[4];

	gyro_data[0] = L3G4200D_INTERRUPT_SRC;
	l3g42xxd_i2c_read(gyro, gyro_data, 1);

        return 0;
}
static void l3g42xxd_report_values(struct l3g42xxd_data *gyro,
                                         struct gyro_val *data)
{
        input_report_abs(gyro->input_dev, ABS_X, data->x);
        input_report_abs(gyro->input_dev, ABS_Y, data->y);
        input_report_abs(gyro->input_dev, ABS_Z, data->z);
        input_sync(gyro->input_dev);
}

static irqreturn_t l3g42xxd_irq_callback(int irq, void *dev_id){
	struct l3g42xxd_data* gyro = dev_id;
        if(gyro)
		schedule_work(&gyro->irq_work);
	return IRQ_HANDLED;
}
static void l3g42xxd_irq_worker(struct work_struct *work){
	struct l3g42xxd_data *gyro = container_of(work, struct l3g42xxd_data, irq_work);
	if (gyro->client){
		int err;
		struct gyro_val data;
		err = l3g42xxd_get_gyro_data(gyro, &data);
		if (err < 0)
			pr_err("get_acceleration_data failed");
		else
			 l3g42xxd_report_values(gyro, &data);
	}
	else{
		pr_warning("%s : i2c_client NULL\n ",__func__);
	}
}
static void l3g42xxd_input_dev_shutdown(struct l3g42xxd_data* gyro)
{
	input_unregister_device(gyro->input_dev);
	free_irq(gyro->pdata->gpio_drdy, gyro);
	input_free_device(gyro->input_dev);
}
static int l3g42xxd_enable(struct l3g42xxd_data* gyro)
{
}
static int l3g42xxd_disable(struct l3g42xxd_data* gyro)
{
}
static int l3g42xxd_init_chip(struct l3g42xxd_data* gyro){
	int err = -1;
        u8 buf[8];
	buf[0] = (AUTO_INCREMENT | L3G4200D_CTRL_REG1);
	buf[1] = 0b00000111;
	buf[2] = 0b00000000;
	buf[3] = 0b10000000;
	buf[4] = 0b00100000;
	buf[5] = 0b00000000;
	buf[6] = 0b00000000;
	err = l3g42xxd_i2c_write(gyro, buf, 6);
        if (err < 0)
                return err;
	buf[0] = (L3G4200D_FIFO_CTRL);
        buf[1] = 0b00000000;
        err = l3g42xxd_i2c_write(gyro, buf, 1);
        if (err < 0)
                return err;
 	buf[0] = (L3G4200D_INTERRUPT_CFG);
        buf[1] = 0b01111111;
        err = l3g42xxd_i2c_write(gyro, buf, 1);
        if (err < 0)
                return err;

	buf[0] = ( L3G4200D_CTRL_REG1);
        buf[1] = 0b00000111 | PM_MASK;
	err = l3g42xxd_i2c_write(gyro, buf, 1);
        if (err < 0)
                return err;

	return 0;
}
static int l3g42xxd_input_dev_init(struct l3g42xxd_data* gyro)
{
	int err;
	gyro->input_dev = input_allocate_device();
	if (!gyro->input_dev){
		err = -ENOMEM;
                dev_err(&gyro->client->dev, "input device allocate failed\n");
                goto exit_input_device_alloc;
	}
	gyro->input_dev->name = "l3g42xxd";
	gyro->input_dev->id.bustype = BUS_I2C;
	gyro->input_dev->dev.parent = &gyro->client->dev;
//	gyro->input_dev->open
//	gyro->input_dev->close 

//#define MPU3050_MIN_VALUE       -32768
//#define MPU3050_MAX_VALUE       32767

	set_bit(EV_ABS, gyro->input_dev->evbit);
	input_set_drvdata(gyro->input_dev, gyro);
	input_set_abs_params(gyro->input_dev, ABS_X,
                             -32768, 32767, 0, 0);
 	input_set_abs_params(gyro->input_dev, ABS_Y,
                             -32768, 32767, 0, 0);
 	input_set_abs_params(gyro->input_dev, ABS_Z,
                             -32768, 32767, 0, 0);

        pr_warning("%s, gyro %p\n", __func__, gyro);
	err = request_irq(gyro->pdata->gpio_drdy,
				      l3g42xxd_irq_callback,
		         (IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING),
					       "l3g42xxd_irq",
						     gyro);
	if (err != 0) {
                pr_err("%s: irq request failed: %d\n", __func__, err);
                err = -ENODEV;
                goto exit_irq_request;
        }
	err = input_register_device(gyro->input_dev);
        if (err) {
                dev_err(&gyro->client->dev,
                        "unable to register input device %s\n",
                        gyro->input_dev->name);
		err = -ENODEV;
                goto exit_register_device;
        }
	err = l3g42xxd_init_chip(gyro);
	if (err < 0){
		goto exit_init_chip;
	}

	{
		struct gyro_val data;
        	l3g42xxd_get_gyro_data(gyro, &data);
	}

	return 0;
exit_init_chip:
	input_unregister_device(gyro->input_dev);
exit_register_device:
	free_irq(gyro->pdata->gpio_drdy, gyro);
exit_irq_request:
	input_free_device(gyro->input_dev);
exit_input_device_alloc:
	return err;
}
static int l3g42xxd_probe(struct i2c_client *client,
                           const struct i2c_device_id *id)
{
	struct l3g42xxd_data *gyro;
	int err = -1;
        pr_err("%s:Enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
                dev_err(&client->dev, "client not i2c capable\n");
                err = -ENODEV;
                goto exit_no_i2c_capablility;
        }
	gyro = kzalloc(sizeof(*gyro), GFP_KERNEL);
        if (gyro == NULL) {
                dev_err(&client->dev,
                        "failed to allocate memory for module data\n");
                err = -ENOMEM;
                goto exit_no_enough_memory;
        }
	gyro->client = client;

	INIT_WORK(&gyro->irq_work, l3g42xxd_irq_worker);

	i2c_set_clientdata(client, gyro);
	pr_info("%s init input device\n",__func__);
	if (client->dev.platform_data == NULL){
		err = -ENODEV;
                goto exit_platform_data_null;
	}
	gyro->pdata = kzalloc(sizeof(*gyro->pdata), GFP_KERNEL);
        if (gyro->pdata == NULL){
		goto exit_malloc_platform_data;
	}

	memcpy(gyro->pdata, client->dev.platform_data, sizeof(*gyro->pdata));
	err = l3g42xxd_input_dev_init(gyro);
	if (err < 0){
		goto exit_input_dev_init; 
	}
	err = misc_register(&l3g42xxd_misc_device);
        if (err < 0) {
                dev_err(&client->dev, "l3g42xxd_device register failed\n");
                goto exit_register_misc_device;
        }
	return 0;
exit_register_misc_device:
	l3g42xxd_input_dev_shutdown(gyro);
exit_input_dev_init:
	kfree(gyro->pdata);
exit_malloc_platform_data:
exit_platform_data_null:
	kfree(gyro);
exit_no_enough_memory:
exit_no_i2c_capablility:
        return err;
}
static int __devexit l3g42xxd_remove(struct i2c_client *client)
{
	struct l3g42xxd_data *gyro = i2c_get_clientdata(client);
	misc_deregister(&l3g42xxd_misc_device);
 	l3g42xxd_input_dev_shutdown(gyro);
	flush_work_sync(&gyro->irq_work);
	kfree(gyro->pdata);
	kfree(gyro);
        return 0;
}

static int l3g42xxd_resume(struct i2c_client *client)
{
        return 0;
}

static int l3g42xxd_suspend(struct i2c_client *client, pm_message_t mesg)
{
        return 0;
}

MODULE_DEVICE_TABLE(i2c, l3g42xxd_id);

static struct i2c_driver l3g42xxd_driver = {
        .driver = {
                   .name = "l3g42xxd",
                   .owner = THIS_MODULE,
                   },
        .probe = l3g42xxd_probe,
	.suspend = l3g42xxd_suspend,
	.resume = l3g42xxd_resume,
        .remove = __exit_p(l3g42xxd_remove),
        .id_table = l3g42xxd_id,
};

static int __init l3g42xxd_init(void)
{
        pr_info("L3G4200D gyroscope driver start \n");
        return i2c_add_driver(&l3g42xxd_driver);
}

static void __exit l3g42xxd_exit(void)
{
        i2c_del_driver(&l3g42xxd_driver);
        return;
}


module_init(l3g42xxd_init);
module_exit(l3g42xxd_exit);

MODULE_DESCRIPTION("l3g4200d gyroscope driver");
MODULE_AUTHOR("CyberTech Roman Meshkevich");
MODULE_LICENSE("GPL");
