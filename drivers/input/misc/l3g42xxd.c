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


struct l3g42xxd_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
//	struct input_polled_dev *input_polled;
	struct platform_device* platform_device;
	struct work_struct irq_work;
	struct l3g4xxd_pdata* pdata;
};
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

static irqreturn_t l3g42xxd_irq_callback(int irq, void *dev_id){
	return IRQ_HANDLED;
}
static void l3g42xxd_irq_worker(struct work_struct *work){

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
	input_set_drvdata(gyro->input_dev, gyro);
	input_set_capability(gyro->input_dev, EV_REL, REL_RX);
        input_set_capability(gyro->input_dev, EV_REL, REL_RY);
        input_set_capability(gyro->input_dev, EV_REL, REL_RZ);
	gyro->input_dev->name = "gyro_l3g42xxd";

//	err = request_irq(gyro->pdata->gpio_drdy,/
//				      l3g42xxd_irq_callback,
//		         (IRQF_TRIGGER_FALLING),
//					       "l3g42xxd_irq", 
//						     gyro);
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
                goto exit_register_device;
        }
	return 0;
exit_register_device:
//	free_irq(gyro->pdata->gpio_drdy, gyro);
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
	
	i2c_set_clientdata(client, gyro);
	
	pr_info("%s init input device\n",__func__);
	if (client->dev.platform_data == NULL){
		
	}

	err = misc_register(&l3g42xxd_misc_device);
        if (err < 0) {
                dev_err(&client->dev, "l3g42xxd_device register failed\n");
                goto exit_register_misc_device;
        }
exit_register_misc_device:
	 kfree(gyro);
exit_no_enough_memory:
exit_no_i2c_capablility:
        return err;
}
static int __devexit l3g42xxd_remove(struct i2c_client *client)
{
	struct l3g42xxd_data *gyro = i2c_get_clientdata(client);
	misc_deregister(&l3g42xxd_misc_device);
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
