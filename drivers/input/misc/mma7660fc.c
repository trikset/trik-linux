/*
 *   MMA7660FC Accelerometer driver
 *
 *   Copyright (c) by Jean-Christophe Rona <rona@archos.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/mma7660fc.h>
// I2C Read & Write 

struct mma7660fc_data{	
	int irqs;
	int opencount;
	int sample_rate;
	int sleep_count;
	int amsr;
	int awsr;
	struct mma7660fc_pdata *pdata;
	struct work_struct irq_work;
	struct i2c_client * client;
	struct input_dev *input_dev;
};

static inline int i2c_write(struct i2c_client *client, int reg, int value)
{
        int ret;
        ret = i2c_smbus_write_byte_data(client, reg, value);
        printk("mma7660fc_write: Reg = 0x%02X, Value = 0x%02X, Ret = %d\n", reg, value, ret);
        return ret;
};

static inline int i2c_read(struct i2c_client *client, int reg)
{
        int value;
        value = i2c_smbus_read_byte_data(client, reg);;
        printk("mma7660fc_read: Reg = 0x%02X, Value = 0x%02X\n", reg, value);
        return value;
};

static int mma7660fc_interrupt_setup(struct i2c_client* client,short events)
{
        int ret = 0;
      u8 data;

      data = i2c_read(client, REG_INTSU) & ~MMA7660FC_INT_SETUP_MASK;
        ret = i2c_write(client, REG_INTSU, data |
                               (events & MMA7660FC_INT_SETUP_MASK));
        return ret;
}
static int mma7660fc_set_mode(struct i2c_client* client,short mode)
{
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_MASK;
        ret = i2c_write(client, REG_MODE, data |
                                (mode & MMA7660FC_MODE_MASK));
        return ret;
}
static int mma7660fc_set_pulse_debounce(struct i2c_client* client,short samples)
{
	int ret = 0;
	u8 data;
	data = i2c_read(client, REG_PDET) & ~MMA7660FC_PULSE_DEBOUNCE_MASK;
	ret = i2c_write(client, REG_PDET, data |
                                ((samples - 1)& MMA7660FC_PULSE_DEBOUNCE_MASK));
	return ret;
}
static int mma7660fc_set_pulse_threshold(struct i2c_client* client,short trshld)
{
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_PD) & ~MMA7660FC_PULSE_THRESHOLD_MASK;
        ret = i2c_write(client, REG_PD, data |
                                (trshld & MMA7660FC_PULSE_THRESHOLD_MASK));
        return ret;
}
static int mma7660fc_set_pulse_axis(struct i2c_client* client,short axis)
{
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_PD) & ~MMA7660FC_PULSE_AXIS_MASK;
        ret = i2c_write(client, REG_PD, data |
                                (axis & MMA7660FC_PULSE_AXIS_MASK));
        return ret;
}
static int mma7660fc_set_sleep_count(struct i2c_client* client,short count)
{
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_SPCNT) & ~MMA7660FC_SLEEP_COUNT_MASK;
        ret = i2c_write(client, REG_SPCNT, data |
                                (count & MMA7660FC_SLEEP_COUNT_MASK));
        return ret;
}
static int mma7660fc_set_auto_wake_samplerate(struct i2c_client* client,short samplerate)
{
        int ret = 0;
        u8 data;
        data = i2c_read(client, REG_SR) & ~MMA7660FC_SR_AUTO_WAKE_MASK;
        ret = i2c_write(client, REG_SR, data |
                                (samplerate & MMA7660FC_SR_AUTO_WAKE_MASK));
        return ret;
}
static int mma7660fc_set_auto_sleep_samplerate(struct i2c_client* client,short samplerate)
{
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_SR) & ~MMA7660FC_SR_AUTO_SLEEP_MASK;
        ret = i2c_write(client, REG_SR, data |
                                (samplerate & MMA7660FC_SR_AUTO_SLEEP_MASK));
        return ret;
}
static int mma7660fc_set_sample_debounce(struct i2c_client* client,short samples)
{
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_SR) & ~MMA7660FC_SR_DEBOUNCE_MASK;
        ret = i2c_write(client, REG_SR, data |
                                (((samples - 1) << MMA7660FC_SR_DEBOUNCE_SHIFT) & MMA7660FC_SR_DEBOUNCE_MASK));
        return ret;
}
static int mma7660fc_enable_auto_wake(struct i2c_client* client,int on)
{
        int ret = 0;
        u8 data;

        if (!on)
                data = i2c_read(client, REG_MODE) | MMA7660FC_MODE_AUTO_WAKE;
        else
                data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_AUTO_WAKE;

        ret = i2c_write(client, REG_MODE, data);
        return ret;
}

static int mma7660fc_enable_auto_sleep(struct i2c_client* client,int on)
{
        int ret = 0;
        u8 data;

        if (!on)
                data = i2c_read(client, REG_MODE) | MMA7660FC_MODE_AUTO_SLEEP;
        else
                data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_AUTO_SLEEP;

        ret = i2c_write(client, REG_MODE, data);
        return ret;
}
static int mma7660fc_set_prescaler(struct i2c_client* client,int divisor)
{
        int ret = 0;
        u8 data;

        if (!divisor)
                data = i2c_read(client, REG_MODE) | MMA7660FC_MODE_PRESCALER;
        else
                data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_PRESCALER;

        ret = i2c_write(client, REG_MODE, data);
        return ret;
}

static int mma7660fc_set_int_pin_type(struct i2c_client* client,int type)
{
        int ret = 0;
        u8 data;

        if (!type)
                data = i2c_read(client, REG_MODE) | MMA7660FC_MODE_INT_TYPE;
        else
                data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_INT_TYPE;

        ret = i2c_write(client, REG_MODE, data);
        return ret;
}
static int mma7660fc_set_int_pin_active_level(struct i2c_client* client,int level)
{
        int ret = 0;
        u8 data;

        if (!level)
                data = i2c_read(client, REG_MODE) | MMA7660FC_MODE_INT_ACTIVE_LEVEL;
        else
                data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_INT_ACTIVE_LEVEL;

        ret = i2c_write(client, REG_MODE, data);
        return ret;
}



//static short mma7660fc_get_interrupt_list(struct i2c_client* client)
//{
  //      u8 data;
//
  //      data = mma7660fc_read(client, REG_INTSU) & MMA7660FC_INT_SETUP_MASK;
    //    return data;
//}
// /sysfs

// /dev/accel_ctrl

static int mma7660fc_ctrl_open(struct inode *inode, struct file *file)
{
        return 0;
}

static int mma7660fc_ctrl_release(struct inode *inode, struct file *file)
{
        return 0;
}

static long
mma7660fc_ctrl_ioctl(struct file *file,
              unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	printk("mma7660fc_ioctl\n");
	return ret;
}
static struct file_operations mma7660fc_ctrl_fops = {
        .owner = THIS_MODULE,
        .open = mma7660fc_ctrl_open,
        .release = mma7660fc_ctrl_release,
        .unlocked_ioctl = mma7660fc_ctrl_ioctl,
};

static struct miscdevice mma7660fc_ctrl_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "accel_ctrl",
        .fops = &mma7660fc_ctrl_fops,
};
//***************************//

#define MMA7660FC_VERSION 	"v2"
#define MMA7660FC_DATE		"11.01.2013"

static const struct i2c_device_id mma7660fc_id[] = {
        {"mma7660fc", 0},
        {},
};

MODULE_DEVICE_TABLE(i2c, mma7660fc_id);

static irqreturn_t mma7660fc_irq_callback(int irq, void *dev_id)
{
	struct mma7660fc_data* data = i2c_get_clientdata((struct i2c_client*)dev_id);
        printk("mma7660fc_isr: got irq %d\n", irq);
	schedule_work(&data->irq_work);
        return IRQ_HANDLED;
}
static void mma7660fc_irq_worker(struct work_struct *work)
{
//	struct mma7660fc_data *data = i2c_get_clientdata(this_client);
	printk("123123123\n");
}

static int mma7660fc_open(struct input_dev *dev)
{
	int res = 0;
	//struct mma7660fc_data* i2c_data = input_get_drvdata(i2c_data->input_dev);
	return res;
}
static void mma7660fc_close(struct input_dev *dev)
{
}

static int mma7660fc_probe(struct i2c_client *client,
                const struct i2c_device_id *id){
	int res=0;
	struct mma7660fc_data* i2c_data;
	
	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE_DATA)){
       		res=-ENODEV;
		return res;
	}
	i2c_data = kzalloc(sizeof(struct mma7660fc_data), GFP_KERNEL);
	if(!i2c_data){
		res = -ENOMEM;
		return res;
	}
	i2c_data->pdata = (struct mma7660fc_pdata*) client->dev.platform_data;
	if(!i2c_data->pdata || (i2c_data->pdata->irq <=0)){
		res = -ENODEV;
		goto exit_check_platform_data_failed;
	}
//todo FLAGS
	printk("mma7660fc_irq\n");

	res = request_irq(i2c_data->pdata->irq,
			mma7660fc_irq_callback,
			(IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING),
			"mma7660_irq",
			client);
	if (res){
		printk(KERN_ERR "mma7660fc_probe: Irq request failed (irq %d)!!\n", i2c_data->pdata->irq);
                goto exit_req_irq_failed;
	}
	INIT_WORK(&i2c_data->irq_work, mma7660fc_irq_worker);
	i2c_set_clientdata(client, i2c_data);
	i2c_data->input_dev = input_allocate_device();
	if (!i2c_data->input_dev){
                res = -ENOMEM;
                printk(KERN_ERR
                       "mma7660fc_probe: Failed to allocate input device");
                goto exit_input_dev_alloc_failed;
        }
	input_set_drvdata(i2c_data->input_dev,i2c_data);

	set_bit(EV_ABS, i2c_data->input_dev->evbit);
	   /* x-axis acceleration */
        input_set_abs_params(i2c_data->input_dev, ABS_X, -32, 31, 0, 0);
        /* y-axis acceleration */
        input_set_abs_params(i2c_data->input_dev, ABS_Y, -32, 31, 0, 0);
        /* z-axis acceleration */
        input_set_abs_params(i2c_data->input_dev, ABS_Z, -32, 31, 0, 0);
        /* pulse event */
        input_set_abs_params(i2c_data->input_dev, ABS_MISC, 0, 255, 0, 0);
        /* GP event */
        input_set_abs_params(i2c_data->input_dev, ABS_TILT_X, 0, 65535, 0, 0);

	i2c_data->input_dev->name = "MMA7660FC Accelerometer";
        i2c_data->input_dev->open = mma7660fc_open;
        i2c_data->input_dev->close = mma7660fc_close;

	res = misc_register(&mma7660fc_ctrl_device);
	
	if (res){
		printk(KERN_ERR "Unregister input device %s\n",i2c_data->input_dev->name);
		goto exit_misc_input_device;
	}
//todo set default parametr for chip ;
	
	mma7660fc_set_mode(client,MMA7660FC_MODE_MEASURE);
	mma7660fc_interrupt_setup(client,MMA7660FC_INT_SHAKE_X|MMA7660FC_INT_SHAKE_Y|MMA7660FC_INT_SHAKE_Z|MMA7660FC_INT_MEASURE|MMA7660FC_INT_TAP);
	mma7660fc_set_pulse_axis(client,MMA7660FC_PULSE_AXIS_X|MMA7660FC_PULSE_AXIS_Y|MMA7660FC_PULSE_AXIS_Z);
	mma7660fc_set_pulse_threshold(client,31);
	mma7660fc_set_pulse_debounce(client,10);
	mma7660fc_set_sleep_count(client,0);
	mma7660fc_set_auto_wake_samplerate(client,MMA7660FC_SR_AUTO_WAKE_120);
	mma7660fc_set_auto_sleep_samplerate(client,MMA7660FC_SR_AUTO_SLEEP_1);
	mma7660fc_set_sample_debounce(client,1);

	mma7660fc_enable_auto_wake(client,ON);
	mma7660fc_enable_auto_sleep(client,ON);
	mma7660fc_set_prescaler(client,DIVIDE_BY_1);
	mma7660fc_set_int_pin_type(client,OPEN_DRAIN);
	mma7660fc_set_int_pin_active_level(client,ACTIVE_LOW);

	res = input_register_device(i2c_data->input_dev);
        if (res) {
                printk(KERN_ERR
                       "mma7660fc_probe: Unable to register input device %s \n",
                       i2c_data->input_dev->name);
                goto exit_input_register_device_failed;
        }

	return 0;

exit_input_register_device_failed:
	misc_deregister(&mma7660fc_ctrl_device);
exit_misc_input_device:
	input_free_device(i2c_data->input_dev);
exit_input_dev_alloc_failed:
	free_irq(i2c_data->pdata->irq, client);
exit_req_irq_failed:
	kfree(i2c_data);
exit_check_platform_data_failed:
	return res;
}
static int mma7660fc_resume(struct i2c_client *client){
	return 0;
}
static int mma7660fc_suspend(struct i2c_client *client, pm_message_t mesg){
	return 0;
}
static int mma7660fc_remove(struct i2c_client *client){
	struct mma7660fc_data *data = i2c_get_clientdata(client);
        misc_deregister(&mma7660fc_ctrl_device);
        input_unregister_device(data->input_dev);
        free_irq(data->pdata->irq, data->client);
        kfree(data);
	return 0;
}
static struct i2c_driver mma7660fc_driver = {
        .driver = {
                .owner  = THIS_MODULE,
                .name   = "mma7660fc",
        },
        .probe          = mma7660fc_probe,
        .suspend        = mma7660fc_suspend,
        .resume         = mma7660fc_resume,
        .remove         = __exit_p(mma7660fc_remove),
        .id_table       = mma7660fc_id,
};

static int __init mma7660fc_init(void)
{
        int res;
	printk("mma7660fc_init\n");
	if ((res = i2c_add_driver(&mma7660fc_driver))) {
                printk("mma7660fc: Driver registration failed, module not inserted.\n");
                return res;
        }
        return 0;
}

static void __exit mma7660fc_exit(void)
{
	i2c_del_driver(&mma7660fc_driver);
	printk("mma7660fc_exit\n");
}

MODULE_DESCRIPTION("Input device driver for MMA7660FC accelerometer");
MODULE_LICENSE("GPL");

module_init(mma7660fc_init)
module_exit(mma7660fc_exit)
