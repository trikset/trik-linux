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
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/mma7660fc.h>
// I2C Read & Write 

struct mma7660fc_data{	
	struct mma7660fc_pdata *pdata;
	struct work_struct irq_work;
	struct i2c_client * client;
	struct input_dev *input_dev;
    struct platform_device* platform_device;
    int open_count;
    int standby;
};
static inline int i2c_write(struct i2c_client *client, int reg, int value){
    int ret;
    ret = i2c_smbus_write_byte_data(client, reg, value);
    printk("mma7660fc_write: Reg = 0x%02X, Value = 0x%02X, Ret = %d\n", reg, value, ret);
    return ret;
}
static inline int i2c_read(struct i2c_client *client, int reg){
        int value;
        value = i2c_smbus_read_byte_data(client, reg);;
        printk("mma7660fc_read: Reg = 0x%02X, Value = 0x%02X\n", reg, value);
        return value;
}
static int mma7660fc_interrupt_setup(struct i2c_client* client,short events){
    int ret = 0;
    u8 data;
    data = i2c_read(client, REG_INTSU) & ~MMA7660FC_INT_SETUP_MASK;
    ret = i2c_write(client, REG_INTSU, data |
                       (events & MMA7660FC_INT_SETUP_MASK));
    i2c_read(client, REG_INTSU);

    return ret;
}
static int mma7660fc_set_mode(struct i2c_client* client,short mode){
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_MASK;
        ret = i2c_write(client, REG_MODE, data |
                                (mode & MMA7660FC_MODE_MASK));
        return ret;
}
static int mma7660fc_set_pulse_debounce(struct i2c_client* client,short samples){
	int ret = 0;
	u8 data;
	data = i2c_read(client, REG_PDET) & ~MMA7660FC_PULSE_DEBOUNCE_MASK;
	ret = i2c_write(client, REG_PDET, data |
                                ((samples - 1)& MMA7660FC_PULSE_DEBOUNCE_MASK));
	return ret;
}
static int mma7660fc_set_pulse_threshold(struct i2c_client* client,short trshld){
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_PD) & ~MMA7660FC_PULSE_THRESHOLD_MASK;
        ret = i2c_write(client, REG_PD, data |
                                (trshld & MMA7660FC_PULSE_THRESHOLD_MASK));
        return ret;
}
static int mma7660fc_set_pulse_axis(struct i2c_client* client,short axis){
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_PD) & ~MMA7660FC_PULSE_AXIS_MASK;
        ret = i2c_write(client, REG_PD, data |
                                (axis & MMA7660FC_PULSE_AXIS_MASK));
        return ret;
}
static int mma7660fc_set_sleep_count(struct i2c_client* client,short count){
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_SPCNT) & ~MMA7660FC_SLEEP_COUNT_MASK;
        ret = i2c_write(client, REG_SPCNT, data |
                                (count & MMA7660FC_SLEEP_COUNT_MASK));
        return ret;
}
static int mma7660fc_set_auto_wake_samplerate(struct i2c_client* client,short samplerate){
        int ret = 0;
        u8 data;
        data = i2c_read(client, REG_SR) & ~MMA7660FC_SR_AUTO_WAKE_MASK;
        ret = i2c_write(client, REG_SR, data |
                                (samplerate & MMA7660FC_SR_AUTO_WAKE_MASK));
        return ret;
}
static int mma7660fc_set_auto_sleep_samplerate(struct i2c_client* client,short samplerate){
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_SR) & ~MMA7660FC_SR_AUTO_SLEEP_MASK;
        ret = i2c_write(client, REG_SR, data |
                                (samplerate & MMA7660FC_SR_AUTO_SLEEP_MASK));
        return ret;
}
static int mma7660fc_set_sample_debounce(struct i2c_client* client,short samples){
        int ret = 0;
        u8 data;

        data = i2c_read(client, REG_SR) & ~MMA7660FC_SR_DEBOUNCE_MASK;
        ret = i2c_write(client, REG_SR, data |
                                (((samples - 1) << MMA7660FC_SR_DEBOUNCE_SHIFT) & MMA7660FC_SR_DEBOUNCE_MASK));
        return ret;
}
static int mma7660fc_enable_auto_wake(struct i2c_client* client,int on){
        int ret = 0;
        u8 data;

        if (!on)
                data = i2c_read(client, REG_MODE) | MMA7660FC_MODE_AUTO_WAKE;
        else
                data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_AUTO_WAKE;

        ret = i2c_write(client, REG_MODE, data);
        return ret;
}
static int mma7660fc_enable_auto_sleep(struct i2c_client* client,int on){
        int ret = 0;
        u8 data;

        if (!on)
                data = i2c_read(client, REG_MODE) | MMA7660FC_MODE_AUTO_SLEEP;
        else
                data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_AUTO_SLEEP;

        ret = i2c_write(client, REG_MODE, data);
        return ret;
}
static int mma7660fc_set_prescaler(struct i2c_client* client,int divisor){
        int ret = 0;
        u8 data;

        if (!divisor)
                data = i2c_read(client, REG_MODE) | MMA7660FC_MODE_PRESCALER;
        else
                data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_PRESCALER;

        ret = i2c_write(client, REG_MODE, data);
        return ret;
}
static int mma7660fc_set_int_pin_type(struct i2c_client* client,int type){
        int ret = 0;
        u8 data;

        if (!type)
                data = i2c_read(client, REG_MODE) | MMA7660FC_MODE_INT_TYPE;
        else
                data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_INT_TYPE;

        ret = i2c_write(client, REG_MODE, data);
        return ret;
}
static int mma7660fc_set_int_pin_active_level(struct i2c_client* client,int level){
        int ret = 0;
        u8 data;

        if (!level)
                data = i2c_read(client, REG_MODE) | MMA7660FC_MODE_INT_ACTIVE_LEVEL;
        else
                data = i2c_read(client, REG_MODE) & ~MMA7660FC_MODE_INT_ACTIVE_LEVEL;

        ret = i2c_write(client, REG_MODE, data);
        return ret;
}
static void mma7660fc_get_values(struct i2c_client* client,int * x, int *y,int *z){
        unsigned char tmp;
        if ((!x)||(!y)||(!z))
            return ;
        /* x, y and z are 6 bits signed values */
        /* Wait for the data to be ready */
        while ((tmp = i2c_read(client, REG_XOUT)) & MMA7660FC_INVALID_FLAG);
        *x = ((signed char) (tmp << 2))/4;

        /* Wait for the data to be ready */
        while ((tmp = i2c_read(client, REG_YOUT)) & MMA7660FC_INVALID_FLAG);
        *y = ((signed char) (tmp << 2))/4;

        /* Wait for the data to be ready */
        while ((tmp = i2c_read(client, REG_ZOUT)) & MMA7660FC_INVALID_FLAG);
        *z = ((signed char) (tmp << 2))/4;
}
struct tilt_status{
     short int shake;
     short int tap;
     short int pola;
     short int bafro;
};
static void mma7660fc_get_tilt_status (struct i2c_client* client,struct tilt_status* status){
    u8 data;
    if (!status)
         return;
    
    while ((data = i2c_read(client, REG_TILT) & MMA7660FC_TILT_STATUS_MASK) & MMA7660FC_INVALID_FLAG);
    status->shake=  data & MMA7660FC_TILT_SHAKE;
    status->tap= data &MMA7660FC_TILT_TAP;
    status->pola= data & MMA7660FC_TILT_PL_MASK;
    status->bafro= data & MMA7660FC_TILT_FB_MASK;
}
struct sample_rate_status{
    short int awsrs;
    short int amsrs;
};
static void  mma7660fc_get_sample_rate_status (struct i2c_client* client,struct sample_rate_status * status){
    u8 data;
    if (!status)
         return;
    data = i2c_read(client, REG_SRST) & 0x03;
    status->awsrs = data & 0x01; 
    status->amsrs = (data >> 1) &0x01;
}

static ssize_t mma7660_state_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    return 1;
}
static ssize_t mma7660_test_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    return 1;    
}
static ssize_t mma7660_test_store(struct device *dev, struct device_attribute *attr,
                    const char *buf, size_t count)
{
    return count;
}

#warning todo device attr all 

static DEVICE_ATTR(state, (S_IRUGO), mma7660_state_read ,NULL); //standby or active only read 
static DEVICE_ATTR(test, (S_IRUGO|S_IWUSR), mma7660_test_read ,mma7660_test_store); //standby or active only read 

static const struct attribute *mma7660fc_manage_attrs[] = {
    &dev_attr_state.attr,
    &dev_attr_test.attr,
    NULL,
};
static const struct attribute_group mma7660fc_manage_attrs_group = {
    .attrs = (struct attribute **) mma7660fc_manage_attrs,
};

static int mma7660fc_ctrl_open(struct inode *inode, struct file *file){
    return 0;
}
static int mma7660fc_ctrl_release(struct inode *inode, struct file *file){
    return 0;
}

static long mma7660fc_ctrl_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
	int ret = 0;
	pr_info("mma7660fc_ioctl\n");
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
    data->client = dev_id;
    schedule_work(&data->irq_work);

    return IRQ_HANDLED;
}
static void mma7660fc_irq_worker(struct work_struct *work){
    int x =0,y=0,z=0;
    struct mma7660fc_data *data = container_of(work, struct mma7660fc_data, irq_work);
    if (data->client){
        #warning todo
        static int x_avg=0, y_avg=0, z_avg=0, cnt=0;
        mma7660fc_get_values(data->client,&x,&y,&z);

        x_avg += x;
        y_avg += y;
        z_avg += z;
        if (++cnt >= 10) {
            input_report_abs(data->input_dev, ABS_X, (short)(x_avg/cnt));
            input_report_abs(data->input_dev, ABS_Y, (short)(y_avg/cnt));
            input_report_abs(data->input_dev, ABS_Z, (short)(z_avg/cnt));
            input_sync(data->input_dev);
            cnt = 0;
            x_avg=0;
            y_avg=0;
            z_avg=0;
        }
    }else{
        printk(KERN_WARNING "mma7660fc_irq_worker i2c client is broken\n");
    }
}

static int mma7660fc_open(struct input_dev *dev){
 #warning atomic operation 
	int res = 0;
    struct i2c_client* client = input_get_drvdata(dev);
    struct mma7660fc_data* i2c_data = i2c_get_clientdata(client);

    i2c_data->open_count++;
    if(i2c_data->open_count>1)
        return res;

    res = request_irq(i2c_data->pdata->irq,
                            mma7660fc_irq_callback,
                            (IRQF_TRIGGER_FALLING),
                            "mma7660_irq",
                            client);
    
    if (res){
            pr_err("mma7660fc_probe: Irq request failed (irq %d)!!\n", i2c_data->pdata->irq);
            i2c_data->open_count--;
            return res;
    }
    //mma7660fc_set_mode(client,MMA7660FC_MODE_MEASURE);
    i2c_read(client,REG_SRST);


    pr_info("fc open \n");
    return res;
}
static void mma7660fc_close(struct input_dev *dev){
    struct i2c_client* client = input_get_drvdata(dev);
    struct mma7660fc_data* i2c_data = i2c_get_clientdata(client);
    i2c_data->open_count--;
    if(i2c_data->open_count>0)
        return;
    free_irq(i2c_data->pdata->irq, client);
    //mma7660fc_set_mode(client,MMA7660FC_MODE_STANDBY);
    pr_info("fc close \n");
}
static int mma7660fc_probe(struct i2c_client *client, const struct i2c_device_id *id){
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

    i2c_data->open_count = 0;

	pr_info("mma7660fc_irq\n");
    INIT_WORK(&i2c_data->irq_work, mma7660fc_irq_worker);
    i2c_set_clientdata(client, i2c_data);

	i2c_data->input_dev = input_allocate_device();
	if (!i2c_data->input_dev){
                res = -ENOMEM;
                printk(KERN_ERR
                       "mma7660fc_probe: Failed to allocate input device");
                goto exit_input_dev_alloc_failed;
        }
	input_set_drvdata(i2c_data->input_dev,client);

	set_bit(EV_ABS, i2c_data->input_dev->evbit);
    set_bit(EV_REP, i2c_data->input_dev->evbit);
    input_set_abs_params(i2c_data->input_dev, ABS_X, -32, 31, 0, 0);
    input_set_abs_params(i2c_data->input_dev, ABS_Y, -32, 31, 0, 0);
    input_set_abs_params(i2c_data->input_dev, ABS_Z, -32, 31, 0, 0);
    //input_set_abs_params(i2c_data->input_dev, ABS_MISC, 0, 255, 0, 0);
    //input_set_abs_params(i2c_data->input_dev, ABS_TILT_X, 0, 65535, 0, 0);
    i2c_data->input_dev->id.bustype = BUS_I2C;
    i2c_data->input_dev->dev.parent = &client->dev;
    i2c_data->input_dev->name = "MMA7660FC Accelerometer";
    i2c_data->input_dev->open = mma7660fc_open;
    i2c_data->input_dev->close = mma7660fc_close;
    res = misc_register(&mma7660fc_ctrl_device);
	pr_info("misc register\n");
    if (res){
		printk(KERN_ERR "Unregister input device %s\n",i2c_data->input_dev->name);
		goto exit_misc_input_device;
	}

    i2c_write(client,REG_MODE,0);
    i2c_write(client,REG_SPCNT,0);
    i2c_write(client,REG_INTSU,0x10);
    i2c_write(client,REG_SR,0x0);
    i2c_write(client,REG_MODE,0x1);

	//mma7660fc_set_mode(client,MMA7660FC_MODE_STANDBY);
	//mma7660fc_interrupt_setup(client,MMA7660FC_INT_MEASURE);
    //mma7660fc_set_pulse_axis(client,MMA7660FC_PULSE_AXIS_X|MMA7660FC_PULSE_AXIS_Y|MMA7660FC_PULSE_AXIS_Z);

	// mma7660fc_set_pulse_threshold(client,31);
	// mma7660fc_set_pulse_debounce(client,255);

	// mma7660fc_set_sleep_count(client,0);
	// mma7660fc_set_auto_wake_samplerate(client,MMA7660FC_SR_AUTO_WAKE_120);
	// mma7660fc_set_auto_sleep_samplerate(client,MMA7660FC_SR_AUTO_SLEEP_1);
	// mma7660fc_set_sample_debounce(client,1);
	// mma7660fc_enable_auto_wake(client,OFF);
	// mma7660fc_enable_auto_sleep(client,OFF);
	// mma7660fc_set_prescaler(client,DIVIDE_BY_1);
	// mma7660fc_set_int_pin_type(client,OPEN_DRAIN);
	// mma7660fc_set_int_pin_active_level(client,ACTIVE_LOW);
    //mma7660fc_set_mode(client,MMA7660FC_MODE_MEASURE);
   
	res = input_register_device(i2c_data->input_dev);
    if (res) {
            pr_err("mma7660fc_probe: Unable to register input device %s \n", i2c_data->input_dev->name);
            goto exit_input_register_device_failed;
    }
    res = sysfs_create_group(&client->dev.kobj, &mma7660fc_manage_attrs_group);
    if (res){
            pr_err("mma7660fc_probe: Unable to create sysfs \n");
            goto exit_sysfs_device_failed;
    }
    
    //register_sysfs_device
	return 0;

exit_sysfs_device_failed:
    	input_unregister_device(i2c_data->input_dev);
exit_input_register_device_failed:
	misc_deregister(&mma7660fc_ctrl_device);
exit_misc_input_device:
	input_free_device(i2c_data->input_dev);
exit_input_dev_alloc_failed:
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
    if (data->open_count)
        return -EBUSY; 
///request irq
    sysfs_remove_group(&client->dev.kobj, &mma7660fc_manage_attrs_group);
    input_unregister_device(data->input_dev);
    misc_deregister(&mma7660fc_ctrl_device);
    input_free_device(data->input_dev);
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

static int __init mma7660fc_init(void){
    int res;
	printk("mma7660fc_init\n");
	if ((res = i2c_add_driver(&mma7660fc_driver))) {
            printk("mma7660fc: Driver registration failed, module not inserted.\n");
            return res;
    }

    return 0;
}

static void __exit mma7660fc_exit(void){
	i2c_del_driver(&mma7660fc_driver);
	printk("mma7660fc_exit\n");
}

MODULE_DESCRIPTION("Input device driver for MMA7660FC accelerometer");
MODULE_LICENSE("GPL");

module_init(mma7660fc_init)
module_exit(mma7660fc_exit)
