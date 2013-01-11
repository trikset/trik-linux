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
static inline int mma7660fc_write(struct i2c_client *client, int reg, int value)
{
        int ret;
        ret = i2c_smbus_write_byte_data(client, reg, value);
        printk("mma7660fc_write: Reg = 0x%02X, Value = 0x%02X, Ret = %d\n", reg, value, re$
        return ret;
};

static inline int mma7660fc_read(struct i2c_client *client, int reg)
{
        int value;
        value = i2c_smbus_read_byte_data(client, reg);;
        printk("mma7660fc_read: Reg = 0x%02X, Value = 0x%02X\n", reg, value);
        return value;
};
// /sysfs

// /dev/accel_ctrl
#ifdef 0
static struct file_operations mma7660fc_ctrl_fops = {
	.owner = THIS_MODULE,
	.open  = mma7660fc_ctrl_open,
	.release = mma7660fc_ctrl_release,
	.unlocked_ioctl = mma7660fc_ctrl_ioctl,
};

static struct miscdevice mma7660fc_ctrl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "accel_ctrl",
	.fops = &mma7660fc_ctrl_fops,
};

//  operation on MMA 7660FC 

#endif 
static int mma7660fc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	return;
}

static int __exit mma7660fc_remove(struct i2c_client *client)
{
	struct mma7660fc_data *data = i2c_get_clientdata(client);
	misc_deregister(&mma7660fc_ctrl_device);
	input_unregister_device(data->input_dev);
	free_irq(data->pdata->irq, client);
	kfree(data);
	return 0;
}

#define MMA7660FC_VERSION 	"v2"
#define MMA7660FC_DATE		"11.01.2013"

static const struct i2c_device_id mma7660fc_id[] = {
        {"mma7660fc", 0},
        {},
};

MODULE_DEVICE_TABLE(i2c, mma7660fc_id);

static int mma7660fc_probe(struct i2c_client *client,
                const struct i2c_device_id *id){
	return 0;
}
static int mma7660fc_resume(struct i2c_client *client){
	return 0;
}
static int mma7660fc_suspend(struct i2c_client *client, pm_message_t mesg){
	return 0;
}
static int mma7660fc_remove(struct i2c_client *client){
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
	printk("mma7660fc_exit\n");
}

MODULE_DESCRIPTION("Input device driver for MMA7660FC accelerometer");
MODULE_LICENSE("GPL");

module_init(mma7660fc_init)
module_exit(mma7660fc_exit)
