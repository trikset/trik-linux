#include <linux/time.h>
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
#include "l3g42xxd.h"

/** Register map */

struct l3g42xxd_data {
	struct input_dev *input_dev;
//	struct input_polled_dev *input_polled;
	struct platform_device *platform_device;
	struct l3g42xxd_platform_data *pdata;
};

struct gyro_val {
        s16 x;
        s16 y;
        s16 z;
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
#if 0
static struct miscdevice l3g42xxd_misc_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "l3g42xxd",
        .fops = &l3g42xxd_misc_fops,
};
#endif 


static int l3g42xxd_get_gyro_data(struct l3g42xxd_chip *chip,
                                         struct gyro_val *data)
{
    u8 _data[L3G4200D_OUT_Z_H - L3G4200D_STATUS_REG+1];
    chip->read_block(chip->dev, L3G4200D_STATUS_REG, sizeof(_data)/sizeof(_data[0]), _data);
    //todo: check why overflow is true
    if ((_data[L3G4200D_STATUS_REG - L3G4200D_STATUS_REG] != 0xff/*L3G4200D_STATUS_REG_EXPECTED*/) && printk_ratelimit()){
        pr_warning("%s: L3G4200D_STATUS_REG unexpected value (0x%02x)\n", __func__, _data[L3G4200D_STATUS_REG - L3G4200D_STATUS_REG]);  
    }
    data->x = (_data[L3G4200D_OUT_X_H - L3G4200D_STATUS_REG] << 8) | _data[L3G4200D_OUT_X_L - L3G4200D_STATUS_REG];
    data->y = (_data[L3G4200D_OUT_Y_H - L3G4200D_STATUS_REG] << 8) | _data[L3G4200D_OUT_Y_L - L3G4200D_STATUS_REG];
    data->z = (_data[L3G4200D_OUT_Z_H - L3G4200D_STATUS_REG] << 8) | _data[L3G4200D_OUT_Z_L - L3G4200D_STATUS_REG];
    chip->read(chip->dev,L3G4200D_INTERRUPT_SRC);
    return 0;
}
static void l3g42xxd_report_values(struct l3g42xxd_chip *chip,
                                         struct gyro_val *data)
{
    input_report_abs(chip->data->input_dev, ABS_X, data->x);
    input_report_abs(chip->data->input_dev, ABS_Y, data->y);
    input_report_abs(chip->data->input_dev, ABS_Z, data->z);
    input_sync(chip->data->input_dev);
}

static irqreturn_t l3g42xxd_irq_callback(int irq, void *dev_id){
	struct l3g42xxd_chip* chip = dev_id;
    if(chip)
		schedule_work(&chip->irq_work);
	return IRQ_HANDLED;
}


static void l3g42xxd_irq_worker(struct work_struct *work){
#if 0
    static u64 _timestamp [2];
    static u32 _counter = 0;
#endif 
    struct l3g42xxd_chip *chip = container_of(work, struct l3g42xxd_chip, irq_work);
    struct gyro_val data;
#if 0    
    _timestamp[0] = local_clock();
#endif
	l3g42xxd_get_gyro_data(chip, &data);
    l3g42xxd_report_values(chip, &data);
#if 0
    _timestamp[1] = local_clock();
    if (_counter %1600 == 0){
        pr_err("[l3g42xxd] operation time = %lld \n", _timestamp[1] - _timestamp[0]);
    }
    _counter++;
#endif
}

static void l3g42xxd_input_dev_shutdown(struct l3g42xxd_chip* chip)
{
    struct l3g42xxd_data* gyro = chip->data;
	input_unregister_device(gyro->input_dev);
	free_irq(chip->irq, chip);
	input_free_device(gyro->input_dev);
}
static int l3g42xxd_init_chip(struct l3g42xxd_chip* chip){
	int err = -1;
    err = chip->write(chip->dev,L3G4200D_CTRL_REG1,0xc7);
    pr_err("%s: L3G4200D_CTRL_REG1 = 0x%02x\n",__func__,chip->read(chip->dev,L3G4200D_CTRL_REG1));

    err = chip->write(chip->dev,L3G4200D_CTRL_REG2,0x00);
    pr_err("%s: L3G4200D_CTRL_REG2 = 0x%02x\n",__func__,chip->read(chip->dev,L3G4200D_CTRL_REG2));

    err = chip->write(chip->dev,L3G4200D_CTRL_REG3,0x80);
    pr_err("%s: L3G4200D_CTRL_REG3 = 0x%02x\n",__func__,chip->read(chip->dev,L3G4200D_CTRL_REG3));

    err = chip->write(chip->dev,L3G4200D_CTRL_REG4,0xa0);
    pr_err("%s: L3G4200D_CTRL_REG4 = 0x%02x\n",__func__,chip->read(chip->dev,L3G4200D_CTRL_REG4));

    err = chip->write(chip->dev,L3G4200D_CTRL_REG5,0x00);
    pr_err("%s: L3G4200D_CTRL_REG5 = 0x%02x\n",__func__,chip->read(chip->dev,L3G4200D_CTRL_REG5));

    err = chip->write(chip->dev,L3G4200D_REF_DATA_CAP,0x00);
    pr_err("%s: L3G4200D_REF_DATA_CAP = 0x%02x\n",__func__,chip->read(chip->dev,L3G4200D_REF_DATA_CAP));

    err = chip->write(chip->dev,L3G4200D_FIFO_CTRL,0x00);
    pr_err("%s: L3G4200D_FIFO_CTRL = 0x%02x\n",__func__,chip->read(chip->dev,L3G4200D_FIFO_CTRL));

    err = chip->write(chip->dev,L3G4200D_INTERRUPT_CFG,0x7F);
    pr_err("%s: L3G4200D_INTERRUPT_CFG = 0x%02x\n",__func__,chip->read(chip->dev,L3G4200D_INTERRUPT_CFG));

    err = chip->write(chip->dev,L3G4200D_CTRL_REG1,( 0xc7 | PM_MASK));
    pr_err("%s: L3G4200D_CTRL_REG1 = 0x%02x\n",__func__,chip->read(chip->dev,L3G4200D_CTRL_REG1));
    
	return 0;
}
static int l3g42xxd_open(struct input_dev *dev)
{
    return 0;
}

static void l3g42xxd_close(struct input_dev *dev)
{
}

static int l3g42xxd_input_dev_init(struct l3g42xxd_chip* chip)
{
    struct l3g42xxd_data* gyro = chip->data;
	int err;
    int device_id;
    device_id = chip->read(chip->dev,L3G4200D_WHO_AM_I);
    
     pr_err("%s:chip :0x%02x (%d)\n",__func__,device_id,device_id);
    switch (device_id){
        case L3G4200D_ID:
        break;
        case L3GD20_ID:
        break;
        default:
            pr_err("%s:Unknown chip :0%02x \n",__func__,device_id);
            return -ENODEV;
        break;
    }
   
    chip->device_address = device_id;

	gyro->input_dev = input_allocate_device();
	if (!gyro->input_dev){
		err = -ENOMEM;
                pr_err("%s:input device allocate failed\n",__func__);
                goto exit_input_device_alloc;
	}
	gyro->input_dev->name = "l3g42xxd";
	gyro->input_dev->id.bustype = chip->bus_type;
	gyro->input_dev->dev.parent = chip->dev;
	gyro->input_dev->open = l3g42xxd_open;
	gyro->input_dev->close  = l3g42xxd_close;

	set_bit(EV_ABS, gyro->input_dev->evbit);
	input_set_drvdata(gyro->input_dev, gyro);
	input_set_abs_params(gyro->input_dev, ABS_X,
                             -32768, 32767, 0, 0);
 	input_set_abs_params(gyro->input_dev, ABS_Y,
                             -32768, 32767, 0, 0);
 	input_set_abs_params(gyro->input_dev, ABS_Z,
                             -32768, 32767, 0, 0);

        pr_warning("%s, gyro %p\n", __func__, gyro);
	err = request_irq(chip->irq,
				      l3g42xxd_irq_callback,
                    (IRQF_TRIGGER_RISING),
					       "l3g42xxd_irq",
						     chip);
	if (err != 0) {
        pr_err("%s: irq request failed: %d\n", __func__, err);
        err = -ENODEV;
        goto exit_irq_request;
    }

	err = input_register_device(gyro->input_dev);
    if (err) {
        pr_err("%s:unable to register input device %s\n",__func__,
        gyro->input_dev->name);
        err = -ENODEV;
        goto exit_register_device;
    }
	err = l3g42xxd_init_chip(chip);
	if (err < 0){
		goto exit_init_chip;
	}
	{
		struct gyro_val data;
        l3g42xxd_get_gyro_data(chip, &data);
	}

	return 0;
exit_init_chip:
	input_unregister_device(gyro->input_dev);
exit_register_device:
	free_irq(chip->irq, chip);
exit_irq_request:
	input_free_device(gyro->input_dev);
exit_input_device_alloc:
	return err;
}
int l3g42xxd_probe(struct device *dev, 
                        u16 bus_type,
                        int irq,
                        l3g42xxd_read_t read,
                        l3g42xxd_write_t write,
                        l3g42xxd_read_block_t read_block,
                        struct l3g42xxd_chip **chip_data)
{
    struct l3g42xxd_chip *chip ;
    int ret = -1;
    
    if (irq <= 0) {
        pr_err("%s : IRQ not configured!\n",__func__);
        ret = -EINVAL;
        goto exit_no_irq;
    }
    chip = kzalloc(sizeof(*chip) + sizeof(*chip->data), GFP_KERNEL);
    if (chip == NULL){
        pr_err("%s:failed to allocate memory for module(chip) data\n",__func__);
        ret = -ENOMEM;
        goto exit_no_memory_chip;
    }
    chip->data = (struct l3g42xxd_data*) (chip + 1);
    
    chip->bus_type = bus_type;
    chip->read = read;
    chip->write = write;
    chip->read_block = read_block;
    chip->irq = irq;
    chip->dev = dev;
    
    INIT_WORK(&chip->irq_work, l3g42xxd_irq_worker);
    
    ret = l3g42xxd_input_dev_init(chip);
    if (ret){
        pr_err("%s:failed to init input device \n",__func__);
        goto exit_input_dev_init; 
    }

    // todo misc register


    pr_err("%s : pointer = %p\n",__func__,chip);
    *chip_data = chip;
    return 0;
exit_input_dev_init:
    kfree(chip);
exit_no_memory_chip:
exit_no_irq:
    return ret;
}
EXPORT_SYMBOL_GPL(l3g42xxd_probe);

void l3g42xxd_suspend(struct l3g42xxd_chip *chip)
{
}
EXPORT_SYMBOL_GPL(l3g42xxd_suspend);

void l3g42xxd_resume(struct l3g42xxd_chip *ac)
{
}
EXPORT_SYMBOL_GPL(l3g42xxd_resume);

void l3g42xxd_remove(struct l3g42xxd_chip *chip)
{
    l3g42xxd_input_dev_shutdown(chip);
    flush_work_sync(&chip->irq_work);
    pr_err("%s : pointer = %p\n",__func__,chip);
    kfree(chip);
}
EXPORT_SYMBOL_GPL(l3g42xxd_remove);

MODULE_DESCRIPTION("L3g42xxd Gyroscope");
MODULE_AUTHOR("romik.momik@trikset.com");
MODULE_LICENSE("GPL");
