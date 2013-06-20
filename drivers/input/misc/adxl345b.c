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

/* ADXL345/6 Register Map */
#define DEVID			0x00	/* R   Device ID */
#define THRESH_TAP		0x1D	/* R/W Tap threshold */
#define OFSX			0x1E	/* R/W X-axis offset */
#define OFSY			0x1F	/* R/W Y-axis offset */
#define OFSZ			0x20	/* R/W Z-axis offset */
#define DUR				0x21	/* R/W Tap duration */
#define LATENT			0x22	/* R/W Tap latency */
#define WINDOW			0x23	/* R/W Tap window */
#define THRESH_ACT		0x24	/* R/W Activity threshold */
#define THRESH_INACT	0x25	/* R/W Inactivity threshold */
#define TIME_INACT		0x26	/* R/W Inactivity time */
#define ACT_INACT_CTL	0x27	/* R/W Axis enable control for activity and */
				/* inactivity detection */
#define THRESH_FF		0x28	/* R/W Free-fall threshold */
#define TIME_FF			0x29	/* R/W Free-fall time */
#define TAP_AXES		0x2A	/* R/W Axis control for tap/double tap */
#define ACT_TAP_STATUS	0x2B	/* R   Source of tap/double tap */
#define BW_RATE			0x2C	/* R/W Data rate and power mode control */
#define POWER_CTL		0x2D	/* R/W Power saving features control */
#define INT_ENABLE		0x2E	/* R/W Interrupt enable control */
#define INT_MAP			0x2F	/* R/W Interrupt mapping control */
#define INT_SOURCE		0x30	/* R   Source of interrupts */
#define DATA_FORMAT		0x31	/* R/W Data format control */
#define DATAX0			0x32	/* R   X-Axis Data 0 */
#define DATAX1			0x33	/* R   X-Axis Data 1 */
#define DATAY0			0x34	/* R   Y-Axis Data 0 */
#define DATAY1			0x35	/* R   Y-Axis Data 1 */
#define DATAZ0			0x36	/* R   Z-Axis Data 0 */
#define DATAZ1			0x37	/* R   Z-Axis Data 1 */
#define FIFO_CTL		0x38	/* R/W FIFO control */
#define FIFO_STATUS		0x39	/* R   FIFO status */

#define DATA_READY	(1 << 7)
#define SINGLE_TAP	(1 << 6)
#define DOUBLE_TAP	(1 << 5)
#define ACTIVITY	(1 << 4)
#define INACTIVITY	(1 << 3)
#define FREE_FALL	(1 << 2)
#define WATERMARK	(1 << 1)
#define OVERRUN		(1 << 0)

#define PCTL_LINK	(1 << 5)
#define PCTL_AUTO_SLEEP (1 << 4)
#define PCTL_MEASURE	(1 << 3)
#define PCTL_SLEEP	(1 << 2)
#define PCTL_WAKEUP(x)	((x) & 0x3)
struct adxl345b_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
//	struct input_polled_dev *input_polled;
	struct platform_device *platform_device;
	struct work_struct irq_work;
	int irq;
	char phys[32];
};

struct accel_val {
	s16 x;
	s16 y;
	s16 z;
};

static int adxl345b_smbus_read(struct i2c_client *client, unsigned char reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int adxl345b_smbus_write(struct i2c_client *client,
			       unsigned char reg, unsigned char val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}
static int adxl345b_smbus_read_block(struct i2c_client *client,
				    unsigned char reg, int count,
				    void *buf)
{
	return i2c_smbus_read_i2c_block_data(client, reg, count, buf);
}
static const struct i2c_device_id adxl345b_id[] = {
        {"adxl345b", 0},
        {},
};

static int adxl345b_misc_open(struct inode *inode, struct file *file){
	pr_info("%s \n",__func__);
	return 0;
}
static int adxl345b_misc_release(struct inode *inode, struct file *file){
	pr_info("%s \n",__func__);
	return 0;
}
static long adxl345b_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
	pr_info("%s \n",__func__);
	return 0;
}
static const struct file_operations adxl345b_misc_fops = {
	.owner 			= THIS_MODULE,
	.open 			= adxl345b_misc_open,
	.release 		= adxl345b_misc_release,
	.unlocked_ioctl = adxl345b_misc_ioctl,
};

static struct miscdevice adxl345b_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "adxl345b",
	.fops = &adxl345b_misc_fops,
};

static irqreturn_t adxl345b_irq_callback(int irq, void *dev_id){
	struct adxl345b_data* accel = dev_id;
        if(accel)
		schedule_work(&accel->irq_work);
	return IRQ_HANDLED;
}
static void accel_irq_worker(struct work_struct *work)
{
	short buf[3];
	struct accel_val;

	struct adxl345b_data *accel = container_of(work, struct adxl345b_data, irq_work);
	adxl345b_smbus_read(accel->client,INT_SOURCE);

	adxl345b_smbus_read_block(accel->client, DATAX0, DATAZ1 - DATAX0 + 1, buf);

	input_report_abs(accel->input_dev, ABS_X, (s16) le16_to_cpu(buf[0]));
	input_report_abs(accel->input_dev, ABS_Y, (s16) le16_to_cpu(buf[1]));
	input_report_abs(accel->input_dev, ABS_Z, (s16) le16_to_cpu(buf[2]));
	input_sync(accel->input_dev);

}
static void adxl345b_input_dev_shutdown(struct adxl345b_data* accel)
{
	pr_info("%s: main\n",__func__);
	input_unregister_device(accel->input_dev);
	free_irq(accel->irq, accel);
	input_free_device(accel->input_dev);

}
static int adxl345b_init_chip(struct adxl345b_data* accel)
{
// #define THRESH_TAP		0x1D	/* R/W Tap threshold */
// #define OFSX			0x1E	/* R/W X-axis offset */
// #define OFSY			0x1F	/* R/W Y-axis offset */
// #define OFSZ			0x20	/* R/W Z-axis offset */
// #define DUR				0x21	/* R/W Tap duration */
// #define LATENT			0x22	/* R/W Tap latency */
// #define WINDOW			0x23	/* R/W Tap window */
// #define THRESH_ACT		0x24	/* R/W Activity threshold */
// #define THRESH_INACT	0x25	/* R/W Inactivity threshold */
// #define TIME_INACT		0x26	/* R/W Inactivity time */
// #define ACT_INACT_CTL	0x27	/* R/W Axis enable control for activity and */
// 				/* inactivity detection */
// #define THRESH_FF		0x28	/* R/W Free-fall threshold */
// #define TIME_FF			0x29	/* R/W Free-fall time */
// #define TAP_AXES		0x2A	/* R/W Axis control for tap/double tap */
// #define ACT_TAP_STATUS	0x2B	/* R   Source of tap/double tap */
// #define BW_RATE			0x2C	/* R/W Data rate and power mode control */
// #define POWER_CTL		0x2D	/* R/W Power saving features control */
// #define INT_ENABLE		0x2E	/* R/W Interrupt enable control */
// #define INT_MAP			0x2F	/* R/W Interrupt mapping control */
// #define INT_SOURCE		0x30	/* R   Source of interrupts */
// #define DATA_FORMAT		0x31	/* R/W Data format control */
// #define DATAX0			0x32	/* R   X-Axis Data 0 */
// #define DATAX1			0x33	/* R   X-Axis Data 1 */
// #define DATAY0			0x34	/* R   Y-Axis Data 0 */
// #define DATAY1			0x35	/* R   Y-Axis Data 1 */
// #define DATAZ0			0x36	/* R   Z-Axis Data 0 */
// #define DATAZ1			0x37	/* R   Z-Axis Data 1 */
// #define FIFO_CTL		0x38	/* R/W FIFO control */
// #define FIFO_STATUS		0x39	/* R   FIFO status */
	pr_err("%s: Devid =0%x\n",__func__,adxl345b_smbus_read(accel->client,DEVID));

	adxl345b_smbus_write(accel->client,THRESH_TAP,0);
	adxl345b_smbus_write(accel->client,OFSX,0);
	adxl345b_smbus_write(accel->client,OFSY,0);
	adxl345b_smbus_write(accel->client,OFSZ,0);
	adxl345b_smbus_write(accel->client,DUR,0);
	adxl345b_smbus_write(accel->client,LATENT,0);
	adxl345b_smbus_write(accel->client,WINDOW,0);
	adxl345b_smbus_write(accel->client,THRESH_ACT,0);
	adxl345b_smbus_write(accel->client,THRESH_INACT,0);
	adxl345b_smbus_write(accel->client,TIME_INACT,0);
	adxl345b_smbus_write(accel->client,ACT_INACT_CTL,0x0);
	adxl345b_smbus_write(accel->client,THRESH_FF,0);
	adxl345b_smbus_write(accel->client,TIME_FF,0);
	adxl345b_smbus_write(accel->client,TAP_AXES,0);
	adxl345b_smbus_write(accel->client,BW_RATE,0x0D);
	
	adxl345b_smbus_write(accel->client,INT_ENABLE,(DATA_READY));
	adxl345b_smbus_write(accel->client,INT_MAP,0);
	adxl345b_smbus_write(accel->client,FIFO_CTL,0);
	adxl345b_smbus_write(accel->client,POWER_CTL,PCTL_MEASURE);
	adxl345b_smbus_read(accel->client,INT_SOURCE);

	 adxl345b_smbus_read(accel->client,DATAX0);
	// adxl345b_smbus_read(accel->client,DATAX1);
	
	// adxl345b_smbus_read(accel->client,DATAY0);
	// adxl345b_smbus_read(accel->client,DATAY1);

	// adxl345b_smbus_read(accel->client,DATAZ0);
	// adxl345b_smbus_read(accel->client,DATAZ1);
	
	
		return 0;
}
static int adxl345b_input_dev_init(struct adxl345b_data* accel)
{
	int err;
	pr_info("%s: main\n",__func__);
	accel->input_dev = input_allocate_device();
	if(!accel->input_dev)
	{
		err = -ENOMEM;
		dev_err(&accel->client->dev, "input device allocate failed\n");
		goto exit_input_device_alloc;
	}
	accel->input_dev->name = "adxl345b";
	accel->input_dev->id.product = 345;
	accel->input_dev->id.bustype = BUS_I2C;
	accel->input_dev->dev.parent = &accel->client->dev;
	accel->input_dev->phys = "adxl345b/input0";


		//accel->input_dev->open = adxl345b_open;
	//accel->input_dev->close = adxl345b_close;
	__set_bit(EV_ABS,accel->input_dev->evbit);
	__set_bit(ABS_X, accel->input_dev->absbit);
	__set_bit(ABS_Y, accel->input_dev->absbit);
	__set_bit(ABS_Z, accel->input_dev->absbit);

	input_set_drvdata(accel->input_dev,accel);
	input_set_abs_params(accel->input_dev, ABS_X,
                             -512, 512, 0, 0);
 	input_set_abs_params(accel->input_dev, ABS_Y,
                             -512, 512, 0, 0);
 	input_set_abs_params(accel->input_dev, ABS_Z,
                             -512, 512, 0, 0);

 	pr_warning("%s, accel %p \n",__func__,accel);
 	err = request_irq(accel->irq,adxl345b_irq_callback,(IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING),"adxl345b_irq",accel);
	if(err){
		pr_err("%s: irq request failed: %d\n", __func__, err);
		err = -ENODEV;
		goto exit_irq_request;
	} 
	err = input_register_device(accel->input_dev);
	if (err) {
		pr_err("%s:unable to register input device %s\n",__func__, accel->input_dev->name);
		err = -ENODEV;
		goto exit_register_device;
	}
	err = adxl345b_init_chip(accel);
	if (err < 0){
		goto exit_init_chip;
	}

	return 0;
exit_init_chip:
	input_unregister_device(accel->input_dev);
exit_register_device:
	free_irq(accel->irq, accel);
exit_irq_request:
	input_free_device(accel->input_dev);
exit_input_device_alloc:
	return err;
}
static int adxl345b_probe(struct i2c_client *client,
                           const struct i2c_device_id *id)
{
	struct adxl345b_data *accel;
	int err = -1;
	pr_info("%s: Main\n",__func__);

	if(!i2c_check_functionality(client->adapter,I2C_FUNC_I2C))
	{
		pr_err("%s client not i2c capable\n",__func__);
		err = -ENODEV;
		goto exit_no_i2c_capability;
	}
	if (!client->irq){
		pr_err("%s no IRQ?\n",__func__);
		err = -ENODEV;
		goto exit_no_irq;
	}
	accel = kzalloc(sizeof(*accel), GFP_KERNEL);
	if (accel == NULL) {
		pr_err("%s:failed to allocate memory for module data\n",__func__);
		err = -ENOMEM;
		goto exit_no_enough_memory;
	}
	accel->irq = client->irq;
	accel->client = client;
	INIT_WORK(&accel->irq_work, accel_irq_worker);
	i2c_set_clientdata(client, accel);

	err = adxl345b_input_dev_init(accel);
	if (err){
		pr_err("%s: input dev init failed\n",__func__);
		goto exit_input_dev_init;
	}
	err = misc_register(&adxl345b_misc_device);
	if (err){
		pr_err("%s: misc register failed\n",__func__);
		goto exit_register_misc_device;
	}
	return 0;
exit_register_misc_device:
	adxl345b_input_dev_shutdown(accel);
exit_input_dev_init:
	kfree(accel);
exit_no_enough_memory:
exit_no_irq:
exit_no_i2c_capability:
	return err;
}
static int __devexit adxl345b_remove(struct i2c_client *client)
{
	struct adxl345b_data *accel = i2c_get_clientdata(client);
	misc_deregister(&adxl345b_misc_device);
 	adxl345b_input_dev_shutdown(accel);
	flush_work_sync(&accel->irq_work);
	kfree(accel);
	return 0;
}
static int adxl345b_resume(struct i2c_client *client)
{
        return 0;
}

static int adxl345b_suspend(struct i2c_client *client, pm_message_t mesg)
{
        return 0;
}
MODULE_DEVICE_TABLE(i2c, adxl345b_id);

static struct i2c_driver adxl345b_driver = {
	.driver = {
		.name = "adxl345b",
		.owner = THIS_MODULE,
	},
	.probe 		= adxl345b_probe,
	.suspend 	= adxl345b_suspend,
	.resume 	= adxl345b_resume,
	.remove 	= __exit_p(adxl345b_remove),
	.id_table 	= adxl345b_id,
};
static int __init adxl345b_init(void)
{
        pr_info("adxl345b accelerometr driver start \n");
        return i2c_add_driver(&adxl345b_driver);
}

static void __exit adxl345b_exit(void)
{
        i2c_del_driver(&adxl345b_driver);
        return;
}
module_init(adxl345b_init);
module_exit(adxl345b_exit);

MODULE_DESCRIPTION("adxl345b gyroscope driver");
MODULE_AUTHOR("CyberTech Roman Meshkevich");
MODULE_LICENSE("GPL");