#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input-polldev.h>
#include <linux/of_device.h>
#include <linux/input.h>


/* register definitions */
#define MMA845X_STATUS			0x00
#define MMA845X_STATUS_ZXYDR	0x08

#define MMA845X_OUT_X_MSB		0x01
#define MMA845X_OUT_X_LSB		0x02

#define MMA845X_OUT_Y_MSB		0x03
#define MMA845X_OUT_Y_LSB		0x04

#define MMA845X_OUT_Z_MSB		0x05
#define MMA845X_OUT_Z_LSB		0x06


#define MMA845X_F_SETUP			0x0A
#define MMA845X_TRIG_CFG		0x0B
#define MMA845X_SYSMOD			0x0B
#define MMA845X_INT_SOURCE		0x0C
#define MMA845X_WHOAMI			0x0D

#define MMA845X_XYZ_DATA_CFG	0x0E
#define MMA845X_HPFILTERCUTOFF	0x0F

#define MMA845X_PL_STATUS		0x10
#define MMA845X_PL_CFG			0x11
#define MMA845X_PL_COUNT		0x12

#define MMA845X_PL_BF_ZCOMP		0x13
#define MMA845X_PL_L_THS_REG	0x14

#define MMA845X_FF_MT_CFG		0x15
#define MMA845X_FF_MT_SRC		0x16
#define MMA845X_FF_MT_THS		0x17
#define MMA845X_FF_MT_COUNT		0x18

#define MMA845X_TRANSIENT_THS	0x1F
#define MMA845X_TRANSIENT_COUNT	0x20

#define MMA845X_PULSE_CFG		0x21
#define MMA845X_PULSE_SRC		0x22

#define MMA845X_PULSE_THSX		0x23
#define MMA845X_PULSE_THSY		0x24
#define MMA845X_PULSE_THSY		0x25
#define MMA845X_PULSE_TMLT		0x26
#define MMA845X_PULSE_LTCY		0x27
#define MMA845X_PULSE_WIND		0x28

#define MMA845X_ASLP_COUNT		0x29

#define MMA845X_CTRL_REG1		0x2A
#define MMA845X_CTRL_REG2		0x2B
#define MMA845X_CTRL_REG3		0x2C
#define MMA845X_CTRL_REG4		0x2D
#define MMA845X_CTRL_REG5		0x2E

#define MMA845X_OFF_X			0x2F
#define MMA845X_OFF_Y			0x30
#define MMA845X_OFF_Z			0x31


#define MMA8451_ID				0x1A
#define MMA8452_ID				0x2A
#define MMA8453_ID				0x3A


enum {
	MODE_2G				=	0,
	MODE_4G,
	MODE_8G,
};

enum {
	MMA_STANDBY 		=	0,
	MMA_ACTIVED,
};
enum {
	SAMPLE_RATE_800		=	0,	
	SAMPLE_RATE_400,
	SAMPLE_RATE_200,
	SAMPLE_RATE_100,
	SAMPLE_RATE_50,
	SAMPLE_RATE_12_5,
	SAMPLE_RATE_6_25,
	SAMPLE_RATE_1_56,
};
struct mma845x_chip_params {
	u8 mode;
	u8 sample_rate_hz;
	u8 state;
};
struct mm845x_driver_data {
	struct i2c_client 	*client;
	struct input_dev	*input_dev;
	struct mutex 		data_lock;
	struct work_struct 	work;
	u8 					chip_id;
	struct mma845x_chip_params params;
	int 				fd_opened;
};
static char* mma845x_state_names[] = {
	"standby",
	"active",
};
static char* mma845x_names[] = {
   "mma8451",
   "mma8452",
   "mma8453",
};
static char* mma845x_modes_names[] = {
	"2g",
	"4g",
	"8g",
};
static char* mma845x_rate_names[] = {
	"800",
	"400",
	"200",
	"100",
	"50",
	"12.5",
	"6.25",
	"1.56"
};
static int mma845x_write_data(struct i2c_client *client, unsigned char reg, unsigned char data){
	return i2c_smbus_write_byte_data(client, reg, data);
};
static int mma845x_read_data(struct i2c_client *client, unsigned char reg){
	return i2c_smbus_read_byte_data(client, reg);
};

struct mma845x_data{
	short x;
	short y;
	short z; 
};
static char * mma845x_id2name(u8 id){
	int index = 0;
	if(id == MMA8451_ID)
		index = 0;
	else if(id == MMA8452_ID)
		index = 1;
	else if(id == MMA8453_ID)
		index = 2;
	return mma845x_names[index];
}


static ssize_t mma845x_sample_rate_hz_store(struct device *dev,
				    						struct device_attribute *attr,
				    						const char *buf, 
				    						size_t count){
	struct input_dev *input_dev = to_input_dev(dev);
	struct mm845x_driver_data *pdata = input_get_drvdata(input_dev);
	return 0;
}
static ssize_t mma845x_sample_rate_hz_show(	struct device *dev,
				   							struct device_attribute *attr, 
				   							char *buf	){
	struct input_dev *input_dev = to_input_dev(dev);
	struct mm845x_driver_data *pdata = input_get_drvdata(input_dev);
	int sample_rate_hz = 0;
	mutex_lock(&pdata->data_lock);
	sample_rate_hz = (int) pdata->params.sample_rate_hz;
	mutex_unlock(&pdata->data_lock);

	return sprintf(buf,"%s\n",mma845x_rate_names[sample_rate_hz]);
}
static ssize_t mma845x_mode_g_store(struct device *dev,
				    				struct device_attribute *attr,
				    				const char *buf, 
				    				size_t count	){
	struct input_dev *input_dev = to_input_dev(dev);
	struct mm845x_driver_data *pdata = input_get_drvdata(input_dev);
	return 0;
}
static ssize_t mma845x_mode_g_show(	struct device *dev,
									struct device_attribute *attr, 
									char *buf	){
	struct input_dev *input_dev = to_input_dev(dev);
	struct mm845x_driver_data *pdata = input_get_drvdata(input_dev);
	int mode;
	mutex_lock(&pdata->data_lock);
	mode = (int) pdata->params.mode;
	mutex_unlock(&pdata->data_lock);
	return sprintf(buf,"%s\n",mma845x_modes_names[mode]);
}
static ssize_t mma845x_enable_store(struct device *dev,
				    				struct device_attribute *attr,
				    				const char *buf, 
				    				size_t count	){
	struct input_dev *input_dev = to_input_dev(dev);
	struct mm845x_driver_data *pdata = input_get_drvdata(input_dev);
	return 0;

}
static ssize_t mma845x_enable_show(struct device *dev,
									struct device_attribute *attr, 
									char *buf	){
	struct input_dev *input_dev = to_input_dev(dev);
	struct mm845x_driver_data *pdata = input_get_drvdata(input_dev);
	int state;
	mutex_lock(&pdata->data_lock);
	state = (int) pdata->params.state;
	mutex_unlock(&pdata->data_lock);
	return sprintf(buf,"%s\n",mma845x_state_names[state]);

}

static DEVICE_ATTR(sample_rate_hz, S_IRUGO|S_IWUSR|S_IWGRP,mma845x_sample_rate_hz_show,mma845x_sample_rate_hz_store);
static DEVICE_ATTR(mode_g, S_IRUGO|S_IWUSR|S_IWGRP,mma845x_mode_g_show,mma845x_mode_g_store);
static DEVICE_ATTR(enable_device, S_IRUGO|S_IWUSR|S_IWGRP,mma845x_enable_show,mma845x_enable_store);

static struct attribute *mma845x_attributes[] = {
	&dev_attr_sample_rate_hz.attr,
	&dev_attr_mode_g.attr,
	&dev_attr_enable_device.attr,
	NULL
};

static const struct attribute_group mma845x_attr_group = {
	.attrs = mma845x_attributes,
};

static void mma845x_work_func(struct work_struct *work)
{
	struct mm845x_driver_data *pdata = container_of(work, struct mma845x_data, work);

	if (mma845x_read_data())
	input_report_abs(pdata->input_dev, ABS_X, data.x);
	input_report_abs(pdata->input_dev, ABS_Y, data.y);
	input_report_abs(pdata->input_dev, ABS_Z, data.z);
	input_sync(pdata->input_dev);
	return;
}
static int __devinit mma845x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int res;
	int chip_id;
	struct i2c_adapter *adapter;
	struct mm845x_driver_data * pdata;

	adapter = to_i2c_adapter(client->dev.parent);
	res 	= i2c_check_functionality(adapter,
					 				I2C_FUNC_SMBUS_BYTE |
					 				I2C_FUNC_SMBUS_BYTE_DATA);
	if (!res){
		pr_err("%s: client doesn't have i2c functionality\n", __func__);
		goto i2c_check_functionality_failed;
	}
	chip_id = mma845x_read_data(client,MMA845X_WHOAMI);
	if (chip_id != MMA8451_ID && chip_id != MMA8452_ID && chip_id != MMA8453_ID){
			pr_err("%s: chip_id 0x%02x is not equal to 0x%02x, 0x%02x, 0x%02x\n",
				__func__, chip_id, MMA8451_ID, MMA8452_ID, MMA8453_ID);
			res = -EINVAL;
			goto chip_read_failed;
	}


	pdata  = kzalloc(sizeof(struct mm845x_driver_data), GFP_KERNEL);
	if (!pdata){
		res = -ENOMEM;
		pr_err("%s: allocation driver data failed\n", __func__);
		goto allocation_failed;
	}
	pdata->client 					= client;
	pdata->chip_id 					= chip_id;
	pdata->params.mode 				= MODE_2G;
	pdata->params.sample_rate_hz 	= SAMPLE_RATE_50;
	pdata->params.state 			= MMA_STANDBY;

	mutex_init(&pdata->data_lock);
	i2c_set_clientdata(client,pdata);
	//device init
	//misc device create
	
	pdata->input_dev = input_allocate_device();
	if (!pdata->input_dev) {
		pr_err("%s: input_allocate_device failed\n",__func__);
		goto input_allocate_device_failed;
	}
	pdata->input_dev->name 			= "mma845x";
	pdata->input_dev->id.bustype 	= BUS_I2C;
	pdata->input_dev->uniq 			= mma845x_id2name(pdata->chip_id);

	set_bit(EV_ABS, pdata->input_dev->evbit);
	input_set_abs_params(pdata->input_dev, ABS_X, -0x1fff, 0x1fff, 0, 0);
	input_set_abs_params(pdata->input_dev, ABS_Y, -0x1fff, 0x1fff, 0, 0);
	input_set_abs_params(pdata->input_dev, ABS_Z, -0x1fff, 0x1fff, 0, 0);
	
	res = input_register_device(pdata->input_dev);
	if (res) {
		pr_err("%s: input register device failed\n",__func__);
		goto inpput_register_failed;
	}
	input_set_drvdata(pdata->input_dev, pdata);
	INIT_WORK(&pdata->work, mma845x_work_func);

	res = sysfs_create_group(&pdata->input_dev->dev.kobj, &mma845x_attr_group);
	if (res) {
		pr_err("%s: sys create group failed\n",__func__);
		goto sysfs_create_group_failed;
	}
	return 0;
sysfs_create_group_failed:
	input_unregister_device(pdata->input_dev);
input_allocate_device_failed:
	input_free_device(pdata->input_dev);
inpput_register_failed:
	kfree(pdata);
allocation_failed:
chip_read_failed:
i2c_check_functionality_failed:
	return res;
}
static int __devexit mma845x_remove(struct i2c_client *client)
{
	struct mm845x_driver_data *pdata = i2c_get_clientdata(client);
	if(pdata){
		sysfs_remove_group(&pdata->input_dev->dev.kobj, &mma845x_attr_group);
		input_unregister_device(pdata->input_dev);
		input_free_device(pdata->input_dev);
		kfree(pdata);
	}
	return 0;
}
static int mma845x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
static int mma845x_resume(struct i2c_client *client)
{
	return 0;
}
static const struct i2c_device_id mma845x_id[] = {
	{"mma845x", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, mma845x_id);
static struct i2c_driver mma845x_driver = {
	.driver 	= {
		   			.name = "mma845x",
		   			.owner = THIS_MODULE,
		   		},
	.suspend	= mma845x_suspend,
	.resume		= mma845x_resume,
	.probe		= mma845x_probe,
	.remove		= __devexit_p(mma845x_remove),
	.id_table	= mma845x_id,
};
static int __init mma845x_init(void)
{
	int res;

	res = i2c_add_driver(&mma845x_driver);
	if (res < 0) {
		printk(KERN_INFO "add mma845x i2c driver failed\n");
		return -ENODEV;
	}
	return res;
}

static void __exit mma845x_exit(void)
{
	i2c_del_driver(&mma845x_driver);
}

MODULE_AUTHOR("Roman Meshkevich	");
MODULE_DESCRIPTION("MMA845X 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");

module_init(mma845x_init);
module_exit(mma845x_exit);