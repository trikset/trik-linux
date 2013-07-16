#include <linux/errno.h>
#include <linux/input.h>	/* BUS_SPI */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/mm.h>


#define F_STATUS			0x00		/* Read Only*/

#define OUT_X_MSB			0x01		/* Read Only*/
#define OUT_X_LSB			0x02		/* Read Only*/
	
#define OUT_Y_MSB			0x03		/* Read Only*/
#define OUT_Y_LSB			0x04		/* Read Only*/

#define OUT_Z_MSB			0x05		/* Read Only*/
#define OUT_Z_LSB			0x06		/* Read Only*/

#define F_SETUP				0x09		/* R/W 		*/
#define TRIG_CFG			0x0A		/* R/W 		*/

#define SYSMOD				0x0B		/* Read Only*/
#define INT_SOURCE			0x0C		/* Read Only*/
#define WHO_AM_I			0x0D		/* Read Only*/

#define XYZ_DATA_CFG		0x0E		/* R/W 		*/
#define HP_FILTER_CUTOFF	0x0F		/* R/W 		*/
#define PL_STATUS			0x10		/* Read Only*/
#define PL_CFG				0x11		/* R/W 		*/
#define PL_COUNT			0x12		/* R/W 		*/
#define PL_BF_ZCOMP			0x13		/* R/W 		*/
#define P_L_THS_REG			0x14		/* R/W 		*/

#define FF_MT_CFG			0x15		/* R/W 		*/
#define FF_MT_SRC			0x16		/* Read Only*/
#define FF_MT_THS			0x17		/* R/W 		*/
#define FF_MT_COUNT			0x18		/* R/W 		*/

#define TRANSIENT_CFG		0x1D		/* R/W 		*/
#define TRANSIENT_SRC		0x1E		/* Read Only*/
#define TRANSIENT_THS		0x1F		/* R/W 		*/
#define TRANSIENT_COUNT		0x20		/* R/W 		*/

#define PULSE_CFG			0x21		/* R/W 		*/
#define PULSE_SRC			0x22		/* Read Only*/
#define PULSE_THSX			0x23		/* R/W 		*/
#define PULSE_THSY			0x24		/* R/W 		*/
#define PULSE_THSZ			0x25		/* R/W 		*/
#define PULSE_TMLT			0x26		/* R/W 		*/
#define PULSE_LTCY			0x27		/* R/W 		*/
#define PULSE_WIND			0x28		/* R/W 		*/
#define ASPL_COUNT			0x29		/* R/W 		*/

#define	CTRL_REG1			0x2A		/* R/W 		*/
#define CTRL_REG2			0x2B		/* R/W 		*/
#define CTRL_REG3			0x2C		/* R/W 		*/
#define CTRL_REG4			0x2D		/* R/W 		*/
#define CTRL_REG5			0x2E		/* R/W 		*/

#define OFF_X				0x2F		/* R/W 		*/
#define OFF_Y				0x30		/* R/W 		*/
#define OFF_Z				0x31		/* R/W 		*/


#define MMA8450_ID			0xc6
#define MMA8451_ID			0x1a
#define MMA8452_ID			0x2a
#define MMA8453_ID			0x3a

// struct mma8451q_platform_data
// {
// 	int gpio_int2;
// };
struct mma8451q_accel_data
{
	u16 x;
	u16 y;
	u16 z;
};
struct mma8451q_driver_data
{
	struct i2c_client * client;
	struct input_dev* input_dev;
	//struct mma8451q_platform_data* pdata;
	u8 client_irq;
	struct work_struct irq_work;
};

static u8 mma8451q_read(struct mma8451q_driver_data* drv_data,u8 reg){
	return i2c_smbus_read_byte_data(drv_data->client, reg);
}

static u8 mma8451q_write(struct mma8451q_driver_data* drv_data,u8 reg,u8 value){
	return i2c_smbus_write_byte_data(drv_data->client, reg, value);
}
static u8 mma8451q_read_block(struct mma8451q_driver_data* drv_data,u8 reg,void * buf,size_t size)
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


static void mma8451q_input_dev_remove(struct mma8451q_driver_data* chip)
{

}
static int mma8451q_input_dev_init(struct mma8451q_driver_data* chip)
{
	return 0;
}
static void mma8451q_irq_worker(struct work_struct *work)
{
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
	u8 device_id;
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
