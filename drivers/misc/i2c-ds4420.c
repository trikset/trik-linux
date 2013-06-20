#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>

#define _VERSION       "v2"
#define MMA7660FC_DATE          "11.01.2013"

#define CTRL_REGISTER 	0xF8

#define DS4420_VALUE_MASK	0x1F
#define DS4420_MODE_MASK	0x80
#define DS4420_MUTE_MASK	0x20
#define MIN_VOLUME		0
#define MAX_VOLUME 		20

struct ds4420_data {
	bool mode;
	bool mute;
	int value;
	struct i2c_client* client;
};

static inline int i2c_write(struct i2c_client *client, int reg, int  value){
	int ret;
	ret = i2c_smbus_write_byte_data(client, reg, value);
	return ret;
}
static inline int i2c_read(struct i2c_client *client, int reg){
	int value;
        value = i2c_smbus_read_byte_data(client, reg);;
        return value;
}
static int  ds4420_set_mode(struct i2c_client* client,bool mode){
	int ret = 0;
 	u8 data;
        if (mode)
                data = i2c_read(client, CTRL_REGISTER) | DS4420_MODE_MASK;
        else
                data = i2c_read(client, CTRL_REGISTER) & ~DS4420_MODE_MASK;
        ret = i2c_write(client,CTRL_REGISTER,data);

	return ret;
}
static int  ds4420_set_mute(struct i2c_client* client,bool mute){
        int ret = 0;
        u8 data;
	if (mute)
                data = i2c_read(client, CTRL_REGISTER) | DS4420_MUTE_MASK;
        else
                data = i2c_read(client, CTRL_REGISTER) & ~DS4420_MUTE_MASK;
	ret = i2c_write(client,CTRL_REGISTER,data);
        return ret;
}
#if 0
static bool ds4420_get_mode(struct i2c_client* client){
	u8 data;
        data = i2c_read(client, CTRL_REGISTER) & DS4420_MODE_MASK;
	return data;
}
static bool ds4420_get_mute(struct i2c_client* client){
	u8 data;
        data = i2c_read(client, CTRL_REGISTER) & DS4420_MUTE_MASK;
        return data;
}
static int ds4420_get_volume(struct i2c_client* client){
	u8 data;
	data = i2c_read(client, CTRL_REGISTER) & DS4420_VALUE_MASK;
	return data;
}
#endif
static int ds4420_set_volume(struct i2c_client* client,int value){
	int ret;
        u8 data;
        data = i2c_read(client, CTRL_REGISTER) & ~DS4420_VALUE_MASK;
	ret = i2c_write(client,CTRL_REGISTER,data | value);
        return ret;
}


static const struct i2c_device_id ds4420_id[] = {
        {"ds4420", 0},
        {},
};
MODULE_DEVICE_TABLE(i2c, ds4420_id);

static ssize_t ds4420_mode_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds4420_data* info = dev_get_drvdata(dev);

	return snprintf(buf,PAGE_SIZE,
			"Mode: %s\n",(info->mode)?"Standby":"Normal");
}
static ssize_t ds4420_mute_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds4420_data* info = dev_get_drvdata(dev);
    	return snprintf(buf,PAGE_SIZE,
                        "Device is %s\n",(info->mute)?"muted":"unmuted");;
}
static ssize_t ds4420_volume_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds4420_data* info = dev_get_drvdata(dev);
 	return snprintf(buf,PAGE_SIZE,
			"%i\n"
                        "Volume: 0..20\n",info->value);
}

static ssize_t ds4420_mute_store(struct device *dev, struct device_attribute *attr,
                    const char *buf, size_t count){
	struct ds4420_data* info = dev_get_drvdata(dev);

	if (!strncmp(buf, "unmute", 6)) {
		info->mute = 0;
	}
	else if (!strncmp(buf,"mute",4)){
		info->mute = 1;
	}
	else
		return -EINVAL;
	ds4420_set_mute(info->client,info->mute);
    return count;
}

static ssize_t ds4420_volume_store(struct device *dev, struct device_attribute *attr,
                    const char *buf, size_t count)
{
	int ret;
        struct ds4420_data* info = dev_get_drvdata(dev);
	s32 value;
	ret = kstrtos32(buf, 0, &value);
	if ((value < MIN_VOLUME)||(value > MAX_VOLUME))
		return -EINVAL;
	ds4420_set_volume(info->client,(info->value=value));
    	return count;
}

static DEVICE_ATTR(mode, (S_IRUGO|S_IWUSR), ds4420_mode_read ,NULL);
static DEVICE_ATTR(mute, (S_IRUGO|S_IWUSR), ds4420_mute_read ,ds4420_mute_store);
static DEVICE_ATTR(volume, (S_IRUGO|S_IWUSR), ds4420_volume_read ,ds4420_volume_store);


static const struct attribute *ds4420_manage_attrs[] = {
    &dev_attr_mode.attr,
    &dev_attr_mute.attr,
    &dev_attr_volume.attr,
    NULL,
};
static const struct attribute_group ds4420_manage_attrs_group = {
    .attrs = (struct attribute **) ds4420_manage_attrs,
};


static int ds4420_probe(struct i2c_client *client, const struct i2c_device_id *id){
	int res=0;
	struct ds4420_data* i2c_data;
 	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE_DATA)){
                res=-ENODEV;
                return res;
        }
	i2c_data = kzalloc(sizeof(struct ds4420_data), GFP_KERNEL);
	if(!i2c_data){
		res = -ENOMEM;
		return res;
	}
	ds4420_set_mode(client,(i2c_data->mode=0));
	ds4420_set_mute(client,(i2c_data->mute=0));
	ds4420_set_volume(client,(i2c_data->value=MAX_VOLUME));

	i2c_set_clientdata(client, i2c_data);
	i2c_data->client = client;
	res = sysfs_create_group(&client->dev.kobj, &ds4420_manage_attrs_group);
  	if (res){
            pr_err("mma7660fc_probe: Unable to create sysfs \n");
            goto exit_sysfs_device_failed;
	}

	return res;
exit_sysfs_device_failed:
	kfree(i2c_data);
	return res;
}
static int ds4420_resume(struct i2c_client *client){
	return 0;
}
static int ds4420_suspend(struct i2c_client *client, pm_message_t mesg){
	return 0;
}
static int ds4420_remove(struct i2c_client *client){
	struct ds4420_data *data = i2c_get_clientdata(client);
	sysfs_remove_group(&client->dev.kobj, &ds4420_manage_attrs_group);
	kfree(data);
	return 0;
}

static struct i2c_driver ds4420_driver = {
        .driver = {
                .owner  = THIS_MODULE,
                .name   = "ds4420",
        },
        .probe          = ds4420_probe,
        .suspend        = ds4420_suspend,
        .resume         = ds4420_resume,
        .remove         = __exit_p(ds4420_remove),
        .id_table       = ds4420_id,
};

static int __init ds4420_init(void){
	int res = 0;
	if ((res = i2c_add_driver(&ds4420_driver))) {
		return res;
	}
	return res;
}

static void __exit ds4420_exit(void){
        i2c_del_driver(&ds4420_driver);
}

MODULE_DESCRIPTION("I2C Amplifier DS4420");
MODULE_LICENSE("GPL");

module_init(ds4420_init)
module_exit(ds4420_exit)

