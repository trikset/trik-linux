	#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/posix-clock.h>
#include <linux/trik-jdx.h>

struct hcsr04_irq_data{
	ktime_t time_stamp[2];
	int count;
	int start;
};

struct hcsr04_drvdata {
	struct trik_jdx_platform_data* data;
	struct delayed_work echo_work;
	struct platform_device* platform_device;
	struct hcsr04_irq_data irq_data;
	struct input_dev *input_dev;
	int cycle_period; // min 50 mS - 1000 mS
	wait_queue_head_t wait;
	struct mutex mutex;
	int irq;
	int enable;
};
static irqreturn_t hcsr04_irq_callback(int irq, void *dev_id){
	struct hcsr04_drvdata *drv_data = dev_id;
	ktime_t timeStamp = ktime_get();
	if (drv_data->irq_data.start){
		if ( drv_data->irq_data.count < 2 )
			drv_data->irq_data.time_stamp[drv_data->irq_data.count] = timeStamp;
	
		++drv_data->irq_data.count;
		if ( drv_data->irq_data.count > 1 )
			wake_up_interruptible(&drv_data->wait);
	}
	return IRQ_HANDLED;
}
static void hcsr04_echo_worker(struct work_struct *work){
	struct hcsr04_drvdata *drv_data = container_of(work, struct hcsr04_drvdata, echo_work.work);
	struct timeval elapsed_tv;  /* elapsed time as timeval */
	ktime_t elapsed_kt;			/* used to store elapsed time */
	int rangeComplete = 0;      /* indicates successful range finding */
	int retval;


	drv_data->irq_data.count = 0;
	drv_data->irq_data.start = 0;
	memset( &drv_data->irq_data.time_stamp, 0, sizeof(drv_data->irq_data.time_stamp) );


	gpio_set_value(drv_data->data->gpio_d1a,0);
	udelay(2);
        gpio_set_value(drv_data->data->gpio_d1a,1);
	udelay(10);
	drv_data->irq_data.start =1;
        gpio_set_value(drv_data->data->gpio_d1a,0);

	wait_event_interruptible_timeout(
			drv_data->wait,
			drv_data->irq_data.count == 2,
			msecs_to_jiffies(50));

 	rangeComplete = (drv_data->irq_data.count == 2);
	if ( rangeComplete ) {
		/* Calculate pulse length */
		elapsed_kt = ktime_sub( drv_data->irq_data.time_stamp[1], drv_data->irq_data.time_stamp[0] );
		elapsed_tv = ktime_to_timeval( elapsed_kt );
	}

	if ( rangeComplete ) {
		input_report_abs(drv_data->input_dev, ABS_DISTANCE, (((int)elapsed_tv.tv_usec)<<4)/931);
		input_report_abs(drv_data->input_dev, ABS_MISC, elapsed_tv.tv_usec);
		input_sync(drv_data->input_dev);
	}
	else {
		input_report_abs(drv_data->input_dev, ABS_DISTANCE, -1);
		input_report_abs(drv_data->input_dev, ABS_MISC, -1);
		input_sync(drv_data->input_dev);
	}

	schedule_delayed_work (&drv_data->echo_work,msecs_to_jiffies(drv_data->cycle_period));	
}

static ssize_t hcsr04_odr_show(	struct device *dev,
									struct device_attribute *attr, 
									char *buf	){
	
	struct hcsr04_drvdata *drv_data = dev_get_drvdata(dev);

	return sprintf(buf,"%d\n",drv_data->cycle_period);
}
static ssize_t hcsr04_odr_store(struct device *dev,
				    				struct device_attribute *attr,
				    				const char *buf, 
				    				size_t count	){
	struct hcsr04_drvdata *drv_data = dev_get_drvdata(dev);
	unsigned long rate;
	rate = simple_strtoul(buf, NULL, 10);
	
	if (rate >= 50 && rate <= 1000) {
		drv_data->cycle_period = rate;		
	}
	return count;
}
static DEVICE_ATTR(odr_selection, S_IRUGO|S_IWUSR|S_IWGRP, hcsr04_odr_show, hcsr04_odr_store);
//static DEVICE_ATTR(state, S_IRUGO|S_IWUSR|S_IWGRP, hcsr04_state_show,hcsr04_state_store);

static struct attribute *hcsr04_attributes[] = {
	&dev_attr_odr_selection.attr,
//	&dev_attr_state.attr,
	NULL
};

static const struct attribute_group hcsr04_attr_group = {
	.attrs = hcsr04_attributes,
};

static int hcsr04_open(struct input_dev *dev)
{
	struct hcsr04_drvdata *drv_data = input_get_drvdata(dev);
	//enable_irq(drv_data->irq);
	schedule_delayed_work(&drv_data->echo_work, 0);
    return 0;
}

static void hcsr04_close(struct input_dev *dev)
{
	struct hcsr04_drvdata *drv_data = input_get_drvdata(dev);
	//disable_irq(drv_data->irq);
	cancel_delayed_work_sync(&drv_data->echo_work);
}
static int hcsr04_probe(struct platform_device *pdev)
{
	int ret = 0;

	struct hcsr04_drvdata	*drv_data;
	struct trik_jdx_platform_data *data = (struct trik_jdx_platform_data*)platform_get_drvdata(pdev);

	if (!data){
		pr_err("%s: no platform data\n",__func__);
		ret = -EINVAL;
		goto exit_plt_data_failed;
	}
	
	drv_data = kzalloc(sizeof(*drv_data), GFP_KERNEL);
	if (drv_data == NULL){
		pr_err(" %s: drv_data allocation memory failed\n",__func__);
		ret = -ENOMEM;
		goto exit_alloc_failed;
	}
	
	INIT_DELAYED_WORK(&drv_data->echo_work, hcsr04_echo_worker);
	init_waitqueue_head(&drv_data->wait);
	mutex_init(&drv_data->mutex);

	drv_data->data = data;
	drv_data->platform_device = pdev;
	drv_data->input_dev = input_allocate_device();
	if (!drv_data->input_dev){
		ret = -ENOMEM;
		pr_err("%s: input device allocate failed\n",__func__);
		goto exit_input_device_alloc;
	}
	drv_data->input_dev->name		= drv_data->platform_device->name;
	drv_data->input_dev->dev.parent	= &drv_data->platform_device->dev;
	drv_data->input_dev->open		= hcsr04_open;
	drv_data->input_dev->close	 	= hcsr04_close;
	drv_data->cycle_period = 100;

	set_bit(EV_ABS, drv_data->input_dev->evbit);
	input_set_abs_params(drv_data->input_dev, ABS_DISTANCE,
                             -1, 400, 0, 0);
	input_set_abs_params(drv_data->input_dev, ABS_MISC,
                             0, INT_MAX, 0, 0);
	input_set_drvdata(drv_data->input_dev,drv_data);
	platform_set_drvdata(pdev,drv_data);

	ret = input_register_device(drv_data->input_dev);
    if (ret) {
        pr_err("%s: register input device failed %s\n",__func__, drv_data->input_dev->name);
        ret = -ENODEV;
        goto exit_register_device;
    }
    drv_data->irq = gpio_to_irq(drv_data->data->gpio_d1b);

    platform_set_drvdata(pdev, drv_data);
	ret = sysfs_create_group(&pdev->dev.kobj, &hcsr04_attr_group);
	if (ret) {
		pr_err("%s: sys create group failed\n",__func__);
		goto sysfs_create_group_failed;
	}


	ret = request_irq(drv_data->irq,
						hcsr04_irq_callback,
						(IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING),
						drv_data->platform_device->name,
						drv_data);
	if (ret){
		pr_err("%s: register irq failed %s\n",__func__, drv_data->platform_device->name);
        ret = -ENODEV;
        goto exit_register_irq;
	}

        udelay(10);
	return 0;
exit_register_irq:
	sysfs_remove_group(&pdev->dev.kobj, &hcsr04_attr_group);
sysfs_create_group_failed:
	input_unregister_device(drv_data->input_dev);
exit_register_device:
	input_free_device(drv_data->input_dev);
exit_input_device_alloc:
	kfree(drv_data);
exit_alloc_failed:
exit_plt_data_failed:
	return ret;
}
static int hcsr04_remove(struct platform_device *pdev)
{

	struct hcsr04_drvdata *drv_data = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&drv_data->echo_work);
	if(flush_delayed_work(&drv_data->echo_work)){
		pr_err("%s: drv_data->echo_work was flushed\n",__func__);
	}
	// flush_workqueue(drv_data->work_queue);
	// destroy_workqueue(drv_data->work_queue);

	disable_irq(drv_data->irq);
	free_irq(drv_data->irq, drv_data);
	sysfs_remove_group(&pdev->dev.kobj, &hcsr04_attr_group);
	input_unregister_device(drv_data->input_dev);
	input_free_device(drv_data->input_dev);
	kfree(drv_data);
	return 0;
}
static struct platform_device_id hcsr04_ids[] = {
	{
		.name		= "trik_jd1",
	}, {
		.name		= "trik_jd2",
	}, 
	{ }
};
#define DRV_NAME 		"hcsr04"
#define DRV_VERSION 	"0.2"

static struct platform_driver hcsr04_driver = {
	.probe		= hcsr04_probe,
	.remove		= __devexit_p(hcsr04_remove),
	.id_table	= hcsr04_ids,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(hcsr04_driver);
MODULE_LICENSE("GPL");
