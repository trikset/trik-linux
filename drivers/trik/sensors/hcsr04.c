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
	__kernel_time_t time_us;
	long dist_mm;
};

struct hcsr04_drvdata {
	struct trik_jdx_platform_data* data;
	struct delayed_work echo_work;
	struct platform_device* platform_device;
	struct hcsr04_irq_data irq_data;
	struct input_dev *input_dev;
	
	wait_queue_head_t wait;
	spinlock_t lock; 
	int irq;
	int enable;
};
static irqreturn_t hcsr04_irq_callback(int irq, void *dev_id){
	struct hcsr04_drvdata *drv_data = dev_id;
	wake_up_interruptible(&drv_data->wait);
	return IRQ_HANDLED;
}
static void hcsr04_echo_worker(struct work_struct *work){
	struct hcsr04_drvdata *drv_data = container_of(work, struct hcsr04_drvdata, echo_work.work);
	struct timespec start;
	struct timespec stop;
	struct timespec diff;
	int retval;
	gpio_set_value(drv_data->data->gpio_d1e,1);
	gpio_set_value(drv_data->data->gpio_d1a,1);
	udelay(10);
	gpio_set_value(drv_data->data->gpio_d1a,0);

	retval =  wait_event_interruptible_timeout(drv_data->wait,gpio_get_value(drv_data->data->gpio_d1b),msecs_to_jiffies(50));
	if (retval>1){
		getnstimeofday(&start);
		retval =  wait_event_interruptible_timeout(drv_data->wait,!gpio_get_value(drv_data->data->gpio_d1b),msecs_to_jiffies(50));
		if(retval>1){
			getnstimeofday(&stop);
			diff = timespec_sub(stop, start);
		 	drv_data->irq_data.time_us = diff.tv_nsec /NSEC_PER_USEC;
		 	drv_data->irq_data.dist_mm = drv_data->irq_data.time_us/58;
			input_report_abs(drv_data->input_dev, ABS_DISTANCE, drv_data->irq_data.dist_mm);
			input_report_abs(drv_data->input_dev, ABS_MISC, drv_data->irq_data.time_us);
			input_sync(drv_data->input_dev);
#if 0
			retval = 60 - retval;
#endif
		} 
	}
	else {
		input_report_abs(drv_data->input_dev, ABS_DISTANCE, -1);
		input_report_abs(drv_data->input_dev, ABS_MISC, -1);
		input_sync(drv_data->input_dev);
	}
#if 0
	else {
		retval = 10;
	}
#endif 
	schedule_delayed_work (&drv_data->echo_work,msecs_to_jiffies(10));	
}
static int hcsr04_open(struct input_dev *dev)
{
	struct hcsr04_drvdata *drv_data = input_get_drvdata(dev);
	//enable_irq(drv_data->irq);
	schedule_delayed_work(&drv_data->echo_work,msecs_to_jiffies(0));
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

	set_bit(EV_ABS, drv_data->input_dev->evbit);
	input_set_abs_params(drv_data->input_dev, ABS_DISTANCE,
                             -1, 10000, 0, 0);
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

	return 0;
exit_register_irq:
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