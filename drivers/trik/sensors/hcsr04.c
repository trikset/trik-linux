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
	int count;
	struct timespec monotime[2];
	__kernel_time_t time_us;
	long dist_mm;
};

struct hcsr04_drvdata {
	struct trik_jdx_platform_data* data;
	struct delayed_work echo_work;
	struct platform_device* platform_device;
	int irq;
	int enable;
	struct hcsr04_irq_data irq_data;
	struct input_dev *input_dev;
};
static irqreturn_t hcsr04_irq_callback(int irq, void *dev_id){
	struct hcsr04_drvdata *drv_data = dev_id;
	if(drv_data){
    	getnstimeofday(&drv_data->irq_data.monotime[drv_data->irq_data.count]);
		drv_data->irq_data.count++;
	}
	return IRQ_HANDLED;
}
static void hcsr04_echo_worker(struct work_struct *work){
	struct hcsr04_drvdata *drv_data = container_of(work, struct hcsr04_drvdata, echo_work.work);
	disable_irq(drv_data->irq);
	if (drv_data->irq_data.count == 2){
		struct timespec diff = timespec_sub(drv_data->irq_data.monotime[1], drv_data->irq_data.monotime[0]);
		drv_data->irq_data.time_us = diff.tv_nsec /NSEC_PER_USEC;
		drv_data->irq_data.dist_mm = drv_data->irq_data.time_us/58;
	} 
	else {
		 drv_data->irq_data.time_us = -1;
		 drv_data->irq_data.dist_mm = -1;
	}

	input_report_abs(drv_data->input_dev, ABS_DISTANCE, drv_data->irq_data.dist_mm);
	input_report_abs(drv_data->input_dev, ABS_MISC, drv_data->irq_data.time_us);
	input_sync(drv_data->input_dev);
	drv_data->irq_data.count =0;
		
	gpio_set_value(drv_data->data->gpio_d1e,1);
	gpio_set_value(drv_data->data->gpio_d1a,1);
	udelay(10);
	gpio_set_value(drv_data->data->gpio_d1a,0);
	enable_irq(drv_data->irq);

	schedule_delayed_work(&drv_data->echo_work,msecs_to_jiffies(60));
}
static int hcsr04_open(struct input_dev *dev)
{
	struct hcsr04_drvdata *drv_data = input_get_drvdata(dev);
	enable_irq(drv_data->irq);
	schedule_delayed_work(&drv_data->echo_work,msecs_to_jiffies(60));
    return 0;
}

static void hcsr04_close(struct input_dev *dev)
{
	struct hcsr04_drvdata *drv_data = input_get_drvdata(dev);
	cancel_delayed_work_sync(&drv_data->echo_work);
	disable_irq(drv_data->irq);
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
	INIT_DELAYED_WORK(&drv_data->echo_work, hcsr04_echo_worker);

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
#define DRV_VERSION 	"0.1"

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