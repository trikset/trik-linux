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

struct hscr04_irq_data{
	bool enable_irq;
	int count;
	struct timespec monotime[2];
};

struct hscr04_drvdata {
	struct trik_jdx_platform_data* data;
	struct work_struct irq_work;
	struct delayed_work echo_work;
	struct platform_device* platform_device;
	int irq;
	int enable;
	struct hscr04_irq_data irq_data;
	struct input_dev *input_dev;
};
static irqreturn_t hscr04_irq_callback(int irq, void *dev_id){
	struct hscr04_drvdata *drv_data = dev_id;
	if(drv_data){
    	getnstimeofday(&drv_data->irq_data.monotime[drv_data->irq_data.count]);
		drv_data->irq_data.count++;
		schedule_work(&drv_data->irq_work);
	}
	return IRQ_HANDLED;
}
static void hscr04_irq_worker(struct work_struct *work){
	pr_err("%s : oops!!!\n",__func__);
}
static void hscr04_echo_worker(struct work_struct *work){
	struct hscr04_drvdata *drv_data = container_of(work, struct hscr04_drvdata, echo_work.work);
	int ret = 0;
	if (drv_data->irq_data.enable_irq){
		free_irq(drv_data->irq, drv_data);
		pr_err("%s : count interrupt = %d\n",__func__,drv_data->irq_data.count);
		if(drv_data->irq_data.count == 2){
			struct timespec diff = timespec_sub(drv_data->irq_data.monotime[1], drv_data->irq_data.monotime[0]);
			pr_err("%s: sec = %lu nsec = %lu  \n",__func__,diff.tv_sec,diff.tv_nsec);
		}
		drv_data->irq_data.count =0;
		drv_data->irq_data.enable_irq = false;
	}
	gpio_set_value(drv_data->data->gpio_d1e,1);
	gpio_set_value(drv_data->data->gpio_d1a,1);
	udelay(10);
	gpio_set_value(drv_data->data->gpio_d1a,0);
	ret = request_irq(drv_data->irq, 
						hscr04_irq_callback,
						(IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING),
						"hsrc04_irq",
						drv_data);
	if (!ret){
		drv_data->irq_data.enable_irq = true;
		//temp
		drv_data->irq_data.count =0;
	}

	schedule_delayed_work(&drv_data->echo_work,msecs_to_jiffies(100));
}

static int hscr04_probe(struct platform_device *pdev)
{
	int ret = 0;

	struct hscr04_drvdata	*drv_data;
	struct trik_jdx_platform_data *data = (struct trik_jdx_platform_data*)dev_get_drvdata(&pdev->dev);

	pr_warning("platform_id = %s \n",pdev->id_entry->name);
	pr_warning("gpio d1a = %d d1e = %d d1b = %d \n",data->gpio_d1a,
													data->gpio_d1e,
													data->gpio_d1b);
	
	drv_data = kzalloc(sizeof(*drv_data), GFP_KERNEL);
	if (drv_data == NULL){
		pr_err(" %s: drv_data allocation memory failed\n",__func__);
		ret = -ENOMEM;
		goto exit_alloc_failed;
	}

	if (!data){
		pr_err("%s: no platform data\n",__func__);
		ret = -EINVAL;
		goto exit_plt_data_failed;
	}
	drv_data->data = data;
	drv_data->input_dev = input_allocate_device();
	if (!drv_data->input_dev){
		ret = -ENOMEM;
		pr_err("%s: input device allocate failed\n",__func__);
		goto exit_input_device_alloc;
	}
	platform_set_drvdata(pdev,drv_data);

	INIT_DELAYED_WORK(&drv_data->echo_work, hscr04_echo_worker);
	INIT_WORK(&drv_data->irq_work, hscr04_irq_worker);
	drv_data->irq = gpio_to_irq(drv_data->data->gpio_d1b);
	schedule_delayed_work(&drv_data->echo_work,msecs_to_jiffies(100));

	return 0;

exit_plt_data_failed:
	kfree(drv_data);
exit_alloc_failed:
	return ret;
}
static int hscr04_remove(struct platform_device *pdev)
{
	struct hscr04_drvdata *drv_data = platform_get_drvdata(pdev);
	pr_err("%s : pointer = %p\n",__func__, drv_data);
	free_irq(drv_data->irq, drv_data);
	cancel_delayed_work_sync(&drv_data->echo_work);
	if(flush_delayed_work(&drv_data->echo_work)){
		pr_err("%s: drv_data->echo_work was flushed\n",__func__);
	}
	if(flush_work_sync(&drv_data->irq_work)){
		pr_err("%s: drv_data->irq_work was flushed\n",__func__);
	}
	kfree(drv_data);
	pr_warning("%s \n ",__func__);
	return 0;
}
static struct platform_device_id hscr04_ids[] = {
	{
		.name		= "trik_jd1",
	}, {
		.name		= "trik_jd2",
	}, 
	{ }
};
#define DRV_NAME 		"hscr04"
#define DRV_VERSION 	"0.1"

static struct platform_driver hscr04_driver = {
	.probe		= hscr04_probe,
	.remove		= __devexit_p(hscr04_remove),
	.id_table	= hscr04_ids,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(hscr04_driver);
MODULE_LICENSE("GPL");