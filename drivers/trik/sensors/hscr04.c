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

#include <linux/trik-jdx.h>

struct hscr04_drvdata {
	struct trik_jdx_platform_data* data;
	struct delayed_work irq_work;
	struct delayed_work echo_work;
	struct platform_device* platform_device;
	int irq;
	int enable;
};
static void hscr04_echo_worker(struct work_struct *work){
	struct hscr04_drvdata *drv_data = container_of(work, struct hscr04_drvdata, echo_work.work);
	struct timespec start, end, diff;
	
	gpio_set_value(drv_data->data->gpio_d1e,1);
	gpio_set_value(drv_data->data->gpio_d1a,1);
	udelay(10);
	gpio_set_value(drv_data->data->gpio_d1a,0);
	
	getnstimeofday(&start);
	do {
		getnstimeofday(&end);
		diff = timespec_sub(end, start);
	} while (gpio_get_value(GPIO_TO_PIN(3, 5)) && (diff.tv_sec == 0 || diff.tv_nsec < 60000000));
	pr_err(" %s: stamp (%lu; %lu) \n",__func__,diff.tv_sec, diff.tv_nsec);

	schedule_delayed_work(&drv_data->echo_work,msecs_to_jiffies(100));
}

static void hscr04_irq_worker(struct work_struct *work){

}
static int hscr04_probe(struct platform_device *pdev)
{
	int ret;

	struct hscr04_drvdata	*drv_data;
	struct trik_jdx_platform_data *data = (struct trik_jdx_platform_data*)dev_get_drvdata(&pdev->dev);

	pr_err("%s : pointer = %p\n",__func__, data);
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
	
	platform_set_drvdata(pdev,drv_data);

	INIT_DELAYED_WORK(&drv_data->echo_work, hscr04_echo_worker);
	INIT_DELAYED_WORK(&drv_data->irq_work, hscr04_irq_worker);
	schedule_delayed_work(&drv_data->echo_work,msecs_to_jiffies(100));

	pr_warning("platform_id = %s \n",pdev->id_entry->name);
	pr_warning("gpio d1a = %d d1e = %d d1b = %d \n",data->gpio_d1a,
													data->gpio_d1e,
													data->gpio_d1b);
	pr_err("%s : pointer = %p\n",__func__, drv_data);
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
	cancel_delayed_work_sync(&drv_data->echo_work);
	cancel_delayed_work_sync(&drv_data->irq_work);
	if(flush_delayed_work(&drv_data->echo_work)){
		pr_err("%s: drv_data->echo_work was flushed\n",__func__);
	}
	if(flush_delayed_work(&drv_data->irq_work)){
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