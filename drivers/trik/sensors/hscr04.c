#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#include <linux/trik-jdx.h>


struct hscr04_drvdata {
	struct trik_jdx_platform_data* data;
	struct work_struct irq_work;
	struct work_struct echo_work;
	struct platform_device* platform_device;
	int irq;
	int enable;
};


static int hscr04_probe(struct platform_device *pdev)
{
	struct trik_jdx_platform_data* data = (struct trik_jdx_platform_data*)dev_get_drvdata(&pdev->dev);
	if (data){
		pr_warning("platform_id = %s \n",pdev->id_entry->name);
		pr_warning("gpio d1a = %d d1e = %d d1b = %d \n",data->gpio_d1a,data->gpio_d1e,data->gpio_d1b);
	}

	return 0;
}
static int hscr04_remove(struct platform_device *pdev)
{
	pr_warning("%s \n ",__func__);
	return 0;
}
static struct platform_device_id hscr04_ids[] = {
	{
		.name		= "da850_trik_jd1",
	}, {
		.name		= "da850_trik_jd2",
	}, 
	{ }
};
#define DRV_NAME "hscr04"
#define DRV_VERSION "0.1"

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