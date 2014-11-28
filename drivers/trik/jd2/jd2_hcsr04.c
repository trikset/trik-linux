#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/mux.h>

#include <linux/trik-jdx.h>


static void dev_trik_jd2_release (struct device* dev){

}
static struct platform_device trik_jd2_device = {
	.name			= "trik_jd2",
	.id				= -1,
	.num_resources	= 0,
	.dev = {
        .release = dev_trik_jd2_release,
    }
};

static struct trik_jdx_platform_data trik_jd2_pdata = {
	.gpio_d1a = GPIO_TO_PIN(3,1),
	.gpio_d1e = GPIO_TO_PIN(0,11),
	.gpio_d1b = GPIO_TO_PIN(3,5),
};
static const struct gpio trik_jd2_gpio_array[] = {
	{GPIO_TO_PIN(3,1),	GPIOF_OUT_INIT_LOW|GPIOF_EXPORT_DIR_FIXED,	"trik_jd2->gpio_d1a"},
	{GPIO_TO_PIN(3,5),	GPIOF_IN|GPIOF_EXPORT_DIR_FIXED,			"trik_jd2->gpio_d1b"},
	{GPIO_TO_PIN(0,11),	GPIOF_OUT_INIT_HIGH|GPIOF_EXPORT_DIR_FIXED,	"trik_jd2->gpio_d1e"},
};
static const short trik_jd2_pins[] = {
	DA850_GPIO3_1,
	DA850_GPIO3_5,
	DA850_GPIO0_11,
	-1
};
int init_module(void)
{
	int ret;
	ret = davinci_cfg_reg_list(trik_jd2_pins);
	if(ret){
		pr_err("%s: trik jd1 pins failed: %d\n",__func__,ret);
		goto exit_mux_failed;
	}
	ret = gpio_request_array(trik_jd2_gpio_array,ARRAY_SIZE(trik_jd2_gpio_array));
	if(ret){
		pr_err("%s: trik_jd1_gpio_array request failed\n",__func__);
		goto exit_gpio_request;
	}

	platform_set_drvdata(&trik_jd2_device,&trik_jd2_pdata);
	ret = platform_device_register(&trik_jd2_device);
	if (ret){
		pr_err("%s: trik jd1 device register failed: %d\n",__func__,ret);
		goto exit_register_failed;
	}
	return 0;

exit_register_failed:
	gpio_free_array(trik_jd2_gpio_array, ARRAY_SIZE(trik_jd2_gpio_array));
exit_gpio_request:
exit_mux_failed:
	return ret;
}


void cleanup_module(void)
{
	platform_device_unregister(&trik_jd2_device);
	gpio_free_array(trik_jd2_gpio_array, ARRAY_SIZE(trik_jd2_gpio_array));
}

MODULE_LICENSE("GPL");

