#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/mux.h>

#include <linux/trik-jdx.h>


static void dev_trik_jf1_release(struct device* dev){

}
static struct platform_device trik_jf1_device = {
	.name			= "trik_jf1",
	.id				= -1,
	.num_resources	= 0,
	.dev = {
		.release = dev_trik_jf1_release,
	}
};


static struct trik_jdx_platform_data trik_jf1_pdata = {
	.gpio_d1a = GPIO_TO_PIN(0,8),
	.gpio_d1b = GPIO_TO_PIN(1, 7),
};


static const struct gpio trik_jf1_gpio_array[] = {
	{GPIO_TO_PIN(0,8),	GPIOF_OUT_INIT_LOW|GPIOF_EXPORT_DIR_FIXED,	"trik_jf1(2)->RTC_ALARM"},	
	{GPIO_TO_PIN(1,7),	GPIOF_IN|GPIOF_EXPORT_DIR_FIXED,	"trik_jf1(3)->TIMER_IO"},
};

static const short trik_jf1_pins[] = {
	DA850_GPIO1_7,  //TIMER_IO
	DA850_GPIO0_8,
	-1
};

int init_module(void)
{

	int ret;
	ret = davinci_cfg_reg_list(trik_jf1_pins);
	if(ret){
		pr_err("%s: trik jf1 pins failed: %d\n",__func__,ret);
		goto exit_mux_failed;
	}
	ret = gpio_request_array(trik_jf1_gpio_array,ARRAY_SIZE(trik_jf1_gpio_array));
	if(ret){
		pr_err("%s: trik_jf1_gpio_array request failed\n",__func__);
		goto exit_gpio_request;
	}
	platform_set_drvdata(&trik_jf1_device,&trik_jf1_pdata);
	ret = platform_device_register(&trik_jf1_device);
	if (ret){
		pr_err("%s: trik jf1 device register failed: %d\n",__func__,ret);
		goto exit_register_failed;
	}

	return 0;
exit_register_failed:
	gpio_free_array(trik_jf1_gpio_array, ARRAY_SIZE(trik_jf1_gpio_array));
exit_gpio_request:
exit_mux_failed:
	return ret;
}

void cleanup_module(void)
{
	platform_device_unregister(&trik_jf1_device);
	gpio_free_array(trik_jf1_gpio_array, ARRAY_SIZE(trik_jf1_gpio_array));
}

MODULE_LICENSE("GPL");
