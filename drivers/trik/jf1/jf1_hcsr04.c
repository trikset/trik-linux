#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/mux.h>


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
static const short trik_jf1_pins[] = {
	DA850_GPIO1_7,  //TIMER_IO
	DA850_GPIO0_8,
	-1
};
static const struct gpio trik_jf1_gpio_array[] = {
	{GPIO_TO_PIN(0,8),	GPIOF_OUT_INIT_LOW|GPIOF_EXPORT_DIR_FIXED,	"trik_jf1(2)->RTC_ALARM"},	
	{GPIO_TO_PIN(1,7),	GPIOF_OUT_INIT_LOW|GPIOF_EXPORT_DIR_FIXED,	"trik_jf1(3)->TIMER_IO"},
};
int init_module(void)
{

	int ret;
	ret = davinci_cfg_reg_list(trik_jf1_pins);
	if(ret){
		pr_err("%s: trik jd1 pins failed: %d\n",__func__,ret);
		goto exit_mux_failed;
	}
	ret = gpio_request_array(trik_jf1_gpio_array,ARRAY_SIZE(trik_jf1_gpio_array));
	if(ret){
		pr_err("%s: trik_jd1_gpio_array request failed\n",__func__);
		goto exit_gpio_request;
	}

	return 0;
exit_gpio_request:
exit_mux_failed:
	return ret;
}

void cleanup_module(void)
{
	gpio_free_array(trik_jf1_gpio_array, ARRAY_SIZE(trik_jf1_gpio_array));
}

MODULE_LICENSE("GPL");