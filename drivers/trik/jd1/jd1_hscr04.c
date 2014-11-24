#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/mux.h>

#include <linux/trik-jdx.h>


static void platform_jd1_release (struct device* dev){

}
static struct platform_device da850_trik_jd1_device = {
	.name			= "da850_trik_jd1",
	.id				= -1,
	.num_resources	= 0,
	.dev = {
        .release = platform_jd1_release,
    }
};

static struct trik_jdx_platform_data da850_trik_jd1_pdata = {
	.gpio_d1a = GPIO_TO_PIN(3,3),
	.gpio_d1e = GPIO_TO_PIN(0,13),
	.gpio_d1b = GPIO_TO_PIN(3,2),
};
static const short da850_trik_jd1_pins[] = {
	DA850_GPIO3_3,
	DA850_GPIO3_2,
	DA850_GPIO0_13,
	-1
};
int init_module(void)
{
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_jd1_pins);
	if(ret){
		pr_err("%s: trik jd1 pins failed: %d\n",__func__,ret);
		goto exit_mux_failed;
	}
	dev_set_drvdata(&da850_trik_jd1_device.dev,&da850_trik_jd1_pdata);
	ret = platform_device_register(&da850_trik_jd1_device);
	if (ret){
		pr_err("%s: trik jd1 device register failed: %d\n",__func__,ret);
		goto exit_register_failed;
	}
exit_register_failed:
exit_mux_failed:
	return ret;
}


void cleanup_module(void)
{
	platform_device_unregister(&da850_trik_jd1_device);
}

MODULE_LICENSE("GPL");

