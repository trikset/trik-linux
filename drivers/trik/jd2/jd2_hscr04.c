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
	ret = gpio_request_one(trik_jd2_pdata.gpio_d1a, 
									GPIOF_OUT_INIT_LOW|GPIOF_EXPORT_DIR_FIXED|GPIOF_EXPORT,
									"trik_jd2->gpio_d1a");
	if(ret){
		pr_err("%s: trik_jd2->gpio_d1a failed\n",__func__);
		goto exit_gp_rq_d1a;
	}
	ret = gpio_request(trik_jd2_pdata.gpio_d1b,
									"trik_jd2->gpio_d1b");
	if(ret){
		pr_err("%s: gtrik_jd2->gpio_d1b failed\n",__func__);
		goto exit_gp_rq_d1b;
	}
	ret = gpio_request_one(trik_jd2_pdata.gpio_d1e,
									GPIOF_OUT_INIT_HIGH|GPIOF_EXPORT_DIR_FIXED|GPIOF_EXPORT, 
									"trik_jd2->gpio_d1e");
	if(ret){
		pr_err("%s: trik_jd2->gpio_d1e failed\n",__func__);
		goto exit_gp_rq_d1e;
	}

	platform_set_drvdata(&trik_jd2_device,&trik_jd2_pdata);
	ret = platform_device_register(&trik_jd2_device);
	if (ret){
		pr_err("%s: trik jd1 device register failed: %d\n",__func__,ret);
		goto exit_register_failed;
	}
	return 0;
exit_gp_rq_d1e:
	gpio_free(trik_jd2_pdata.gpio_d1b);
exit_gp_rq_d1b:
	gpio_free(trik_jd2_pdata.gpio_d1a);
exit_gp_rq_d1a:
exit_register_failed:
exit_mux_failed:
	return ret;
}


void cleanup_module(void)
{
	platform_device_unregister(&trik_jd2_device);
	gpio_free(trik_jd2_pdata.gpio_d1e);
	gpio_free(trik_jd2_pdata.gpio_d1b);
	gpio_free(trik_jd2_pdata.gpio_d1a);
}

MODULE_LICENSE("GPL");

