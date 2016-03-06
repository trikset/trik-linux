#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/mux.h>


static void dev_trik_jx2_release(struct device* dev){
}

static struct platform_device trik_jx2_device = {
	.name			= "trik_jx2",
	.id				= -1,
	.num_resources	= 0,
	.dev = {
		.release = dev_trik_jx2_release,
	}
};

static const short trik_jx2_pins[] = {
	DA850_GPIO1_11,
	-1
};

static const struct gpio trik_jx2_gpio_array[] = {
	{GPIO_TO_PIN(1,11),	GPIOF_OUT_INIT_LOW|GPIOF_EXPORT_DIR_FIXED,	"trik_jx2->GPIO1_11"},
};

int init_module(void) {
	int ret;
	ret = davinci_cfg_reg_list(trik_jx2_pins);
	if(ret){
		pr_err("%s: trik jx2 pins failed: %d\n",__func__,ret);
		goto exit_mux_failed;
	}
	ret = gpio_request_array(trik_jx2_gpio_array,ARRAY_SIZE(trik_jx2_gpio_array));
	if(ret){
		pr_err("%s: trik_jx2_gpio_array request failed\n",__func__);
		goto exit_gpio_request;
	}

	return 0;
exit_gpio_request:
exit_mux_failed:
	return ret;
}

void cleanup_module(void) {
	gpio_free_array(trik_jx2_gpio_array, ARRAY_SIZE(trik_jx2_gpio_array));
}

MODULE_LICENSE("GPL");

