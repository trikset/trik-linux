#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/mux.h>
#include <mach/da8xx.h>


static struct platform_device * jc0_pd;
static struct platform_device * jc1_pd;
static struct platform_device * jc2_pd;

static const short trik_jcx_pins[] = {
	DA850_ECAP0_APWM0,
	DA850_ECAP1_APWM1,
	DA850_ECAP2_APWM2,
	DA850_GPIO2_4/*PC_EN*/,
	-1,
};
#define DA8XX_ECAP0_BASE	0x01F06000
static struct resource trik_jcx_pwm0_resource[] = {
	{
		.start	= DA8XX_ECAP0_BASE,
		.end	= DA8XX_ECAP0_BASE + 0xfff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_DA8XX_ECAP0,
		.end	= IRQ_DA8XX_ECAP0,
		.flags	= IORESOURCE_IRQ,
	},
};

#define DA8XX_ECAP1_BASE	0x01F07000
static struct resource trik_jcx_pwm1_resource[] = {
	{
		.start	= DA8XX_ECAP1_BASE,
		.end	= DA8XX_ECAP1_BASE + 0xfff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_DA8XX_ECAP1,
		.end	= IRQ_DA8XX_ECAP1,
		.flags	= IORESOURCE_IRQ,
	},
};
#define DA8XX_ECAP2_BASE	0x01F08000
static struct resource trik_jcx_pwm2_resource[] = {
	{
		.start	= DA8XX_ECAP2_BASE,
		.end	= DA8XX_ECAP2_BASE + 0xfff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_DA8XX_ECAP2,
		.end	= IRQ_DA8XX_ECAP2,
		.flags	= IORESOURCE_IRQ,
	},
};

int init_module(void)
{
	int ret;
	ret = davinci_cfg_reg_list(trik_jcx_pins);
	if(ret){
		pr_err("%s: trik jcx pins failed: %d\n",__func__,ret);
		goto exit_mux_failed;
	}
	ret =  gpio_request_one(GPIO_TO_PIN(2,4),GPIOF_OUT_INIT_HIGH|GPIOF_EXPORT_DIR_FIXED,"PC_EN");
	if (ret){
		pr_err("%s: PC_EN gpio request failed: %d\n",__func__, ret);
		goto exit_gpio_request_failed;
	}
	jc0_pd = platform_device_register_simple("ecap",0,trik_jcx_pwm0_resource,ARRAY_SIZE(trik_jcx_pwm0_resource));
	if (IS_ERR(jc0_pd)) {
		ret = -ENOMEM;
		goto exit_pd_registor_ecap0;
	}
	jc1_pd = platform_device_register_simple("ecap",1,trik_jcx_pwm1_resource,ARRAY_SIZE(trik_jcx_pwm1_resource));
	if (IS_ERR(jc1_pd)) {
		ret = -ENOMEM;
		goto exit_pd_registor_ecap1;
	}
	jc2_pd = platform_device_register_simple("ecap",2,trik_jcx_pwm2_resource,ARRAY_SIZE(trik_jcx_pwm2_resource));
	if (IS_ERR(jc2_pd)) {
		ret = -ENOMEM;
		goto exit_pd_registor_ecap2;
	}
	return 0;
exit_pd_registor_ecap2:
	platform_device_unregister(jc1_pd);
exit_pd_registor_ecap1:
	platform_device_unregister(jc0_pd);
exit_pd_registor_ecap0:
	gpio_free(GPIO_TO_PIN(2,4));
exit_gpio_request_failed:
exit_mux_failed:
	return ret;
};
void cleanup_module(void)
{
	platform_device_unregister(jc2_pd);
	platform_device_unregister(jc1_pd);
	platform_device_unregister(jc0_pd);
	gpio_free(GPIO_TO_PIN(2,4));
}

MODULE_LICENSE("GPL");