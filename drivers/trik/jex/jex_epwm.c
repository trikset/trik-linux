#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/mux.h>
#include <mach/da8xx.h>

#include <linux/pwm/ehrpwm.h>

static struct platform_device * je0_pd;
static struct platform_device * je1_pd;

static const short trik_jex_pwm_pins[] = {
	DA850_EHRPWM0_B,
	DA850_EHRPWM1_A,
	DA850_EHRPWM1_B,
	DA850_GPIO2_5,	/*PE0_EN*/
	DA850_GPIO2_3,	/*PE1_EN*/
	-1
};

#define DA8XX_EHRPWM0_BASE	0x01F00000
static struct resource da850_ehrpwm0_resource[] = {
	{
		.start	= DA8XX_EHRPWM0_BASE,
		.end	= DA8XX_EHRPWM0_BASE + 0x1fff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_DA8XX_EHRPWM0TZ,
		.end	= IRQ_DA8XX_EHRPWM0TZ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= IRQ_DA8XX_EHRPWM0,
		.end	= IRQ_DA8XX_EHRPWM0,
		.flags	= IORESOURCE_IRQ,
	},
};
#define DA8XX_EHRPWM1_BASE	0x01F02000
static struct resource da850_ehrpwm1_resource[] = {
	{
		.start	= DA8XX_EHRPWM1_BASE,
		.end	= DA8XX_EHRPWM1_BASE + 0x1fff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_DA8XX_EHRPWM1TZ,
		.end	= IRQ_DA8XX_EHRPWM1TZ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= IRQ_DA8XX_EHRPWM1,
		.end	= IRQ_DA8XX_EHRPWM1,
		.flags	= IORESOURCE_IRQ,
	},
};
static const struct gpio trik_jex_gpio_array[] = {
	{GPIO_TO_PIN(2,5),	GPIOF_OUT_INIT_HIGH|GPIOF_EXPORT_DIR_FIXED,	"trik_jex->pe0_en"},
	{GPIO_TO_PIN(2,3),	GPIOF_OUT_INIT_HIGH|GPIOF_EXPORT_DIR_FIXED,	"trik_jex->pe1_en"},
};
static const struct ehrpwm_platform_data da850_ehrpwm0_data = {
	.channel_mask = 0x02,
};
static const struct ehrpwm_platform_data da850_ehrpwm1_data = {
	.channel_mask = 0x03,
};
int init_module(void)
{
	int ret;

	ret = davinci_cfg_reg_list(trik_jex_pwm_pins);
	if (ret) {
		pr_err("%s: ehrpwm0 pinmux setup failed: %d\n", __func__, ret);
		goto exit_pin_mux;
	}
	ret = gpio_request_array(trik_jex_gpio_array,ARRAY_SIZE(trik_jex_gpio_array));
	if(ret){
		pr_err("%s: trik_jex_gpio_array request failed\n",__func__);
		goto exit_gpio_request;
	}

	je0_pd = platform_device_register_resndata(NULL,
											"ehrpwm",
												0,
												da850_ehrpwm0_resource,
												ARRAY_SIZE(da850_ehrpwm0_resource),
												&da850_ehrpwm0_data,
												sizeof(da850_ehrpwm0_data));
	if (IS_ERR(je0_pd)){
		ret = -ENOMEM;
		pr_err("%s: eHRPWM module0 registration failed\n",__func__);
		goto exit_pd_registor_ehrpwm0;
	}
	je1_pd = platform_device_register_resndata(NULL,
											"ehrpwm",
												1,
												da850_ehrpwm1_resource,
												ARRAY_SIZE(da850_ehrpwm1_resource),
												&da850_ehrpwm1_data,
												sizeof(da850_ehrpwm1_data));
	if (IS_ERR(je1_pd)){
		ret = -ENOMEM;
		pr_err("%s: eHRPWM module1 registration failed\n",__func__);
		goto exit_pd_registor_ehrpwm1;
	}
	return 0;
exit_pd_registor_ehrpwm1:
	platform_device_unregister(je0_pd);
exit_pd_registor_ehrpwm0:
	gpio_free_array(trik_jex_gpio_array, ARRAY_SIZE(trik_jex_gpio_array));
exit_gpio_request:
exit_pin_mux:
	return ret;
}
void cleanup_module(void)
{
	platform_device_unregister(je1_pd);
	platform_device_unregister(je0_pd);
//	gpio_free(GPIO_TO_PIN(2,4));
	gpio_free_array(trik_jex_gpio_array, ARRAY_SIZE(trik_jex_gpio_array));
}

MODULE_LICENSE("GPL");