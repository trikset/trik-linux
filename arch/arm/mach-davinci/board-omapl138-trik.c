/*
 * Hawkboard.org based on TI's OMAP-L138 Platform
 *
 * Initial code: Syed Mohammed Khasim
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of
 * any kind, whether express or implied.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/system_info.h>

#include <mach/cp_intc.h>
#include <mach/da8xx.h>
#include <mach/mux.h>
#include <mach/aemif.h>
#include <mach/spi.h>
#include <linux/mma7660fc.h>
#include <linux/wl12xx.h>
#include <linux/da8xx-ili9340-fb.h>


#define DA850TRIK_POW_CON_REG		DA850_GPIO5_14
#define DA850TRIK_POW_CON_PIN		GPIO_TO_PIN(5, 14)

#define DA850TRIK_LED_CON_REG		DA850_GPIO5_7
#define DA850TRIK_LED_CON_PIN           GPIO_TO_PIN(5, 7)
		
#define DA850TRIK_MMCSD_CD_REG		DA850_GPIO4_1
#define DA850TRIK_MMCSD_CD_PIN		GPIO_TO_PIN(4, 1)
#define DA850TRIK_USB20_OC_REG		DA850_GPIO5_15
#define DA850TRIK_USB20_OC_PIN		GPIO_TO_PIN(5, 15)
#define DA850TRIK_AUDIO_RESET_REG	DA850_GPIO6_15
#define DA850TRIK_AUDIO_RESET_PIN	GPIO_TO_PIN(6, 15)
#define DA850TRIK_ACCEL_IRQ_REG		DA850_GPIO0_13
#define DA850TRIK_ACCEL_IRQ_PIN		GPIO_TO_PIN(0, 13)

#define DA850TRIK_WLAN_IRQ_REG		DA850_GPIO5_10
#define DA850TRIK_WLAN_IRQ_PIN		GPIO_TO_PIN(5, 10)
#define DA850TRIK_WLAN_ENABLE_REG	DA850_GPIO5_11
#define DA850TRIK_WLAN_ENABLE_PIN	GPIO_TO_PIN(5, 11)
#define DA850TRIK_BT_ENABLE_REG		DA850_GPIO5_12
#define DA850TRIK_BT_ENABLE_PIN		GPIO_TO_PIN(5, 12)

#define DA850TRIK_MSP430_TEST_REG	DA850_GPIO5_5
#define DA850TRIK_MSP430_TEST_PIN	GPIO_TO_PIN(5, 5)
#define DA850TRIK_MSP430_RESET_REG	DA850_GPIO5_13
#define DA850TRIK_MSP430_RESET_PIN	GPIO_TO_PIN(5, 13)

#if defined(CONFIG_DAVINCI_EHRPWM) || defined(CONFIG_DAVINCI_EHRPWM_MODULE)
#define HAS_EHRPWM 1
#else
#define HAS_EHRPWM 0
#endif

#if defined(CONFIG_ECAP_PWM) || \
        defined(CONFIG_ECAP_PWM_MODULE)
#define HAS_ECAP_PWM 1
#else
#define HAS_ECAP_PWM 0
#endif

#if defined(CONFIG_ECAP_CAP) || defined(CONFIG_ECAP_CAP_MODULE)
#define HAS_ECAP_CAP 1
#else
#define HAS_ECAP_CAP 0
#endif


enum {DA850TRIK_SPI_FLASH, DA850TRIK_SPI_WLAN};


/*
 * Power management
 */
static struct davinci_pm_config da850_pm_pdata = {
	.sleepcount = 128,
};

static struct platform_device da850_pm_device = {
	.name           = "pm-davinci",
	.dev = {
		.platform_data	= &da850_pm_pdata,
	},
	.id             = -1,
};

/*
 * UART0 & UART1
 */
static const short da850trik_uart_pins[] __initconst = {
	DA850_UART0_RXD, DA850_UART0_TXD, DA850_NUART0_CTS, DA850_NUART0_RTS,
	DA850_UART1_RXD, DA850_UART1_TXD,
	-1
};

static struct davinci_uart_config da850trik_uart_config __initdata = {
	.enabled_uarts = 0x3,
};

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init da850trik_console_init(void)
{
	if (!machine_is_omapl138_trikboard())
		return 0;

	return add_preferred_console("ttyS", 1, "115200");
}
console_initcall(da850trik_console_init);
#endif

static __init void da850trik_serial_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(da850trik_uart_pins);
	if (ret) {
		pr_err("%s: UART0/1 mux setup failed: %d\n",
			__func__, ret);
		return;
	}
	davinci_serial_init(&da850trik_uart_config);
}


/*
 * audio
 */
static const short da850trik_mcasp_pins[] __initconst = {
	DA850_AHCLKX, DA850_ACLKX, DA850_AFSX,
	DA850_AXR_7,	/* TX */
	DA850_AXR_9,	/* RX */
	-1
};

static u8 da850trik_iis_serializer_direction[] = {
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	TX_MODE,
	INACTIVE_MODE,	RX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data da850trik_snd_data = {
	.tx_dma_offset	= 0x2000,
	.rx_dma_offset	= 0x2000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(da850trik_iis_serializer_direction),
	.tdm_slots	= 2,
	.serial_dir	= da850trik_iis_serializer_direction,
	.asp_chan_q	= EVENTQ_0,
	.version	= MCASP_VERSION_2,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};

static void __init da850trik_audio_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(da850trik_mcasp_pins);
	if (ret) {
		pr_err("%s: mcasp mux setup failed: %d\n",
			__func__, ret);
		return;
	}

	ret = davinci_cfg_reg(DA850TRIK_AUDIO_RESET_REG);
	if (ret) {
		pr_err("%s: audio codec reset mux setup failed: %d\n",
			__func__, ret);
		return;
	}

	ret = gpio_request(DA850TRIK_AUDIO_RESET_PIN, "audio codec reset");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO for audio codec reset: %d\n",
			__func__, ret);
		return;
	}
	gpio_direction_output(DA850TRIK_AUDIO_RESET_PIN, 1);

	da8xx_register_mcasp(0, &da850trik_snd_data);
}


/*
 * Accelerometer
 */
static struct mma7660fc_pdata da850trik_mma7660fc_pdata;

void __init da850trik_accel_init(void)
{
	int ret;

	ret = davinci_cfg_reg(DA850TRIK_ACCEL_IRQ_REG);
	if (ret) {
		pr_err("%s: accelerometer mux setup failed: %d\n",
			__func__, ret);
		return;
	}

	ret = gpio_request_one(DA850TRIK_ACCEL_IRQ_PIN,
			GPIOF_DIR_IN, "accelerometer enable");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO for accelerometer IRQ: %d\n",
			__func__, ret);
		return;
	}

	da850trik_mma7660fc_pdata.irq = gpio_to_irq(DA850TRIK_ACCEL_IRQ_PIN);
}


/*
 * MSP430
 */
static const short da850trik_msp430_pins[] __initconst = {
	DA850_CLKOUT0,
	DA850TRIK_MSP430_TEST_REG,
	DA850TRIK_MSP430_RESET_REG,
	-1
};

static struct gpio da850trik_msp430_gpios[] __initdata = {
	{ DA850TRIK_MSP430_TEST_PIN,  GPIOF_OUT_INIT_LOW, "msp430 test pin" },
	{ DA850TRIK_MSP430_RESET_PIN, GPIOF_OUT_INIT_LOW, "msp430 rst pin"  },
};

#define PLL0_OSCEL_OFFS		0x0104
#define PLL0_OSCDIV_OFFS	0x0124
#define PLL0_CKEN_OFFS		0x0148

static __init void da850trik_msp430_init(void)
{
	int			ret;
	unsigned __iomem	*pll0_oscel, *pll0_oscdiv, *pll0_cken;

	ret = davinci_cfg_reg_list(da850trik_msp430_pins);
	if (ret) {
		pr_err("%s: msp430 mux setup failed: %d\n",
			__func__, ret);
		return;
	}

	ret = gpio_request_array(da850trik_msp430_gpios,
				 ARRAY_SIZE(da850trik_msp430_gpios));
	if (ret < 0) {
		pr_err("%s: failed to request GPIO for msp430: %d\n",
			__func__, ret);
		return;
	}

	gpio_set_value(DA850TRIK_MSP430_RESET_PIN, 0);
	msleep(10);
	if (da850_pm_pdata.cpupll_reg_base) {
		pll0_oscel  = (unsigned*)(da850_pm_pdata.cpupll_reg_base + PLL0_OSCEL_OFFS);
		pll0_oscdiv = (unsigned*)(da850_pm_pdata.cpupll_reg_base + PLL0_OSCDIV_OFFS);
		pll0_cken   = (unsigned*)(da850_pm_pdata.cpupll_reg_base + PLL0_CKEN_OFFS);

		/* select OBSCLK in OCSRC field of PLL0_OSCEL */
		*pll0_oscel = 0x14;
		/* set OD1EN=1 and RATIO=0 fields in PLL0_OSCDIV */
		*pll0_oscel = 0x8000;
		/* set OBSEN=1 field in PLL0_CKEN */
		*pll0_cken |= 0x02;
	} else {
		pr_err("%s: failed to setup PLL0_CLKOUT\n", __func__);
	}
	gpio_set_value(DA850TRIK_MSP430_RESET_PIN, 1);
	msleep(10);
}


/*
 * I2C0
 */
static struct i2c_board_info __initdata da850trik_i2c0_devices[] = {
};

static struct davinci_i2c_platform_data da850trik_i2c0_pdata = {
	.bus_freq	= 100,	/* kHz */
	.bus_delay	= 0,	/* usec */
};

static __init void da850trik_i2c0_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(da850_i2c0_pins);
	if (ret) {
		pr_warning("da850trik_i2c0_init: I2C0 mux setup failed: %d\n",
				ret);
		return;
	}

	ret = da8xx_register_i2c(0, &da850trik_i2c0_pdata);
	if (ret) {
		pr_warning("da850trik_i2c0_init: I2C0 registration failed: %d\n",
				ret);
		return;
	}

	i2c_register_board_info(1, da850trik_i2c0_devices,
			ARRAY_SIZE(da850trik_i2c0_devices));
}


/*
 * I2C1
 */
static struct i2c_board_info __initdata da850trik_i2c1_devices[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	{
		I2C_BOARD_INFO("mma7660fc", 0x4c),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &da850trik_mma7660fc_pdata,
	},
};

static struct davinci_i2c_platform_data da850trik_i2c1_pdata = {
	.bus_freq	= 100,	/* kHz */
	.bus_delay	= 0,	/* usec */
};

static __init void da850trik_i2c1_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(da850_i2c1_pins);
	if (ret) {
		pr_warning("da850trik_i2c1_init: I2C1 mux setup failed: %d\n",
				ret);
		return;
	}

	ret = da8xx_register_i2c(1, &da850trik_i2c1_pdata);
	if (ret) {
		pr_warning("da850trik_i2c1_init: I2C1 registration failed: %d\n",
				ret);
		return;
	}

	i2c_register_board_info(2, da850trik_i2c1_devices,
			ARRAY_SIZE(da850trik_i2c1_devices));
}


/*
 * GPIO Buttons
 */

#define DA850TRIK_KEYS_DEBOUNCE_MS	10
/*
 * At 200ms polling interval it is possible to miss an
 * event by tapping very lightly on the push button but most
 * pushes do result in an event; longer intervals require the
 * user to hold the button whereas shorter intervals require
 * more CPU time for polling.
 */
#define DA850TRIK_GPIO_KEYS_POLL_MS	200

static const short da850trik_gpio_keys_pins[] __initconst = {
	DA850_GPIO3_4,	/* sw2 */
	DA850_GPIO2_0,	/* sw3 */
	DA850_GPIO3_14,	/* sw4 */
	DA850_GPIO3_15,	/* sw5 */
	DA850_GPIO3_13,	/* sw6 */
	DA850_GPIO3_12,	/* sw7 */
	-1
};

static struct gpio_keys_button da850trik_gpio_keys[] = {
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(3, 4),
		.code		   = KEY_F2,
		.desc		   = "sw2",
	},
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(2, 0),
		.code		   = KEY_F3,
		.desc		   = "sw3",
	},
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(3, 14),
		.code		   = KEY_F4,
		.desc		   = "sw4",
	},
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(3, 15),
		.code		   = KEY_F5,
		.desc		   = "sw5",
	},
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(3, 13),
		.code		   = KEY_F6,
		.desc		   = "sw6",
	},
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(3, 12),
		.code		   = KEY_F7,
		.desc		   = "sw7",
	}
};

static struct gpio_keys_platform_data da850trik_gpio_keys_data = {
	.buttons	= da850trik_gpio_keys,
	.nbuttons	= ARRAY_SIZE(da850trik_gpio_keys),
	.poll_interval	= DA850TRIK_GPIO_KEYS_POLL_MS,
};

static struct platform_device da850trik_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &da850trik_gpio_keys_data,
	}
};

static void __init da850trik_gpio_keys_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(da850trik_gpio_keys_pins);
	if (ret) {
		pr_err("%s: gpio-keys mux setup failed: %d\n",
			__func__, ret);
		return;
	}
	platform_device_register(&da850trik_gpio_keys_device);
}


/*
 * MMC/SD
 */
static const short da850trik_mmcsd0_pins[] __initconst = {
	DA850_MMCSD0_DAT_0, DA850_MMCSD0_DAT_1, DA850_MMCSD0_DAT_2,
	DA850_MMCSD0_DAT_3, DA850_MMCSD0_CLK, DA850_MMCSD0_CMD,
	-1
};

static int da850trik_mmc_get_cd(int index)
{
	return !gpio_get_value(DA850TRIK_MMCSD_CD_PIN);
}

static struct davinci_mmc_config da850trik_mmc_config = {
	.get_cd		= da850trik_mmc_get_cd,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
	.version	= MMC_CTLR_VERSION_2,
};

static __init void da850trik_mmc_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(da850trik_mmcsd0_pins);
	if (ret) {
		pr_err("%s: MMC/SD mux setup failed: %d\n",
			__func__, ret);
		return;
	}

	ret = davinci_cfg_reg(DA850TRIK_MMCSD_CD_REG);
	if (ret) {
		pr_err("%s: MMC/SD media change mux setup failed: %d\n",
			__func__, ret);
		return;
	}

	ret = gpio_request(DA850TRIK_MMCSD_CD_PIN, "MMC/SD media change detection");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO for mmc/sd media change detection: %d\n",
			__func__, ret);
		return;
	}
	gpio_direction_input(DA850TRIK_MMCSD_CD_PIN);

	ret = da8xx_register_mmcsd0(&da850trik_mmc_config);
	if (ret) {
		pr_err("%s: MMC/SD registration failed: %d\n",
			__func__, ret);
		gpio_free(DA850TRIK_MMCSD_CD_PIN);
		return;
	}
}


/*
 * USB-2.0 & USB-1.1
 */
static int da850trik_usb11_power_status = 1;
static int da850trik_usb11_set_power(unsigned port, int on)
{
	da850trik_usb11_power_status = on;
	return 0;
}

static int da850trik_usb11_get_power(unsigned port)
{
	return da850trik_usb11_power_status;
}

static int da850trik_usb11_get_oci(unsigned port)
{
	return 0;
}

static int da850trik_usb11_ocic_notify(da8xx_ocic_handler_t handler)
{
	return 0;
}

static struct da8xx_ohci_root_hub da850trik_usb11_pdata = {
	.set_power      = da850trik_usb11_set_power,
	.get_power      = da850trik_usb11_get_power,
	.get_oci        = da850trik_usb11_get_oci,
	.ocic_notify    = da850trik_usb11_ocic_notify,
	/* TPS2087 switch @ 5V */
	.potpgt         = (3 + 1) / 2,  /* 3 ms max */
};

static irqreturn_t da850trik_usb20_ocic_irq(int irq, void *dev_id)
{
	printk(KERN_ERR "%s: over-current situation detected\n", __func__);
	return IRQ_HANDLED;
}
static __init void da850trik_pwm_init(void)
{
	int ret;
	char mask = 0;
	ret = davinci_cfg_reg(DA850TRIK_POW_CON_REG);
        if (ret) {
                pr_err("%s: power connection gpio setup failed: %d\n",
                        __func__, ret);
                return;
       	}
	ret = gpio_request(DA850TRIK_POW_CON_PIN, "power connection gpio");
        if (ret < 0) {
                pr_err("%s: failed to request GPIO for power connection gpio: %d\n",
                        __func__, ret);
                return;
        }
	gpio_set_value(DA850TRIK_POW_CON_PIN, 1);

        ret = gpio_direction_output(DA850TRIK_POW_CON_PIN, 0);
	if (ret < 0) {
                pr_err("%s: failed gpio_direction_output GPIO : %d\n",
                        __func__, ret);
                return;
        }

	ret = gpio_export(DA850TRIK_POW_CON_PIN, 0);
	if (ret < 0) {
                pr_err("%s: failed to gpio_export power connection: %d\n",
                        __func__, ret);
                return;
        }


	ret = davinci_cfg_reg_list(da850_ehrpwm0_pins);
	if (ret)
		pr_warning("da850trik_pwm_init:"
				" ehrpwm0 mux setup failed: %d\n", ret);
	else
		mask |=  BIT(0) | BIT(1);
	ret = davinci_cfg_reg_list(da850_ehrpwm1_pins);
	if (ret)
		pr_warning("da850trik_pwm_init:"
                                " ehrpwm1 mux setup failed: %d\n", ret);
        else
                mask |= BIT(2) | BIT(3);
	da850_register_ehrpwm(mask);
}
static __init void da850trik_cap_init(void)
{
	int ret = 0;
	ret = davinci_cfg_reg(DA850_ECAP0_APWM0);
	if (ret)
		pr_warning("da850_evm_init:ecap mux failed:%d\n"
                                                , ret);
	else {
		ret = da850_register_ecap_cap(0);
		if (ret)
			pr_warning("da850trik_cap_init: "
        	                    "eCAP 0 registration failed: %d\n", ret);
	}
	ret = davinci_cfg_reg(DA850_ECAP1_APWM1);
        if (ret)
                pr_warning("da850_evm_init:ecap mux failed:%d\n"
                                                , ret);
        else {
		ret = da850_register_ecap_cap(1);
        	if (ret)
                	pr_warning("da850trik_cap_init:"
                        	    "eCAP 1 regisration failed: %d\n", ret);
	}
	ret = davinci_cfg_reg(DA850_ECAP2_APWM2);
        if (ret)
                pr_warning("da850_evm_init:ecap mux failed:%d\n"
                                                , ret);
        else {
		ret = da850_register_ecap_cap(2);
        	if (ret)
                	pr_warning("da850trik_cap_init"
                        	    "eCAP 2 registration failed: %d\n", ret);
	}
}
static __init void da850trik_usb_init(void)
{
	int ret, irq;
	u32 cfgchip2;

	ret = davinci_cfg_reg(DA850TRIK_USB20_OC_REG);
	if (ret) {
		pr_err("%s: USB 2.0 overcurrent mux setup failed: %d\n",
			__func__, ret);
		return;
	}

	ret = gpio_request_one(DA850TRIK_USB20_OC_PIN,
			GPIOF_DIR_IN, "USB 2.0 overcurrent");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO for USB 2.0 "
			"overcurrent indicator: %d\n", __func__, ret);
		return;
	}

	irq = gpio_to_irq(DA850TRIK_USB20_OC_PIN);
	ret = request_irq(irq, da850trik_usb20_ocic_irq, IRQF_DISABLED |
				    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				    "USB 2.0 overcurrent", NULL);
	if (ret) {
		pr_err("%s: failed to request IRQ for USB 2.0 "
			       "overcurrent: %d\n", __func__, ret);
		gpio_free(DA850TRIK_USB20_OC_PIN);
		return;
	}

	/*
	 * Set up USB clock/mode in the CFGCHIP2 register.
	 * FYI:  CFGCHIP2 is 0x0000ef00 initially.
	 */
	cfgchip2 = __raw_readl(DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));

	/* USB2.0 PHY reference clock is 24 MHz */
	cfgchip2 &= ~CFGCHIP2_REFFREQ;
	cfgchip2 |=  CFGCHIP2_REFFREQ_24MHZ;

	/*
	 * Select internal reference clock for USB 2.0 PHY
	 * and use it as a clock source for USB 1.1 PHY
	 * (this is the default setting anyway).
	 */
	cfgchip2 &= ~CFGCHIP2_USB1PHYCLKMUX;
	cfgchip2 |=  CFGCHIP2_USB2PHYCLKMUX;

	/*
	 * We have to override VBUS/ID signals when MUSB is configured into the
	 * host-only mode -- ID pin will float if no cable is connected, so the
	 * controller won't be able to drive VBUS thinking that it's a B-device.
	 * Otherwise, we want to use the OTG mode and enable VBUS comparators.
	 */
	cfgchip2 &= ~CFGCHIP2_OTGMODE;
#ifdef	CONFIG_USB_MUSB_HOST
	cfgchip2 |=  CFGCHIP2_FORCE_HOST_VBUS_LOW;
#else
	cfgchip2 |=  CFGCHIP2_SESENDEN | CFGCHIP2_VBDTCTEN;
#endif
	__raw_writel(cfgchip2, DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));

	/*
	 * TPS2065 switch @ 5V supplies 1 A (sustains 1.5 A),
	 * with the power on to power good time of 3 ms.
	 */
	ret = da8xx_register_usb20(500, 20);
	if (ret)
		pr_err("%s: USB 2.0 registration failed: %d\n",
			__func__, ret);

	ret = da8xx_register_usb11(&da850trik_usb11_pdata);
	if (ret)
		pr_err("%s: USB 1.1 registration failed: %d\n",
			__func__, ret);
}


/*
 * SPI flash
 */
static struct mtd_partition da850trik_spiflash_part[] = {
	[0] = {
		.name = "U-Boot",
		.offset = 0,
		.size = SZ_256K,
		.mask_flags = MTD_WRITEABLE,
	},
	[1] = {
		.name = "U-Boot-Env1",
		.offset = MTDPART_OFS_APPEND,
		.size = SZ_64K,
		.mask_flags = MTD_WRITEABLE,
	},
	[2] = {
		.name = "U-Boot-Env2",
		.offset = MTDPART_OFS_APPEND,
		.size = SZ_64K,
		.mask_flags = MTD_WRITEABLE,
	},
};

static struct flash_platform_data da850trik_spiflash_data = {
	.name		= "m25p80",
	.parts		= da850trik_spiflash_part,
	.nr_parts	= ARRAY_SIZE(da850trik_spiflash_part),
	.type		= "m25p128",
};


/*
 * WIFI/BLUETOOTH
 */
static int da850trik_wlan_power_state = -1;
static int da850trik_bluetooth_power_state = -1;

static void da850trik_wlan_set_power(bool power_on);
static void da850trik_bluetooth_set_power(bool power_on);

static const short da850trik_wl1271_pins[] __initconst = {
	DA850TRIK_WLAN_IRQ_REG,
	DA850TRIK_WLAN_ENABLE_REG,
	DA850TRIK_BT_ENABLE_REG,
	-1
};

static struct gpio da850trik_wl1271_gpios[] __initdata = {
	{ DA850TRIK_WLAN_ENABLE_PIN, GPIOF_OUT_INIT_LOW, "wlan power"      },
	{ DA850TRIK_WLAN_IRQ_PIN,    GPIOF_IN,           "wlan irq"        },
	{ DA850TRIK_BT_ENABLE_PIN,   GPIOF_OUT_INIT_LOW, "bluetooth power" },
};

static struct wl12xx_platform_data da850trik_wlan_data = {
	.irq			= -1,
	.set_power		= da850trik_wlan_set_power,
	.board_ref_clock	= WL12XX_REFCLOCK_38,
	.platform_quirks	= WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};

static void da850trik_wl1271_power_on(void)
{
	pr_err("Powering wifi/bt on wl12xx\n");
	/* VIO & slow clock must be stable already */

	/* BT_EN/WL_EN must be low */
	gpio_set_value(DA850TRIK_WLAN_ENABLE_PIN, 1);
	gpio_set_value(DA850TRIK_BT_ENABLE_PIN, 1);
	msleep(70);

	/* short pulse for at least 10ms */
	gpio_set_value(DA850TRIK_WLAN_ENABLE_PIN, 0);
	gpio_set_value(DA850TRIK_BT_ENABLE_PIN, 0);
	usleep_range(15000, 15000);
	/* delay for at least 64 mks */
	gpio_set_value(DA850TRIK_WLAN_ENABLE_PIN, 1);
	gpio_set_value(DA850TRIK_BT_ENABLE_PIN, 1);
	usleep_range(1000, 1000);

	da850trik_wlan_power_state = 0;
	da850trik_bluetooth_power_state = 0;
}

static void da850trik_wlan_set_power(bool power_on)
{
	if (da850trik_wlan_power_state < 0)
		da850trik_wl1271_power_on();

	if (power_on  && (da850trik_wlan_power_state == 1)) return;
	if (!power_on && (da850trik_wlan_power_state == 0)) return;

	da850trik_wlan_power_state = power_on ? 1 : 0;
	if (power_on) {
		pr_err("%s: WLAN POWER ON\n", __func__);
		gpio_set_value(DA850TRIK_WLAN_ENABLE_PIN, 0);
		msleep(70);
	} else {
		pr_err("%s: WLAN POWER OFF\n", __func__);
		gpio_set_value(DA850TRIK_WLAN_ENABLE_PIN, 1);
		msleep(70);
	}
}

static void da850trik_bluetooth_set_power(bool power_on)
{
	if (da850trik_bluetooth_power_state < 0)
		da850trik_wl1271_power_on();

	if (power_on  && (da850trik_bluetooth_power_state == 1)) return;
	if (!power_on && (da850trik_bluetooth_power_state == 0)) return;

	da850trik_bluetooth_power_state = power_on ? 1 : 0;
	if (power_on) {
		gpio_set_value(DA850TRIK_BT_ENABLE_PIN, 0);
		msleep(150);
	} else {
		gpio_set_value(DA850TRIK_BT_ENABLE_PIN, 1);
		msleep(150);
	}
}

static void __init da850trik_wl1271_init(void)
{
	int irq, ret;

	ret = davinci_cfg_reg_list(da850trik_wl1271_pins);
	if (ret) {
		pr_err("%s: WLAN/BT mux setup failed: %d\n",
			__func__, ret);
		return;
	}

	ret = gpio_request_array(da850trik_wl1271_gpios,
				 ARRAY_SIZE(da850trik_wl1271_gpios));
	if (ret < 0) {
		pr_err("%s: failed to request GPIO for WLAN/BT: %d\n",
			__func__, ret);
		return;
	}

	irq = gpio_to_irq(DA850TRIK_WLAN_IRQ_PIN);
	if (irq < 0) {
		pr_err("%s: failed to convert GPIO to IRQ for WLAN/BT: %d\n",
			__func__, ret);
		gpio_free(DA850TRIK_WLAN_ENABLE_PIN);
		gpio_free(DA850TRIK_WLAN_IRQ_PIN);
		gpio_free(DA850TRIK_BT_ENABLE_PIN);
		return;
	}

	da850trik_wlan_data.irq = irq;

	da850trik_wl1271_power_on();

	/* turn on bluetooth */
	da850trik_bluetooth_set_power(1);
}


/*
 * SPI0
 */
static struct davinci_spi_config da850trik_spiflash_cfg = {
	.io_type	= SPI_IO_TYPE_DMA,
	.c2tdelay	= 8,
	.t2cdelay	= 8,
};

static struct davinci_spi_config da850trik_wlan_cfg = {
	.io_type	= SPI_IO_TYPE_DMA,
	.c2tdelay	= 8,
	.t2cdelay	= 8,
};

static struct spi_board_info da850trik_spi0_info[] = {
	[DA850TRIK_SPI_FLASH] = {
		.modalias		= "m25p80",
		.controller_data	= &da850trik_spiflash_cfg,
		.platform_data		= &da850trik_spiflash_data,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 25000000,
		.bus_num		= 0,
		.chip_select		= 0,
	},
	[DA850TRIK_SPI_WLAN] = {
		.modalias		= "wl1271_spi",
		.controller_data	= &da850trik_wlan_cfg,
		.platform_data		= &da850trik_wlan_data,
		.mode			= SPI_MODE_0,
		.max_speed_hz   	= 25000000,
		.bus_num		= 0,
		.chip_select		= 1,
	},
};

static void __init da850trik_spi0_init(void)
{
	int ret;

	da850trik_spi0_info[DA850TRIK_SPI_WLAN].irq = da850trik_wlan_data.irq;

	ret = da8xx_register_spi(0, da850trik_spi0_info,
				 ARRAY_SIZE(da850trik_spi0_info));
	if (ret)
		pr_err("%s: failed to register SPI0 interface: %d\n",
			__func__, ret);
}


/*
 * ILI9340-based LCD
 */
static const short da850trik_lcd_extra_pins[] __initconst = {
	DA850_GPIO6_12, // LCD backlight
	DA850_GPIO8_10, // LCD reset
	-1
};

static void da850trik_lcd_backlight_ctrl(bool _backlight)
{
	gpio_set_value(GPIO_TO_PIN(6, 12), _backlight);
}

static void da850trik_lcd_power_ctrl(bool _power_up)
{
	if (_power_up) {
		gpio_set_value(GPIO_TO_PIN(8, 10), 0);
		udelay(10);
		gpio_set_value(GPIO_TO_PIN(8, 10), 1);
	} else {
		gpio_set_value(GPIO_TO_PIN(8, 10), 0);
	}
}

#define DA8XX_LCD_CNTRL_BASE		0x01e13000
static struct resource da850trik_lcdc_resources[] = {
	[0] = { /* registers */
		.start  = DA8XX_LCD_CNTRL_BASE,
		.end    = DA8XX_LCD_CNTRL_BASE + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = { /* interrupt */
		.start  = IRQ_DA8XX_LCDINT,
		.end    = IRQ_DA8XX_LCDINT,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct da8xx_ili9340_pdata da850trik_lcdc_pdata = {
	.xres			= 240,
	.yres			= 320,
	.xflip			= true,
	.yflip			= false,
	.xyswap			= false,
	.visual_mode		= DA8XX_LCDC_VISUAL_565,
	.screen_height		= 49, //48,96mm
	.screen_width		= 37, //36,72mm
	.fps			= 50, //20ms delay between memory write and redrawing

	.lcdc_lidd_mode		= DA8XX_LCDC_LIDD_MODE_8080ASYNC,
	.lcdc_lidd_cs		= DA8XX_LCDC_LIDD_CS0,
	.lcdc_lidd_ale_pol	= DA8XX_LCDC_LIDD_POL_ACTIVE_LOW,
	.lcdc_lidd_rs_en_pol	= DA8XX_LCDC_LIDD_POL_ACTIVE_LOW,
	.lcdc_lidd_ws_dir_pol	= DA8XX_LCDC_LIDD_POL_ACTIVE_LOW,
	.lcdc_lidd_cs_pol	= DA8XX_LCDC_LIDD_POL_ACTIVE_LOW,
	.lcdc_lidd_edma_burst	= 16,

        // Data taken from ch 19.3.2
	.lcdc_lidd_mclk_ns	= 0,	// Although not actually exported, it is used as granularity for timings below
					// Run at LCD_CLK for best granularity
	.lcdc_t_ta_ns		= 0,	// Tchw=0
	.lcdc_t_rhold_ns	= 90,	// Taht=10, Trdh=90/90
	.lcdc_t_rstrobe_ns	= 355,	// Trdl=45/355, Trcs=45/355
	.lcdc_t_rsu_ns		= 0,	// Tast=0, no CS->RD_low timeout
	.lcdc_t_whold_ns	= 33,   // Taht=10, Twrh=33, Tdht=10
	.lcdc_t_wstrobe_ns	= 33,	// Twrl=33, Tcs=15
	.lcdc_t_wsu_ns		= 0,	// Tast=0, no CS->WR_low timeout

	.display_t_reset_to_ready_ms	= 120,
	.display_t_sleep_in_out_ms	= 120,

	.display_idle			= false,
	.display_backlight		= true,
	.display_brightness		= 0x100,
	.display_inversion		= false,
	.display_gamma			= DA8XX_LCDC_DISPLAY_GAMMA_DEFAULT,

	.cb_power_ctrl		= &da850trik_lcd_power_ctrl,
	.cb_backlight_ctrl	= &da850trik_lcd_backlight_ctrl,

};

static struct platform_device da850trik_lcdc_device = {
	.name		= "da8xx_lcdc_ili9340",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(da850trik_lcdc_resources),
	.resource	= da850trik_lcdc_resources,
	.dev = {
		.platform_data 		= &da850trik_lcdc_pdata,
	},
};

static __init int da850trik_lcd_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(da850_lcdcntl_pins);
	if (ret)
		pr_warning("%s: LCD ctrl pinmux setup failed: %d\n", __func__, ret);

	ret = davinci_cfg_reg_list(da850trik_lcd_extra_pins);
	if (ret)
		pr_warning("%s: LCD extra pinmux setup failed: %d\n", __func__, ret);

	ret = gpio_request_one(GPIO_TO_PIN(6, 12), GPIOF_OUT_INIT_LOW, "LCD backlight");
	if (ret)
		pr_warning("%s: LCD backlight gpio request failed: %d\n", __func__, ret);

	ret = gpio_request_one(GPIO_TO_PIN(8, 10), GPIOF_OUT_INIT_LOW, "LCD reset");
	if (ret)
		pr_warning("%s: LCD reset gpio request failed: %d\n", __func__, ret);

	ret = platform_device_register(&da850trik_lcdc_device);
	if (ret) {
		pr_err("%s: LCD platform device register failed: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}




/*
 * platform driver
 */
static ssize_t da850trik_mmc_cd_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", da850trik_mmc_get_cd(0));
}

static ssize_t da850trik_audio_codec_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", !gpio_get_value(DA850TRIK_AUDIO_RESET_PIN) ? "reset" : "normal");
}

static ssize_t da850trik_audio_codec_write(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	size_t length;

	if (count < 1) return -EINVAL;
	length = (buf[count - 1] == '\n') ? count - 1 : count;

	if (strncasecmp(buf, "reset", length) == 0){
		gpio_set_value(DA850TRIK_AUDIO_RESET_PIN, 0);
		return count;
	}
	if (strncasecmp(buf, "normal", length) == 0){
		gpio_set_value(DA850TRIK_AUDIO_RESET_PIN, 1);
		return count;
	}
	return -EINVAL;
}

static ssize_t da850trik_wlan_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (da850trik_wlan_power_state < 0) return sprintf(buf, "power_off\n");
	else return sprintf(buf, "%s\n", da850trik_wlan_power_state ? "on" : "off");
}

static ssize_t da850trik_wlan_write(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	size_t length;

	if (count < 1) return -EINVAL;
	length = (buf[count - 1] == '\n') ? count - 1 : count;

	if (strncasecmp(buf, "power_on", length) == 0){
		da850trik_wl1271_power_on();
		return count;
	}
	if (strncasecmp(buf, "on", length) == 0){
		da850trik_wlan_set_power(1);
		return count;
	}
	if (strncasecmp(buf, "off", length) == 0){
		da850trik_wlan_set_power(0);
		return count;
	}
	return -EINVAL;
}

static ssize_t da850trik_bluetooth_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (da850trik_bluetooth_power_state < 0) return sprintf(buf, "power_off\n");
	else return sprintf(buf, "%s\n", da850trik_bluetooth_power_state ? "on" : "off");
}

static ssize_t da850trik_bluetooth_write(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	size_t length;

	if (count < 1) return -EINVAL;
	length = (buf[count - 1] == '\n') ? count - 1 : count;

	if (strncasecmp(buf, "power_on", length) == 0){
		da850trik_wl1271_power_on();
		return count;
	}
	if (strncasecmp(buf, "on", length) == 0){
		da850trik_bluetooth_set_power(1);
		return count;
	}
	if (strncasecmp(buf, "off", length) == 0){
		da850trik_bluetooth_set_power(0);
		return count;
	}
	return -EINVAL;
}

static ssize_t da850trik_msp430_test_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(DA850TRIK_MSP430_TEST_PIN));
}

static ssize_t da850trik_msp430_test_write(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	size_t length;

	if (count < 1) return -EINVAL;
	length = (buf[count - 1] == '\n') ? count - 1 : count;

	if (strncasecmp(buf, "0", length) == 0){
		gpio_set_value(DA850TRIK_MSP430_TEST_PIN, 0);
		return count;
	}
	if (strncasecmp(buf, "1", length) == 0){
		gpio_set_value(DA850TRIK_MSP430_TEST_PIN, 1);
		return count;
	}
	return -EINVAL;
}

static ssize_t da850trik_msp430_rst_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(DA850TRIK_MSP430_RESET_PIN));
}

static ssize_t da850trik_msp430_rst_write(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	size_t length;

	if (count < 1) return -EINVAL;
	length = (buf[count - 1] == '\n') ? count - 1 : count;

	if (strncasecmp(buf, "0", length) == 0){
		gpio_set_value(DA850TRIK_MSP430_RESET_PIN, 0);
		return count;
	}
	if (strncasecmp(buf, "1", length) == 0){
		gpio_set_value(DA850TRIK_MSP430_RESET_PIN, 1);
		return count;
	}
	return -EINVAL;
}

static const DEVICE_ATTR(mmc_card_detect,   0444, da850trik_mmc_cd_read,      NULL);
static const DEVICE_ATTR(audio_codec,       0644, da850trik_audio_codec_read, da850trik_audio_codec_write);
static const DEVICE_ATTR(wlan,              0644, da850trik_wlan_read,        da850trik_wlan_write);
static const DEVICE_ATTR(bluetooth,         0644, da850trik_bluetooth_read,   da850trik_bluetooth_write);
static const DEVICE_ATTR(msp430_test,       0644, da850trik_msp430_test_read, da850trik_msp430_test_write);
static const DEVICE_ATTR(msp430_reset,      0644, da850trik_msp430_rst_read,  da850trik_msp430_rst_write);

static const struct attribute *da850trik_manage_attrs[] = {
	&dev_attr_mmc_card_detect.attr,
	&dev_attr_audio_codec.attr,
	&dev_attr_wlan.attr,
	&dev_attr_bluetooth.attr,
	&dev_attr_msp430_test.attr,
	&dev_attr_msp430_reset.attr,
	NULL,
};

static const struct attribute_group da850trik_manage_attrs_group = {
	.attrs = (struct attribute **) da850trik_manage_attrs,
};

static int __devinit da850trik_manage_probe(struct platform_device *pdev)
{
	int ret;

	ret = sysfs_create_group(&pdev->dev.kobj, &da850trik_manage_attrs_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to export da850trik state, "
			            "error: %d\n", ret);
		platform_set_drvdata(pdev, NULL);
		return ret;
	}
	return 0;
}

static int __devexit da850trik_manage_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &da850trik_manage_attrs_group);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver da850trik_manage_driver = {
	.probe		= da850trik_manage_probe,
	.remove		= __devexit_p(da850trik_manage_remove),
	.driver		= {
		.name	= "da850trik",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device da850trik_manage_device = {
	.name		= "da850trik",
	.id		= -1,
	.num_resources	= 0,
};

module_platform_driver(da850trik_manage_driver);








/*
 * The following EDMA channels/slots are not being used by drivers (for
 * example: Timer, GPIO, UART events etc) on da850/omap-l138 EVM/Hawkboard,
 * hence they are being reserved for codecs on the DSP side.
 */
static const s16 da850_dma0_rsv_chans[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma0_rsv_slots[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30, 50},
	{-1, -1}
};

static const s16 da850_dma1_rsv_chans[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma1_rsv_slots[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30, 90},
	{-1, -1}
};

static struct edma_rsv_info da850_edma_cc0_rsv = {
	.rsv_chans	= da850_dma0_rsv_chans,
	.rsv_slots	= da850_dma0_rsv_slots,
};

static struct edma_rsv_info da850_edma_cc1_rsv = {
	.rsv_chans	= da850_dma1_rsv_chans,
	.rsv_slots	= da850_dma1_rsv_slots,
};

static struct edma_rsv_info *da850_edma_rsv[2] = {
	&da850_edma_cc0_rsv,
	&da850_edma_cc1_rsv,
};

#ifdef CONFIG_CPU_FREQ
static __init int da850trik_init_cpufreq(void)
{
	switch (system_rev & 0xF) {
	case 3:
		da850_max_speed = 456000;
		break;
	case 2:
		da850_max_speed = 408000;
		break;
	case 1:
		da850_max_speed = 372000;
		break;
	}

	return da850_register_cpufreq("pll0_sysclk3");
}
#else
static __init int da850_evm_init_cpufreq(void) { return 0; }
#endif
static struct gpio_led da850trik_leds[] = {
        {
                .active_low = 1,
                .gpio = DA850TRIK_LED_CON_PIN, /* assigned at runtime */
                .name = "led_ctrl", /* assigned at runtime */
        },
};
static struct gpio_led_platform_data da850trik_leds_pdata = {
        .leds = da850trik_leds,
        .num_leds = ARRAY_SIZE(da850trik_leds),
};

static struct platform_device da850trik_leds_device = {
        .name           = "leds-gpio",
        .id             = -1,
        .dev = {
                .platform_data = &da850trik_leds_pdata
        }
};


static __init void da850trik_led_init(void){
	int ret;

	ret = davinci_cfg_reg(DA850TRIK_LED_CON_REG);
	if (ret){
		pr_warning("Could not req cfg LEDS");
	}
	gpio_request_one(DA850TRIK_LED_CON_PIN, GPIOF_OUT_INIT_LOW, "LED");

	ret = platform_device_register(&da850trik_leds_device);
        if (ret) {
                pr_warning("Could not register baseboard GPIO expander LEDS");
        }
}



static __init void da850trik_init(void)
{
	int ret;

	da850trik_serial_init();

	ret = da850_register_edma(da850_edma_rsv);
	if (ret)
		pr_warning("%s: EDMA registration failed: %d\n",
			__func__, ret);

	ret = da8xx_register_watchdog();
	if (ret)
		pr_warning("%s: watchdog registration failed: %d\n",
			__func__, ret);

	ret = da8xx_register_rtc();
	if (ret)
		pr_warning("%s: rtc setup failed: %d\n",
			__func__, ret);

	ret = da850trik_init_cpufreq();
	if (ret)
		pr_warning("%s: cpufreq registration failed: %d\n",
			__func__, ret);

	ret = da8xx_register_cpuidle();
	if (ret)
		pr_warning("%s: cpuidle registration failed: %d\n",
			__func__, ret);

	ret = da850_register_pm(&da850_pm_device);
	if (ret)
		pr_warning("%s: suspend registration failed: %d\n",
			__func__, ret);

	da850trik_mmc_init();
	da850trik_usb_init();
	da850trik_gpio_keys_init();
	da850trik_wl1271_init();
	da850trik_audio_init();
	da850trik_accel_init();
	da850trik_i2c0_init();
	da850trik_i2c1_init();
	da850trik_spi0_init();
	da850trik_lcd_init();
	da850trik_msp430_init();
	if (HAS_EHRPWM) {
		da850trik_pwm_init();
	}
	if (HAS_ECAP_PWM) {
		da850trik_cap_init();
	}
	da850trik_led_init();
	platform_device_register(&da850trik_manage_device);
}


static void __init da850trik_map_io(void)
{
	da850_init();
	printk("**da850init**\n");
}

MACHINE_START(OMAPL138_TRIKBOARD, "AM18x/OMAP-L138 Trikboard")
	.atag_offset	= 0x100,
	.map_io		= da850trik_map_io,
	.init_irq	= cp_intc_init,
	.timer		= &davinci_timer,
	.init_machine	= da850trik_init,
	.init_late	= davinci_init_late,
	.dma_zone_size	= SZ_128M,
	.restart	= da8xx_restart,
MACHINE_END
