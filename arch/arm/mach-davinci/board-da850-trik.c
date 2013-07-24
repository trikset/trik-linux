/*
 * Trikboard based on TI's OMAP-L138 Platform
 *
 * Initial code: Syed Mohammed Khasim
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com
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

#include <linux/wl12xx.h>
#include <linux/da8xx-ili9340-fb.h>
#include <linux/l3g42xxd.h>
 

static const short da850_trik_uart0_pins[] __initconst = {
	DA850_UART0_RXD, DA850_UART0_TXD,
	DA850_NUART0_CTS,DA850_NUART0_RTS,
	-1
};
static const short da850_trik_uart1_pins[] __initconst = {
	DA850_UART1_RXD, DA850_UART1_TXD,
	DA850_GPIO0_13,DA850_GPIO1_14,
	-1
};

static struct davinci_uart_config da850_trik_uart_config __initdata = {
	.enabled_uarts = 0x7
};
static __init int da850_trik_uart0(void){
	int ret;

	ret = davinci_cfg_reg_list(da850_trik_uart0_pins);
	if (ret){
		pr_err("%s: UART0 mux setup failed: %d\n",__func__,ret);
		return ret;
	}
	return 0;
}

#warning TODO module parameters uart1 debug on/off
static __init int da850_trik_uart1(void){
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_uart1_pins);
	if (ret){
		pr_err("%s: UART1 mux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request_one(GPIO_TO_PIN(0,13), GPIOF_OUT_INIT_HIGH, "ext UART1 enable");
	if (ret){
		pr_warning("%s: ext UART1 enable gpio request failed: %d\n", __func__, ret);
	}
	ret = gpio_request_one(GPIO_TO_PIN(1,14), GPIOF_OUT_INIT_LOW, "ext UART1 power enable");
	if (ret){
		pr_warning("%s: ext UART1 power enable gpio request failed: %d\n", __func__, ret);
	}
	ret = gpio_export(GPIO_TO_PIN(0,13), 1);
	if (ret){
		pr_warning("%s: ext UART1 enable gpio export failed: %d\n", __func__, ret);
	}
	ret = gpio_export(GPIO_TO_PIN(1,14), 1);
	if (ret){
		pr_warning("%s: ext UART1 power enable gpio export failed: %d\n", __func__, ret);
	}
	return 0;

}
static __init int da850_trik_serial_init(void)
{
	int ret;
	
	ret = davinci_serial_init(&da850_trik_uart_config);
	if (ret){
		pr_err("%s: serial init failed: %d\n", __func__, ret);
		return ret;
	}
	ret = da850_trik_uart0();
	if (ret)
		pr_warning("%s: uart0 init failed: %d\n", __func__, ret);
	
	ret = da850_trik_uart1();
	if (ret)
		pr_warning("%s: uart1 init failed: %d\n", __func__, ret);
	
	return 0;
}

#warning TODO setter cpu frequency
static __init int da850_trik_init_cpufreq(void){
	return 0;
}

#warning TODO setter cpu cpuidle
static __init int da850_trik_init_cpuidle(void){
	int ret;
	ret = da8xx_register_cpuidle();
	if (ret) {
		pr_err("%s: cpuidle registration failed: %d\n", __func__, ret);
		return ret;
	}
	return 0;
}

static struct davinci_pm_config da850_trik_pm_pdata = {
	.sleepcount = 128,
};

static struct platform_device da850_trik_pm_device = {
	.name           = "pm-davinci",
	.dev = {
		.platform_data	= &da850_trik_pm_pdata,
	},
	.id             = -1,
};
#warning TODO setter cpu suspend
static __init int da850_trik_init_cpususpend(void){
	
	int ret;

	ret = da850_register_pm(&da850_trik_pm_device);
	if (ret) {
		pr_err("%s: suspend registration failed: %d\n", __func__, ret);
		return ret;
	}
	return 0;
}

static const short da850_trik_sd0_pins[] __initconst = {
	DA850_MMCSD0_DAT_0, DA850_MMCSD0_DAT_1, DA850_MMCSD0_DAT_2,
	DA850_MMCSD0_DAT_3, DA850_MMCSD0_CLK, DA850_MMCSD0_CMD,
	DA850_GPIO4_1,
	-1
};
static int da850_trik_sd0_get_cd(int index)
{
	return !gpio_get_value(GPIO_TO_PIN(4,1));
}
static struct davinci_mmc_config da850_trik_sd0_config = {
	.get_cd		= da850_trik_sd0_get_cd,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
	.version	= MMC_CTLR_VERSION_2,
};
static __init int da850_trik_sd0_init(void){
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_sd0_pins);
	if (ret) {
		pr_err("%s: MMC/SD0 mux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request(GPIO_TO_PIN(4,1),"SD0 card detect");
	if (ret){
		pr_err("%s: SD0 card detect gpio request failed: %d\n", __func__,ret);
		return ret;
	}
	ret = gpio_direction_input(GPIO_TO_PIN(4,1));
	if (ret){
		pr_err("%s: SD0 card detect gpio direction input failed: %d\n", __func__,ret);
		goto exit_sd0_init;
	}
	ret = da8xx_register_mmcsd0(&da850_trik_sd0_config);
	if (ret) {
		pr_err("%s: MMC/SD registration failed: %d\n",__func__, ret);
		goto exit_sd0_init;
	}
	return 0;
exit_sd0_init:
	gpio_free(GPIO_TO_PIN(4,1));
	return ret;
}
#warning Temp solution 

static int jd1_jd2 = 1; // '1' - gy-80; '0' - bwsensor

EXPORT_SYMBOL (jd1_jd2);
static int __init set_jd1_d2(char *str)
{
	if (!strcasecmp(str,"gy-80"))
		jd1_jd2 = 1;
	else if (!strcasecmp(str,"bwsensor"))
		jd1_jd2 = 0;
	else 
		return 1;
	return 0;
}

__setup("trik.jd1_jd2=", set_jd1_d2);


/* JD1 & JD2 pins init */

static const short jd1_jd2_pins[] = {
	DA850_GPIO3_3,	/*D1A*/
	DA850_GPIO3_2,	/*D1B*/
	DA850_GPIO3_5,	/*D2B*/
	DA850_GPIO3_1,	/*D2A*/
        -1
};

static void trik_sensor_init(void){
	//init gpio
	//
	u32 cfgchip2 = 0;
	int ret = 0;
	pr_err("%s start\n",__func__);
	// cfgchip2 = __raw_readl(DA8XX_SYSCFG1_VIRT(DA8XX_PUPD_ENA));
	// //cfgchip2 &= 0x0000;
	// cfgchip2 |= 0x0200;
	// __raw_writel(cfgchip2,DA8XX_SYSCFG1_VIRT(DA8XX_PUPD_ENA));

	// cfgchip2 = __raw_readl(DA8XX_SYSCFG1_VIRT(DA8XX_PUPD_SEL));	
	// cfgchip2 |= 0x0000;
	// __raw_writel(cfgchip2,DA8XX_SYSCFG1_VIRT(DA8XX_PUPD_SEL));

	ret = davinci_cfg_reg_list(jd1_jd2_pins);
        if (ret) {
                pr_err("%s: trik_sensor mux setup failed: %d\n",
                        __func__, ret);
                return;
        }
	ret = gpio_request_one(GPIO_TO_PIN(3, 3), GPIOF_OUT_INIT_LOW, "D1A");
    ret = gpio_request_one(GPIO_TO_PIN(3, 1), GPIOF_OUT_INIT_LOW, "D2A");

    ret = gpio_request_one(GPIO_TO_PIN(3, 2), GPIOF_IN, "D1B");
    ret = gpio_request_one(GPIO_TO_PIN(3, 5), GPIOF_IN, "D2B");
    
    gpio_export(GPIO_TO_PIN(3, 1),1);
	gpio_export(GPIO_TO_PIN(3, 2),1);

    gpio_export(GPIO_TO_PIN(3, 3),1);
    gpio_export(GPIO_TO_PIN(3, 5),1);
    
    gpio_set_value(GPIO_TO_PIN(3, 3),0);
	gpio_set_value(GPIO_TO_PIN(3, 1),0);
	
	gpio_set_value(GPIO_TO_PIN(3, 2),0);
	gpio_set_value(GPIO_TO_PIN(3, 5),0);

    pr_err("%s end\n",__func__);
}


static struct i2c_board_info __initdata da850_trik_i2c0_devices[] = {
	{	
		I2C_BOARD_INFO("l3g42xxd", 0x69),
	},
	{
		I2C_BOARD_INFO("adxl345b", 0x53),
	},
	{
		I2C_BOARD_INFO("bmp085",0x77),
	},
	{
		I2C_BOARD_INFO("hmc5883l",0x1e),
	}
};
static struct davinci_i2c_platform_data da850_trik_i2c0_pdata = {
	.bus_freq	= 100,	/* kHz */
	.bus_delay	= 0,	/* usec */
};

static __init int da850_trik_i2c0_init(void)
{
	int ret;
	ret = davinci_cfg_reg_list(da850_i2c0_pins);
	if (ret){
		pr_err("%s: I2C0 mux setup failed: %d\n", __func__, ret);
	}
	if (jd1_jd2){
		trik_sensor_init();
		
		da850_trik_i2c0_devices[0].irq = gpio_to_irq(GPIO_TO_PIN(3,5));
		da850_trik_i2c0_devices[1].irq = gpio_to_irq(GPIO_TO_PIN(3,2));
		
		ret = i2c_register_board_info(1,da850_trik_i2c0_devices,ARRAY_SIZE(da850_trik_i2c0_devices));
		if (ret){
			pr_err("%s: I2C0 register board info failed: %d\n", __func__, ret);
			return ret;
		}
	}
	ret = da8xx_register_i2c(0, &da850_trik_i2c0_pdata);
	if (ret){
		pr_err("%s: I2C0 register failed: %d\n", __func__, ret);
		return ret;
	}
	return 0;
}
static const short da850_trik_accel_pins[] __initconst = {
	DA850_GPIO5_4,DA850_GPIO5_3,
	-1
};
static struct i2c_board_info __initdata da850_trik_i2c1_devices[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	{
		I2C_BOARD_INFO("mma8451q", 0x1C),
	},
	{
		#warning add i2c alsa device driver
		I2C_BOARD_INFO("ds4420",0x50),
	},
};
static struct davinci_i2c_platform_data da850_trik_i2c1_pdata = {
	.bus_freq	= 100,	/* kHz */
	.bus_delay	= 0,	/* usec */
};

static __init int da850_trik_i2c1_init(void)
{
	int ret; 

	ret = davinci_cfg_reg_list(da850_i2c1_pins);
	if (ret){
		pr_err("%s: I2C1 mux setup failed: %d\n", __func__, ret);
	}

	ret = davinci_cfg_reg_list(da850_trik_accel_pins);
	if (ret){
		pr_err("%s: accel mux setup failed: %d\n", __func__, ret);
	}
	
	da850_trik_i2c1_devices[1].irq = gpio_to_irq(GPIO_TO_PIN(5,4));

	ret = i2c_register_board_info(2,da850_trik_i2c1_devices,ARRAY_SIZE(da850_trik_i2c1_devices));
	if (ret){
		pr_err("%s: I2C1 register board info failed: %d\n", __func__, ret);
		return ret;
	}
	
	ret = da8xx_register_i2c(1, &da850_trik_i2c1_pdata);
	if (ret){
		pr_err("%s: I2C1 register failed: %d\n", __func__, ret);
		return ret;
	}
	return 0;
}

static struct mtd_partition da850_trik_spiflash_parts[] = {
	[0] = {
		.name = "uboot",
		.offset = 0,
		.size = SZ_256K,
		.mask_flags = MTD_WRITEABLE,
	},
	[1] = {
		.name = "uboot-env1",
		.offset = MTDPART_OFS_APPEND,
		.size = SZ_256K,
		.mask_flags = MTD_WRITEABLE,
	},
	[2] = {
		.name = "uboot-env2",
		.offset = MTDPART_OFS_APPEND,
		.size = SZ_256K,
		.mask_flags = MTD_WRITEABLE,
	},
	[3] = {
		.name = "config-periph",
		.offset = MTDPART_OFS_APPEND,
                .size = SZ_256K,
                .mask_flags = MTD_WRITEABLE,
	},
	[4] = {
		.name = "kernel",
		.offset = MTDPART_OFS_APPEND,
		.size = SZ_2M+SZ_1M,
	},
	[5] = {
		.name = "RootFS",
		.offset = MTDPART_OFS_APPEND,
		.size   = MTDPART_SIZ_FULL,
	},
};
static const struct flash_platform_data da850_trik_spiflash_data = {
	.name		= "m25p80",
	.parts		= da850_trik_spiflash_parts,
	.nr_parts	= ARRAY_SIZE(da850_trik_spiflash_parts),
	.type		= "m25p128",
};
static struct davinci_spi_config da850_trik_spi0_cfg = {
	.io_type	= SPI_IO_TYPE_DMA,
	.c2tdelay	= 8,
	.t2cdelay	= 8,
};

static struct spi_board_info da850_trik_spi0_info[] = {
	[0] = {
		.modalias			= "m25p80",
		.controller_data	= &da850_trik_spi0_cfg,
		.platform_data		= &da850_trik_spiflash_data,
		.mode				= SPI_MODE_0,
		.max_speed_hz		= 25000000,
		.bus_num			= 0,
		.chip_select		= 0,
	},
};
static const short da850_trik_spi0_pins[] __initconst = {
	DA850_SPI0_SIMO,DA850_SPI0_SOMI,
	DA850_SPI0_CS_0,DA850_SPI0_CLK,
	-1
};
static __init int da850_trik_spi0_init(void)
{
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_spi0_pins);
	if (ret){
		pr_err("%s: spi0 pinmux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = da8xx_register_spi(0, da850_trik_spi0_info, ARRAY_SIZE(da850_trik_spi0_info));
	if (ret) {
		pr_err("%s: spi0 setup failed: %d\n",__func__,  ret);
		return ret;
	}
	return 0;	
}
static struct davinci_spi_config da850_trik_spi1_cfg = {
	.io_type	= SPI_IO_TYPE_DMA,
	.c2tdelay	= 8,
	.t2cdelay	= 8,
};
const short da850_trik_gyro_pins[] __initconst = {
	DA850_GPIO2_9,DA850_GPIO2_8,
	-1
};
static struct spi_board_info da850_trik_spi1_info[] = {
	[0] = {
		.modalias               = "",                  /* Stub */
		.mode                   = SPI_MODE_0,
		.max_speed_hz           = 10000000,       /* max sample rate at 3V */
		.bus_num                = 1,
		.chip_select            = 0,
        },
	[1] = {
		.modalias 		= "l3g42xxd",
		.controller_data 	= &da850_trik_spi1_cfg,
		.platform_data 		= NULL,
		.mode 			= SPI_MODE_0, //SPI_NO_CS
		.max_speed_hz		= 10000000,
		.bus_num		= 1,
		.chip_select		= 1,
	},
};
const short da850_trik_spi1_pins[] __initconst = {
	DA850_SPI1_SIMO,
	DA850_SPI1_SOMI,
	DA850_SPI1_CLK,
	DA850_GPIO2_7, 	/* TP9	*/
	-1
};

static u8 da850_trik_spi1_chipselect[] = { SPI_INTERN_CS, GPIO_TO_PIN(2,7)};

#warning TO DO match driver and gpio irq
static __init int da850_trik_spi1_init(void)
{
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_spi1_pins);
	if (ret){
		pr_err("%s: spi1 pinmux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = davinci_cfg_reg_list(da850_trik_gyro_pins);
	if (ret){
		pr_err("%s: gyro pinmux setup failed: %d\n", __func__, ret);
		return ret;
	}

	da8xx_spi_pdata[1].num_chipselect        = ARRAY_SIZE(da850_trik_spi1_chipselect);
	da8xx_spi_pdata[1].chip_sel                = da850_trik_spi1_chipselect;

	da850_trik_spi1_info[1].irq = gpio_to_irq(GPIO_TO_PIN(2,8));

	ret = gpio_request_one(GPIO_TO_PIN(2,7),GPIOF_OUT_INIT_HIGH, "GYRO_CS(TP9)");
	if (ret){
		pr_warning("%s: can not open spi1 GYRO_CS(TP9) : %d\n", __func__, ret);
	}

	ret = da8xx_register_spi(1, da850_trik_spi1_info, ARRAY_SIZE(da850_trik_spi1_info));
	if (ret) {
		pr_err("da837_init_spi1: spi1 setup failed: %d\n", ret);
		gpio_free(GPIO_TO_PIN(2,7));
		return ret;
	}

	return 0;
}
static u8 da850_trik_iis_serializer_direction[] = {
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	TX_MODE,
	INACTIVE_MODE,	RX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static const short da850_trik_mcasp_pins[] __initconst = {
	DA850_AHCLKX, DA850_ACLKX, 
	DA850_AFSX, DA850_AXR_7,/* TX */
	DA850_AXR_9,/* RX */DA850_GPIO6_15,/*RESET*/
	-1
};
static struct snd_platform_data da850_trik_snd_data = {
	.tx_dma_offset	= 0x2000,
	.rx_dma_offset	= 0x2000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(da850_trik_iis_serializer_direction),
	.tdm_slots	= 2,
	.serial_dir	= da850_trik_iis_serializer_direction,
	.asp_chan_q	= EVENTQ_1,
	.version	= MCASP_VERSION_2,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};
static __init int da850_trik_audio_init(void)
{
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_mcasp_pins);
	if (ret){
		pr_err("%s: mcasp pinmux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request(GPIO_TO_PIN(6,15), "audio codec reset");
	if (ret) {
		pr_err("%s:  GPIO  audio reset pin request failed: %d\n",
			__func__, ret);
		return ret;
	}
	ret = gpio_direction_output(GPIO_TO_PIN(6,15), 1);
	if (ret) {
		pr_err("%s: GPIO audio reset pin direction request failed: %d\n", __func__, ret);
		goto exit_audio_init;
	}
	ret = gpio_export(GPIO_TO_PIN(6,15), 1);
	if (ret) {
		pr_err("%s: GPIO audio reset pin export failed: %d \n", __func__, ret);
		goto exit_audio_init;
	}
	da8xx_register_mcasp(0,&da850_trik_snd_data);
	return 0;
exit_audio_init:
	gpio_free(GPIO_TO_PIN(4,1));
	return ret;
}

/*
 * ILI9340-based LCD
 */


int display_orientation = 1; // '1' - landscape; '0' - portrait

EXPORT_SYMBOL (display_orientation);
static int __init set_orientation(char *str)
{
	if (!strcasecmp(str,"landscape"))
		display_orientation = 1;
	else if (!strcasecmp(str,"portrait"))
		display_orientation = 0;
	else 
		return 1;
	return 0;
}


__setup("trik.display_orientation=", set_orientation);

static const short da850_trik_lcd_extra_pins[] __initconst = {
	DA850_GPIO6_12, // LCD backlight
	DA850_GPIO8_10, // LCD reset
	-1
};

static void da850_trik_lcd_backlight_ctrl(bool _backlight)
{
	gpio_set_value(GPIO_TO_PIN(6, 12), _backlight);
}

static void da850_trik_lcd_power_ctrl(bool _power_up)
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
static struct resource da850_trik_lcdc_resources[] = {
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

static struct da8xx_ili9340_pdata da850_trik_lcdc_pdata = {
	.visual_mode		= DA8XX_LCDC_VISUAL_565,
	.visual_mode_red_blue_swap	= true, // fix for NewHeaven display with messed red and blue components
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

	.cb_power_ctrl		= &da850_trik_lcd_power_ctrl,
	.cb_backlight_ctrl	= &da850_trik_lcd_backlight_ctrl,

};

static struct platform_device da850_trik_lcdc_device = {
	.name		= "da8xx_lcdc_ili9340",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(da850_trik_lcdc_resources),
	.resource	= da850_trik_lcdc_resources,
	.dev = {
		.platform_data 		= &da850_trik_lcdc_pdata,
	},
};

static __init int da850_trik_lcd_init(void){
	int ret;

	ret = davinci_cfg_reg_list(da850_lcdcntl_pins);
	if (ret) {
		pr_err("%s: LCD ctrl pinmux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = davinci_cfg_reg_list(da850_trik_lcd_extra_pins);
	if (ret) {
		pr_err("%s: LCD extra pinmux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request_one(GPIO_TO_PIN(6, 12), GPIOF_OUT_INIT_LOW, "LCD backlight");
	if (ret){
		pr_err("%s: LCD backlight gpio request failed: %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request_one(GPIO_TO_PIN(8, 10), GPIOF_OUT_INIT_LOW, "LCD reset");
	if (ret) {
		pr_warning("%s: LCD reset gpio request failed: %d\n", __func__, ret);
		goto exit_request_one;
	}
	if (display_orientation){
		da850_trik_lcdc_pdata.xres			= 320;
		da850_trik_lcdc_pdata.yres			= 240;
		da850_trik_lcdc_pdata.xflip			= false;
		da850_trik_lcdc_pdata.yflip			= false;
		da850_trik_lcdc_pdata.xyswap			= true;
		da850_trik_lcdc_pdata.screen_height		= 37; //36,72mm
		da850_trik_lcdc_pdata.screen_width		= 49; //48,96mm
	}
	else 
	{
		da850_trik_lcdc_pdata.xres			= 240;
		da850_trik_lcdc_pdata.yres			= 320;
		da850_trik_lcdc_pdata.xflip			= true;
		da850_trik_lcdc_pdata.yflip			= false;
		da850_trik_lcdc_pdata.xyswap			= false;
		da850_trik_lcdc_pdata.screen_height		= 49; //48,96mm
		da850_trik_lcdc_pdata.screen_width		= 37; //36,72mm
	}

	ret = platform_device_register(&da850_trik_lcdc_device);
	if (ret) {
		pr_err("%s: LCD platform device register failed: %d\n", __func__, ret);
		goto exit_register_device;
	}
	return 0;
exit_register_device:
	gpio_free(GPIO_TO_PIN(8, 10));
exit_request_one:
	gpio_free(GPIO_TO_PIN(6, 12));
	return ret;
}
static const short da850_trik_leds_pins[] __initconst = {
	DA850_GPIO5_9,
	DA850_GPIO5_8,
	DA850_GPIO5_7,
	-1
};
static struct gpio_led da850_trik_leds[] = {
	{
		.active_low = 1,
		.gpio = GPIO_TO_PIN(5,7), /* assigned at runtime */
		.name = "led_red", /* assigned at runtime */
	},
	{
		.active_low = 1,
		.gpio = GPIO_TO_PIN(5,8), /* assigned at runtime */
		.name = "led_green", /* assigned at runtime */
	},
	{
		.active_low = 1,
		.gpio = GPIO_TO_PIN(5,9), /* assigned at runtime */
		.name = "led_power", /* assigned at runtime */
	},

};
static struct gpio_led_platform_data da850_trik_leds_pdata = {
	.leds = da850_trik_leds,
	.num_leds = ARRAY_SIZE(da850_trik_leds),
};

static struct platform_device da850_trik_leds_device = {
	.name           = "leds-gpio",
	.id             = -1,
	.dev = {
		.platform_data = &da850_trik_leds_pdata
	}
};
static __init int da850_trik_led_init(void){
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_leds_pins);
	if (ret) {
		pr_err("%s: gpio-leds mux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = platform_device_register(&da850_trik_leds_device);
	if (ret) {
		pr_err("Could not register baseboard GPIO expander LEDS");
		return ret;
	}
	return 0;
}
#define DA850_TRIK_KEYS_DEBOUNCE_MS	10
#define DA850_TRIK_GPIO_KEYS_POLL_MS	200
static const short da850_trik_gpio_keys_pins[] __initconst = {
//	DA850_GPIO5_8,  /* sw1 */
	DA850_GPIO3_4,	/* sw2 */
	DA850_GPIO2_0,	/* sw3 */
	DA850_GPIO1_9,	/* sw4 */
	DA850_GPIO3_15,	/* sw5 */
	DA850_GPIO3_13,	/* sw6 */
	DA850_GPIO3_12,	/* sw7 */
	-1
};
static struct gpio_keys_button da850_trik_gpio_keys[] = {
#if 0
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850_TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(5, 8),
		.code		   = KEY_F1,
		.desc		   = "sw1",
	},
#endif 
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850_TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(3, 4),
		.code		   = KEY_F2,
		.desc		   = "sw2",
	},
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850_TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(2, 0),
		.code		   = KEY_F3,
		.desc		   = "sw3",
	},
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850_TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(1, 9),
		.code		   = KEY_F4,
		.desc		   = "sw4",
	},
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850_TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(3, 15),
		.code		   = KEY_F5,
		.desc		   = "sw5",
	},
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850_TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(3, 13),
		.code		   = KEY_F6,
		.desc		   = "sw6",
	},
	{
		.type		   = EV_KEY,
		.active_low	   = 1,
		.wakeup		   = 0,
		.debounce_interval = DA850_TRIK_KEYS_DEBOUNCE_MS,
		.gpio		   = GPIO_TO_PIN(3, 12),
		.code		   = KEY_F7,
		.desc		   = "sw7",
	}
};
static struct gpio_keys_platform_data da850_trik_gpio_keys_data = {
	.buttons	= da850_trik_gpio_keys,
	.nbuttons	= ARRAY_SIZE(da850_trik_gpio_keys),
	.poll_interval	= DA850_TRIK_GPIO_KEYS_POLL_MS,
};
static struct platform_device da850_trik_gpio_keys_device = {
	.name			= "gpio-keys",
	.id				= -1,
	.num_resources	= 0,
	.dev			= {
		.platform_data	= &da850_trik_gpio_keys_data,
	}
};
static __init int da850_trik_keys_init(void){
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_gpio_keys_pins);
	if (ret) {
		pr_err("%s: gpio-keys mux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = platform_device_register(&da850_trik_gpio_keys_device);
	if (ret) {
		pr_err("%s: gpio-keys platform register failed: %d\n", __func__, ret);
		return ret;
	}
	return 0;
}

/*	WL1271 Wifi module init */

int enable_wifi = 1;
EXPORT_SYMBOL (enable_wifi);
static int __init set_enable_wifi(char *str)
{
	if (!strcasecmp(str,"on"))
		enable_wifi = 1;
	else if (!strcasecmp(str,"off"))
		enable_wifi = 0;
	else 
		return 1;

	return 0;
}
__setup("trik.wifi=", set_enable_wifi);

static const short da850_trik_wifi_pins[] __initconst = {
	DA850_MMCSD1_DAT_0, DA850_MMCSD1_DAT_1, DA850_MMCSD1_DAT_2,
	DA850_MMCSD1_DAT_3, DA850_MMCSD1_CLK, DA850_MMCSD1_CMD,
	DA850_GPIO6_9, DA850_GPIO6_8, DA850_GPIO5_11,
	-1
};

static void wl12xx_set_power(int index, bool power_on)
{
	static bool power_state;

	pr_debug("Powering %s wl12xx", power_on ? "on" : "off");

	if (power_on == power_state)
		return;
	power_state = power_on;

	if (power_on) {
		/* Power up sequence required for wl127x devices */
		gpio_set_value(GPIO_TO_PIN(5,11), 1);
		usleep_range(15000, 15000);
		gpio_set_value(GPIO_TO_PIN(5,11), 0);
		usleep_range(1000, 1000);
		gpio_set_value(GPIO_TO_PIN(5,11), 1);
		msleep(70);
	} else {
		gpio_set_value(GPIO_TO_PIN(5,11), 0);
	}
}
static struct davinci_mmc_config da850_trik_wl12xx_mmc_config = {
	.set_power	= wl12xx_set_power,
	.wires		= 4,
	.max_freq	= 24000000,
	.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD| MMC_CAP_NONREMOVABLE,
	.version	= MMC_CTLR_VERSION_2,
};

static struct wl12xx_platform_data da850_trik_wl12xx_wlan_data __initdata = {
	.irq			= -1,
	.board_ref_clock	= WL12XX_REFCLOCK_38,
	.platform_quirks	= WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};




static __init int da850_trik_wifi_init(void){
	int ret;

	ret = davinci_cfg_reg_list(da850_trik_wifi_pins);
	if (ret)
		pr_warning("%s: wi-fi pin mux setup failed: %d\n", __func__, ret);

	ret = gpio_request_one(GPIO_TO_PIN(6, 8), GPIOF_OUT_INIT_HIGH, "wi-	");
	if (ret)
		pr_warning("%s: could not request wi-fi enable all gpio: %d\n", __func__, ret);

        ret = gpio_request_one(GPIO_TO_PIN(5,11), GPIOF_OUT_INIT_LOW, "wi-fi_en");
	if (ret)
		pr_warning("%s: could not request wi-fi enable gpio: %d\n", __func__, ret);

	ret = gpio_request_one(GPIO_TO_PIN(6, 9), GPIOF_IN, "wi-fi_irq");
	if (ret)
		pr_warning("%s: could not request wi-fi irq gpio: %d\n",__func__, ret);

	da850_trik_wl12xx_wlan_data.irq = gpio_to_irq(GPIO_TO_PIN(6, 9));
	ret = wl12xx_set_platform_data(&da850_trik_wl12xx_wlan_data);
	if (ret) {
		pr_err("%s: could not set wl12xx platform data: %d\n", __func__, ret);
		goto exit_release_gpio;
	}

	ret = da850_register_mmcsd1(&da850_trik_wl12xx_mmc_config);
	if (ret) {
		pr_err("%s: wl12xx/mmc registration failed: %d\n", __func__, ret);
		goto exit_release_gpio;
	}
	return 0;
exit_release_gpio:
	gpio_free(GPIO_TO_PIN(6, 9));
	gpio_free(GPIO_TO_PIN(5,11));
	gpio_free(GPIO_TO_PIN(6, 8));
	return ret;
}
static const short da850_trik_bluetooth_pins[] __initconst = {
	DA850_GPIO6_11, /*BT_EN_33 */
	DA850_GPIO6_10,  /*BT_WU_33*/
	-1
};
#warning TODO Bluetooth init
static __init int da850_trik_bluetooth_init(void){
	int ret;

	ret = davinci_cfg_reg_list(da850_trik_bluetooth_pins);
	if (ret) {
		pr_err("%s: Bluetooth mux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request_one(GPIO_TO_PIN(6, 11), GPIOF_OUT_INIT_HIGH, "BT_EN_33");
	if (ret){
		pr_warning("%s: could not request BT_EN_33 gpio: %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_export(GPIO_TO_PIN(6, 11),1);
	if (ret){
		pr_warning("%s: could not export BT_EN_33 gpio: %d\n", __func__, ret);
		gpio_free(GPIO_TO_PIN(6, 11));
		return ret;
	}

	return 0;
}
static const short da850_trik_usb_pins[] __initconst = {
	DA850_GPIO5_15,	/*USB FAULT*/
	DA850_GPIO6_1,	/*MODE_B*/
	-1
};
#warning TODO USB HOST & OTG MODE
#ifndef CONFIG_USB_MUSB_HOST
#error Only USB_MUSB_HOST supported now!
#endif

static int da850_trik_set_power(unsigned port, int on)
{
	pr_warning("%s: port - %d, value - %d\n", __func__, port, on);
	return 0;
}
static int da850_trik_get_power(unsigned port)
{
	pr_warning("%s: port - %d\n", __func__, port);
	return 0;
}
static int da850_trik_get_oci(unsigned port)
{
	return !gpio_get_value(GPIO_TO_PIN(5,15));
}

static int da850_trik_ocic_notify(da8xx_ocic_handler_t handler)
{
	pr_warning("%s: \n", __func__);
	return 0;	
}
static struct da8xx_ohci_root_hub da850_trik_usb11_pdata = {
	.set_power      = da850_trik_set_power,
	.get_power      = da850_trik_get_power,
	.get_oci        = da850_trik_get_oci,
	.ocic_notify    = da850_trik_ocic_notify,
	/* TPS2087 switch @ 5V */
	.potpgt         = (3 + 1) / 2,  /* 3 ms max */
};

static __init int da850_trik_usb_init(void){
	int ret;
	u32 cfgchip2;

	ret = davinci_cfg_reg_list(da850_trik_usb_pins);
	if (ret) {
		pr_err("%s: USB mux setup failed: %d\n", __func__, ret);
		return ret;
	}
	cfgchip2 = __raw_readl(DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));
	cfgchip2 &= ~CFGCHIP2_REFFREQ;
	cfgchip2 |=  CFGCHIP2_REFFREQ_24MHZ;

	cfgchip2 &= ~CFGCHIP2_USB1PHYCLKMUX;
	cfgchip2 |=  CFGCHIP2_USB2PHYCLKMUX;

	cfgchip2 &= ~CFGCHIP2_OTGMODE;
	cfgchip2 |=  CFGCHIP2_FORCE_HOST;

	__raw_writel(cfgchip2, DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));

	ret = gpio_request_one(GPIO_TO_PIN(5,15), GPIOF_DIR_IN, "USB Fault");
	if (ret) {
		pr_err("%s: USB Fault gpio request failed: %d\n", __func__, ret);
		goto request_usb_fault;
	}
	ret = da8xx_register_usb11(&da850_trik_usb11_pdata);
	if (ret) {
		pr_warning("%s: USB 1.1 registration failed: %d\n", __func__, ret);
		goto register_ohci;
	}

	ret = da8xx_register_usb20(500, 20);
	if (ret) {
		pr_err("%s: USB 2.0 registration failed: %d\n",__func__, ret);
		goto register_otg;
	}

	return 0;
register_otg:
register_ohci:
	gpio_free(GPIO_TO_PIN(5,15));
request_usb_fault:
	return ret;
}

static const short da850_trik_msp_pins[] __initconst = {
	DA850_GPIO5_6, /* Interrupt */
	DA850_GPIO5_5,/* TEST_MCTRL*/
	DA850_GPIO5_13, /*RESET_MCTRL*/
	-1
};


static __init int da850_trik_msp430_init(void){
	int ret;
	
	ret = davinci_cfg_reg_list(da850_trik_msp_pins);
	if (ret) {
		pr_err("%s: MSP430 mux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request_one(GPIO_TO_PIN(5,13),GPIOF_OUT_INIT_LOW,"MSP Reset");
	if (ret){
		pr_err("%s: MSP Reset gpio request failed: %d\n",__func__, ret);
		goto request_msp_reset;
	}
	ret = gpio_request_one(GPIO_TO_PIN(5,5),GPIOF_OUT_INIT_LOW,"MSP Test");
	if (ret){
		pr_err("%s: MSP Reset gpio request failed: %d\n",__func__, ret);
		goto request_msp_test;
	}

	ret = gpio_export(GPIO_TO_PIN(5,13),1);
	if (ret){
		pr_warning("%s: MSP reset gpio export failed: %d\n", __func__, ret);
	}

	
	gpio_set_value(GPIO_TO_PIN(5,13), 0);
	msleep(10);
	gpio_set_value(GPIO_TO_PIN(5,13), 1);
	msleep(10);


	return 0;
request_msp_test:
request_msp_reset:
	return ret;
}
static const short da850_trik_gpio_extra_pins[] __initconst = {
	DA850_GPIO4_12, /* TP10 	*/
	DA850_GPIO3_14, /* POWER12V */
	DA850_GPIO2_7,  /* TP9 		*/
	DA850_GPIO6_13, /* VPIF  GPIO_1*/
	DA850_GPIO2_1,  /* VPIF  GPIO_2*/

//	DA850_GPIO5_9,  /* POWER_ON */
//	DA850_GPIO3_8,  /*PWR_LEVEL	*/
	-1
};

static __init int da850_trik_gpio_extra_init(void){
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_gpio_extra_pins);
	if (ret) {
		pr_err("%s: GPIO_EXTRA mux setup failed: %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request_one(GPIO_TO_PIN(4,12),GPIOF_OUT_INIT_LOW,"TP10");
	if (ret){
		pr_err("%s: TP10 gpio request failed: %d\n",__func__, ret);
		return ret;
	}
	ret = gpio_export(GPIO_TO_PIN(4,12),1);
	if (ret){
		pr_warning("%s: TP10 gpio export failed: %d\n", __func__, ret);
	}

	ret = gpio_request_one(GPIO_TO_PIN(3,14),GPIOF_OUT_INIT_LOW,"Power12V");
	if (ret){
		pr_err("%s: Power12V gpio request failed: %d\n",__func__, ret);
		return ret;
	}
	ret = gpio_export(GPIO_TO_PIN(3,14),1);
	if (ret){
		pr_warning("%s: Power12V gpio export failed: %d\n", __func__, ret);
	}
	ret = gpio_request_one(GPIO_TO_PIN(6,13),GPIOF_OUT_INIT_LOW,"VPIF_GPIO_1");
	if (ret){
		pr_err("%s: VPIF_GPIO_1 gpio request failed: %d\n",__func__, ret);
		return ret;
	}
	ret = gpio_export(GPIO_TO_PIN(6,13),1);
	if (ret){
		pr_warning("%s: VPIF_GPIO_1 gpio export failed: %d\n", __func__, ret);
	}
	ret = gpio_request_one(GPIO_TO_PIN(2,1),GPIOF_OUT_INIT_LOW,"VPIF_GPIO_2");
	if (ret){
		pr_err("%s: VPIF_GPIO_2 gpio request failed: %d\n",__func__, ret);
		return ret;
	}
	ret = gpio_export(GPIO_TO_PIN(2,1),1);
	if (ret){
		pr_warning("%s: VPIF_GPIO_2 gpio export failed: %d\n", __func__, ret);
	}

	return 0;
}

static const short da850_trik_clk_pins[] __initconst = {
	DA850_CLKOUT0,
	-1
};
#define PLL0_OSCEL_OFFS		0x0104
#define PLL0_OSCDIV_OFFS	0x0124
#define PLL0_CKEN_OFFS		0x0148
static __init int da850_trik_buffer_clk_init(void)
{
	int 	ret;
	unsigned __iomem	*pll0_oscel, *pll0_oscdiv, *pll0_cken;

	ret = davinci_cfg_reg_list(da850_trik_clk_pins);
	if (ret) {
		pr_err("%s: clk buffer  mux setup failed: %d\n",__func__, ret);
		return ret;
	}
	if (da850_trik_pm_pdata.cpupll_reg_base) {
		pll0_oscel  = (unsigned*)(da850_trik_pm_pdata.cpupll_reg_base + PLL0_OSCEL_OFFS);
		pll0_oscdiv = (unsigned*)(da850_trik_pm_pdata.cpupll_reg_base + PLL0_OSCDIV_OFFS);
		pll0_cken   = (unsigned*)(da850_trik_pm_pdata.cpupll_reg_base + PLL0_CKEN_OFFS);

		/* select OBSCLK in OCSRC field of PLL0_OSCEL */
		*pll0_oscel = 0x14;
		/* set OD1EN=1 and RATIO=0 fields in PLL0_OSCDIV */
		*pll0_oscel = 0x8000;
		/* set OBSEN=1 field in PLL0_CKEN */
		*pll0_cken |= 0x02;
	} else {
		pr_err("%s: failed to setup PLL0_CLKOUT\n", __func__);
	}
	return 0;
}

static const short da850_trik_ehrpwm0_pins[] __initconst = {
#if 0
		DA850_EHRPWM0_A,
#endif
		DA850_EHRPWM0_B,
		DA850_GPIO2_5/*PE0_EN*/,
		-1
};

static const short da850_trik_ehrpwm1_pins[] __initconst = {
		DA850_EHRPWM1_A,
		DA850_EHRPWM1_B,
		DA850_GPIO2_3/*PE1_EN*/,
		-1
};
static __init int da850_trik_ehrpwm_init(void){
	int ret;
	char mask = 0;

	ret = davinci_cfg_reg_list(da850_trik_ehrpwm0_pins);
	if (ret){
		pr_err("%s: ehrpwm0 pins mux setup failed: %d\n", __func__, ret);
		return ret;
	}
	mask |=  BIT(1);
	ret = gpio_request_one(GPIO_TO_PIN(2,5),GPIOF_OUT_INIT_HIGH,"PE0_EN");
	if (ret){
		pr_err("%s: PE0_EN gpio request failed: %d\n",__func__, ret);
		goto request_pe0_en_failed;
	}
	ret = gpio_export(GPIO_TO_PIN(2,5),1);
	if (ret){
		pr_err("%s: PE0_EN gpio export failed: %d\n",__func__, ret);
		goto export_pe0_en_failed;
	}

	ret = davinci_cfg_reg_list(da850_trik_ehrpwm1_pins);
	if (ret){
		pr_err("%s: ehrpwm1 pins mux setup failed: %d\n", __func__, ret);
		goto cfg_reg_ehrpwm1_failed;
	}
	mask |= BIT(2) | BIT(3);
	ret = gpio_request_one(GPIO_TO_PIN(2,3),GPIOF_OUT_INIT_HIGH,"PE1_EN");
	if (ret){
		pr_err("%s: PE1_EN gpio request failed: %d\n",__func__, ret);
		goto request_pe1_en_failed;
	}
	ret = gpio_export(GPIO_TO_PIN(2,3),1);
	if (ret){
		pr_err("%s: PE1_EN gpio export failed: %d\n",__func__, ret);
		goto export_pe1_en_failed;
	}
	da850_register_ehrpwm(mask);
	return 0;
export_pe1_en_failed:
	gpio_free(GPIO_TO_PIN(2,3));
request_pe1_en_failed:
cfg_reg_ehrpwm1_failed:
export_pe0_en_failed:
	gpio_free(GPIO_TO_PIN(2,5));
request_pe0_en_failed:
	return ret;
}

static int ecap = 1; // '1' - PWM; '0' - ECAp

EXPORT_SYMBOL (ecap);
static int __init set_ecap(char *str)
{
	if (!strcasecmp(str,"pwm"))
		ecap = 1;
	else if (!strcasecmp(str,"ecap"))
		ecap = 0;
	else 
		return 1;
	return 0;
}

__setup("trik.jcx=", set_ecap);

static const short da850_trik_cap_pins[] __initconst = {
	DA850_ECAP0_APWM0,
	DA850_ECAP1_APWM1,
	DA850_ECAP2_APWM2,
	DA850_GPIO2_4/*PC_EN*/,
	-1
};
static __init int da850_trik_cap_apwm_init(void){
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_cap_pins);
	if (ret){
		pr_err("%s: cap pins mux setup failed: %d\n", __func__, ret);
		goto cfg_reg_cap_failed;
	}
	ret =  gpio_request_one(GPIO_TO_PIN(2,4),GPIOF_OUT_INIT_HIGH,"PC_EN");
	if (ret){
		pr_err("%s: PC_EN gpio request failed: %d\n",__func__, ret);
		goto request_pc_en_failed;
	}
	ret = gpio_export(GPIO_TO_PIN(2,4),1);
	if (ret){
		pr_err("%s: PC_EN gpio export failed: %d\n",__func__, ret);
		goto export_pc_en_failed;
	}
	if (ecap){
		ret = da850_register_ecap(0);
		if (ret){
			pr_warning("%s: register ecap.pwm 0 failed: %d\n",__func__, ret);
		}
		ret = da850_register_ecap(1);
		if (ret){
			pr_warning("%s: register ecap.pwm 1 failed: %d\n",__func__, ret);
		}
		ret = da850_register_ecap(2);
		if (ret){
			pr_warning("%s: register ecap.pwm 2 failed: %d\n",__func__, ret);
		}
	}
	else
	{
		ret = da850_register_ecap_cap(0);
		if (ret){
			pr_warning("%s: register ecap.cpwm 0 failed: %d\n",__func__, ret);
		}
		ret = da850_register_ecap_cap(1);
		if (ret){
			pr_warning("%s: register ecap.cpwm 1 failed: %d\n",__func__, ret);
		}
		ret = da850_register_ecap_cap(2);
		if (ret){
			pr_warning("%s: register ecap.cpwm 2 failed: %d\n",__func__, ret);
		}
	}
	return 0;
export_pc_en_failed:
	gpio_free(GPIO_TO_PIN(2,4));
request_pc_en_failed:
cfg_reg_cap_failed:
	return ret;
}
static const short da850_trik_pwr_con_pins[] __initconst = {
	DA850_GPIO5_14,
	-1
};
static __init int da850_trik_pwr_con_init(void){
	int ret;
	ret = davinci_cfg_reg_list(da850_trik_pwr_con_pins);
	if (ret){
		pr_err("%s: power connection pins mux setup failed: %d\n", __func__, ret);
		goto cfg_reg_pwr_con_failed;
	}
	ret =  gpio_request_one(GPIO_TO_PIN(5,14),GPIOF_OUT_INIT_HIGH,"POWER_CON");
	if (ret){
		pr_err("%s: POWER_CON gpio request failed: %d\n",__func__, ret);
		goto request_pwr_con_failed;
	}
	ret = gpio_export(GPIO_TO_PIN(5,14),1);
	if (ret){
		pr_err("%s: POWER_CON gpio export failed: %d\n",__func__, ret);
		goto export_pwr_con_failed;
	}
	return 0;
export_pwr_con_failed:
	gpio_free(GPIO_TO_PIN(5,14));
request_pwr_con_failed:
cfg_reg_pwr_con_failed:
	return ret;
}	
static ssize_t trik_sensor_d1_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	//DA850_GPIO3_3,	/*D1A*/
	//DA850_GPIO3_2,	/*D1B*/
	struct timespec start, end;
	gpio_direction_output(GPIO_TO_PIN(3, 3),1);
	udelay(1000);
	gpio_direction_output(GPIO_TO_PIN(3, 3),0);
	getnstimeofday(&start);
	do {
		getnstimeofday(&end);
	} while(gpio_get_value(GPIO_TO_PIN(3, 2)));

	return sprintf(buf,"%lu\n", timespec_sub(end,start).tv_nsec);
}
static ssize_t trik_sensor_d2_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	//DA850_GPIO3_5,	/*D2B*/
	//DA850_GPIO3_1,	/*D2A*/
	struct timespec start, end;
	gpio_direction_output(GPIO_TO_PIN(3, 1),1);
        udelay(1000);
        gpio_direction_output(GPIO_TO_PIN(3, 1),0);
        getnstimeofday(&start);
        do {
		getnstimeofday(&end);
        } while(gpio_get_value(GPIO_TO_PIN(3, 5)));

	return sprintf(buf,"%lu\n", timespec_sub(end,start).tv_nsec);
}

static const DEVICE_ATTR(sensor_d1,	(S_IRUSR|S_IRGRP|S_IROTH), trik_sensor_d1_read, NULL);
static const DEVICE_ATTR(sensor_d2,	(S_IRUSR|S_IRGRP|S_IROTH), trik_sensor_d2_read,   NULL);

static const struct attribute *da850_trik_manage_attrs[] = {
	&dev_attr_sensor_d1.attr,
	&dev_attr_sensor_d2.attr,
    NULL,
};


static const struct attribute_group da850_trik_manage_attrs_group = {
	.attrs = (struct attribute **) da850_trik_manage_attrs,
};
static int __devinit da850_trik_manage_probe(struct platform_device *pdev)
{
	int ret;
	ret = sysfs_create_group(&pdev->dev.kobj, &da850_trik_manage_attrs_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to export da850_trik state, "
			            "error: %d\n", ret);
		platform_set_drvdata(pdev, NULL);
		return ret;
	}
	trik_sensor_init();
	return 0;
}

static int __devexit da850_trik_manage_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &da850_trik_manage_attrs_group);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver da850_trik_manage_driver = {
	.probe		= da850_trik_manage_probe,
	.remove		= __devexit_p(da850_trik_manage_remove),
	.driver		= {
		.name	= "da850_trik",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device da850_trik_manage_device = {
	.name		= "da850_trik",
	.id		= -1,
	.num_resources	= 0,
};

module_platform_driver(da850_trik_manage_driver);


static __init int da850_bwsensor_init(void){
	return platform_device_register(&da850_trik_manage_device);
};

static __init void da850_trik_init(void)
{
	int ret;
	
	ret = da850_trik_serial_init();
	if (ret)
		pr_warning("%s: serial initialized failed: %d\n", __func__, ret);

	ret = da850_register_edma(NULL);
	if (ret)
		pr_warning("%s: EDMA registration failed: %d\n", __func__, ret);

	ret = da8xx_register_watchdog();
	if (ret)
		pr_warning("%s: watchdog registration failed: %d\n", __func__, ret);

	ret = da8xx_register_rtc();
	if (ret)
		pr_warning("%s: rtc setup failed: %d\n", __func__, ret);

	ret = da850_trik_init_cpufreq();
	if (ret)
		pr_warning("%s: cpu frequency init failed: %d\n", __func__, ret);

	ret = da850_trik_init_cpuidle();
	if (ret)
		pr_warning("%s: cpuidle mode init failed: %d\n", __func__, ret);

	ret = da850_trik_init_cpususpend();
	if (ret)
		pr_warning("%s: suspend mode init failed: %d\n", __func__, ret);
	ret = da850_trik_sd0_init();
	if (ret)
		pr_warning("%s: sd0 interface init failed: %d\n", __func__, ret);
	ret = da850_trik_buffer_clk_init();
	if  (ret)
		pr_warning("%s: buffer clk init failed: %d\n", __func__, ret);

	ret = da850_trik_wifi_init();
	if (ret)
		pr_warning("%s: wifi interface init failed: %d\n", __func__, ret);

	ret = da850_trik_bluetooth_init();
	if (ret)
		pr_warning("%s: bluetooth interface init failed: %d\n", __func__, ret);

	ret = da850_trik_usb_init();
	if (ret)
		pr_warning("%s: usb interface init failed: %d\n", __func__, ret);

	ret = da850_trik_msp430_init();
	if (ret)
		pr_warning("%s: msp430 init failed: %d\n", __func__, ret);

	ret = da850_trik_i2c0_init();
	if (ret)
		pr_warning("%s: i2c0 bus init failed: %d\n", __func__, ret);
	ret = da850_trik_i2c1_init();
	if (ret)
		pr_warning("%s: i2c1 bus init failed: %d\n", __func__, ret);

	ret = da850_trik_audio_init();
        if (ret)
                pr_warning("%s: audio init failed: %d\n", __func__, ret);

	ret = da850_trik_spi0_init();
	if (ret)
		pr_warning("%s: spi0 bus init failed: %d\n", __func__, ret);


	ret = da850_trik_spi1_init();
	if (ret)
		pr_warning("%s: spi1 bus init failed: %d\n", __func__, ret);

	ret = da850_trik_lcd_init();
	if (ret){
		pr_warning("%s: lcd init failed: %d\n", __func__, ret);
	}
	ret = da850_trik_led_init();
	if (ret){
		pr_warning("%s: led init failed: %d\n", __func__, ret);
	}
	ret = da850_trik_keys_init();
	if (ret){
		pr_warning("%s: keys init failed: %d\n", __func__, ret);
	}

	ret = da850_trik_gpio_extra_init();
	if (ret){
		pr_warning("%s: gpio_extra init failed: %d\n", __func__, ret);
	}
	
	ret = da850_trik_cap_apwm_init();
	if (ret){
		pr_warning("%s: cap apwm init failed: %d\n", __func__, ret);
	}
	ret = da850_trik_ehrpwm_init();
	if (ret){
		pr_warning("%s: ehrpwm init failed: %d\n", __func__, ret);
	}
	ret = da850_trik_pwr_con_init();
	if (ret){
		pr_warning("%s: power connections init failed: %d\n", __func__, ret);	
	}

	//pwm
	
	//vpif 
	if (!jd1_jd2){
		ret = da850_bwsensor_init();
		if (ret){
			pr_warning("%s: power connections init failed: %d\n", __func__, ret);	
		}
	}
}
#ifdef CONFIG_SERIAL_8250_CONSOLE
static __init int da850_trik_console_init(void)
{
	if (!machine_is_davinci_da850_trik())
		return 0;
	return add_preferred_console("ttyS", 1, "115200");
}
console_initcall(da850_trik_console_init);
#endif

static void __init da850_trik_map_io(void)
{
	da850_init();
}
MACHINE_START(DAVINCI_DA850_TRIK, "DA850/AM18x/OMAP-L138 Trikboard")
	.atag_offset		= 0x100,
	.map_io			= da850_trik_map_io,
	.init_irq		= cp_intc_init,
	.timer			= &davinci_timer,
	.init_machine		= da850_trik_init,
	.init_late		= davinci_init_late,
	.dma_zone_size		= SZ_128M,
	.restart		= da8xx_restart,
MACHINE_END
