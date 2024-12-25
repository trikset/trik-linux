/*
 * eCAP driver for PWM Capture
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/kdev_t.h>
#include <linux/pwm/ecap_cap.h>

/* eCAP register offsets */
#define ECEINT					0x2C
#define ECFLG					0x2E
#define ECCLR					0x30

/* Interrupt enable bits */
#define CAP1_INT				0x2
#define CAP2_INT				0x4
#define CAP3_INT				0x8
#define CAP4_INT				0x10
#define OVF_INT					0x20
#define INT_FLAG_CLR				0x3F

/* Generic */
#define EC_WRAP4				0x3
#define EC_RUN					0x1
#define EC_SYNCO_DIS				0x2

#define EC_DISABLE				0x0
#define EC_ENABLE				0x1
#define TIMEOUT					10
#define TOTAL_EVENTS				4

static DEFINE_MUTEX(ecap_mutex);

static unsigned int ecap_val[4];

static inline unsigned int ecap_read_long(void __iomem *base, int offset)
{
	return __raw_readl(base + offset);
}
static inline unsigned short ecap_read_short(void __iomem *base, int offset)
{
	return __raw_readw(base + offset);
}
static inline void ecap_write_short(void __iomem *base, int offset, short val)
{
	__raw_writew(val, base + offset);
}

/**
 * Capture event polarity select
 * @base	: base address of the ecap module
 * @polarity=0	: Capture events 1, 2, 3 and 4 triggers on rising edge
 * @polarity=1	: Capture events 1 and 3 triggers on falling edge,
			 event 0 and 2 on rising edge
 */
static void ecap_cap_polarity(void __iomem *base, short polarity)
{
	ecap_write_short(base, ECCTL1,
		(ecap_read_short(base, ECCTL1) & ~BIT(0)) | EC_RISING);
	ecap_write_short(base, ECCTL1,
		(ecap_read_short(base, ECCTL1) & ~BIT(4)) | (EC_RISING<<4));
	ecap_write_short(base, ECCTL1,
		(ecap_read_short(base, ECCTL1) & ~BIT(2)) | (polarity<<2));
	ecap_write_short(base, ECCTL1,
		(ecap_read_short(base, ECCTL1) & ~BIT(6)) | (polarity<<6));
}

/**
 * Capture mode selection
 * @base	: base address of the ecap module
 * @mode=0	: Absolute mode
 * @mode=1	: Delta mode
*/
static void ecap_cap_mode(void __iomem *base, short mode)
{
	int i;

	/* Bit 1, 3, 5 and 7 represents the
	 * aboslute and delta mode in the control reg 1 */
	for (i = 1; i <= 7; i += 2)
		ecap_write_short(base, ECCTL1,
			(ecap_read_short(base, ECCTL1) & ~BIT(i)) |
			((mode & 0x1)<<i));
}

/**
 * Enable loading of cap1-4 register on capture event
 * @base	: base address of the ecap module
*/
static inline void ecap_enable(void __iomem *base)
{
	ecap_write_short(base, ECCTL1,
		ecap_read_short(base, ECCTL1) | BIT(8));
}

/**
 * Selects continuous or one shot mode control
 * @base	: base address of the ecap module
 * @val=0	: Operates in continuous mode
 * @val=1	: Operates in one-shot mode
  */
static inline void ecap_op_mode(void __iomem *base, short val)
{
	ecap_write_short(base, ECCTL2,
		(ecap_read_short(base, ECCTL2) & ~BIT(0)) | (val & 0x1));
}

/**
 * ecap_prescale - Prescales the capture signals
 * @base          : base address of ecap modules
 * @prescale      : Divides the capture signal by this value
 */
static inline void ecap_prescale(void __iomem *base, int prescale)
{
	ecap_write_short(base, ECCTL1,
			(ecap_read_short(base, ECCTL1) & 0x1FF)
			| (prescale << 9));
}

/**
 * Enables the specific interrupt of the ecap module
 */
static inline void ecap_int_enable(void __iomem *base, short flag)
{
	ecap_write_short(base, ECCLR, INT_FLAG_CLR);
	ecap_write_short(base, ECEINT, flag);
}

/**
 * Starts time stamp counter
 */
static inline void ecap_free_run(void __iomem *base)
{
	ecap_write_short(base, ECCTL2,
		ecap_read_short(base, ECCTL2) | (EC_RUN << 4));
}

/**
 * Stops times stamp counter
 */
static inline void ecap_freeze_counter(void __iomem *base)
{
	ecap_write_short(base, ECCTL2,
	ecap_read_short(base, ECCTL2) & ~(EC_RUN << 4));
}

/**
 * Rearm the capture register load in
 * single shot mode
 */
static inline void ecap_rearm(void __iomem *base)
{
	ecap_write_short(base, ECCTL2,
		ecap_read_short(base, ECCTL2) | BIT(3));
}

/**
 * Initializes and configures the ecap module
 */
int ecap_cap_config(struct pwm_device *p)
{
	struct ecap_pwm *ecap = to_ecap_pwm(p);
	struct ecap_cap ecap_cap = ecap->ecap_cap;

	if (ecap_cap.edge > EC_RISING_FALLING ||
			ecap_cap.prescale > 0x1F)
		return -EINVAL;
	mutex_lock(&ecap_mutex);
	clk_enable(ecap->clk);
	ecap_cap_polarity(ecap->mmio_base, EC_RISING);
	ecap_cap_mode(ecap->mmio_base, EC_ABS_MODE);
	ecap_enable(ecap->mmio_base);
	ecap_prescale(ecap->mmio_base, ecap_cap.prescale);

	/* Selects capture as an operating mode */
	ecap_write_short(ecap->mmio_base, ECCTL2,
			(ecap_read_short(ecap->mmio_base, ECCTL2)
						& ~BIT(9)));
	/* configures to one shot mode */
	ecap_op_mode(ecap->mmio_base, EC_ONESHOT);
	/* Disables the sync out */
	ecap_write_short(ecap->mmio_base, ECCTL2,
			ecap_read_short(ecap->mmio_base, ECCTL2)
						| (EC_SYNCO_DIS << 6));
	/* Disables the sync in */
	ecap_write_short(ecap->mmio_base, ECCTL2,
			  ecap_read_short(ecap->mmio_base, ECCTL2)
						| (EC_DISABLE << 6));
	/* Wrap after capture event 4 in continuous mode,
	 * stops after capture event 4 in single shot mode
	 */
	ecap_write_short(ecap->mmio_base, ECCTL2,
			ecap_read_short(ecap->mmio_base, ECCTL2)
						| (EC_WRAP4 << 1));
	clk_disable(ecap->clk);
	mutex_unlock(&ecap_mutex);
	return 0;
}

irqreturn_t ecap_davinci_isr(int this_irq, void *dev_id)
{
	struct ecap_pwm *ecap = dev_id;

	ecap_val[0] = ecap_read_long(ecap->mmio_base, CAP1);
	ecap_val[1] = ecap_read_long(ecap->mmio_base, CAP2);
	ecap_val[2] = ecap_read_long(ecap->mmio_base, CAP3);
	ecap_val[3] = ecap_read_long(ecap->mmio_base, CAP4);

	complete(&ecap->comp);
	ecap_write_short(ecap->mmio_base, ECCLR, INT_FLAG_CLR);
	return IRQ_HANDLED;
}

static int get_cap_value(struct ecap_pwm *ecap, short polarity)
{
	int r = 0;
	
	clk_enable(ecap->clk);
	ecap_cap_polarity(ecap->mmio_base,
					polarity);
	ecap_free_run(ecap->mmio_base);
	init_completion(&ecap->comp);
	ecap_rearm(ecap->mmio_base);
	ecap_int_enable(ecap->mmio_base, CAP4_INT);
	pr_err("ECTRL1 = 0x%x, ECTRL2 = 0x%x\n", 
		ecap_read_short(ecap->mmio_base, ECCTL1),
		ecap_read_short(ecap->mmio_base, ECCTL2));
	/* Waits for interrupt to occur */
	r = wait_for_completion_interruptible_timeout(&ecap->comp,
							TIMEOUT*HZ);
	
	ecap_freeze_counter(ecap->mmio_base);
	clk_disable(ecap->clk);
 
	return r;
}

static unsigned int get_freq(unsigned int diff, int prescale, unsigned long sys_freq)
{
	unsigned int freq;

	if (diff == 0)
		return 0;
	return freq = sys_freq / (diff / prescale);
}

ssize_t prescale_show(struct pwm_device *p, char *buf)
{
  struct ecap_pwm *ecap = to_ecap_pwm(p);

  return sprintf(buf, "%d\n",
			ecap->ecap_cap.prescale);
}

ssize_t prescale_store(struct pwm_device *p, const char *buf, size_t len)
{
	unsigned long result;
	struct ecap_pwm *ecap = to_ecap_pwm(p);

  if (!kstrtoul(buf, 10, &result)) {
		ecap_prescale(ecap->mmio_base, result);
	    ecap->ecap_cap.prescale = result;
	}

	return len;
}

ssize_t period_freq_show(struct pwm_device *p, char *buf)
{
	struct ecap_pwm *ecap = to_ecap_pwm(p);
	unsigned int freq1, freq2, freq3;
	int prescale_val, ret;

	mutex_lock(&ecap_mutex);
	prescale_val = ecap->ecap_cap.prescale;

	if (!prescale_val)
		prescale_val = 1;
	else
		/*
		 * Divides the input pwm with the prescaler
		 * value multiplied by 2
		 */
		prescale_val *= 2;

	ret = get_cap_value(ecap,EC_RISING);
	mutex_unlock(&ecap_mutex);
	if (!ret)
		return sprintf(buf, "-1\n");

	freq1 = get_freq((ecap_val[1] - ecap_val[0]), prescale_val, p->tick_hz);
	freq2 = get_freq((ecap_val[2] - ecap_val[1]), prescale_val, p->tick_hz);
	freq3 = get_freq((ecap_val[3] - ecap_val[2]), prescale_val, p->tick_hz);

	return sprintf(buf, "%d,%d,%d\n", freq1, freq2, freq3);
}
unsigned long ticks_to_ns(unsigned long ticks, unsigned long sys_freq)
{
	unsigned long long ns;

	ns = ticks;
	ns *= 1000000000UL;
	do_div(ns, sys_freq);
	return ns;
}
ssize_t period_ns_show(struct pwm_device *p, char *buf)
{
	struct ecap_pwm *ecap = to_ecap_pwm(p);
	int prescale_val, ret;

	mutex_lock(&ecap_mutex);
	prescale_val = ecap->ecap_cap.prescale;

	if (!prescale_val)
		prescale_val = 1;
	else
		/*
		 * Divides the input pwm with the prescaler
		 * value multiplied by 2
		 */
		prescale_val *= 2;
	ret = get_cap_value(ecap,EC_RISING);
	mutex_unlock(&ecap_mutex);
	if (!ret)
		return sprintf(buf, "-1\n");

	return sprintf(buf, "%lu,%lu,%lu\n", 
							ticks_to_ns((ecap_val[1] - ecap_val[0])/prescale_val, p->tick_hz), 
							ticks_to_ns((ecap_val[2] - ecap_val[1])/prescale_val, p->tick_hz), 
							ticks_to_ns((ecap_val[3] - ecap_val[2])/prescale_val, p->tick_hz));

}
ssize_t duty_ns_show(struct pwm_device *p, char *buf){
		int ret;
	unsigned long diff1, diff2;

  struct ecap_pwm *ecap = to_ecap_pwm(p);

	mutex_lock(&ecap_mutex);
	/* Setting ecap register to calculate duty cycle */
	if (ecap->ecap_cap.prescale > 0)
		ecap_prescale(ecap->mmio_base, 0);

	ret = get_cap_value(ecap,EC_RISING_FALLING);

	/* Resetting it back*/
	ecap_cap_polarity(ecap->mmio_base, EC_RISING);
	if (ecap->ecap_cap.prescale)
		ecap_prescale(ecap->mmio_base,
				 ecap->ecap_cap.prescale);
	mutex_unlock(&ecap_mutex);
	if (!ret)
		return sprintf(buf, "-1\n");
	diff1 = ecap_val[1] - ecap_val[0];
	diff2 = ecap_val[2] - ecap_val[0];
	

	return sprintf(buf, "%lu\n", ticks_to_ns(diff1, p->tick_hz));
}
ssize_t duty_percent_show(struct pwm_device *p, char *buf)
{
	int ret;
	unsigned int diff1, diff2;
	
	struct ecap_pwm *ecap = to_ecap_pwm(p);

	mutex_lock(&ecap_mutex);
	/* Setting ecap register to calculate duty cycle */
	if (ecap->ecap_cap.prescale > 0)
		ecap_prescale(ecap->mmio_base, 0);

	ret = get_cap_value(ecap,EC_RISING_FALLING);

	/* Resetting it back*/
	ecap_cap_polarity(ecap->mmio_base, EC_RISING);
	if (ecap->ecap_cap.prescale)
		ecap_prescale(ecap->mmio_base,
				 ecap->ecap_cap.prescale);
	mutex_unlock(&ecap_mutex);
	if (!ret)
		return sprintf(buf, "-1\n");
	diff1 = ecap_val[1] - ecap_val[0];
	diff2 = ecap_val[2] - ecap_val[0];
	return sprintf(buf, "%d\n", (diff1 * 100)/diff2);
}

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Driver for Davinci eCAP peripheral");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ecap");