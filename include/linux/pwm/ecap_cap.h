/*
 * This program is free software; you may redistribute and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#ifndef __LINUX_CAPTURE_H
#define __LINUX_CAPTURE_H

#define EC_RISING				0x0
#define EC_RISING_FALLING			0x1

#define EC_ABS_MODE				0x0
#define EC_DELTA_MODE				0x1

#define EC_ONESHOT				0x1
#include <linux/irqreturn.h>
#include <linux/pwm/pwm.h>

int ecap_cap_config(struct pwm_device *p);
irqreturn_t ecap_davinci_isr(int this_irq, void *dev_id);

ssize_t duty_ns_show(struct pwm_device *p, char *buf);
ssize_t duty_percent_show(struct pwm_device *p, char *buf);

ssize_t period_ns_show(struct pwm_device *p, char *buf);
ssize_t period_freq_show(struct pwm_device *p, char *buf);

ssize_t prescale_show(struct pwm_device *p, char *buf);
ssize_t prescale_store(struct pwm_device *p, const char *buf, size_t len);
#endif
