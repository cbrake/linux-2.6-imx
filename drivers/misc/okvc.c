/*
 * okvc.c
 *
 * OKI Valve Controller IO Driver
 *
 * Copyright (C) 2017 Cliff Brake <cbrake@bec-systems.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/rotary_encoder.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/hrtimer.h>

#define DRV_NAME "okvc-misc"

#define ENCODER_1_A 136
#define ENCODER_1_B 135
#define ENCODER_2_A 27
#define ENCODER_2_B 26
#define DIR_1 117
//#define IN_A 117
#define DIR_2 42
#define SLEEP_1 113
#define SLEEP_2 12
#define MODE_1_1 119
//#define IN_B 119
#define MODE_1_2 120
#define MODE_2_1 46
#define MODE_2_2 10
#define TXS1 129
#define TXS2 11
#define TXS_OE 18

extern int vf610_read_raw_internal(int channel);

enum okvc_state
{
	OKVC_IDLE,

	/* following state is for non-encoder motor operation */
	OKVC_MOTOR_FORWARD,
	OKVC_MOTOR_BACKWARD,

	/* following states are for encoder motor operation */
	OKVC_ENC_FORWARD,
	OKVC_ENC_BACKWARD,

	/* following states are for spray valve driver */
	OKVC_SPRAY_PRE,
	OKVC_SPRAY_SPRAY,
	OKVC_SPRAY_POST,

	/* follwoing states are for precision valve */
	OKVC_PREC_VALVE_ADC,
	OKVC_PREC_VALVE_TKS,
};

struct okvc
{
	int pos;
	int irq;

	/* variables for non-encoder motor operation */
	int forward_ms;
	int backward_ms;

	/* variables for encoder motor operation */
	int forward_cnt;
	int backward_cnt;
	int pwm_duty_percentx10;
	int reverse_encoder;

	/* variables for spray operation */
	int spray_pre_ms;
	int spray_ms;
	int spray_post_ms;

	/* variables for prec valve operation */
	int prec_valve_ms;
	int adc_average;
	int adc_average_start_ms;

	/* variables to initiate operation */
	int run_motor;
	int run_motor_enc;
	int run_spray;
	int run_prec_valve;
	int forward;
	int backward;

	/* internal data structures */
	struct pwm_device *pwm;
	enum okvc_state state;
	struct hrtimer timer;
	struct work_struct adc_work;
};

static int pwm_period = 500000;

static int okvc_pwm_high(struct okvc *okvc_)
{
	return pwm_config(okvc_->pwm, pwm_period, pwm_period);
}

static int okvc_pwm_config(struct okvc *okvc_)
{
	int ret;
	int pwm_duty_ns;

	pwm_duty_ns = ((okvc_->pwm_duty_percentx10) * pwm_period) / 1000;
	printk("PWM duty=%dns, period=%dns\n", pwm_duty_ns, pwm_period);
	ret = pwm_config(okvc_->pwm, pwm_duty_ns, pwm_period);
	if (ret)
	{
		pr_err("OK: error setting pwm_config: %d\n", ret);
		return ret;
	}

	ret = pwm_enable(okvc_->pwm);
	if (ret)
	{
		pr_err("OK: error enabling pwm: %d\n", ret);
		return ret;
	}

	return 0;
}

enum
{
	SETUP_MOTOR_BRAKE,
	SETUP_MOTOR_CW,
	SETUP_MOTOR_CCW,
};

void setup_motor(struct okvc *okvc_, int state)
{
	switch (state)
	{
	case SETUP_MOTOR_BRAKE:
		okvc_pwm_high(okvc_);
		gpio_set_value(MODE_1_1, 1);
		gpio_set_value(DIR_1, 0);
		break;
	case SETUP_MOTOR_CW:
		okvc_pwm_config(okvc_);
		gpio_set_value(MODE_1_1, 0);
		gpio_set_value(DIR_1, 0);
		break;
	case SETUP_MOTOR_CCW:
		okvc_pwm_config(okvc_);
		gpio_set_value(MODE_1_1, 0);
		gpio_set_value(DIR_1, 1);
		break;
	}
}

/* Sysfs Attributes */
static ssize_t adc_show(struct device *cdev,
						struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 40, "%d\n", vf610_read_raw_internal(8));
}

static DEVICE_ATTR_RO(adc);

static ssize_t adc_pressure_input_show(struct device *cdev,
									   struct device_attribute *attr, char *buf)
{
	int total = 0;
	int i;

	for (i = 0; i < 8; i++)
	{
		total += vf610_read_raw_internal(0);
	}

	return snprintf(buf, 40, "%d\n", total / 8);
}

static DEVICE_ATTR_RO(adc_pressure_input);

static ssize_t pos_show(struct device *cdev,
						struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->pos);
}

static DEVICE_ATTR_RO(pos);

static ssize_t forward_cnt_show(struct device *cdev,
								struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->forward_cnt);
}

static ssize_t forward_cnt_store(struct device *cdev,
								 struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	// FIXME add bounds checking
	okvc_->forward_cnt = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR_RW(forward_cnt);

static ssize_t backward_cnt_show(struct device *cdev,
								 struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->backward_cnt);
}

static ssize_t backward_cnt_store(struct device *cdev,
								  struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	okvc_->backward_cnt = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR_RW(backward_cnt);

static ssize_t pwm_show(struct device *cdev,
						struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->pwm_duty_percentx10);
}

static ssize_t pwm_store(struct device *cdev,
						 struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	okvc_->pwm_duty_percentx10 = simple_strtoul(buf, NULL, 0);
	okvc_pwm_config(okvc_);
	return count;
}

static DEVICE_ATTR_RW(pwm);

static ssize_t run_motor_enc_show(struct device *cdev,
								  struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->run_motor_enc);
}

static ssize_t run_motor_enc_store(struct device *cdev,
								   struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	int run_ = simple_strtoul(buf, NULL, 0);

	if (run_ && (okvc_->state != OKVC_IDLE))
	{
		printk("Error, already running\n");
		return count;
	}
	else if (run_ && !okvc_->run_motor_enc)
	{
		okvc_->pos = 0;
		gpio_set_value(TXS1, 1);
		setup_motor(okvc_, SETUP_MOTOR_CW);
		okvc_->state = OKVC_ENC_FORWARD;
	}
	else if (!run_)
	{
		gpio_set_value(TXS1, 0);
		setup_motor(okvc_, SETUP_MOTOR_BRAKE);
		printk("Stopping motor\n");
		okvc_->state = OKVC_IDLE;
	}

	okvc_->run_motor_enc = run_;

	return count;
}

static DEVICE_ATTR_RW(run_motor_enc);

static ssize_t run_motor_show(struct device *cdev,
							  struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->run_motor);
}

static ssize_t run_motor_store(struct device *cdev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	int run_ = simple_strtoul(buf, NULL, 0);

	if (run_ && (okvc_->state != OKVC_IDLE))
	{
		printk("Error, already running\n");
		return count;
	}
	else if (run_ && !okvc_->run_motor)
	{
		okvc_->pos = 0;
		gpio_set_value(TXS1, 1);
		setup_motor(okvc_, SETUP_MOTOR_CW);
		hrtimer_start(&okvc_->timer, ms_to_ktime(okvc_->forward_ms), HRTIMER_MODE_REL);
		okvc_->state = OKVC_MOTOR_FORWARD;
	}
	else if (!run_)
	{
		hrtimer_cancel(&okvc_->timer);
		gpio_set_value(TXS1, 0);
		setup_motor(okvc_, SETUP_MOTOR_BRAKE);
		okvc_->state = OKVC_IDLE;
		printk("Stopping motor\n");
	}

	okvc_->run_motor = run_;

	return count;
}

static DEVICE_ATTR_RW(run_motor);

static ssize_t run_spray_show(struct device *cdev,
							  struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->run_spray);
}

static ssize_t run_spray_store(struct device *cdev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	int run_ = simple_strtoul(buf, NULL, 0);

	if (run_ && (okvc_->state != OKVC_IDLE))
	{
		printk("Error, already running\n");
		return count;
	}
	else if (run_ && !okvc_->run_spray)
	{
		gpio_set_value(TXS2, 1);
		hrtimer_start(&okvc_->timer, ms_to_ktime(okvc_->spray_pre_ms), HRTIMER_MODE_REL);
		okvc_->state = OKVC_SPRAY_PRE;
	}
	else if (!run_)
	{
		hrtimer_cancel(&okvc_->timer);
		gpio_set_value(TXS1, 0);
		gpio_set_value(TXS2, 0);
		okvc_->state = OKVC_IDLE;
		printk("Stopping spray\n");
	}

	okvc_->run_spray = run_;

	return count;
}

static DEVICE_ATTR_RW(run_spray);

static ssize_t run_prec_valve_show(struct device *cdev,
								   struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->run_prec_valve);
}

static ssize_t run_prec_valve_store(struct device *cdev,
									struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	int run_ = simple_strtoul(buf, NULL, 0);

	if (okvc_->prec_valve_ms < 10)
	{
		printk("prec_valve_ms must be greater than 10, not running\n");
		return count;
	}

	if (run_ && (okvc_->state != OKVC_IDLE))
	{
		printk("Error, already running\n");
		return count;
	}
	else if (run_ && !okvc_->run_prec_valve)
	{
		gpio_set_value(TXS1, 1);
		hrtimer_start(&okvc_->timer, ms_to_ktime(okvc_->adc_average_start_ms), HRTIMER_MODE_REL);
		okvc_->state = OKVC_PREC_VALVE_ADC;
		okvc_->adc_average = 0;
	}
	else if (!run_)
	{
		hrtimer_cancel(&okvc_->timer);
		gpio_set_value(TXS1, 0);
		okvc_->state = OKVC_IDLE;
		printk("Stopping precision valve\n");
	}

	okvc_->run_prec_valve = run_;

	return count;
}

static DEVICE_ATTR_RW(run_prec_valve);

static ssize_t reverse_encoder_show(struct device *cdev,
									struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->reverse_encoder);
}

static ssize_t reverse_encoder_store(struct device *cdev,
									 struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	okvc_->reverse_encoder = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR_RW(reverse_encoder);

static ssize_t forward_show(struct device *cdev,
							struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->forward);
}

static ssize_t forward_store(struct device *cdev,
							 struct device_attribute *attr, const char *buf, size_t count)
{
	int forward = simple_strtoul(buf, NULL, 0);

	struct okvc *okvc_ = cdev->driver_data;
	if (forward && (okvc_->state != OKVC_IDLE))
	{
		printk("Motor already running\n");
		return count;
	}
	else if (forward)
	{
		gpio_set_value(TXS1, 1);
		setup_motor(okvc_, SETUP_MOTOR_CW);
	}
	else
	{
		setup_motor(okvc_, SETUP_MOTOR_BRAKE);
	}

	okvc_->forward = forward;
	return count;
}

static DEVICE_ATTR_RW(forward);

static ssize_t backward_show(struct device *cdev,
							 struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->backward);
}

static ssize_t backward_store(struct device *cdev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	int backward = simple_strtoul(buf, NULL, 0);

	struct okvc *okvc_ = cdev->driver_data;
	if (okvc_->state != OKVC_IDLE)
	{
		printk("Driver already running\n");
		return count;
	}
	else if (backward)
	{
		setup_motor(okvc_, SETUP_MOTOR_CCW);
	}
	else
	{
		setup_motor(okvc_, SETUP_MOTOR_BRAKE);
	}

	okvc_->backward = backward;
	return count;
}

static DEVICE_ATTR_RW(backward);

static ssize_t spray_pre_ms_show(struct device *cdev,
								 struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->spray_pre_ms);
}

static ssize_t spray_pre_ms_store(struct device *cdev,
								  struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	okvc_->spray_pre_ms = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR_RW(spray_pre_ms);

static ssize_t spray_ms_show(struct device *cdev,
							 struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->spray_ms);
}

static ssize_t spray_ms_store(struct device *cdev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	okvc_->spray_ms = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR_RW(spray_ms);

static ssize_t spray_post_ms_show(struct device *cdev,
								  struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->spray_post_ms);
}

static ssize_t spray_post_ms_store(struct device *cdev,
								   struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	okvc_->spray_post_ms = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR_RW(spray_post_ms);

static ssize_t prec_valve_ms_show(struct device *cdev,
								  struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->prec_valve_ms);
}

static ssize_t prec_valve_ms_store(struct device *cdev,
								   struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	okvc_->prec_valve_ms = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR_RW(prec_valve_ms);

static ssize_t adc_average_show(struct device *cdev,
								struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->adc_average);
}

static DEVICE_ATTR_RO(adc_average);

static ssize_t adc_average_start_ms_show(struct device *cdev,
										 struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->adc_average_start_ms);
}

static ssize_t adc_average_start_ms_store(struct device *cdev,
										  struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	okvc_->adc_average_start_ms = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR_RW(adc_average_start_ms);

static ssize_t forward_ms_show(struct device *cdev,
							   struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->forward_ms);
}

static ssize_t forward_ms_store(struct device *cdev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	okvc_->forward_ms = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR_RW(forward_ms);

static ssize_t backward_ms_show(struct device *cdev,
								struct device_attribute *attr, char *buf)
{
	struct okvc *okvc_ = cdev->driver_data;
	return snprintf(buf, 40, "%d\n", okvc_->backward_ms);
}

static ssize_t backward_ms_store(struct device *cdev,
								 struct device_attribute *attr, const char *buf, size_t count)
{
	struct okvc *okvc_ = cdev->driver_data;
	okvc_->backward_ms = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR_RW(backward_ms);

static struct attribute *okvc_attrs[] = {
	&dev_attr_pos.attr,
	&dev_attr_adc.attr,
	&dev_attr_adc_pressure_input.attr,
	&dev_attr_forward_cnt.attr,
	&dev_attr_backward_cnt.attr,
	&dev_attr_pwm.attr,
	&dev_attr_run_motor.attr,
	&dev_attr_run_motor_enc.attr,
	&dev_attr_run_spray.attr,
	&dev_attr_run_prec_valve.attr,
	&dev_attr_reverse_encoder.attr,
	&dev_attr_forward.attr,
	&dev_attr_backward.attr,
	&dev_attr_spray_pre_ms.attr,
	&dev_attr_spray_ms.attr,
	&dev_attr_spray_post_ms.attr,
	&dev_attr_forward_ms.attr,
	&dev_attr_backward_ms.attr,
	&dev_attr_prec_valve_ms.attr,
	&dev_attr_adc_average.attr,
	&dev_attr_adc_average_start_ms.attr,
	NULL,
};

static const struct attribute_group okvc_attr_group = {
	.attrs = okvc_attrs,
};

static struct gpio okvc_gpios[] = {
	{ENCODER_1_A, GPIOF_IN, "ENCODER_1_A"},
	{ENCODER_1_B, GPIOF_IN, "ENCODER_1_B"},
	{SLEEP_1, GPIOF_OUT_INIT_HIGH, "SLEEP_1"},
	{DIR_1, GPIOF_OUT_INIT_LOW, "DIR_1"},
	{MODE_1_1, GPIOF_OUT_INIT_HIGH, "MODE_1_1"},
	{TXS_OE, GPIOF_OUT_INIT_HIGH, "TXS_OE"},
	{TXS1, GPIOF_OUT_INIT_LOW, "TXS1"},
	{TXS2, GPIOF_OUT_INIT_LOW, "TXS2"},
};

static irqreturn_t okvc_irq(int irq, void *dev_id)
{
	int ret;
	struct okvc *okvc_ = dev_id;

	okvc_->pos++;

	/*
	int b = !!gpio_get_value(ENCODER_1_B);
	
	if (b ^ okvc_->reverse_encoder) {
		okvc_->pos--;
	} else {
		okvc_->pos++;
	}
	*/

	switch (okvc_->state)
	{
	case OKVC_ENC_FORWARD:
		// running forward
		if (okvc_->pos > okvc_->forward_cnt)
		{
			setup_motor(okvc_, SETUP_MOTOR_BRAKE);
			gpio_set_value(TXS1, 0);
			printk("pos = %d, changing directions\n", okvc_->pos);
			okvc_->pos = 0;
			setup_motor(okvc_, SETUP_MOTOR_CCW);
			okvc_->state = OKVC_ENC_BACKWARD;
			ret = pwm_enable(okvc_->pwm);
			if (ret)
			{
				pr_err("OK: error enabling pwm: %d\n", ret);
				return ret;
			}
		}
		break;
	case OKVC_ENC_BACKWARD:
		// running backward
		if (okvc_->pos > (okvc_->backward_cnt))
		{
			setup_motor(okvc_, SETUP_MOTOR_BRAKE);
			okvc_->state = OKVC_IDLE;
			printk("pos = %d, cycle complete\n", okvc_->pos);
			okvc_->run_motor_enc = 0;
		}
		break;
	default:
		break;
	}

	return IRQ_HANDLED;
}

static void handle_adc_work(struct work_struct *work)
{
	struct okvc *okvc_ = container_of(work, struct okvc, adc_work);
	int adc_total = 0;
	int i;
	for (i = 0; i < 8; i++)
	{
		adc_total += vf610_read_raw_internal(8);
	}
	okvc_->adc_average = adc_total / 8;
	if (!gpio_get_value(TXS1))
	{
		printk("Warning, ADC did not finish before TXS1 went low\n");
	}
}

static enum hrtimer_restart okvc_timer_callback(struct hrtimer *timer)
{
	struct okvc *okvc_ = container_of(timer, struct okvc, timer);
	enum hrtimer_restart ret = HRTIMER_NORESTART;

	switch (okvc_->state)
	{
	case OKVC_MOTOR_FORWARD:
		setup_motor(okvc_, SETUP_MOTOR_CCW);
		gpio_set_value(TXS1, 0);
		hrtimer_forward_now(&okvc_->timer, ms_to_ktime(okvc_->backward_ms));
		ret = HRTIMER_RESTART;
		okvc_->state = OKVC_MOTOR_BACKWARD;
		break;
	case OKVC_MOTOR_BACKWARD:
		setup_motor(okvc_, SETUP_MOTOR_BRAKE);
		okvc_->run_motor = 0;
		okvc_->state = OKVC_IDLE;
		break;
	case OKVC_SPRAY_PRE:
		gpio_set_value(TXS1, 1);
		hrtimer_forward_now(&okvc_->timer, ms_to_ktime(okvc_->spray_ms));
		ret = HRTIMER_RESTART;
		okvc_->state = OKVC_SPRAY_SPRAY;
		break;
	case OKVC_SPRAY_SPRAY:
		gpio_set_value(TXS1, 0);
		hrtimer_forward_now(&okvc_->timer, ms_to_ktime(okvc_->spray_post_ms));
		ret = HRTIMER_RESTART;
		okvc_->state = OKVC_SPRAY_POST;
		break;
	case OKVC_SPRAY_POST:
		gpio_set_value(TXS2, 0);
		okvc_->state = OKVC_IDLE;
		okvc_->run_spray = 0;
		break;
	case OKVC_PREC_VALVE_ADC:
		schedule_work(&okvc_->adc_work);
		okvc_->state = OKVC_PREC_VALVE_TKS;
		hrtimer_forward_now(&okvc_->timer, ms_to_ktime(okvc_->prec_valve_ms -
													   okvc_->adc_average_start_ms));
		ret = HRTIMER_RESTART;
		break;
	case OKVC_PREC_VALVE_TKS:
		gpio_set_value(TXS1, 0);
		okvc_->state = OKVC_IDLE;
		okvc_->run_prec_valve = 0;
		break;
	default:
		break;
	}

	return ret;
}

static int __init okvc_probe(struct platform_device *pdev)
{
	int ret;
	struct okvc *okvc_;

	printk("okvc_probe\n");

	okvc_ = kzalloc(sizeof(struct okvc), GFP_KERNEL);
	if (!okvc_)
	{
		pr_err("OK: Error allocating device struct\n");
		return -ENOMEM;
	}

	// we fill in some default values to make testing easier
	okvc_->forward_cnt = 100;
	okvc_->backward_cnt = 50;
	okvc_->pwm_duty_percentx10 = 550;
	okvc_->spray_pre_ms = 50;
	okvc_->spray_ms = 100;
	okvc_->spray_post_ms = 75;
	okvc_->forward_ms = 33;
	okvc_->backward_ms = 66;
	okvc_->prec_valve_ms = 50;
	okvc_->adc_average_start_ms = 10;

	ret = gpio_request_array(okvc_gpios, ARRAY_SIZE(okvc_gpios));

	if (ret)
	{
		pr_err("OK: Error requesting gpios: %d\n", ret);
		gpio_free_array(okvc_gpios, ARRAY_SIZE(okvc_gpios));
		kfree(okvc_);
		return -EINVAL;
	}

	okvc_->irq = gpio_to_irq(ENCODER_1_A);

	ret = request_irq(okvc_->irq, okvc_irq,
					  IRQF_TRIGGER_FALLING, DRV_NAME, okvc_);

	if (ret)
	{
		pr_err("OK: Error requestion irq: %d\n", ret);
		gpio_free_array(okvc_gpios, ARRAY_SIZE(okvc_gpios));
		kfree(okvc_);
		return -EINVAL;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &okvc_attr_group);

	if (ret)
	{
		pr_err("OK: Error registering device groups: %d\n", ret);
		free_irq(okvc_->irq, okvc_);
		gpio_free_array(okvc_gpios, ARRAY_SIZE(okvc_gpios));
		kfree(okvc_);
		return -EINVAL;
	}

	okvc_->pwm = pwm_get(&pdev->dev, NULL);
	if (IS_ERR(okvc_->pwm))
	{
		pr_err("OK: unable to request PWM\n");
		free_irq(okvc_->irq, okvc_);
		gpio_free_array(okvc_gpios, ARRAY_SIZE(okvc_gpios));
		kfree(okvc_);
		sysfs_remove_group(&pdev->dev.kobj, &okvc_attr_group);
		return -EINVAL;
	}

	gpio_export(ENCODER_1_A, 0);
	gpio_export(ENCODER_1_B, 0);
	gpio_export(ENCODER_2_A, 0);
	gpio_export(ENCODER_2_B, 0);
	gpio_export(DIR_1, 0);
	gpio_export(MODE_1_1, 0);

	hrtimer_init(&okvc_->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	okvc_->timer.function = okvc_timer_callback;

	platform_set_drvdata(pdev, okvc_);
	INIT_WORK(&okvc_->adc_work, handle_adc_work);

	return okvc_pwm_config(okvc_);
}

static int okvc_remove(struct platform_device *pdev)
{
	struct okvc *okvc_ = platform_get_drvdata(pdev);
	printk("okvc_remove\n");

	gpio_free_array(okvc_gpios, ARRAY_SIZE(okvc_gpios));
	kfree(okvc_);
	free_irq(okvc_->irq, okvc_);
	sysfs_remove_group(&pdev->dev.kobj, &okvc_attr_group);
	pwm_free(okvc_->pwm);
	hrtimer_try_to_cancel(&okvc_->timer);

	return 0;
}

static struct of_device_id okvc_of_match[] = {
	{
		.compatible = DRV_NAME,
	},
	{}};

MODULE_DEVICE_TABLE(of, okvc_of_match);

static struct platform_driver okvc_platform_driver = {
	.remove = okvc_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = okvc_of_match,
		//.groups = okvc_groups,
	},
};

static int okvc_init(void)
{
	int ret;

	printk("okvc_init, v21\n");

	ret = platform_driver_probe(&okvc_platform_driver, okvc_probe);
	if (ret)
	{
		pr_err("%s: platform driver register failed\n", __func__);
		return ret;
	}

	return 0;
}
module_init(okvc_init);

static void __exit okvc_exit(void)
{
	printk("okvc_exit\n");
	platform_driver_unregister(&okvc_platform_driver);
}
module_exit(okvc_exit);

MODULE_AUTHOR("Cliff Brake <cbrake@bec-systems.com>");
MODULE_DESCRIPTION("OKI Valve Controller IO Driver");
MODULE_LICENSE("GPL");
