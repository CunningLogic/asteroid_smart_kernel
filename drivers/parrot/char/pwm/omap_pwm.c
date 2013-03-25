/**
********************************************************************************
* @file omap_pwm.c
* @brief kernel driver for omap pwm
*
* Copyright (C) 2010 Parrot S.A.
*
* @author     François MULLER <francois.muller@parrot.com>
* @date       8-Sept-2010
********************************************************************************
*/

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <linux/err.h>
#include <linux/clk.h>

#include <mach/hardware.h>
#include <plat/dmtimer.h>

#include "pwm_ops.h"
#include "pwm_ioctl.h"

#define PWM_NB_TIMER 12
#define MAX_UINT32	0xFFFFFFFF

struct s_omap_pwm
{
	struct omap_dm_timer *timer;
	uint8_t pwm_active;
	uint32_t ratio; // between 0 and PWM_WIDTH_MAX
	uint32_t freq; // in Hz
};

static struct s_omap_pwm omap_pwm[PWM_NB_TIMER];
static uint32_t pwm_precision_mask = MAX_UINT32;


static bool omap_pwm_is_active(uint32_t n)
{
	return n < PWM_NB_TIMER ? (omap_pwm[n].pwm_active != 0) : false;
}

static inline int omap_pwm_set_active(uint32_t n, bool active)
{
	int ret = 0;

	if (active)
	{
		omap_pwm[n].timer = omap_dm_timer_request_specific(n);
		if (omap_pwm[n].timer == NULL) {
			printk(KERN_ERR "failed to request pwm timer %i\n", (int)n);
			ret = -ENODEV;
		}
		else
		{
			omap_dm_timer_set_pwm(omap_pwm[n].timer, false, true, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
			omap_dm_timer_set_load(omap_pwm[n].timer, true, pwm_precision_mask);
			omap_dm_timer_set_match(omap_pwm[n].timer, true, pwm_precision_mask);
		    omap_dm_timer_set_prescaler(omap_pwm[n].timer, 0);

			omap_pwm[n].pwm_active = active;
		}
	}
	else
	{
		if (omap_pwm[n].timer)
			omap_dm_timer_free(omap_pwm[n].timer);
		omap_pwm[n].pwm_active = active;
	}

	return ret;
}

static unsigned int compute_ratio(unsigned int ratio)
{
	unsigned int val;

	if (ratio > PWM_WIDTH_MAX)
		return -EINVAL;
	/* XXX overflow happen if length > 16 */

	// Cross-multiplication in 64 bits needed (26 000 000 * 10 000) MAX
	// div_u64 used since there is a bug in GCC
	val = div_u64(((unsigned long long int)ratio * (MAX_UINT32 - pwm_precision_mask) + PWM_WIDTH_MAX/2), PWM_WIDTH_MAX);

	if (val == MAX_UINT32 - pwm_precision_mask && val != 0)
		val = MAX_UINT32 - pwm_precision_mask - 1;

	return val;
}

/**
 * reserve a pwm. Note that it is to the caller to make sure, there is no
 * concurent access
 *
 * @param timer : timer number
 * @return 0 or an error code
 */
static int omap_pwm_request(unsigned int timer)
{
	if (timer >= PWM_NB_TIMER)
		return -EINVAL;

	if (omap_pwm_is_active(timer))
		return -EBUSY;

	omap_pwm_set_active(timer, true);

	return 0;
}

/**
 * release a pwm. Note that it is to the caller to make sure, there is no
 * concurent access
 *
 * @param timer : timer number
 * @return 0 or an error code
 */
static int omap_pwm_release(unsigned int timer)
{
	if (timer >= PWM_NB_TIMER)
		return -EINVAL;

	if (!omap_pwm_is_active(timer))
		return -EBUSY;

	omap_pwm_set_active(timer, false);

	return 0;
}

/**
 * start a timer
 *
 * @param timer the timer to use
 *
 * @return error code
 */
static int omap_pwm_start(unsigned int timer)
{
	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	omap_dm_timer_enable(omap_pwm[timer].timer);
	omap_dm_timer_start(omap_pwm[timer].timer);
	omap_dm_timer_trigger(omap_pwm[timer].timer);

	return 0;
}

/** stop a timer
 *
 * @param timer
 * @return error code
 */
static int omap_pwm_stop(unsigned int timer)
{
	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	omap_dm_timer_stop(omap_pwm[timer].timer);
	omap_dm_timer_disable(omap_pwm[timer].timer);

	return 0;
}

/* ratio = 0 ... 100 00 */
/**
 * configure the duty cycle
 *
 * @param timer
 * @param ratio (percentage * 100 of the dc)
 *
 * @return error code
 */
static int omap_pwm_set_width(unsigned int timer, unsigned int ratio)
{
	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	if (ratio > PWM_WIDTH_MAX)
		return -EINVAL;

	omap_pwm[timer].ratio = ratio; // store user ratio
	omap_dm_timer_set_match(omap_pwm[timer].timer, true, pwm_precision_mask + compute_ratio(ratio)); // apply ratio

	return 0;
}

static int omap_pwm_get_width(unsigned int timer, unsigned int *ratio)
{
	unsigned int current_ratio;

	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	// use a double Cross-multiplication to obtain the current value
	current_ratio = div_u64(((unsigned long long int)compute_ratio(omap_pwm[timer].ratio) * PWM_WIDTH_MAX), (MAX_UINT32 - pwm_precision_mask));

	*ratio = current_ratio;

	return 0;
}

/** configure a timer freq
 *
 * if the timer if not stopped, the value will taken in account a the next
 * reload (if autoreload is on)
 *
 * @param timer
 * @param freq in HZ
 *
 * @return error code
 */
static int omap_pwm_set_freq(unsigned int timer, unsigned int freq)
{
	uint32_t clk_speed, pwm_ratio, pwm_speed;
	unsigned int ratio;

	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	omap_dm_timer_set_source(omap_pwm[timer].timer, OMAP_TIMER_SRC_SYS_CLK); // force sys_clk, other clock are 32khz or external
	clk_speed = clk_get_rate(omap_dm_timer_get_fclk(omap_pwm[timer].timer)); // get sys_clk speed (26Mhz during test)

	if (freq >= (clk_speed/2) || freq == 0)
		return -EINVAL;

	// get the divider needed to reach the desired frequency (this is also the pwm ratio precision)
	pwm_ratio = (clk_speed/2) / freq;
	pwm_speed = (clk_speed/2) / pwm_ratio;  // compute the obtained frequency
	omap_pwm[timer].freq = pwm_speed;		// store it for the user

	pwm_precision_mask = MAX_UINT32 - pwm_ratio;	// change the pwm mask
	omap_dm_timer_set_load(omap_pwm[timer].timer, true, pwm_precision_mask); // and store it in the register

	// since we changed frequency, we must recompute the ratio
	omap_pwm_get_width(timer, &ratio);
	omap_pwm_set_width(timer, ratio);

	return 0;
}

static int omap_pwm_get_freq(unsigned int timer, unsigned int *freq)
{
	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	*freq = omap_pwm[timer].freq;

	return 0;
}

struct pwm_ops omap_pwm_ops =
{
	.pwm_max = PWM_NB_TIMER-1,
	.pwm_start = omap_pwm_start,
	.pwm_stop = omap_pwm_stop,
	.pwm_request = omap_pwm_request,
	.pwm_release = omap_pwm_release,
	.pwm_set_width = omap_pwm_set_width,
	.pwm_set_freq = omap_pwm_set_freq,
	.pwm_get_width = omap_pwm_get_width,
	.pwm_get_freq = omap_pwm_get_freq,
	.owner = THIS_MODULE,
};

static int __devinit omap_pwm_init(void)
{
	int i;

	for(i=0; i< PWM_NB_TIMER; i++)
	{
		omap_pwm[i].freq = 0;
		omap_pwm[i].ratio = 0;
		omap_pwm[i].pwm_active = 0;
	}

	return register_pwm(&omap_pwm_ops);
}

static void __exit omap_pwm_exit(void)
{
	unregister_pwm(&omap_pwm_ops);
}

module_init(omap_pwm_init);
module_exit(omap_pwm_exit);

MODULE_AUTHOR("PARROT SA");
MODULE_DESCRIPTION("omap pwm driver");
MODULE_LICENSE("GPL");
