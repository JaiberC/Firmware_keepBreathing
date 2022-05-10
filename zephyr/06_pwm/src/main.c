/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based LED fade
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/pwm.h>

<<<<<<< HEAD
#define PWM_MOTOR_NODE	DT_ALIAS(pwm_valve_air) //pwm_motor1
=======
#define PWM_MOTOR_NODE	DT_ALIAS(pwm_motor)
>>>>>>> 1f43912ddb54cb9b8c1e97c42d41a5a18a25811a

#if DT_NODE_HAS_STATUS(PWM_MOTOR_NODE, okay)
#define PWM_LABEL	DT_PWMS_LABEL(PWM_MOTOR_NODE)
#define PWM_CHANNEL	DT_PWMS_CHANNEL(PWM_MOTOR_NODE)
#define PWM_FLAGS	DT_PWMS_FLAGS(PWM_MOTOR_NODE)
#else
#error "Unsupported board: pwm-motor devicetree alias is not defined"
#define PWM_LABEL	""
#define PWM_CHANNEL	0
#define PWM_FLAGS	0
#endif

<<<<<<< HEAD
#define PWM1_MOTOR_NODE	DT_ALIAS(pwm_motor) //pwm_motor1
=======
#define PWM1_MOTOR_NODE	DT_ALIAS(pwm_motor1)

>>>>>>> 1f43912ddb54cb9b8c1e97c42d41a5a18a25811a
#if DT_NODE_HAS_STATUS(PWM1_MOTOR_NODE, okay)
#define PWM1_LABEL	DT_PWMS_LABEL(PWM1_MOTOR_NODE)
#define PWM1_CHANNEL	DT_PWMS_CHANNEL(PWM1_MOTOR_NODE)
#define PWM1_FLAGS	DT_PWMS_FLAGS(PWM1_MOTOR_NODE)
#else
#error "Unsupported board: pwm-motor1 devicetree alias is not defined"
#define PWM1_LABEL	""
#define PWM1_CHANNEL	0
#define PWM1_FLAGS	0
#endif
/*
 * This period should be fast enough to be above the flicker fusion
 * threshold. The steps should also be small enough, and happen
 * quickly enough, to make the output fade change appear continuous.
 */
<<<<<<< HEAD
#define PERIOD_USEC	3333U // 300 Hz Para la válvula de salida
=======
#define PERIOD_USEC	10U
>>>>>>> 1f43912ddb54cb9b8c1e97c42d41a5a18a25811a
#define NUM_STEPS	10U
#define STEP_USEC	(PERIOD_USEC / NUM_STEPS)
#define SLEEP_MSEC	100U



void main(void)
{
	const struct device *pwm, *pwm1;
	uint32_t pulse_width = 0U;
	uint8_t dir = 1U;
	int ret;

	printk("PWM-based LED fade\n");
	printk("Channel = %d\n", PWM_CHANNEL);

	pwm = device_get_binding(PWM_LABEL);

	if (!pwm) {
		printk("Error: didn't find %s device\n", PWM_LABEL);
		return;
	}
	
	pwm1 = device_get_binding(PWM1_LABEL);
<<<<<<< HEAD
	if (!pwm1) {
=======
	if (!pwm) {
>>>>>>> 1f43912ddb54cb9b8c1e97c42d41a5a18a25811a
		printk("Error: didn't find %s device\n", PWM1_LABEL);
		return;
	}
	//ret = pwm_pin_set_cycles(pwm1, PWM1_CHANNEL, 1001,
	//			       1000, PWM1_FLAGS);
	

	while (1) {
		ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD_USEC,
				       pulse_width, PWM_FLAGS);
		int period = PERIOD_USEC;
<<<<<<< HEAD
		if (ret) {
			while(ret){
				printk("Error %d: failed to set pulse width %d and period %d in ch0\n", ret, pulse_width, period);
				period= period-2;
				ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, period,
				       pulse_width, PWM_FLAGS);
			}
			
			return;
		}
		ret = pwm_pin_set_usec(pwm1, PWM1_CHANNEL, PERIOD_USEC, pulse_width, PWM1_FLAGS);

		if (ret) {
=======
		if (ret) {
			while(ret){
				printk("Error %d: failed to set pulse width %d and period %d in ch0\n", ret, pulse_width, period);
				period= period-2;
				ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, period,
				       pulse_width, PWM_FLAGS);
			}
			
			return;
		}
		ret = pwm_pin_set_usec(pwm1, PWM1_CHANNEL, PERIOD_USEC, pulse_width, PWM1_FLAGS);

		if (ret) {
>>>>>>> 1f43912ddb54cb9b8c1e97c42d41a5a18a25811a
			printk("Error %d: failed to set pulse width in ch1\n", ret);
			return;
		}

		if (dir) {
			pulse_width += STEP_USEC;
			if (pulse_width >= PERIOD_USEC) {
				pulse_width = PERIOD_USEC - STEP_USEC;
				dir = 0U;
			}
		} else {
			if (pulse_width >= STEP_USEC) {
				pulse_width -= STEP_USEC;
			} else {
				pulse_width = STEP_USEC;
				dir = 1U;
			}
		}
		printk("Pulse width: %d\n", pulse_width);
		k_sleep(K_MSEC(SLEEP_MSEC));
	}
}
