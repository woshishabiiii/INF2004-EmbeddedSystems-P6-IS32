/*
 *	drv_servo.c  (see drv_servo.h)
 *	125 MHz / divider 125 = 1 MHz -> one PWM count = 1 us.
 *	TOP = 19999 -> period 20 000 us = 50 Hz.  CC = pulse width in us.
 */

#include "drv_servo.h"

#define SERVO_PIN		12U
#define SERVO_SLICE		6U
#define SERVO_DIV_INT125	(125U << 4)	/* DIV register: integer part in bits 11:4 */
#define SERVO_TOP		19999U
#define PWM_SLICE_REG(s, off)	(PWM_BASE + ((UW)(s) * 0x14U) + (UW)(off))
#define REG_ALIAS_CLR		0x3000U

/* Same as in drv_motor.c: release only, never re-assert the reset, so the
   motors keep running whichever driver starts first. */
LOCAL void pwm_block_release(void)
{
	out_w(RESETS_RESET + REG_ALIAS_CLR, RESETS_RESET_PWM);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
		/* a few clock cycles */
	}
}

EXPORT ER servo_init(void)
{
	pwm_block_release();
	out_w(PWM_SLICE_REG(SERVO_SLICE, PWM_CHx_CSR), 0U);
	out_w(PWM_SLICE_REG(SERVO_SLICE, PWM_CHx_DIV), SERVO_DIV_INT125);
	out_w(PWM_SLICE_REG(SERVO_SLICE, PWM_CHx_TOP), SERVO_TOP);
	out_w(PWM_SLICE_REG(SERVO_SLICE, PWM_CHx_CTR), 0U);
	servo_set_deg(90);
	out_w(PWM_SLICE_REG(SERVO_SLICE, PWM_CHx_CSR), PWM_CH_CSR_EN);
	(void)pwm_set_pin(SERVO_PIN);
	return E_OK;
}

EXPORT void servo_set_us(W pulse_us)
{
	if (pulse_us < SERVO_US_MIN) {
		pulse_us = SERVO_US_MIN;
	} else if (pulse_us > SERVO_US_MAX) {
		pulse_us = SERVO_US_MAX;
	} else {
		/* in range */
	}
	(void)pwm_set_cc(SERVO_PIN, (UW)pulse_us);
}

EXPORT void servo_set_deg(W deg)
{
	if (deg < 30) {
		deg = 30;
	} else if (deg > 150) {
		deg = 150;
	} else {
		/* in range */
	}
	servo_set_us(SERVO_US_AT_30DEG +
		     (((deg - 30) * (SERVO_US_AT_150DEG - SERVO_US_AT_30DEG)) / 120));
}
