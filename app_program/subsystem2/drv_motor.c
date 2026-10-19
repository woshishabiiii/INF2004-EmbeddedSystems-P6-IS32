/*
 *	drv_motor.c
 *	Robo Pico DC motor driver (see drv_motor.h).
 *
 *	Robo Pico truth table (datasheet section 4):
 *	  A low,  B low   -> brake
 *	  A high, B low   -> one direction  (which one depends on the wiring)
 *	  A low,  B high  -> the other direction
 *	  A high, B high  -> coast (outputs open)
 *	For speed control one input carries the PWM and the other stays low,
 *	so the motor alternates between drive and brake (never coast).
 *
 *	PWM: clk_sys 125 MHz, divider 1, TOP 7811 -> 125e6 / 7812 = 16.0 kHz,
 *	below the Robo Pico's 20 kHz driver limit.  The compare register of a
 *	slice holds channel A in bits 15:0 and channel B in bits 31:16; both
 *	are written in ONE store, so A and B are never high together.
 */

#include "drv_motor.h"

#define PWM_SLICE_REG(s, off)	(PWM_BASE + ((UW)(s) * 0x14U) + (UW)(off))
#define MOTOR_PWM_TOP		7811U
#define MOTOR_PWM_DIV_INT1	(1U << 4)	/* DIV register: integer part in bits 11:4 */
#define REG_ALIAS_CLR		0x3000U
#define CC_B_SHIFT		16U

/* Wiring fix-ups: set to -1 if a wheel turns BACKWARDS for a positive
   duty, instead of swapping the motor wires. */
#define MOTOR_DIR_LEFT		(1)	/* MEASURE */
#define MOTOR_DIR_RIGHT		(-1)	/* MEASURE */

typedef struct {
	UINT	pin_a;
	UINT	pin_b;
	UW	slice;
	W	dir;
} MOTOR_CH;

LOCAL const MOTOR_CH s_ch[2] = {
	{10U, 11U, 5U, MOTOR_DIR_LEFT},		/* MOTOR_LEFT  (M2) */
	{ 8U,  9U, 4U, MOTOR_DIR_RIGHT},	/* MOTOR_RIGHT (M1) */
};

/* The kernel only takes PWM out of reset when its own PWM timer option is
   on, which this build does not use.  Writing the bit to the atomic CLEAR
   alias releases the block without ever putting it back into reset, so
   the motor and servo drivers can both call this, in any order. */
LOCAL void pwm_block_release(void)
{
	out_w(RESETS_RESET + REG_ALIAS_CLR, RESETS_RESET_PWM);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
		/* a few clock cycles */
	}
}

LOCAL void slice_setup(UW slice)
{
	out_w(PWM_SLICE_REG(slice, PWM_CHx_CSR), 0U);		/* stop */
	out_w(PWM_SLICE_REG(slice, PWM_CHx_DIV), MOTOR_PWM_DIV_INT1);
	out_w(PWM_SLICE_REG(slice, PWM_CHx_TOP), MOTOR_PWM_TOP);
	out_w(PWM_SLICE_REG(slice, PWM_CHx_CC), 0U);		/* A and B low: brake */
	out_w(PWM_SLICE_REG(slice, PWM_CHx_CTR), 0U);
	out_w(PWM_SLICE_REG(slice, PWM_CHx_CSR), PWM_CH_CSR_EN);	/* run */
}

EXPORT ER motor_init(void)
{
	UINT i;

	pwm_block_release();
	for (i = 0U; i < 2U; i++) {
		slice_setup(s_ch[i].slice);
		(void)pwm_set_pin(s_ch[i].pin_a);	/* pin function = PWM */
		(void)pwm_set_pin(s_ch[i].pin_b);
	}
	return E_OK;
}

EXPORT void motor_set(UINT side, W duty)
{
	const MOTOR_CH	*ch;
	UW		mag;
	UW		cc;

	if (side > MOTOR_RIGHT) {
		return;
	}
	ch = &s_ch[side];
	duty *= ch->dir;
	if (duty > MOTOR_DUTY_MAX) {
		duty = MOTOR_DUTY_MAX;
	} else if (duty < -MOTOR_DUTY_MAX) {
		duty = -MOTOR_DUTY_MAX;
	} else {
		/* in range */
	}

	/* permille -> compare value; CC = TOP + 1 keeps the output high */
	mag = (UW)((duty < 0) ? -duty : duty);
	mag = (mag * (MOTOR_PWM_TOP + 1U)) / (UW)MOTOR_DUTY_MAX;
	if (duty > 0) {
		cc = mag;			/* PWM on A, B low */
	} else if (duty < 0) {
		cc = mag << CC_B_SHIFT;		/* PWM on B, A low */
	} else {
		cc = 0U;			/* both low: brake */
	}
	out_w(PWM_SLICE_REG(ch->slice, PWM_CHx_CC), cc);
}

EXPORT void motor_brake_all(void)
{
	motor_set(MOTOR_LEFT, 0);
	motor_set(MOTOR_RIGHT, 0);
}
