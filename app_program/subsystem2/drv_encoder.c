/*
 *	drv_encoder.c
 *	Quadrature decoding in software ("x4": every edge of A and B counts).
 *
 *	The two channels A and B are square waves 90 degrees apart.  The pair
 *	(A,B) steps through 00 -> 01 -> 11 -> 10 -> 00 one way and the reverse
 *	the other way.  Every edge we look up (previous state, new state) in a
 *	table: +1, -1, or 0 (no change / impossible jump).
 */

#include "drv_encoder.h"
#include "../gpio_irq.h"

/* Set to -1 if a wheel counts down when it rolls forward */
#define ENC_SIGN_LEFT	(1)	/* MEASURE: -1 if it counts down rolling forward */
#define ENC_SIGN_RIGHT	(-1)	/* MEASURE: -1 if it counts down rolling forward */

typedef struct {
	UINT		pin_a;
	UINT		pin_b;
	W		sign;
	volatile W	count;
	volatile UW	errors;
	volatile UW	state;		/* last (A << 1) | B */
} ENC_CH;

LOCAL ENC_CH s_enc[2] = {
	{ 2U,  3U, ENC_SIGN_LEFT,  0, 0U, 0U},
	{16U, 17U, ENC_SIGN_RIGHT, 0, 0U, 0U},
};

/* index = (old_state << 2) | new_state */
LOCAL const B s_qdec[16] = {
	 0, -1, +1,  0,
	+1,  0,  0, -1,
	-1,  0,  0, +1,
	 0, +1, -1,  0
};

LOCAL UW read_state(const ENC_CH *e)
{
	UW in = in_w(GPIO_IN);

	return (((in >> e->pin_a) & 1U) << 1) | ((in >> e->pin_b) & 1U);
}

/* Runs in interrupt context (called by gpio_irq.c) */
LOCAL void enc_edge(UINT pin, UW events, UW t_us)
{
	ENC_CH	*e;
	UW	now;
	UW	idx;

	(void)events;
	(void)t_us;

	e = ((pin == s_enc[ENC_LEFT].pin_a) || (pin == s_enc[ENC_LEFT].pin_b)) ?
	    &s_enc[ENC_LEFT] : &s_enc[ENC_RIGHT];
	now = read_state(e);
	idx = (e->state << 2) | now;
	if ((now != e->state) && (s_qdec[idx] == 0)) {
		e->errors++;		/* both pins changed: an edge was missed */
	} else {
		e->count += (W)s_qdec[idx] * e->sign;
	}
	e->state = now;
}

/* Encoder outputs may be open-collector, so use the pull-up (harmless if
   they are push-pull) and turn off the reset-default pull-down. */
LOCAL void input_pull_up(UINT pin)
{
	(void)gpio_set_pin(pin, GPIO_MODE_IN);
	clr_w(GPIO(pin), GPIO_PDE);
	set_w(GPIO(pin), GPIO_PUE);
}

EXPORT ER encoder_init(void)
{
	UINT	i;
	ER	er = E_OK;

	for (i = 0U; i < 2U; i++) {
		ENC_CH *e = &s_enc[i];

		input_pull_up(e->pin_a);
		input_pull_up(e->pin_b);
		e->state = read_state(e);
		e->count = 0;
		e->errors = 0U;
		if (er == E_OK) {
			er = gpio_irq_attach(e->pin_a, GPIO_EDGE_BOTH, enc_edge);
		}
		if (er == E_OK) {
			er = gpio_irq_attach(e->pin_b, GPIO_EDGE_BOTH, enc_edge);
		}
	}
	return er;
}

EXPORT W encoder_count(UINT side)
{
	return (side <= ENC_RIGHT) ? s_enc[side].count : 0;
}

EXPORT UW encoder_errors(UINT side)
{
	return (side <= ENC_RIGHT) ? s_enc[side].errors : 0U;
}
