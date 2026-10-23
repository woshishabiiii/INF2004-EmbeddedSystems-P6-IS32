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
#include "../bitpack.h"

/* Set to -1 if a wheel counts down when it rolls forward */
#define ENC_SIGN_LEFT	(1)	/* MEASURE: -1 if it counts down rolling forward */
#define ENC_SIGN_RIGHT	(-1)	/* MEASURE: -1 if it counts down rolling forward */
/* 2026-10-23: both flipped. With encoder A and B swapped on both motors,
   rolling forward gave enc [-1510, -1494] and FAULT 3 with the old signs. */

typedef struct {
	UINT		pin_a;
	UINT		pin_b;
	W		sign;
} ENC_CH;

LOCAL const ENC_CH s_enc[2] = {
	{ 2U,  3U, ENC_SIGN_LEFT},
	{16U, 17U, ENC_SIGN_RIGHT},
};

/* Shared with Motion Control (drv_encoder.h).  The RP2040 is little-
   endian, so h[0] is bits 0-15 of w.  Written only here, in the
   interrupt (core 1), by halfword; read only as the whole word. */
typedef union {
	UW	w;
	UH	h[2];
} ENC_WORD;

BP_STATIC_ASSERT(sizeof(ENC_WORD) == 4U, enc_word_is_one_word);

LOCAL volatile ENC_WORD	s_counts;
LOCAL volatile UB	s_stat[2];	/* state 2 bits + error count 6 bits, per wheel */

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

/* Runs in interrupt context (called by gpio_irq.c).  The only writer of
   s_counts and s_stat. */
LOCAL void enc_edge(UINT pin, UW events, UW t_us)
{
	UINT	side;
	UW	stat;
	UW	old;
	UW	now;
	UW	idx;

	(void)events;
	(void)t_us;

	side = ((pin == s_enc[ENC_LEFT].pin_a) || (pin == s_enc[ENC_LEFT].pin_b)) ?
	       ENC_LEFT : ENC_RIGHT;
	stat = s_stat[side];
	old = stat & ENC_STAT_STATE_MASK;
	now = read_state(&s_enc[side]);
	idx = (old << 2) | now;
	if ((now != old) && (s_qdec[idx] == 0)) {
		UW err = stat >> ENC_STAT_ERR_SHIFT;	/* both pins changed: an edge was missed */

		if (err < ENC_STAT_ERR_MAX) {
			err++;			/* saturates at 63 */
		}
		stat = err << ENC_STAT_ERR_SHIFT;
	} else {
		/* 16-bit store of this wheel's half only: wraps at 65,536 */
		s_counts.h[side] = (UH)(s_counts.h[side] + (UH)(s_qdec[idx] * s_enc[side].sign));
		stat &= ~ENC_STAT_STATE_MASK;
	}
	s_stat[side] = (UB)(stat | now);
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

	s_counts.w = 0U;			/* before the interrupt is attached */
	for (i = 0U; i < 2U; i++) {
		const ENC_CH *e = &s_enc[i];

		input_pull_up(e->pin_a);
		input_pull_up(e->pin_b);
		s_stat[i] = (UB)read_state(e);	/* error count 0 */
		if (er == E_OK) {
			er = gpio_irq_attach(e->pin_a, GPIO_EDGE_BOTH, enc_edge);
		}
		if (er == E_OK) {
			er = gpio_irq_attach(e->pin_b, GPIO_EDGE_BOTH, enc_edge);
		}
	}
	return er;
}

EXPORT UW encoder_word(void)
{
	return s_counts.w;			/* one 32-bit load */
}

EXPORT UW encoder_errors(UINT side)
{
	return (side <= ENC_RIGHT) ? ((UW)s_stat[side] >> ENC_STAT_ERR_SHIFT) : 0U;
}
