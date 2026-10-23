/*
 *	barcode_core.c
 *	Barcode capture (see barcode_core.h).
 */

#include "barcode_core.h"
#include "../bitpack.h"

#define BC_MIN_REPORTED	3U	/* fewer elements than this is not worth reporting */

void bc_init(BC_CORE *b, const BC_PARAMS *p)
{
	*b = (BC_CORE){0};
	b->p = *p;
}

void bc_reset(BC_CORE *b)
{
	b->capturing = false;
	b->broken = false;
	b->n = 0U;
	b->widest_us = 0U;
}

void bc_edge(BC_CORE *b, uint32_t t_us, uint8_t level)
{
	uint32_t gap;

	t_us &= BP_T31_MASK;			/* 31-bit edge time (drv_barcode.h) */
	if (!b->capturing) {
		if (level == b->p.bar_level) {		/* first edge into a bar */
			b->t[0] = t_us;
			b->n = 1U;
			b->widest_us = 0U;
			b->last_level = level;
			b->capturing = true;
			b->broken = false;
		}
		return;
	}
	if (b->broken) {
		b->t[0] = t_us;			/* only keep time, to find the end */
		return;
	}

	if (level == b->last_level) {
		return;		/* two transitions that cancelled out: no change */
	}
	gap = bp_dt31(t_us, b->t[b->n - 1U]);	/* wrap-safe */
	if (gap < b->p.t_min_us) {
		/* Chatter: this edge and the one before it are a glitch pair.
		   Removing both keeps the levels alternating. */
		b->n--;
		b->last_level = (uint8_t)(b->last_level ^ 1U);
		if (b->n == 0U) {
			b->capturing = false;
		}
		return;
	}
	if (b->n >= BC_MAX_EDGES) {
		/* far too many edges: this capture cannot be a barcode */
		b->broken = true;
		b->t[0] = t_us;
		b->n = 1U;
		return;
	}
	b->t[b->n] = t_us;
	b->n++;
	b->last_level = level;
	if (gap > b->widest_us) {
		b->widest_us = gap;
	}
}

bool bc_poll(BC_CORE *b, uint32_t now_us, BC_RESULT *res)
{
	uint32_t	w[CODE39_ELEMENTS];
	uint32_t	t_end, i, n;

	if (!b->capturing) {
		return false;
	}
	t_end = b->p.t_end_factor * b->widest_us;
	if (t_end < b->p.t_end_min_us) {
		t_end = b->p.t_end_min_us;
	}
	if (bp_dt31(now_us, b->t[b->n - 1U]) <= t_end) {
		return false;			/* still inside the barcode */
	}

	/* Capture finished: widths = edges - 1 */
	n = b->broken ? (BC_MAX_EDGES + 1U) : (b->n - 1U);
	b->capturing = false;
	res->elements = n;
	res->c = '?';
	res->reversed = false;
	res->narrow_us = 0U;

	if (n < BC_MIN_REPORTED) {
		return false;			/* a line crossing, not a barcode */
	}
	if (n != CODE39_ELEMENTS) {
		res->result = C39_BAD_COUNT;
	} else {
		res->narrow_us = UINT32_MAX;
		for (i = 0U; i < CODE39_ELEMENTS; i++) {
			w[i] = bp_dt31(b->t[i + 1U], b->t[i]);
			if (w[i] < res->narrow_us) {
				res->narrow_us = w[i];
			}
		}
		res->result = code39_decode(w, CODE39_ELEMENTS, &res->c, &res->reversed);
	}
	if (res->result == C39_OK) {
		b->ok++;
	} else {
		b->failed++;
	}
	return true;
}

int32_t bc_command_for(char c)
{
	switch (c) {
	case 'A':
		return LB_CMD_TURN_LEFT;
	case 'B':
		return LB_CMD_TURN_RIGHT;
	case 'C':
		return LB_CMD_STRAIGHT;
	case 'D':
		return LB_CMD_U_TURN;
	default:
		return LB_CMD_NONE;		/* e.g. the sample 'Z' */
	}
}
