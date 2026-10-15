/*
 *	line_core.c
 *	Line sensing and the line-following rule (see line_core.h).
 */

#include "line_core.h"

#define LS_RATE_PERIOD_MS	100U	/* d_rate is per 100 ms */
#define LS_HELD_MAX_MS		5000U	/* longer holds add nothing (and cannot overflow) */
#define LS_POS_FAR		2

/* ---- Region A ----------------------------------------------------------- */
void lc_init(LC_CORE *c, const LC_PARAMS *p, uint32_t now_ms)
{
	*c = (LC_CORE){0};
	c->p = *p;
	c->prev_l = -1;
	c->prev_r = -1;
	c->last_black_ms = now_ms;	/* the T_lost clock starts at power-up */
}

void lc_step(LC_CORE *c, bool left_black, bool right_black, uint32_t now_ms,
	     int32_t odo_mm, EVT_LIST *ev)
{
	int32_t l = left_black ? 1 : 0;
	int32_t r = right_black ? 1 : 0;

	/* 1. Status, sent only when it changes */
	if ((l != c->prev_l) || (r != c->prev_r)) {
		(void)evt_list_add(ev, (uint8_t)EV_LINE_STATUS, l, r);
		c->prev_l = l;
		c->prev_r = r;
	}

	/* 2. Which side was the line last seen on, and when */
	if (left_black || right_black) {
		c->last_black_ms = now_ms;
		c->lost_sent = false;
		if (left_black != right_black) {
			c->last_side = left_black ? -1 : 1;
		}
	}

	/* 3. Junction: both black for junction_ms AND junction_mm of travel,
	      so a car standing still on a black band is not a junction. */
	if (left_black && right_black) {
		if (!c->both) {
			c->both = true;
			c->both_ms = now_ms;
			c->both_mm = odo_mm;
		}
		if (!c->junction_sent &&
		    ((now_ms - c->both_ms) >= c->p.junction_ms) &&
		    ((odo_mm - c->both_mm) >= c->p.junction_mm)) {
			(void)evt_list_add(ev, (uint8_t)EV_JUNCTION, odo_mm, 0);
			c->junction_sent = true;
			c->junctions++;
		}
	} else {
		c->both = false;
		c->junction_sent = false;
	}

	/* 4. Line lost: no black for T_lost */
	if (!left_black && !right_black && !c->lost_sent &&
	    ((now_ms - c->last_black_ms) > c->p.lost_ms)) {
		(void)evt_list_add(ev, (uint8_t)EV_LINE_LOST, c->last_side, 0);
		c->lost_sent = true;
	}
}

/* ---- steering ----------------------------------------------------------- */
void ls_init(LS_STATE *s, const LS_PARAMS *p, uint32_t now_ms)
{
	*s = (LS_STATE){0};
	s->p = *p;
	ls_restart(s, now_ms, p->start_side);
}

void ls_restart(LS_STATE *s, uint32_t now_ms, int32_t side)
{
	s->since_ms = now_ms;
	s->last_black_ms = now_ms;
	s->last_side = (side < 0) ? -1 : 1;
	s->on_since_ms = now_ms;
	s->last_on_ms = 0U;		/* assume the line is between the sensors */
}

void ls_update(LS_STATE *s, bool left_black, bool right_black, uint32_t now_ms)
{
	bool was_one = (s->l != s->r);
	bool is_one = (left_black != right_black);

	if ((left_black != s->l) || (right_black != s->r)) {
		s->since_ms = now_ms;
	}
	if (is_one && (!was_one || (left_black != s->l))) {
		s->on_since_ms = now_ms;		/* one sensor alone on black */
	} else if (was_one && !left_black && !right_black) {
		s->last_on_ms = now_ms - s->on_since_ms;	/* ... and now both white */
	} else {
		/* no episode starts or ends */
	}
	s->l = left_black;
	s->r = right_black;
	if (left_black || right_black) {
		s->last_black_ms = now_ms;
		if (left_black != right_black) {
			s->last_side = left_black ? -1 : 1;
		}
	}
}

/* Turn toward side (-1 left, +1 right) by slowing the INSIDE wheel by
   2 x delta; the outside wheel keeps the base speed.  No wheel is ever
   asked for more than the base speed, so there is always room to steer,
   and the car slows down in curves by itself. */
static void steer(LS_OUT *out, int32_t speed, int32_t side, int32_t delta)
{
	out->v_left = speed - ((side < 0) ? (2 * delta) : 0);
	out->v_right = speed - ((side > 0) ? (2 * delta) : 0);
}

void ls_output(const LS_STATE *s, uint32_t now_ms, int32_t base_mm_s, LS_OUT *out)
{
	uint32_t	held_ms = now_ms - s->since_ms;
	int32_t		delta;

	if (held_ms > LS_HELD_MAX_MS) {
		held_ms = LS_HELD_MAX_MS;
	}
	out->side = s->last_side;
	out->lost = (!s->l && !s->r && ((now_ms - s->last_black_ms) > s->p.t_lost_ms));

	if (s->l && s->r) {
		/* junction or crossing band: drive straight across */
		out->pos = 0;
		steer(out, base_mm_s, 0, 0);
	} else if (s->l || s->r) {
		/* one sensor on the line: correct toward it, harder the longer
		   it stays there (the line is not coming back by itself) */
		delta = s->p.d_min + (int32_t)((held_ms * (uint32_t)s->p.d_rate) / LS_RATE_PERIOD_MS);
		if (delta > s->p.d_max) {
			delta = s->p.d_max;
		}
		out->pos = s->l ? -1 : 1;
		steer(out, base_mm_s, out->pos, delta);
	} else if (s->last_on_ms >= s->p.t_escape_ms) {
		/* both white, and the last sensor had been on black a long
		   time: the line slipped out past it.  Slow down and turn
		   firmly back toward it. */
		out->pos = s->last_side * LS_POS_FAR;
		steer(out, (base_mm_s * s->p.slow_pct) / 100, s->last_side, s->p.d_search);
	} else {
		/* both white, the line left the sensor quickly: it is between
		   the sensors.  Lean gently toward the side it was last seen, so
		   a sensor touches its edge again and tells us where it is.
		   After t_drift without a touch the lean starts to grow, by
		   d_bias every further t_drift (a curve can cancel a fixed
		   lean), up to d_search.  Not sooner: crossing a short gap in
		   the line should not swing the car off its heading. */
		uint32_t white_ms = now_ms - s->last_black_ms;
		uint32_t grow_ms;

		if (white_ms > LS_HELD_MAX_MS) {
			white_ms = LS_HELD_MAX_MS;
		}
		grow_ms = (white_ms > s->p.t_drift_ms) ? (white_ms - s->p.t_drift_ms) : 0U;
		out->pos = 0;
		delta = s->p.d_bias + (int32_t)((grow_ms * (uint32_t)s->p.d_bias) / s->p.t_drift_ms);
		if (delta > s->p.d_search) {
			delta = s->p.d_search;
		}
		steer(out, base_mm_s, s->last_side, delta);
	}
}
