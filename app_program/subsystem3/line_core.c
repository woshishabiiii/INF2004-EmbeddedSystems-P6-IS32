/*
 *	line_core.c
 *	Line sensing and the line-following rule (see line_core.h).
 */

#include "line_core.h"
#include "../bitpack.h"

#define LS_HELD_MAX_MM		1000	/* longer holds add nothing (and cannot overflow) */
#define LS_POS_FAR		2
#define PERMILLE		1000

/* travelled since `from`, never negative, capped at LS_HELD_MAX_MM */
static int32_t travelled(int32_t odo_mm, int32_t from_mm)
{
	int32_t d = odo_mm - from_mm;

	if (d < 0) {
		d = 0;
	}
	return (d > LS_HELD_MAX_MM) ? LS_HELD_MAX_MM : d;
}

/* ---- Region A ----------------------------------------------------------- */
void lc_init(LC_CORE *c, const LC_PARAMS *p, int32_t odo_mm)
{
	*c = (LC_CORE){0};
	c->p = *p;
	c->prev_l = -1;
	c->prev_r = -1;
	c->last_black_mm = odo_mm;	/* the T_lost distance starts here */
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

	/* 2. Which side was the line last seen on, and where */
	if (left_black || right_black) {
		c->last_black_mm = odo_mm;
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

	/* 4. Line lost: no black over T_lost of travel.  A car standing
	      still is never "lost" by waiting. */
	if (!left_black && !right_black && !c->lost_sent &&
	    ((odo_mm - c->last_black_mm) > c->p.lost_mm)) {
		(void)evt_list_add(ev, (uint8_t)EV_LINE_LOST, c->last_side, 0);
		c->lost_sent = true;
	}
}

/* ---- steering ----------------------------------------------------------- */
void ls_init(LS_STATE *s, const LS_PARAMS *p, int32_t odo_mm)
{
	*s = (LS_STATE){0};
	s->p = *p;
	ls_restart(s, odo_mm, p->start_side);
}

void ls_restart(LS_STATE *s, int32_t odo_mm, int32_t side)
{
	s->since_mm = odo_mm;
	s->last_black_mm = odo_mm;
	s->last_side = (side < 0) ? -1 : 1;
	s->on_since_mm = odo_mm;
	s->last_on_mm = 0;		/* assume the line is between the sensors */
}

void ls_update(LS_STATE *s, bool left_black, bool right_black, int32_t odo_mm)
{
	bool was_one = (s->l != s->r);
	bool is_one = (left_black != right_black);

	if ((left_black != s->l) || (right_black != s->r)) {
		s->since_mm = odo_mm;
	}
	if (is_one && (!was_one || (left_black != s->l))) {
		s->on_since_mm = odo_mm;		/* one sensor alone on black */
	} else if (was_one && !left_black && !right_black) {
		s->last_on_mm = travelled(odo_mm, s->on_since_mm);	/* ... and now both white */
	} else {
		/* no episode starts or ends */
	}
	s->l = left_black;
	s->r = right_black;
	if (left_black || right_black) {
		s->last_black_mm = odo_mm;
		if (left_black != right_black) {
			s->last_side = left_black ? -1 : 1;
		}
	}
}

/* Turn toward side (-1 left, +1 right) by slowing the INSIDE wheel by
   2 x delta; the outside wheel keeps the speed.  No wheel is ever asked
   for more than the speed, so there is always room to steer, and the car
   slows down in curves by itself. */
static void steer(LS_OUT *out, int32_t speed, int32_t side, int32_t delta)
{
	out->v_left = speed - ((side < 0) ? (2 * delta) : 0);
	out->v_right = speed - ((side > 0) ? (2 * delta) : 0);
}

/* permille of the speed -> mm/s */
static int32_t of_speed(int32_t speed_mm_s, int32_t permille)
{
	return (speed_mm_s * permille) / PERMILLE;
}

void ls_output(const LS_STATE *s, int32_t odo_mm, int32_t speed_mm_s, LS_OUT *out)
{
	int32_t held_mm = travelled(odo_mm, s->since_mm);
	int32_t white_mm = travelled(odo_mm, s->last_black_mm);
	int32_t d;

	out->side = s->last_side;
	out->lost = (!s->l && !s->r && ((odo_mm - s->last_black_mm) > s->p.lost_mm));

	if (s->l && s->r) {
		/* junction or crossing band: drive straight across */
		out->pos = 0;
		steer(out, speed_mm_s, 0, 0);
	} else if (s->l || s->r) {
		/* one sensor on the line: correct toward it, harder the further
		   it stays there (the line is not coming back by itself) */
		d = s->p.d_min + ((held_mm * s->p.d_rate) / s->p.rate_mm);
		if (d > s->p.d_max) {
			d = s->p.d_max;
		}
		out->pos = s->l ? -1 : 1;
		steer(out, speed_mm_s, out->pos, of_speed(speed_mm_s, d));
	} else if (s->last_on_mm >= s->p.escape_mm) {
		/* both white, and the last sensor had been on black a long
		   way: the line slipped out past it.  Slow down and turn
		   firmly back toward it. */
		out->pos = s->last_side * LS_POS_FAR;
		steer(out, (speed_mm_s * s->p.slow_pct) / 100, s->last_side,
		      of_speed(speed_mm_s, s->p.d_search));
	} else {
		/* both white, the line left the sensor quickly: it is between
		   the sensors.  Lean gently toward the side it was last seen, so
		   a sensor touches its edge again and tells us where it is.
		   After drift_mm without a touch the lean starts to grow, by
		   d_bias every further drift_mm (a curve can cancel a fixed
		   lean), up to d_search.  Not sooner: crossing a short gap in
		   the line should not swing the car off its heading. */
		int32_t grow_mm = (white_mm > s->p.drift_mm) ? (white_mm - s->p.drift_mm) : 0;

		out->pos = 0;
		d = s->p.d_bias + ((grow_mm * s->p.d_bias) / s->p.drift_mm);
		if (d > s->p.d_search) {
			d = s->p.d_search;
		}
		steer(out, speed_mm_s, s->last_side, of_speed(speed_mm_s, d));
	}
}

/* ---- packed status word (line_core.h) ----------------------------------- */
#define LCS_CHAR_MIN	32		/* ' ' */

uint32_t lc_status_pack(const LC_STATUS *s)
{
	return (s->left_black ? 1U : 0U) |
	       (s->right_black ? 2U : 0U) |
	       (bp_sat_s((int32_t)s->last_char, LCS_CHAR_MIN, 6U) << 2) |
	       (bp_sat_u((int32_t)s->reads_ok, 8U) << 8) |
	       (bp_sat_u((int32_t)s->reads_failed, 8U) << 16) |
	       (bp_sat_u((int32_t)s->junctions, 8U) << 24);
}

void lc_status_unpack(uint32_t w, LC_STATUS *s)
{
	s->left_black = ((w & 1U) != 0U);
	s->right_black = ((w & 2U) != 0U);
	s->last_char = (char)bp_from_s((w >> 2) & BP_MAX(6U), LCS_CHAR_MIN);
	s->reads_ok = (w >> 8) & 0xFFU;
	s->reads_failed = (w >> 16) & 0xFFU;
	s->junctions = w >> 24;
}
