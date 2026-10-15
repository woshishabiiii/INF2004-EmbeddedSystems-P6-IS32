/*
 *	line_core.h
 *	Line sensing and the line-following algorithm (Buddy 3).
 *
 *	The two line sensors straddle the 18 mm line about 25 mm apart:
 *	  both white  -> centred (or the line is lost: see T_lost below)
 *	  left black  -> the car has drifted right: steer left
 *	  right black -> the car has drifted left: steer right
 *	  both black  -> a junction or crossing band
 *
 *	Two parts, both pure C (stdint / stdbool only), unit-tested on a
 *	laptop in testing/host/:
 *	  lc_*  Region A of Figure 5, run by the Line and Barcode task every
 *	        5 ms: LINE_STATUS (on change), JUNCTION and LINE_LOST events.
 *	  ls_*  the steering rule, called by Vehicle Controller (the only task
 *	        that sends motion commands) to turn the latest line status
 *	        into left and right wheel speeds.
 *
 *	Both white is ambiguous: the line is either between the sensors or
 *	has slipped outside one of them.  The rule tells them apart by how
 *	the line left the last sensor: if that sensor had been on black for
 *	t_escape_ms or more (the car was already steering hard and still not
 *	turning enough, as in a curve), the line went OUT past it; if it left
 *	quickly, the correction worked and the line is between the sensors.
 */

#ifndef LINE_CORE_H
#define LINE_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_evt.h"

/* ---- Region A: line status, junction, line lost ------------------------ */
typedef struct {
	uint32_t	junction_ms;	/* both black for at least this long ... */
	int32_t		junction_mm;	/* ... and over at least this much travel */
	uint32_t	lost_ms;	/* no black seen for this long = line lost (T_lost) */
} LC_PARAMS;

typedef struct {
	LC_PARAMS	p;
	int32_t		prev_l;		/* last reported status, -1 = none yet */
	int32_t		prev_r;
	bool		both;		/* both sensors on black now */
	uint32_t	both_ms;	/* ... since this time */
	int32_t		both_mm;	/* ... since this odometer reading */
	bool		junction_sent;
	int32_t		last_side;	/* -1 left, +1 right, 0 unknown */
	uint32_t	last_black_ms;
	bool		lost_sent;
	uint32_t	junctions;	/* count, for telemetry */
} LC_CORE;

void	lc_init(LC_CORE *c, const LC_PARAMS *p, uint32_t now_ms);
void	lc_step(LC_CORE *c, bool left_black, bool right_black, uint32_t now_ms,
		int32_t odo_mm, EVT_LIST *ev);

/* ---- the steering rule (used by Vehicle Controller) -------------------- */
typedef struct {
	int32_t		d_min;		/* mm/s of steering when a sensor first touches */
	int32_t		d_rate;		/* mm/s added per 100 ms the sensor stays on black */
	int32_t		d_max;		/* mm/s, largest correction */
	int32_t		d_bias;		/* mm/s gentle lean toward the last side while centred */
	int32_t		d_search;	/* mm/s when the line has slipped outside */
	uint32_t	t_escape_ms;	/* a sensor on black at least this long before it went
					   white: the line slipped OUTSIDE it (a curve) */
	uint32_t	t_drift_ms;	/* no touch for this long: the lean grows by d_bias per t_drift (T_drift) */
	uint32_t	t_lost_ms;	/* no black for this long: line lost (T_lost) */
	int32_t		slow_pct;	/* % of the base speed while turning back to the line */
	int32_t		start_side;	/* side to lean toward before any black is seen */
} LS_PARAMS;

typedef struct {
	LS_PARAMS	p;
	bool		l;		/* latest status */
	bool		r;
	uint32_t	since_ms;	/* when the latest status began */
	uint32_t	last_black_ms;
	int32_t		last_side;	/* -1 left, +1 right */
	uint32_t	on_since_ms;	/* when one sensor alone went black */
	uint32_t	last_on_ms;	/* how long it stayed black, last time */
} LS_STATE;

typedef struct {
	int32_t		v_left;		/* mm/s */
	int32_t		v_right;
	bool		lost;		/* no black for t_lost_ms */
	int32_t		side;		/* side the line was last seen */
	int32_t		pos;		/* rough line position: -2 far left .. +2 far right */
} LS_OUT;

void	ls_init(LS_STATE *s, const LS_PARAMS *p, uint32_t now_ms);
void	ls_restart(LS_STATE *s, uint32_t now_ms, int32_t side);	/* e.g. after a turn */
void	ls_update(LS_STATE *s, bool left_black, bool right_black, uint32_t now_ms);
void	ls_output(const LS_STATE *s, uint32_t now_ms, int32_t base_mm_s, LS_OUT *out);

#endif /* LINE_CORE_H */
