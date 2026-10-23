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
 *	escape_mm of travel or more (the car was already steering hard and
 *	still not turning enough, as in a curve), the line went OUT past it;
 *	if it left quickly, the correction worked and the line is between
 *	the sensors.
 *
 *	Behaviour, not time: every "how long" is millimetres of travel from
 *	the odometer, and every correction is a fraction of the speed, so
 *	the car follows the same path at any speed.
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
	int32_t		lost_mm;	/* no black seen over this much travel = line lost (T_lost) */
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
	int32_t		last_black_mm;	/* odometer when black was last seen */
	bool		lost_sent;
	uint32_t	junctions;	/* count, for telemetry */
} LC_CORE;

void	lc_init(LC_CORE *c, const LC_PARAMS *p, int32_t odo_mm);
void	lc_step(LC_CORE *c, bool left_black, bool right_black, uint32_t now_ms,
		int32_t odo_mm, EVT_LIST *ev);

/* ---- the steering rule (used by Vehicle Controller) -------------------- */
/* Corrections are permille OF THE SPEED; distances are mm of travel. */
typedef struct {
	int32_t		d_min;		/* steering when a sensor first touches */
	int32_t		d_rate;		/* added per rate_mm the sensor stays on black */
	int32_t		rate_mm;
	int32_t		d_max;		/* largest correction */
	int32_t		d_bias;		/* gentle lean toward the last side while centred */
	int32_t		d_search;	/* when the line has slipped outside */
	int32_t		escape_mm;	/* a sensor on black at least this far before it went
					   white: the line slipped OUTSIDE it (a curve) */
	int32_t		drift_mm;	/* no touch for this far: the lean grows by d_bias per drift_mm */
	int32_t		lost_mm;	/* no black for this far: line lost (T_lost) */
	int32_t		slow_pct;	/* % of the speed while turning back to the line */
	int32_t		start_side;	/* side to lean toward before any black is seen */
} LS_PARAMS;

typedef struct {
	LS_PARAMS	p;
	bool		l;		/* latest status */
	bool		r;
	int32_t		since_mm;	/* odometer when the latest status began */
	int32_t		last_black_mm;
	int32_t		last_side;	/* -1 left, +1 right */
	int32_t		on_since_mm;	/* odometer when one sensor alone went black */
	int32_t		last_on_mm;	/* how far it stayed black, last time */
} LS_STATE;

typedef struct {
	int32_t		v_left;		/* mm/s */
	int32_t		v_right;
	bool		lost;		/* no black for lost_mm */
	int32_t		side;		/* side the line was last seen */
	int32_t		pos;		/* rough line position: -2 far left .. +2 far right */
} LS_OUT;

/* odo_mm: the odometer (always grows, Motion Control's snapshot) */
void	ls_init(LS_STATE *s, const LS_PARAMS *p, int32_t odo_mm);
void	ls_restart(LS_STATE *s, int32_t odo_mm, int32_t side);	/* e.g. after a turn */
void	ls_update(LS_STATE *s, bool left_black, bool right_black, int32_t odo_mm);
void	ls_output(const LS_STATE *s, int32_t odo_mm, int32_t speed_mm_s, LS_OUT *out);

/* ---- line and barcode status (one 32-bit word, for telemetry) -----------
 * Written only by the Line and Barcode task (core 1) with one 32-bit store,
 * read by Telemetry (core 0) with one 32-bit load: no torn copy.
 *   bit 0       left sensor on black
 *   bit 1       right sensor on black
 *   bits 2-7    last decoded character - 32 (' '..'_'; '-' = none)
 *   bits 8-15   good barcode reads    }
 *   bits 16-23  failed barcode reads  } stop at 255
 *   bits 24-31  junctions             }                                    */
typedef struct {
	bool		left_black;
	bool		right_black;
	char		last_char;	/* '-' if none */
	uint32_t	reads_ok;
	uint32_t	reads_failed;
	uint32_t	junctions;
} LC_STATUS;

uint32_t lc_status_pack(const LC_STATUS *s);
void	lc_status_unpack(uint32_t w, LC_STATUS *s);

#endif /* LINE_CORE_H */
