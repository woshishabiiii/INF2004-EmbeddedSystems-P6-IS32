/*
 *	vc_core.c
 *	Vehicle Controller logic (see vc_core.h).
 *
 *	Conventions
 *	  side:  -1 = left, +1 = right (as in line_core.h)
 *	  turns: + degrees = left (counter-clockwise), as in the motion API
 *	  moves: + mm = forward
 *	Every motion command that ends with EV_DONE gets a new move id; a
 *	DONE with any other id is stale and ignored.  Every scan request gets
 *	a new request id for the same reason.
 */

#include "vc_core.h"
#include "scan_core.h"			/* SCAN_IDLE / SCAN_WATCH / SCAN_PROFILE */
#include "../subsystem3/barcode_core.h"	/* LB_CMD_... */
#include "../subsystem4/imu_core.h"	/* TERRAIN_... */
#include "../subsystem4/safety_core.h"	/* FLT_NAV */
#include "../bitpack.h"			/* bp_expand16() */

#define HOLD_DEADBAND	3	/* mm/s: smaller changes are not re-sent */
#define MIN_LEG_MM	50	/* shortest bypass leg */
#define DEG_RIGHT_ANGLE	90
#define DEG_U_TURN	180
#define DEG_FULL	360
#define PI_X1000	3142
#define MS_PER_S	1000

static const char * const s_names[VC_STATES] = {
	"STARTUP", "READY", "FOLLOW", "EXEC", "SEARCH", "WAIT_ACK", "SCANNING",
	"BYPASS", "REVERSE", "FIND_LINE", "ALIGN", "SAFE_STOP", "FINISHED", "HALTED"
};

static const char * const s_nav_names[NAV_REFUSED + 1] = {
	"?", "pending", "exec", "continue", "bypass", "reverse",
	"rejoined", "search", "found", "finish", "refused"
};

const char *vc_state_name(int32_t state)
{
	return ((state >= 0) && (state < (int32_t)VC_STATES)) ? s_names[state] : "?";
}

const char *vc_nav_name(int32_t nav)
{
	return ((nav > 0) && (nav <= NAV_REFUSED)) ? s_nav_names[nav] : "?";
}

static int32_t iabs32(int32_t x)
{
	return (x < 0) ? -x : x;
}

/* degrees to turn toward a side: left = + */
static int32_t toward(int32_t side, int32_t deg)
{
	return (side < 0) ? deg : -deg;
}

/* ---- actions -------------------------------------------------------------- */
static void act_add(VC_ACTIONS *act, uint8_t type, int32_t a, int32_t b, int32_t c)
{
	if (act->n < VC_ACT_MAX) {
		act->a[act->n].type = type;
		act->a[act->n].a = a;
		act->a[act->n].b = b;
		act->a[act->n].c = c;
		act->n++;
	}
}

static void tel(VC_ACTIONS *act, uint8_t type, int32_t p1, int32_t p2)
{
	act_add(act, VA_TEL, (int32_t)type, p1, p2);
}

static int32_t new_move_id(VC_CORE *v)
{
	v->move_id = app_id_next(v->move_id);	/* 8 bits on the wire: 1..255 */
	v->expect_id = v->move_id;
	v->hold_valid = false;
	return v->move_id;
}

/* base_speed (SET base_speed) is the car's speed limit: no move, hump
   crossing or approach is ever faster, so one value slows the whole car. */
static int32_t capped(const VC_CORE *v, int32_t speed)
{
	return (speed < v->p.base_speed) ? speed : v->p.base_speed;
}

/* Step watchdog.  Each move or turn should take its wheel travel at its
   own speed; if its DONE has not come after twice that plus
   step_margin_ms, the step is stuck (Motion Control's own limit, 2 x
   expected + its margin, normally fires first as a motion fault).  The
   limit comes from the step itself, so a slower car, a longer detour or
   a bigger turn simply gets more time: nothing is a fixed duration. */
static void arm_step(VC_CORE *v, int32_t travel_mm, int32_t speed)
{
	uint32_t expected_ms = (uint32_t)((iabs32(travel_mm) * MS_PER_S) / ((speed > 0) ? speed : 1));

	v->step_deadline_ms = v->now_ms + (2U * expected_ms) + v->p.step_margin_ms;
	v->step_timed = true;
}

static void do_move(VC_CORE *v, VC_ACTIONS *act, int32_t mm, int32_t speed)
{
	int32_t id = new_move_id(v);
	int32_t s = capped(v, speed);

	act_add(act, VA_MOVE, mm, s, id);
	arm_step(v, mm, s);
	v->cmd_speed = s;
}

static void do_turn(VC_CORE *v, VC_ACTIONS *act, int32_t deg)
{
	int32_t id = new_move_id(v);

	act_add(act, VA_TURN, deg, 0, id);
	/* each wheel runs along an arc of pi x track x deg / 360 */
	arm_step(v, (iabs32(deg) * PI_X1000 * v->p.track_mm) / (DEG_FULL * 1000), v->p.turn_speed);
	v->cmd_speed = v->p.turn_speed;
}

static void do_hold(VC_CORE *v, VC_ACTIONS *act, uint32_t now, int32_t l, int32_t r)
{
	if (!v->hold_valid || (iabs32(l - v->hold_l) >= HOLD_DEADBAND) ||
	    (iabs32(r - v->hold_r) >= HOLD_DEADBAND) ||
	    ((now - v->hold_ms) >= v->p.hold_refresh_ms)) {
		act_add(act, VA_HOLD, l, r, (int32_t)v->steer_src_ms);
		v->hold_valid = true;
		v->hold_l = l;
		v->hold_r = r;
		v->hold_ms = now;
		v->expect_id = 0;		/* a speed hold replaces any move */
		v->step_timed = false;
		v->cmd_speed = (iabs32(l) > iabs32(r)) ? iabs32(l) : iabs32(r);
	}
}

static void do_stop(VC_CORE *v, VC_ACTIONS *act)
{
	act_add(act, VA_STOP, 0, 0, 0);
	v->hold_valid = false;
	v->expect_id = 0;
	v->step_timed = false;
}

/* WAIT_ACK watchdog: a stop from cmd_speed should take at most
   cmd_speed / min_decel; allow twice that, plus t_ack_ms for Motion
   Control to answer.  A faster car gets longer, nothing is fixed. */
static void arm_ack(VC_CORE *v)
{
	uint32_t stop_ms = (uint32_t)((v->cmd_speed * MS_PER_S) / ((v->p.min_decel > 0) ? v->p.min_decel : 1));

	v->ack_deadline_ms = v->now_ms + (2U * stop_ms) + v->p.t_ack_ms;
}

static void do_go(VC_ACTIONS *act)
{
	act_add(act, VA_CLEAR_STOP, 0, 0, 0);
}

/* A WATCH normally carries SCAN_STOP_FLAG: Obstacle Scan then stops the
   car itself on a confirmed close range.  Not while the car is driving
   past an obstacle it has decided to pass (Continue, alert_mute_odo); the
   flag is set again once it is past (vc_tick). */
static void scan_set(VC_CORE *v, VC_ACTIONS *act, uint32_t mode, int32_t stop_mm)
{
	int32_t		id;
	uint32_t	flags = 0U;

	v->req_id = app_id_next(v->req_id);	/* 8 bits on the wire: 1..255 */
	id = v->req_id;
	v->watch_stops = false;
	if (mode == SCAN_WATCH) {
		v->watch_id = id;
		if (v->odo_mm >= v->alert_mute_odo) {
			v->watch_stops = true;
			flags = SCAN_STOP_FLAG;
		}
	} else if (mode == SCAN_PROFILE) {
		v->scan_id = id;
		v->watch_id = 0;
	} else {
		v->watch_id = 0;	/* idle: every ALERT still queued is stale */
	}
	v->zone_near = false;
	act_add(act, VA_SCAN, (int32_t)(mode | flags), stop_mm, id);
}

static void enter(VC_CORE *v, VC_ACTIONS *act, VC_STATE s, uint32_t now, int32_t why)
{
	v->state = s;
	v->entered_ms = now;
	v->step = 0;
	tel(act, (uint8_t)EV_STATE, (int32_t)s, why);
}

static bool running(const VC_CORE *v)
{
	return (v->state >= VC_FOLLOW) && (v->state <= VC_ALIGN);
}

static bool on_hump(const VC_CORE *v)
{
	return (v->terrain != TERRAIN_LEVEL) || ((v->odo_mm - v->hump_until_odo) < 0);
}

/* ---- Safe Stop (safety fault) and Halted (mission failure) ---------------- */
static void safe_stop(VC_CORE *v, VC_ACTIONS *act, uint32_t now, int32_t why)
{
	do_stop(v, act);
	scan_set(v, act, SCAN_IDLE, 0);
	if ((why != VC_WHY_FAULT) && (why != VC_WHY_IMPACT)) {
		act_add(act, VA_FAULT, (int32_t)FLT_NAV, 0, 0);	/* Safety Monitor latches it */
	}
	v->why = why;
	v->pending = 0;
	enter(v, act, VC_SAFE_STOP, now, why);
}

/* A mission failure is not a fault: stop with CMD_STOP, report why, and
   let START carry on -- nothing to latch or RESTART (feedback IS32). */
static void mission_stop(VC_CORE *v, VC_ACTIONS *act, uint32_t now, int32_t why)
{
	do_stop(v, act);
	scan_set(v, act, SCAN_IDLE, 0);
	v->why = why;
	v->pending = 0;
	enter(v, act, VC_HALTED, now, why);
}

/* ---- Follow Line ---------------------------------------------------------- */
static void start_search(VC_CORE *v, VC_ACTIONS *act, uint32_t now, int32_t side);

static void follow_steer(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	LS_OUT	o;
	int32_t	speed = v->p.base_speed;

	if (on_hump(v)) {
		if (v->ls.l && v->ls.r) {
			/* both black on a hump: the IR sensors may have lifted
			   off the floor (no reflection reads black), so this is
			   not a line reading: hold the heading for a moment */
			do_hold(v, act, now, capped(v, v->p.hump_speed), capped(v, v->p.hump_speed));
			return;
		}
		/* a sensor that sees WHITE is close to the floor, so the
		   reading is real: keep following the line, at hump speed.
		   (A pitch reading alone must never blind the car: a curve
		   taken "straight on the encoders" leaves the line.) */
		if (v->p.hump_speed < speed) {
			speed = v->p.hump_speed;
		}
	}
	if (v->acquiring && (v->ls.l || v->ls.r)) {
		v->acquiring = false;		/* the line is under the sensors again */
	}
	if (v->acquiring && (v->odo_mm >= v->acquire_odo)) {
		/* after a turn or a bypass the line should appear within a few
		   centimetres; if not, the car is pointing past it: look now,
		   rather than after T_lost */
		v->acquiring = false;
		start_search(v, act, now, v->ls.last_side);
		return;
	}
	if (v->zone_near && (v->p.slow_speed < speed)) {
		speed = v->p.slow_speed;
	}
	if ((v->pending != 0) && (v->p.junction_speed < speed)) {
		speed = v->p.junction_speed;
	}
	ls_output(&v->ls, v->odo_mm, speed, &o);
	if (o.lost) {
		start_search(v, act, now, o.side);
	} else {
		do_hold(v, act, now, o.v_left, o.v_right);
	}
}

/* side:    where the line most likely is if neither sensor sees it (the
	    steering leans that way, and a search starts that way);
   acquire: the car has just turned or left the line, so the line must
	    show up within acquire_mm or the car searches at once.  Not after
	    START or a Continue: the car is still where the line put it. */
static void resume_follow(VC_CORE *v, VC_ACTIONS *act, uint32_t now, int32_t side, bool acquire)
{
	ls_restart(&v->ls, v->odo_mm, side);
	v->junction_mute_odo = v->odo_mm + v->p.junction_mute_mm;
	v->acquiring = acquire && !(v->ls.l || v->ls.r);
	v->acquire_odo = v->odo_mm + v->p.acquire_mm;
	scan_set(v, act, SCAN_WATCH, v->p.stop_mm);
	enter(v, act, VC_FOLLOW, now, VC_WHY_NONE);
	follow_steer(v, act, now);
}

static void start_run(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	v->retries = 0U;
	v->pending = 0;
	v->commands_done = 0U;
	v->obstacles_passed = 0U;
	v->run_start_ms = now;
	v->run_start_odo = v->odo_mm;
	v->alert_mute_odo = v->odo_mm;
	v->hump_until_odo = v->odo_mm;
	v->why = VC_WHY_NONE;
	do_go(act);
	resume_follow(v, act, now, v->ls.p.start_side, false);
}

/* START after HALTED: carry on with the same run (its counters and the
   run summary continue).  The operator has put the car back on the line. */
static void resume_run(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	v->retries = 0U;
	v->pending = 0;
	v->alert_mute_odo = v->odo_mm;
	v->hump_until_odo = v->odo_mm;
	v->why = VC_WHY_NONE;
	do_go(act);
	resume_follow(v, act, now, v->ls.last_side, false);
}

/* ---- Line Search ---------------------------------------------------------- */
/* Line search widths, from the parameters (at least one sweep each side,
   never wider than 180 degrees: the last spin is 2 x the widest) */
static int32_t search_step(const VC_CORE *v)
{
	return (v->p.search_step_deg > 0) ? v->p.search_step_deg : DEG_RIGHT_ANGLE;
}

static int32_t search_spins(const VC_CORE *v)
{
	int32_t max = (v->p.search_max_deg < DEG_U_TURN) ? v->p.search_max_deg : DEG_U_TURN;
	int32_t n = max / search_step(v);

	return 2 * ((n > 0) ? n : 1);
}

/* how far spin k ends from where the search began: step, step, 2 step, 2 step, ... */
static int32_t search_amp(const VC_CORE *v, int32_t k)
{
	int32_t a = search_step(v) * ((k / 2) + 1);

	return (a < DEG_U_TURN) ? a : DEG_U_TURN;
}

static void start_search(VC_CORE *v, VC_ACTIONS *act, uint32_t now, int32_t side)
{
	v->search_side = (side < 0) ? -1 : 1;
	tel(act, (uint8_t)EV_NAV, NAV_SEARCH, v->search_side);
	scan_set(v, act, SCAN_IDLE, 0);		/* no ALERTs while spinning */
	enter(v, act, VC_SEARCH, now, VC_WHY_NONE);
	do_turn(v, act, toward(v->search_side, search_amp(v, 0)));	/* that side first */
}

/* Spin k ends search_amp(k) degrees to one side of where the search began:
   even spins on the side the line was last seen, odd ones on the other. */
static void search_done(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	int32_t k = v->step + 1;

	if (k >= search_spins(v)) {
		mission_stop(v, act, now, VC_WHY_SEARCH_FAILED);
	} else {
		v->step = k;
		do_turn(v, act, toward(((k % 2) == 0) ? v->search_side : -v->search_side,
				       search_amp(v, k - 1) + search_amp(v, k)));
	}
}

/* ---- Execute a barcode command ------------------------------------------- */
static int32_t exec_angle(int32_t cmd)
{
	switch (cmd) {
	case LB_CMD_TURN_LEFT:
		return DEG_RIGHT_ANGLE;
	case LB_CMD_TURN_RIGHT:
		return -DEG_RIGHT_ANGLE;
	case LB_CMD_U_TURN:
		return DEG_U_TURN;
	default:
		return 0;
	}
}

static void start_exec(VC_CORE *v, VC_ACTIONS *act, uint32_t now, bool at_junction)
{
	v->exec_cmd = v->pending;
	v->pending = 0;
	tel(act, (uint8_t)EV_NAV, NAV_EXEC, v->exec_cmd);
	enter(v, act, VC_EXEC, now, VC_WHY_NONE);

	if (v->exec_cmd == LB_CMD_STRAIGHT) {
		/* drive across the junction band, then follow on */
		v->step = 1;
		do_move(v, act, at_junction ? v->p.cross_mm : MIN_LEG_MM, v->p.junction_speed);
	} else if (at_junction && (v->exec_cmd != LB_CMD_U_TURN)) {
		/* put the wheel axle over the middle of the junction first */
		do_move(v, act, v->p.line_to_axle_mm + v->p.line_half_mm, v->p.junction_speed);
	} else {
		/* U-turn: turn round where we are, just before the junction,
		   so the sensors come back onto the line we arrived on and
		   cannot pick up a side branch; or no junction came at all */
		v->step = 1;
		scan_set(v, act, SCAN_IDLE, 0);
		do_turn(v, act, exec_angle(v->exec_cmd));
	}
}

static void exec_done(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	if (v->step == 0) {
		v->step = 1;
		scan_set(v, act, SCAN_IDLE, 0);
		do_turn(v, act, exec_angle(v->exec_cmd));
	} else {
		int32_t angle = exec_angle(v->exec_cmd);
		int32_t side = v->ls.last_side;		/* straight on: as before */

		/* After a turn the line is most likely further round in the
		   same direction: spins slip, so a car turns a little short */
		if (angle != 0) {
			side = (angle > 0) ? -1 : 1;
		}
		v->commands_done++;
		v->last_cmd = v->exec_cmd;
		resume_follow(v, act, now, side, true);
	}
}

/* ---- Obstacles -------------------------------------------------------------- */
static void obstacle_stop(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	do_stop(v, act);		/* Obstacle Scan has usually set it already */
	scan_set(v, act, SCAN_IDLE, 0);
	v->pending = 0;
	v->rescanned = false;
	arm_ack(v);
	enter(v, act, VC_WAIT_ACK, now, VC_WHY_NONE);
}

static void retry_or_stop(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	if (v->retries < v->p.n_retry) {
		v->retries++;
		tel(act, (uint8_t)EV_NAV, NAV_REVERSE, (int32_t)v->retries);
		do_go(act);
		enter(v, act, VC_REVERSE, now, VC_WHY_NONE);
		do_move(v, act, -v->p.back_mm, v->p.move_speed);
	} else {
		mission_stop(v, act, now, VC_WHY_NO_ROUTE);
	}
}

static void plan_bypass(VC_CORE *v, int32_t side, int32_t shift, const OS_PROFILE *prof)
{
	int32_t sin_t = os_sin1000(v->p.bypass_deg);
	int32_t cos_t = os_cos1000(v->p.bypass_deg);
	int32_t d1 = (shift * 1000) / sin_t;			/* sideways leg */
	int32_t ahead = (d1 * cos_t) / 1000;			/* forward gained on it */
	int32_t needed = prof->closest_mm + v->p.us_to_axle_mm +
			 v->p.obst_depth_mm + v->p.rear_clear_mm;	/* axle past the far side */
	int32_t d2 = needed - ahead;

	if (d1 < MIN_LEG_MM) {
		d1 = MIN_LEG_MM;
	}
	if (d2 < MIN_LEG_MM) {
		d2 = MIN_LEG_MM;
	}
	v->side = side;
	v->shift_mm = shift;
	v->plan[0].turn_deg = toward(side, v->p.bypass_deg);	/* out */
	v->plan[0].drive_mm = 0;
	v->plan[1].turn_deg = 0;
	v->plan[1].drive_mm = d1;
	v->plan[2].turn_deg = toward(-side, v->p.bypass_deg);	/* parallel to the line */
	v->plan[2].drive_mm = 0;
	v->plan[3].turn_deg = 0;
	v->plan[3].drive_mm = d2;				/* past the obstacle */
	v->plan[4].turn_deg = toward(-side, v->p.bypass_deg);	/* back toward the line */
	v->plan[4].drive_mm = 0;
	v->plan_n = 5U;
	v->plan_i = 0U;
}

static void start_find_line(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	int32_t dist = ((v->shift_mm * 1000) / os_sin1000(v->p.bypass_deg)) + v->p.find_extra_mm;

	scan_set(v, act, SCAN_WATCH, v->p.bypass_stop_mm);
	enter(v, act, VC_FIND_LINE, now, VC_WHY_NONE);
	do_move(v, act, dist, v->p.find_speed);
}

static void next_step(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	const VC_STEP *st;

	if (v->plan_i >= v->plan_n) {
		start_find_line(v, act, now);
		return;
	}
	st = &v->plan[v->plan_i];
	v->plan_i++;
	if (st->turn_deg != 0) {
		scan_set(v, act, SCAN_IDLE, 0);
		do_turn(v, act, st->turn_deg);
	} else {
		scan_set(v, act, SCAN_WATCH, v->p.bypass_stop_mm);
		do_move(v, act, st->drive_mm, v->p.move_speed);
	}
}

/* Continue, Bypass or Reverse and Retry (Figure 7, DECIDE ROUTE) */
static void decide(VC_CORE *v, const OS_PROFILE *prof, VC_ACTIONS *act, uint32_t now)
{
	int32_t	corridor = (v->p.car_width_mm / 2) + v->p.margin_mm;
	int32_t	need = v->p.car_width_mm + v->p.margin_mm;
	int32_t	left_shift, right_shift, side, shift;
	bool	left_ok, right_ok;

	if (!prof->valid) {
		retry_or_stop(v, act, now);	/* saw nothing clearly: back off, look again */
		return;
	}
	if ((prof->x_right_mm < -corridor) || (prof->x_left_mm > corridor)) {
		/* the obstacle is beside the path, not on it */
		tel(act, (uint8_t)EV_NAV, NAV_CONTINUE, prof->closest_mm);
		v->alert_mute_odo = v->odo_mm + prof->closest_mm + v->p.us_to_axle_mm +
				    v->p.obst_depth_mm + v->p.rear_clear_mm;
		v->retries = 0U;
		do_go(act);
		resume_follow(v, act, now, v->ls.last_side, false);
		return;
	}
	left_shift = corridor - prof->x_left_mm;	/* move left until clear of the left edge */
	right_shift = corridor + prof->x_right_mm;
	left_ok = (prof->left_clear_mm >= need);
	right_ok = (prof->right_clear_mm >= need);

	if (left_ok && right_ok) {
		/* the smaller detour; if they are about equal, the side with
		   fewer missing echoes (less unknown), then the preferred side */
		if ((left_shift + v->p.tie_mm) < right_shift) {
			side = -1;
		} else if ((right_shift + v->p.tie_mm) < left_shift) {
			side = 1;
		} else if (prof->unknown_left != prof->unknown_right) {
			side = (prof->unknown_left < prof->unknown_right) ? -1 : 1;
		} else {
			side = (v->p.pref_side < 0) ? -1 : 1;
		}
	} else if (left_ok) {
		side = -1;
	} else if (right_ok) {
		side = 1;
	} else {
		retry_or_stop(v, act, now);
		return;
	}
	shift = (side < 0) ? left_shift : right_shift;
	plan_bypass(v, side, shift, prof);
	tel(act, (uint8_t)EV_NAV, NAV_BYPASS, side * shift);
	do_go(act);
	enter(v, act, VC_BYPASS, now, VC_WHY_NONE);
	next_step(v, act, now);
}

static void align_done(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	if (v->step == 0) {
		v->step = 1;
		scan_set(v, act, SCAN_IDLE, 0);
		do_turn(v, act, toward(v->side, v->p.bypass_deg));	/* back to the line's heading */
	} else {
		v->obstacles_passed++;
		v->retries = 0U;
		tel(act, (uint8_t)EV_NAV, NAV_REJOINED, (int32_t)v->obstacles_passed);
		resume_follow(v, act, now, -v->side, true);	/* we came from v->side */
	}
}

/* ---- inputs ----------------------------------------------------------------- */
static void command(VC_CORE *v, int32_t cmd, VC_ACTIONS *act, uint32_t now)
{
	switch (cmd) {
	case CMD_START:
		if ((v->state == VC_HALTED) && !v->faulted) {
			resume_run(v, act, now);
		} else if (((v->state == VC_READY) || (v->state == VC_FINISHED)) && !v->faulted) {
			start_run(v, act, now);
		} else {
			tel(act, (uint8_t)EV_NAV, NAV_REFUSED, cmd);
		}
		break;
	case CMD_STOP:
		if (running(v)) {
			do_stop(v, act);
			scan_set(v, act, SCAN_IDLE, 0);
			v->pending = 0;
			enter(v, act, VC_READY, now, VC_WHY_NONE);
		}
		break;
	case CMD_RESTART:
		if (running(v)) {
			do_stop(v, act);
			scan_set(v, act, SCAN_IDLE, 0);
			enter(v, act, VC_READY, now, VC_WHY_NONE);
		}
		if (v->faulted) {
			act_add(act, VA_RESTART, 0, 0, 0);	/* EV_FAULT_CLEAR follows */
		} else if ((v->state == VC_SAFE_STOP) || (v->state == VC_HALTED)) {
			enter(v, act, VC_READY, now, VC_WHY_NONE);
		} else {
			/* nothing to clear */
		}
		break;
	case CMD_FINISH:
		if (running(v) || (v->state == VC_READY) || (v->state == VC_HALTED)) {
			do_stop(v, act);
			scan_set(v, act, SCAN_IDLE, 0);
			tel(act, (uint8_t)EV_NAV, NAV_FINISH, v->odo_mm - v->run_start_odo);
			enter(v, act, VC_FINISHED, now, VC_WHY_NONE);
		}
		break;
	default:
		break;
	}
}

static void on_line_status(VC_CORE *v, const APP_EVT *m, VC_ACTIONS *act, uint32_t now)
{
	bool black = (m->p1 != 0) || (m->p2 != 0);

	ls_update(&v->ls, m->p1 != 0, m->p2 != 0, v->odo_mm);
	switch (v->state) {
	case VC_FOLLOW:
		/* latency log: this reading -> the hold.  The event carries the
		   low 16 bits of its time; it is a few ms old at most. */
		v->steer_src_ms = bp_expand16(now, m->t_ms);
		follow_steer(v, act, now);
		v->steer_src_ms = 0U;
		break;
	case VC_SEARCH:
		if (black) {
			tel(act, (uint8_t)EV_NAV, NAV_FOUND, 0);
			resume_follow(v, act, now, v->ls.last_side, false);	/* the hold replaces the turn */
		}
		break;
	case VC_FIND_LINE:
		if (black) {
			enter(v, act, VC_ALIGN, now, VC_WHY_NONE);
			do_move(v, act, v->p.line_to_axle_mm + v->p.align_extra_mm, v->p.find_speed);
		}
		break;
	default:
		break;				/* other states drive by distance */
	}
}

static void on_done(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	switch (v->state) {
	case VC_EXEC:
		exec_done(v, act, now);
		break;
	case VC_SEARCH:
		search_done(v, act, now);
		break;
	case VC_REVERSE:
		do_stop(v, act);		/* stop, then scan again */
		arm_ack(v);
	enter(v, act, VC_WAIT_ACK, now, VC_WHY_NONE);
		break;
	case VC_BYPASS:
		next_step(v, act, now);
		break;
	case VC_FIND_LINE:
		start_search(v, act, now, -v->side);	/* the line is back on the other side */
		break;
	case VC_ALIGN:
		align_done(v, act, now);
		break;
	default:
		break;
	}
}

static void on_alert(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	switch (v->state) {
	case VC_FOLLOW:
	case VC_EXEC:
		obstacle_stop(v, act, now);
		break;
	case VC_BYPASS:
	case VC_FIND_LINE:
	case VC_ALIGN:
		mission_stop(v, act, now, VC_WHY_BLOCKED);	/* already stopped by Obstacle Scan */
		break;
	default:
		break;
	}
}

void vc_init(VC_CORE *v, const VC_PARAMS *p, const LS_PARAMS *ls, uint32_t now_ms)
{
	*v = (VC_CORE){0};
	v->p = *p;
	v->state = VC_STARTUP;
	v->entered_ms = now_ms;
	v->now_ms = now_ms;
	v->terrain = TERRAIN_LEVEL;
	v->hump_until_odo = 0;
	ls_init(&v->ls, ls, 0);		/* the odometer starts at 0 */
}

void vc_tune(VC_CORE *v, int32_t base_speed, int32_t stop_mm)
{
	if (base_speed > 0) {
		v->p.base_speed = base_speed;
	}
	if (stop_mm > 0) {
		v->p.stop_mm = stop_mm;
	}
}

void vc_event(VC_CORE *v, const APP_EVT *m, uint32_t now_ms, int32_t odo_mm, VC_ACTIONS *act)
{
	act->n = 0U;
	v->now_ms = now_ms;
	v->odo_mm = odo_mm;

	switch (m->type) {
	case EV_FAULT:
		v->faulted = true;
		v->fault_bits = m->p1;
		tel(act, m->type, m->p1, m->p2);
		if (v->state != VC_SAFE_STOP) {
			safe_stop(v, act, now_ms, VC_WHY_FAULT);
		}
		break;
	case EV_FAULT_CLEAR:
		v->faulted = false;
		v->fault_bits = 0;
		tel(act, m->type, 0, 0);
		if (v->state == VC_SAFE_STOP) {
			enter(v, act, VC_READY, now_ms, VC_WHY_NONE);	/* CMD_STOP stays set */
		}
		break;
	case EV_IMPACT:
		if (running(v)) {
			safe_stop(v, act, now_ms, VC_WHY_IMPACT);
		}
		break;
	case EV_COMMAND:
		command(v, m->p1, act, now_ms);
		break;
	case EV_BUTTON:
		command(v, (m->p1 == BTN_START) ? CMD_START : CMD_STOP, act, now_ms);
		break;
	case EV_TERRAIN:
		v->terrain = m->p1;
		if (m->p1 == TERRAIN_LEVEL) {
			v->hump_until_odo = odo_mm + v->p.hump_grace_mm;
			ls_restart(&v->ls, odo_mm, v->ls.last_side);	/* T_lost starts again */
		}
		break;
	case EV_LINE_STATUS:
		on_line_status(v, m, act, now_ms);
		break;
	case EV_JUNCTION:
		tel(act, m->type, m->p1, m->p2);
		if ((v->state == VC_FOLLOW) && !on_hump(v) && (odo_mm >= v->junction_mute_odo)) {
			if (v->pending != 0) {
				start_exec(v, act, now_ms, true);
			} else {
				v->junction_mute_odo = odo_mm + v->p.junction_mute_mm;	/* straight on */
			}
		}
		break;
	case EV_BARCODE:
		tel(act, m->type, m->p1, m->p2);
		if ((v->state == VC_FOLLOW) && !on_hump(v) && (m->p2 != LB_CMD_NONE)) {
			v->pending = m->p2;
			v->pending_odo = odo_mm;
			tel(act, (uint8_t)EV_NAV, NAV_PENDING, m->p2);
		}
		break;
	case EV_LINE_LOST:
	case EV_BARCODE_BAD:
		tel(act, m->type, m->p1, m->p2);	/* for the record; steering decides */
		break;
	case EV_ALERT:
		if ((v->watch_id != 0) && (m->p2 == v->watch_id) && (odo_mm >= v->alert_mute_odo)) {
			on_alert(v, act, now_ms);
		}
		break;
	case EV_RANGE_ZONE:
		if ((v->watch_id != 0) && (m->p2 == v->watch_id)) {
			v->zone_near = (m->p1 != 0);
		}
		break;
	case EV_DONE:
		if ((v->expect_id != 0) && (m->p2 == v->expect_id)) {
			v->expect_id = 0;
			v->step_timed = false;
			on_done(v, act, now_ms);
		}
		break;
	case EV_STOP_ACK:
		if (v->state == VC_WAIT_ACK) {
			scan_set(v, act, SCAN_PROFILE, 0);
			enter(v, act, VC_SCANNING, now_ms, VC_WHY_NONE);
		}
		break;
	default:
		break;
	}
}

void vc_profile(VC_CORE *v, const APP_EVT *m, const OS_PROFILE *prof, uint32_t now_ms,
		int32_t odo_mm, VC_ACTIONS *act)
{
	act->n = 0U;
	v->now_ms = now_ms;
	v->odo_mm = odo_mm;
	if ((v->state == VC_SCANNING) && (m->p2 == v->scan_id)) {
		decide(v, prof, act, now_ms);
	}
}

void vc_tick(VC_CORE *v, uint32_t now_ms, int32_t odo_mm, bool sensors_ready, VC_ACTIONS *act)
{
	uint32_t age = now_ms - v->entered_ms;
	bool	 overdue = v->step_timed && ((int32_t)(now_ms - v->step_deadline_ms) > 0);

	act->n = 0U;
	v->now_ms = now_ms;
	v->odo_mm = odo_mm;
	v->sensors_ready = sensors_ready;

	switch (v->state) {
	case VC_STARTUP:
		if (sensors_ready || (age >= v->p.t_startup_ms)) {
			enter(v, act, VC_READY, now_ms, VC_WHY_NONE);
		}
		break;
	case VC_FOLLOW:
		if ((v->watch_id != 0) && !v->watch_stops && (odo_mm >= v->alert_mute_odo)) {
			scan_set(v, act, SCAN_WATCH, v->p.stop_mm);	/* past it: stop on ALERT again */
		}
		if ((v->pending != 0) && (v->p.d_exec_mm > 0) &&
		    ((odo_mm - v->pending_odo) >= v->p.d_exec_mm) && !on_hump(v)) {
			start_exec(v, act, now_ms, false);	/* no junction came */
		} else {
			follow_steer(v, act, now_ms);
		}
		break;
	case VC_EXEC:
		if (overdue) {
			mission_stop(v, act, now_ms, VC_WHY_EXEC_TIMEOUT);
		}
		break;
	case VC_SEARCH:
		if (overdue) {
			mission_stop(v, act, now_ms, VC_WHY_SEARCH_FAILED);
		}
		break;
	case VC_WAIT_ACK:
		if ((int32_t)(now_ms - v->ack_deadline_ms) > 0) {
			safe_stop(v, act, now_ms, VC_WHY_ACK_TIMEOUT);
		}
		break;
	case VC_SCANNING:
		if (age > v->p.t_scan_ms) {
			if (!v->rescanned) {
				v->rescanned = true;		/* retryable: scan once more */
				scan_set(v, act, SCAN_PROFILE, 0);
				enter(v, act, VC_SCANNING, now_ms, VC_WHY_NONE);
			} else {
				mission_stop(v, act, now_ms, VC_WHY_SCAN_FAILED);
			}
		}
		break;
	case VC_BYPASS:
	case VC_REVERSE:
	case VC_FIND_LINE:
	case VC_ALIGN:
		if (overdue) {
			mission_stop(v, act, now_ms, VC_WHY_BYPASS_TIMEOUT);
		}
		break;
	default:
		break;				/* READY, SAFE_STOP, FINISHED, HALTED: wait for commands */
	}
}

bool vc_is_running(int32_t state)
{
	return (state >= (int32_t)VC_FOLLOW) && (state <= (int32_t)VC_ALIGN);
}

/* ---- packed status (vc_core.h) -------------------------------------------- */
BP_STATIC_ASSERT(VC_STATES <= 16, vc_state_fits_4_bits);

void vc_st_pack(const VC_ST *s, VC_ST_PK *pk)
{
	pk->w[0] = bp_sat_u(s->state, 4U) |
		   (bp_sat_u(s->why, 4U) << 4) |
		   (bp_sat_u(s->pending, 3U) << 8) |
		   (bp_sat_u((int32_t)s->vq_high, 6U) << 11) |
		   (bp_sat_u((int32_t)s->commands_done, 8U) << 17) |
		   (bp_sat_u((int32_t)s->obstacles_passed, 7U) << 25);
	pk->w[1] = s->run_start_ms;
	pk->w[2] = bp_sat_u(s->run_start_odo, 24U);
}

void vc_st_unpack(const VC_ST_PK *pk, VC_ST *s)
{
	uint32_t w0 = pk->w[0];

	s->state = (int32_t)(w0 & BP_MAX(4U));
	s->why = (int32_t)((w0 >> 4) & BP_MAX(4U));
	s->pending = (int32_t)((w0 >> 8) & BP_MAX(3U));
	s->vq_high = (w0 >> 11) & BP_MAX(6U);
	s->commands_done = (w0 >> 17) & BP_MAX(8U);
	s->obstacles_passed = w0 >> 25;
	s->run_start_ms = pk->w[1];
	s->run_start_odo = (int32_t)(pk->w[2] & BP_MAX(24U));
}
