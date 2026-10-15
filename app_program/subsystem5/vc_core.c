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

#define HOLD_DEADBAND	3	/* mm/s: smaller changes are not re-sent */
#define MIN_LEG_MM	50	/* shortest bypass leg */
#define DEG_RIGHT_ANGLE	90
#define DEG_U_TURN	180
#define DEG_SEARCH	120	/* line search: this far to one side, then twice as far back */

static const char * const s_names[VC_STATES] = {
	"STARTUP", "READY", "FOLLOW", "EXEC", "SEARCH", "WAIT_ACK", "SCANNING",
	"BYPASS", "REVERSE", "FIND_LINE", "ALIGN", "SAFE_STOP", "FINISHED"
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
	v->move_id = (v->move_id >= INT32_MAX) ? 1 : (v->move_id + 1);
	v->expect_id = v->move_id;
	v->hold_valid = false;
	return v->move_id;
}

static void do_move(VC_CORE *v, VC_ACTIONS *act, int32_t mm, int32_t speed)
{
	int32_t id = new_move_id(v);

	act_add(act, VA_MOVE, mm, speed, id);
}

static void do_turn(VC_CORE *v, VC_ACTIONS *act, int32_t deg)
{
	int32_t id = new_move_id(v);

	act_add(act, VA_TURN, deg, 0, id);
}

static void do_hold(VC_CORE *v, VC_ACTIONS *act, uint32_t now, int32_t l, int32_t r)
{
	if (!v->hold_valid || (iabs32(l - v->hold_l) >= HOLD_DEADBAND) ||
	    (iabs32(r - v->hold_r) >= HOLD_DEADBAND) ||
	    ((now - v->hold_ms) >= v->p.hold_refresh_ms)) {
		act_add(act, VA_HOLD, l, r, 0);
		v->hold_valid = true;
		v->hold_l = l;
		v->hold_r = r;
		v->hold_ms = now;
		v->expect_id = 0;		/* a speed hold replaces any move */
	}
}

static void do_stop(VC_CORE *v, VC_ACTIONS *act)
{
	act_add(act, VA_STOP, 0, 0, 0);
	v->hold_valid = false;
	v->expect_id = 0;
}

static void do_go(VC_ACTIONS *act)
{
	act_add(act, VA_CLEAR_STOP, 0, 0, 0);
}

static void scan_set(VC_CORE *v, VC_ACTIONS *act, uint32_t mode, int32_t stop_mm)
{
	int32_t id = ((v->watch_id > v->scan_id) ? v->watch_id : v->scan_id) + 1;

	if (mode == SCAN_WATCH) {
		v->watch_id = id;
	} else if (mode == SCAN_PROFILE) {
		v->scan_id = id;
		v->watch_id = 0;
	} else {
		v->watch_id = 0;	/* idle: every ALERT still queued is stale */
	}
	v->zone_near = false;
	act_add(act, VA_SCAN, (int32_t)mode, stop_mm, id);
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

static bool on_hump(const VC_CORE *v, uint32_t now)
{
	return (v->terrain != TERRAIN_LEVEL) || ((int32_t)(v->hump_until_ms - now) > 0);
}

/* ---- Safe Stop ------------------------------------------------------------ */
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

/* ---- Follow Line ---------------------------------------------------------- */
static void start_search(VC_CORE *v, VC_ACTIONS *act, uint32_t now, int32_t side);

static void follow_steer(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	LS_OUT	o;
	int32_t	speed = v->p.base_speed;

	if (on_hump(v, now)) {
		/* the IR sensors lift off the floor on a hump: hold the
		   heading on the encoders and ignore the line for a moment */
		do_hold(v, act, now, v->p.hump_speed, v->p.hump_speed);
		return;
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
	ls_output(&v->ls, now, speed, &o);
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
	ls_restart(&v->ls, now, side);
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
	v->hump_until_ms = now;
	v->why = VC_WHY_NONE;
	do_go(act);
	resume_follow(v, act, now, v->ls.p.start_side, false);
}

/* ---- Line Search ---------------------------------------------------------- */
static void start_search(VC_CORE *v, VC_ACTIONS *act, uint32_t now, int32_t side)
{
	v->search_side = (side < 0) ? -1 : 1;
	v->episode_ms = now;
	tel(act, (uint8_t)EV_NAV, NAV_SEARCH, v->search_side);
	scan_set(v, act, SCAN_IDLE, 0);		/* no ALERTs while spinning */
	enter(v, act, VC_SEARCH, now, VC_WHY_NONE);
	do_turn(v, act, toward(v->search_side, DEG_SEARCH));	/* that side first */
}

static void search_done(VC_CORE *v, VC_ACTIONS *act, uint32_t now)
{
	if (v->step == 0) {
		v->step = 1;			/* sweep across to the other side */
		do_turn(v, act, toward(-v->search_side, 2 * DEG_SEARCH));
	} else if (v->step == 1) {
		v->step = 2;			/* back to where the search began */
		do_turn(v, act, toward(v->search_side, DEG_SEARCH));
	} else {
		safe_stop(v, act, now, VC_WHY_SEARCH_FAILED);
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
	v->episode_ms = now;
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
	do_stop(v, act);
	scan_set(v, act, SCAN_IDLE, 0);
	v->pending = 0;
	v->episode_ms = now;
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
		safe_stop(v, act, now, VC_WHY_NO_ROUTE);
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
		if (((v->state == VC_READY) || (v->state == VC_FINISHED)) && !v->faulted) {
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
		} else if (v->state == VC_SAFE_STOP) {
			enter(v, act, VC_READY, now, VC_WHY_NONE);
		} else {
			/* nothing to clear */
		}
		break;
	case CMD_FINISH:
		if (running(v) || (v->state == VC_READY)) {
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

static void on_line_status(VC_CORE *v, const APP_MSG *m, VC_ACTIONS *act, uint32_t now)
{
	bool black = (m->p1 != 0) || (m->p2 != 0);

	ls_update(&v->ls, m->p1 != 0, m->p2 != 0, now);
	switch (v->state) {
	case VC_FOLLOW:
		follow_steer(v, act, now);
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
		safe_stop(v, act, now, VC_WHY_BLOCKED);
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
	v->terrain = TERRAIN_LEVEL;
	v->hump_until_ms = now_ms;
	ls_init(&v->ls, ls, now_ms);
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

void vc_event(VC_CORE *v, const APP_MSG *m, uint32_t now_ms, int32_t odo_mm, VC_ACTIONS *act)
{
	act->n = 0U;
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
			v->hump_until_ms = now_ms + v->p.hump_grace_ms;
			ls_restart(&v->ls, now_ms, v->ls.last_side);	/* T_lost starts again */
		}
		break;
	case EV_LINE_STATUS:
		on_line_status(v, m, act, now_ms);
		break;
	case EV_JUNCTION:
		tel(act, m->type, m->p1, m->p2);
		if ((v->state == VC_FOLLOW) && !on_hump(v, now_ms) && (odo_mm >= v->junction_mute_odo)) {
			if (v->pending != 0) {
				start_exec(v, act, now_ms, true);
			} else {
				v->junction_mute_odo = odo_mm + v->p.junction_mute_mm;	/* straight on */
			}
		}
		break;
	case EV_BARCODE:
		tel(act, m->type, m->p1, m->p2);
		if ((v->state == VC_FOLLOW) && !on_hump(v, now_ms) && (m->p2 != LB_CMD_NONE)) {
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

void vc_profile(VC_CORE *v, const APP_MSG *m, const OS_PROFILE *prof, uint32_t now_ms,
		int32_t odo_mm, VC_ACTIONS *act)
{
	act->n = 0U;
	v->odo_mm = odo_mm;
	if ((v->state == VC_SCANNING) && (m->p2 == v->scan_id)) {
		decide(v, prof, act, now_ms);
	}
}

void vc_tick(VC_CORE *v, uint32_t now_ms, int32_t odo_mm, bool sensors_ready, VC_ACTIONS *act)
{
	uint32_t age = now_ms - v->entered_ms;
	uint32_t episode = now_ms - v->episode_ms;

	act->n = 0U;
	v->odo_mm = odo_mm;
	v->sensors_ready = sensors_ready;

	switch (v->state) {
	case VC_STARTUP:
		if (sensors_ready || (age >= v->p.t_startup_ms)) {
			enter(v, act, VC_READY, now_ms, VC_WHY_NONE);
		}
		break;
	case VC_FOLLOW:
		if ((v->pending != 0) && (v->p.d_exec_mm > 0) &&
		    ((odo_mm - v->pending_odo) >= v->p.d_exec_mm) && !on_hump(v, now_ms)) {
			start_exec(v, act, now_ms, false);	/* no junction came */
		} else {
			follow_steer(v, act, now_ms);
		}
		break;
	case VC_EXEC:
		if (episode > v->p.t_exec_ms) {
			safe_stop(v, act, now_ms, VC_WHY_EXEC_TIMEOUT);
		}
		break;
	case VC_SEARCH:
		if (episode > v->p.t_search_ms) {
			safe_stop(v, act, now_ms, VC_WHY_SEARCH_FAILED);
		}
		break;
	case VC_WAIT_ACK:
		if (age > v->p.t_ack_ms) {
			safe_stop(v, act, now_ms, VC_WHY_ACK_TIMEOUT);
		}
		break;
	case VC_SCANNING:
		if (age > v->p.t_scan_ms) {
			safe_stop(v, act, now_ms, VC_WHY_SCAN_FAILED);
		}
		break;
	case VC_BYPASS:
	case VC_REVERSE:
	case VC_FIND_LINE:
	case VC_ALIGN:
		if (episode > v->p.t_bypass_ms) {
			safe_stop(v, act, now_ms, VC_WHY_BYPASS_TIMEOUT);
		}
		break;
	default:
		break;				/* READY, SAFE_STOP, FINISHED: wait for commands */
	}
}
