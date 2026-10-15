/*
 *	mc_core.c
 *	Motion Control logic (see mc_core.h).  Integer arithmetic only: the
 *	RP2040 has no floating-point unit.
 *
 *	One call of mc_core_step():
 *	  1. stop bits first -- a set bit always wins, on every call;
 *	  and on the 20 ms control tick only:
 *	  2. measure both wheel speeds from the encoder counts;
 *	  3. take the new commands (a new one replaces the current move);
 *	  4. run the active move: speed hold, distance or turn;
 *	  5. one PI loop per wheel, with feed-forward and anti-windup;
 *	  6. stall, encoder-direction and move-timeout checks;
 *	  7. update the snapshot shared with the other subsystems.
 */

#include "mc_core.h"

#define MC_INTEG_MAX		(MC_DUTY_MAX * 1000)	/* integrator limit, permille x 1000 */
#define MC_TICKS_PER_S		(1000 / MC_PERIOD_MS)	/* 50 */
#define MC_BACKWARDS_DUTY	300	/* permille: only judged when pushing this hard */
#define MC_BACKWARDS_SPEED	80	/* mm/s the wrong way */
#define MC_BACKWARDS_TICKS	15	/* 300 ms: longer than any normal braking */
#define MC_SETTLE_TICKS_MAX	25	/* 500 ms: report DONE even if a wheel creeps */
#define MC_MAX_DISTANCE_MM	10000
#define MC_MAX_TURN_DEG		360
#define MC_CDEG_PER_RAD		5730	/* 100 x 180 / pi */
#define MC_CAL_MIN		500	/* permille limits for calibration factors */
#define MC_CAL_MAX		1500
#define MC_KP_MAX_X1000		20000
#define MC_KI_MAX_X1000		50000
#define MC_HALF_MM_PER_M	500	/* (left + right) / 2 x 1000 mm per metre */

static int32_t iabs32(int32_t x)
{
	return (x < 0) ? -x : x;
}

static int32_t clamp32(int32_t x, int32_t lo, int32_t hi)
{
	if (x < lo) {
		return lo;
	}
	if (x > hi) {
		return hi;
	}
	return x;
}

/* Integer square root, x >= 0 */
static int32_t isqrt32(int32_t x)
{
	uint32_t v = (x > 0) ? (uint32_t)x : 0U;
	uint32_t r = 0U;
	uint32_t bit = 1UL << 30;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit != 0U) {
		if (v >= (r + bit)) {
			v -= r + bit;
			r = (r >> 1) + bit;
		} else {
			r >>= 1;
		}
		bit >>= 2;
	}
	return (int32_t)r;
}

/* Counts -> mm, for the small values of one move (|counts| < 2 000 000) */
static int32_t counts_to_mm(const MC_CORE *c, int32_t counts)
{
	return (counts * 1000) / c->p.counts_per_m;
}

static void brake(MC_CORE *c)
{
	uint32_t i;

	for (i = 0U; i < MC_WHEELS; i++) {
		c->w[i].duty = 0;
		c->w[i].integ = 0;
		c->w[i].target = 0;
		c->w[i].backwards = 0;
	}
	c->hold_common = 0;
	c->hold_cmd[MC_LEFT] = 0;
	c->hold_cmd[MC_RIGHT] = 0;
	c->vcmd = 0;
	c->stall_ticks = 0;
}

void mc_core_init(MC_CORE *c, const MC_PARAMS *p, const int32_t count[MC_WHEELS])
{
	uint32_t i, k;

	*c = (MC_CORE){0};
	c->p = *p;
	c->p.speed_window = clamp32(p->speed_window, 1, (int32_t)MC_SPEED_WINDOW_MAX);
	c->cpm_base = p->counts_per_m;
	c->c90_base = p->counts_per_90deg;
	c->mode = MC_IDLE;
	for (i = 0U; i < MC_WHEELS; i++) {
		for (k = 0U; k <= MC_SPEED_WINDOW_MAX; k++) {
			c->w[i].hist[k] = count[i];
		}
		c->snap.count[i] = count[i];
	}
	c->snap.mode = (int32_t)MC_IDLE;
}

void mc_core_tune(MC_CORE *c, int32_t kp_x1000, int32_t ki_x1000,
		  int32_t dist_cal_permille, int32_t turn_cal_permille)
{
	if ((kp_x1000 >= 0) && (kp_x1000 <= MC_KP_MAX_X1000)) {
		c->p.kp_x1000 = kp_x1000;
	}
	if ((ki_x1000 >= 0) && (ki_x1000 <= MC_KI_MAX_X1000)) {
		c->p.ki_x1000 = ki_x1000;
	}
	/* cal = 1000 x commanded / measured, from a calibration run */
	if ((dist_cal_permille >= MC_CAL_MIN) && (dist_cal_permille <= MC_CAL_MAX)) {
		c->p.counts_per_m = (c->cpm_base * dist_cal_permille) / 1000;
	}
	if ((turn_cal_permille >= MC_CAL_MIN) && (turn_cal_permille <= MC_CAL_MAX)) {
		c->p.counts_per_90deg = (c->c90_base * turn_cal_permille) / 1000;
	}
}

bool mc_cmd_valid(const MC_CMD *cmd, int32_t max_speed)
{
	switch (cmd->type) {
	case MC_CMD_SPEED:
		return (iabs32(cmd->a) <= max_speed) && (iabs32(cmd->b) <= max_speed);
	case MC_CMD_DISTANCE:
		return (cmd->a != 0) && (iabs32(cmd->a) <= MC_MAX_DISTANCE_MM) &&
		       (cmd->b > 0) && (cmd->b <= max_speed);
	case MC_CMD_TURN:
		return (cmd->a != 0) && (iabs32(cmd->a) <= MC_MAX_TURN_DEG) &&
		       (cmd->b >= 0) && (cmd->b <= max_speed);
	default:
		return false;
	}
}

/* ---- 1. stop bits ------------------------------------------------------- */
static void handle_stop_bits(MC_CORE *c, const MC_IN *in, MC_OUT *out)
{
	if (in->fault_stop) {
		c->await_fault_flag = false;		/* the latch is now visible */
		if (c->mode != MC_FAULT) {
			brake(c);
			c->mode = MC_FAULT;
			c->ack_pending = false;
			out->flush = true;
			out->stop_latency_us = (in->now_us - in->stop_req_us) | 1U;
		}
	} else if (in->cmd_stop) {
		if ((c->mode != MC_STOPPED) && (c->mode != MC_FAULT)) {
			brake(c);
			c->mode = MC_STOPPED;
			c->ack_pending = true;
			c->stop_req_us = in->stop_req_us;
			out->flush = true;
			out->stop_latency_us = (in->now_us - in->stop_req_us) | 1U;
		} else if ((c->mode == MC_FAULT) && !c->await_fault_flag) {
			/* FAULT_STOP was cleared while CMD_STOP is still set */
			c->mode = MC_STOPPED;
			c->ack_pending = false;
		} else {
			/* already stopped */
		}
	} else if ((c->mode == MC_STOPPED) ||
		   ((c->mode == MC_FAULT) && !c->await_fault_flag)) {
		c->mode = MC_IDLE;			/* both bits clear: back to Idle */
		c->ack_pending = false;
		c->snap.fault_code = MC_FLT_NONE;
	} else {
		/* running normally */
	}
}

/* ---- 2. measurement ----------------------------------------------------- */
static void measure(MC_CORE *c, const int32_t count[MC_WHEELS])
{
	int32_t		wnd = c->p.speed_window;
	int32_t		cpm = c->p.counts_per_m;
	int32_t		dsum = 0;
	int32_t		mean;
	uint32_t	i;
	int32_t		k;

	for (i = 0U; i < MC_WHEELS; i++) {
		MC_WHEEL *w = &c->w[i];

		for (k = (int32_t)MC_SPEED_WINDOW_MAX; k > 0; k--) {
			w->hist[k] = w->hist[k - 1];
		}
		w->hist[0] = count[i];
		dsum += w->hist[0] - w->hist[1];	/* counts this period */

		/* counts over the window -> mm/s:
		   (counts x 1000 mm/m x 50 periods/s) / (window x counts/m) */
		w->speed = ((w->hist[0] - w->hist[wnd]) * 1000 * MC_TICKS_PER_S) / (wnd * cpm);
	}

	/* Distances in mm, carrying the remainder so nothing is lost and
	   nothing overflows however long the run.  dsum = 2 x mean counts. */
	c->odo_rem += iabs32(dsum) * MC_HALF_MM_PER_M;
	c->snap.odo_mm += c->odo_rem / cpm;
	c->odo_rem %= cpm;
	c->dist_rem += dsum * MC_HALF_MM_PER_M;
	c->snap.dist_mm += c->dist_rem / cpm;
	c->dist_rem %= cpm;

	/* acceleration over MC_ACCEL_WINDOW periods: one encoder count per
	   speed window is a large step, so a single period would be noise */
	mean = (c->w[MC_LEFT].speed + c->w[MC_RIGHT].speed) / 2;
	for (k = (int32_t)MC_ACCEL_WINDOW; k > 0; k--) {
		c->mean_hist[k] = c->mean_hist[k - 1];
	}
	c->mean_hist[0] = mean;
	c->snap.accel_mm_s2 = ((mean - c->mean_hist[MC_ACCEL_WINDOW]) * MC_TICKS_PER_S) /
			      (int32_t)MC_ACCEL_WINDOW;
	c->snap.speed_mm_s = mean;
}

/* ---- 3. commands -------------------------------------------------------- */
static void start_move(MC_CORE *c, const MC_CMD *cmd)
{
	int32_t expect_ms;
	int32_t mean = (c->w[MC_LEFT].speed + c->w[MC_RIGHT].speed) / 2;

	c->start[MC_LEFT] = c->w[MC_LEFT].hist[0];
	c->start[MC_RIGHT] = c->w[MC_RIGHT].hist[0];
	c->stall_ticks = 0;
	c->settle_ticks = 0;

	switch (cmd->type) {
	case MC_CMD_SPEED:
		if (c->mode != MC_SPEED) {
			c->hold_common = mean;		/* ramp on from the present speed */
		}
		c->hold_cmd[MC_LEFT] = cmd->a;
		c->hold_cmd[MC_RIGHT] = cmd->b;
		c->mode = MC_SPEED;
		break;
	case MC_CMD_DISTANCE:
		c->dir = (cmd->a > 0) ? 1 : -1;
		c->goal_counts = (iabs32(cmd->a) * c->p.counts_per_m) / 1000;
		c->vmax = (cmd->b < c->p.max_speed) ? cmd->b : c->p.max_speed;
		c->vcmd = clamp32(c->dir * mean, 0, c->vmax);	/* smooth hand-over */
		c->move_type = MC_MOVE_DISTANCE;
		c->move_id = cmd->c;
		expect_ms = (iabs32(cmd->a) * 1000) / c->vmax;
		c->ticks_left = ((2 * expect_ms) + c->p.timeout_margin_ms) / MC_PERIOD_MS;
		c->mode = MC_DISTANCE;
		break;
	case MC_CMD_TURN:
		c->dir = (cmd->a > 0) ? 1 : -1;		/* + = left (counter-clockwise) */
		c->goal_counts = (iabs32(cmd->a) * c->p.counts_per_90deg) / 90;
		c->vmax = (cmd->b > 0) ? cmd->b : c->p.turn_speed;
		c->vcmd = 0;
		c->move_type = MC_MOVE_TURN;
		c->move_id = cmd->c;
		expect_ms = (counts_to_mm(c, c->goal_counts) * 1000) / c->vmax;
		c->ticks_left = ((2 * expect_ms) + c->p.timeout_margin_ms) / MC_PERIOD_MS;
		c->mode = MC_TURN;
		break;
	default:
		break;
	}
}

static void take_commands(MC_CORE *c, const MC_IN *in)
{
	uint32_t i;

	for (i = 0U; i < in->ncmd; i++) {
		if ((c->mode == MC_STOPPED) || (c->mode == MC_FAULT) ||
		    !mc_cmd_valid(&in->cmd[i], c->p.max_speed)) {
			c->rejected++;
		} else {
			start_move(c, &in->cmd[i]);
		}
	}
}

/* ---- 4. the active move ------------------------------------------------- */
static void run_speed(MC_CORE *c)
{
	int32_t want = (c->hold_cmd[MC_LEFT] + c->hold_cmd[MC_RIGHT]) / 2;
	int32_t diff = (c->hold_cmd[MC_RIGHT] - c->hold_cmd[MC_LEFT]) / 2;
	int32_t step = (c->p.accel * MC_PERIOD_MS) / 1000;

	/* Ramp the common speed; scale the steering part with it, so the
	   wheels keep their ratio while accelerating (no wheel reverses
	   just because the car is starting off). */
	if (c->hold_common < want) {
		c->hold_common = (c->hold_common + step < want) ? (c->hold_common + step) : want;
	} else {
		c->hold_common = (c->hold_common - step > want) ? (c->hold_common - step) : want;
	}
	if ((want != 0) && (c->hold_common != want)) {
		diff = (diff * c->hold_common) / want;
	}
	c->w[MC_LEFT].target = c->hold_common - diff;
	c->w[MC_RIGHT].target = c->hold_common + diff;
}

static void run_profile(MC_CORE *c)
{
	int32_t prog_l = iabs32(c->w[MC_LEFT].hist[0] - c->start[MC_LEFT]);
	int32_t prog_r = iabs32(c->w[MC_RIGHT].hist[0] - c->start[MC_RIGHT]);
	int32_t prog = (prog_l + prog_r) / 2;
	int32_t remain_mm, v_brake, trim;

	if (prog >= c->goal_counts) {
		brake(c);
		c->mode = MC_SETTLE;
		c->settle_ticks = 0;
		return;
	}

	/* Trapezoid: ramp up at accel, cap at vmax, and slow down so that
	   v^2 = 2 a d still reaches zero at the goal. */
	c->vcmd += (c->p.accel * MC_PERIOD_MS) / 1000;
	remain_mm = counts_to_mm(c, c->goal_counts - prog);
	v_brake = isqrt32(2 * c->p.accel * remain_mm);
	if (c->vcmd > c->vmax) {
		c->vcmd = c->vmax;
	}
	if (c->vcmd > v_brake) {
		c->vcmd = v_brake;
	}
	if (c->vcmd < c->p.min_speed) {
		c->vcmd = c->p.min_speed;
	}

	/* Keep both wheels level: slow the one that is ahead, speed up the other */
	trim = c->p.k_straight * counts_to_mm(c, prog_l - prog_r);
	trim = clamp32(trim, -(c->vcmd / 2), c->vcmd / 2);

	if (c->mode == MC_DISTANCE) {
		c->w[MC_LEFT].target = c->dir * (c->vcmd - trim);
		c->w[MC_RIGHT].target = c->dir * (c->vcmd + trim);
	} else {				/* turn: wheels opposite, + = left */
		c->w[MC_LEFT].target = -c->dir * (c->vcmd - trim);
		c->w[MC_RIGHT].target = c->dir * (c->vcmd + trim);
	}
	c->ticks_left--;
}

/* ---- 5. per-wheel PI with feed-forward and anti-windup ------------------ */
static int32_t wheel_pi(const MC_PARAMS *p, MC_WHEEL *w)
{
	int32_t err, out, step;

	if (w->target == 0) {
		w->integ = 0;
		return 0;			/* 0 = brake */
	}
	err = w->target - w->speed;
	out = ((p->kff_x1000 * w->target) / 1000)
	    + ((w->target > 0) ? p->deadband : -p->deadband)
	    + ((p->kp_x1000 * err) / 1000)
	    + (w->integ / 1000);

	/* Integrate only when the output is not saturated, or when the error
	   pulls it back out of saturation. */
	step = (p->ki_x1000 * err) / MC_TICKS_PER_S;
	if (((out < MC_DUTY_MAX) || (err < 0)) && ((out > -MC_DUTY_MAX) || (err > 0))) {
		w->integ = clamp32(w->integ + step, -MC_INTEG_MAX, MC_INTEG_MAX);
	}
	return clamp32(out, -MC_DUTY_MAX, MC_DUTY_MAX);
}

/* ---- 6. checks ---------------------------------------------------------- */
static void own_fault(MC_CORE *c, MC_OUT *out, int32_t code)
{
	brake(c);
	c->mode = MC_FAULT;
	c->await_fault_flag = true;
	c->snap.fault_code = code;
	out->fault = true;
}

static void check_wheels(MC_CORE *c, MC_OUT *out)
{
	bool		stalled = false;
	uint32_t	i;

	for (i = 0U; i < MC_WHEELS; i++) {
		MC_WHEEL *w = &c->w[i];

		if ((iabs32(w->duty) > c->p.stall_duty) && (iabs32(w->speed) < c->p.stall_speed)) {
			stalled = true;
		}
		if ((iabs32(w->duty) >= MC_BACKWARDS_DUTY) &&
		    (((w->duty > 0) && (w->speed < -MC_BACKWARDS_SPEED)) ||
		     ((w->duty < 0) && (w->speed > MC_BACKWARDS_SPEED)))) {
			w->backwards++;
		} else {
			w->backwards = 0;
		}
	}
	c->stall_ticks = stalled ? (c->stall_ticks + 1) : 0;

	if ((c->w[MC_LEFT].backwards >= MC_BACKWARDS_TICKS) ||
	    (c->w[MC_RIGHT].backwards >= MC_BACKWARDS_TICKS)) {
		own_fault(c, out, MC_FLT_ENCODER);
	} else if (c->stall_ticks >= c->p.stall_ticks) {
		own_fault(c, out, MC_FLT_STALL);
	} else if (((c->mode == MC_DISTANCE) || (c->mode == MC_TURN)) && (c->ticks_left <= 0)) {
		own_fault(c, out, MC_FLT_TIMEOUT);
	} else {
		/* healthy */
	}
}

/* ---- 7. snapshot -------------------------------------------------------- */
static void update_snapshot(MC_CORE *c, uint32_t now_ms)
{
	uint32_t i;

	c->snap.seq++;
	c->snap.t_ms = now_ms;
	for (i = 0U; i < MC_WHEELS; i++) {
		c->snap.count[i] = c->w[i].hist[0];
		c->snap.speed[i] = c->w[i].speed;
		c->snap.target[i] = c->w[i].target;
		c->snap.duty[i] = c->w[i].duty;
	}
	/* yaw rate = (v_right - v_left) / track, rad/s -> centidegrees/s */
	c->snap.yaw_cdeg_s = ((c->w[MC_RIGHT].speed - c->w[MC_LEFT].speed) * MC_CDEG_PER_RAD) /
			     c->p.track_mm;
	c->snap.mode = (int32_t)c->mode;
}

/* ---- one step ----------------------------------------------------------- */
void mc_core_step(MC_CORE *c, const MC_IN *in, MC_OUT *out)
{
	uint32_t i;

	out->flush = false;
	out->fault = false;
	out->stop_latency_us = 0U;
	evt_list_clear(&out->ev);

	handle_stop_bits(c, in, out);

	if (in->tick) {
		measure(c, in->count);
		take_commands(c, in);

		switch (c->mode) {
		case MC_SPEED:
			run_speed(c);
			break;
		case MC_DISTANCE:
		case MC_TURN:
			run_profile(c);
			break;
		case MC_SETTLE:
			c->settle_ticks++;
			if (((iabs32(c->w[MC_LEFT].speed) < c->p.settle_speed) &&
			     (iabs32(c->w[MC_RIGHT].speed) < c->p.settle_speed)) ||
			    (c->settle_ticks >= MC_SETTLE_TICKS_MAX)) {
				(void)evt_list_add(&out->ev, (uint8_t)EV_DONE, c->move_type, c->move_id);
				c->mode = MC_IDLE;
			}
			break;
		case MC_STOPPED:
			if (c->ack_pending &&
			    (iabs32(c->w[MC_LEFT].speed) < c->p.settle_speed) &&
			    (iabs32(c->w[MC_RIGHT].speed) < c->p.settle_speed)) {
				(void)evt_list_add(&out->ev, (uint8_t)EV_STOP_ACK,
						   (int32_t)(in->now_us - c->stop_req_us), 0);
				c->ack_pending = false;
			}
			break;
		default:
			break;				/* Idle, Fault: braked */
		}

		if ((c->mode == MC_SPEED) || (c->mode == MC_DISTANCE) || (c->mode == MC_TURN)) {
			for (i = 0U; i < MC_WHEELS; i++) {
				c->w[i].duty = wheel_pi(&c->p, &c->w[i]);
			}
			check_wheels(c, out);
		} else {
			for (i = 0U; i < MC_WHEELS; i++) {
				c->w[i].duty = 0;
				c->w[i].integ = 0;
			}
		}
		update_snapshot(c, in->now_ms);
	}

	for (i = 0U; i < MC_WHEELS; i++) {
		out->duty[i] = c->w[i].duty;
	}
}
