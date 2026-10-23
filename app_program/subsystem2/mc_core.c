/*
 *	mc_core.c
 *	Motion Control logic (see mc_core.h).  Integer arithmetic only: the
 *	RP2040 has no floating-point unit.
 *
 *	One call of mc_core_step():
 *	  1. stop bits first -- a set bit always wins, on every call;
 *	  between ticks (woken by a new speed hold): the hold is applied at
 *	  once -- new wheel targets and duties -- so steering does not wait up
 *	  to 20 ms for the next tick (design-review feedback IS32);
 *	  and on the 20 ms control tick only:
 *	  2. measure both wheel speeds from the encoder counts;
 *	  3. take the new commands (a new one replaces the current move);
 *	  4. run the active move: speed hold, distance or turn;
 *	  5. one PI loop per wheel, with feed-forward and anti-windup;
 *	  6. stall, encoder-direction and move-timeout checks;
 *	  7. update the snapshot shared with the other subsystems.
 */

#include <string.h>
#include "mc_core.h"
#include "../bitpack.h"

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
#define MC_DECEL_MIN		100	/* mm/s^2: limits for a learned stopping rate */
#define MC_DECEL_MAX		20000

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

/* ---- packed command, snapshot and encoder word (mc_core.h) -------------- */
BP_STATIC_ASSERT(sizeof(MC_CMD_PK) == 8U, mc_cmd_pk_is_8_bytes);
BP_STATIC_ASSERT(sizeof(MC_SNAP_PK) == MC_SNAP_PK_BYTES, mc_snap_pk_size);
BP_STATIC_ASSERT(MC_SNAP_PK_BITS <= (MC_SNAP_PK_BYTES * 8U), mc_snap_pk_fits);

#define PK_SPEED_BITS	10U
#define PK_SPEED_MIN	(-512)
#define PK_DIST_BITS	15U
#define PK_DIST_MIN	(-16384)
#define PK_DEG_BITS	10U
#define PK_DEG_MIN	(-512)
#define PK_VMAX_BITS	9U
#define PK_ID_BITS	8U
#define PK_T16_BITS	16U
#define PK_C_PRESENT	0x10000	/* c of an unpacked SPEED: bit 16 = time present */

void mc_cmd_pack(const MC_CMD *cmd, MC_CMD_PK *pk)
{
	BP_W w;

	w.rec = pk->b;
	w.pos = 0U;
	memset(pk->b, 0, sizeof(pk->b));
	bp_wraw(&w, 2U, cmd->type);
	switch (cmd->type) {
	case MC_CMD_SPEED:
		bp_ws(&w, PK_SPEED_BITS, PK_SPEED_MIN, cmd->a);
		bp_ws(&w, PK_SPEED_BITS, PK_SPEED_MIN, cmd->b);
		bp_wraw(&w, 1U, (cmd->c != 0) ? 1U : 0U);
		bp_wraw(&w, PK_T16_BITS, (uint32_t)cmd->c);	/* low 16 bits: wraps */
		break;
	case MC_CMD_DISTANCE:
		bp_ws(&w, PK_DIST_BITS, PK_DIST_MIN, cmd->a);
		bp_wu(&w, PK_VMAX_BITS, cmd->b);
		bp_wu(&w, PK_ID_BITS, cmd->c);
		break;
	case MC_CMD_TURN:
		bp_ws(&w, PK_DEG_BITS, PK_DEG_MIN, cmd->a);
		bp_wu(&w, PK_VMAX_BITS, cmd->b);
		bp_wu(&w, PK_ID_BITS, cmd->c);
		break;
	default:
		break;			/* invalid: only the type is kept */
	}
}

void mc_cmd_unpack(const MC_CMD_PK *pk, MC_CMD *cmd)
{
	BP_R r;

	r.rec = pk->b;
	r.pos = 0U;
	memset(cmd, 0, sizeof(*cmd));
	cmd->type = (uint8_t)bp_ru(&r, 2U);
	switch (cmd->type) {
	case MC_CMD_SPEED:
		cmd->a = bp_rs(&r, PK_SPEED_BITS, PK_SPEED_MIN);
		cmd->b = bp_rs(&r, PK_SPEED_BITS, PK_SPEED_MIN);
		if (bp_ru(&r, 1U) != 0U) {
			/* bit 16 keeps a time whose low 16 bits are 0 non-zero;
			   only the low 16 bits are used (bp_age16) */
			cmd->c = PK_C_PRESENT | (int32_t)bp_ru(&r, PK_T16_BITS);
		}
		break;
	case MC_CMD_DISTANCE:
		cmd->a = bp_rs(&r, PK_DIST_BITS, PK_DIST_MIN);
		cmd->b = (int32_t)bp_ru(&r, PK_VMAX_BITS);
		cmd->c = (int32_t)bp_ru(&r, PK_ID_BITS);
		break;
	case MC_CMD_TURN:
		cmd->a = bp_rs(&r, PK_DEG_BITS, PK_DEG_MIN);
		cmd->b = (int32_t)bp_ru(&r, PK_VMAX_BITS);
		cmd->c = (int32_t)bp_ru(&r, PK_ID_BITS);
		break;
	default:
		break;
	}
}

void mc_snap_pack(const MC_SNAPSHOT *s, MC_SNAP_PK *pk)
{
	BP_W		w;
	uint32_t	i;

	memset(pk, 0, sizeof(*pk));
	w.rec = (uint8_t *)pk->w;
	w.pos = 0U;
	bp_wraw(&w, 8U, s->seq);
	bp_wraw(&w, 16U, s->t_ms);
	for (i = 0U; i < MC_WHEELS; i++) {
		bp_wraw(&w, 16U, (uint32_t)s->count[i]);
	}
	for (i = 0U; i < MC_WHEELS; i++) {
		bp_ws(&w, 10U, -512, s->speed[i]);
	}
	for (i = 0U; i < MC_WHEELS; i++) {
		bp_ws(&w, 10U, -512, s->target[i]);
	}
	for (i = 0U; i < MC_WHEELS; i++) {
		bp_ws(&w, 11U, -1024, s->duty[i]);
	}
	bp_ws(&w, 10U, -512, s->speed_mm_s);
	bp_ws(&w, 15U, -16384, s->accel_mm_s2);
	bp_ws(&w, 25U, -16777216, s->dist_mm);
	bp_wu(&w, 24U, s->odo_mm);
	bp_ws(&w, 17U, -65536, s->yaw_cdeg_s);
	bp_wu(&w, 3U, s->mode);
	bp_wraw(&w, 3U, s->fault_bits);
	bp_wu(&w, 10U, s->steer_lat_ms);
	bp_wu(&w, 10U, s->steer_lat_max_ms);
	bp_wu(&w, 17U, s->stop_lat_max_us);
	bp_wu(&w, 10U, s->stop_dist_mm);
	bp_wu(&w, 15U, s->stop_decel_mm_s2);
}

void mc_snap_unpack(const MC_SNAP_PK *pk, MC_SNAPSHOT *s)
{
	BP_R		r;
	uint32_t	i;

	r.rec = (const uint8_t *)pk->w;
	r.pos = 0U;
	s->seq = bp_ru(&r, 8U);
	s->t_ms = bp_ru(&r, 16U);
	for (i = 0U; i < MC_WHEELS; i++) {
		s->count[i] = (int32_t)(int16_t)bp_ru(&r, 16U);	/* low 16 bits, signed */
	}
	for (i = 0U; i < MC_WHEELS; i++) {
		s->speed[i] = bp_rs(&r, 10U, -512);
	}
	for (i = 0U; i < MC_WHEELS; i++) {
		s->target[i] = bp_rs(&r, 10U, -512);
	}
	for (i = 0U; i < MC_WHEELS; i++) {
		s->duty[i] = bp_rs(&r, 11U, -1024);
	}
	s->speed_mm_s = bp_rs(&r, 10U, -512);
	s->accel_mm_s2 = bp_rs(&r, 15U, -16384);
	s->dist_mm = bp_rs(&r, 25U, -16777216);
	s->odo_mm = (int32_t)bp_ru(&r, 24U);
	s->yaw_cdeg_s = bp_rs(&r, 17U, -65536);
	s->mode = (int32_t)bp_ru(&r, 3U);
	s->fault_bits = bp_ru(&r, 3U);
	s->steer_lat_ms = (int32_t)bp_ru(&r, 10U);
	s->steer_lat_max_ms = (int32_t)bp_ru(&r, 10U);
	s->stop_lat_max_us = (int32_t)bp_ru(&r, 17U);
	s->stop_dist_mm = (int32_t)bp_ru(&r, 10U);
	s->stop_decel_mm_s2 = (int32_t)bp_ru(&r, 15U);
}

/* odo_mm's bit position in the layout above: seq 8 + t_ms 16 + count 2x16 +
   speed 2x10 + target 2x10 + duty 2x11 + speed_mm_s 10 + accel 15 + dist 25 */
#define SNAP_ODO_POS	168U

int32_t mc_snap_odo(const MC_SNAP_PK *pk)
{
	return (int32_t)bp_get((const uint8_t *)pk->w, SNAP_ODO_POS, 24U);
}

void mc_enc_init(MC_ENC *e, uint32_t word)
{
	e->last = word;
	e->pos[MC_LEFT] = (int32_t)(int16_t)(uint16_t)word;
	e->pos[MC_RIGHT] = (int32_t)(int16_t)(uint16_t)(word >> 16);
}

void mc_enc_update(MC_ENC *e, uint32_t word)
{
	/* 16-bit differences, wrap-safe: (int16_t)(new - old) */
	e->pos[MC_LEFT] += (int32_t)(int16_t)(uint16_t)((uint16_t)word - (uint16_t)e->last);
	e->pos[MC_RIGHT] += (int32_t)(int16_t)(uint16_t)((uint16_t)(word >> 16) -
							   (uint16_t)(e->last >> 16));
	e->last = word;
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
		c->coasting = false;			/* a fault always brakes */
		if (c->mode != MC_FAULT) {
			brake(c);
			c->mode = MC_FAULT;
			c->ack_pending = false;
			out->flush = true;
			out->stop_latency_us = (in->now_us - in->stop_req_us) | 1U;
		}
	} else if (in->cmd_stop) {
		if ((c->mode != MC_STOPPED) && (c->mode != MC_FAULT)) {
			brake(c);			/* duty 0 at once ... */
			c->coasting = (c->p.stop_coast != 0);	/* ... free-wheeling, not braking */
			c->coast_ticks = 0;
			c->stop_speed = iabs32(c->snap.speed_mm_s);
			c->mode = MC_STOPPED;
			c->ack_pending = true;
			c->stop_req_us = in->stop_req_us;
			c->stop_odo = c->snap.odo_mm;
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
		c->coasting = false;
		c->snap.fault_bits = MC_FLT_NONE;
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

/* How fast the car is slowing down, mm/s^2 (+ = slowing, either
   direction), over the last MC_ACCEL_WINDOW periods. */
static int32_t coast_slowing(const MC_CORE *c)
{
	return ((iabs32(c->mean_hist[MC_ACCEL_WINDOW]) - iabs32(c->mean_hist[0])) * MC_TICKS_PER_S) /
	       (int32_t)MC_ACCEL_WINDOW;
}

/* Learn how hard this car really stops: speed^2 / (2 x stopping
   distance), from every stop that started at a real speed, averaged
   (3/4 old + 1/4 new).  Obstacle Scan uses it to start a stop early
   enough that the car comes to rest stop_mm from the obstacle at any
   speed (no guessed braking distance). */
static void learn_stop(MC_CORE *c)
{
	int32_t v = c->stop_speed;
	int32_t d = (c->snap.stop_dist_mm > 0) ? c->snap.stop_dist_mm : 1;
	int32_t decel;

	if (v < (2 * c->p.settle_speed)) {
		return;				/* too slow to tell anything */
	}
	decel = clamp32((v * v) / (2 * d), MC_DECEL_MIN, MC_DECEL_MAX);
	c->snap.stop_decel_mm_s2 = (c->snap.stop_decel_mm_s2 == 0) ? decel :
				   (((3 * c->snap.stop_decel_mm_s2) + decel) / 4);
}

/* The travel speed of a speed hold: the FASTER wheel.  Steering slows the
   inside wheel and leaves the outside one at this speed, so steering never
   changes it, and only a real change of speed is ramped at accel. */
static int32_t hold_base(const MC_CORE *c)
{
	int32_t l = iabs32(c->hold_cmd[MC_LEFT]);
	int32_t r = iabs32(c->hold_cmd[MC_RIGHT]);

	return (l > r) ? l : r;
}

/* How much one wheel's target may change in one tick: steer_pm_mm
   permille of the travel speed for every mm the car travels in that tick
   (travel speed x tick).  Measured per mm of travel, so the smoothing
   shapes the PATH the same way at any speed.  Never less than one tick of
   the start ramp, so it cannot hold back starting off. */
static int32_t steer_room(const MC_CORE *c, int32_t base, int32_t ramp_step)
{
	int32_t travel_um = base * MC_PERIOD_MS;		/* mm/s x ms = um this tick */
	int32_t room = (base * c->p.steer_pm_mm / 10) * travel_um / 100000;	/* mm/s */

	return (room > ramp_step) ? room : ramp_step;
}

/* ---- 3. commands -------------------------------------------------------- */
static void start_move(MC_CORE *c, const MC_CMD *cmd)
{
	int32_t expect_ms;
	int32_t mean = (c->w[MC_LEFT].speed + c->w[MC_RIGHT].speed) / 2;

	c->start[MC_LEFT] = c->w[MC_LEFT].hist[0];
	c->start[MC_RIGHT] = c->w[MC_RIGHT].hist[0];
	/* A new speed hold while already holding speed (line following
	   re-sends one at least every VC_HOLD_REFRESH_MS) is the SAME move:
	   keep counting a stall.  Resetting here meant a car pushing against
	   something while following the line was never caught. */
	if (!((cmd->type == MC_CMD_SPEED) && (c->mode == MC_SPEED))) {
		c->stall_ticks = 0;
	}
	c->settle_ticks = 0;

	switch (cmd->type) {
	case MC_CMD_SPEED:
		if (c->mode != MC_SPEED) {
			/* ramp on from the present speed of the faster wheel */
			c->hold_common = (iabs32(c->w[MC_LEFT].speed) > iabs32(c->w[MC_RIGHT].speed)) ?
					 iabs32(c->w[MC_LEFT].speed) : iabs32(c->w[MC_RIGHT].speed);
			c->steer_room[MC_LEFT] = steer_room(c, hold_base(c), (c->p.accel * MC_PERIOD_MS) / 1000);
			c->steer_room[MC_RIGHT] = c->steer_room[MC_LEFT];
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
			if ((in->cmd[i].type == MC_CMD_SPEED) && (in->cmd[i].c != 0)) {
				/* latency log: line reading -> these wheel targets */
				c->snap.steer_lat_ms = (int32_t)bp_age16(in->now_ms,
									 (uint32_t)in->cmd[i].c);
				if (c->snap.steer_lat_ms > c->snap.steer_lat_max_ms) {
					c->snap.steer_lat_max_ms = c->snap.steer_lat_ms;
				}
			}
		}
	}
}

/* ---- 4. the active move ------------------------------------------------- */
/* Wheel targets: the commanded wheel speeds scaled by (ramped travel speed
   / commanded travel speed), so while the car starts off the wheels keep
   their ratio (no wheel reverses).  Each wheel's target then moves toward
   that by at most steer_room, so a steering change is a short ramp, not a
   jump: smooth wheels, and the outside wheel is never pushed above the
   travel speed.  The first step is taken at once, even between ticks
   (steering latency, feedback IS32). */
static void speed_targets(MC_CORE *c)
{
	int32_t  base = hold_base(c);
	uint32_t i;

	for (i = 0U; i < MC_WHEELS; i++) {
		int32_t want = (base != 0) ? ((c->hold_cmd[i] * c->hold_common) / base) : 0;
		int32_t delta = want - c->w[i].target;

		if (c->p.steer_pm_mm > 0) {
			delta = clamp32(delta, -c->steer_room[i], c->steer_room[i]);
			c->steer_room[i] -= iabs32(delta);
		}
		c->w[i].target += delta;
	}
}

static void run_speed(MC_CORE *c)
{
	int32_t want = hold_base(c);
	int32_t step = (c->p.accel * MC_PERIOD_MS) / 1000;
	uint32_t i;

	for (i = 0U; i < MC_WHEELS; i++) {
		c->steer_room[i] = steer_room(c, want, step);
	}

	/* Ramp the common speed by one tick's worth */
	if (c->hold_common < want) {
		c->hold_common = (c->hold_common + step < want) ? (c->hold_common + step) : want;
	} else {
		c->hold_common = (c->hold_common - step > want) ? (c->hold_common - step) : want;
	}
	speed_targets(c);
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
/* Limit how fast a wheel's drive can GROW (rise permille per call), so a
   start or a speed-up does not jerk the car.  Less drive, braking and a
   change of direction through zero are never delayed. */
static int32_t limit_rise(int32_t prev, int32_t want, int32_t rise)
{
	int32_t out = want;

	if (rise > 0) {
		if (want > 0) {
			int32_t base = (prev > 0) ? prev : 0;

			out = (want > (base + rise)) ? (base + rise) : want;
		} else if (want < 0) {
			int32_t base = (prev < 0) ? prev : 0;

			out = (want < (base - rise)) ? (base - rise) : want;
		} else {
			out = 0;
		}
	}
	return out;
}

/* integrate: TRUE on the tick only (the integral is per 20 ms period) */
static int32_t wheel_pi(const MC_PARAMS *p, MC_WHEEL *w, bool integrate)
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
	out = clamp32(out, -MC_DUTY_MAX, MC_DUTY_MAX);
	{
		int32_t lim = limit_rise(w->duty, out, p->duty_rise);

		/* While the rise is being limited the wheel lags on purpose:
		   do not let the integral wind up on that error. */
		step = (p->ki_x1000 * err) / MC_TICKS_PER_S;
		if (integrate && (lim == out) &&
		    ((out < MC_DUTY_MAX) || (err < 0)) && ((out > -MC_DUTY_MAX) || (err > 0))) {
			w->integ = clamp32(w->integ + step, -MC_INTEG_MAX, MC_INTEG_MAX);
		}
		out = lim;
	}
	return out;
}

/* ---- 6. checks ---------------------------------------------------------- */
static void own_fault(MC_CORE *c, MC_OUT *out, uint32_t bit)
{
	brake(c);
	c->mode = MC_FAULT;
	c->await_fault_flag = true;
	c->snap.fault_bits = bit;
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
	if ((int32_t)out->stop_latency_us > c->snap.stop_lat_max_us) {
		c->snap.stop_lat_max_us = (int32_t)out->stop_latency_us;
	}

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
			if (c->coasting) {
				c->coast_ticks++;
				if ((iabs32(c->w[MC_LEFT].speed) < c->p.settle_speed) &&
				    (iabs32(c->w[MC_RIGHT].speed) < c->p.settle_speed)) {
					c->coasting = false;	/* stopped: now hold it with the brake */
				} else if ((c->coast_ticks > (int32_t)MC_ACCEL_WINDOW) &&
					   (coast_slowing(c) < c->p.accel)) {
					/* coasting slows the car less than its own ramp
					   rate (a slope, a light car): brake instead, so
					   a stop is never slower than a ramp down */
					c->coasting = false;
				} else {
					/* still coasting down */
				}
			}
			if (c->ack_pending &&
			    (iabs32(c->w[MC_LEFT].speed) < c->p.settle_speed) &&
			    (iabs32(c->w[MC_RIGHT].speed) < c->p.settle_speed)) {
				c->snap.stop_dist_mm = c->snap.odo_mm - c->stop_odo;
				learn_stop(c);
				(void)evt_list_add(&out->ev, (uint8_t)EV_STOP_ACK,
						   (int32_t)(in->now_us - c->stop_req_us),
						   c->snap.stop_dist_mm);
				c->ack_pending = false;
			}
			break;
		default:
			break;				/* Idle, Fault: braked */
		}

		if ((c->mode == MC_SPEED) || (c->mode == MC_DISTANCE) || (c->mode == MC_TURN)) {
			for (i = 0U; i < MC_WHEELS; i++) {
				c->w[i].duty = wheel_pi(&c->p, &c->w[i], true);
			}
			check_wheels(c, out);
		} else {
			for (i = 0U; i < MC_WHEELS; i++) {
				c->w[i].duty = 0;
				c->w[i].integ = 0;
			}
		}
		update_snapshot(c, in->now_ms);
	} else if (in->ncmd > 0U) {
		/* Woken between ticks by a new speed hold: apply it now.  The
		   speed measurement, the ramp and the integrators only move on
		   the tick; a distance or turn command starts on the next tick. */
		take_commands(c, in);
		if (c->mode == MC_SPEED) {
			speed_targets(c);
			for (i = 0U; i < MC_WHEELS; i++) {
				c->w[i].duty = wheel_pi(&c->p, &c->w[i], false);
			}
		}
	} else {
		/* stop bits only */
	}

	for (i = 0U; i < MC_WHEELS; i++) {
		out->duty[i] = c->w[i].duty;
	}
	out->coast = c->coasting && (c->mode == MC_STOPPED);
}
