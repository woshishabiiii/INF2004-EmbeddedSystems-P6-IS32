/*
 *	unit_tests.c
 *	Host unit tests for the firmware's pure logic modules.  Each check
 *	prints one line; the summary line says how many failed.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../app_program/app_evt.h"
#include "../../app_program/subsystem1/cmd_parse.h"
#include "../../app_program/subsystem1/tel_json.h"
#include "../../app_program/subsystem2/mc_core.h"
#include "../../app_program/subsystem2/motion_cfg.h"
#include "../../app_program/subsystem3/code39.h"
#include "../../app_program/subsystem3/barcode_core.h"
#include "../../app_program/subsystem3/line_core.h"
#include "../../app_program/subsystem3/line_cfg.h"
#include "../../app_program/subsystem4/imu_math.h"
#include "../../app_program/subsystem4/imu_core.h"
#include "../../app_program/subsystem4/imu_cfg.h"
#include "../../app_program/subsystem4/safety_core.h"
#include "../../app_program/subsystem5/os_profile.h"
#include "../../app_program/subsystem5/scan_core.h"
#include "../../app_program/subsystem5/scan_cfg.h"
#include "../../app_program/subsystem5/vc_core.h"
#include "../../app_program/subsystem5/vc_cfg.h"

static int g_n, g_fail;

static void check(const char *id, int ok, const char *what)
{
	g_n++;
	if (!ok) {
		g_fail++;
	}
	printf("[%s] %-8s %s\n", ok ? "PASS" : "FAIL", id, what);
}

static int has_event(const EVT_LIST *l, uint8_t type)
{
	uint32_t i;

	for (i = 0U; i < l->n; i++) {
		if (l->ev[i].type == type) {
			return 1;
		}
	}
	return 0;
}

/* ===== Code 39 ============================================================= */
static void reverse(uint32_t *w, uint32_t n)
{
	uint32_t i;

	for (i = 0U; i < n / 2U; i++) {
		uint32_t t = w[i];

		w[i] = w[n - 1U - i];
		w[n - 1U - i] = t;
	}
}

static void test_code39(void)
{
	const char	*alpha = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
	uint32_t	w[CODE39_ELEMENTS], i, k;
	char		c;
	bool		rev;
	int		ok_fwd = 1, ok_back = 1, ok_ramp = 1;

	for (k = 0U; alpha[k] != '\0'; k++) {
		(void)code39_encode(alpha[k], 3000U, 9000U, w);
		if ((code39_decode(w, CODE39_ELEMENTS, &c, &rev) != C39_OK) || (c != alpha[k]) || rev) {
			ok_fwd = 0;
		}
		reverse(w, CODE39_ELEMENTS);
		if ((code39_decode(w, CODE39_ELEMENTS, &c, &rev) != C39_OK) || (c != alpha[k]) || !rev) {
			ok_back = 0;
		}
		/* the car speeds up by 30 % while crossing the barcode */
		(void)code39_encode(alpha[k], 3000U, 9000U, w);
		for (i = 0U; i < CODE39_ELEMENTS; i++) {
			w[i] = (w[i] * (130U - i)) / 100U;
		}
		if ((code39_decode(w, CODE39_ELEMENTS, &c, &rev) != C39_OK) || (c != alpha[k])) {
			ok_ramp = 0;
		}
	}
	check("C39-01", ok_fwd, "all 43 characters decode forwards");
	check("C39-02", ok_back, "all 43 characters decode backwards, flagged reversed");
	check("C39-03", ok_ramp, "all decode with a 30 % speed change across the barcode");
	(void)code39_encode('A', 4000U, 9000U, w);
	check("C39-04", (code39_decode(w, CODE39_ELEMENTS, &c, &rev) == C39_OK) && (c == 'A'),
	      "wide/narrow ratio 2.25 still decodes");
	(void)code39_encode('A', 3000U, 9000U, w);
	w[10] = 3000U;		/* a wide bar of the data character read as narrow */
	check("C39-05", code39_decode(w, CODE39_ELEMENTS, &c, &rev) != C39_OK,
	      "a misread element is rejected, never a wrong character");
	check("C39-06", code39_decode(w, 27U, &c, &rev) == C39_BAD_COUNT, "27 widths: bad count");
	check("C39-07", !code39_encode('*', 3U, 9U, w) && !code39_encode('a', 3U, 9U, w),
	      "'*' and lower case cannot be encoded as data");
	(void)code39_encode('D', 3000U, 9000U, w);
	for (i = 10U; i < 20U; i++) {
		w[i] *= 2U;		/* the car is twice as slow over the data character ... */
	}
	for (i = 20U; i < CODE39_ELEMENTS; i++) {
		w[i] *= 3U;		/* ... and three times as slow over the stop character */
	}
	check("C39-08", (code39_decode(w, CODE39_ELEMENTS, &c, &rev) == C39_OK) && (c == 'D'),
	      "speed changing 3x between characters still decodes");
	for (i = 0U; i < CODE39_ELEMENTS; i++) {
		w[i] = 5000U;
	}
	check("C39-09", code39_decode(w, CODE39_ELEMENTS, &c, &rev) == C39_BAD_WIDTHS,
	      "all elements the same width: rejected");
}

/* ===== barcode capture ======================================================= */
static const BC_PARAMS k_bc = {LB_T_MIN_US, LB_T_END_MIN_MS * 1000U, LB_T_END_FACTOR, 1U};

/* feed "*c*" with narrow width n us; glitch = add a 100 us chatter pair in element 4 */
static bool feed(BC_CORE *b, char c, uint32_t n, bool glitch, bool merged, BC_RESULT *res)
{
	uint32_t	w[CODE39_ELEMENTS], t = 1000U, i;
	uint8_t		lvl = 1U;

	(void)code39_encode(c, n, 3U * n, w);
	for (i = 0U; i < CODE39_ELEMENTS; i++) {
		bc_edge(b, t, lvl);
		if (glitch && (i == 4U)) {
			bc_edge(b, t + (w[i] / 2U), (uint8_t)(lvl ^ 1U));
			bc_edge(b, t + (w[i] / 2U) + 100U, lvl);
		}
		if (merged && (i == 6U)) {
			bc_edge(b, t + (w[i] / 2U), lvl);	/* an edge that changes nothing */
		}
		t += w[i];
		lvl ^= 1U;
	}
	bc_edge(b, t, 0U);
	if (bc_poll(b, t + 1000U, res)) {
		return false;			/* must not end before the quiet time */
	}
	return bc_poll(b, t + 1000000U, res);
}

static void test_barcode(void)
{
	BC_CORE		b;
	BC_RESULT	r;
	uint32_t	i, t = 0U;

	bc_init(&b, &k_bc);
	check("BC-01", feed(&b, 'B', 15000U, false, false, &r) && (r.result == C39_OK) && (r.c == 'B'),
	      "clean capture decodes; not before the quiet time");
	bc_init(&b, &k_bc);
	check("BC-02", feed(&b, 'C', 15000U, true, false, &r) && (r.result == C39_OK) && (r.c == 'C'),
	      "a 100 us chatter pair inside a bar is removed");
	bc_init(&b, &k_bc);
	check("BC-03", feed(&b, 'D', 7500U, false, true, &r) && (r.result == C39_OK) && (r.c == 'D'),
	      "an edge that leaves the level unchanged is ignored");
	bc_init(&b, &k_bc);
	bc_edge(&b, 1000U, 1U);
	bc_edge(&b, 91000U, 0U);			/* crossing an 18 mm line */
	check("BC-04", !bc_poll(&b, 2000000U, &r) && (b.ok == 0U) && (b.failed == 0U),
	      "one bar (a line crossing) is ignored silently");
	bc_init(&b, &k_bc);
	for (i = 0U; i < 60U; i++) {
		bc_edge(&b, t, (uint8_t)((i % 2U) == 0U));
		t += 5000U;
	}
	check("BC-05", bc_poll(&b, t + 1000000U, &r) && (r.result == C39_BAD_COUNT),
	      "60 edges: not a barcode");
	check("BC-06", (bc_command_for('A') == LB_CMD_TURN_LEFT) && (bc_command_for('B') == LB_CMD_TURN_RIGHT) &&
	      (bc_command_for('C') == LB_CMD_STRAIGHT) && (bc_command_for('D') == LB_CMD_U_TURN) &&
	      (bc_command_for('Z') == LB_CMD_NONE), "command table");
}

/* ===== line sensing and steering =============================================== */
static void test_line(void)
{
	LC_CORE		c;
	LC_PARAMS	p = {LB_JUNCTION_MS, LB_JUNCTION_MM, LB_T_LOST_MS};
	EVT_LIST	ev;
	LS_STATE	s;
	LS_PARAMS	lp = {LS_D_MIN, LS_D_RATE, LS_D_MAX, LS_D_BIAS, LS_D_SEARCH, LS_T_ESCAPE_MS, LS_T_DRIFT_MS,
			      LB_T_LOST_MS, LS_SLOW_PCT, LS_START_SIDE};
	LS_OUT		o, o2;
	int		ok;

	lc_init(&c, &p, 0U);
	evt_list_clear(&ev);
	lc_step(&c, false, false, 5U, 0, &ev);
	lc_step(&c, false, false, 10U, 1, &ev);
	check("LC-01", (ev.n == 1U) && (ev.ev[0].type == EV_LINE_STATUS), "status sent once, then only on change");
	evt_list_clear(&ev);
	lc_step(&c, true, true, 15U, 2, &ev);
	lc_step(&c, true, true, 40U, 3, &ev);		/* long, but only 1 mm of travel */
	check("LC-02", !has_event(&ev, EV_JUNCTION), "both black while barely moving: no junction");
	lc_step(&c, true, true, 45U, 8, &ev);
	lc_step(&c, true, true, 50U, 12, &ev);
	ok = 0;
	{
		uint32_t i;

		for (i = 0U; i < ev.n; i++) {
			ok += (ev.ev[i].type == EV_JUNCTION) ? 1 : 0;
		}
	}
	check("LC-03", ok == 1, "junction reported exactly once per black band");
	evt_list_clear(&ev);
	lc_step(&c, false, true, 60U, 14, &ev);
	lc_step(&c, false, false, 70U, 16, &ev);
	lc_step(&c, false, false, 70U + LB_T_LOST_MS + 1U, 20, &ev);
	lc_step(&c, false, false, 80U + LB_T_LOST_MS + 1U, 20, &ev);
	check("LC-04", has_event(&ev, EV_LINE_LOST) && (ev.ev[ev.n - 1U].p1 == 1),
	      "line lost after T_lost, on the side last seen (right)");

	ls_init(&s, &lp, 0U);
	ls_update(&s, true, false, 100U);
	ls_output(&s, 100U, 200, &o);
	ls_output(&s, 400U, 200, &o2);
	check("LS-01", (o.v_left < o.v_right) && ((o2.v_right - o2.v_left) > (o.v_right - o.v_left)),
	      "left on black: steer left, harder the longer it stays");
	ls_update(&s, false, true, 500U);
	ls_output(&s, 500U, 200, &o);
	check("LS-02", o.v_left > o.v_right, "right on black: steer right");
	ls_update(&s, true, true, 600U);
	ls_output(&s, 600U, 200, &o);
	check("LS-03", o.v_left == o.v_right, "both black: straight across");
	ls_update(&s, false, true, 700U);
	ls_update(&s, false, false, 710U);
	ls_output(&s, 720U, 200, &o);
	check("LS-04", (o.v_left > o.v_right) && ((o.v_left - o.v_right) == 2 * LS_D_BIAS),
	      "both white just after the right sensor: lean gently right");
	ls_output(&s, 710U + LS_T_DRIFT_MS, 200, &o);
	ls_output(&s, 710U + (2U * LS_T_DRIFT_MS), 200, &o2);
	check("LS-05", ((o.v_left - o.v_right) == 2 * LS_D_BIAS) &&
	      ((o2.v_left - o2.v_right) == 4 * LS_D_BIAS) && (o2.v_left == 200) && !o2.lost,
	      "centred: the lean holds for T_drift, then grows (doubled after 2 x T_drift), full speed");
	ls_output(&s, 710U + LB_T_LOST_MS + 10U, 200, &o);
	check("LS-06", o.lost && (o.side == 1), "white past T_lost: lost, on the right");
	ls_update(&s, false, true, 3000U);
	ls_update(&s, false, false, 3000U + LS_T_ESCAPE_MS + 20U);
	ls_output(&s, 3000U + LS_T_ESCAPE_MS + 30U, 200, &o);
	check("LS-07", (o.v_left == (200 * LS_SLOW_PCT) / 100) && ((o.v_left - o.v_right) == 2 * LS_D_SEARCH),
	      "right sensor black a long time, then white: line went outside, slow and turn firmly right");
	ls_output(&s, 3000U + LS_T_ESCAPE_MS + 30U, 300, &o2);
	check("LS-08", (o2.v_left <= 300) && (o2.v_right <= 300), "no wheel is asked for more than the base speed");
}

/* ===== motion control ============================================================ */
static const MC_PARAMS k_mc = {
	MC_COUNTS_PER_M, MC_COUNTS_PER_90DEG, MC_TRACK_MM, MC_KFF_X1000, MC_DEADBAND,
	MC_KP_X1000, MC_KI_X1000, MC_MAX_SPEED, MC_MIN_SPEED, MC_ACCEL, MC_TURN_SPEED,
	MC_K_STRAIGHT, MC_SETTLE_SPEED, MC_STALL_DUTY, MC_STALL_SPEED, MC_STALL_TICKS,
	MC_TIMEOUT_MARGIN_MS, MC_SPEED_WINDOW
};

typedef struct {
	MC_CORE	c;
	double	cnt[2];
	double	v[2];
	uint32_t t;
	bool	stall;		/* the wheels cannot turn */
	int	sign[2];	/* -1 = encoder wired backwards */
} RIG;

/* one 20 ms period: a simple wheel model, then the controller */
static void rig_tick(RIG *r, MC_OUT *out, const MC_CMD *cmd, bool cmd_stop, bool fault_stop)
{
	MC_IN	in;
	int	i;

	for (i = 0; i < 2; i++) {
		double d = r->c.w[i].duty;
		double vss = (fabs(d) < 230.0) ? 0.0 : ((fabs(d) - 230.0) / 1.7) * ((d > 0.0) ? 1.0 : -1.0);

		if (r->stall) {
			vss = 0.0;
		}
		r->v[i] += (vss - r->v[i]) * 0.02 / 0.06;
		if (d == 0.0) {
			r->v[i] *= 0.4;
		}
		r->cnt[i] += r->v[i] * 0.02 * 2.0 * r->sign[i];
	}
	r->t += 20U;
	memset(&in, 0, sizeof(in));
	in.now_ms = r->t;
	in.now_us = r->t * 1000U;
	in.count[0] = (int32_t)floor(r->cnt[0]);
	in.count[1] = (int32_t)floor(r->cnt[1]);
	in.tick = true;
	in.cmd_stop = cmd_stop;
	in.fault_stop = fault_stop;
	in.ncmd = (cmd != NULL) ? 1U : 0U;
	in.cmd = cmd;
	mc_core_step(&r->c, &in, out);
}

static void rig_init(RIG *r)
{
	int32_t z[2] = {0, 0};

	memset(r, 0, sizeof(*r));
	r->sign[0] = 1;
	r->sign[1] = 1;
	mc_core_init(&r->c, &k_mc, z);
}

static void test_motion(void)
{
	RIG	r;
	MC_OUT	out;
	MC_CMD	cmd;
	MC_IN	in;
	int	i, done = 0, ack = 0, fault = 0;
	int32_t	first_target, integ, stop_mm;
	uint32_t k;

	/* distance move */
	rig_init(&r);
	memset(&cmd, 0, sizeof(cmd));
	cmd.type = MC_CMD_DISTANCE;
	cmd.a = 500;
	cmd.b = 200;
	cmd.c = 7;
	rig_tick(&r, &out, &cmd, false, false);
	for (i = 0; (i < 400) && !done; i++) {
		rig_tick(&r, &out, NULL, false, false);
		if (has_event(&out.ev, EV_DONE) && (out.ev.ev[0].p2 == 7)) {
			done = 1;
		}
	}
	check("MC-01", done && (fabs((r.cnt[0] + r.cnt[1]) / 2.0 - 1000.0) < 20.0),
	      "500 mm move: DONE with its id, within 10 mm");
	check("MC-02", fabs(r.cnt[0] - r.cnt[1]) < 10.0, "both wheels travel the same (straight)");
	check("MC-03", (r.c.snap.odo_mm >= 495) && (r.c.snap.odo_mm <= 510) && (r.c.snap.dist_mm == r.c.snap.odo_mm),
	      "odometer and net distance agree");

	/* turn */
	rig_init(&r);
	cmd.type = MC_CMD_TURN;
	cmd.a = 90;
	cmd.b = 0;
	cmd.c = 8;
	done = 0;
	rig_tick(&r, &out, &cmd, false, false);
	for (i = 0; (i < 400) && !done; i++) {
		rig_tick(&r, &out, NULL, false, false);
		done = has_event(&out.ev, EV_DONE);
	}
	check("MC-04", done && (r.cnt[0] < 0.0) && (r.cnt[1] > 0.0) &&
	      (fabs(r.cnt[1] - MC_COUNTS_PER_90DEG) < 8.0) && (fabs(-r.cnt[0] - MC_COUNTS_PER_90DEG) < 8.0),
	      "left turn 90: wheels opposite, each about counts_per_90deg");

	/* speed hold ramps up; stop wins at once */
	rig_init(&r);
	cmd.type = MC_CMD_SPEED;
	cmd.a = 200;
	cmd.b = 200;
	rig_tick(&r, &out, &cmd, false, false);
	first_target = r.c.w[0].target;
	for (i = 0; i < 50; i++) {
		rig_tick(&r, &out, NULL, false, false);
	}
	check("MC-05", (first_target <= 13) && (abs(r.c.w[0].speed - 200) < 25),
	      "speed hold ramps up gently, then holds 200 mm/s");
	memset(&in, 0, sizeof(in));
	in.cmd_stop = true;
	in.count[0] = (int32_t)r.cnt[0];
	in.count[1] = (int32_t)r.cnt[1];
	mc_core_step(&r.c, &in, &out);			/* the wake-up, no tick */
	check("MC-06", (out.duty[0] == 0) && (out.duty[1] == 0) && out.flush && (r.c.mode == MC_STOPPED),
	      "CMD_STOP: duty 0 on the wake-up itself, queue flushed");
	cmd.type = MC_CMD_SPEED;
	for (i = 0; (i < 50) && !ack; i++) {
		rig_tick(&r, &out, &cmd, true, false);
		ack = has_event(&out.ev, EV_STOP_ACK);
	}
	check("MC-07", ack && (r.c.rejected > 0U) && (r.c.w[0].duty == 0),
	      "STOP_ACK once stopped; commands refused while stopped");
	rig_tick(&r, &out, &cmd, true, true);
	rig_tick(&r, &out, &cmd, false, true);
	check("MC-08", r.c.mode == MC_FAULT, "FAULT_STOP wins over everything");
	rig_tick(&r, &out, NULL, false, false);
	check("MC-09", r.c.mode == MC_IDLE, "both bits clear: back to Idle");

	/* stall */
	rig_init(&r);
	r.stall = true;
	cmd.type = MC_CMD_SPEED;
	cmd.a = 300;
	cmd.b = 300;
	rig_tick(&r, &out, &cmd, false, false);
	for (i = 0; (i < 200) && !fault; i++) {
		rig_tick(&r, &out, NULL, false, false);
		fault = out.fault;
	}
	check("MC-10", fault && (r.c.snap.fault_code == MC_FLT_STALL) && (out.duty[0] == 0),
	      "blocked wheels: stall fault, motors off");
	rig_tick(&r, &out, NULL, false, false);
	check("MC-11", r.c.mode == MC_FAULT, "own fault holds until FAULT_STOP has been seen and cleared");

	/* encoder wired backwards */
	rig_init(&r);
	r.sign[0] = -1;
	fault = 0;
	rig_tick(&r, &out, &cmd, false, false);
	for (i = 0; (i < 200) && !fault; i++) {
		rig_tick(&r, &out, NULL, false, false);
		fault = out.fault;
	}
	check("MC-12", fault && (r.c.snap.fault_code == MC_FLT_ENCODER), "encoder backwards: detected, not a runaway");

	memset(&cmd, 0, sizeof(cmd));
	cmd.type = MC_CMD_DISTANCE;
	check("MC-13", !mc_cmd_valid(&cmd, MC_MAX_SPEED), "zero distance invalid");
	cmd.type = 9U;
	check("MC-14", !mc_cmd_valid(&cmd, MC_MAX_SPEED), "unknown command type invalid");
	rig_init(&r);
	mc_core_tune(&r.c, 1500, -1, 1100, 400);
	check("MC-15", (r.c.p.kp_x1000 == 1500) && (r.c.p.ki_x1000 == MC_KI_X1000) &&
	      (r.c.p.counts_per_m == (MC_COUNTS_PER_M * 11) / 10) && (r.c.p.counts_per_90deg == MC_COUNTS_PER_90DEG),
	      "tune: valid values used, unset (-1) and out-of-range ignored");

	/* a speed hold between ticks is applied at once (feedback IS32:
	   steering latency); the integrators only move on the tick */
	rig_init(&r);
	memset(&cmd, 0, sizeof(cmd));
	cmd.type = MC_CMD_SPEED;
	cmd.a = 200;
	cmd.b = 200;
	rig_tick(&r, &out, &cmd, false, false);
	for (i = 0; i < 50; i++) {
		rig_tick(&r, &out, NULL, false, false);
	}
	integ = r.c.w[0].integ;
	memset(&in, 0, sizeof(in));
	in.now_ms = r.t + 7U;
	in.now_us = in.now_ms * 1000U;
	in.count[0] = (int32_t)floor(r.cnt[0]);
	in.count[1] = (int32_t)floor(r.cnt[1]);
	cmd.a = 150;
	cmd.b = 250;
	cmd.c = (int32_t)(r.t + 4U);		/* the line reading was 3 ms earlier */
	in.ncmd = 1U;
	in.cmd = &cmd;
	mc_core_step(&r.c, &in, &out);		/* woken by the hold, no tick */
	check("MC-16", (r.c.w[0].target == 150) && (r.c.w[1].target == 250) && (out.duty[0] < out.duty[1]) &&
	      (r.c.w[0].integ == integ) && (r.c.snap.steer_lat_ms == 3) && (r.c.snap.steer_lat_max_ms == 3),
	      "speed hold between ticks: new targets and duties at once, latency logged");

	/* stopping distance and stop latency are logged */
	ack = 0;
	stop_mm = -1;
	for (i = 0; (i < 50) && !ack; i++) {
		rig_tick(&r, &out, NULL, true, false);
		for (k = 0U; k < out.ev.n; k++) {
			if (out.ev.ev[k].type == EV_STOP_ACK) {
				ack = 1;
				stop_mm = out.ev.ev[k].p2;
			}
		}
	}
	check("MC-17", ack && (stop_mm > 0) && (stop_mm < 60) && (r.c.snap.stop_dist_mm == stop_mm) &&
	      (r.c.snap.stop_lat_max_us > 0), "STOP_ACK carries the stopping distance; stop latency logged");
}

/* ===== IMU maths and terrain ========================================================= */
static const IMU_PARAMS k_imu = {
	IMU_AXIS_FWD, IMU_SIGN_FWD, IMU_AXIS_UP, IMU_SIGN_UP, IMU_AXIS_LAT, IMU_SIGN_LAT,
	IMU_CAL_SAMPLES, IMU_LPF_SHIFT, IMU_CLIMB_CDEG, IMU_LEVEL_CDEG, IMU_TERRAIN_MM, IMU_HUMP_END_MM, IMU_HUMP_END_MS,
	IMU_PRE_SAMPLES, IMU_HUMP_MIN_MM, IMU_WHEELBASE_MM, IMU_IMPACT_MG, IMU_IMPACT_WINDOW_MS,
	IMU_IMPACT_DROP_PCT, IMU_IMPACT_MIN_SPEED, IMU_STILL_SPEED, IMU_STILL_DEV_MG,
	IMU_ACCEL_MM_S2, IMU_TURN_CDEG_S, IMU_CLASS_HOLD, IMU_FAIL_LIMIT
};

static void imu_feed(IMU_CORE *c, double pitch_deg, double extra_fwd_mg, IMU_MOTION *m,
		     uint32_t *t, IMU_OUT *out, EVT_LIST *all)
{
	int32_t	a[3];
	double	p = (pitch_deg + 1.5) * 3.14159265 / 180.0;	/* board tilted 1.5 deg */
	uint32_t i;

	a[0] = (int32_t)lround(1000.0 * sin(p) + extra_fwd_mg);
	a[1] = 0;
	a[2] = (int32_t)lround(1000.0 * cos(p));
	*t += 10U;
	imu_core_step(c, a, true, m, *t, out);
	for (i = 0U; (i < out->ev.n) && (all->n < EVT_LIST_MAX); i++) {
		all->ev[all->n++] = out->ev.ev[i];
	}
}

static void test_imu(void)
{
	int32_t		e, maxe = 0, d;
	uint32_t	x, t = 0U;
	IMU_CORE	c;
	IMU_OUT		out;
	IMU_MOTION	m;
	EVT_LIST	all;
	int		i, terr_changes = 0, humps = 0, impact = 0;
	int32_t		h = 0;

	for (d = -17999; d <= 17999; d += 7) {
		double r = d * 3.14159265358979 / 18000.0;

		e = imu_atan2_cdeg((int32_t)lround(1000.0 * sin(r)), (int32_t)lround(1000.0 * cos(r))) - d;
		e = (e > 18000) ? (e - 36000) : ((e < -18000) ? (e + 36000) : e);	/* +180 = -180 */
		if (abs(e) > maxe) {
			maxe = abs(e);
		}
	}
	check("IMU-01", maxe <= 7, "atan2 within 0.07 degree all round (at 1000 mg resolution)");
	maxe = 0;
	for (d = -9000; d <= 9000; d += 13) {
		e = imu_sin_x1e4(d) - (int32_t)lround(10000.0 * sin(d * 3.14159265358979 / 18000.0));
		if (abs(e) > maxe) {
			maxe = abs(e);
		}
	}
	check("IMU-02", maxe <= 3, "sine within 0.0003");
	for (x = 0U, e = 0; x < 2000000U; x += 997U) {
		uint32_t q = imu_isqrt(x);

		if ((q * q > x) || ((q + 1U) * (q + 1U) <= x)) {
			e = 1;
		}
	}
	check("IMU-03", e == 0, "integer square root exact");

	imu_core_init(&c, &k_imu);
	memset(&m, 0, sizeof(m));
	m.valid = true;
	all.n = 0U;
	for (i = 0; i < 100; i++) {
		imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
	}
	check("IMU-04", c.calibrated && (abs(c.base_cdeg - 150) <= 3), "calibration finds the 1.5 degree mounting tilt");
	for (i = 0; i < 50; i++) {
		imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
	}
	check("IMU-05", abs(c.pitch_cdeg) <= 20, "pitch reads level after calibration");

	/* braking: a strong forward reading for 60 ms, but no travel */
	all.n = 0U;
	for (i = 0; i < 6; i++) {
		imu_feed(&c, 0.0, -400.0, &m, &t, &out, &all);
	}
	for (i = 0; i < 40; i++) {
		imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
	}
	check("IMU-06", (c.terrain == TERRAIN_LEVEL) && !c.in_hump, "braking jolt without travel is not a slope");

	/* a 20 mm high, 300 mm long hump at 150 mm/s: pitch follows the slope */
	m.speed_mm_s = 150;
	all.n = 0U;
	for (i = 0; i < 400; i++) {
		double s_mm = i * 1.5;
		double slope = (s_mm < 300.0) ? (20.0 * 3.14159265 / 300.0) * sin(2.0 * 3.14159265 * s_mm / 300.0) : 0.0;

		m.dist_mm = (int32_t)s_mm;
		imu_feed(&c, atan(slope) * 180.0 / 3.14159265, 0.0, &m, &t, &out, &all);
		if (has_event(&out.ev, EV_TERRAIN)) {
			terr_changes++;
		}
		if (has_event(&out.ev, EV_HUMP)) {
			humps++;
			h = out.ev.ev[out.ev.n - 1U].p1;
		}
	}
	check("IMU-07", terr_changes >= 2, "climbing then level / descending reported");
	check("IMU-08", (humps == 1) && (h >= 15) && (h <= 25), "one hump, height 20 mm +/- 5");
	check("IMU-09", c.max_hump_mm == h, "highest hump of the run kept");

	/* collision: jolt and the wheels stop */
	all.n = 0U;
	m.speed_mm_s = 200;
	imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
	imu_feed(&c, 0.0, -2500.0, &m, &t, &out, &all);
	m.speed_mm_s = 20;
	for (i = 0; i < 5; i++) {
		imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
		impact |= out.impact ? 1 : 0;
	}
	check("IMU-10", impact && has_event(&all, EV_IMPACT), "jolt plus a sudden speed drop: collision");
	impact = 0;
	m.speed_mm_s = 200;
	for (i = 0; i < 60; i++) {
		imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
	}
	imu_feed(&c, 0.0, -2500.0, &m, &t, &out, &all);
	for (i = 0; i < 30; i++) {
		imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
		impact |= out.impact ? 1 : 0;
	}
	check("IMU-11", !impact && (c.bumps > 0U), "jolt while the car keeps going: a bump, not a collision");

	/* motion classes */
	m.yaw_cdeg_s = 6000;
	for (i = 0; i < 10; i++) {
		imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
	}
	check("IMU-12", c.motion == MOTION_TURNING, "60 deg/s: turning");
	m.yaw_cdeg_s = 0;
	m.speed_mm_s = 0;
	m.accel_mm_s2 = 0;
	for (i = 0; i < 40; i++) {
		imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
	}
	check("IMU-13", c.motion == MOTION_STATIONARY, "no speed, no vibration: stationary");
	for (i = 0; i < 5; i++) {
		imu_core_step(&c, NULL, false, &m, t, &out);
	}
	check("IMU-14", out.fault, "five failed reads in a row: IMU fault");
}

/* ===== safety monitor ========================================================= */
static void test_safety(void)
{
	SM_CORE		s;
	SM_PARAMS	p = {{100U, 50U, 0U, 0U, 0U}, 2U};
	SM_OUT		o;
	uint32_t	age[HEALTH_SLOTS] = {10U, 10U, 0U, 0U, 0U};
	bool		alive[HEALTH_SLOTS] = {true, true, false, false, false};

	sm_init(&s, &p);
	sm_on_poll(&s, 0U, 1U, age, alive, &o);
	sm_on_poll(&s, 1U, 1U, age, alive, &o);
	check("SM-01", o.ev.n == 0U, "a one-sample press is ignored (debounce)");
	sm_on_poll(&s, 0U, 1U, age, alive, &o);
	sm_on_poll(&s, 0U, 1U, age, alive, &o);
	check("SM-02", (o.ev.n == 1U) && (o.ev.ev[0].type == EV_BUTTON) && (o.ev.ev[0].p1 == BTN_START),
	      "START after two samples");
	age[1] = 80U;
	sm_on_poll(&s, 1U, 1U, age, alive, &o);
	check("SM-03", o.raise == FLT_HEALTH, "a silent task raises FLT_HEALTH");
	sm_on_poll(&s, 1U, 1U, age, alive, &o);
	check("SM-04", o.raise == 0U, "... once, not every poll");
	sm_on_flags(&s, FLT_HEALTH, &o);
	sm_on_poll(&s, 0U, 1U, age, alive, &o);
	sm_on_poll(&s, 0U, 1U, age, alive, &o);
	check("SM-05", o.set_restart && (o.ev.n == 0U), "START while faulted is a RESTART request");
}

/* ===== obstacle profile and scan ================================================= */
static void test_profile(void)
{
	int32_t		ang[25], rng[25];
	uint32_t	n = 0U;
	int32_t		a;
	OS_PROFILE	p;

	/* a 100 mm box 300 mm ahead, centred; nothing else.  The 15 degree
	   beam sees the box's face (300 mm) while any part of it points there. */
	for (a = 30; a <= 150; a += 5) {
		double r = 3.14159265 * a / 180.0;
		double xr = 300.0 * cos(r) / sin(r);

		ang[n] = a;
		rng[n] = (fabs(xr) <= (50.0 + 300.0 * tan(7.5 * 3.14159265 / 180.0))) ? 300 : OS_INVALID;
		n++;
	}
	os_build_profile(ang, rng, n, OS_R_MAX_MM, OS_CORRIDOR_MM, &p);
	check("OP-01", p.valid && (p.closest_mm >= 280) && (p.closest_mm <= 300) &&
	      (p.x_left_mm <= -50) && (p.x_right_mm >= 50) && (p.width_mm <= 180),
	      "centred box: never narrower than it is, closest point about right");
	check("OP-02", (p.left_clear_mm == OS_CLEAR_OPEN) && (p.unknown_left > 0U) && (p.unknown_right > 0U),
	      "no echo either side: open, but counted as unknown");
	/* a wall on the left, 150 mm closer than the box */
	rng[24] = 180;		/* 150 deg */
	rng[23] = 190;		/* 145 deg */
	os_build_profile(ang, rng, n, OS_R_MAX_MM, OS_CORRIDOR_MM, &p);
	check("OP-03", p.valid && (p.closest_mm >= 280) && (p.left_clear_mm < OS_CLEAR_OPEN),
	      "the box in the path is the obstacle, the wall beside it limits the left gap");
	/* one missing echo in the middle of the box */
	rng[24] = OS_INVALID;
	rng[23] = OS_INVALID;
	rng[12] = OS_INVALID;	/* 90 deg */
	os_build_profile(ang, rng, n, OS_R_MAX_MM, OS_CORRIDOR_MM, &p);
	check("OP-04", p.valid && (p.x_left_mm <= -50) && (p.x_right_mm >= 50),
	      "a missing echo inside the box does not split it");
	n = 0U;
	os_build_profile(ang, rng, n, OS_R_MAX_MM, OS_CORRIDOR_MM, &p);
	check("OP-05", !p.valid, "no readings: no profile (not a clear path)");
}

static const SCAN_PARAMS k_sc = {
	OS_STOP_MM, OS_SLOW_MM, OS_SLOW_HYST_MM, OS_CONFIRM, OS_GAP_MS, OS_SETTLE_BASE_MS,
	OS_SETTLE_MS_PER_DEG, OS_RETRIES, OS_R_MAX_MM, OS_CORRIDOR_MM, OS_COARSE_STEP, OS_FINE_STEP,
	OS_FINE_MARGIN, OS_MISSING_LIMIT, OS_PROFILE_MAX_MS
};

static void test_scan(void)
{
	SCAN_CORE	s;
	SCAN_IN		in;
	SCAN_OUT	out;
	int		i, triggers = 0, alert = 0, profile = 0, fault = 0, visited[181];
	int		gap_ok = 1;
	uint32_t	last_trig = 0U;
	int		pending = 0;
	int		stop_with_alert = 0, stray_stop = 0;

	memset(visited, 0, sizeof(visited));
	scan_core_init(&s, &k_sc);
	memset(&in, 0, sizeof(in));
	in.have_req = true;
	in.req.mode = SCAN_WATCH | SCAN_STOP_FLAG;
	in.req.id = 5;
	for (i = 0; i < 100; i++) {
		in.now_ms = (uint32_t)(i * 10);
		in.echo = pending ? SCAN_ECHO_DONE : SCAN_ECHO_NONE;
		in.range_mm = 200;
		pending = 0;
		scan_core_step(&s, &in, &out);
		in.have_req = false;
		if (out.trigger) {
			if (triggers && ((in.now_ms - last_trig) < OS_GAP_MS)) {
				gap_ok = 0;
			}
			triggers++;
			last_trig = in.now_ms;
			pending = 1;
		}
		if (has_event(&out.ev, EV_ALERT)) {
			alert++;
			stop_with_alert += out.stop_motors ? 1 : 0;
		} else if (out.stop_motors) {
			stray_stop++;
		}
	}
	check("SC-01", gap_ok && (triggers >= 10), "watch: pings at least 70 ms apart");
	check("SC-02", alert == 1, "two close readings: one ALERT, not repeated");
	check("SC-08", (stop_with_alert == 1) && (stray_stop == 0),
	      "WATCH with the stop flag: the ALERT also stops the car, once");

	/* profile against a box 250 mm ahead, 120 mm wide */
	in.have_req = true;
	in.req.mode = SCAN_PROFILE;
	in.req.id = 6;
	for (i = 100; (i < 1100) && !profile; i++) {
		in.now_ms = (uint32_t)(i * 10);
		if (pending) {
			double r = 3.14159265 * s.servo_deg / 180.0;
			double xr = 250.0 * cos(r) / sin(r);

			in.echo = (fabs(xr) <= 60.0) ? SCAN_ECHO_DONE : SCAN_ECHO_FAR;
			in.range_mm = (int32_t)lround(250.0 / sin(r));
		} else {
			in.echo = SCAN_ECHO_NONE;
		}
		pending = 0;
		scan_core_step(&s, &in, &out);
		in.have_req = false;
		if (out.trigger) {
			pending = 1;
			visited[s.servo_deg] = 1;
		}
		if (has_event(&out.ev, EV_PROFILE)) {
			profile = 1;
		}
	}
	check("SC-03", profile && s.profile.valid && (s.profile.closest_mm <= 250) && (s.profile.closest_mm > 230),
	      "profile: box found 250 mm ahead");
	check("SC-04", visited[30] && visited[60] && visited[120] && visited[150] && visited[85] && visited[95],
	      "coarse 30..150 and fine 5-degree steps around the box");
	check("SC-05", (i - 100) * 10 < (int)OS_PROFILE_MAX_MS, "profile finished well within its time limit");
	check("SC-06", s.servo_deg == 90, "servo back to the front afterwards");

	/* sensor not answering */
	scan_core_init(&s, &k_sc);
	in.have_req = true;
	in.req.mode = SCAN_WATCH;
	for (i = 0; (i < 300) && !fault; i++) {
		in.now_ms = (uint32_t)(i * 10);
		in.echo = pending ? SCAN_ECHO_MISSING : SCAN_ECHO_NONE;
		pending = 0;
		scan_core_step(&s, &in, &out);
		in.have_req = false;
		pending = out.trigger ? 1 : 0;
		fault = out.fault;
	}
	check("SC-07", fault, "no response ten times in a row: sensor fault");

	/* without the flag (driving past a known obstacle): ALERT only */
	scan_core_init(&s, &k_sc);
	memset(&in, 0, sizeof(in));
	in.have_req = true;
	in.req.mode = SCAN_WATCH;
	in.req.id = 9;
	alert = 0;
	stray_stop = 0;
	pending = 0;
	for (i = 0; i < 40; i++) {
		in.now_ms = (uint32_t)(i * 10);
		in.echo = pending ? SCAN_ECHO_DONE : SCAN_ECHO_NONE;
		in.range_mm = 200;
		pending = 0;
		scan_core_step(&s, &in, &out);
		in.have_req = false;
		pending = out.trigger ? 1 : 0;
		alert += has_event(&out.ev, EV_ALERT) ? 1 : 0;
		stray_stop += out.stop_motors ? 1 : 0;
	}
	check("SC-09", (alert == 1) && (stray_stop == 0), "WATCH without the stop flag: ALERT, no stop");
}

/* ===== vehicle controller ======================================================= */
static const VC_PARAMS k_vc = {
	VC_BASE_SPEED, VC_SLOW_SPEED, VC_HUMP_SPEED, VC_JUNCTION_SPEED, VC_MOVE_SPEED,
	VC_FIND_SPEED, VC_CAR_WIDTH_MM, VC_MARGIN_MM, VC_US_TO_AXLE_MM, VC_LINE_TO_AXLE_MM,
	VC_REAR_CLEAR_MM, VC_OBST_DEPTH_MM, VC_LINE_HALF_MM, VC_STOP_MM, VC_BYPASS_STOP_MM,
	VC_BYPASS_DEG, VC_BACK_MM, VC_N_RETRY, VC_TIE_MM, VC_PREF_SIDE, VC_FIND_EXTRA_MM,
	VC_ALIGN_EXTRA_MM, VC_D_EXEC_MM, VC_JUNCTION_MUTE_MM, VC_CROSS_MM, VC_ACQUIRE_MM, VC_T_STARTUP_MS,
	VC_T_ACK_MS, VC_T_SCAN_MS, VC_T_SEARCH_MS, VC_T_EXEC_MS, VC_T_BYPASS_MS,
	VC_HUMP_GRACE_MS, VC_HOLD_REFRESH_MS
};
static const LS_PARAMS k_ls = {LS_D_MIN, LS_D_RATE, LS_D_MAX, LS_D_BIAS, LS_D_SEARCH, LS_T_ESCAPE_MS, LS_T_DRIFT_MS,
			       LB_T_LOST_MS, LS_SLOW_PCT, LS_START_SIDE};

static int32_t g_odo;		/* odometer the controller is told about */

static void ev(VC_CORE *v, uint8_t type, int32_t p1, int32_t p2, uint32_t t, VC_ACTIONS *a)
{
	APP_MSG m;

	memset(&m, 0, sizeof(m));
	m.type = type;
	m.p1 = p1;
	m.p2 = p2;
	vc_event(v, &m, t, g_odo, a);
}

static const VC_ACTION *find(const VC_ACTIONS *a, uint8_t type)
{
	uint32_t i;

	for (i = 0U; i < a->n; i++) {
		if (a->a[i].type == type) {
			return &a->a[i];
		}
	}
	return NULL;
}

/* bring a controller to SCANNING and hand it a profile */
static void to_scanning(VC_CORE *v, VC_ACTIONS *a)
{
	vc_init(v, &k_vc, &k_ls, 0U);
	vc_tick(v, 20U, 0, true, a);
	ev(v, EV_COMMAND, CMD_START, 0, 30U, a);
	ev(v, EV_ALERT, 240, v->watch_id, 40U, a);
	ev(v, EV_STOP_ACK, 100, 0, 60U, a);
}

static int bypass_side(VC_CORE *v, OS_PROFILE *p, VC_ACTIONS *a)
{
	APP_MSG m;

	memset(&m, 0, sizeof(m));
	m.type = EV_PROFILE;
	m.p1 = 1;
	m.p2 = v->scan_id;
	vc_profile(v, &m, p, 100U, 0, a);
	return (v->state == VC_BYPASS) ? v->side : 0;
}

static void test_vc(void)
{
	VC_CORE			v;
	VC_ACTIONS		a;
	OS_PROFILE		p;
	const VC_ACTION		*x;
	int32_t			id;
	bool			ok;
	APP_MSG			m;

	vc_init(&v, &k_vc, &k_ls, 0U);
	ev(&v, EV_COMMAND, CMD_START, 0, 10U, &a);
	check("VC-01", (v.state == VC_STARTUP) && (find(&a, VA_TEL) != NULL), "START before the sensors are ready: refused");
	vc_tick(&v, 20U, 0, true, &a);
	ev(&v, EV_COMMAND, CMD_START, 0, 30U, &a);
	check("VC-02", (v.state == VC_FOLLOW) && (find(&a, VA_CLEAR_STOP) != NULL) &&
	      (find(&a, VA_SCAN) != NULL) && (find(&a, VA_HOLD) != NULL), "START: release, watch ahead, drive");
	g_odo = 100;
	ev(&v, EV_BARCODE, 'A', LB_CMD_TURN_LEFT, 40U, &a);
	g_odo = 500;
	ev(&v, EV_JUNCTION, 500, 0, 50U, &a);
	x = find(&a, VA_MOVE);
	check("VC-03", (v.state == VC_EXEC) && (x != NULL) && (x->a == VC_LINE_TO_AXLE_MM + VC_LINE_HALF_MM),
	      "barcode A, then a junction: drive the axle onto the junction");
	id = (x != NULL) ? x->c : 0;
	ev(&v, EV_DONE, MC_MOVE_DISTANCE, id + 99, 60U, &a);
	check("VC-04", find(&a, VA_TURN) == NULL, "a DONE with the wrong id is ignored");
	ev(&v, EV_DONE, MC_MOVE_DISTANCE, id, 70U, &a);
	x = find(&a, VA_TURN);
	check("VC-05", (x != NULL) && (x->a == 90), "then turn left 90");
	ev(&v, EV_DONE, MC_MOVE_TURN, (x != NULL) ? x->c : 0, 80U, &a);
	check("VC-06", (v.state == VC_FOLLOW) && (v.commands_done == 1U), "then follow the new branch");
	g_odo = 0;
	ev(&v, EV_COMMAND, CMD_STOP, 0, 90U, &a);
	check("VC-07", (v.state == VC_READY) && (find(&a, VA_STOP) != NULL), "STOP: READY, motors stopped");
	ev(&v, EV_FAULT, (int32_t)FLT_SCAN, 0, 100U, &a);
	check("VC-08", v.state == VC_SAFE_STOP, "a fault: Safe Stop from any state");
	ev(&v, EV_COMMAND, CMD_START, 0, 110U, &a);
	check("VC-09", v.state == VC_SAFE_STOP, "START refused while faulted");
	ev(&v, EV_COMMAND, CMD_RESTART, 0, 120U, &a);
	check("VC-10", find(&a, VA_RESTART) != NULL, "RESTART asks Safety Monitor to clear");
	ev(&v, EV_FAULT_CLEAR, 0, 0, 130U, &a);
	check("VC-11", v.state == VC_READY, "faults cleared: READY");

	memset(&p, 0, sizeof(p));
	p.valid = true;
	p.closest_mm = 240;
	p.left_clear_mm = OS_CLEAR_OPEN;
	p.right_clear_mm = OS_CLEAR_OPEN;
	p.x_left_mm = -60;
	p.x_right_mm = 60;
	to_scanning(&v, &a);
	check("VC-12", bypass_side(&v, &p, &a) == VC_PREF_SIDE, "centred box: the preferred side");
	p.x_left_mm = -20;
	p.x_right_mm = 100;
	to_scanning(&v, &a);
	check("VC-13", bypass_side(&v, &p, &a) == -1, "box sticking out right: pass on the left (shorter)");
	p.x_left_mm = -60;
	p.x_right_mm = 60;
	p.left_clear_mm = 120;
	to_scanning(&v, &a);
	check("VC-14", bypass_side(&v, &p, &a) == 1, "no room on the left: pass on the right");
	p.left_clear_mm = OS_CLEAR_OPEN;
	p.unknown_left = 4U;
	to_scanning(&v, &a);
	check("VC-15", bypass_side(&v, &p, &a) == 1, "equal detours: the side with fewer missing echoes");
	p.unknown_left = 0U;
	p.x_left_mm = 150;
	p.x_right_mm = 250;
	to_scanning(&v, &a);
	(void)bypass_side(&v, &p, &a);
	check("VC-16", (v.state == VC_FOLLOW) && (find(&a, VA_CLEAR_STOP) != NULL), "box beside the path: continue");
	p.x_left_mm = -60;
	p.x_right_mm = 60;
	p.left_clear_mm = 100;
	p.right_clear_mm = 100;
	to_scanning(&v, &a);
	(void)bypass_side(&v, &p, &a);
	x = find(&a, VA_MOVE);
	check("VC-17", (v.state == VC_REVERSE) && (x != NULL) && (x->a == -VC_BACK_MM), "no room either side: reverse");
	v.retries = VC_N_RETRY;
	v.state = VC_SCANNING;
	(void)bypass_side(&v, &p, &a);
	check("VC-18", (v.state == VC_HALTED) && (v.why == VC_WHY_NO_ROUTE) && (find(&a, VA_FAULT) == NULL) &&
	      (find(&a, VA_STOP) != NULL), "still no room after the retries: HALTED (mission failure, no fault latched)");
	p.valid = false;
	to_scanning(&v, &a);
	(void)bypass_side(&v, &p, &a);
	check("VC-19", v.state == VC_REVERSE, "nothing seen clearly: back off and look again");
	to_scanning(&v, &a);
	vc_tick(&v, 60U + VC_T_SCAN_MS + 10U, 0, true, &a);
	x = find(&a, VA_SCAN);
	ok = (v.state == VC_SCANNING) && (x != NULL) && (x->a == (int32_t)SCAN_PROFILE);
	vc_tick(&v, 60U + (2U * VC_T_SCAN_MS) + 30U, 0, true, &a);
	check("VC-20", ok && (v.state == VC_HALTED) && (v.why == VC_WHY_SCAN_FAILED) && (find(&a, VA_FAULT) == NULL),
	      "no profile in time: scan once more, then HALTED");
	check("VC-21", (strcmp(vc_state_name(VC_FIND_LINE), "FIND_LINE") == 0) &&
	      (strcmp(vc_state_name(VC_HALTED), "HALTED") == 0), "state names");

	v.commands_done = 3U;
	ev(&v, EV_COMMAND, CMD_START, 0, 60U + (2U * VC_T_SCAN_MS) + 50U, &a);
	check("VC-22", (v.state == VC_FOLLOW) && (find(&a, VA_CLEAR_STOP) != NULL) && (v.commands_done == 3U),
	      "START after HALTED carries on with the same run (no RESTART needed)");

	/* Obstacle Scan stops the car itself, except while passing an
	   obstacle the controller decided to pass (Continue) */
	vc_init(&v, &k_vc, &k_ls, 0U);
	vc_tick(&v, 20U, 0, true, &a);
	ev(&v, EV_COMMAND, CMD_START, 0, 30U, &a);
	x = find(&a, VA_SCAN);
	ok = (x != NULL) && (x->a == (int32_t)(SCAN_WATCH | SCAN_STOP_FLAG));
	p.valid = true;
	p.x_left_mm = 150;
	p.x_right_mm = 250;
	p.left_clear_mm = OS_CLEAR_OPEN;
	p.right_clear_mm = OS_CLEAR_OPEN;
	to_scanning(&v, &a);
	(void)bypass_side(&v, &p, &a);
	x = find(&a, VA_SCAN);
	ok = ok && (v.state == VC_FOLLOW) && (x != NULL) && (x->a == (int32_t)SCAN_WATCH);
	vc_tick(&v, 200U, v.alert_mute_odo + 1, true, &a);
	x = find(&a, VA_SCAN);
	check("VC-23", ok && (x != NULL) && (x->a == (int32_t)(SCAN_WATCH | SCAN_STOP_FLAG)),
	      "WATCH asks Obstacle Scan to stop the car; not while passing a known obstacle, then again");

	memset(&m, 0, sizeof(m));
	m.type = EV_LINE_STATUS;
	m.p1 = 1;
	m.t_ms = 1234U;
	vc_event(&v, &m, 1236U, v.alert_mute_odo + 2, &a);
	x = find(&a, VA_HOLD);
	check("VC-24", (x != NULL) && (x->c == 1234), "a steering hold carries the time of its line reading");
}

/* ===== commands and telemetry =================================================== */
static void test_cmd(void)
{
	CMD		c;
	uint32_t	seq = 0U;

	check("CMD-01", (cmd_parse("1 start", 7U, &seq, true, &c) == CMD_OK) && (c.verb == CMD_START), "lower case verb");
	check("CMD-02", (cmd_parse("2 STOP\r\n", 8U, &seq, true, &c) == CMD_OK) && (c.verb == CMD_STOP), "CR LF tolerated");
	check("CMD-03", cmd_parse("2 STOP", 6U, &seq, true, &c) == CMD_ERR_REPEAT, "old sequence number refused");
	check("CMD-04", cmd_parse("3 STOP now", 10U, &seq, true, &c) == CMD_ERR_FORMAT, "trailing word refused");
	check("CMD-05", cmd_parse("x STOP", 6U, &seq, true, &c) == CMD_ERR_FORMAT, "no sequence number refused");
	check("CMD-06", cmd_parse("4 SET turn_cal 1600", 19U, &seq, true, &c) == CMD_ERR_RANGE, "out of range");
	check("CMD-07", cmd_parse("4 SET Base_speed 200", 20U, &seq, true, &c) == CMD_ERR_NAME, "names are exact");
	check("CMD-08", (cmd_parse("4 SET dist_cal 980", 18U, &seq, true, &c) == CMD_OK) && (c.param == 4) && (c.value == 980),
	      "SET dist_cal 980");
	check("CMD-09", cmd_parse("5 SET kp_x1000 1200", 19U, &seq, false, &c) == CMD_ERR_STATE, "SET refused while running");
	check("CMD-10", cmd_parse("6 SET kp_x1000 1200 99999999999999999999999999999999999999999999999999", 70U, &seq, true, &c) ==
	      CMD_ERR_FORMAT, "longer than 64 bytes refused");
	check("CMD-11", cmd_parse("6 SET ki_x1000 -5", 17U, &seq, true, &c) == CMD_ERR_RANGE, "negative refused by range");
	check("CMD-12", cmd_parse("999999999999 START", 18U, &seq, true, &c) == CMD_ERR_FORMAT, "absurd number refused");
}

static void test_tel(void)
{
	TEL_DATA	d;
	char		buf[TEL_JSON_MAX];
	char		small[20];
	APP_MSG		m;
	TEL_SUMMARY	s;
	TEL_TIMING	tm;
	int32_t		n1, n2, n3, n4;
	uint32_t	k;
	TJ		j;

	memset(&d, 0xFF, sizeof(d));		/* every number as large as it gets */
	d.line_l = true;
	d.line_r = true;
	d.imu_ok = true;
	d.mqtt_up = true;
	d.state = VC_FIND_LINE;
	d.motion = MOTION_DECELERATING;
	d.barcode = '%';
	d.v[0] = INT32_MIN;
	d.v[1] = INT32_MIN;
	d.enc[0] = INT32_MIN;
	d.enc[1] = INT32_MIN;
	d.odo_mm = INT32_MIN;
	d.pitch_cdeg = INT32_MIN;
	d.hmax_mm = INT32_MIN;
	d.range_mm = INT32_MIN;
	d.yaw_cdeg_s = INT32_MIN;
	d.pending = INT32_MIN;
	n1 = tel_fmt_telemetry(&d, buf, TEL_JSON_MAX);
	n2 = tel_fmt_status(&d, buf, TEL_JSON_MAX);
	n3 = tel_fmt_profile(UINT32_MAX, true, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN,
			     UINT32_MAX, UINT32_MAX, buf, TEL_JSON_MAX);
	memset(&s, 0xFF, sizeof(s));
	s.dist_mm = INT32_MIN;
	s.hmax_mm = INT32_MIN;
	n4 = tel_fmt_summary(&s, buf, TEL_JSON_MAX);
	printf("        largest possible: telemetry %d, status %d, profile %d, summary %d bytes\n", n1, n2, n3, n4);
	check("TEL-01", (n1 > 0) && (n2 > 0) && (n3 > 0) && (n4 > 0),
	      "every periodic message fits in 200 bytes even with the largest numbers");
	memset(&m, 0, sizeof(m));
	m.type = EV_BARCODE;
	m.p1 = '"';
	n1 = tel_fmt_event(&m, buf, TEL_JSON_MAX);
	buf[(n1 > 0) ? n1 : 0] = '\0';
	check("TEL-02", (n1 > 0) && (strstr(buf, "\"c\":\"\\\"\"") != NULL), "a quote inside a string is escaped");
	m.type = EV_LINE_STATUS;
	check("TEL-03", tel_fmt_event(&m, buf, TEL_JSON_MAX) == 0, "line status is in the telemetry, not an event");
	tj_begin(&j, small, sizeof(small));
	tj_str(&j, "message", "far too long for this buffer");
	check("TEL-04", tj_end(&j) == -1, "a message that does not fit is refused, never cut short");
	d.v[0] = 212;
	d.v[1] = 208;
	n1 = tel_fmt_telemetry(&d, buf, TEL_JSON_MAX);
	buf[(n1 > 0) ? n1 : 0] = '\0';
	check("TEL-05", strstr(buf, "\"v\":[212,208]") != NULL, "wheel speeds as a pair");

	memset(&tm, 0, sizeof(tm));
	tm.t_ms = 0xFFFFFFFFU;
	for (k = 0U; k < TEL_TIM_TASKS; k++) {
		tm.max_us[k] = 0xFFFFFFFFU;
	}
	tm.overruns = 0xFFFFFFFFU;
	tm.irq_per_s = 0xFFFFFFFFU;
	tm.irq_max_us = 0xFFFFFFFFU;
	tm.vq_high = 0xFFFFFFFFU;
	tm.dropped = 0xFFFFFFFFU;
	tm.steer_lat_max_ms = INT32_MAX;
	tm.stop_lat_max_us = INT32_MIN;
	tm.stop_dist_mm = INT32_MAX;
	tm.reconnect_ms = 0xFFFFFFFFU;
	n1 = tel_fmt_timing(&tm, buf, TEL_JSON_MAX);
	check("TEL-06", (n1 > 0) && (strstr(buf, "\"ev\":\"tim\"") != NULL),
	      "timing message fits in 200 bytes with the largest values");

	check("EVT-01", evt_is_safety(EV_FAULT) && evt_is_safety(EV_IMPACT) && evt_is_safety(EV_ALERT) &&
	      evt_is_safety(EV_STOP_ACK) && evt_is_safety(EV_BUTTON) && evt_is_safety(EV_COMMAND) &&
	      evt_is_safety(EV_FAULT_CLEAR) && !evt_is_safety(EV_LINE_STATUS) && !evt_is_safety(EV_JUNCTION) &&
	      !evt_is_safety(EV_DONE) && !evt_is_safety(EV_TERRAIN) && !evt_is_safety(EV_RANGE_ZONE),
	      "queue policy: safety events must arrive, the rest may be dropped and counted");
}

int main(void)
{
	test_code39();
	test_barcode();
	test_line();
	test_motion();
	test_imu();
	test_safety();
	test_profile();
	test_scan();
	test_vc();
	test_cmd();
	test_tel();
	printf("\nUNIT: %d checks, %d failed\n", g_n, g_fail);
	return (g_fail == 0) ? 0 : 1;
}
