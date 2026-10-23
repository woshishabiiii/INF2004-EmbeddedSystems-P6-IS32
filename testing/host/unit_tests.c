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
#include "../../app_program/subsystem1/tel_bin.h"
#include "../../app_program/bitpack.h"
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

/* feed "*c*" with narrow width n us; glitch = add a 100 us chatter pair in element 4.
   Times are 31-bit edge times as drv_barcode.c stores them, starting at g_bc_t0. */
static uint32_t g_bc_t0 = 1000U;

#define T31(x)	((uint32_t)(x) & 0x7FFFFFFFU)

static bool feed(BC_CORE *b, char c, uint32_t n, bool glitch, bool merged, BC_RESULT *res)
{
	uint32_t	w[CODE39_ELEMENTS], t = g_bc_t0, i;
	uint8_t		lvl = 1U;

	(void)code39_encode(c, n, 3U * n, w);
	for (i = 0U; i < CODE39_ELEMENTS; i++) {
		bc_edge(b, T31(t), lvl);
		if (glitch && (i == 4U)) {
			bc_edge(b, T31(t + (w[i] / 2U)), (uint8_t)(lvl ^ 1U));
			bc_edge(b, T31(t + (w[i] / 2U) + 100U), lvl);
		}
		if (merged && (i == 6U)) {
			bc_edge(b, T31(t + (w[i] / 2U)), lvl);	/* an edge that changes nothing */
		}
		t += w[i];
		lvl ^= 1U;
	}
	bc_edge(b, T31(t), 0U);
	if (bc_poll(b, T31(t + 1000U), res)) {
		return false;			/* must not end before the quiet time */
	}
	return bc_poll(b, T31(t + 1000000U), res);
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
	/* wraparound: the capture starts 0.2 s before the 31-bit time wraps */
	g_bc_t0 = 0x80000000U - 200000U;
	bc_init(&b, &k_bc);
	check("BC-07", feed(&b, 'B', 15000U, false, false, &r) && (r.result == C39_OK) && (r.c == 'B') &&
	      (r.narrow_us == 15000U), "a barcode across the 31-bit time wrap decodes, widths unchanged");
	bc_init(&b, &k_bc);
	bc_edge(&b, 0x7FFFF000U | 0x80000000U, 1U);	/* the level bit left in: masked */
	bc_edge(&b, 0x00001000U, 0U);
	check("BC-08", (b.n == 2U) && (b.widest_us == 0x2000U),
	      "bp_dt31: edge times are masked before and after subtracting");
	g_bc_t0 = 1000U;
	{
		LC_STATUS st = {true, false, '_', 3U, 300U, 7U}, u;

		lc_status_unpack(lc_status_pack(&st), &u);
		check("BC-09", u.left_black && !u.right_black && (u.last_char == '_') && (u.reads_ok == 3U) &&
		      (u.reads_failed == 255U) && (u.junctions == 7U),
		      "line status word round-trips; counts stop at 255");
		st.last_char = '-';
		lc_status_unpack(lc_status_pack(&st), &u);
		check("BC-10", u.last_char == '-', "'-' (no barcode yet) fits the 6-bit character");
	}
	check("BC-06", (bc_command_for('A') == LB_CMD_TURN_LEFT) && (bc_command_for('B') == LB_CMD_TURN_RIGHT) &&
	      (bc_command_for('C') == LB_CMD_STRAIGHT) && (bc_command_for('D') == LB_CMD_U_TURN) &&
	      (bc_command_for('Z') == LB_CMD_NONE), "command table");
}

/* ===== line sensing and steering =============================================== */
static void test_line(void)
{
	LC_CORE		c;
	LC_PARAMS	p = {LB_JUNCTION_MS, LB_JUNCTION_MM, LB_LOST_MM};
	EVT_LIST	ev;
	LS_STATE	s;
	LS_PARAMS	lp = {LS_D_MIN, LS_D_RATE, LS_RATE_MM, LS_D_MAX, LS_D_BIAS, LS_D_SEARCH, LS_ESCAPE_MM,
			      LS_DRIFT_MM, LB_LOST_MM, LS_SLOW_PCT, LS_START_SIDE};
	LS_OUT		o, o2;
	int32_t		bias200 = (200 * LS_D_BIAS) / 1000;	/* the lean at 200 mm/s, mm/s */
	int		ok;

	lc_init(&c, &p, 0);
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
	lc_step(&c, false, false, 90000U, 16, &ev);	/* a long wait, standing still */
	check("LC-05", !has_event(&ev, EV_LINE_LOST), "standing still off the line: not lost (T_lost is travel)");
	lc_step(&c, false, false, 90010U, 14 + LB_LOST_MM + 1, &ev);
	lc_step(&c, false, false, 90020U, 14 + LB_LOST_MM + 2, &ev);
	check("LC-04", has_event(&ev, EV_LINE_LOST) && (ev.ev[ev.n - 1U].p1 == 1),
	      "line lost after T_lost of travel, on the side last seen (right)");

	ls_init(&s, &lp, 0);
	ls_update(&s, true, false, 100);
	ls_output(&s, 100, 200, &o);
	ls_output(&s, 160, 200, &o2);
	check("LS-01", (o.v_left < o.v_right) && ((o2.v_right - o2.v_left) > (o.v_right - o.v_left)),
	      "left on black: steer left, harder the further it stays");
	ls_update(&s, false, true, 200);
	ls_output(&s, 200, 200, &o);
	check("LS-02", o.v_left > o.v_right, "right on black: steer right");
	ls_update(&s, true, true, 220);
	ls_output(&s, 220, 200, &o);
	check("LS-03", o.v_left == o.v_right, "both black: straight across");
	ls_update(&s, false, true, 240);
	ls_update(&s, false, false, 242);
	ls_output(&s, 244, 200, &o);
	check("LS-04", (o.v_left > o.v_right) && ((o.v_left - o.v_right) == 2 * bias200),
	      "both white just after the right sensor: lean gently right");
	ls_output(&s, 242 + LS_DRIFT_MM, 200, &o);
	ls_output(&s, 242 + (2 * LS_DRIFT_MM), 200, &o2);
	check("LS-05", ((o.v_left - o.v_right) == 2 * bias200) &&
	      ((o2.v_left - o2.v_right) == 4 * bias200) && (o2.v_left == 200) && !o2.lost,
	      "centred: the lean holds for drift_mm, then grows (doubled after 2 x drift_mm), full speed");
	ls_output(&s, 242 + LB_LOST_MM + 10, 200, &o);
	check("LS-06", o.lost && (o.side == 1), "white past T_lost of travel: lost, on the right");
	ls_update(&s, false, true, 3000);
	ls_update(&s, false, false, 3000 + LS_ESCAPE_MM + 5);
	ls_output(&s, 3000 + LS_ESCAPE_MM + 6, 200, &o);
	check("LS-07", (o.v_left == (200 * LS_SLOW_PCT) / 100) &&
	      ((o.v_left - o.v_right) == 2 * ((200 * LS_D_SEARCH) / 1000)),
	      "right sensor black a long way, then white: line went outside, slow and turn firmly right");
	ls_output(&s, 3000 + LS_ESCAPE_MM + 6, 300, &o2);
	check("LS-08", (o2.v_left <= 300) && (o2.v_right <= 300), "no wheel is asked for more than the base speed");

	/* LS-09: the same situation at half the speed gives the same PATH:
	   every correction is the same fraction of the speed, so the turn
	   per millimetre (wheel difference / speed) does not change. */
	ls_init(&s, &lp, 0);
	ls_update(&s, true, false, 100);
	ls_output(&s, 130, 200, &o);
	ls_output(&s, 130, 100, &o2);
	check("LS-09", ((o.v_right - o.v_left) == 2 * (o2.v_right - o2.v_left)) &&
	      (o.v_right == 200) && (o2.v_right == 100) && (o2.v_left >= 0),
	      "slower car: correction scales with the speed (same curvature), inside wheel never reverses");
}

/* ===== motion control ============================================================ */
static const MC_PARAMS k_mc = {
	MC_COUNTS_PER_M, MC_COUNTS_PER_90DEG, MC_TRACK_MM, MC_KFF_X1000, MC_DEADBAND,
	MC_KP_X1000, MC_KI_X1000, MC_MAX_SPEED, MC_MIN_SPEED, MC_ACCEL, MC_TURN_SPEED,
	MC_K_STRAIGHT, MC_SETTLE_SPEED, MC_STALL_DUTY, MC_STALL_SPEED, MC_STALL_TICKS,
	MC_TIMEOUT_MARGIN_MS, MC_SPEED_WINDOW, MC_DUTY_RISE, MC_STOP_COAST, MC_STEER_PM_MM
};

typedef struct {
	MC_CORE	c;
	double	cnt[2];
	double	v[2];
	uint32_t t;
	bool	stall;		/* the wheels cannot turn */
	int	sign[2];	/* -1 = encoder wired backwards */
	double	coast_keep;	/* > 0: while Motion Control coasts, speed kept per period
				   (1.0 = rolling downhill, coasting does not slow it) */
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
		if ((d == 0.0) && r->c.coasting && (r->coast_keep > 0.0)) {
			r->v[i] *= r->coast_keep;	/* free-wheeling */
		} else {
			r->v[i] += (vss - r->v[i]) * 0.02 / 0.06;
			if (d == 0.0) {
				r->v[i] *= 0.4;
			}
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
	check("MC-10", fault && (r.c.snap.fault_bits == MC_FLT_STALL) && (out.duty[0] == 0),
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
	check("MC-12", fault && (r.c.snap.fault_bits == MC_FLT_ENCODER), "encoder backwards: detected, not a runaway");

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
	{
		bool at_once = (r.c.w[0].target < 200) &&
			       (r.c.w[1].target >= 200) && (out.duty[0] < out.duty[1]) &&
			       (r.c.w[0].integ == integ) && (r.c.snap.steer_lat_ms == 3) &&
			       (r.c.snap.steer_lat_max_ms == 3);

		for (i = 0; i < 20; i++) {
			rig_tick(&r, &out, NULL, false, false);
		}
		check("MC-16", at_once && (r.c.w[0].target == 150) && (r.c.w[1].target == 250),
		      "speed hold between ticks: starts at once (as a short ramp, MC_STEER_PM_MM), "
		      "latency logged; the full command within 20 ticks");
	}

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

	/* smooth start: the drive never grows by more than MC_DUTY_RISE per tick,
	   and the speed still settles on the target */
	rig_init(&r);
	memset(&cmd, 0, sizeof(cmd));
	cmd.type = MC_CMD_SPEED;
	cmd.a = 200;
	cmd.b = 200;
	{
		int32_t	prev[2] = {0, 0};
		int	jerk = 0;

		rig_tick(&r, &out, &cmd, false, false);
		for (i = 0; i < 100; i++) {
			for (k = 0U; k < 2U; k++) {
				if ((MC_DUTY_RISE > 0) && ((out.duty[k] - prev[k]) > MC_DUTY_RISE)) {
					jerk = 1;
				}
				prev[k] = out.duty[k];
			}
			rig_tick(&r, &out, NULL, false, false);
		}
		check("MC-18", !jerk && (abs(r.c.w[0].speed - 200) < 25) && (abs(r.c.w[1].speed - 200) < 25),
		      "smooth start: drive rises at most MC_DUTY_RISE per tick, speed still reaches 200 mm/s");
	}

	/* smooth stop: CMD_STOP cuts the drive at once and coasts until the
	   wheels have stopped, then brakes; FAULT_STOP never coasts */
	{
		int coast_at_stop, coast_after;

		rig_tick(&r, &out, NULL, true, false);
		coast_at_stop = (out.coast == (MC_STOP_COAST != 0)) && (out.duty[0] == 0) && (out.duty[1] == 0) &&
				(out.stop_latency_us != 0U);
		for (i = 0; i < 50; i++) {
			rig_tick(&r, &out, NULL, true, false);
		}
		coast_after = !out.coast && (out.duty[0] == 0) && (out.duty[1] == 0);
		rig_init(&r);
		rig_tick(&r, &out, &cmd, false, false);
		for (i = 0; i < 60; i++) {
			rig_tick(&r, &out, NULL, false, false);
		}
		rig_tick(&r, &out, NULL, false, true);
		check("MC-19", coast_at_stop && coast_after && !out.coast && (out.duty[0] == 0) && (out.duty[1] == 0),
		      "CMD_STOP: duty 0 at once, coast until stopped, then brake; FAULT_STOP brakes at once");
	}
	/* MC-20: line following re-sends the speed hold every 200 ms; a car
	   pushing against something must still raise the stall fault */
	{
		bool	fault = false;
		int	n;

		rig_init(&r);
		cmd.type = MC_CMD_SPEED;
		cmd.a = 120;
		cmd.b = 120;
		rig_tick(&r, &out, &cmd, false, false);
		for (n = 0; n < 50; n++) {
			rig_tick(&r, &out, NULL, false, false);
		}
		r.stall = true;
		for (n = 0; (n < 200) && !fault; n++) {
			rig_tick(&r, &out, ((n % 10) == 0) ? &cmd : NULL, false, false);	/* refresh */
			fault = out.fault;
		}
		check("MC-20", fault && (r.c.snap.fault_bits == MC_FLT_STALL) && (out.duty[0] == 0),
		      "blocked while holding speed, hold refreshed every 200 ms: stall fault still raised");
	}

	/* MC-21: the car learns how hard it really stops, from its own stops */
	{
		bool	ack = false;
		int32_t	v0, d0, first;
		int	n;

		rig_init(&r);
		check("MC-21a", r.c.snap.stop_decel_mm_s2 == 0, "no stopping rate before the first stop");
		cmd.type = MC_CMD_SPEED;
		cmd.a = 200;
		cmd.b = 200;
		rig_tick(&r, &out, &cmd, false, false);
		for (n = 0; n < 60; n++) {
			rig_tick(&r, &out, NULL, false, false);
		}
		for (n = 0; (n < 50) && !ack; n++) {
			rig_tick(&r, &out, NULL, true, false);
			ack = has_event(&out.ev, EV_STOP_ACK);
		}
		v0 = r.c.stop_speed;
		d0 = (r.c.snap.stop_dist_mm > 0) ? r.c.snap.stop_dist_mm : 1;
		first = r.c.snap.stop_decel_mm_s2;
		check("MC-21", ack && (v0 > 150) && (first == ((v0 * v0) / (2 * d0))),
		      "after a stop from speed: stopping rate = speed^2 / (2 x stopping distance)");
		/* a stop that starts from (almost) standing still teaches nothing */
		rig_tick(&r, &out, NULL, false, false);		/* START: stop bits clear */
		ack = false;
		for (n = 0; (n < 20) && !ack; n++) {
			rig_tick(&r, &out, NULL, true, false);
			ack = has_event(&out.ev, EV_STOP_ACK);
		}
		check("MC-21b", ack && (r.c.snap.stop_decel_mm_s2 == first),
		      "a stop from standing still leaves the learned rate unchanged");
	}

	/* MC-22: coasting that does not slow the car (downhill) turns into braking */
	{
		bool	ack = false;
		int	n, coast_ticks = 0;

		rig_init(&r);
		r.coast_keep = 1.0;
		cmd.type = MC_CMD_SPEED;
		cmd.a = 200;
		cmd.b = 200;
		rig_tick(&r, &out, &cmd, false, false);
		for (n = 0; n < 60; n++) {
			rig_tick(&r, &out, NULL, false, false);
		}
		for (n = 0; (n < 50) && !ack; n++) {
			rig_tick(&r, &out, NULL, true, false);
			coast_ticks += r.c.coasting ? 1 : 0;
			ack = has_event(&out.ev, EV_STOP_ACK);
		}
		check("MC-22", ack && (coast_ticks <= (int)MC_ACCEL_WINDOW + 2) && (out.duty[0] == 0),
		      "coasting that does not slow the car switches to the brake; the stop still completes");
	}
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
	m.running = true;		/* terrain and humps are measured on a run */
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

/* ===== IMU status words ====================================================== */
static void test_pack_imu(void)
{
	IC_STATUS	a = {true, false, -8192, TERRAIN_DESCENDING, MOTION_IMPACT, 255, 255, 255U, -18000, 4095};
	IC_STATUS	b;
	IC_STATUS_PK	pk;

	ic_status_pack(&a, &pk);
	ic_status_unpack(&pk, &b);
	check("PK-I01", (b.calibrated && !b.ok && (b.pitch_cdeg == -8192) &&
	      (b.terrain == TERRAIN_DESCENDING) && (b.motion == MOTION_IMPACT) && (b.last_hump_mm == 255) &&
	      (b.max_hump_mm == 255) && (b.humps == 255U) && (b.heading_cdeg == -18000) && (b.g0_mg == 4095)),
	      "IMU status (3 words) round-trips at the field limits");
	a.pitch_cdeg = 9000;
	a.max_hump_mm = 400;
	a.humps = 1000U;
	ic_status_pack(&a, &pk);
	ic_status_unpack(&pk, &b);
	check("PK-I02", (b.pitch_cdeg == 8191) && (b.max_hump_mm == 255) && (b.humps == 255U),
	      "pitch past +-81.9 deg and humps past 255 saturate, never wrap");
}

/* ===== safety monitor ========================================================= */
/* IMU-15..17: "level" comes from the floor the car stands on before a run,
   and slopes / humps are only measured while the car drives itself. */
static void test_imu_level(void)
{
	IMU_CORE	c;
	IMU_OUT		out;
	IMU_MOTION	m;
	EVT_LIST	all;
	uint32_t	t = 0U, k;
	int32_t		before;
	int		i, slopes = 0, humps = 0;

	/* calibrated while held 4.4 degrees nose-up (car of 23 Oct) */
	imu_core_init(&c, &k_imu);
	memset(&m, 0, sizeof(m));
	m.valid = true;
	all.n = 0U;
	for (i = 0; i < 100; i++) {
		imu_feed(&c, 4.4, 0.0, &m, &t, &out, &all);
	}
	/* put down level, waiting in READY */
	for (i = 0; i < 10; i++) {
		imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
	}
	before = c.pitch_cdeg;
	m.waiting = true;
	for (i = 0; i < 300; i++) {
		imu_feed(&c, 0.0, 0.0, &m, &t, &out, &all);
	}
	check("IMU-15", (before < -300) && (abs(c.pitch_cdeg) <= 30),
	      "calibrated tilted (reads -4.4 deg level): standing still in READY re-learns level (0 +/- 0.3)");

	/* carried by hand (not on a run): tilted 20 degrees over 200 mm */
	m.waiting = false;
	m.running = false;
	m.speed_mm_s = 150;
	all.n = 0U;
	for (i = 0; i < 150; i++) {
		m.dist_mm = 1000 + i * 2;
		imu_feed(&c, (i < 100) ? 20.0 : 0.0, 0.0, &m, &t, &out, &all);
	}
	for (k = 0U; k < all.n; k++) {
		slopes += ((all.ev[k].type == EV_TERRAIN) && (all.ev[k].p1 != TERRAIN_LEVEL)) ? 1 : 0;
		humps += (all.ev[k].type == EV_HUMP) ? 1 : 0;
	}
	check("IMU-16", (slopes == 0) && (humps == 0) && (c.terrain == TERRAIN_LEVEL),
	      "not on a run (carried, put down): no slope, no hump reported");

	/* on a run the reference is NOT re-learned: a real slope is reported */
	m.running = true;
	slopes = 0;
	all.n = 0U;
	for (i = 0; i < 100; i++) {
		m.dist_mm = 2000 + i * 2;
		imu_feed(&c, 6.0, 0.0, &m, &t, &out, &all);
	}
	for (k = 0U; k < all.n; k++) {
		slopes += ((all.ev[k].type == EV_TERRAIN) && (all.ev[k].p1 == TERRAIN_CLIMBING)) ? 1 : 0;
	}
	check("IMU-17", (slopes == 1) && (c.terrain == TERRAIN_CLIMBING),
	      "on a run, a real 6 degree slope is still reported (level is only re-learned in READY)");
}

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

	/* 16-bit health stamps across the 65.536 s wrap */
	{
		uint16_t	st[HEALTH_SLOTS] = {0U, 0U, 0U, 0U, 0U};
		uint32_t	now = 65000U;
		bool		ok = true;

		sm_init(&s, &p);
		while (now < 66500U) {		/* motion stamps every 20 ms, polled 1 ms later */
			st[HEALTH_MOTION] = (uint16_t)(now | 1U);
			sm_health_ages(&s, now + 1U, st, age, alive);
			ok = ok && alive[HEALTH_MOTION] && (age[HEALTH_MOTION] <= 1U) && !alive[HEALTH_LINE];
			now += 20U;
		}
		check("SM-06", ok, "health ages stay small across the 16-bit wrap (stamps from 65,000 ms)");
		st[HEALTH_MOTION] = (uint16_t)(now + 3U);	/* stamped just after 'now' was read */
		sm_health_ages(&s, now, st, age, alive);
		ok = (age[HEALTH_MOTION] == 0U);
		sm_health_ages(&s, now + 65536U + 100U, st, age, alive);	/* silent for 65.6 s */
		ok = ok && (age[HEALTH_MOTION] == 65536U + 100U);	/* aged from when it was first seen */
		check("SM-07", ok, "a stamp newer than 'now' is fresh; a task silent > 65.5 s stays silent");
	}
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
	OS_STOP_MM, OS_SLOW_EXTRA_MM, OS_SLOW_HYST_MM, OS_DECEL_GUESS, OS_CONFIRM, OS_GAP_MS, OS_SETTLE_BASE_MS,
	OS_SETTLE_MS_PER_DEG, OS_RETRIES, OS_R_MAX_MM, OS_CORRIDOR_MM, OS_COARSE_STEP, OS_FINE_STEP,
	OS_FINE_MARGIN, OS_MISSING_LIMIT, OS_PROFILE_MAX_MS
};

/* Largest range (mm) at which two readings in a row give an ALERT, for a
   car at speed_mm_s with a learned stopping rate decel (0 = not yet). */
static int32_t alert_point(int32_t speed_mm_s, int32_t decel)
{
	int32_t r;

	for (r = 700; r > 0; r--) {
		SCAN_CORE	s;
		SCAN_IN		in;
		SCAN_OUT	out;
		int		i, pending = 0, alert = 0;

		scan_core_init(&s, &k_sc);
		memset(&in, 0, sizeof(in));
		in.have_req = true;
		in.req.mode = SCAN_WATCH | SCAN_STOP_FLAG;
		in.req.id = 9;
		in.speed_mm_s = speed_mm_s;
		in.stop_decel_mm_s2 = decel;
		for (i = 0; (i < 40) && !alert; i++) {
			in.now_ms = (uint32_t)(i * 10);
			in.echo = pending ? SCAN_ECHO_DONE : SCAN_ECHO_NONE;
			in.range_mm = r;
			pending = 0;
			scan_core_step(&s, &in, &out);
			in.have_req = false;
			pending = out.trigger ? 1 : 0;
			alert = has_event(&out.ev, EV_ALERT) ? 1 : 0;
		}
		if (alert) {
			return r;
		}
	}
	return 0;
}

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
		in.range_mm = OS_STOP_MM - 50;	/* a close reading, whatever the tuned distance */
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

	/* SC-10..12: the ALERT point comes from the car's own motion, so it
	   comes to REST about stop_mm away at any speed */
	{
		int32_t lead300 = ((300 * (int32_t)(OS_CONFIRM * OS_GAP_MS)) / 1000) + ((300 * 300) / (2 * OS_DECEL_GUESS));

		check("SC-10", alert_point(0, 0) == (OS_STOP_MM - 1),
		      "standing still: ALERT just inside stop_mm");
		check("SC-11", (alert_point(300, 0) == (OS_STOP_MM + lead300 - 1)) &&
		      (alert_point(300, 0) > alert_point(100, 0)) && (alert_point(100, 0) > alert_point(0, 0)),
		      "faster: ALERT earlier by the confirming pings + the stopping distance at that speed");
		check("SC-12", (alert_point(300, 4 * OS_DECEL_GUESS) < alert_point(300, 0)) &&
		      (alert_point(300, 4 * OS_DECEL_GUESS) > alert_point(0, 0)),
		      "a car that has measured a harder stop starts its stop later (learned, not guessed)");
	}

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
		in.range_mm = OS_STOP_MM - 50;	/* a close reading, whatever the tuned distance */
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
	VC_T_ACK_MS, VC_T_SCAN_MS, MC_TRACK_MM, MC_TURN_SPEED, MC_ACCEL, VC_STEP_MARGIN_MS,
	VC_HUMP_GRACE_MM, VC_HOLD_REFRESH_MS, VC_SEARCH_STEP_DEG, VC_SEARCH_MAX_DEG
};
static const LS_PARAMS k_ls = {LS_D_MIN, LS_D_RATE, LS_RATE_MM, LS_D_MAX, LS_D_BIAS, LS_D_SEARCH, LS_ESCAPE_MM,
			       LS_DRIFT_MM, LB_LOST_MM, LS_SLOW_PCT, LS_START_SIDE};

static int32_t g_odo;		/* odometer the controller is told about */

static void ev(VC_CORE *v, uint8_t type, int32_t p1, int32_t p2, uint32_t t, VC_ACTIONS *a)
{
	APP_EVT m;

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
	APP_EVT m;

	memset(&m, 0, sizeof(m));
	m.type = EV_PROFILE;
	m.p1 = 1;
	m.p2 = v->scan_id;
	vc_profile(v, &m, p, 100U, 0, a);
	return (v->state == VC_BYPASS) ? v->side : 0;
}

/* Start a run and drive past T_lost without seeing the line: a line
   search starts at t = 1000 ms with its first VC_SEARCH_STEP_DEG spin. */
static bool start_a_search(VC_CORE *v, const VC_PARAMS *p, VC_ACTIONS *a)
{
	g_odo = 0;
	vc_init(v, p, &k_ls, 0U);
	vc_tick(v, 20U, 0, true, a);
	ev(v, EV_COMMAND, CMD_START, 0, 30U, a);
	vc_tick(v, 1000U, LB_LOST_MM + 10, true, a);
	return (v->state == VC_SEARCH) && (find(a, VA_TURN) != NULL);
}

/* the watchdog a spin of deg gets: 2 x its own expected time + the margin */
static uint32_t spin_limit(const VC_PARAMS *p, int32_t deg)
{
	int32_t arc = (deg * 3142 * p->track_mm) / (360 * 1000);

	return (2U * (uint32_t)((arc * 1000) / p->turn_speed)) + p->step_margin_ms;
}

/* VC-28/29: a hump (or a pitch reading that only looks like one) never
   blinds the line follower while a sensor still sees the floor */
static void test_vc_hump(void)
{
	VC_CORE		v;
	VC_ACTIONS	a;
	const VC_ACTION	*x;
	int32_t		hs = (k_vc.hump_speed < k_vc.base_speed) ? k_vc.hump_speed : k_vc.base_speed;
	bool		ok;

	g_odo = 0;
	vc_init(&v, &k_vc, &k_ls, 0U);
	vc_tick(&v, 20U, 0, true, &a);
	ev(&v, EV_COMMAND, CMD_START, 0, 30U, &a);
	g_odo = 100;
	ev(&v, EV_TERRAIN, TERRAIN_CLIMBING, 80, 40U, &a);
	ev(&v, EV_LINE_STATUS, 1, 0, 50U, &a);		/* left black, right WHITE */
	x = find(&a, VA_HOLD);
	ok = (x != NULL) && (x->a < x->b) && (x->b <= hs);
	check("VC-28", ok, "on a hump, one sensor black and one white: still steers toward the line, at hump speed");
	ev(&v, EV_LINE_STATUS, 1, 1, 60U, &a);		/* both black: lifted off the floor? */
	x = find(&a, VA_HOLD);
	check("VC-29", (x != NULL) && (x->a == hs) && (x->b == hs),
	      "on a hump, both black (sensors may be lifted): straight on at hump speed");
}

/* VC-30: the line search sweeps ever wider either side of where the line
   was lost (+180, then -180 degrees from there), so the
   whole circle is covered before the car gives up */
static void test_vc_search_sweep(void)
{
	VC_CORE			v;
	VC_ACTIONS		a;
	const VC_ACTION		*x;
	static const int32_t	want[] = {180, -360};
	int32_t			heading = 0;
	int32_t			widest = 0;
	uint32_t		n = 0U;
	uint32_t		t = 1000U;
	bool			ok = start_a_search(&v, &k_vc, &a);
	bool			seq_ok = true;
	int32_t			first_sign;

	x = find(&a, VA_TURN);
	first_sign = ((x != NULL) && (x->a < 0)) ? -1 : 1;	/* the side the line was last seen */
	while (ok && (x != NULL) && (n < 10U)) {
		seq_ok = seq_ok && (n < (sizeof(want) / sizeof(want[0]))) && (x->a == (first_sign * want[n]));
		heading += x->a;
		widest = (heading > widest) ? heading : ((-heading > widest) ? -heading : widest);
		n++;
		t += 100U;
		ev(&v, EV_DONE, MC_MOVE_TURN, x->c, t, &a);
		x = (v.state == VC_SEARCH) ? find(&a, VA_TURN) : NULL;
	}
	check("VC-30", ok && seq_ok && (n == 2U) && (widest == 180) && (v.state == VC_HALTED) &&
	      (v.why == VC_WHY_SEARCH_FAILED),
	      "line search sweeps 180 degrees each side (the whole circle), then HALTED");
}

/* VC-25..27: no fixed time for a search, a barcode command or a bypass:
   every step's watchdog comes from the step itself */
static void test_vc_watchdog(void)
{
	VC_CORE		v;
	VC_ACTIONS	a;
	VC_PARAMS	slow = k_vc;
	const VC_ACTION	*x;
	uint32_t	lim = spin_limit(&k_vc, VC_SEARCH_STEP_DEG);
	uint32_t	lim_slow;
	bool		ok;

	ok = start_a_search(&v, &k_vc, &a);
	vc_tick(&v, 1000U + lim - 20U, LB_LOST_MM + 10, true, &a);
	ok = ok && (v.state == VC_SEARCH);
	vc_tick(&v, 1000U + lim + 40U, LB_LOST_MM + 10, true, &a);
	check("VC-25", ok && (v.state == VC_HALTED) && (v.why == VC_WHY_SEARCH_FAILED),
	      "a spin with no DONE is stopped at 2 x its own expected time + margin, not a fixed time");

	slow.turn_speed = k_vc.turn_speed / 3;		/* a car that spins three times slower */
	lim_slow = spin_limit(&slow, VC_SEARCH_STEP_DEG);
	ok = start_a_search(&v, &slow, &a);
	vc_tick(&v, 1000U + lim + 40U, LB_LOST_MM + 10, true, &a);
	check("VC-26", ok && (v.state == VC_SEARCH) && (lim_slow > lim),
	      "a slower spin gets proportionally longer: it is not cut short");

	/* a barcode command at a junction, after SET base_speed 80 */
	g_odo = 0;
	vc_init(&v, &k_vc, &k_ls, 0U);
	vc_tune(&v, 80, -1);
	vc_tick(&v, 20U, 0, true, &a);
	ev(&v, EV_COMMAND, CMD_START, 0, 30U, &a);
	g_odo = 200;
	ev(&v, EV_LINE_STATUS, 1, 0, 40U, &a);		/* on the line */
	ev(&v, EV_BARCODE, 'A', LB_CMD_TURN_LEFT, 50U, &a);
	ev(&v, EV_JUNCTION, 200, 0, 60U, &a);
	x = find(&a, VA_MOVE);
	{
		int32_t  mm = k_vc.line_to_axle_mm + k_vc.line_half_mm;
		uint32_t lim_move = (2U * (uint32_t)((mm * 1000) / 80)) + k_vc.step_margin_ms;

		ok = (v.state == VC_EXEC) && (x != NULL) && (x->b == 80);
		vc_tick(&v, 60U + lim_move - 20U, 200, true, &a);
		ok = ok && (v.state == VC_EXEC);
		vc_tick(&v, 60U + lim_move + 40U, 200, true, &a);
		check("VC-27", ok && (v.state == VC_HALTED) && (v.why == VC_WHY_EXEC_TIMEOUT),
		      "SET base_speed 80 caps the junction move at 80 mm/s, and its watchdog follows that speed");
	}
}

static void test_vc(void)
{
	VC_CORE			v;
	VC_ACTIONS		a;
	OS_PROFILE		p;
	const VC_ACTION		*x;
	int32_t			id;
	bool			ok;
	APP_EVT			m;

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
	/* the car drove on past the obstacle, seeing the line (T_lost is
	   travel: 590 mm without the line would be a line loss) */
	memset(&m, 0, sizeof(m));
	m.type = EV_LINE_STATUS;
	m.p2 = 1;
	vc_event(&v, &m, 190U, v.alert_mute_odo, &a);
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
static const uint8_t *cmdb(uint32_t seq, int32_t verb, int32_t param, int32_t value)
{
	static uint8_t	b[CMD_BYTES];
	CMD		c;

	c.seq = seq;
	c.verb = verb;
	c.param = param;
	c.value = value;
	cmd_encode(&c, b);
	return b;
}

static void test_cmd(void)
{
	CMD		c;
	uint32_t	seq = 0U;
	uint8_t		raw[CMD_BYTES];
	uint32_t	i;
	bool		ok;

	check("CMD-01", (cmd_parse(cmdb(1U, CMD_START, 0, 0), 4U, &seq, true, &c) == CMD_OK) && (c.verb == CMD_START),
	      "4-byte START");
	check("CMD-02", (cmd_parse(cmdb(2U, CMD_STOP, 0, 0), 4U, &seq, true, &c) == CMD_OK) && (c.verb == CMD_STOP),
	      "4-byte STOP");
	check("CMD-03", cmd_parse(cmdb(2U, CMD_STOP, 0, 0), 4U, &seq, true, &c) == CMD_ERR_REPEAT,
	      "same sequence number refused");
	check("CMD-04", (cmd_parse(cmdb(3U, CMD_STOP, 0, 0), 5U, &seq, true, &c) == CMD_ERR_FORMAT) &&
	      (cmd_parse(cmdb(3U, CMD_STOP, 0, 0), 3U, &seq, true, &c) == CMD_ERR_FORMAT),
	      "anything but 4 bytes refused");
	memcpy(raw, cmdb(3U, CMD_STOP, 0, 0), 4U);
	raw[1] |= 0x40U;				/* a spare bit */
	check("CMD-05", cmd_parse(raw, 4U, &seq, true, &c) == CMD_ERR_FORMAT, "spare bits set: a layout we do not know");
	check("CMD-06", cmd_parse(cmdb(4U, CMD_VERB_SET, 5, 1600), 4U, &seq, true, &c) == CMD_ERR_RANGE, "out of range");
	memcpy(raw, cmdb(4U, CMD_VERB_SET, 0, 200), 4U);
	raw[1] = (uint8_t)(raw[1] | (7U << 3));		/* parameter 7: not in the whitelist */
	check("CMD-07", cmd_parse(raw, 4U, &seq, true, &c) == CMD_ERR_NAME, "unknown parameter refused");
	check("CMD-08", (cmd_parse(cmdb(4U, CMD_VERB_SET, 4, 980), 4U, &seq, true, &c) == CMD_OK) && (c.param == 4) &&
	      (c.value == 980), "SET dist_cal 980");
	check("CMD-09", cmd_parse(cmdb(5U, CMD_VERB_SET, 2, 1200), 4U, &seq, false, &c) == CMD_ERR_STATE,
	      "SET refused while running");
	check("CMD-10", (cmd_parse(cmdb(5U, CMD_VERB_SET, 3, 50000), 4U, &seq, true, &c) == CMD_OK) && (c.value == 50000),
	      "the largest value (ki_x1000 = 50,000) fits the 16-bit field");
	memcpy(raw, cmdb(6U, 0, 0, 0), 4U);
	check("CMD-11", cmd_parse(raw, 4U, &seq, true, &c) == CMD_ERR_VERB, "verb 0 refused");

	/* 8-bit sequence across the wrap: 250 .. 255, 0, 1 .. 10 all new */
	seq = 249U;
	ok = true;
	for (i = 250U; i < 250U + 17U; i++) {
		ok = ok && (cmd_parse(cmdb(i & 0xFFU, CMD_STOP, 0, 0), 4U, &seq, true, &c) == CMD_OK);
	}
	check("CMD-12", ok && (seq == 10U), "sequence 250..255, 0..10: every one new across the wrap");
	ok = (cmd_parse(cmdb(10U + 127U, CMD_STOP, 0, 0), 4U, &seq, true, &c) == CMD_OK);
	ok = ok && (cmd_parse(cmdb(137U + 128U - 256U + 0U, CMD_STOP, 0, 0), 4U, &seq, true, &c) == CMD_ERR_REPEAT);
	ok = ok && (cmd_parse(cmdb(136U, CMD_STOP, 0, 0), 4U, &seq, true, &c) == CMD_ERR_REPEAT);
	check("CMD-13", ok, "127 ahead is new; 128 ahead and 1 behind are repeats");
}

/* One record's bytes as hex, for the decoder vectors */
static void hex_line(FILE *f, const char *name, const uint8_t *b, int32_t n)
{
	int32_t i;

	fprintf(f, "%s ", name);
	for (i = 0; i < n; i++) {
		fprintf(f, "%02x", b[i]);
	}
	fprintf(f, "\n");
}

static FILE *g_vec;		/* --vectors FILE: records for tools/test_mqtt_decode.py */

static void test_tel(void)
{
	TEL_DATA	d;
	uint8_t		buf[TEL_BIN_MAX];
	APP_EVT		m;
	TEL_SUMMARY	s;
	TEL_TIMING	tm;
	OS_PROFILE	p;
	int32_t		n;
	uint32_t	k;
	BP_R		r;
	bool		ok;

	memset(&d, 0, sizeof(d));
	d.t_ms = 0x12345678U;
	d.state = VC_FIND_LINE;
	d.v[0] = -469;
	d.v[1] = 212;
	d.enc[0] = 65500 + 100;
	d.enc[1] = -3;
	d.odo_mm = 123456;
	d.line_l = true;
	d.barcode = 'B';
	d.pending = 2;
	d.pitch_cdeg = -1234;
	d.motion = MOTION_DECELERATING;
	d.hmax_mm = 25;
	d.range_mm = 4095;
	d.yaw_cdeg_s = -41300;
	d.faults = FLT_STALL | FLT_MOVE_TIMEOUT | FLT_NO_ECHO | FLT_WIFI_DOWN | FLT_RESTART_REQ;
	n = tel_bin_telemetry(&d, buf, TEL_BIN_MAX);
	r.rec = buf;
	r.pos = 0U;
	ok = (n == 23) && (bp_ru(&r, 4U) == TEL_BIN_VERSION) && (bp_ru(&r, 4U) == TEL_T_TELEMETRY) &&
	     (bp_ru(&r, 24U) == 0x345678U) && (bp_ru(&r, 4U) == (uint32_t)VC_FIND_LINE) &&
	     (bp_rs(&r, 10U, -512) == -469) && (bp_rs(&r, 10U, -512) == 212) &&
	     (bp_ru(&r, 16U) == ((65500U + 100U) & 0xFFFFU)) && (bp_ru(&r, 16U) == 0xFFFDU) &&
	     (bp_ru(&r, 24U) == 123456U) && (bp_ru(&r, 2U) == 1U) && (bp_rs(&r, 6U, 32) == 'B') &&
	     (bp_ru(&r, 3U) == 2U) && (bp_rs(&r, 14U, -8192) == -1234) && (bp_ru(&r, 3U) == MOTION_DECELERATING) &&
	     (bp_ru(&r, 8U) == 25U) && (bp_ru(&r, 12U) == 4095U) && (bp_rs(&r, 10U, -512) == -413) &&
	     (bp_ru(&r, 12U) == (0x201U | 0x400U | 0x800U)) && (r.pos == 182U);
	check("TEL-01", ok, "telemetry: 182 bits = 23 B, every field read back (faults: 10 latched + NO_ECHO + WIFI_DOWN)");
	if (g_vec != NULL) {
		hex_line(g_vec, "telemetry", buf, n);
	}
	memset(&d, 0xFF, sizeof(d));		/* every number as large as it gets */
	d.line_l = true;
	d.line_r = true;
	d.imu_ok = true;
	d.v[0] = INT32_MIN;
	d.pitch_cdeg = INT32_MIN;
	d.wifi = INT32_MIN;
	ok = (tel_bin_telemetry(&d, buf, TEL_BIN_MAX) == 23) && (tel_bin_status(&d, buf, TEL_BIN_MAX) == 18);
	check("TEL-02", ok && (tel_bin_telemetry(&d, buf, 22U) == -1),
	      "extreme values saturate into the same sizes; a record that does not fit is refused");

	memset(&d, 0, sizeof(d));
	d.t_ms = 1000U;
	d.state = VC_SAFE_STOP;
	d.why = VC_WHY_IMPACT;
	d.faults = FLT_IMPACT | FLT_WIFI_DOWN;
	d.mc_fault = (int32_t)MC_FLT_ENCODER;
	d.imu_ok = true;
	d.heading_cdeg = -17950;
	d.pings = 70000U;
	d.wifi = -3;
	d.mqtt_connects = 3U;
	d.tx_drops = 5U;
	d.rx_drops = 300U;
	d.net_errors = 1U;
	d.reconnect_ms = 9500U;
	n = tel_bin_status(&d, buf, TEL_BIN_MAX);
	r.rec = buf;
	r.pos = 8U + 24U;
	ok = (n == 18) && (bp_ru(&r, 4U) == (uint32_t)VC_SAFE_STOP) && (bp_ru(&r, 4U) == (uint32_t)VC_WHY_IMPACT) &&
	     (bp_ru(&r, 12U) == (0x008U | 0x800U)) && (bp_ru(&r, 3U) == MC_FLT_ENCODER) && (bp_ru(&r, 1U) == 1U) &&
	     (bp_rs(&r, 9U, -256) == -179) && (bp_ru(&r, 16U) == 65535U) && (bp_rs(&r, 3U, -4) == -3) &&
	     (bp_ru(&r, 8U) == 3U) && (bp_ru(&r, 16U) == 5U) && (bp_ru(&r, 8U) == 255U) &&
	     (bp_ru(&r, 8U) == 1U) && (bp_ru(&r, 18U) == 9500U) && (r.pos == 142U);
	check("TEL-03", ok, "status: 142 bits = 18 B; the link state is only the WIFI_DOWN bit; the outage length is reported");
	if (g_vec != NULL) {
		hex_line(g_vec, "status", buf, n);
	}

	memset(&m, 0, sizeof(m));
	m.type = EV_STOP_ACK;
	check("TEL-04", tel_bin_event(&m, 0U, buf, TEL_BIN_MAX) == 0, "STOP_ACK is not published as an event");
	m.type = EV_TERRAIN;
	m.p1 = TERRAIN_CLIMBING;
	m.p2 = -812;
	n = tel_bin_event(&m, 70000U, buf, TEL_BIN_MAX);
	r.rec = buf;
	r.pos = 8U;
	ok = (n == 9) && (bp_ru(&r, 24U) == 70000U) && (bp_ru(&r, 5U) == EV_TERRAIN) &&
	     (bp_ru(&r, 2U) == TERRAIN_CLIMBING) && (bp_rs(&r, 14U, -8192) == -812);
	check("TEL-05", ok, "event: 9 B, payload laid out exactly as APP_MSG");
	if (g_vec != NULL) {
		hex_line(g_vec, "event", buf, n);
	}
	n = tel_bin_ack(5U, 200U, false, CMD_ERR_REPEAT, buf, TEL_BIN_MAX);
	check("TEL-06", (n == 6) && (buf[4] == 200U) && ((buf[5] & 1U) == 0U) && (((buf[5] >> 1) & 7U) == CMD_ERR_REPEAT),
	      "ack: 6 B (seq, accepted, error)");
	if (g_vec != NULL) {
		hex_line(g_vec, "ack", buf, n);
	}
	memset(&p, 0, sizeof(p));
	p.valid = true;
	p.x_left_mm = -120;
	p.x_right_mm = 60;
	p.width_mm = 180;
	p.x_center_mm = -30;
	p.closest_mm = 300;
	p.closest_deg = 95;
	p.left_clear_mm = OS_CLEAR_OPEN;
	p.right_clear_mm = 240;
	p.unknown_left = 2U;
	p.unknown_right = 0U;
	p.npoints = 9U;
	n = tel_bin_profile(77U, &p, buf, TEL_BIN_MAX);
	check("TEL-07", n == 17, "profile: 17 B (the 13-byte OS_PROFILE_PK behind the header)");
	if (g_vec != NULL) {
		hex_line(g_vec, "profile", buf, n);
	}
	memset(&s, 0, sizeof(s));
	s.t_ms = 99000U;
	s.time_ms = 95000U;
	s.dist_mm = -250;
	s.hmax_mm = 21;
	s.humps = 2U;
	s.commands = 4U;
	s.obstacles = 1U;
	s.reads_ok = 4U;
	s.reads_failed = 300U;
	n = tel_bin_summary(&s, buf, TEL_BIN_MAX);
	check("TEL-08", n == 16, "summary: 16 B");
	if (g_vec != NULL) {
		hex_line(g_vec, "summary", buf, n);
	}
	memset(&tm, 0, sizeof(tm));
	tm.t_ms = 0xFFFFFFFFU;
	for (k = 0U; k < TEL_TIM_TASKS; k++) {
		tm.max_us[k] = 100U * (k + 1U);
	}
	tm.max_us[6] = 0xFFFFFFFFU;
	tm.overruns = 3U;
	tm.irq_per_s = 3300U;
	tm.irq_max_us = 12U;
	tm.vq_high = 4U;
	tm.dropped = 0U;
	tm.steer_lat_max_ms = 2;
	tm.stop_lat_max_us = 450;
	tm.stop_dist_mm = 31;
	tm.reconnect_ms = 1000000U;
	n = tel_bin_timing(&tm, buf, TEL_BIN_MAX);
	check("TEL-09", (n == 34) && (n == (int32_t)TEL_BIN_MAX), "timing: 269 bits in 34 B, the largest record (NET_PAYLOAD_MAX)");
	if (g_vec != NULL) {
		hex_line(g_vec, "timing", buf, n);
		hex_line(g_vec, "cmd_set", cmdb(201U, CMD_VERB_SET, 3, 50000), 4);
		for (k = 1U; k <= EV_TYPE_MAX; k++) {	/* the decoder's event table must match */
			fprintf(g_vec, "layout %u %u %d %u %d\n", k, app_evt_bits((uint8_t)k, 1U), (int)app_evt_min((uint8_t)k, 1U),
				app_evt_bits((uint8_t)k, 2U), (int)app_evt_min((uint8_t)k, 2U));
		}
	}

	check("EVT-01", evt_is_safety(EV_FAULT) && evt_is_safety(EV_IMPACT) && evt_is_safety(EV_ALERT) &&
	      evt_is_safety(EV_STOP_ACK) && evt_is_safety(EV_BUTTON) && evt_is_safety(EV_COMMAND) &&
	      evt_is_safety(EV_FAULT_CLEAR) && !evt_is_safety(EV_LINE_STATUS) && !evt_is_safety(EV_JUNCTION) &&
	      !evt_is_safety(EV_DONE) && !evt_is_safety(EV_TERRAIN) && !evt_is_safety(EV_RANGE_ZONE),
	      "queue policy: safety events must arrive, the rest may be dropped and counted");
}

/* ---- minimum-bit layouts: motion (mc_core.h) ------------------------------------ */
static int cmd_round(uint8_t type, int32_t a, int32_t b, int32_t c, MC_CMD *out)
{
	MC_CMD		in;
	MC_CMD_PK	pk;

	memset(&in, 0, sizeof(in));
	in.type = type;
	in.a = a;
	in.b = b;
	in.c = c;
	mc_cmd_pack(&in, &pk);
	mc_cmd_unpack(&pk, out);
	return (out->type == type) && (out->a == a) && (out->b == b);
}

static void test_pack_motion(void)
{
	MC_CMD		o;
	MC_SNAPSHOT	s, u;
	MC_SNAP_PK	spk;
	MC_ENC		e;
	int		ok;

	check("PK-M01", sizeof(MC_CMD_PK) == 8U, "MC_CMD_PK is 8 bytes (motion_cmd_q slot 12 B, was 20 B)");
	ok = cmd_round(MC_CMD_DISTANCE, 10000, 400, 255, &o) && (o.c == 255) &&
	     cmd_round(MC_CMD_DISTANCE, -10000, 1, 1, &o) && (o.c == 1);
	check("PK-M02", ok, "DISTANCE round-trips at +-10,000 mm, speed 1..400, id 1..255");
	ok = cmd_round(MC_CMD_TURN, 360, 0, 7, &o) && cmd_round(MC_CMD_TURN, -360, 400, 7, &o) && (o.c == 7);
	check("PK-M03", ok, "TURN round-trips at +-360 deg, speed 0..400");
	ok = cmd_round(MC_CMD_SPEED, -400, 400, 0, &o) && (o.c == 0) &&
	     cmd_round(MC_CMD_SPEED, 400, -400, 65536 * 3, &o) && (o.c != 0) &&
	     ((uint16_t)o.c == 0U);
	check("PK-M04", ok, "SPEED round-trips at +-400; a line time with low 16 bits 0 is still 'present'");
	(void)cmd_round(MC_CMD_DISTANCE, 20000, 600, 300, &o);
	ok = (o.a == 16383) && (o.b == 511) && (o.c == 255);
	(void)cmd_round(MC_CMD_SPEED, -900, 900, 0, &o);
	ok = ok && (o.a == -512) && (o.b == 511);
	check("PK-M05", ok, "out-of-range command fields saturate, never wrap");

	memset(&s, 0, sizeof(s));
	s.seq = 250U + 300U;			/* 8 bits: 550 & 255 = 38 */
	s.t_ms = 65000U + 1000U;		/* 16 bits: wraps */
	s.count[0] = 65500 + 100;
	s.count[1] = -32768;
	s.speed[0] = -469;
	s.speed[1] = 511;
	s.target[0] = -400;
	s.target[1] = 400;
	s.duty[0] = -1000;
	s.duty[1] = 1000;
	s.speed_mm_s = -300;
	s.accel_mm_s2 = -9380;
	s.dist_mm = -16777216;
	s.odo_mm = 16777215;
	s.yaw_cdeg_s = -41300;
	s.mode = MC_FAULT;
	s.fault_bits = MC_FLT_STALL | MC_FLT_ENCODER;
	s.steer_lat_ms = 3;
	s.steer_lat_max_ms = 1023;
	s.stop_lat_max_us = 131071;
	s.stop_dist_mm = 1023;
	s.stop_decel_mm_s2 = 20000;
	mc_snap_pack(&s, &spk);
	mc_snap_unpack(&spk, &u);
	ok = (sizeof(MC_SNAP_PK) == 36U) && (u.seq == 38U) && (u.t_ms == ((65000U + 1000U) & 0xFFFFU)) &&
	     ((uint16_t)u.count[0] == (uint16_t)(65500 + 100)) && (u.count[1] == -32768) &&
	     (u.speed[0] == -469) && (u.speed[1] == 511) && (u.target[0] == -400) && (u.target[1] == 400) &&
	     (u.duty[0] == -1000) && (u.duty[1] == 1000) && (u.speed_mm_s == -300) &&
	     (u.accel_mm_s2 == -9380) && (u.dist_mm == -16777216) && (u.odo_mm == 16777215) &&
	     (u.yaw_cdeg_s == -41300) && (u.mode == MC_FAULT) && (u.fault_bits == s.fault_bits) &&
	     (u.steer_lat_ms == 3) && (u.steer_lat_max_ms == 1023) && (u.stop_lat_max_us == 131071) &&
	     (u.stop_dist_mm == 1023) && (u.stop_decel_mm_s2 == 20000);
	check("PK-M06", ok, "snapshot (36 B) round-trips every field at its min / max");
	s.speed[0] = 2000;
	s.odo_mm = 20000000;
	s.stop_lat_max_us = 500000;
	s.steer_lat_ms = -5;
	s.stop_decel_mm_s2 = 40000;
	mc_snap_pack(&s, &spk);
	mc_snap_unpack(&spk, &u);
	check("PK-M07", (u.speed[0] == 511) && (u.odo_mm == 16777215) && (u.stop_lat_max_us == 131071) &&
	      (u.steer_lat_ms == 0) && (u.stop_decel_mm_s2 == 32767), "out-of-range snapshot fields saturate, never wrap");

	/* encoder word: 16-bit halves that wrap, 32-bit positions in the task */
	mc_enc_init(&e, 65500U | (65500U << 16));
	mc_enc_update(&e, (uint32_t)(uint16_t)(65500U + 100U) | ((uint32_t)(uint16_t)(65500U - 100U) << 16));
	ok = (e.pos[MC_LEFT] == -36 + 100) && (e.pos[MC_RIGHT] == -36 - 100);
	{
		uint32_t	i;
		uint32_t	l = 65500U;

		for (i = 0U; i < 5000U; i++) {	/* 16 counts per 20 ms read, forward */
			l += 16U;
			mc_enc_update(&e, (uint32_t)(uint16_t)l | (e.last & 0xFFFF0000U));
		}
	}
	ok = ok && (e.pos[MC_LEFT] == -36 + 80000);
	check("PK-M08", ok, "encoder halves cross the 16-bit wrap (start 65,500); 80,000 counts summed in 32 bits");

	/* the odometer read on its own: the fields either side all ones */
	{
		static const int32_t	odo[] = {0, 2413, 16777215, 20000000};
		static const int32_t	want[] = {0, 2413, 16777215, 16777215};
		uint32_t		i;

		ok = true;
		s.dist_mm = 16777215;
		s.yaw_cdeg_s = 65535;
		for (i = 0U; i < (sizeof(odo) / sizeof(odo[0])); i++) {
			s.odo_mm = odo[i];
			mc_snap_pack(&s, &spk);
			mc_snap_unpack(&spk, &u);
			ok = ok && (mc_snap_odo(&spk) == want[i]) && (u.odo_mm == want[i]);
		}
	}
	check("PK-M09", ok, "mc_snap_odo() reads the same odometer as a full unpack (0, 2413, max, saturated)");
}

/* ===== the 8-byte event record: every type at its limits ==================== */
static bool evt_round_trip(uint8_t type, int32_t p1, int32_t p2, int32_t want1, int32_t want2)
{
	APP_EVT	e = {type, SRC_VC, 255U, 65535U, p1, p2};
	APP_EVT	u;
	APP_MSG	m;

	app_msg_pack(&e, &m);
	app_msg_unpack(&m, &u);
	return (u.type == type) && (u.source == SRC_VC) && (u.seq == 255U) && (u.t_ms == 65535U) &&
	       (u.p1 == want1) && (u.p2 == want2);
}

static void test_pack_events(void)
{
	bool		lim = (sizeof(APP_MSG) == 8U);
	bool		sat = true;
	uint32_t	k;
	uint32_t	src;

	for (k = 1U; k <= EV_TYPE_MAX; k++) {
		uint8_t		t = (uint8_t)k;
		uint32_t	n1 = app_evt_bits(t, 1U);
		uint32_t	n2 = app_evt_bits(t, 2U);
		int32_t		lo1 = (n1 == 0U) ? 0 : app_evt_min(t, 1U);
		int32_t		lo2 = (n2 == 0U) ? 0 : app_evt_min(t, 2U);
		int32_t		hi1 = (n1 == 0U) ? 0 : (int32_t)((int64_t)lo1 + (int64_t)BP_MAX(n1));
		int32_t		hi2 = (n2 == 0U) ? 0 : (int32_t)((int64_t)lo2 + (int64_t)BP_MAX(n2));

		lim = lim && ((n1 + n2) <= 32U) && evt_round_trip(t, lo1, lo2, lo1, lo2) &&
		      evt_round_trip(t, hi1, hi2, hi1, hi2);
		if (t != EV_FAULT) {			/* a bit mask: masked, see below */
			sat = sat && evt_round_trip(t, lo1 - 1, lo2 - 1, lo1, lo2) &&
			      evt_round_trip(t, hi1 + 1, hi2 + 1, hi1, hi2);
		}
	}
	check("EVT-02", lim, "every event type round-trips p1 and p2 at their min and max (8 B, payload <= 32 bits)");
	check("EVT-03", sat, "every event type saturates values below the min or above the max, never wraps");
	sat = evt_round_trip(EV_FAULT, (int32_t)FLT_ALL, 0, (int32_t)FLT_ALL, 0) &&
	      evt_round_trip(EV_FAULT, (int32_t)(FLT_ALL | FLT_NO_ECHO), 0, (int32_t)FLT_ALL, 0);
	for (src = 0U; src < 8U; src++) {	/* 3-bit source, 8-bit seq, 16-bit time */
		APP_EVT	e = {EV_HUMP, (uint8_t)src, (uint8_t)(src * 37U), (uint16_t)(src * 9000U), 18, 25};
		APP_EVT	u;
		APP_MSG	m;

		app_msg_pack(&e, &m);
		app_msg_unpack(&m, &u);
		sat = sat && (u.source == src) && (u.seq == e.seq) && (u.t_ms == e.t_ms);
	}
	check("EVT-04", sat && evt_round_trip(EV_BUTTON, BTN_STOP, 0, BTN_STOP, 0),
	      "fault mask keeps the 10 latching bits only; header fields and the 1-bit button round-trip");
}

/* ===== obstacle scan and VC layouts ========================================== */
static void test_pack_scan(void)
{
	SCAN_REQ	r = {SCAN_WATCH | SCAN_STOP_FLAG, 120, 255}, u;
	SCAN_REQ_PK	rpk;
	OS_PROFILE	p, q;
	OS_PROFILE_PK	ppk;
	VC_ST		v = {VC_HALTED, VC_WHY_BYPASS_TIMEOUT, LB_CMD_U_TURN, 300U, 200U, 4000000000U, 16777215, 32U};
	VC_ST		w;
	VC_ST_PK	vpk;
	SCAN_ST		sst, sst2;
	uint32_t	sw;
	bool		ok;

	scan_req_pack(&r, &rpk);
	scan_req_unpack(&rpk, &u);
	ok = (sizeof(SCAN_REQ_PK) == 4U) && (u.mode == r.mode) && (u.stop_mm == 120) && (u.id == 255) &&
	     (rpk.w < (1U << 20));
	r.stop_mm = 600;
	scan_req_pack(&r, &rpk);
	scan_req_unpack(&rpk, &u);
	ok = ok && (u.stop_mm == 600);
	r.stop_mm = 0;
	scan_req_pack(&r, &rpk);
	scan_req_unpack(&rpk, &u);
	ok = ok && (u.stop_mm == 0);
	check("PK-S01", ok, "SCAN_REQ (4 B, 20 bits): stop 120 / 600 / 0 = default and id 255 round-trip");
	r.stop_mm = 5000;
	scan_req_pack(&r, &rpk);
	scan_req_unpack(&rpk, &u);
	ok = (u.stop_mm == 630);
	r.stop_mm = 50;
	scan_req_pack(&r, &rpk);
	scan_req_unpack(&rpk, &u);
	check("PK-S02", ok && (u.stop_mm == 120), "out-of-range stop distances saturate, never become 'default'");

	ok = (scan_range_mm(0U) == 0) && (scan_range_mm(23323U) == 3999) && (scan_range_mm(23324U) == 4000) &&
	     (scan_range_mm(23330U) == OS_INVALID) && (scan_range_mm(32767U) == OS_INVALID) &&
	     (scan_range_mm(0xFFFFFFFFU) == OS_INVALID) && (OS_INVALID == 4095);
	check("PK-S03", ok, "echo width -> 12-bit range; past 4,000 mm (~23.3 ms) is 4,095 = no echo, never 904 mm");

	memset(&p, 0, sizeof(p));
	p.valid = true;
	p.x_left_mm = -1024;
	p.x_right_mm = 1023;
	p.width_mm = 2047;
	p.x_center_mm = -5;
	p.closest_mm = 800;
	p.closest_deg = 150;
	p.left_clear_mm = OS_CLEAR_OPEN;
	p.right_clear_mm = 0;
	p.unknown_left = SCAN_SLOTS;
	p.unknown_right = 1U;
	p.npoints = 31U;
	os_profile_pack(&p, &ppk);
	os_profile_unpack(&ppk, &q);
	ok = (sizeof(OS_PROFILE_PK) == 13U) && q.valid && (q.x_left_mm == -1024) && (q.x_right_mm == 1023) &&
	     (q.width_mm == 2047) && (q.x_center_mm == -5) && (q.closest_mm == 800) && (q.closest_deg == 150) &&
	     (q.left_clear_mm == OS_CLEAR_OPEN) && (q.right_clear_mm == 0) && (q.unknown_left == SCAN_SLOTS) &&
	     (q.unknown_right == 1U) && (q.npoints == 31U);
	check("PK-S04", ok, "OS_PROFILE (13 B, 97 bits) round-trips every field at its limits");
	p.x_left_mm = -3000;
	p.closest_mm = 5000;
	p.npoints = OS_MAX_POINTS;
	os_profile_pack(&p, &ppk);
	os_profile_unpack(&ppk, &q);
	check("PK-S05", (q.x_left_mm == -1024) && (q.closest_mm == 1023) && (q.npoints == 31U),
	      "profile fields saturate, never wrap");

	vc_st_pack(&v, &vpk);
	vc_st_unpack(&vpk, &w);
	ok = (w.state == VC_HALTED) && (w.why == VC_WHY_BYPASS_TIMEOUT) && (w.pending == LB_CMD_U_TURN) &&
	     (w.commands_done == 255U) && (w.obstacles_passed == 127U) && (w.run_start_ms == 4000000000U) &&
	     (w.run_start_odo == 16777215) && (w.vq_high == 32U);
	check("PK-S06", ok && (VC_STATES <= 16), "VC status: 14 states and 10 reasons in 4 bits each; counts saturate");

	sst.mode = SCAN_PROFILE;
	sst.range_mm = OS_INVALID;
	sst.pings = 65535U;
	sw = scan_st_pack(&sst);
	scan_st_unpack(sw, &sst2);
	ok = (sw < (1UL << 30)) && (sst2.mode == SCAN_PROFILE) && (sst2.range_mm == OS_INVALID) && (sst2.pings == 65535U);
	sst.mode = SCAN_WATCH;
	sst.range_mm = 0;
	sst.pings = 65536U + 7U;		/* the counter wraps: only the low 16 bits are shared */
	scan_st_unpack(scan_st_pack(&sst), &sst2);
	ok = ok && (sst2.mode == SCAN_WATCH) && (sst2.range_mm == 0) && (sst2.pings == 7U);
	sst.range_mm = 9000;			/* out of range: saturates, never wraps */
	scan_st_unpack(scan_st_pack(&sst), &sst2);
	ok = ok && (sst2.range_mm == 4095);
	check("PK-S07", ok, "Scan status in one word (30 bits): mode 2, range 12 (saturates), pings low 16");
}

int main(int argc, char **argv)
{
	if ((argc == 3) && (strcmp(argv[1], "--vectors") == 0)) {
		g_vec = fopen(argv[2], "w");	/* records for tools/test_mqtt_decode.py */
	}
	test_code39();
	test_barcode();
	test_line();
	test_motion();
	test_pack_motion();
	test_imu();
	test_imu_level();
	test_pack_imu();
	test_safety();
	test_profile();
	test_scan();
	test_pack_events();
	test_pack_scan();
	test_vc();
	test_vc_watchdog();
	test_vc_search_sweep();
	test_vc_hump();
	test_cmd();
	test_tel();
	if (g_vec != NULL) {
		(void)fclose(g_vec);
	}
	printf("\nUNIT: %d checks, %d failed\n", g_n, g_fail);
	return (g_fail == 0) ? 0 : 1;
}
