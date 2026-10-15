/*
 *	sim.c
 *	Closed-loop host simulator.  Runs the firmware's own logic modules
 *	(the *_core.c files in app_program/subsystemN) against a simulated car
 *	on a simulated course, with stand-ins for the kernel objects that
 *	behave like the real ones: message buffers with fixed capacity, event
 *	flags, periodic releases.  The glue below mirrors the task files
 *	(task_*.c) step for step.
 *
 *	Usage:  ./sim            run every scenario, print PASS/FAIL
 *	        ./sim -v NAME    run one scenario with a trace
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_world.h"

#include "../../app_program/app_evt.h"
#include "../../app_program/subsystem1/cmd_parse.h"
#include "../../app_program/subsystem1/tel_json.h"
#include "../../app_program/subsystem2/mc_core.h"
#include "../../app_program/subsystem2/motion_cfg.h"
#include "../../app_program/subsystem3/line_core.h"
#include "../../app_program/subsystem3/barcode_core.h"
#include "../../app_program/subsystem3/line_cfg.h"
#include "../../app_program/subsystem4/imu_core.h"
#include "../../app_program/subsystem4/safety_core.h"
#include "../../app_program/subsystem4/imu_cfg.h"
#include "../../app_program/subsystem5/scan_core.h"
#include "../../app_program/subsystem5/vc_core.h"
#include "../../app_program/subsystem5/scan_cfg.h"
#include "../../app_program/subsystem5/vc_cfg.h"

#define PI		3.14159265358979
#define MC_CMD_STOP	0x01U
#define MC_FAULT_STOP	0x02U
#define MCQ_CAP		8
#define SCQ_CAP		4
#define VCQ_CAP		32
#define TELQ_CAP	16
#define EDGE_CAP	64
#define MAX_SCRIPT	16
#define NAV_CODES	(NAV_REFUSED + 1)

static unsigned g_seed = 12345U;

/* ---- the kernel stand-ins ----------------------------------------------- */
typedef struct {
	APP_MSG	m[VCQ_CAP];
	int	head;
	int	n;
	int	cap;
	int	drops;
} QUEUE;

static bool q_push(QUEUE *q, uint8_t src, uint8_t type, int32_t p1, int32_t p2, uint32_t t_ms)
{
	APP_MSG *m;

	if (q->n >= q->cap) {
		q->drops++;
		return false;
	}
	m = &q->m[(q->head + q->n) % q->cap];
	m->type = type;
	m->source = src;
	m->seq = 0;
	m->t_ms = t_ms;
	m->p1 = p1;
	m->p2 = p2;
	q->n++;
	return true;
}

static bool q_pop(QUEUE *q, APP_MSG *out)
{
	if (q->n == 0) {
		return false;
	}
	*out = q->m[q->head];
	q->head = (q->head + 1) % q->cap;
	q->n--;
	return true;
}

/* ---- scenario script -------------------------------------------------------- */
#define SC_CMD		1	/* MQTT command text */
#define SC_BUTTON	2	/* a = 20 / 21 */
#define SC_IMU_FAIL	3	/* a = 1 fail, 0 ok */

typedef struct {
	uint32_t	t_ms;
	int		kind;
	int		a;
	const char	*text;
} SCRIPT;

/* ---- the whole system ------------------------------------------------------- */
typedef struct {
	const char	*name;
	WORLD		w;
	CAR		car;
	uint32_t	t_ms;
	uint32_t	t_us;
	bool		verbose;

	/* kernel objects */
	QUEUE		vcq;
	QUEUE		telq;
	MC_CMD		mq[MCQ_CAP];
	int		mq_n;
	SCAN_REQ	sq[SCQ_CAP];
	int		sq_n;
	uint32_t	stop_flags;
	uint32_t	fault_flags;
	uint32_t	health[HEALTH_SLOTS];

	/* subsystem 2 */
	MC_CORE		mc;
	MC_SNAPSHOT	snap;
	uint32_t	stop_req_us;
	bool		was_stopped;
	/* subsystem 3 */
	LC_CORE		lc;
	BC_CORE		bc;
	uint32_t	edge_t[EDGE_CAP];
	uint8_t		edge_l[EDGE_CAP];
	int		edge_n;
	bool		bc_level;
	/* subsystem 4 */
	IMU_CORE	imu;
	SM_CORE		sm;
	bool		imu_fail;
	int		btn[2];		/* 1 = released */
	/* subsystem 5 */
	SCAN_CORE	sc;
	OS_PROFILE	prof;
	bool		ping_in_flight;
	uint32_t	ping_done_us;
	uint32_t	echo_high_until_us;
	double		ping_range;
	bool		ping_refused;
	VC_CORE		vc;
	/* subsystem 1 */
	uint32_t	last_seq;
	int32_t		params[CMD_NPARAMS];

	/* script */
	SCRIPT		script[MAX_SCRIPT];
	int		nscript;

	/* observations */
	uint32_t	states_seen;
	int		nav[NAV_CODES];
	int32_t		nav_last[NAV_CODES];
	char		barcodes[16];
	int		nbarcodes;
	int		bad_reads;
	int32_t		humps[8];
	int		nhumps;
	double		max_dev_follow;
	double		max_dev_all;
	uint32_t	follow_ms;
	double		sum_dev;
	int		json_max;
	int		json_fail;
	int		json_n;
	int		queue_errors;
	int32_t		last_why;
	uint32_t	stop_cmd_us;
	uint32_t	stop_latency_us;
	bool		measure_stop;
	uint32_t	impact_ms;
	uint32_t	crash_ms;
} SIM;

static void trace(SIM *s, const char *fmt, const char *a, int32_t b, int32_t c)
{
	if (s->verbose) {
		printf("%7.3f  ", s->t_ms / 1000.0);
		printf(fmt, a, b, c);
		printf("   (x %.0f y %.0f th %.0f)\n", s->car.x, s->car.y, s->car.th * 180.0 / PI);
	}
}

/* ---- telemetry (subsystem 1) ------------------------------------------------ */
static void json_check(SIM *s, int32_t n)
{
	if (n < 0) {
		s->json_fail++;
	} else if (n > 0) {
		s->json_n++;
		if (n > s->json_max) {
			s->json_max = n;
		}
		if (n > (int32_t)TEL_JSON_MAX) {
			s->json_fail++;
		}
	} else {
		/* not published */
	}
}

static void telemetry_step(SIM *s, bool periodic, bool heartbeat)
{
	APP_MSG		m;
	char		buf[TEL_JSON_MAX];
	TEL_DATA	d;

	while (q_pop(&s->telq, &m)) {
		if (m.type == EV_PROFILE_DATA) {
			json_check(s, tel_fmt_profile(m.t_ms, s->prof.valid, s->prof.x_left_mm,
						      s->prof.x_right_mm, s->prof.closest_mm, s->prof.closest_deg,
						      s->prof.left_clear_mm, s->prof.right_clear_mm,
						      s->prof.unknown_left, s->prof.unknown_right, buf, TEL_JSON_MAX));
		} else {
			json_check(s, tel_fmt_event(&m, buf, TEL_JSON_MAX));
		}
		if (m.type == EV_STATE) {
			s->states_seen |= 1U << m.p1;
			if (m.p1 == VC_SAFE_STOP) {
				s->last_why = m.p2;
			}
		} else if (m.type == EV_NAV) {
			if ((m.p1 > 0) && (m.p1 < NAV_CODES)) {
				s->nav[m.p1]++;
				s->nav_last[m.p1] = m.p2;
			}
		} else if (m.type == EV_BARCODE) {
			if (s->nbarcodes < 15) {
				s->barcodes[s->nbarcodes++] = (char)m.p1;
			}
		} else if (m.type == EV_BARCODE_BAD) {
			s->bad_reads++;
		} else if (m.type == EV_HUMP) {
			if (s->nhumps < 8) {
				s->humps[s->nhumps++] = m.p1;
			}
		} else if (m.type == EV_IMPACT) {
			if (s->impact_ms == 0U) {
				s->impact_ms = s->t_ms;
			}
		} else {
			/* nothing to record */
		}
	}
	if (periodic || heartbeat) {
		memset(&d, 0, sizeof(d));
		d.t_ms = s->t_ms;
		d.state = s->vc.state;
		d.v[0] = s->snap.speed[0];
		d.v[1] = s->snap.speed[1];
		d.enc[0] = s->snap.count[0];
		d.enc[1] = s->snap.count[1];
		d.odo_mm = s->snap.odo_mm;
		d.barcode = '-';
		d.motion = s->imu.motion;
		d.pitch_cdeg = s->imu.pitch_cdeg;
		d.hmax_mm = s->imu.max_hump_mm;
		d.range_mm = s->sc.last_range;
		d.faults = s->sm.latched;
		json_check(s, tel_fmt_telemetry(&d, buf, TEL_JSON_MAX));
		json_check(s, tel_fmt_status(&d, buf, TEL_JSON_MAX));
	}
}

static void command_text(SIM *s, const char *text)
{
	CMD		c;
	cmd_err_t	r;
	char		buf[TEL_JSON_MAX];

	r = cmd_parse(text, (uint32_t)strlen(text), &s->last_seq,
		      (s->vc.state == VC_READY) || (s->vc.state == VC_FINISHED), &c);
	if (r == CMD_OK) {
		if (c.verb == CMD_VERB_SET) {
			s->params[c.param] = c.value;
		} else if (!q_push(&s->vcq, SRC_TELEMETRY, EV_COMMAND, c.verb, c.value, s->t_ms)) {
			s->fault_flags |= FLT_QUEUE_OVF;
		} else {
			/* forwarded */
		}
	}
	json_check(s, tel_fmt_ack(s->t_ms, c.seq, r == CMD_OK, (int32_t)r, buf, TEL_JSON_MAX));
	trace(s, "cmd \"%s\" -> %d %d", text, (int32_t)r, 0);
}

/* ---- motion control (mirrors task_motion.c) ----------------------------------- */
static void push_list_vc(SIM *s, uint8_t src, const EVT_LIST *l)
{
	uint32_t i;

	for (i = 0U; i < l->n; i++) {
		if (!q_push(&s->vcq, src, l->ev[i].type, l->ev[i].p1, l->ev[i].p2, s->t_ms)) {
			s->fault_flags |= FLT_QUEUE_OVF;
		}
	}
}

static void motion_step(SIM *s, bool tick)
{
	MC_IN	in;
	MC_OUT	out;
	MC_CMD	cmds[MCQ_CAP];
	int	i;

	in.now_ms = s->t_ms;
	in.now_us = s->t_us;
	in.count[0] = car_count(&s->car, 0);
	in.count[1] = car_count(&s->car, 1);
	in.cmd_stop = (s->stop_flags & MC_CMD_STOP) != 0U;
	in.fault_stop = (s->stop_flags & MC_FAULT_STOP) != 0U;
	in.stop_req_us = s->stop_req_us;
	in.tick = tick;
	in.ncmd = 0U;
	in.cmd = cmds;
	if (tick) {
		for (i = 0; i < s->mq_n; i++) {
			cmds[i] = s->mq[i];
		}
		in.ncmd = (uint32_t)s->mq_n;
		s->mq_n = 0;
	}
	if (s->was_stopped && !in.cmd_stop && !in.fault_stop) {
		mc_core_tune(&s->mc, s->params[2], s->params[3], s->params[4], s->params[5]);
	}
	s->was_stopped = in.cmd_stop || in.fault_stop;

	mc_core_step(&s->mc, &in, &out);
	s->car.duty[0] = out.duty[0];
	s->car.duty[1] = out.duty[1];
	if (s->measure_stop && (out.duty[0] == 0) && (out.duty[1] == 0)) {
		s->stop_latency_us = s->t_us - s->stop_cmd_us;
		s->measure_stop = false;
	}
	if (out.flush) {
		s->mq_n = 0;
	}
	if (out.fault) {
		s->stop_flags |= MC_FAULT_STOP;
		s->fault_flags |= FLT_MOTION;
		trace(s, "MC own fault %s%d %d", "", s->mc.snap.fault_code, 0);
	}
	for (i = 0; i < (int)out.ev.n; i++) {
		trace(s, "MC event %s%d p2 %d", "", out.ev.ev[i].type, out.ev.ev[i].p2);
	}
	push_list_vc(s, SRC_MOTION, &out.ev);
	if (tick) {
		s->snap = s->mc.snap;
		s->health[HEALTH_MOTION] = s->t_ms | 1U;
	}
}

/* ---- safety monitor (mirrors task_safety.c) ------------------------------------- */
static void safety_apply(SIM *s, const SM_OUT *out)
{
	if (out->latch) {
		s->stop_flags |= MC_FAULT_STOP;
		s->stop_req_us = s->t_us;
		motion_step(s, false);			/* the wake-up */
		trace(s, "SAFETY latch %s0x%x %d", "", (int32_t)s->sm.latched, 0);
	}
	if (out->release) {
		s->stop_flags &= ~MC_FAULT_STOP;
		trace(s, "SAFETY release%s %d %d", "", 0, 0);
	}
	if (out->clear_flags) {
		s->fault_flags = 0U;
	}
	if (out->set_restart) {
		s->fault_flags |= FLT_RESTART_REQ;
	}
	s->fault_flags |= out->raise;
	push_list_vc(s, SRC_SAFETY, &out->ev);
}

static void safety_flags(SIM *s)
{
	SM_OUT		out;
	uint32_t	ptn = s->fault_flags & sm_wait_pattern(&s->sm);

	if (ptn != 0U) {
		sm_on_flags(&s->sm, ptn, &out);
		safety_apply(s, &out);
	}
}

static void safety_poll(SIM *s)
{
	SM_OUT		out;
	uint32_t	age[HEALTH_SLOTS];
	bool		alive[HEALTH_SLOTS];
	uint32_t	i;

	for (i = 0U; i < HEALTH_SLOTS; i++) {
		alive[i] = (s->health[i] != 0U);
		age[i] = s->t_ms - s->health[i];
	}
	sm_on_poll(&s->sm, (uint32_t)s->btn[0], (uint32_t)s->btn[1], age, alive, &out);
	safety_apply(s, &out);
}

/* ---- line and barcode (mirrors task_line.c) -------------------------------------- */
static void line_step(SIM *s)
{
	EVT_LIST	ev;
	BC_RESULT	res;
	int		i;
	bool		l = car_sensor_black(&s->car, &s->w, CAR_LINE_FWD, CAR_LINE_HALF);
	bool		r = car_sensor_black(&s->car, &s->w, CAR_LINE_FWD, -CAR_LINE_HALF);

	evt_list_clear(&ev);
	lc_step(&s->lc, l, r, s->t_ms, s->snap.odo_mm, &ev);
	push_list_vc(s, SRC_LINE, &ev);
	if ((getenv("SIM_LINE") != NULL) && (ev.n > 0U) && (ev.ev[0].type == EV_LINE_STATUS)) {
		trace(s, "line %s%d%d", "", l ? 1 : 0, r ? 1 : 0);
	}

	for (i = 0; i < s->edge_n; i++) {
		bc_edge(&s->bc, s->edge_t[i], s->edge_l[i]);
	}
	s->edge_n = 0;
	if (bc_poll(&s->bc, s->t_us, &res)) {
		evt_list_clear(&ev);
		if (res.result == C39_OK) {
			(void)evt_list_add(&ev, EV_BARCODE, (int32_t)res.c, bc_command_for(res.c));
		} else {
			(void)evt_list_add(&ev, EV_BARCODE_BAD, (int32_t)res.result, (int32_t)res.elements);
		}
		push_list_vc(s, SRC_LINE, &ev);
	}
	s->health[HEALTH_LINE] = s->t_ms | 1U;
}

/* ---- IMU (mirrors task_imu.c) ------------------------------------------------------ */
static void imu_step(SIM *s)
{
	int32_t		a[3];
	IMU_MOTION	mot;
	IMU_OUT		out;
	uint32_t	i;

	imu_read(&s->car, 12.0, a);
	mot.valid = true;
	mot.dist_mm = s->snap.dist_mm;
	mot.speed_mm_s = s->snap.speed_mm_s;
	mot.accel_mm_s2 = s->snap.accel_mm_s2;
	mot.yaw_cdeg_s = s->snap.yaw_cdeg_s;
	imu_core_step(&s->imu, a, !s->imu_fail, &mot, s->t_ms, &out);
	if (out.fault && (IMU_REQUIRED != 0)) {
		s->fault_flags |= FLT_IMU;
	}
	if (out.impact && (IMU_IMPACT_STOPS != 0)) {
		s->fault_flags |= FLT_IMPACT;
	}
	for (i = 0U; i < out.ev.n; i++) {
		const APP_MSG *m = &out.ev.ev[i];

		if ((m->type != EV_MOTION) && ((m->type != EV_IMPACT) || (IMU_IMPACT_STOPS != 0))) {
			if (!q_push(&s->vcq, SRC_IMU, m->type, m->p1, m->p2, s->t_ms)) {
				s->fault_flags |= FLT_QUEUE_OVF;
			}
		}
		(void)q_push(&s->telq, SRC_IMU, m->type, m->p1, m->p2, s->t_ms);
		if (m->type == EV_HUMP) {
			trace(s, "HUMP %s%d mm (max %d)", "", m->p1, m->p2);
		} else if ((m->type == EV_TERRAIN) && (getenv("SIM_TERRAIN") != NULL)) {
			trace(s, "terrain %s%d pitch %d cdeg", "", m->p1, m->p2);
		} else {
			/* motion classes are not traced */
		}
	}
	s->health[HEALTH_IMU] = s->t_ms | 1U;
}

/* ---- obstacle scan (mirrors task_scan.c and the HC-SR04 driver) -------------------- */
static void scan_step(SIM *s)
{
	SCAN_IN		in;
	SCAN_OUT	out;
	int		i;

	memset(&in, 0, sizeof(in));
	in.now_ms = s->t_ms;
	for (i = 0; i < s->sq_n; i++) {
		in.req = s->sq[i];			/* only the newest one counts */
		in.have_req = true;
	}
	s->sq_n = 0;
	if (s->ping_refused) {
		in.echo = SCAN_ECHO_MISSING;
		s->ping_refused = false;
	} else if (s->ping_in_flight && ((int32_t)(s->t_us - s->ping_done_us) >= 0)) {
		s->ping_in_flight = false;
		if (s->ping_range < 0.0) {
			in.echo = SCAN_ECHO_FAR;
		} else {
			in.echo = SCAN_ECHO_DONE;
			in.range_mm = (int32_t)lround(s->ping_range);
		}
	} else {
		in.echo = SCAN_ECHO_NONE;
	}
	in.echo_high = (int32_t)(s->echo_high_until_us - s->t_us) > 0;

	scan_core_step(&s->sc, &in, &out);
	s->car.servo_cmd = out.servo_deg;
	if (out.trigger) {
		if (in.echo_high || s->ping_in_flight) {
			s->ping_refused = true;
		} else {
			s->ping_range = us_range(&s->car, &s->w);
			s->ping_in_flight = true;
			if (s->ping_range < 0.0) {
				s->ping_done_us = s->t_us + 30000U;	/* the driver's timeout */
				s->echo_high_until_us = s->t_us + 38000U;
			} else {
				s->ping_done_us = s->t_us + 500U + (uint32_t)(s->ping_range * 2.0 / 0.343);
				s->echo_high_until_us = s->ping_done_us;
			}
		}
	}
	if (out.fault && (OS_REQUIRED != 0)) {
		s->fault_flags |= FLT_SCAN;
	}
	if (out.profile_ready) {
		s->prof = s->sc.profile;
		(void)q_push(&s->telq, SRC_SCAN, EV_PROFILE_DATA, 0, 0, s->t_ms);
		if (s->verbose) {
			printf("%7.3f  PROFILE valid %d x %d..%d closest %d at %d clear L %d R %d unk L %u R %u\n",
			       s->t_ms / 1000.0, s->prof.valid, s->prof.x_left_mm, s->prof.x_right_mm,
			       s->prof.closest_mm, s->prof.closest_deg, s->prof.left_clear_mm,
			       s->prof.right_clear_mm, s->prof.unknown_left, s->prof.unknown_right);
		}
	}
	push_list_vc(s, SRC_SCAN, &out.ev);
	s->health[HEALTH_SCAN] = s->t_ms | 1U;
}

/* ---- vehicle controller (mirrors task_vc.c) ------------------------------------------ */
static bool sim_motion_send(SIM *s, uint8_t type, int32_t a, int32_t b, int32_t c)
{
	MC_CMD cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.type = type;
	cmd.a = a;
	cmd.b = b;
	cmd.c = c;
	if (!mc_cmd_valid(&cmd, MC_MAX_SPEED) || (s->mq_n >= MCQ_CAP)) {
		return false;
	}
	s->mq[s->mq_n++] = cmd;
	return true;
}

static void run_actions(SIM *s, const VC_ACTIONS *act)
{
	uint32_t i;

	for (i = 0U; i < act->n; i++) {
		const VC_ACTION *a = &act->a[i];

		switch (a->type) {
		case VA_HOLD:
			if (getenv("SIM_HOLD") != NULL) {
				trace(s, "hold %s%d %d", "", a->a, a->b);
			}
			(void)sim_motion_send(s, MC_CMD_SPEED, a->a, a->b, 0);
			break;
		case VA_MOVE:
			trace(s, "MOVE %s%d mm id %d", "", a->a, a->c);
			if (!sim_motion_send(s, MC_CMD_DISTANCE, a->a, a->b, a->c)) {
				s->queue_errors++;
				s->fault_flags |= FLT_QUEUE_OVF;
			}
			break;
		case VA_TURN:
			trace(s, "TURN %s%d deg id %d", "", a->a, a->c);
			if (!sim_motion_send(s, MC_CMD_TURN, a->a, 0, a->c)) {
				s->queue_errors++;
				s->fault_flags |= FLT_QUEUE_OVF;
			}
			break;
		case VA_STOP:
			s->stop_flags |= MC_CMD_STOP;
			s->stop_req_us = s->t_us;
			motion_step(s, false);		/* tk_wup_tsk: acts at once */
			break;
		case VA_CLEAR_STOP:
			s->stop_flags &= ~MC_CMD_STOP;
			break;
		case VA_SCAN:
			if (s->sq_n < SCQ_CAP) {
				s->sq[s->sq_n].mode = (uint32_t)a->a;
				s->sq[s->sq_n].stop_mm = a->b;
				s->sq[s->sq_n].id = a->c;
				s->sq_n++;
			} else {
				s->queue_errors++;
				s->fault_flags |= FLT_QUEUE_OVF;
			}
			break;
		case VA_FAULT:
			s->fault_flags |= (uint32_t)a->a;
			break;
		case VA_RESTART:
			s->fault_flags |= FLT_RESTART_REQ;
			break;
		case VA_TEL:
			(void)q_push(&s->telq, SRC_VC, (uint8_t)a->a, a->b, a->c, s->t_ms);
			if (a->a == EV_STATE) {
				trace(s, "VC -> %s (why %d) %d", vc_state_name(a->b), a->c, 0);
			} else if (a->a == EV_NAV) {
				trace(s, "nav %s %d (%d)", vc_nav_name(a->b), a->c, 0);
			} else if (a->a == EV_BARCODE) {
				trace(s, "barcode %s'%c' cmd %d", "", a->b, a->c);
			} else {
				/* forwarded */
			}
			break;
		default:
			break;
		}
	}
}

static void vc_drain(SIM *s)
{
	APP_MSG		m;
	VC_ACTIONS	act;

	while (q_pop(&s->vcq, &m)) {
		if (m.type == EV_PROFILE) {
			vc_profile(&s->vc, &m, &s->prof, s->t_ms, s->snap.odo_mm, &act);
		} else {
			if (((m.type == EV_COMMAND) && (m.p1 == CMD_START)) ||
			    ((m.type == EV_BUTTON) && (m.p1 == BTN_START))) {
				vc_tune(&s->vc, s->params[0], s->params[1]);
			}
			vc_event(&s->vc, &m, s->t_ms, s->snap.odo_mm, &act);
		}
		run_actions(s, &act);
		safety_flags(s);
	}
}

/* ---- parameters, exactly as the task files build them ----------------------------------- */
static const MC_PARAMS k_mc = {
	MC_COUNTS_PER_M, MC_COUNTS_PER_90DEG, MC_TRACK_MM, MC_KFF_X1000, MC_DEADBAND,
	MC_KP_X1000, MC_KI_X1000, MC_MAX_SPEED, MC_MIN_SPEED, MC_ACCEL, MC_TURN_SPEED,
	MC_K_STRAIGHT, MC_SETTLE_SPEED, MC_STALL_DUTY, MC_STALL_SPEED, MC_STALL_TICKS,
	MC_TIMEOUT_MARGIN_MS, MC_SPEED_WINDOW
};
static const LC_PARAMS k_lc = {LB_JUNCTION_MS, LB_JUNCTION_MM, LB_T_LOST_MS};
static const BC_PARAMS k_bc = {LB_T_MIN_US, LB_T_END_MIN_MS * 1000U, LB_T_END_FACTOR, (uint8_t)LB_BAR_LEVEL};
static const LS_PARAMS k_ls = {
	LS_D_MIN, LS_D_RATE, LS_D_MAX, LS_D_BIAS, LS_D_SEARCH, LS_T_ESCAPE_MS, LS_T_DRIFT_MS, LB_T_LOST_MS,
	LS_SLOW_PCT, LS_START_SIDE
};
static const IMU_PARAMS k_imu = {
	IMU_AXIS_FWD, IMU_SIGN_FWD, IMU_AXIS_UP, IMU_SIGN_UP, IMU_AXIS_LAT, IMU_SIGN_LAT,
	IMU_CAL_SAMPLES, IMU_LPF_SHIFT, IMU_CLIMB_CDEG, IMU_LEVEL_CDEG, IMU_TERRAIN_MM, IMU_HUMP_END_MM, IMU_HUMP_END_MS,
	IMU_PRE_SAMPLES, IMU_HUMP_MIN_MM, IMU_WHEELBASE_MM, IMU_IMPACT_MG, IMU_IMPACT_WINDOW_MS,
	IMU_IMPACT_DROP_PCT, IMU_IMPACT_MIN_SPEED, IMU_STILL_SPEED, IMU_STILL_DEV_MG,
	IMU_ACCEL_MM_S2, IMU_TURN_CDEG_S, IMU_CLASS_HOLD, IMU_FAIL_LIMIT
};
static const SM_PARAMS k_sm = {
	{SM_HEALTH_MOTION_MS, SM_HEALTH_LINE_MS, SM_HEALTH_IMU_MS, SM_HEALTH_SCAN_MS, SM_HEALTH_VC_MS},
	SM_RESTART_POLLS
};
static const SCAN_PARAMS k_sc = {
	OS_STOP_MM, OS_SLOW_MM, OS_SLOW_HYST_MM, OS_CONFIRM, OS_GAP_MS, OS_SETTLE_BASE_MS,
	OS_SETTLE_MS_PER_DEG, OS_RETRIES, OS_R_MAX_MM, OS_CORRIDOR_MM, OS_COARSE_STEP, OS_FINE_STEP,
	OS_FINE_MARGIN, OS_MISSING_LIMIT, OS_PROFILE_MAX_MS
};
static const VC_PARAMS k_vc = {
	VC_BASE_SPEED, VC_SLOW_SPEED, VC_HUMP_SPEED, VC_JUNCTION_SPEED, VC_MOVE_SPEED,
	VC_FIND_SPEED, VC_CAR_WIDTH_MM, VC_MARGIN_MM, VC_US_TO_AXLE_MM, VC_LINE_TO_AXLE_MM,
	VC_REAR_CLEAR_MM, VC_OBST_DEPTH_MM, VC_LINE_HALF_MM, VC_STOP_MM, VC_BYPASS_STOP_MM,
	VC_BYPASS_DEG, VC_BACK_MM, VC_N_RETRY, VC_TIE_MM, VC_PREF_SIDE, VC_FIND_EXTRA_MM,
	VC_ALIGN_EXTRA_MM, VC_D_EXEC_MM, VC_JUNCTION_MUTE_MM, VC_CROSS_MM, VC_ACQUIRE_MM, VC_T_STARTUP_MS,
	VC_T_ACK_MS, VC_T_SCAN_MS, VC_T_SEARCH_MS, VC_T_EXEC_MS, VC_T_BYPASS_MS,
	VC_HUMP_GRACE_MS, VC_HOLD_REFRESH_MS
};

static void sim_init(SIM *s, const char *name, double x, double y, double th_deg)
{
	int32_t cnt[2] = {0, 0};
	int i;

	memset(s, 0, sizeof(*s));
	srand(g_seed);
	s->name = name;
	w_init(&s->w);
	car_init(&s->car, x, y, th_deg);
	s->vcq.cap = VCQ_CAP;
	s->telq.cap = TELQ_CAP;
	s->stop_flags = MC_CMD_STOP;		/* READY until START */
	s->btn[0] = 1;
	s->btn[1] = 1;
	for (i = 0; i < (int)CMD_NPARAMS; i++) {
		s->params[i] = -1;
	}
	mc_core_init(&s->mc, &k_mc, cnt);
	s->was_stopped = true;
	lc_init(&s->lc, &k_lc, 0U);
	bc_init(&s->bc, &k_bc);
	imu_core_init(&s->imu, &k_imu);
	sm_init(&s->sm, &k_sm);
	scan_core_init(&s->sc, &k_sc);
	vc_init(&s->vc, &k_vc, &k_ls, 0U);
}

static void script(SIM *s, uint32_t t_ms, int kind, int a, const char *text)
{
	if (s->nscript < MAX_SCRIPT) {
		s->script[s->nscript].t_ms = t_ms;
		s->script[s->nscript].kind = kind;
		s->script[s->nscript].a = a;
		s->script[s->nscript].text = text;
		s->nscript++;
	}
}

/* ---- one millisecond ------------------------------------------------------------------- */
static void sim_ms(SIM *s)
{
	int	k;
	VC_ACTIONS act;

	/* physics in 100 us steps; barcode edges timestamped like the ISR */
	for (k = 0; k < 10; k++) {
		bool lvl;

		car_step(&s->car, &s->w, 0.0001);
		s->t_us += 100U;
		lvl = car_sensor_black(&s->car, &s->w, CAR_LINE_FWD, CAR_BC_LEFT);
		if ((lvl != s->bc_level) && (s->edge_n < EDGE_CAP)) {
			s->edge_t[s->edge_n] = s->t_us;
			s->edge_l[s->edge_n] = lvl ? 1U : 0U;
			s->edge_n++;
		}
		s->bc_level = lvl;
	}
	s->t_ms++;
	if (s->car.crashed && (s->crash_ms == 0U)) {
		s->crash_ms = s->t_ms;
	}

	for (k = 0; k < s->nscript; k++) {
		const SCRIPT *e = &s->script[k];

		if (e->t_ms == s->t_ms) {
			if (e->kind == SC_CMD) {
				if (strstr(e->text, "STOP") != NULL) {
					s->stop_cmd_us = s->t_us;
					s->measure_stop = true;
				}
				command_text(s, e->text);
			} else if (e->kind == SC_BUTTON) {
				s->btn[(e->a == 20) ? 0 : 1] = 0;
			} else {
				s->imu_fail = (e->a != 0);
			}
		}
		if ((e->kind == SC_BUTTON) && (e->t_ms + 100U == s->t_ms)) {
			s->btn[(e->a == 20) ? 0 : 1] = 1;	/* released after 100 ms */
		}
	}

	if ((s->t_ms % MC_PERIOD_MS) == 0U) {
		motion_step(s, true);
	}
	if ((s->t_ms % LB_PERIOD_MS) == 1U) {
		line_step(s);
	}
	if ((s->t_ms % IMU_PERIOD_MS) == 3U) {
		imu_step(s);
	}
	if ((s->t_ms % OS_PERIOD_MS) == 7U) {
		scan_step(s);
	}
	vc_drain(s);
	if ((s->t_ms % VC_TICK_MS) == 10U) {
		bool ready = s->imu.calibrated || (IMU_REQUIRED == 0);

		vc_tick(&s->vc, s->t_ms, s->snap.odo_mm, ready, &act);
		run_actions(s, &act);
		s->health[HEALTH_VC] = s->t_ms | 1U;
		vc_drain(s);
	}
	safety_flags(s);
	if ((s->t_ms % SM_PERIOD_MS) == 15U) {
		safety_poll(s);
		vc_drain(s);
	}
	if ((s->t_ms % 100U) == 50U) {
		telemetry_step(s, (s->t_ms % 500U) == 50U, (s->t_ms % 1000U) == 50U);
	}

	/* how far is the axle from the line? */
	{
		double d = w_line_dist(&s->w, s->car.x, s->car.y);

		if (d > s->max_dev_all) {
			s->max_dev_all = d;
		}
		if ((s->vc.state == VC_FOLLOW) && (s->t_ms > 2000U)) {
			s->follow_ms++;
			s->sum_dev += d;
			if (d > s->max_dev_follow) {
				s->max_dev_follow = d;
			}
		}
	}
}

static void sim_run(SIM *s, uint32_t until_ms)
{
	while (s->t_ms < until_ms) {
		sim_ms(s);
	}
	telemetry_step(s, false, false);
}

/* ---- checks -------------------------------------------------------------------------------- */
static int g_fail;
static int g_checks;

static void check(SIM *s, bool ok, const char *what)
{
	g_checks++;
	if (!ok) {
		g_fail++;
	}
	printf("  [%s] %-16s %s\n", ok ? "PASS" : "FAIL", s->name, what);
}

static void report(SIM *s)
{
	printf("  %-16s end: state %s, x %.0f y %.0f th %.0f, odo %d mm, follow dev max %.1f mean %.1f, "
	       "humps %d, barcodes %d (bad %d), json max %d\n",
	       s->name, vc_state_name(s->vc.state), s->car.x, s->car.y, s->car.th * 180.0 / PI,
	       s->snap.odo_mm, s->max_dev_follow, (s->follow_ms > 0U) ? (s->sum_dev / s->follow_ms) : 0.0,
	       s->nhumps, s->nbarcodes, s->bad_reads, s->json_max);
}

static bool common_ok(SIM *s)
{
	return (s->json_fail == 0) && (s->vcq.drops == 0) && (s->queue_errors == 0);
}

static double heading_deg(const SIM *s)
{
	double h = fmod(s->car.th * 180.0 / PI, 360.0);

	if (h > 180.0) {
		h -= 360.0;
	}
	if (h < -180.0) {
		h += 360.0;
	}
	return h;
}

/* ---- scenarios ------------------------------------------------------------------------------ */
static void oval(SIM *s, double r, bool cw)
{
	double L = 1200.0;

	if (!cw) {			/* counter-clockwise: left turns */
		w_line(&s->w, -200.0, 0.0, L, 0.0);
		w_arc(&s->w, L, r, r, -90.0, 90.0);
		w_line(&s->w, L, 2.0 * r, -200.0, 2.0 * r);
		w_arc(&s->w, -200.0, r, r, 90.0, 270.0);
	} else {			/* clockwise: right turns */
		w_line(&s->w, -200.0, 0.0, L, 0.0);
		w_arc(&s->w, L, -r, r, 90.0, -90.0);
		w_line(&s->w, L, -2.0 * r, -200.0, -2.0 * r);
		w_arc(&s->w, -200.0, -r, r, 270.0, 90.0);
	}
}

static void sc_oval(bool verbose, const char *name, double r, bool cw, const char *set)
{
	SIM *s = calloc(1, sizeof(SIM));

	sim_init(s, name, 0.0, 0.0, 0.0);
	s->verbose = verbose;
	oval(s, r, cw);
	if (set != NULL) {
		script(s, 1400U, SC_CMD, 0, set);
	}
	script(s, 1500U, SC_CMD, 0, "7 START");
	sim_run(s, 40000U);
	report(s);
	check(s, (s->states_seen & (1U << VC_SEARCH)) == 0U, "never lost the line");
	check(s, s->max_dev_follow < 25.0, "axle within 25 mm of the line");
	check(s, s->snap.odo_mm > (s->vc.p.base_speed * 24), "average speed at least 65 % of the base speed");
	check(s, s->vc.state == VC_FOLLOW, "still following at the end");
	check(s, common_ok(s), "no queue overflow, every message <= 200 bytes");
	free(s);
}

static void sc_barcode(bool verbose, const char *name, char c, int expect, bool backwards)
{
	SIM *s = calloc(1, sizeof(SIM));
	double hd;

	sim_init(s, name, 0.0, 0.0, 0.0);
	s->verbose = verbose;
	w_line(&s->w, -200.0, 0.0, 2400.0, 0.0);
	if (backwards) {
		w_barcode(&s->w, 441.0, 0.0, 180.0, c, true);	/* laid the other way round */
	} else {
		w_barcode(&s->w, 300.0, 0.0, 0.0, c, false);
	}
	w_line(&s->w, 900.0, -1500.0, 900.0, 1500.0);		/* a + junction */
	script(s, 1500U, SC_CMD, 0, "1 START");
	sim_run(s, (expect == LB_CMD_U_TURN) ? 12500U : 14000U);
	report(s);
	hd = heading_deg(s);
	check(s, (s->nbarcodes >= 1) && (s->barcodes[0] == c), "barcode decoded at speed");
	switch (expect) {
	case LB_CMD_TURN_LEFT:
		check(s, (s->car.y > 500.0) && (fabs(s->car.x - 900.0) < 40.0) && (fabs(hd - 90.0) < 20.0),
		      "turned left and follows the left branch");
		break;
	case LB_CMD_TURN_RIGHT:
		check(s, (s->car.y < -500.0) && (fabs(s->car.x - 900.0) < 40.0) && (fabs(hd + 90.0) < 20.0),
		      "turned right and follows the right branch");
		break;
	case LB_CMD_STRAIGHT:
		check(s, (s->car.x > 1500.0) && (fabs(s->car.y) < 40.0), "went straight across");
		break;
	case LB_CMD_U_TURN:
		check(s, (s->car.x < 750.0) && (fabs(s->car.y) < 40.0) && (fabs(hd) > 160.0),
		      "turned round and follows the line back");
		break;
	default:
		check(s, (s->car.x > 1500.0) && (fabs(s->car.y) < 40.0) && (s->nav[NAV_EXEC] == 0),
		      "no command: straight on");
		break;
	}
	check(s, (s->states_seen & (1U << VC_SAFE_STOP)) == 0U, "no Safe Stop");
	check(s, common_ok(s), "no queue overflow, every message <= 200 bytes");
	free(s);
}

static void sc_obstacle(bool verbose, const char *name, double y1, double y2, bool wall_left,
			int expect_side)
{
	SIM *s = calloc(1, sizeof(SIM));

	sim_init(s, name, 0.0, 0.0, 0.0);
	s->verbose = verbose;
	w_line(&s->w, -200.0, 0.0, 4000.0, 0.0);
	w_box(&s->w, 1000.0, y1, 1120.0, y2, true);
	if (wall_left) {
		w_box(&s->w, 800.0, 200.0, 1400.0, 220.0, true);
	}
	script(s, 1500U, SC_CMD, 0, "1 START");
	sim_run(s, 30000U);
	report(s);
	check(s, s->car.crash_count == 0, "no collision");
	check(s, s->nav[NAV_BYPASS] == 1, "one bypass");
	check(s, (expect_side == 0) || (s->nav[NAV_BYPASS] == 0) ||
	      ((expect_side < 0) ? (s->nav_last[NAV_BYPASS] < 0) : (s->nav_last[NAV_BYPASS] > 0)),
	      (expect_side == 0) ? "either side (centred box)" :
	      ((expect_side < 0) ? "passed on the left (shorter detour)" : "passed on the right (wall on the left)"));
	check(s, s->nav[NAV_REJOINED] == 1, "rejoined the line");
	check(s, (s->vc.state == VC_FOLLOW) && (s->car.x > 2000.0) && (fabs(s->car.y) < 30.0),
	      "following the original line past the obstacle");
	check(s, s->max_dev_all < 450.0, "detour stays within 450 mm of the line");
	check(s, common_ok(s), "no queue overflow, every message <= 200 bytes");
	free(s);
}

static void sc_hump(bool verbose)
{
	SIM *s = calloc(1, sizeof(SIM));

	sim_init(s, "hump", 0.0, 0.0, 0.0);
	s->verbose = verbose;
	w_line(&s->w, -200.0, 0.0, 5000.0, 0.0);
	w_hump(&s->w, 800.0, 150.0, 12.0);		/* short, lower */
	w_hump(&s->w, 2000.0, 400.0, 25.0);		/* long, higher */
	script(s, 1500U, SC_CMD, 0, "1 START");
	sim_run(s, 25000U);
	report(s);
	if (s->nhumps >= 2) {
		printf("  %-16s hump estimates %d mm (true 12) and %d mm (true 25), highest %d\n",
		       s->name, s->humps[0], s->humps[1], s->imu.max_hump_mm);
	}
	check(s, s->nhumps == 2, "two humps reported");
	check(s, (s->nhumps >= 1) && (s->humps[0] >= 8) && (s->humps[0] <= 16), "first hump 12 mm +/- 4");
	check(s, (s->nhumps >= 2) && (s->humps[1] >= 19) && (s->humps[1] <= 31), "second hump 25 mm +/- 6");
	check(s, (s->imu.max_hump_mm >= 19) && (s->imu.max_hump_mm <= 31), "highest hump of the run");
	check(s, (s->states_seen & ((1U << VC_SEARCH) | (1U << VC_SAFE_STOP))) == 0U,
	      "kept the line, no Safe Stop");
	check(s, (s->vc.state == VC_FOLLOW) && (s->car.x > 3000.0), "still following after both humps");
	check(s, common_ok(s), "no queue overflow, every message <= 200 bytes");
	free(s);
}

static void sc_dead_end(bool verbose)
{
	SIM *s = calloc(1, sizeof(SIM));

	sim_init(s, "dead_end", 0.0, 0.0, 0.0);
	s->verbose = verbose;
	w_line(&s->w, -200.0, 0.0, 700.0, 0.0);
	script(s, 1500U, SC_CMD, 0, "1 START");
	script(s, 20000U, SC_CMD, 0, "2 RESTART");
	script(s, 21000U, SC_CMD, 0, "3 START");
	sim_run(s, 19000U);
	check(s, (s->vc.state == VC_SAFE_STOP) && (s->last_why == VC_WHY_SEARCH_FAILED),
	      "search fails -> Safe Stop (search failed)");
	check(s, (s->car.duty[0] == 0.0) && (s->car.duty[1] == 0.0) && s->sm.faulted,
	      "motors off, fault latched");
	sim_run(s, 20500U);
	check(s, (s->vc.state == VC_READY) && !s->sm.faulted, "RESTART clears the fault -> READY");
	sim_run(s, 21500U);
	check(s, s->vc.state == VC_FOLLOW, "START runs again");
	report(s);
	check(s, common_ok(s), "no queue overflow, every message <= 200 bytes");
	free(s);
}

static void sc_stop_start(bool verbose)
{
	SIM *s = calloc(1, sizeof(SIM));

	sim_init(s, "stop_start", 0.0, 0.0, 0.0);
	s->verbose = verbose;
	w_line(&s->w, -200.0, 0.0, 5000.0, 0.0);
	script(s, 1500U, SC_CMD, 0, "1 START");
	script(s, 4000U, SC_CMD, 0, "2 STOP");
	script(s, 4500U, SC_CMD, 0, "2 START");		/* repeated seq: refused */
	script(s, 5000U, SC_CMD, 0, "3 SET base_speed 250");
	script(s, 5200U, SC_CMD, 0, "4 START");
	script(s, 9000U, SC_BUTTON, 21, NULL);		/* STOP button */
	script(s, 9500U, SC_CMD, 0, "5 FINISH");
	sim_run(s, 4100U);
	check(s, (s->vc.state == VC_READY) && (s->stop_latency_us < 1000U),
	      "STOP: READY, duty 0 within 1 ms");
	sim_run(s, 5100U);
	check(s, s->vc.state == VC_READY, "repeated sequence number ignored");
	sim_run(s, 8000U);
	check(s, (s->vc.state == VC_FOLLOW) && (s->vc.p.base_speed == 250), "SET used from the next START");
	sim_run(s, 9100U);
	check(s, s->vc.state == VC_READY, "STOP button");
	sim_run(s, 10000U);
	check(s, (s->vc.state == VC_FINISHED) && (s->nav[NAV_FINISH] == 1), "FINISH and run summary");
	report(s);
	check(s, common_ok(s), "no queue overflow, every message <= 200 bytes");
	free(s);
}

static void sc_imu_fault(bool verbose)
{
	SIM *s = calloc(1, sizeof(SIM));

	sim_init(s, "imu_fault", 0.0, 0.0, 0.0);
	s->verbose = verbose;
	w_line(&s->w, -200.0, 0.0, 5000.0, 0.0);
	script(s, 1500U, SC_CMD, 0, "1 START");
	script(s, 3000U, SC_IMU_FAIL, 1, NULL);
	script(s, 4000U, SC_IMU_FAIL, 0, NULL);
	script(s, 3500U, SC_BUTTON, 20, NULL);		/* RESTART too early: fault persists */
	script(s, 4500U, SC_BUTTON, 20, NULL);		/* START button while faulted = RESTART */
	script(s, 5000U, SC_BUTTON, 20, NULL);		/* START */
	sim_run(s, 3100U);
	check(s, (s->vc.state == VC_SAFE_STOP) && ((s->sm.latched & FLT_IMU) != 0U) &&
	      (s->car.duty[0] == 0.0), "IMU silent -> FLT_IMU -> Safe Stop, motors off");
	sim_run(s, 3900U);
	check(s, s->vc.state == VC_SAFE_STOP, "restart refused while the IMU is still silent");
	sim_run(s, 4900U);
	check(s, (s->vc.state == VC_READY) && !s->sm.faulted, "IMU back + RESTART -> READY");
	sim_run(s, 6000U);
	check(s, s->vc.state == VC_FOLLOW, "START button runs again");
	report(s);
	check(s, common_ok(s), "no queue overflow");
	free(s);
}

static void sc_impact(bool verbose)
{
	SIM *s = calloc(1, sizeof(SIM));

	sim_init(s, "impact", 0.0, 0.0, 0.0);
	s->verbose = verbose;
	w_line(&s->w, -200.0, 0.0, 5000.0, 0.0);
	w_box(&s->w, 1000.0, -60.0, 1100.0, 60.0, false);	/* the sonar cannot see it */
	script(s, 1500U, SC_CMD, 0, "1 START");
	sim_run(s, 12000U);
	report(s);
	check(s, s->crash_ms != 0U, "the car hit the hidden obstacle");
	check(s, (s->vc.state == VC_SAFE_STOP) &&
	      ((s->last_why == VC_WHY_IMPACT) || (s->last_why == VC_WHY_FAULT)),
	      "collision -> Safe Stop");
	check(s, (s->impact_ms != 0U) && (s->impact_ms - s->crash_ms < 300U),
	      "impact detected within 300 ms");
	check(s, (s->car.duty[0] == 0.0) && (s->car.duty[1] == 0.0), "motors off");
	free(s);
}

static void sc_gap(bool verbose)
{
	SIM *s = calloc(1, sizeof(SIM));

	sim_init(s, "line_gap", 0.0, 0.0, 0.0);
	s->verbose = verbose;
	w_line(&s->w, -200.0, 0.0, 900.0, 0.0);
	w_line(&s->w, 960.0, 0.0, 3000.0, 0.0);		/* a 60 mm gap */
	script(s, 1500U, SC_CMD, 0, "1 START");
	sim_run(s, 14000U);
	report(s);
	check(s, (s->states_seen & (1U << VC_SEARCH)) == 0U, "drove across a 60 mm gap");
	check(s, (s->car.x > 1500.0) && (fabs(s->car.y) < 30.0), "following after the gap");
	free(s);
}

typedef struct {
	const char	*name;
	int		id;
} SCN;

static const SCN k_scn[] = {
	{"oval_left", 1}, {"oval_right", 2}, {"oval_tight", 3}, {"oval_fast", 4}, {"oval_slow", 5},
	{"barcode_A", 10}, {"barcode_B", 11}, {"barcode_C", 12}, {"barcode_D", 13}, {"barcode_Z", 14}, {"barcode_A_back", 15},
	{"obstacle_mid", 20}, {"obstacle_right", 21}, {"obstacle_wall", 22},
	{"hump", 30}, {"dead_end", 31}, {"stop_start", 32}, {"imu_fault", 33}, {"impact", 34},
	{"line_gap", 35}
};

static void run_one(int id, bool v)
{
	switch (id) {
	case 1:  sc_oval(v, "oval_left", 300.0, false, NULL); break;
	case 2:  sc_oval(v, "oval_right", 300.0, true, NULL); break;
	case 3:  sc_oval(v, "oval_tight", 200.0, false, NULL); break;
	case 4:  sc_oval(v, "oval_fast", 300.0, false, "1 SET base_speed 300"); break;
	case 5:  sc_oval(v, "oval_slow", 200.0, true, "1 SET base_speed 120"); break;
	case 10: sc_barcode(v, "barcode_A", 'A', LB_CMD_TURN_LEFT, false); break;
	case 11: sc_barcode(v, "barcode_B", 'B', LB_CMD_TURN_RIGHT, false); break;
	case 12: sc_barcode(v, "barcode_C", 'C', LB_CMD_STRAIGHT, false); break;
	case 13: sc_barcode(v, "barcode_D", 'D', LB_CMD_U_TURN, false); break;
	case 14: sc_barcode(v, "barcode_Z", 'Z', LB_CMD_NONE, false);
		break;
	case 15: sc_barcode(v, "barcode_A_back", 'A', LB_CMD_TURN_LEFT, true); break;
	case 20: sc_obstacle(v, "obstacle_mid", -60.0, 60.0, false, 0); break;
	case 21: sc_obstacle(v, "obstacle_right", -100.0, 20.0, false, -1); break;
	case 22: sc_obstacle(v, "obstacle_wall", -60.0, 60.0, true, 1); break;
	case 30: sc_hump(v); break;
	case 31: sc_dead_end(v); break;
	case 32: sc_stop_start(v); break;
	case 33: sc_imu_fault(v); break;
	case 34: sc_impact(v); break;
	case 35: sc_gap(v); break;
	default: break;
	}
}

int main(int argc, char **argv)
{
	size_t	i;
	bool	verbose = (argc > 2) && (strcmp(argv[1], "-v") == 0);

	/* robustness run: sim -p SEED DEADBAND KFF TRACK COUNTS_PER_MM RIGHT_EFF */
	if ((argc >= 8) && (strcmp(argv[1], "-p") == 0)) {
		g_seed = (unsigned)atoi(argv[2]);
		g_db_true = atof(argv[3]);
		g_kff_true = atof(argv[4]);
		g_track_true = atof(argv[5]);
		g_counts_mm = atof(argv[6]);
		g_right_eff = atof(argv[7]);
		printf("robustness run: seed %u deadband %.0f kff %.2f track %.0f counts/mm %.2f right %.2f\n",
		       g_seed, g_db_true, g_kff_true, g_track_true, g_counts_mm, g_right_eff);
		if ((argc == 10) && (strcmp(argv[8], "-v") == 0)) {
			argv[2] = argv[9];
			verbose = true;
			argc = 3;
		} else {
			argc = 1;
		}
	}

	for (i = 0; i < sizeof(k_scn) / sizeof(k_scn[0]); i++) {
		if ((argc > 2) && (strcmp(argv[2], k_scn[i].name) != 0)) {
			continue;
		}
		run_one(k_scn[i].id, verbose);
	}
	printf("\nSIMULATION: %d checks, %d failed\n", g_checks, g_fail);
	return (g_fail == 0) ? 0 : 1;
}
