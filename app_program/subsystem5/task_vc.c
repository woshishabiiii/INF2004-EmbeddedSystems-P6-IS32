/*
 *	task_vc.c
 *	Vehicle Controller task (Figure 4, and the VC states of Figure 7),
 *	core 1, priority 9.  The statechart is in vc_core.c; this file is the
 *	RTOS side.
 *
 *	It waits on ONE inbox, vc_event_q, but never longer than the next
 *	20 ms tick, so its timers and the steering still run when no event
 *	arrives.  Every step returns a list of actions, carried out here in
 *	order: this task is the only caller of the motion API and the only
 *	sender of scan requests.
 */

#include "subsystem5.h"
#include "vc_cfg.h"
#include "../subsystem1/subsystem1.h"	/* telemetry_q, param_get() */
#include "../subsystem2/subsystem2.h"	/* the motion API */
#include "../subsystem2/motion_cfg.h"	/* MC_TRACK_MM, MC_TURN_SPEED: a spin's duration */
#include "../subsystem3/subsystem3.h"	/* lb_steer_params */
#include "../subsystem4/subsystem4.h"	/* fault_flags, health_stamp(), imu_get_status() */
#include "../subsystem4/imu_cfg.h"	/* IMU_REQUIRED */

LOCAL VC_CORE		s_vc;
LOCAL VC_STATUS		s_status;	/* this task only */
LOCAL volatile UW	s_status_w[3];	/* s_status packed (VC_ST_PK): the copy telemetry reads */
LOCAL UB		s_seq_tel;		/* 8-bit APP_MSG sequence, wraps */
LOCAL UW		s_hold_drops;	/* speed holds lost to a full motion queue */

LOCAL const VC_PARAMS s_params = {
	VC_BASE_SPEED,		/* base_speed */
	VC_SLOW_SPEED,		/* slow_speed */
	VC_HUMP_SPEED,		/* hump_speed */
	VC_JUNCTION_SPEED,	/* junction_speed */
	VC_MOVE_SPEED,		/* move_speed */
	VC_FIND_SPEED,		/* find_speed */
	VC_CAR_WIDTH_MM,	/* car_width_mm */
	VC_MARGIN_MM,		/* margin_mm */
	VC_US_TO_AXLE_MM,	/* us_to_axle_mm */
	VC_LINE_TO_AXLE_MM,	/* line_to_axle_mm */
	VC_REAR_CLEAR_MM,	/* rear_clear_mm */
	VC_OBST_DEPTH_MM,	/* obst_depth_mm */
	VC_LINE_HALF_MM,	/* line_half_mm */
	VC_STOP_MM,		/* stop_mm */
	VC_BYPASS_STOP_MM,	/* bypass_stop_mm */
	VC_BYPASS_DEG,		/* bypass_deg */
	VC_BACK_MM,		/* back_mm */
	VC_N_RETRY,		/* n_retry */
	VC_TIE_MM,		/* tie_mm */
	VC_PREF_SIDE,		/* pref_side */
	VC_FIND_EXTRA_MM,	/* find_extra_mm */
	VC_ALIGN_EXTRA_MM,	/* align_extra_mm */
	VC_D_EXEC_MM,		/* d_exec_mm */
	VC_JUNCTION_MUTE_MM,	/* junction_mute_mm */
	VC_CROSS_MM,		/* cross_mm */
	VC_ACQUIRE_MM,		/* acquire_mm */
	VC_T_STARTUP_MS,	/* t_startup_ms */
	VC_T_ACK_MS,		/* t_ack_ms */
	VC_T_SCAN_MS,		/* t_scan_ms */
	MC_TRACK_MM,		/* track_mm */
	MC_TURN_SPEED,		/* turn_speed */
	MC_ACCEL,		/* min_decel */
	VC_STEP_MARGIN_MS,	/* step_margin_ms */
	VC_HUMP_GRACE_MM,	/* hump_grace_mm */
	VC_HOLD_REFRESH_MS,	/* hold_refresh_ms */
	VC_SEARCH_STEP_DEG,	/* search_step_deg */
	VC_SEARCH_MAX_DEG	/* search_max_deg */
};

EXPORT void vc_get_status(VC_STATUS *out)
{
	VC_ST_PK pk;

	pk.w[0] = s_status_w[0];		/* one load per word */
	pk.w[1] = s_status_w[1];
	pk.w[2] = s_status_w[2];
	vc_st_unpack(&pk, out);
}

LOCAL void publish_status(void)
{
	VC_ST_PK pk;

	vc_st_pack(&s_status, &pk);
	s_status_w[0] = pk.w[0];		/* one store per word */
	s_status_w[1] = pk.w[1];
	s_status_w[2] = pk.w[2];
}

/* A queue that refuses a command means the system is overloaded: Safety
   Monitor stops the car (design: "a full queue sets a fault"). */
LOCAL void check(ER er)
{
	if (er != E_OK) {
		(void)tk_set_flg(fault_flags, FLT_QUEUE_OVF);
	}
}

LOCAL void run_actions(const VC_ACTIONS *act)
{
	UW i;

	for (i = 0U; i < act->n; i++) {
		const VC_ACTION *a = &act->a[i];

		switch (a->type) {
		case VA_HOLD:
			if (motion_hold_speed(a->a, a->b, (UW)a->c) != E_OK) {
				s_hold_drops++;		/* re-sent within VC_HOLD_REFRESH_MS */
			}
			break;
		case VA_MOVE:
			check((a->a >= 0) ? motion_move_forward(a->a, a->b, a->c) :
					    motion_move_backward(-a->a, a->b, a->c));
			break;
		case VA_TURN:
			check((a->a >= 0) ? motion_turn_left(a->a, a->c) :
					    motion_turn_right(-a->a, a->c));
			break;
		case VA_STOP:
			(void)motion_stop();
			break;
		case VA_CLEAR_STOP:
			(void)motion_clear_stop();
			break;
		case VA_SCAN:
			check(scan_request((UW)a->a, a->b, a->c));
			break;
		case VA_FAULT:
			(void)tk_set_flg(fault_flags, (UINT)a->a);
			break;
		case VA_RESTART:
			(void)tk_set_flg(fault_flags, FLT_RESTART_REQ);
			break;
		case VA_TEL:
			/* telemetry is best effort: a full queue drops the record */
			(void)app_send(telemetry_q, (UB)SRC_VC, &s_seq_tel, (UB)a->a, a->b, a->c);
			if (a->a == (W)EV_STATE) {
				tm_printf((UB*)"[VC] -> %s", (const UB*)vc_state_name(a->b));
				if (a->c != VC_WHY_NONE) {
					tm_printf((UB*)" (reason %d)", a->c);
				}
				tm_printf((UB*)"\n");
			} else if (a->a == (W)EV_NAV) {
				tm_printf((UB*)"[VC] nav %d (%d)\n", a->b, a->c);
			} else {
				/* forwarded sensor event: in the telemetry only */
			}
			break;
		default:
			break;
		}
	}
}

LOCAL void update_status(void)
{
	s_status.state = (W)s_vc.state;
	s_status.why = s_vc.why;
	s_status.pending = s_vc.pending;
	s_status.commands_done = s_vc.commands_done;
	s_status.obstacles_passed = s_vc.obstacles_passed;
	s_status.run_start_ms = s_vc.run_start_ms;
	s_status.run_start_odo = s_vc.run_start_odo;
	publish_status();
}

/* Events still waiting in the inbox, plus the one just taken */
LOCAL void log_backlog(void)
{
	T_RMBF	r;

	if (tk_ref_mbf(vc_event_q, &r) == E_OK) {
		UW waiting = VC_EVENT_SLOTS - ((UW)r.frbufsz / (UW)MBF_BYTES(1U, sizeof(APP_MSG))) + 1U;

		if (waiting > s_status.vq_high) {
			s_status.vq_high = waiting;
		}
	}
}

LOCAL BOOL sensors_ready(void)
{
	IMU_STATUS imu;

	imu_get_status(&imu);
	return (imu.calibrated || (IMU_REQUIRED == 0)) ? TRUE : FALSE;
}

EXPORT void vc_task(INT stacd, void *exinf)
{
	APP_MSG		m;		/* packed, as it comes out of vc_event_q */
	APP_EVT		e;
	VC_ACTIONS	act;
	OS_PROFILE	prof;
	W		odo_mm = 0;
	UW		next_tick;
	INT		len;

	(void)stacd;
	(void)exinf;

	vc_init(&s_vc, &s_params, &lb_steer_params, app_ms());
	next_tick = app_ms() + (UW)VC_TICK_MS;
	tm_printf((UB*)"[VC] STARTUP: waiting for the sensors, then READY (press START)\n");

	while (1) {
		W	wait = (W)(next_tick - app_ms());
		UW	now;

		UW	t0;

		len = tk_rcv_mbf(vc_event_q, &m, (wait > 0) ? (TMO)wait : TMO_POL);
		t0 = app_us();
		now = app_ms();
		(void)motion_get_odo(&odo_mm);	/* on failure: keep the last one */

		if (len == (INT)sizeof(m)) {
			log_backlog();
			app_msg_unpack(&m, &e);
			if (e.type == EV_PROFILE) {
				if (scan_profile_get(&prof) != E_OK) {
					prof.valid = false;	/* lock not taken: treat as no profile */
				}
				vc_profile(&s_vc, &e, &prof, now, odo_mm, &act);
			} else {
				if (((e.type == EV_COMMAND) && (e.p1 == CMD_START)) ||
				    ((e.type == EV_BUTTON) && (e.p1 == BTN_START))) {
					/* SET values are copied at START */
					vc_tune(&s_vc, param_get(PARAM_BASE_SPEED), param_get(PARAM_STOP_MM));
				}
				vc_event(&s_vc, &e, now, odo_mm, &act);
			}
			run_actions(&act);
		}

		if ((W)(now - next_tick) >= 0) {
			next_tick += (UW)VC_TICK_MS;
			if ((W)(now - next_tick) >= 0) {
				next_tick = now + (UW)VC_TICK_MS;	/* far behind: do not catch up */
				app_task_overrun(TIM_VC, 1U);
			}
			vc_tick(&s_vc, now, odo_mm, (sensors_ready() != FALSE), &act);
			run_actions(&act);
			health_stamp(HEALTH_VC);
		}
		update_status();
		app_task_time(TIM_VC, t0);
	}
}
