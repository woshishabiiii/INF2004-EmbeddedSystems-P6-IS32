/*
 *	task_scan.c
 *	Obstacle Scan task (Figure 7), core 0, priority 10, released every
 *	10 ms.  The logic is in scan_core.c and os_profile.c; this file is
 *	the RTOS side.  Nothing here waits: servo settling, the 70 ms trigger
 *	gap and the echo are all checked again on the next 10 ms step.  The
 *	only busy-wait is the 10 us trigger pulse in drv_hcsr04.c.
 *
 *	On a confirmed close range during a WATCH request with SCAN_STOP_FLAG,
 *	this task sets CMD_STOP itself (motion_stop) BEFORE it sends the ALERT,
 *	so the stop does not depend on vc_event_q or on Vehicle Controller
 *	(design-review feedback IS32).  Vehicle Controller still decides what
 *	happens next and is the only task that clears CMD_STOP.
 */

#include "subsystem5.h"
#include "scan_cfg.h"
#include "drv_hcsr04.h"
#include "drv_servo.h"
#include "../subsystem1/subsystem1.h"	/* telemetry_q */
#include "../subsystem2/subsystem2.h"	/* motion_stop() */
#include "../subsystem4/subsystem4.h"	/* fault_flags, health_stamp() */

LOCAL SCAN_CORE		s_scan;
LOCAL OS_PROFILE	s_profile;	/* copy for Vehicle Controller and telemetry */
LOCAL SCAN_STATUS	s_status = {0, -1, 0U};
LOCAL UH		s_seq_vc;
LOCAL UH		s_seq_tel;

LOCAL const SCAN_PARAMS s_params = {
	OS_STOP_MM,		/* stop_mm */
	OS_SLOW_MM,		/* slow_mm */
	OS_SLOW_HYST_MM,	/* slow_hyst_mm */
	OS_CONFIRM,		/* confirm */
	OS_GAP_MS,		/* gap_ms */
	OS_SETTLE_BASE_MS,	/* settle_base_ms */
	OS_SETTLE_MS_PER_DEG,	/* settle_ms_per_deg */
	OS_RETRIES,		/* retries */
	OS_R_MAX_MM,		/* r_max_mm */
	OS_CORRIDOR_MM,		/* corridor_mm */
	OS_COARSE_STEP,		/* coarse_step */
	OS_FINE_STEP,		/* fine_step */
	OS_FINE_MARGIN,		/* fine_margin */
	OS_MISSING_LIMIT,	/* missing_limit */
	OS_PROFILE_MAX_MS	/* profile_max_ms */
};

EXPORT void scan_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_scan);
}

/* Written by this task BEFORE it sends EV_PROFILE; read by Vehicle
   Controller and Telemetry AFTER they receive it.  The next write comes
   only after Vehicle Controller asks for another profile. */
EXPORT void scan_profile_get(OS_PROFILE *out)
{
	*out = s_profile;
}

EXPORT void scan_get_status(SCAN_STATUS *out)
{
	*out = s_status;
}

EXPORT ER scan_request(UW mode, W stop_mm, W id)
{
	SCAN_REQ req;

	req.mode = mode;
	req.stop_mm = stop_mm;
	req.id = id;
	return tk_snd_mbf(scan_request_q, &req, (INT)sizeof(req), TMO_POL);
}

LOCAL INT echo_code(INT hc)
{
	switch (hc) {
	case HC_DONE:
		return SCAN_ECHO_DONE;
	case HC_NO_ECHO:
		return SCAN_ECHO_FAR;
	case HC_NO_RESPONSE:
		return SCAN_ECHO_MISSING;
	default:
		return SCAN_ECHO_NONE;		/* idle or still waiting */
	}
}

EXPORT void scan_task(INT stacd, void *exinf)
{
	SCAN_IN		in = {0};
	SCAN_OUT	out;
	SCAN_REQ	req = {0};
	W		servo_now = -1;
	BOOL		refused = FALSE;
	W		mm = 0;

	(void)stacd;
	(void)exinf;

	(void)servo_init();			/* centred, facing forward */
	if (hcsr04_init() != E_OK) {
		tm_printf((UB*)"[OS] ultrasonic set-up failed\n");
	}
	scan_core_init(&s_scan, &s_params);

	while (1) {
		UW	t0;
		INT	missed;

		(void)tk_slp_tsk(TMO_FEVR);
		t0 = app_us();
		missed = tk_can_wup(TSK_SELF);	/* never "catch up" after a delay */
		if (missed > 0) {
			app_task_overrun(TIM_SCAN, (UW)missed);
		}
		health_stamp(HEALTH_SCAN);

		/* Inputs: the newest request, the ping result */
		in.now_ms = app_ms();
		in.have_req = FALSE;
		while (tk_rcv_mbf(scan_request_q, &req, TMO_POL) == (INT)sizeof(req)) {
			in.req = req;			/* only the newest one counts */
			in.have_req = TRUE;
		}
		in.echo = refused ? SCAN_ECHO_MISSING : echo_code(hcsr04_poll(&mm));
		in.range_mm = mm;
		in.echo_high = hcsr04_echo_high();
		refused = FALSE;

		scan_core_step(&s_scan, &in, &out);

		/* Outputs: a confirmed obstacle stops the car first */
		if (out.stop_motors) {
			(void)motion_stop();
			tm_printf((UB*)"[OS] obstacle at %d mm: CMD_STOP set\n", s_scan.last_range);
		}
		if (out.servo_deg != servo_now) {
			servo_now = out.servo_deg;
			servo_set_deg(servo_now);
		}
		if (out.trigger && (hcsr04_trigger() != E_OK)) {
			refused = TRUE;			/* reported as "no response" next step */
		}
		if (out.fault && (OS_REQUIRED != 0)) {
			(void)tk_set_flg(fault_flags, FLT_SCAN);
		}
		if (out.profile_ready) {
			s_profile = s_scan.profile;	/* BEFORE EV_PROFILE is sent */
			(void)app_send(telemetry_q, (UB)SRC_SCAN, &s_seq_tel, (UB)EV_PROFILE_DATA, 0, 0);
			tm_printf((UB*)"[OS] profile: valid %d, x %d..%d mm, closest %d mm at %d deg, "
				  "clear L %d R %d, unknown L %u R %u\n",
				  (INT)s_profile.valid, s_profile.x_left_mm, s_profile.x_right_mm,
				  s_profile.closest_mm, s_profile.closest_deg, s_profile.left_clear_mm,
				  s_profile.right_clear_mm, s_profile.unknown_left, s_profile.unknown_right);
		}
		/* EV_ALERT must arrive; range zones and profiles are dropped and
		   counted if the inbox is full (evt_is_safety()) */
		if (app_send_events(vc_event_q, (UB)SRC_SCAN, &s_seq_vc, &out.ev, NULL) != E_OK) {
			(void)tk_set_flg(fault_flags, FLT_QUEUE_OVF);
		}

		s_status.mode = (W)(s_scan.req.mode & SCAN_MODE_MASK);
		s_status.range_mm = s_scan.last_range;
		s_status.pings = s_scan.pings;
		app_task_time(TIM_SCAN, t0);
	}
}
