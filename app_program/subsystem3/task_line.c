/*
 *	task_line.c
 *	The Line and Barcode task (Figure 5 of the design review), core 1,
 *	priority 8, released every 5 ms.  The logic is in line_core.c
 *	(Region A) and barcode_core.c / code39.c (Region B); this file is the
 *	RTOS side.
 *	  Region A: sample both line sensors -> LINE_STATUS on change,
 *	            JUNCTION, LINE_LOST
 *	  Region B: move the edges the GP27 interrupt stored in edge_buf into
 *	            the capture, and decode it once the sensor has been quiet
 *	            for T_end.
 */

#include "subsystem3.h"
#include "line_cfg.h"
#include "drv_ir.h"
#include "drv_barcode.h"
#include "../subsystem2/subsystem2.h"	/* motion_get_odo(): odometer */
#include "../subsystem4/subsystem4.h"	/* fault_flags, health_stamp() */
#include "../subsystem5/subsystem5.h"	/* vc_event_q */

#define LB_CAL_SAMPLES	32U

LOCAL LC_CORE		s_lc;
LOCAL BC_CORE		s_bc;
LOCAL UB		s_seq;		/* 8-bit APP_MSG sequence, wraps */
LOCAL LB_STATUS		s_status = {false, false, '-', 0U, 0U, 0U};	/* this task only */
LOCAL volatile UW	s_status_w;	/* s_status packed: the copy telemetry reads */
LOCAL BOOL		s_resend;	/* a LINE_STATUS was dropped: send it again */

LOCAL const LC_PARAMS s_lc_params = {
	LB_JUNCTION_MS,		/* junction_ms */
	LB_JUNCTION_MM,		/* junction_mm */
	LB_LOST_MM		/* lost_mm */
};

LOCAL const BC_PARAMS s_bc_params = {
	LB_T_MIN_US,			/* t_min_us */
	LB_T_END_MIN_MS * 1000U,	/* t_end_min_us */
	LB_T_END_FACTOR,		/* t_end_factor */
	(UB)LB_BAR_LEVEL		/* bar_level */
};

EXPORT void line_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_line);
}

EXPORT void lb_get_status(LB_STATUS *out)
{
	lc_status_unpack(s_status_w, out);	/* one 32-bit load; one writer (this task) */
}

/* Queue-overflow policy (evt_is_safety()): none of this task's events is a
   safety event, so a full vc_event_q drops and counts them.  A dropped
   LINE_STATUS is sent again on the next 5 ms cycle, so Vehicle Controller
   never steers on a stale reading for longer than that. */
LOCAL void send_events(const EVT_LIST *ev)
{
	UW	dropped = 0U;
	UW	i;

	if (app_send_events(vc_event_q, (UB)SRC_LINE, &s_seq, ev, &dropped) != E_OK) {
		(void)tk_set_flg(fault_flags, FLT_QUEUE_OVF);
	}
	for (i = 0U; i < ev->n; i++) {
		if ((ev->ev[i].type == (UB)EV_LINE_STATUS) && ((dropped & (1UL << i)) != 0U)) {
			s_resend = TRUE;
		}
	}
}

LOCAL BOOL has_line_status(const EVT_LIST *ev)
{
	UW i;

	for (i = 0U; i < ev->n; i++) {
		if (ev->ev[i].type == (UB)EV_LINE_STATUS) {
			return TRUE;
		}
	}
	return FALSE;
}

/* Startup checks: report the barcode sensor's analogue level and the
   line sensors' digital levels.  Place the car centred on a straight
   line before power-up: both line sensors should then read white. */
LOCAL void calibrate(void)
{
	UH	mean, lo, hi;
	BOOL	l, r;
	ER	er = ir_barcode_ao(LB_CAL_SAMPLES, &mean, &lo, &hi);

	if (er == E_OK) {
		tm_printf((UB*)"[LB] barcode AO: mean %u, range %u..%u (0..4095)\n", mean, lo, hi);
	} else {
		tm_printf((UB*)"[LB] barcode AO not readable: %d\n", er);
	}
	ir_read_line(&l, &r);
	tm_printf((UB*)"[LB] line sensors at start: left %s, right %s%s\n",
		  l ? (UB*)"BLACK" : (UB*)"white", r ? (UB*)"BLACK" : (UB*)"white",
		  (l || r) ? (UB*)"  (expected both white when centred)" : (UB*)"");
}

LOCAL void barcode_step(UW now_us)
{
	UW		e;
	BC_RESULT	res;
	EVT_LIST	ev;

	while (barcode_pop(&e)) {
		bc_edge(&s_bc, e & BC_EDGE_TIME, ((e & BC_EDGE_LEVEL) != 0U) ? 1U : 0U);
	}
	if (bc_poll(&s_bc, now_us, &res)) {
		evt_list_clear(&ev);
		if (res.result == C39_OK) {
			(void)evt_list_add(&ev, (UB)EV_BARCODE, (W)res.c, bc_command_for(res.c));
			s_status.last_char = res.c;
			tm_printf((UB*)"[LB] barcode '%c'%s, narrow %u us\n", (INT)res.c,
				  res.reversed ? (UB*)" (read backwards)" : (UB*)"", res.narrow_us);
		} else {
			(void)evt_list_add(&ev, (UB)EV_BARCODE_BAD, (W)res.result, (W)res.elements);
			tm_printf((UB*)"[LB] barcode not decoded: reason %d, %u elements (need 29)\n",
				  (INT)res.result, res.elements);
		}
		s_status.reads_ok = s_bc.ok;
		s_status.reads_failed = s_bc.failed;
		send_events(&ev);
	}
}

EXPORT void line_task(INT stacd, void *exinf)
{
	EVT_LIST	ev;
	W		odo_mm = 0;
	UW		overflows = 0U;
	BOOL		l, r;

	(void)stacd;
	(void)exinf;

	(void)ir_init();
	if (barcode_init() != E_OK) {
		tm_printf((UB*)"[LB] barcode interrupt set-up failed\n");
		(void)tk_set_flg(fault_flags, FLT_LINE);
	}
	calibrate();
	(void)motion_get_odo(&odo_mm);	/* the T_lost distance starts here */
	lc_init(&s_lc, &s_lc_params, odo_mm);
	bc_init(&s_bc, &s_bc_params);

	s_status_w = lc_status_pack(&s_status);
	while (1) {
		UW	t0;
		INT	missed;

		(void)tk_slp_tsk(TMO_FEVR);
		t0 = app_us();
		missed = tk_can_wup(TSK_SELF);	/* never "catch up" after a delay */
		if (missed > 0) {
			app_task_overrun(TIM_LINE, (UW)missed);
		}

		/* Region A */
		ir_read_line(&l, &r);
		(void)motion_get_odo(&odo_mm);	/* on failure: keep the last one */
		evt_list_clear(&ev);
		lc_step(&s_lc, (l != FALSE), (r != FALSE), app_ms(), odo_mm, &ev);
		if (s_resend && !has_line_status(&ev)) {
			(void)evt_list_add(&ev, (UB)EV_LINE_STATUS, (l != FALSE) ? 1 : 0, (r != FALSE) ? 1 : 0);
		}
		s_resend = FALSE;
		s_status.left_black = (l != FALSE);
		s_status.right_black = (r != FALSE);
		s_status.junctions = s_lc.junctions;
		send_events(&ev);

		/* Region B */
		barcode_step(app_us());
		if (barcode_overflows() != overflows) {
			/* Edges were lost: this capture cannot be decoded.  Abandon
			   it and report it; the car carries on (a missed barcode is
			   a mission problem, not a safety fault). */
			overflows = barcode_overflows();
			bc_reset(&s_bc);
			s_bc.failed++;
			s_status.reads_failed = s_bc.failed;
			evt_list_clear(&ev);
			(void)evt_list_add(&ev, (UB)EV_BARCODE_BAD, BC_REASON_OVERFLOW, 0);
			send_events(&ev);
			tm_printf((UB*)"[LB] edge_buf overflow: capture abandoned\n");
		}

		s_status_w = lc_status_pack(&s_status);	/* one 32-bit store */
		health_stamp(HEALTH_LINE);
		app_task_time(TIM_LINE, t0);
	}
}
