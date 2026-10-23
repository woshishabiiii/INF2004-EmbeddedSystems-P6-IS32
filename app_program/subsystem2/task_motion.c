/*
 *	task_motion.c
 *	The Motion Control task (Figure 6 of the design review), core 1,
 *	priority 6.  The control logic itself is in mc_core.c; this file is
 *	the RTOS side: it reads the hardware and the kernel objects, calls
 *	mc_core_step() and writes the results back out.
 *
 *	Wake-ups:
 *	  - every 20 ms from the cyclic handler (s_tick = TRUE): full control step;
 *	  - at once from motion_stop() / motion_fault_stop(): stop bits, so a
 *	    stop is acted on within microseconds, not at the next tick;
 *	  - at once from motion_hold_speed(): the new wheel speeds are applied
 *	    straight away (steering does not wait for the tick).
 */

#include "subsystem2.h"
#include "motion_cfg.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../gpio_irq.h"
#include "../subsystem1/subsystem1.h"	/* param_get(): SET values */
#include "../subsystem4/subsystem4.h"	/* fault_flags, health_stamp() */
#include "../subsystem5/subsystem5.h"	/* vc_event_q */

IMPORT volatile UW	mc_stop_req_us;	/* motion_api.c */

LOCAL volatile UW	s_ticks;	/* counted by the cyclic handler (one writer) */
LOCAL MC_CORE		s_core;
LOCAL MC_SNAP_PK	s_snap;		/* dist_snapshot, packed, guarded by mtx_snapshot */
LOCAL MC_ENC		s_enc;		/* 32-bit positions from the 16-bit encoder word */
LOCAL UB		s_seq;		/* 8-bit APP_MSG sequence, wraps */

LOCAL const MC_PARAMS s_params = {
	MC_COUNTS_PER_M,	/* counts_per_m */
	MC_COUNTS_PER_90DEG,	/* counts_per_90deg */
	MC_TRACK_MM,		/* track_mm */
	MC_KFF_X1000,		/* kff_x1000 */
	MC_DEADBAND,		/* deadband */
	MC_KP_X1000,		/* kp_x1000 */
	MC_KI_X1000,		/* ki_x1000 */
	MC_MAX_SPEED,		/* max_speed */
	MC_MIN_SPEED,		/* min_speed */
	MC_ACCEL,		/* accel */
	MC_TURN_SPEED,		/* turn_speed */
	MC_K_STRAIGHT,		/* k_straight */
	MC_SETTLE_SPEED,	/* settle_speed */
	MC_STALL_DUTY,		/* stall_duty */
	MC_STALL_SPEED,		/* stall_speed */
	MC_STALL_TICKS,		/* stall_ticks */
	MC_TIMEOUT_MARGIN_MS,	/* timeout_margin_ms */
	MC_SPEED_WINDOW,	/* speed_window */
	MC_DUTY_RISE,		/* duty_rise */
	MC_STOP_COAST,		/* stop_coast */
	MC_STEER_PM_MM		/* steer_pm_mm */
};

/* Own fault bits (3, mc_core.h) -> their bits in the shared fault bitmask */
LOCAL UINT fault_flag_for(UW mc_bits)
{
	UINT flags = 0U;

	if ((mc_bits & MC_FLT_STALL) != 0U) {
		flags |= FLT_STALL;
	}
	if ((mc_bits & MC_FLT_TIMEOUT) != 0U) {
		flags |= FLT_MOVE_TIMEOUT;
	}
	if ((mc_bits & MC_FLT_ENCODER) != 0U) {
		flags |= FLT_ENCODER;
	}
	return flags;
}

EXPORT void motion_cyclic_handler(void *exinf)
{
	(void)exinf;
	s_ticks++;
	(void)tk_wup_tsk(tid_motion);
}

EXPORT ER motion_get_snapshot(MC_SNAPSHOT *out)
{
	MC_SNAP_PK	copy;
	ER		er = tk_loc_mtx(mtx_snapshot, 2);

	if (er == E_OK) {
		copy = s_snap;			/* 36 B, whole words, no read-modify-write */
		(void)tk_unl_mtx(mtx_snapshot);
		mc_snap_unpack(&copy, out);	/* outside the lock */
	}
	return er;
}

EXPORT ER motion_get_odo(W *odo_mm)
{
	MC_SNAP_PK	copy;
	ER		er = tk_loc_mtx(mtx_snapshot, 2);

	if (er == E_OK) {
		copy = s_snap;			/* 36 B, whole words */
		(void)tk_unl_mtx(mtx_snapshot);
		*odo_mm = (W)mc_snap_odo(&copy);	/* outside the lock */
	}
	return er;
}

/* SET values (MQTT, accepted only in READY) are copied when the car is
   released at START, never in the middle of a run.  A value that was
   never SET reads as PARAM_UNSET (-1), which mc_core_tune() ignores. */
LOCAL void load_params(void)
{
	mc_core_tune(&s_core, param_get(PARAM_KP_X1000), param_get(PARAM_KI_X1000),
		     param_get(PARAM_DIST_CAL), param_get(PARAM_TURN_CAL));
}

EXPORT void motion_task(INT stacd, void *exinf)
{
	MC_CMD	cmds[8];
	MC_CMD_PK pk;
	MC_IN	in;
	MC_OUT	out;
	BOOL	was_cmd_stop = TRUE;
	UW	seen_ticks = 0U;
	ER	er;

	(void)stacd;
	(void)exinf;

	/* Startup checks: motors braked first, then the shared GPIO
	   interrupt (this task is on core 1) and the encoders. */
	(void)motor_init();
	er = gpio_irq_start();
	if (er == E_OK) {
		er = encoder_init();
	}
	if (er != E_OK) {
		tm_printf((UB*)"[MC] encoder set-up failed: %d\n", er);
		(void)tk_set_flg(fault_flags, FLT_ENCODER);
	}
	mc_enc_init(&s_enc, encoder_word());
	in.count[MC_LEFT] = s_enc.pos[MC_LEFT];
	in.count[MC_RIGHT] = s_enc.pos[MC_RIGHT];
	mc_core_init(&s_core, &s_params, in.count);
	tm_printf((UB*)"[MC] ready: %d counts/m, %d counts per 90 deg\n",
		  MC_COUNTS_PER_M, MC_COUNTS_PER_90DEG);

	while (1) {
		T_RFLG	rflg;
		UINT	flags = 0U;

		UW	t0;
		UW	missed;

		(void)tk_slp_tsk(TMO_FEVR);
		t0 = app_us();

		/* Inputs: stop bits and new commands on EVERY wake-up */
		if (tk_ref_flg(stop_flags, &rflg) == E_OK) {
			flags = rflg.flgptn;
		}
		in.now_ms = app_ms();
		in.now_us = app_us();
		mc_enc_update(&s_enc, encoder_word());	/* one load: both wheels */
		in.count[MC_LEFT] = s_enc.pos[MC_LEFT];
		in.count[MC_RIGHT] = s_enc.pos[MC_RIGHT];
		in.cmd_stop = ((flags & MC_CMD_STOP) != 0U);
		in.fault_stop = ((flags & MC_FAULT_STOP) != 0U);
		in.stop_req_us = mc_stop_req_us;
		missed = s_ticks - seen_ticks;
		in.tick = (missed != 0U);		/* a 20 ms release (not just a wake-up) */
		if (missed > 1U) {
			app_task_overrun(TIM_MOTION, missed - 1U);	/* a tick came while busy */
		}
		seen_ticks = s_ticks;
		in.ncmd = 0U;
		in.cmd = cmds;
		while ((in.ncmd < 8U) &&
		       (tk_rcv_mbf(motion_cmd_q, &pk, TMO_POL) == (INT)sizeof(MC_CMD_PK))) {
			mc_cmd_unpack(&pk, &cmds[in.ncmd]);
			in.ncmd++;
		}
		if (was_cmd_stop && !in.cmd_stop && !in.fault_stop) {
			load_params();			/* released at START */
		}
		was_cmd_stop = in.cmd_stop || in.fault_stop;

		mc_core_step(&s_core, &in, &out);

		/* Outputs: motors first */
		if (out.coast) {
			motor_coast(MOTOR_LEFT);	/* CMD_STOP: roll to a stop */
			motor_coast(MOTOR_RIGHT);
		} else {
			motor_set(MOTOR_LEFT, out.duty[MC_LEFT]);
			motor_set(MOTOR_RIGHT, out.duty[MC_RIGHT]);
		}

		if (out.flush) {
			while (tk_rcv_mbf(motion_cmd_q, &pk, TMO_POL) > 0) {
				/* stale commands are never run after a stop */
			}
		}
		if ((out.stop_latency_us != 0U) && (in.stop_req_us != 0U)) {
			/* the 6-bit skipped-edge counts too (INT-21: they stay 0) */
			tm_printf((UB*)"[MC] %s: duty 0 after %u us, encoder errors %u/%u\n",
				  in.fault_stop ? (UB*)"FAULT_STOP" : (UB*)"CMD_STOP",
				  out.stop_latency_us, encoder_errors(ENC_LEFT), encoder_errors(ENC_RIGHT));
		}
		if (out.fault) {
			(void)motion_fault_stop();
			(void)tk_set_flg(fault_flags, fault_flag_for(s_core.snap.fault_bits));
			tm_printf((UB*)"[MC] FAULT 0x%x (1 stall, 2 timeout, 4 encoder direction)\n",
				  s_core.snap.fault_bits);
			if ((s_core.snap.fault_bits & MC_FLT_ENCODER) != 0U) {
				/* say which wheel, and what to change */
				tm_printf((UB*)"[MC] driven forward, measured: left %d mm/s, right %d mm/s\n",
					  s_core.w[MC_LEFT].speed, s_core.w[MC_RIGHT].speed);
				tm_printf((UB*)"[MC] a NEGATIVE wheel: if it really rolls forward, flip its "
					  "ENC_SIGN_ in drv_encoder.c; if it rolls backward, flip its "
					  "MOTOR_DIR_ in drv_motor.c\n");
			}
		}
		/* EV_STOP_ACK must arrive; a lost EV_DONE is covered by Vehicle
		   Controller's time limits (evt_is_safety()) */
		if (app_send_events(vc_event_q, (UB)SRC_MOTION, &s_seq, &out.ev, NULL) != E_OK) {
			(void)tk_set_flg(fault_flags, FLT_QUEUE_OVF);
		}

		if (in.tick) {
			MC_SNAP_PK packed;

			mc_snap_pack(&s_core.snap, &packed);	/* outside the lock */
			if (tk_loc_mtx(mtx_snapshot, 2) == E_OK) {
				s_snap = packed;
				(void)tk_unl_mtx(mtx_snapshot);
			}
			health_stamp(HEALTH_MOTION);
		}
		app_task_time(TIM_MOTION, t0);
	}
}
