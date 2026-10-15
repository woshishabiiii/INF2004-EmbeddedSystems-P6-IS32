/*
 *	task_imu.c
 *	IMU and Terrain task, core 0, priority 11, released every 10 ms.
 *	The logic is in imu_core.c; this file is the RTOS side: one 6-byte
 *	accelerometer burst read per period, Motion Control's snapshot for
 *	distance and speed, then events out.
 *	  to Vehicle Controller: EV_TERRAIN, EV_HUMP, EV_IMPACT
 *	  to Telemetry:          EV_TERRAIN, EV_HUMP, EV_MOTION, EV_IMPACT
 */

#include "subsystem4.h"
#include "imu_cfg.h"
#include "imu_math.h"
#include "drv_lsm303.h"
#include "../subsystem1/subsystem1.h"	/* telemetry_q */
#include "../subsystem2/subsystem2.h"	/* motion_get_snapshot() */
#include "../subsystem5/subsystem5.h"	/* vc_event_q */

LOCAL IMU_CORE		s_imu;
LOCAL IMU_STATUS	s_status;
LOCAL UH		s_seq_vc;
LOCAL UH		s_seq_tel;
LOCAL BOOL		s_terrain_resend;	/* a terrain change was dropped: send it again */

LOCAL const IMU_PARAMS s_params = {
	IMU_AXIS_FWD,		/* axis_fwd */
	IMU_SIGN_FWD,		/* sign_fwd */
	IMU_AXIS_UP,		/* axis_up */
	IMU_SIGN_UP,		/* sign_up */
	IMU_AXIS_LAT,		/* axis_lat */
	IMU_SIGN_LAT,		/* sign_lat */
	IMU_CAL_SAMPLES,	/* cal_samples */
	IMU_LPF_SHIFT,		/* lpf_shift */
	IMU_CLIMB_CDEG,		/* climb_cdeg */
	IMU_LEVEL_CDEG,		/* level_cdeg */
	IMU_TERRAIN_MM,		/* terrain_mm */
	IMU_HUMP_END_MM,	/* hump_end_mm */
	IMU_HUMP_END_MS,	/* hump_end_ms */
	IMU_PRE_SAMPLES,	/* pre_samples */
	IMU_HUMP_MIN_MM,	/* hump_min_mm */
	IMU_WHEELBASE_MM,	/* wheelbase_mm */
	IMU_IMPACT_MG,		/* impact_mg */
	IMU_IMPACT_WINDOW_MS,	/* impact_window_ms */
	IMU_IMPACT_DROP_PCT,	/* impact_drop_pct */
	IMU_IMPACT_MIN_SPEED,	/* impact_min_speed */
	IMU_STILL_SPEED,	/* still_speed */
	IMU_STILL_DEV_MG,	/* still_dev_mg */
	IMU_ACCEL_MM_S2,	/* accel_mm_s2 */
	IMU_TURN_CDEG_S,	/* turn_cdeg_s */
	IMU_CLASS_HOLD,		/* class_hold */
	IMU_FAIL_LIMIT		/* fail_limit */
};

EXPORT void imu_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_imu);
}

EXPORT void imu_get_status(IMU_STATUS *out)
{
	*out = s_status;		/* single writer (this task) */
}

/* "-1.05" from centidegrees (tm_printf has no %f) */
LOCAL void print_cdeg(W cdeg)
{
	W mag = (cdeg < 0) ? -cdeg : cdeg;

	tm_printf((UB*)"%s%d.%02d", (cdeg < 0) ? (UB*)"-" : (UB*)"", mag / 100, mag % 100);
}

LOCAL void route_events(const EVT_LIST *ev)
{
	UW i;

	for (i = 0U; i < ev->n; i++) {
		const APP_MSG *m = &ev->ev[i];

		/* Vehicle Controller needs the terrain (slow down, ignore the
		   line on a hump) and collisions, but not the motion class */
		if ((m->type != EV_MOTION) && ((m->type != EV_IMPACT) || (IMU_IMPACT_STOPS != 0))) {
			BOOL dropped = FALSE;

			/* An impact must arrive (safety event); terrain and humps are
			   dropped and counted if the inbox is full, and a lost
			   terrain change is sent again next cycle (evt_is_safety()) */
			if (app_send_event(vc_event_q, (UB)SRC_IMU, &s_seq_vc, m->type, m->p1, m->p2,
					   &dropped) != E_OK) {
				(void)tk_set_flg(fault_flags, FLT_QUEUE_OVF);
			}
			if (dropped && (m->type == (UB)EV_TERRAIN)) {
				s_terrain_resend = TRUE;
			}
		}
		/* telemetry is best effort: a full queue just drops the record */
		(void)app_send(telemetry_q, (UB)SRC_IMU, &s_seq_tel, m->type, m->p1, m->p2);

		if (m->type == EV_HUMP) {
			tm_printf((UB*)"[IMU] hump %d mm (highest %d mm)\n", m->p1, m->p2);
		} else if (m->type == EV_IMPACT) {
			tm_printf((UB*)"[IMU] IMPACT: jolt %d mg at %d mm/s\n", m->p1, m->p2);
		} else {
			/* terrain and motion changes are in the telemetry */
		}
	}
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	IMU_OUT		out;
	IMU_MOTION	mot;
	MC_SNAPSHOT	snap = {0};
	W		a[3] = {0, 0, 0};
	W		mx = 0, my = 0, mz = 0;
	BOOL		mag_ok = FALSE;
	BOOL		was_cal = FALSE;
	UW		n = 0U;
	ER		er;

	(void)stacd;
	(void)exinf;

	/* Startup checks: identity register, configuration read-back */
	er = lsm_open();
	if (er == E_OK) {
		er = lsm_accel_init();
	}
	if (er == E_OK) {
		mag_ok = (lsm_mag_init() == E_OK) ? TRUE : FALSE;
		tm_printf((UB*)"[IMU] LSM303DLHC found; keep the car still and level for 1 s\n");
	} else {
		tm_printf((UB*)"[IMU] set-up failed (%d): check wiring, pull-ups, I2C0 = GP4/GP5\n", er);
	}
	imu_core_init(&s_imu, &s_params);

	while (1) {
		BOOL	ok;
		UW	t0;
		INT	missed;

		(void)tk_slp_tsk(TMO_FEVR);
		t0 = app_us();
		missed = tk_can_wup(TSK_SELF);	/* never "catch up" after a delay */
		if (missed > 0) {
			app_task_overrun(TIM_IMU, (UW)missed);
		}
		health_stamp(HEALTH_IMU);

		ok = (lsm_accel_read(&a[0], &a[1], &a[2]) == E_OK) ? TRUE : FALSE;
		mot.valid = (motion_get_snapshot(&snap) == E_OK);
		mot.dist_mm = snap.dist_mm;
		mot.speed_mm_s = snap.speed_mm_s;
		mot.accel_mm_s2 = snap.accel_mm_s2;
		mot.yaw_cdeg_s = snap.yaw_cdeg_s;

		imu_core_step(&s_imu, a, (ok != FALSE), &mot, app_ms(), &out);

		if (out.fault && (IMU_REQUIRED != 0)) {
			(void)tk_set_flg(fault_flags, FLT_IMU);
		}
		if (out.impact && (IMU_IMPACT_STOPS != 0)) {
			(void)tk_set_flg(fault_flags, FLT_IMPACT);
		}
		if (s_terrain_resend) {
			BOOL dropped = FALSE;

			s_terrain_resend = FALSE;
			(void)app_send_event(vc_event_q, (UB)SRC_IMU, &s_seq_vc, (UB)EV_TERRAIN,
					     s_imu.terrain, s_imu.pitch_cdeg, &dropped);
			s_terrain_resend = dropped;
		}
		route_events(&out.ev);

		if (s_imu.calibrated && !was_cal) {
			was_cal = TRUE;
			tm_printf((UB*)"[IMU] calibrated: forward %d mg, up %d mg, |g| %d mg, mounting tilt ",
				  s_imu.f0, s_imu.u0, s_imu.g0);
			print_cdeg(s_imu.base_cdeg);
			tm_printf((UB*)" deg\n");
		}

		/* Magnetometer heading, 10 times a second, for cross-checking
		   the encoder turn rate (no calibration: relative changes only) */
		if (mag_ok && (++n >= IMU_MAG_EVERY)) {
			n = 0U;
			if (lsm_mag_read(&mx, &my, &mz) == E_OK) {
				s_status.heading_cdeg = imu_atan2_cdeg(my, mx);
			}
		}

		s_status.calibrated = s_imu.calibrated ? TRUE : FALSE;
		s_status.ok = ok;
		s_status.pitch_cdeg = s_imu.pitch_cdeg;
		s_status.terrain = s_imu.terrain;
		s_status.motion = s_imu.motion;
		s_status.last_hump_mm = s_imu.last_hump_mm;
		s_status.max_hump_mm = s_imu.max_hump_mm;
		s_status.humps = s_imu.humps;
		s_status.g0_mg = s_imu.g0;
		app_task_time(TIM_IMU, t0);
	}
}
