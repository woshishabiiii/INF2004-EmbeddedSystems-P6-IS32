/*
 *	vc_cfg.h
 *	Vehicle Controller tuning values (Buddy 5), each with its unit.
 *	MEASURE = measure on your car;  TUNE = adjust on the track.
 *	Pure #defines, so the host simulator can include it too.
 */

#ifndef VC_CFG_H
#define VC_CFG_H

#define VC_TICK_MS		20	/* timers and steering refresh */

/* ---- speeds, mm/s ------------------------------------------------------- */
#define VC_BASE_SPEED		200	/* line following (SET base_speed)      TUNE */
#define VC_SLOW_SPEED		120	/* something ahead within the slow zone  TUNE */
#define VC_HUMP_SPEED		150	/* on a hump: straight, encoder heading  TUNE */
#define VC_JUNCTION_SPEED	150	/* barcode command waiting for its junction */
#define VC_MOVE_SPEED		150	/* distance moves (junctions, bypass legs) */
#define VC_FIND_SPEED		120	/* creeping toward the line after a bypass */

/* ---- geometry ----------------------------------------------------------- */
#define VC_CAR_WIDTH_MM		160	/* widest point of the car               MEASURE */
#define VC_MARGIN_MM		50	/* extra room kept from an obstacle      TUNE */
#define VC_US_TO_AXLE_MM	80	/* ultrasonic sensor ahead of the wheel axle  MEASURE */
#define VC_LINE_TO_AXLE_MM	60	/* line sensors ahead of the wheel axle  MEASURE */
#define VC_REAR_CLEAR_MM	120	/* axle to the back of the car, plus margin  MEASURE */
#define VC_OBST_DEPTH_MM	150	/* assumed obstacle depth (sonar only sees the front)  MEASURE */
#define VC_LINE_HALF_MM		9	/* half the 18 mm line */

/* ---- obstacles ------------------------------------------------------------ */
#define VC_STOP_MM		250	/* ALERT distance while following (SET stop_mm)  TUNE */
#define VC_BYPASS_STOP_MM	120	/* ALERT distance during a bypass         TUNE */
#define VC_BYPASS_DEG		45	/* angle of the sideways legs */
#define VC_BACK_MM		150	/* reverse this far before scanning again */
#define VC_N_RETRY		2	/* reverse-and-retry attempts */
#define VC_TIE_MM		20	/* detours within this of each other count as equal */
#define VC_PREF_SIDE		(-1)	/* -1 pass on the left, +1 right, when equal */
#define VC_FIND_EXTRA_MM	150	/* search this much past where the line should be */
#define VC_ALIGN_EXTRA_MM	10	/* drive on after touching the line, then turn */

/* ---- barcodes and junctions ---------------------------------------------- */
#define VC_D_EXEC_MM		1000	/* no junction within this: run the command here (0 = never)  TUNE */
#define VC_JUNCTION_MUTE_MM	80	/* ignore junctions this soon after one */
#define VC_CROSS_MM		40	/* "go straight": drive across the junction */
#define VC_ACQUIRE_MM		60	/* after a turn or bypass: no line by then = search  TUNE */

/* ---- time limits, ms ------------------------------------------------------ */
#define VC_T_STARTUP_MS		3000	/* sensors must be ready by then */
#define VC_T_ACK_MS		1500	/* stop -> STOP_ACK */
#define VC_T_SCAN_MS		10000	/* request -> PROFILE */
#define VC_T_SEARCH_MS		8000	/* whole line search */
#define VC_T_EXEC_MS		6000	/* whole barcode command */
#define VC_T_BYPASS_MS		20000	/* bypass and line recovery */
#define VC_HUMP_GRACE_MS	500	/* after a hump, before line loss counts again */
#define VC_HOLD_REFRESH_MS	200	/* re-send the wheel speeds at least this often */

#endif /* VC_CFG_H */
