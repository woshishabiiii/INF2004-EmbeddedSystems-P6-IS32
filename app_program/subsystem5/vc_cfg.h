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
/* VC_BASE_SPEED is also the car's SPEED LIMIT: every other speed below is
   capped at it, so "SET base_speed 80" slows everything (moves, humps,
   junctions), not just line following. */
#define VC_BASE_SPEED		140	/* line following (SET base_speed)      TUNE */
#define VC_SLOW_SPEED		90	/* something ahead within the slow zone  TUNE */
#define VC_HUMP_SPEED		100	/* on a hump: straight, encoder heading  TUNE */
#define VC_JUNCTION_SPEED	100	/* barcode command waiting for its junction */
#define VC_MOVE_SPEED		100	/* distance moves (junctions, bypass legs) */
#define VC_FIND_SPEED		80	/* creeping toward the line after a bypass */

/* ---- geometry ----------------------------------------------------------- */
#define VC_CAR_WIDTH_MM		160	/* widest point of the car               MEASURE */
#define VC_MARGIN_MM		50	/* extra room kept from an obstacle      TUNE */
#define VC_US_TO_AXLE_MM	80	/* ultrasonic sensor ahead of the wheel axle  MEASURE */
#define VC_LINE_TO_AXLE_MM	60	/* line sensors ahead of the wheel axle  MEASURE */
#define VC_REAR_CLEAR_MM	120	/* axle to the back of the car, plus margin  MEASURE */
#define VC_OBST_DEPTH_MM	150	/* assumed obstacle depth (sonar only sees the front)  MEASURE */
#define VC_LINE_HALF_MM		9	/* half the 18 mm line */

/* ---- obstacles ------------------------------------------------------------ */
#define VC_STOP_MM		150	/* ALERT distance while following (SET stop_mm)  TUNE */
#define VC_BYPASS_STOP_MM	120	/* ALERT distance during a bypass         TUNE */
#define VC_BYPASS_DEG		45	/* angle of the sideways legs */
#define VC_BACK_MM		150	/* reverse this far before scanning again */
#define VC_N_RETRY		2	/* reverse-and-retry attempts */
#define VC_TIE_MM		20	/* detours within this of each other count as equal */
#define VC_PREF_SIDE		(-1)	/* -1 pass on the left, +1 right, when equal */
/* Line search: spin on the spot, sweeping ever wider either side of where
   the line was lost: +step, -step, +2 step, -2 step, ... up to +-max, so a
   nearby line is found quickly and +-180 covers the whole circle.  max must
   be a multiple of step and at most 180 (the last spin is 2 x max). */
#define VC_SEARCH_STEP_DEG	180	/* first sweep, and how much wider each one gets  TUNE */
#define VC_SEARCH_MAX_DEG	180	/* widest sweep either side (180 = full circle)   TUNE */
#define VC_FIND_EXTRA_MM	150	/* search this much past where the line should be */
#define VC_ALIGN_EXTRA_MM	10	/* drive on after touching the line, then turn */

/* ---- barcodes and junctions ---------------------------------------------- */
#define VC_D_EXEC_MM		1000	/* no junction within this: run the command here (0 = never)  TUNE */
#define VC_JUNCTION_MUTE_MM	80	/* ignore junctions this soon after one */
#define VC_CROSS_MM		40	/* "go straight": drive across the junction */
#define VC_ACQUIRE_MM		60	/* after a turn or bypass: no line by then = search  TUNE */

/* ---- time limits, ms ------------------------------------------------------ */
#define VC_T_STARTUP_MS		3000	/* sensors must be ready by then */
#define VC_T_ACK_MS		1500	/* stop -> STOP_ACK: Motion Control's answer, on top of
					   2 x (speed / its ramp rate) for the car to slow down */
#define VC_T_SCAN_MS		10000	/* request -> PROFILE */
/* No fixed time for a barcode command, a line search or a bypass: each
   move or turn has a watchdog of 2 x its own expected time (wheel travel
   / its speed) + VC_STEP_MARGIN_MS, so slower speeds, longer detours and
   bigger turns automatically get more time.  The three limits above are
   watchdogs on other tasks answering (sensors, Motion Control, the scan),
   not on how fast the car drives. */
#define VC_STEP_MARGIN_MS	2000	/* more than Motion Control's own margin  TUNE */
#define VC_HUMP_GRACE_MM	100	/* after a hump: travel before line loss counts again */
#define VC_HOLD_REFRESH_MS	200	/* re-send the wheel speeds at least this often */

#endif /* VC_CFG_H */
