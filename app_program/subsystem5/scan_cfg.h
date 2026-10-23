/*
 *	scan_cfg.h
 *	Obstacle Scan tuning values (Buddy 5), each with its unit.
 *	MEASURE = measure on your car;  TUNE = adjust on the track.
 *	Pure #defines, so the host simulator can include it too.
 */

#ifndef SCAN_CFG_H
#define SCAN_CFG_H

#include "vc_cfg.h"		/* car width and margin */

#define OS_PERIOD_MS		10	/* scan state machine step */

/* 1 = the car may not run without the ultrasonic sensor (no response is
   a fault).  0 while it is not wired yet: failures are only printed. */
#define OS_REQUIRED		1

/* Obstacle stop, speed-aware (scan_core.h): the ALERT fires at OS_STOP_MM
   plus the distance the car still covers at its measured speed (the
   confirming pings, then its own measured stopping distance), so it comes
   to rest about OS_STOP_MM from the obstacle at any speed. */
#define OS_STOP_MM		150	/* come to rest this far from an obstacle  TUNE */
#define OS_SLOW_EXTRA_MM	150	/* slow down this much before the ALERT point  TUNE */
#define OS_SLOW_HYST_MM		50	/* back to normal speed beyond that + this */
#define OS_DECEL_GUESS		1000	/* mm/s^2, used only until the car has measured
					   its own stop (Motion Control learns it)  MEASURE */
#define OS_CONFIRM		2U	/* close readings in a row for an ALERT */
#define OS_GAP_MS		70U	/* between triggers (datasheet: > 60 ms) */
#define OS_SETTLE_BASE_MS	40U	/* servo settling after any move         MEASURE */
#define OS_SETTLE_MS_PER_DEG	3U	/* ... plus this per degree moved        MEASURE */
#define OS_RETRIES		2U	/* extra pings when there is no echo */
#define OS_R_MAX_MM		800	/* farther = background, not obstacle */
#define OS_CORRIDOR_MM		((VC_CAR_WIDTH_MM / 2) + VC_MARGIN_MM)	/* the car's path, each side */
#define OS_COARSE_STEP		30	/* deg: 30, 60, 90, 120, 150 */
#define OS_FINE_STEP		5	/* deg */
#define OS_FINE_MARGIN		25	/* deg scanned beyond the outer coarse hits */
#define OS_MISSING_LIMIT	10U	/* pings in a row with no response = fault */
#define OS_PROFILE_MAX_MS	8000U	/* a whole profile must finish within this */

#endif /* SCAN_CFG_H */
