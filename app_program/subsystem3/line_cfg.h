/*
 *	line_cfg.h
 *	Line and Barcode tuning values (Barr-C: named, with units).
 *	MEASURE = set from your own measurements;  TUNE = adjust on the track.
 *	Pure #defines, so the host simulator can include it too.
 */

#ifndef LINE_CFG_H
#define LINE_CFG_H

/* ---- pins and levels ---------------------------------------------------- */
#define LB_PERIOD_MS		5	/* line sampling period */
#define LB_LEFT_PIN		6U	/* Grove 5: left line sensor DO */
#define LB_RIGHT_PIN		7U	/* Grove 7: right line sensor DO */
#define LB_BC_ADC_CH		0	/* GP26 = ADC0: barcode sensor AO (calibration) */
#define LB_BLACK_LEVEL		1U	/* line sensor DO level over BLACK          MEASURE */
#define LB_BAR_LEVEL		1U	/* barcode sensor DO level over a bar       MEASURE */

/* ---- Region A ----------------------------------------------------------- */
#define LB_JUNCTION_MS		10U	/* both black at least this long ...           TUNE */
#define LB_JUNCTION_MM		4	/* ... and over this much travel (d_junction)  TUNE */
#define LB_T_LOST_MS		2000U	/* no black seen for this long = line lost (T_lost)  TUNE */

/* ---- barcode capture ----------------------------------------------------- */
#define LB_T_MIN_US		300U	/* shorter elements are comparator chatter     TUNE */
#define LB_T_END_MIN_MS		60U	/* minimum quiet time that ends a capture      TUNE */
#define LB_T_END_FACTOR		4U	/* ... or 4 x the widest element so far */

/* ---- line-following rule (used by Vehicle Controller, line_core.h) ------- */
/* Steering by delta mm/s slows the inside wheel by 2 x delta; the outside
   wheel keeps the base speed. */
#define LS_D_MIN		35	/* mm/s at first touch                          TUNE */
#define LS_D_RATE		30	/* mm/s more per 100 ms still on black          TUNE */
#define LS_D_MAX		90	/* mm/s largest correction                      TUNE */
#define LS_D_BIAS		12	/* mm/s lean toward the last side while centred TUNE */
#define LS_D_SEARCH		60	/* mm/s when the line has slipped outside       TUNE */
#define LS_T_ESCAPE_MS		250U	/* on black this long, then white: line went outside  TUNE */
#define LS_T_DRIFT_MS		250U	/* no touch for this long: the lean grows by D_BIAS per T_drift  TUNE */
#define LS_SLOW_PCT		60	/* % of base speed while turning back to the line */
#define LS_START_SIDE		1	/* side to lean toward before any black is seen */

#endif /* LINE_CFG_H */
