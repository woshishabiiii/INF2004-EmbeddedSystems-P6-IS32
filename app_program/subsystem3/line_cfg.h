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
/* JUNCTION versus a "both black" barcode band (design-review feedback IS32).
   A junction is both line sensors on black for LB_JUNCTION_MS AND for
   LB_JUNCTION_MM of travel (a car standing on black is not a junction).
   The supplied course puts each barcode 20 mm BESIDE the line: with the
   line sensors 25 mm apart (12.5 mm from the centre) and the bars starting
   29 mm from the centre, the line sensors never see a bar (16.5 mm clear),
   so no barcode band can look like a junction.  If a course ever puts a
   band ACROSS the line, raise LB_JUNCTION_MM to 12: a wide bar gives at
   most 9 mm of both-black travel, an 18 mm crossing line about 18 mm.
   Cost: a crossing line met at an angle shows less both-black travel
   (18 mm - 25 mm x tan(angle)), so 12 mm tolerates about 13 degrees of
   approach angle where 4 mm tolerates about 29; and the junction is
   reported 8 mm later, so VC_LINE_TO_AXLE_MM's move must shrink by 8 mm. */
#define LB_JUNCTION_MS		10U	/* both black at least this long ...           TUNE */
#define LB_JUNCTION_MM		4	/* ... and over this much travel (d_junction)  TUNE */
/* T_lost: with two straddling sensors "both white" is either centred or
   the start of a line loss.  The steering leans toward the side last seen
   (LS_D_BIAS, growing after LS_T_DRIFT_MS) so a centred car touches an
   edge well within T_lost; only then is the line lost.  At 200 mm/s,
   2 s is 400 mm of travel: validate at the target speed and lower it if
   the weave touches an edge reliably (feedback IS32). */
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
