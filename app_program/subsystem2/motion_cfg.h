/*
 *	motion_cfg.h
 *	Motion Control tuning and calibration values, each with its unit
 *	(Barr-C: no magic numbers).
 *	  MEASURE = replace with your own measurement (tutorial labs M4-M6)
 *	  TUNE    = adjust on the car until the behaviour is right
 *	Gains and both calibration factors can also be changed over MQTT
 *	with "SET" while the car is in READY (subsystem1/cmd_parse.c).
 */

#ifndef MOTION_CFG_H
#define MOTION_CFG_H

/* ---- calibration -------------------------------------------------------- */
#define MC_COUNTS_PER_M		2000	/* encoder counts per metre of travel        MEASURE */
#define MC_TRACK_MM		130	/* wheel centre to wheel centre              MEASURE */

/* Counts per wheel for a 90 degree spin = quarter of the circle the wheels
   run on (pi x track / 4) x counts per mm.  Wheel slip makes the real
   value a little larger: correct it with SET turn_cal after a test. */
#define MC_COUNTS_PER_90DEG	((((MC_TRACK_MM * 3142) / 4) * (MC_COUNTS_PER_M / 10)) / 100000)

/* ---- speed controller (per wheel) ------------------------------------- */
#define MC_KFF_X1000		1600	/* feed-forward, permille per mm/s, x1000   MEASURE */
#define MC_DEADBAND		250	/* permille needed before the wheel turns   MEASURE */
#define MC_KP_X1000		1000	/* permille per mm/s of speed error, x1000  TUNE */
#define MC_KI_X1000		4000	/* permille per mm of accumulated error, x1000  TUNE */
#define MC_SPEED_WINDOW		2	/* periods (x 20 ms) used to measure speed  TUNE */

/* ---- motion profile ----------------------------------------------------- */
#define MC_MAX_SPEED		400	/* mm/s, hard limit for every command */
#define MC_MIN_SPEED		60	/* mm/s, slowest speed while a move is running */
#define MC_ACCEL		600	/* mm/s^2 (ramps; stops the wheels slipping)  TUNE */
#define MC_TURN_SPEED		150	/* mm/s wheel speed while spinning           TUNE */
#define MC_K_STRAIGHT		4	/* mm/s of trim per mm of left/right difference  TUNE */
#define MC_SETTLE_SPEED		20	/* mm/s: slower than this counts as stopped */

/* ---- fault detection ---------------------------------------------------- */
#define MC_STALL_DUTY		600	/* permille */
#define MC_STALL_SPEED		10	/* mm/s */
#define MC_STALL_TICKS		25	/* x 20 ms = 0.5 s */
#define MC_TIMEOUT_MARGIN_MS	1000	/* a move may take 2 x expected + this */

#endif /* MOTION_CFG_H */
