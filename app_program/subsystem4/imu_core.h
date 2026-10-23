/*
 *	imu_core.h
 *	IMU and Terrain logic (Buddy 4): calibration, tilt (pitch), terrain
 *	(level / climbing / descending), hump detection and height, motion
 *	classification, collision (impact) detection.  Pure C, unit-tested
 *	and simulated on a laptop in testing/host/.
 *
 *	Hump height.  The LSM303DLHC has no gyroscope, so pitch comes from the
 *	direction of gravity in the low-pass filtered accelerometer reading,
 *	after removing the car's own forward acceleration (measured by the
 *	encoders).  Two estimates are kept and the larger one is reported:
 *	  - integral:  h += ds x sin(pitch), ds = encoder travel.  Right for
 *	    a hump longer than the car, where the body follows the slope.
 *	  - wheelbase: L x sin(largest pitch).  Right for a hump shorter than
 *	    the car, where the body tilts by asin(H / L) while one end is on
 *	    top and the other on the floor.
 *	Both are approximations: calibrate with humps of known height.
 */

#ifndef IMU_CORE_H
#define IMU_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_evt.h"

/* EV_TERRAIN.p1 */
#define TERRAIN_LEVEL		0
#define TERRAIN_CLIMBING	1
#define TERRAIN_DESCENDING	2

/* EV_MOTION.p1: motion classes */
#define MOTION_STATIONARY	0
#define MOTION_MOVING		1	/* steady speed, straight */
#define MOTION_ACCELERATING	2
#define MOTION_DECELERATING	3
#define MOTION_TURNING		4
#define MOTION_CLIMBING		5
#define MOTION_DESCENDING	6
#define MOTION_IMPACT		7
#define MOTION_CLASSES		8

#define IMU_PRE_MAX		32U	/* longest pre-hump history, samples */

typedef struct {
	uint32_t	axis_fwd;	/* accelerometer axis pointing forward: 0 X, 1 Y, 2 Z */
	int32_t		sign_fwd;	/* +1 / -1 */
	uint32_t	axis_up;
	int32_t		sign_up;
	uint32_t	axis_lat;	/* the remaining (sideways) axis */
	int32_t		sign_lat;
	uint32_t	cal_samples;	/* samples at rest for calibration */
	int32_t		lpf_shift;	/* low-pass: y += (x - y) / 2^shift */
	int32_t		climb_cdeg;	/* pitch beyond this = climbing / descending */
	int32_t		level_cdeg;	/* back within this = level (hysteresis) */
	int32_t		terrain_mm;	/* travel while tilted before climbing / descending
					   counts (braking and bumps tilt the car too) */
	int32_t		hump_end_mm;	/* level over this much travel = hump finished
					   (more than the wheelbase: the wheels cross the
					   hump after the castor) */
	uint32_t	hump_end_ms;	/* ... or level this long while standing still */
	uint32_t	pre_samples;	/* history added to a hump when it is detected */
	int32_t		hump_min_mm;	/* smaller "humps" are ignored */
	int32_t		wheelbase_mm;	/* drive axle to castor contact */
	int32_t		impact_mg;	/* horizontal jolt that may be a collision */
	uint32_t	impact_window_ms;	/* ... confirmed if speed drops within this */
	int32_t		impact_drop_pct;	/* ... to this % of the speed before */
	int32_t		impact_min_speed;	/* mm/s: slower than this cannot be a collision */
	int32_t		still_speed;	/* mm/s */
	int32_t		still_dev_mg;	/* vibration below this = still */
	int32_t		accel_mm_s2;	/* |dv/dt| above this = accelerating */
	int32_t		turn_cdeg_s;	/* |yaw rate| above this = turning */
	uint32_t	class_hold;	/* samples a new class must last before it is reported */
	uint32_t	fail_limit;	/* consecutive failed reads = IMU fault */
} IMU_PARAMS;

/* From Motion Control's snapshot, plus what the car is doing */
typedef struct {
	bool		valid;
	int32_t		dist_mm;	/* net distance, signed */
	int32_t		speed_mm_s;
	int32_t		accel_mm_s2;
	int32_t		yaw_cdeg_s;
	bool		running;	/* on a run (Vehicle Controller FOLLOW ... ALIGN):
					   only then are slopes and humps measured */
	bool		waiting;	/* in READY, waiting for START: standing still,
					   "level" is re-learned from the floor */
} IMU_MOTION;

typedef struct {
	IMU_PARAMS	p;
	/* calibration */
	bool		calibrated;
	uint32_t	cal_n;
	int32_t		cal_sum[3];
	int32_t		f0;		/* level gravity, forward and up, mg */
	int32_t		u0;
	int32_t		g0;		/* |gravity| at rest, mg */
	int32_t		base_cdeg;	/* mounting tilt */
	/* filters */
	int32_t		f;		/* filtered forward, up, sideways, mg */
	int32_t		u;
	int32_t		l;
	int32_t		acc_f;		/* filtered encoder acceleration, mm/s^2 */
	/* terrain and hump */
	int32_t		pitch_cdeg;
	int32_t		terrain;
	int32_t		tilt_mm;	/* travel so far with the pitch beyond climb_cdeg */
	int32_t		tilt_dir;	/* its direction: +1 up, -1 down */
	bool		in_hump;
	int32_t		h_um;
	int32_t		peak_um;
	int32_t		pmax_cdeg;
	uint32_t	level_since_ms;
	int32_t		level_mm;	/* travel since the terrain became level */
	int32_t		pre[IMU_PRE_MAX];
	uint32_t	pre_i;
	int32_t		pre_sum;
	bool		have_dist;
	int32_t		prev_dist;
	int32_t		last_hump_mm;
	int32_t		max_hump_mm;
	uint32_t	humps;
	/* impact */
	bool		impact_pending;
	uint32_t	impact_t_ms;
	int32_t		impact_speed;
	int32_t		impact_jump;
	uint32_t	impact_until_ms;
	uint32_t	bumps;		/* jolts that were not collisions */
	/* classification */
	int32_t		motion;		/* MOTION_... reported */
	int32_t		candidate;
	uint32_t	cand_n;
	/* health */
	uint32_t	fails;
} IMU_CORE;

typedef struct {
	EVT_LIST	ev;		/* EV_TERRAIN, EV_HUMP, EV_MOTION, EV_IMPACT */
	bool		fault;		/* the IMU stopped answering */
	bool		impact;		/* collision confirmed */
} IMU_OUT;

void	imu_core_init(IMU_CORE *c, const IMU_PARAMS *p);

/* One sample, every 10 ms.  a_mg = X, Y, Z in mg (ignored when !valid). */
void	imu_core_step(IMU_CORE *c, const int32_t a_mg[3], bool valid,
		      const IMU_MOTION *m, uint32_t now_ms, IMU_OUT *out);

/* ---- IMU results shared with telemetry and Vehicle Controller -----------
 * The IMU task (core 0) is the only writer; Vehicle Controller reads on
 * core 1.  Shared as three packed words (IC_STATUS_PK), each written with
 * one 32-bit store and read with one load.  The fields in different words
 * are independent, so a reader that gets two words from different 10 ms
 * cycles still gets valid values.
 *   w[0]  bit 0      calibrated          w[1]  bits 0-7    last hump mm
 *         bit 1      last read ok              bits 8-15   humps (stop at 255)
 *         bits 2-3   terrain (TERRAIN_)        bits 16-27  g0 mg (0..4095)
 *         bits 4-6   motion (MOTION_)    w[2]  bits 0-15   heading cdeg, +32768
 *         bits 7-20  pitch cdeg, +8192 (+-81.9 deg)
 *         bits 21-28 highest hump mm (stop at 255)                          */
typedef struct {
	bool		calibrated;	/* 1 s calibration done */
	bool		ok;		/* last read succeeded */
	int32_t		pitch_cdeg;	/* + = nose up, centidegrees */
	int32_t		terrain;	/* TERRAIN_... */
	int32_t		motion;		/* MOTION_... */
	int32_t		last_hump_mm;
	int32_t		max_hump_mm;	/* highest hump of the run */
	uint32_t	humps;
	int32_t		heading_cdeg;	/* magnetometer, uncalibrated (cross-check only) */
	int32_t		g0_mg;		/* |gravity| measured at calibration (about 1000) */
} IC_STATUS;

typedef struct {
	uint32_t	w[3];
} IC_STATUS_PK;

void	ic_status_pack(const IC_STATUS *s, IC_STATUS_PK *pk);
void	ic_status_unpack(const IC_STATUS_PK *pk, IC_STATUS *s);

#endif /* IMU_CORE_H */
