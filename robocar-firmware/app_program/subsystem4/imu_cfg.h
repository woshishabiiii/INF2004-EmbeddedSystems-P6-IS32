/*
 *	imu_cfg.h
 *	IMU and Terrain, and Safety Monitor tuning values (Buddy 4).
 *	MEASURE = set from your own measurements;  TUNE = adjust on the car.
 *	Pure #defines, so the host simulator can include it too.
 */

#ifndef IMU_CFG_H
#define IMU_CFG_H

#define IMU_PERIOD_MS		10	/* 100 Hz, same as the accelerometer's data rate */

/* 1 = the car may not run without the IMU (an I2C failure is a fault).
   0 while the IMU is not wired yet: failures are only printed. */
#define IMU_REQUIRED		1

/* Mounting: which accelerometer axis points forward, up and sideways,
   and their signs (0 = X, 1 = Y, 2 = Z).  With the car level, "up" must
   read about +1000 mg; tilting the nose up must make "forward" POSITIVE. */
#define IMU_AXIS_FWD		0U	/* MEASURE */
#define IMU_SIGN_FWD		(1)	/* MEASURE */
#define IMU_AXIS_UP		2U	/* MEASURE */
#define IMU_SIGN_UP		(1)	/* MEASURE */
#define IMU_AXIS_LAT		1U	/* MEASURE */
#define IMU_SIGN_LAT		(1)	/* MEASURE */

#define IMU_CAL_SAMPLES		100U	/* 1 s at rest, level, right after power-up */
#define IMU_LPF_SHIFT		3	/* low-pass: y += (x - y) / 8, about 80 ms */
#define IMU_CLIMB_CDEG		300	/* pitch above +3.00 deg = climbing      TUNE */
#define IMU_LEVEL_CDEG		150	/* back below 1.50 deg = level (hysteresis) */
#define IMU_TERRAIN_MM		20	/* travel while tilted before it counts      TUNE */
#define IMU_HUMP_END_MM		(IMU_WHEELBASE_MM + 40)	/* level over this much travel = hump over */
#define IMU_HUMP_END_MS		2000U	/* ... or level this long while stopped */
#define IMU_PRE_SAMPLES		20U	/* 200 ms of rise counted before the threshold */
#define IMU_HUMP_MIN_MM		3	/* smaller "humps" are not reported */
#define IMU_WHEELBASE_MM	120	/* drive axle to castor contact          MEASURE */

#define IMU_IMPACT_MG		1500	/* horizontal jolt that may be a collision TUNE */
#define IMU_IMPACT_WINDOW_MS	150U	/* ... if the speed drops within this */
#define IMU_IMPACT_DROP_PCT	50	/* ... to half the speed before, or less */
#define IMU_IMPACT_MIN_SPEED	80	/* mm/s: slower = cannot be our collision */
#define IMU_IMPACT_STOPS	1	/* 1 = a collision is a fault (Safe Stop) */

#define IMU_STILL_SPEED		10	/* mm/s */
#define IMU_STILL_DEV_MG	40	/* vibration below this = still */
#define IMU_ACCEL_MM_S2		250	/* |dv/dt| above this = accelerating */
#define IMU_TURN_CDEG_S		3000	/* |yaw rate| above 30 deg/s = turning */
#define IMU_CLASS_HOLD		5U	/* samples (50 ms) before a new class is reported */
#define IMU_FAIL_LIMIT		5U	/* failed reads in a row = IMU fault */
#define IMU_MAG_EVERY		10U	/* magnetometer heading every 10 samples (100 ms) */

/* ---- Safety Monitor ------------------------------------------------------ */
#define SM_PERIOD_MS		20	/* buttons and health every 20 ms */
#define SM_RESTART_POLLS	5U	/* faults must stay clear 5 x 20 ms */
#define SM_BTN_START_PIN	20U	/* Robo Pico user button GP20 */
#define SM_BTN_STOP_PIN		21U	/* Robo Pico user button GP21 */
/* Oldest allowed health stamp, ms (several periods: core 0 also runs the
   kernel's USB and WiFi tasks above ours).  0 = not checked. */
#define SM_HEALTH_MOTION_MS	100U
#define SM_HEALTH_LINE_MS	50U
#define SM_HEALTH_IMU_MS	250U
#define SM_HEALTH_SCAN_MS	250U
#define SM_HEALTH_VC_MS		200U

#endif /* IMU_CFG_H */
