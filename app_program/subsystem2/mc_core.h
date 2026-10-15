/*
 *	mc_core.h
 *	Motion Control logic (Buddy 2): per-wheel PI speed control, distance
 *	and turn moves with a trapezoidal speed profile, straight-line
 *	correction, the stop latch, stall / timeout / encoder checks and the
 *	odometry snapshot.
 *
 *	Pure C (stdint / stdbool only): no kernel calls, no registers.  The
 *	Motion Control task (task_motion.c) reads the encoders and stop bits,
 *	calls mc_core_step() and writes the duties it returns to the motors.
 *	The same file runs on a laptop in testing/host/ (unit tests and the
 *	car simulator).
 */

#ifndef MC_CORE_H
#define MC_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_evt.h"

#define MC_PERIOD_MS		20	/* control period (cyclic handler) */
#define MC_WHEELS		2U
#define MC_LEFT			0U
#define MC_RIGHT		1U
#define MC_DUTY_MAX		1000	/* duty in permille: 1000 = 100 % */
#define MC_SPEED_WINDOW_MAX	8U
#define MC_ACCEL_WINDOW		5U	/* periods (100 ms) over which acceleration is measured */

/* ---- commands (motion_cmd_q, 16 bytes) ---------------------------------- */
#define MC_CMD_SPEED		1U	/* a = left mm/s,  b = right mm/s */
#define MC_CMD_DISTANCE		2U	/* a = signed mm (+ forward), b = speed mm/s, c = move id */
#define MC_CMD_TURN		3U	/* a = signed deg (+ left), b = wheel speed mm/s (0 = default), c = move id */

typedef struct {
	uint8_t		type;		/* MC_CMD_... */
	uint8_t		pad[3];
	int32_t		a;
	int32_t		b;
	int32_t		c;
} MC_CMD;

/* EV_DONE.p1 */
#define MC_MOVE_DISTANCE	1
#define MC_MOVE_TURN		2

/* Own fault codes (MC_SNAPSHOT.fault_code) */
#define MC_FLT_NONE		0
#define MC_FLT_STALL		1	/* high duty, wheel not turning */
#define MC_FLT_TIMEOUT		2	/* a move took far longer than expected */
#define MC_FLT_ENCODER		3	/* wheel turns against its duty: encoder or motor wired backwards */

typedef enum {
	MC_IDLE = 0,		/* braked, waiting for a command */
	MC_SPEED,		/* holding wheel speeds (line following) */
	MC_DISTANCE,		/* driving a distance */
	MC_TURN,		/* spinning on the spot */
	MC_SETTLE,		/* move finished, braked, waiting for the wheels to stop */
	MC_STOPPED,		/* CMD_STOP: braked, commands refused */
	MC_FAULT		/* FAULT_STOP: braked until Safety Monitor clears it */
} MC_MODE;

/* Tuning and calibration (defaults in motion_cfg.h) */
typedef struct {
	int32_t	counts_per_m;		/* encoder counts per metre of travel */
	int32_t	counts_per_90deg;	/* counts per wheel for a 90 degree spin */
	int32_t	track_mm;		/* distance between the wheel contact points */
	int32_t	kff_x1000;		/* feed-forward, permille per mm/s, x1000 */
	int32_t	deadband;		/* permille needed to start the wheel turning */
	int32_t	kp_x1000;		/* proportional gain, permille per mm/s, x1000 */
	int32_t	ki_x1000;		/* integral gain, permille per mm, x1000 */
	int32_t	max_speed;		/* mm/s */
	int32_t	min_speed;		/* mm/s, slowest speed while a move is running */
	int32_t	accel;			/* mm/s^2, ramp for moves and speed changes */
	int32_t	turn_speed;		/* mm/s wheel speed while spinning */
	int32_t	k_straight;		/* mm/s of trim per mm of left/right difference */
	int32_t	settle_speed;		/* mm/s: slower than this = stopped */
	int32_t	stall_duty;		/* permille */
	int32_t	stall_speed;		/* mm/s */
	int32_t	stall_ticks;		/* control periods */
	int32_t	timeout_margin_ms;
	int32_t	speed_window;		/* periods used to measure speed, 1..MC_SPEED_WINDOW_MAX */
} MC_PARAMS;

/* What Motion Control shares with the rest of the car (dist_snapshot) */
typedef struct {
	uint32_t	seq;		/* +1 every control period */
	uint32_t	t_ms;
	int32_t		count[MC_WHEELS];	/* raw encoder counts */
	int32_t		speed[MC_WHEELS];	/* measured, mm/s */
	int32_t		target[MC_WHEELS];	/* commanded, mm/s */
	int32_t		duty[MC_WHEELS];	/* permille */
	int32_t		speed_mm_s;	/* mean of both wheels */
	int32_t		accel_mm_s2;	/* change of the mean speed per second, over 100 ms */
	int32_t		dist_mm;	/* net distance since start-up (reversing subtracts) */
	int32_t		odo_mm;		/* total distance travelled (always grows) */
	int32_t		yaw_cdeg_s;	/* turn rate from the wheel speeds, + = left */
	int32_t		mode;		/* MC_MODE */
	int32_t		fault_code;	/* MC_FLT_... */
} MC_SNAPSHOT;

typedef struct {
	int32_t	integ;			/* PI integrator, permille x 1000 */
	int32_t	target;			/* mm/s */
	int32_t	speed;			/* mm/s */
	int32_t	duty;			/* permille */
	int32_t	hist[MC_SPEED_WINDOW_MAX + 1U];	/* recent counts, for speed */
	int32_t	backwards;		/* periods spent turning against the duty */
} MC_WHEEL;

typedef struct {
	MC_PARAMS	p;		/* in use (after calibration) */
	int32_t		cpm_base;	/* counts_per_m as configured */
	int32_t		c90_base;	/* counts_per_90deg as configured */
	MC_MODE		mode;
	MC_WHEEL	w[MC_WHEELS];
	int32_t		start[MC_WHEELS];	/* counts when the move started */
	int32_t		goal_counts;	/* per wheel, for distance / turn */
	int32_t		dir;		/* +1 / -1 */
	int32_t		vmax;		/* mm/s for this move */
	int32_t		vcmd;		/* ramped profile speed, mm/s */
	int32_t		hold_common;	/* speed mode: ramped common speed */
	int32_t		hold_cmd[MC_WHEELS];	/* speed mode: requested wheel speeds */
	int32_t		move_type;	/* MC_MOVE_... of the move being finished */
	int32_t		move_id;
	int32_t		ticks_left;	/* move timeout counter */
	int32_t		settle_ticks;
	int32_t		stall_ticks;
	bool		await_fault_flag;	/* own fault: stay in MC_FAULT until FAULT_STOP is seen */
	bool		ack_pending;	/* STOP_ACK still to be sent */
	uint32_t	stop_req_us;	/* when the stop was requested */
	int32_t		odo_rem;	/* odometer remainder (mm x counts_per_m) */
	int32_t		dist_rem;	/* net distance remainder */
	int32_t		mean_hist[MC_ACCEL_WINDOW + 1U];	/* mean speeds, newest first */
	uint32_t	rejected;	/* invalid commands refused */
	MC_SNAPSHOT	snap;
} MC_CORE;

typedef struct {
	uint32_t	now_ms;
	uint32_t	now_us;
	int32_t		count[MC_WHEELS];
	bool		tick;		/* TRUE on the 20 ms release: run the control loop */
	bool		cmd_stop;	/* stop_flags CMD_STOP */
	bool		fault_stop;	/* stop_flags FAULT_STOP */
	uint32_t	stop_req_us;	/* time of the latest stop request */
	uint32_t	ncmd;		/* new commands, oldest first (only read on a tick) */
	const MC_CMD	*cmd;
} MC_IN;

typedef struct {
	int32_t		duty[MC_WHEELS];	/* write these to the motors */
	bool		flush;		/* discard every queued command */
	bool		fault;		/* own fault: set FAULT_STOP and FLT_MOTION */
	uint32_t	stop_latency_us;	/* non-zero once: request -> duty 0 */
	EVT_LIST	ev;		/* EV_DONE, EV_STOP_ACK */
} MC_OUT;

void	mc_core_init(MC_CORE *c, const MC_PARAMS *p, const int32_t count[MC_WHEELS]);

/* SET values copied at START: gains x1000 and calibration in permille
   (1000 = as configured).  Out-of-range values are ignored. */
void	mc_core_tune(MC_CORE *c, int32_t kp_x1000, int32_t ki_x1000,
		     int32_t dist_cal_permille, int32_t turn_cal_permille);

void	mc_core_step(MC_CORE *c, const MC_IN *in, MC_OUT *out);

/* Validity of a command (also used by the API before queueing it) */
bool	mc_cmd_valid(const MC_CMD *cmd, int32_t max_speed);

#endif /* MC_CORE_H */
