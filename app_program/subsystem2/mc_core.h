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

/* ---- commands (motion_cmd_q: MC_CMD_PK, 8 bytes) ------------------------- */
#define MC_CMD_SPEED		1U	/* a = left mm/s,  b = right mm/s, c = time (ms) of the
				   line reading that caused it, 0 = none (latency log;
				   only its low 16 bits are used: bp_age16) */
#define MC_CMD_DISTANCE		2U	/* a = signed mm (+ forward), b = speed mm/s, c = move id */
#define MC_CMD_TURN		3U	/* a = signed deg (+ left), b = wheel speed mm/s (0 = default), c = move id */

/* A command, unpacked: what mc_core_step() works on */
typedef struct {
	uint8_t		type;		/* MC_CMD_... */
	uint8_t		pad[3];
	int32_t		a;
	int32_t		b;
	int32_t		c;
} MC_CMD;

/* A command, packed: what goes through motion_cmd_q (8 + 4 B per slot,
   was 16 + 4 B).  Fields LSB first (bitpack.h); signed values stored
   with an offset; values outside the range saturate (the API checks
   mc_cmd_valid() first, so in practice they never do).
     bits 0-1   type          2   MC_CMD_SPEED / DISTANCE / TURN
     SPEED      a left        10  -512..511 mm/s  (valid +-MC_MAX_SPEED = 400)
                b right       10  -512..511 mm/s
                c present     1   1 = c holds a line-reading time
                c time        16  low 16 bits of app_ms(): latency only
                                  (bp_age16), wraps every 65.5 s   = 39 bits
     DISTANCE   a mm          15  -16384..16383   (valid +-MC_MAX_DISTANCE_MM = 10,000)
                b speed       9   0..511 mm/s     (valid 1..400)
                c move id     8   0..255 (app_id_next)              = 34 bits
     TURN       a degrees     10  -512..511       (valid +-MC_MAX_TURN_DEG = 360)
                b speed       9   0..511 mm/s     (valid 0..400, 0 = default)
                c move id     8                                     = 29 bits
   Largest: SPEED, 39 bits (Version C planned 42 for one shared layout). */
#define MC_CMD_PK_BYTES		8U

typedef struct {
	uint8_t		b[MC_CMD_PK_BYTES];
} MC_CMD_PK;

void	mc_cmd_pack(const MC_CMD *cmd, MC_CMD_PK *pk);
void	mc_cmd_unpack(const MC_CMD_PK *pk, MC_CMD *cmd);

/* EV_DONE.p1 */
#define MC_MOVE_DISTANCE	1
#define MC_MOVE_TURN		2

/* Own faults: one bit each (MC_SNAPSHOT.fault_bits, 3 bits), never an
   integer code.  The task raises FLT_STALL / FLT_MOVE_TIMEOUT /
   FLT_ENCODER in fault_flags for them. */
#define MC_FLT_NONE		0x0U
#define MC_FLT_STALL		0x1U	/* high duty, wheel not turning */
#define MC_FLT_TIMEOUT		0x2U	/* a move took far longer than expected */
#define MC_FLT_ENCODER		0x4U	/* wheel turns against its duty: encoder or motor wired backwards */

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
	int32_t	duty_rise;		/* permille per call: largest increase of a wheel's drive (0 = none) */
	int32_t	stop_coast;		/* 1: CMD_STOP coasts until stopped, then brakes; 0: brakes */
	int32_t	steer_pm_mm;		/* smooth steering: in a speed hold a wheel's speed
					   changes by at most this permille of the travel
					   speed per mm travelled (0 = no limit) */
} MC_PARAMS;

/* What Motion Control shares with the rest of the car (dist_snapshot),
   unpacked.  Only the packed MC_SNAP_PK below is shared: Motion Control
   packs it and copies it in under mtx_snapshot; motion_get_snapshot()
   copies it out under the same mutex and unpacks the copy. */
typedef struct {
	uint32_t	seq;		/* +1 every control period (8 bits shared: wraps) */
	uint32_t	t_ms;		/* low 16 bits shared: bp_age16() only */
	int32_t		count[MC_WHEELS];	/* encoder position (low 16 bits shared) */
	int32_t		speed[MC_WHEELS];	/* measured, mm/s */
	int32_t		target[MC_WHEELS];	/* commanded, mm/s */
	int32_t		duty[MC_WHEELS];	/* permille */
	int32_t		speed_mm_s;	/* mean of both wheels */
	int32_t		accel_mm_s2;	/* change of the mean speed per second, over 100 ms */
	int32_t		dist_mm;	/* net distance since start-up (reversing subtracts) */
	int32_t		odo_mm;		/* total distance travelled (always grows) */
	int32_t		yaw_cdeg_s;	/* turn rate from the wheel speeds, + = left */
	int32_t		mode;		/* MC_MODE */
	uint32_t	fault_bits;	/* MC_FLT_... bits */
	int32_t		steer_lat_ms;	/* line reading -> new wheel targets, last hold */
	int32_t		steer_lat_max_ms;	/* ... largest so far */
	int32_t		stop_lat_max_us;	/* stop request -> duty 0, largest so far */
	int32_t		stop_dist_mm;	/* travel from CMD_STOP to stopped, last stop */
	int32_t		stop_decel_mm_s2;	/* how hard this car really stops, learned from
						   its own stops (speed^2 / 2 x distance); 0 = not yet */
} MC_SNAPSHOT;

/* The packed snapshot (88 B -> 36 B).  Fields LSB first (bitpack.h):
     seq            8   wraps
     t_ms          16   low 16 bits, wraps every 65.5 s
     count[2]    2x16   low 16 bits of each position (telemetry)
     speed[2]    2x10   -512..511 mm/s (top speed at full duty ~469)
     target[2]   2x10   -512..511 mm/s (commands +-400)
     duty[2]     2x11   -1024..1023 permille (+-MC_DUTY_MAX)
     speed_mm_s    10   -512..511 mm/s
     accel_mm_s2   15   -16384..16383 mm/s^2 (worst: +-938 mm/s in 100 ms)
     dist_mm       25   +-16.7 km net
     odo_mm        24   0..16.7 km (same as EV_JUNCTION)
     yaw_cdeg_s    17   +-655 deg/s (wheels +-469 mm/s, 130 mm track: +-413)
     mode           3   7 MC_MODEs
     fault_bits     3   MC_FLT_...
     steer_lat_ms  10   0..1023 ms   } latency logs: a few ms expected,
     steer_lat_max 10   0..1023 ms   } saturate rather than wrap
     stop_lat_max  17   0..131 ms in us (same as EV_STOP_ACK)
     stop_dist_mm  10   0..1023 mm   (same as EV_STOP_ACK)
     stop_decel    15   0..32767 mm/s^2 (learned, clamped to 100..20,000; 0 = not yet)
                  ---
                  277 bits = 35 B, stored in 9 words = 36 B (whole-word copy) */
#define MC_SNAP_PK_BITS		277U
#define MC_SNAP_PK_BYTES	36U

typedef struct {
	uint32_t	w[MC_SNAP_PK_BYTES / 4U];
} MC_SNAP_PK;

void	mc_snap_pack(const MC_SNAPSHOT *s, MC_SNAP_PK *pk);
void	mc_snap_unpack(const MC_SNAP_PK *pk, MC_SNAPSHOT *s);

/* Only the odometer, read straight from the packed snapshot: the Line and
   Vehicle Controller tasks need nothing else, and one 24-bit read costs
   far less than unpacking all 277 bits on every cycle. */
int32_t	mc_snap_odo(const MC_SNAP_PK *pk);

/* ---- encoder transfer (ISR -> task) --------------------------------------
 * The GPIO interrupt keeps each wheel's count in one 16-bit half of a
 * 32-bit word (left = bits 0-15, right = bits 16-31) and is the only
 * writer.  Motion Control reads the whole word with ONE 32-bit load (both
 * counts from the same moment) and adds each 16-bit difference to its own
 * 32-bit position, which the moves use.  The difference is right while a
 * wheel moves fewer than 32,768 counts between two reads: at 2,000 counts/m
 * that is 16 m, and the task reads every 20 ms (16 counts at 400 mm/s). */
typedef struct {
	uint32_t	last;		/* word at the previous read */
	int32_t		pos[MC_WHEELS];	/* 32-bit positions */
} MC_ENC;

void	mc_enc_init(MC_ENC *e, uint32_t word);
void	mc_enc_update(MC_ENC *e, uint32_t word);

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
	int32_t		stop_odo;	/* odometer when CMD_STOP was seen */
	int32_t		odo_rem;	/* odometer remainder (mm x counts_per_m) */
	int32_t		dist_rem;	/* net distance remainder */
	int32_t		mean_hist[MC_ACCEL_WINDOW + 1U];	/* mean speeds, newest first */
	uint32_t	rejected;	/* invalid commands refused */
	bool		coasting;
	int32_t		coast_ticks;	/* ticks spent coasting since CMD_STOP */
	int32_t		steer_room[MC_WHEELS];	/* wheel target change still allowed before
						   the next tick (steer_pm_mm), mm/s */
	int32_t		stop_speed;	/* mean |speed| when CMD_STOP arrived, mm/s */	/* CMD_STOP: motors free-wheeling until the wheels stop */
	MC_SNAPSHOT	snap;
} MC_CORE;

typedef struct {
	uint32_t	now_ms;
	uint32_t	now_us;
	int32_t		count[MC_WHEELS];
	bool		tick;		/* TRUE on the 20 ms release: run the control loop;
					   FALSE: woken by a stop or a new speed hold */
	bool		cmd_stop;	/* stop_flags CMD_STOP */
	bool		fault_stop;	/* stop_flags FAULT_STOP */
	uint32_t	stop_req_us;	/* time of the latest stop request */
	uint32_t	ncmd;		/* new commands, oldest first (taken on every call) */
	const MC_CMD	*cmd;
} MC_IN;

typedef struct {
	int32_t		duty[MC_WHEELS];	/* write these to the motors */
	bool		flush;		/* discard every queued command */
	bool		fault;		/* own fault: set FAULT_STOP and the FLT_ bit for snap.fault_bits */
	uint32_t	stop_latency_us;	/* non-zero once: request -> duty 0 */
	bool		coast;		/* TRUE: let the motors coast (duty is 0) instead of braking */
	EVT_LIST	ev;		/* EV_DONE, EV_STOP_ACK [latency us, stop distance mm] */
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
