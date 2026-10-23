/*
 *	safety_core.h
 *	Safety Monitor logic (Buddy 4): the fault latch, RESTART handling,
 *	the START/STOP buttons and task-health supervision.  Pure C.
 *
 *	Rules (Figure 3 of the design review):
 *	  - any fault bit -> latch FAULT_STOP at once and report EV_FAULT;
 *	  - the latch is released only after a RESTART (MQTT or the START
 *	    button) AND every fault bit has stayed clear for restart_polls
 *	    polls (20 ms each); then EV_FAULT_CLEAR is reported;
 *	  - buttons are debounced over two polls; START while faulted is a
 *	    RESTART request;
 *	  - a periodic task whose health stamp is older than its limit
 *	    raises FLT_HEALTH.
 */

#ifndef SAFETY_CORE_H
#define SAFETY_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_evt.h"

/* Fault table (design-review feedback IS32: separate true safety faults
 * from retryable mission failures).
 *
 * SAFETY FAULTS: a bit below.  Safety Monitor latches FAULT_STOP (motors
 * off) and only a RESTART, after every bit has stayed clear for 100 ms,
 * releases it.  Vehicle Controller shows SAFE_STOP.
 *
 * MISSION FAILURES: no fault bit.  Vehicle Controller stops the car with
 * CMD_STOP, shows HALTED with the reason (VC_WHY_... in vc_core.h) and
 * START carries on: line not found, no way round after the retries, no
 * obstacle profile after a rescan, obstacle during a bypass, a barcode
 * command or a bypass taking too long.
 *
 * COUNTED ONLY (the car carries on): a barcode capture lost to an
 * edge_buf overflow (EV_BARCODE_BAD), a non-safety event dropped from
 * vc_event_q (evt_is_safety()), a telemetry record or message dropped.
 *
 * Any task sets ITS OWN bit with tk_set_flg().
 *
 * Minimum-bit representation (Tribunal 1): each fault condition is one
 * bit of this shared bitmask, never an integer code.  Bits 0-9 latch
 * (FLT_ALL); bits 13-14 are STATUS bits that are reported but never latch
 * (no echo is normal whenever nothing is in range; Wi-Fi loss leaves the
 * STOP button and the obstacle stop working).  tk_set_flg / tk_clr_flg
 * do the read-modify-write inside the kernel, so two cores setting bits
 * at the same time cannot lose one. */
#define FLT_STALL		0x0001U	/* Motion: high duty, wheel not turning */
#define FLT_LINE		0x0002U	/* barcode interrupt could not be set up (start-up) */
#define FLT_IMU			0x0004U	/* IMU not answering (IMU_REQUIRED) */
#define FLT_IMPACT		0x0008U	/* collision detected */
#define FLT_SCAN		0x0010U	/* ultrasonic not answering: the car is blind */
#define FLT_QUEUE_OVF		0x0020U	/* a SAFETY event or a motion command could not be queued */
#define FLT_HEALTH		0x0040U	/* a task stopped stamping task_health */
#define FLT_NAV			0x0080U	/* Vehicle Controller: wheels did not stop after CMD_STOP */
#define FLT_ENCODER		0x0100U	/* Motion: wheel turns against its duty (encoder or motor backwards) */
#define FLT_MOVE_TIMEOUT	0x0200U	/* Motion: a move took far longer than expected */
#define FLT_ALL			0x03FFU	/* every latching fault (10 bits) */
#define FLT_NO_ECHO		0x2000U	/* STATUS, never latches: last ping heard no echo (Obstacle Scan) */
#define FLT_WIFI_DOWN		0x4000U	/* STATUS, never latches: MQTT link down (Telemetry) */
#define FLT_STATUS		(FLT_NO_ECHO | FLT_WIFI_DOWN)
#define FLT_RESTART_REQ		0x8000U	/* not a fault: a RESTART was requested */

/* task_health slots: each periodic task stamps its slot every cycle */
#define HEALTH_MOTION		0U
#define HEALTH_LINE		1U
#define HEALTH_IMU		2U
#define HEALTH_SCAN		3U
#define HEALTH_VC		4U
#define HEALTH_SLOTS		5U

#define SM_BTN_RELEASED		1U	/* buttons are active low */

typedef struct {
	uint32_t	limit_ms[HEALTH_SLOTS];	/* longest allowed age; 0 = not checked */
	uint32_t	restart_polls;		/* faults must stay clear this many polls */
} SM_PARAMS;

typedef struct {
	SM_PARAMS	p;
	bool		faulted;	/* FAULT_STOP latched */
	uint32_t	latched;	/* fault bits seen since the latch */
	uint32_t	restart_left;	/* polls until the latch is released */
	uint32_t	prev[2];	/* button levels: [0] START, [1] STOP */
	uint32_t	stable[2];
	uint32_t	health_bad;	/* slots already reported as silent */
	uint16_t	h_last[HEALTH_SLOTS];	/* stamp seen at the previous poll */
	uint32_t	h_seen_ms[HEALTH_SLOTS];	/* full time of that stamp */
} SM_CORE;

typedef struct {
	bool		latch;		/* set FAULT_STOP now */
	bool		release;	/* clear FAULT_STOP now */
	bool		clear_flags;	/* clear every bit of fault_flags */
	bool		set_restart;	/* set FLT_RESTART_REQ */
	uint32_t	raise;		/* fault bits to set */
	EVT_LIST	ev;		/* EV_FAULT, EV_FAULT_CLEAR, EV_BUTTON */
} SM_OUT;

void	 sm_init(SM_CORE *s, const SM_PARAMS *p);

/* The bits to wait for in fault_flags: while latched and no restart is
   counting down, only FLT_RESTART_REQ (the fault bits stay set until the
   restart, so waiting for them would return at once, every time). */
uint32_t sm_wait_pattern(const SM_CORE *s);

/* The wait returned these bits */
void	 sm_on_flags(SM_CORE *s, uint32_t ptn, SM_OUT *out);

/* Health stamps are 16-bit ms times (app_ms() | 1, 0 = never stamped),
   one halfword per task, each written only by its own task: a halfword
   store does not touch the stamp next to it.  The age of a NEW stamp is
   the wrap-safe signed 16-bit difference (a stamp a moment newer than
   'now' is fresh, not 65 s old).  An unchanged stamp ages from the full
   32-bit time it was first seen, so a task that stays silent for more
   than 65.5 s is never mistaken for a fresh one after the wrap. */
void	 sm_health_ages(SM_CORE *s, uint32_t now_ms, const uint16_t stamp[HEALTH_SLOTS],
			uint32_t age_ms[HEALTH_SLOTS], bool alive[HEALTH_SLOTS]);

/* The 20 ms poll: button levels (SM_BTN_RELEASED = not pressed) and the
   age of every health stamp (alive[i] FALSE = that task has not stamped yet) */
void	 sm_on_poll(SM_CORE *s, uint32_t btn_start, uint32_t btn_stop,
		    const uint32_t age_ms[HEALTH_SLOTS], const bool alive[HEALTH_SLOTS],
		    SM_OUT *out);

#endif /* SAFETY_CORE_H */
