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

/* fault_flags bits.  Any task sets ITS OWN bit with tk_set_flg(). */
#define FLT_MOTION		0x0001U	/* stall, move timeout, encoder direction */
#define FLT_LINE		0x0002U	/* barcode interrupt set-up, edge_buf overflow */
#define FLT_IMU			0x0004U	/* IMU not answering */
#define FLT_IMPACT		0x0008U	/* collision detected */
#define FLT_SCAN		0x0010U	/* ultrasonic not answering, profile timeout */
#define FLT_QUEUE_OVF		0x0020U	/* a send to vc_event_q failed */
#define FLT_HEALTH		0x0040U	/* a task stopped stamping task_health */
#define FLT_NAV			0x0080U	/* Vehicle Controller: search, retry or stop-ack limit */
#define FLT_ALL			0x00FFU
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

/* The 20 ms poll: button levels (SM_BTN_RELEASED = not pressed) and the
   age of every health stamp (alive[i] FALSE = that task has not stamped yet) */
void	 sm_on_poll(SM_CORE *s, uint32_t btn_start, uint32_t btn_stop,
		    const uint32_t age_ms[HEALTH_SLOTS], const bool alive[HEALTH_SLOTS],
		    SM_OUT *out);

#endif /* SAFETY_CORE_H */
