/*
 *	vc_core.h
 *	Vehicle Controller logic (Buddy 5): the mission statechart of Figure 4
 *	and the Vehicle Controller states of Figure 7 -- follow the line, run
 *	barcode commands at junctions, search for a lost line, stop and scan
 *	for obstacles, choose Continue / Bypass / Reverse and Retry, bypass,
 *	find and rejoin the line, Safe Stop, Halted and Finished.
 *
 *	Two kinds of stop (fault table in safety_core.h):
 *	  SAFE_STOP  a safety fault: Safety Monitor has latched FAULT_STOP;
 *	             leave with RESTART, then START.
 *	  HALTED     a mission failure (line not found, no way round, ...):
 *	             stopped with CMD_STOP, nothing latched; START carries on.
 *
 *	Pure C.  It never calls the motion API itself: every step returns a
 *	short list of ACTIONS (hold speed, move, turn, stop, scan request,
 *	raise a fault, telemetry record) which the Vehicle Controller task
 *	carries out in order.  The host simulator in testing/host/ carries
 *	them out on a simulated car instead.
 */

#ifndef VC_CORE_H
#define VC_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_evt.h"
#include "../subsystem3/line_core.h"	/* the line-following rule */
#include "os_profile.h"

typedef enum {
	VC_STARTUP = 0,		/* tasks starting, sensors self-testing */
	VC_READY,		/* motors held (CMD_STOP); waiting for START */
	VC_FOLLOW,		/* following the line */
	VC_EXEC,		/* running a barcode command at a junction */
	VC_SEARCH,		/* line lost: turning to find it */
	VC_WAIT_ACK,		/* obstacle: waiting for the wheels to stop */
	VC_SCANNING,		/* obstacle: coarse + fine scan */
	VC_BYPASS,		/* driving round the obstacle */
	VC_REVERSE,		/* backing off before scanning again */
	VC_FIND_LINE,		/* heading back to the line */
	VC_ALIGN,		/* line found: lining up with it */
	VC_SAFE_STOP,		/* safety fault: motors off until RESTART */
	VC_FINISHED,		/* FINISH received: run summary sent */
	VC_HALTED,		/* mission failure: stopped (CMD_STOP), START carries on */
	VC_STATES
} VC_STATE;

/* Actions */
#define VA_HOLD		1U	/* a = left mm/s, b = right mm/s, c = time of the line
				   reading that caused it (0 = none), for the latency log */
#define VA_MOVE		2U	/* a = signed mm (+ forward), b = speed, c = move id */
#define VA_TURN		3U	/* a = signed degrees (+ left), c = move id */
#define VA_STOP		4U	/* set CMD_STOP */
#define VA_CLEAR_STOP	5U	/* clear CMD_STOP */
#define VA_SCAN		6U	/* a = SCAN_... (+ SCAN_STOP_FLAG), b = stop mm, c = request id */
#define VA_FAULT	7U	/* a = fault bits to raise */
#define VA_RESTART	8U	/* ask Safety Monitor to clear the faults */
#define VA_TEL		9U	/* telemetry record: a = EV_..., b = p1, c = p2 */
#define VC_ACT_MAX	10U

typedef struct {
	uint8_t		type;
	int32_t		a;
	int32_t		b;
	int32_t		c;
} VC_ACTION;

typedef struct {
	uint32_t	n;
	VC_ACTION	a[VC_ACT_MAX];
} VC_ACTIONS;

/* EV_NAV.p1: navigation decisions reported to telemetry */
#define NAV_PENDING	1	/* barcode command stored: p2 = LB_CMD_... */
#define NAV_EXEC	2	/* barcode command started: p2 = LB_CMD_... */
#define NAV_CONTINUE	3	/* obstacle outside the path */
#define NAV_BYPASS	4	/* p2 = sideways shift mm, - left / + right */
#define NAV_REVERSE	5	/* p2 = attempt number */
#define NAV_REJOINED	6	/* back on the line after a bypass */
#define NAV_SEARCH	7	/* line lost: p2 = side tried first */
#define NAV_FOUND	8	/* line found again */
#define NAV_FINISH	9	/* p2 = distance of the run, mm */
#define NAV_REFUSED	10	/* command refused in this state: p2 = CMD_... */

/* EV_STATE.p2: why SAFE_STOP (1-3: safety) or HALTED (4-9: mission) was entered */
#define VC_WHY_NONE		0
#define VC_WHY_FAULT		1	/* SAFE_STOP: a fault reported by Safety Monitor */
#define VC_WHY_IMPACT		2	/* SAFE_STOP: collision */
#define VC_WHY_ACK_TIMEOUT	3	/* SAFE_STOP: the wheels did not stop in time */
#define VC_WHY_SCAN_FAILED	4	/* HALTED: no profile, even after a rescan */
#define VC_WHY_NO_ROUTE		5	/* HALTED: no side clear after every retry */
#define VC_WHY_SEARCH_FAILED	6	/* HALTED: line not found */
#define VC_WHY_BLOCKED		7	/* HALTED: obstacle during a bypass */
#define VC_WHY_EXEC_TIMEOUT	8	/* HALTED: a barcode command took too long */
#define VC_WHY_BYPASS_TIMEOUT	9	/* HALTED: the bypass took too long */

typedef struct {
	int32_t		base_speed;
	int32_t		slow_speed;
	int32_t		hump_speed;
	int32_t		junction_speed;
	int32_t		move_speed;
	int32_t		find_speed;
	int32_t		car_width_mm;
	int32_t		margin_mm;
	int32_t		us_to_axle_mm;
	int32_t		line_to_axle_mm;
	int32_t		rear_clear_mm;
	int32_t		obst_depth_mm;
	int32_t		line_half_mm;
	int32_t		stop_mm;
	int32_t		bypass_stop_mm;
	int32_t		bypass_deg;
	int32_t		back_mm;
	uint32_t	n_retry;
	int32_t		tie_mm;		/* detours closer than this count as equal */
	int32_t		pref_side;
	int32_t		find_extra_mm;
	int32_t		align_extra_mm;
	int32_t		d_exec_mm;
	int32_t		junction_mute_mm;
	int32_t		cross_mm;
	int32_t		acquire_mm;	/* after a turn, the line must be seen within this */
	uint32_t	t_startup_ms;
	uint32_t	t_ack_ms;
	uint32_t	t_scan_ms;
	uint32_t	t_search_ms;
	uint32_t	t_exec_ms;
	uint32_t	t_bypass_ms;
	uint32_t	hump_grace_ms;
	uint32_t	hold_refresh_ms;
} VC_PARAMS;

#define VC_PLAN_MAX	6U

typedef struct {
	int32_t		turn_deg;	/* + left; 0 = a drive step */
	int32_t		drive_mm;
} VC_STEP;

typedef struct {
	VC_PARAMS	p;
	LS_STATE	ls;		/* line-following rule state */
	VC_STATE	state;
	uint32_t	entered_ms;	/* when this state began */
	uint32_t	episode_ms;	/* when this exec / search / bypass began */
	int32_t		why;		/* VC_WHY_... of the last SAFE_STOP or HALTED */
	bool		faulted;	/* Safety Monitor has latched a fault */
	int32_t		fault_bits;
	bool		sensors_ready;
	/* inputs kept */
	int32_t		odo_mm;
	int32_t		terrain;	/* TERRAIN_... */
	uint32_t	hump_until_ms;
	bool		zone_near;
	/* barcode command */
	int32_t		pending;	/* LB_CMD_..., 0 = none */
	int32_t		pending_odo;
	int32_t		junction_mute_odo;
	bool		acquiring;	/* following again, line not seen yet */
	int32_t		acquire_odo;	/* ... search for it if not seen by here */
	int32_t		alert_mute_odo;
	int32_t		exec_cmd;
	int32_t		step;		/* sub-step of EXEC, SEARCH, ALIGN */
	/* motion */
	int32_t		move_id;	/* last id handed out */
	int32_t		expect_id;	/* DONE we are waiting for (0 = none) */
	bool		hold_valid;
	int32_t		hold_l;
	int32_t		hold_r;
	uint32_t	hold_ms;
	uint32_t	steer_src_ms;	/* time of the line reading being handled (0 = none) */
	/* scanning */
	bool		watch_stops;	/* the active WATCH has SCAN_STOP_FLAG */
	bool		rescanned;	/* this obstacle has been scanned a second time */
	int32_t		watch_id;
	int32_t		scan_id;
	uint32_t	retries;
	VC_STEP		plan[VC_PLAN_MAX];
	uint32_t	plan_n;
	uint32_t	plan_i;
	int32_t		side;		/* bypass side: -1 left, +1 right */
	int32_t		shift_mm;
	int32_t		search_side;
	/* statistics for telemetry and the run summary */
	uint32_t	run_start_ms;
	int32_t		run_start_odo;
	uint32_t	commands_done;
	uint32_t	obstacles_passed;
	int32_t		last_cmd;
} VC_CORE;

void	vc_init(VC_CORE *v, const VC_PARAMS *p, const LS_PARAMS *ls, uint32_t now_ms);

/* SET values copied at START; -1 (never set) keeps the configured value */
void	vc_tune(VC_CORE *v, int32_t base_speed, int32_t stop_mm);

/* An event from vc_event_q.  odo_mm: Motion Control's odometer. */
void	vc_event(VC_CORE *v, const APP_MSG *m, uint32_t now_ms, int32_t odo_mm, VC_ACTIONS *act);

/* EV_PROFILE comes with the profile itself (copied from Obstacle Scan) */
void	vc_profile(VC_CORE *v, const APP_MSG *m, const OS_PROFILE *prof, uint32_t now_ms,
		   int32_t odo_mm, VC_ACTIONS *act);

/* Every VC_TICK_MS: timers and steering.  sensors_ready: start-up
   self-tests done (IMU calibrated or not required). */
void	vc_tick(VC_CORE *v, uint32_t now_ms, int32_t odo_mm, bool sensors_ready, VC_ACTIONS *act);

const char *vc_state_name(int32_t state);
const char *vc_nav_name(int32_t nav);

#endif /* VC_CORE_H */
