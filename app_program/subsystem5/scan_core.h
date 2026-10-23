/*
 *	scan_core.h
 *	Obstacle Scan logic (Buddy 5): forward watch (Stage 0), coarse scan
 *	(Stage 1), fine scan (Stage 2) and profile (Stage 3).  Pure C.
 *
 *	The Obstacle Scan task calls scan_core_step() every 10 ms with the
 *	newest request from Vehicle Controller and the result of the last
 *	ultrasonic ping, and carries out what it returns: the servo angle,
 *	whether to fire a new ping, and events.  Every wait (servo settling,
 *	the 70 ms gap between pings, the echo) is a state that is checked
 *	again on the next 10 ms step, so nothing ever blocks.
 */

#ifndef SCAN_CORE_H
#define SCAN_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_evt.h"
#include "os_profile.h"

/* Requests (scan_request_q, from Vehicle Controller) */
#define SCAN_IDLE		0U	/* no pings, servo centred */
#define SCAN_WATCH		1U	/* face forward, ping, ALERT when close */
#define SCAN_PROFILE		2U	/* coarse + fine scan, then PROFILE */
#define SCAN_MODE_MASK		0x03U	/* 3 modes: 2 bits */
/* WATCH flag: on a confirmed close range, Obstacle Scan sets CMD_STOP
   itself (motion_stop) before it sends the ALERT, so the stop does not
   wait for vc_event_q or the Vehicle Controller (design-review feedback
   IS32).  Vehicle Controller leaves it off only while it deliberately
   drives past an obstacle it has decided to pass (Continue). */
#define SCAN_STOP_FLAG		0x10U

typedef struct {
	uint32_t	mode;		/* SCAN_..., plus SCAN_STOP_FLAG for WATCH */
	int32_t		stop_mm;	/* WATCH: come to rest this far away (0 = default) */
	int32_t		id;		/* echoed in ALERT / RANGE_ZONE / PROFILE */
} SCAN_REQ;

/* A request, packed: what goes through scan_request_q (4 + 4 B per slot,
   was 12 + 4 B).  20 bits:
     bits 0-1    mode        SCAN_IDLE / WATCH / PROFILE
     bit 2       SCAN_STOP_FLAG
     bits 3-11   stop_mm - 120, 0..480 (120..600 mm: SET stop_mm 150..600
                 and VC_BYPASS_STOP_MM 120); 511 = 0 = the default;
                 outside the range saturates
     bits 12-19  id, 0..255 (app_id_next)                                 */
#define SCAN_REQ_STOP_MIN	120
#define SCAN_REQ_STOP_DEFAULT	511U

typedef struct {
	uint32_t	w;
} SCAN_REQ_PK;

void	scan_req_pack(const SCAN_REQ *r, SCAN_REQ_PK *pk);
void	scan_req_unpack(const SCAN_REQ_PK *pk, SCAN_REQ *r);

/* Latest Obstacle Scan state, for telemetry.  Written only by the Obstacle
   Scan task, read by Telemetry: shared as ONE packed word, one 32-bit
   store and one load, so no lock is needed (30 bits):
     bits 0-1    mode      SCAN_IDLE / WATCH / PROFILE (0..2)
     bits 2-13   range mm  12 bits, OS_INVALID (4,095) = none
     bits 14-29  pings     low 16 bits: a counter, only ever subtracted */
typedef struct {
	uint32_t	mode;		/* SCAN_..., without SCAN_STOP_FLAG */
	int32_t		range_mm;	/* last forward range */
	uint32_t	pings;
} SCAN_ST;

uint32_t scan_st_pack(const SCAN_ST *s);
void	scan_st_unpack(uint32_t w, SCAN_ST *s);

/* Echo width (us) -> range: 12-bit unsigned mm, 0..4,000; anything
   farther (an echo over ~23.3 ms, or a saturated 15-bit width) is
   OS_INVALID (4,095) = no echo, which is unknown and never free space. */
#define SCAN_RANGE_MAX_MM	4000
int32_t	scan_range_mm(uint32_t width_us);

/* Result of the ping in flight, from the ultrasonic driver */
#define SCAN_ECHO_NONE		0	/* nothing new */
#define SCAN_ECHO_DONE		1	/* range_mm is valid */
#define SCAN_ECHO_FAR		2	/* ECHO rose but no echo came back: unknown */
#define SCAN_ECHO_MISSING	3	/* ECHO never rose: sensor not answering */

/* WATCH is speed-aware: the ALERT fires at stop_mm PLUS the distance the
   car still covers from that reading (the confirming pings at its speed,
   then its own stopping distance, speed^2 / 2 x the deceleration Motion
   Control measured on its real stops), so the car comes to REST about
   stop_mm from the obstacle whatever its speed. */
typedef struct {
	int32_t		stop_mm;	/* default distance to come to rest at */
	int32_t		slow_extra_mm;	/* NEAR (slow down) this much before the ALERT point */
	int32_t		slow_hyst_mm;	/* FAR again beyond NEAR + this */
	int32_t		decel_guess;	/* mm/s^2 until Motion Control has measured a stop */
	uint32_t	confirm;	/* close readings in a row for ALERT */
	uint32_t	gap_ms;		/* between triggers (datasheet: > 60 ms) */
	uint32_t	settle_base_ms;	/* servo settling: base ... */
	uint32_t	settle_ms_per_deg;	/* ... + this per degree moved */
	uint32_t	retries;	/* extra pings when there is no echo */
	int32_t		r_max_mm;	/* farther = background */
	int32_t		corridor_mm;	/* half-width of the car's path (half width + margin) */
	int32_t		coarse_step;	/* deg, 30..150 */
	int32_t		fine_step;	/* deg */
	int32_t		fine_margin;	/* deg added either side of the obstacle */
	uint32_t	missing_limit;	/* "sensor not answering" pings in a row = fault */
	uint32_t	profile_max_ms;	/* a whole profile must finish within this */
} SCAN_PARAMS;

typedef struct {
	uint32_t	now_ms;
	bool		have_req;	/* a new request arrived */
	SCAN_REQ	req;
	int32_t		echo;		/* SCAN_ECHO_... */
	int32_t		range_mm;	/* 0..4,000 mm, OS_INVALID = none */
	bool		echo_high;	/* ECHO still high from the last ping */
	int32_t		speed_mm_s;	/* the car's measured speed (+ forward) */
	int32_t		stop_decel_mm_s2;	/* its learned stopping rate, 0 = not yet */
} SCAN_IN;

typedef struct {
	int32_t		servo_deg;	/* command the servo to this angle */
	bool		trigger;	/* fire a ping now */
	bool		fault;		/* sensor not answering (the car is blind) */
	bool		stop_motors;	/* confirmed ALERT with SCAN_STOP_FLAG: set CMD_STOP now */
	bool		profile_ready;	/* scan_core.profile holds a new result */
	EVT_LIST	ev;		/* EV_ALERT, EV_RANGE_ZONE, EV_PROFILE */
} SCAN_OUT;

#define SCAN_SLOTS	25U	/* 30..150 degrees in 5 degree slots */

typedef struct {
	SCAN_PARAMS	p;
	SCAN_REQ	req;		/* active request */
	int32_t		stop_mm;
	int32_t		servo_deg;
	uint32_t	settle_until_ms;
	bool		in_flight;	/* a ping is waiting for its echo */
	uint32_t	last_trig_ms;
	uint32_t	missing;
	/* watch */
	uint32_t	close_n;
	bool		alert_sent;
	int32_t		zone_near;	/* -1 unknown, 0 far, 1 near */
	int32_t		last_range;	/* mm, OS_INVALID (4,095) = none */
	/* profile */
	int32_t		phase;
	int32_t		angles[SCAN_SLOTS];
	uint32_t	n_angles;
	uint32_t	idx;
	uint32_t	tries;
	int32_t		range_at[SCAN_SLOTS];
	bool		sampled[SCAN_SLOTS];
	uint32_t	start_ms;
	OS_PROFILE	profile;	/* last result */
	uint32_t	pings;		/* statistics */
} SCAN_CORE;

void	scan_core_init(SCAN_CORE *s, const SCAN_PARAMS *p);
void	scan_core_step(SCAN_CORE *s, const SCAN_IN *in, SCAN_OUT *out);

#endif /* SCAN_CORE_H */
