/*
 *	scan_core.c
 *	Obstacle Scan logic (see scan_core.h).
 */

#include "scan_core.h"
#include "../bitpack.h"

/* A scan gives at most SCAN_SLOTS points: the profile's packed point
   counts (os_profile.h) must hold that many */
BP_STATIC_ASSERT(SCAN_SLOTS <= BP_MAX(OS_PK_COUNT_BITS), scan_points_fit_profile);

#define SERVO_CENTRE	90
#define SERVO_MIN	30
#define SERVO_MAX	150
#define SLOT_DEG	5
#define CLUSTER_MM	150	/* coarse points this close to the nearest = same obstacle */

#define STAGE_COARSE	1
#define STAGE_FINE	2

#define PH_MOVE		0
#define PH_PING		1
#define PH_WAIT		2

#define MS_PER_S	1000

static int32_t iabs32(int32_t x)
{
	return (x < 0) ? -x : x;
}

void scan_core_init(SCAN_CORE *s, const SCAN_PARAMS *p)
{
	*s = (SCAN_CORE){0};
	s->p = *p;
	s->req.mode = SCAN_IDLE;
	s->stop_mm = p->stop_mm;
	s->servo_deg = SERVO_CENTRE;
	s->zone_near = -1;
	s->last_range = OS_INVALID;
}

static void set_servo(SCAN_CORE *s, int32_t deg, uint32_t now_ms)
{
	if (deg != s->servo_deg) {
		s->settle_until_ms = now_ms + s->p.settle_base_ms +
				     (s->p.settle_ms_per_deg * (uint32_t)iabs32(deg - s->servo_deg));
		s->servo_deg = deg;
	}
}

static bool can_trigger(const SCAN_CORE *s, const SCAN_IN *in)
{
	return !s->in_flight && !in->echo_high &&
	       ((in->now_ms - s->last_trig_ms) >= s->p.gap_ms) &&
	       ((int32_t)(in->now_ms - s->settle_until_ms) >= 0);
}

static void fire(SCAN_CORE *s, const SCAN_IN *in, SCAN_OUT *out)
{
	out->trigger = true;
	s->in_flight = true;
	s->last_trig_ms = in->now_ms;
	s->pings++;
}

/* The ping in flight has finished?  Returns its SCAN_ECHO_... or NONE. */
static int32_t take_echo(SCAN_CORE *s, const SCAN_IN *in, SCAN_OUT *out)
{
	if (!s->in_flight || (in->echo == SCAN_ECHO_NONE)) {
		return SCAN_ECHO_NONE;
	}
	s->in_flight = false;
	if (in->echo == SCAN_ECHO_MISSING) {
		s->missing++;
		if (s->missing >= s->p.missing_limit) {
			out->fault = true;		/* the sensor is not answering */
		}
	} else {
		s->missing = 0U;
	}
	return in->echo;
}

/* ---- Stage 0: watch forward ---------------------------------------------- */
/* Distance the car still covers after the reading that starts an ALERT:
   the confirming pings (confirm x gap at its speed), then its own
   stopping distance (speed^2 / 2 x the deceleration it measured on its
   own stops).  Standing still or reversing: 0. */
static int32_t approach_mm(const SCAN_CORE *s, const SCAN_IN *in)
{
	int32_t v = (in->speed_mm_s > 0) ? in->speed_mm_s : 0;
	int32_t a = (in->stop_decel_mm_s2 > 0) ? in->stop_decel_mm_s2 : s->p.decel_guess;

	if (a <= 0) {
		a = 1;
	}
	return ((v * (int32_t)(s->p.confirm * s->p.gap_ms)) / MS_PER_S) + ((v * v) / (2 * a));
}

static void watch_step(SCAN_CORE *s, const SCAN_IN *in, int32_t echo, SCAN_OUT *out)
{
	set_servo(s, SERVO_CENTRE, in->now_ms);

	if (echo == SCAN_ECHO_DONE) {
		int32_t r = in->range_mm;
		int32_t alert_at = s->stop_mm + approach_mm(s, in);
		int32_t near_at = alert_at + s->p.slow_extra_mm;

		s->last_range = r;
		if ((r < near_at) && (s->zone_near != 1)) {
			s->zone_near = 1;
			(void)evt_list_add(&out->ev, (uint8_t)EV_RANGE_ZONE, 1, s->req.id);
		} else if ((r > (near_at + s->p.slow_hyst_mm)) && (s->zone_near != 0)) {
			s->zone_near = 0;
			(void)evt_list_add(&out->ev, (uint8_t)EV_RANGE_ZONE, 0, s->req.id);
		} else {
			/* zone unchanged */
		}
		s->close_n = (r < alert_at) ? (s->close_n + 1U) : 0U;
		if ((s->close_n >= s->p.confirm) && !s->alert_sent) {
			(void)evt_list_add(&out->ev, (uint8_t)EV_ALERT, r, s->req.id);
			s->alert_sent = true;
			out->stop_motors = ((s->req.mode & SCAN_STOP_FLAG) != 0U);
		}
	} else if (echo != SCAN_ECHO_NONE) {
		s->last_range = OS_INVALID;	/* no echo: nothing confirmed close */
		s->close_n = 0U;
	} else {
		/* no result this step */
	}
	if (can_trigger(s, in)) {
		fire(s, in, out);
	}
}

/* ---- Stages 1-3: coarse scan, fine scan, profile ------------------------- */
static void start_profile(SCAN_CORE *s, uint32_t now_ms)
{
	int32_t a;
	uint32_t i;

	for (i = 0U; i < SCAN_SLOTS; i++) {
		s->sampled[i] = false;
		s->range_at[i] = OS_INVALID;
	}
	s->n_angles = 0U;
	for (a = SERVO_MIN; a <= SERVO_MAX; a += s->p.coarse_step) {
		s->angles[s->n_angles] = a;
		s->n_angles++;
	}
	s->phase = STAGE_COARSE * 10 + PH_MOVE;
	s->idx = 0U;
	s->start_ms = now_ms;
}

static void plan_fine(SCAN_CORE *s)
{
	int32_t		rmin = INT32_MAX, amin = SERVO_CENTRE, amax = SERVO_CENTRE;
	int32_t		lo, hi, a;
	bool		any = false;
	uint32_t	i;

	/* nearest coarse hit inside the car's path; anything nearer beside
	   the path (a wall) must not pull the fine scan away from it */
	for (i = 0U; i < SCAN_SLOTS; i++) {
		int32_t r = s->range_at[i];
		int32_t x;

		a = SERVO_MIN + ((int32_t)i * SLOT_DEG);
		x = (r * os_cos1000(a)) / 1000;
		if (s->sampled[i] && (r != OS_INVALID) && (r <= s->p.r_max_mm) && (r < rmin) &&
		    (x <= s->p.corridor_mm) && (x >= -s->p.corridor_mm)) {
			rmin = r;
		}
	}
	for (i = 0U; (i < SCAN_SLOTS) && (rmin == INT32_MAX); i++) {
		int32_t r = s->range_at[i];

		if (s->sampled[i] && (r != OS_INVALID) && (r <= s->p.r_max_mm)) {
			rmin = r;		/* nothing in the path: take the nearest anywhere */
		}
	}
	for (i = 0U; i < SCAN_SLOTS; i++) {
		int32_t r = s->range_at[i];

		if (s->sampled[i] && (r != OS_INVALID) && (r <= (rmin + CLUSTER_MM)) &&
		    (r <= s->p.r_max_mm)) {
			a = SERVO_MIN + ((int32_t)i * SLOT_DEG);
			if (!any) {
				amin = a;
				any = true;
			}
			amax = a;
		}
	}
	/* edges of the obstacle lie within fine_margin of its outer coarse
	   points; if nothing was seen, look again around straight ahead */
	lo = amin - s->p.fine_margin;
	hi = amax + s->p.fine_margin;
	lo = (lo < SERVO_MIN) ? SERVO_MIN : (lo - (lo % SLOT_DEG));
	hi = (hi > SERVO_MAX) ? SERVO_MAX : (hi - (hi % SLOT_DEG));

	s->n_angles = 0U;
	for (a = hi; (a >= lo) && (s->n_angles < SCAN_SLOTS); a -= s->p.fine_step) {
		if (!s->sampled[(a - SERVO_MIN) / SLOT_DEG]) {
			s->angles[s->n_angles] = a;	/* sweep back down from 150 */
			s->n_angles++;
		}
	}
	s->idx = 0U;
	s->phase = STAGE_FINE * 10 + PH_MOVE;
}

static void finish_profile(SCAN_CORE *s, SCAN_OUT *out, bool timed_out)
{
	int32_t		ang[SCAN_SLOTS], rng[SCAN_SLOTS];
	uint32_t	i, n = 0U;

	for (i = 0U; i < SCAN_SLOTS; i++) {
		if (s->sampled[i]) {
			ang[n] = SERVO_MIN + ((int32_t)i * SLOT_DEG);
			rng[n] = s->range_at[i];
			n++;
		}
	}
	os_build_profile(ang, rng, n, s->p.r_max_mm, s->p.corridor_mm, &s->profile);
	if (timed_out) {
		/* not a fault: an invalid profile makes Vehicle Controller back
		   off and scan again (a retryable mission failure, IS32) */
		s->profile.valid = false;
	}
	out->profile_ready = true;
	(void)evt_list_add(&out->ev, (uint8_t)EV_PROFILE, s->profile.valid ? 1 : 0, s->req.id);
	s->req.mode = SCAN_IDLE;		/* Vehicle Controller decides what next */
}

static void record(SCAN_CORE *s, int32_t range)
{
	int32_t slot = (s->angles[s->idx] - SERVO_MIN) / SLOT_DEG;

	if ((slot >= 0) && (slot < (int32_t)SCAN_SLOTS)) {
		s->range_at[slot] = range;
		s->sampled[slot] = true;
	}
	s->idx++;
	s->tries = 0U;
}

static void profile_step(SCAN_CORE *s, const SCAN_IN *in, int32_t echo, SCAN_OUT *out)
{
	int32_t stage = s->phase / 10;
	int32_t ph = s->phase % 10;

	if ((in->now_ms - s->start_ms) > s->p.profile_max_ms) {
		finish_profile(s, out, true);
		set_servo(s, SERVO_CENTRE, in->now_ms);
		return;
	}
	if (ph == PH_WAIT) {
		if (echo == SCAN_ECHO_DONE) {
			record(s, in->range_mm);
			ph = PH_MOVE;
		} else if (echo != SCAN_ECHO_NONE) {
			s->tries++;
			if (s->tries > s->p.retries) {
				record(s, OS_INVALID);		/* unknown, NOT free */
				ph = PH_MOVE;
			} else {
				ph = PH_PING;			/* try again */
			}
		} else {
			/* still waiting for the echo */
		}
	}
	if (ph == PH_MOVE) {
		if (s->idx >= s->n_angles) {
			if (stage == STAGE_COARSE) {
				plan_fine(s);
				stage = STAGE_FINE;
			}
			if ((stage == STAGE_FINE) && (s->idx >= s->n_angles)) {
				finish_profile(s, out, false);
				set_servo(s, SERVO_CENTRE, in->now_ms);
				return;
			}
		}
		set_servo(s, s->angles[s->idx], in->now_ms);
		ph = PH_PING;
	}
	if ((ph == PH_PING) && can_trigger(s, in)) {
		fire(s, in, out);
		ph = PH_WAIT;
	}
	s->phase = (stage * 10) + ph;
}

void scan_core_step(SCAN_CORE *s, const SCAN_IN *in, SCAN_OUT *out)
{
	int32_t echo;

	out->trigger = false;
	out->fault = false;
	out->stop_motors = false;
	out->profile_ready = false;
	evt_list_clear(&out->ev);

	echo = take_echo(s, in, out);		/* finish the ping in flight first */

	if (in->have_req) {
		s->req = in->req;
		s->stop_mm = (in->req.stop_mm > 0) ? in->req.stop_mm : s->p.stop_mm;
		s->close_n = 0U;
		s->alert_sent = false;
		s->zone_near = -1;
		echo = SCAN_ECHO_NONE;		/* that ping belonged to the old request */
		if ((s->req.mode & SCAN_MODE_MASK) == SCAN_PROFILE) {
			start_profile(s, in->now_ms);
		}
	}

	switch (s->req.mode & SCAN_MODE_MASK) {
	case SCAN_WATCH:
		watch_step(s, in, echo, out);
		break;
	case SCAN_PROFILE:
		profile_step(s, in, echo, out);
		break;
	default:
		set_servo(s, SERVO_CENTRE, in->now_ms);	/* idle: centred, silent */
		break;
	}
	out->servo_deg = s->servo_deg;
}

/* ---- packed request (scan_core.h) ---------------------------------------- */
#define SRQ_FLAG_BIT	0x04U
#define SRQ_STOP_POS	3U
#define SRQ_STOP_BITS	9U
#define SRQ_ID_POS	12U

void scan_req_pack(const SCAN_REQ *r, SCAN_REQ_PK *pk)
{
	uint32_t stop;

	if (r->stop_mm == 0) {
		stop = SCAN_REQ_STOP_DEFAULT;
	} else {
		stop = bp_sat_s(r->stop_mm, SCAN_REQ_STOP_MIN, SRQ_STOP_BITS);
		if (stop == SCAN_REQ_STOP_DEFAULT) {
			stop--;			/* a huge stop_mm saturates to 631, not "default" */
		}
	}
	pk->w = (r->mode & SCAN_MODE_MASK) |
		(((r->mode & SCAN_STOP_FLAG) != 0U) ? SRQ_FLAG_BIT : 0U) |
		(stop << SRQ_STOP_POS) |
		(bp_sat_u(r->id, 8U) << SRQ_ID_POS);
}

void scan_req_unpack(const SCAN_REQ_PK *pk, SCAN_REQ *r)
{
	uint32_t stop = (pk->w >> SRQ_STOP_POS) & BP_MAX(SRQ_STOP_BITS);

	r->mode = (pk->w & SCAN_MODE_MASK) | (((pk->w & SRQ_FLAG_BIT) != 0U) ? SCAN_STOP_FLAG : 0U);
	r->stop_mm = (stop == SCAN_REQ_STOP_DEFAULT) ? 0 : bp_from_s(stop, SCAN_REQ_STOP_MIN);
	r->id = (int32_t)((pk->w >> SRQ_ID_POS) & BP_MAX(8U));
}

uint32_t scan_st_pack(const SCAN_ST *s)
{
	return bp_sat_u((int32_t)s->mode, 2U) |
	       (bp_sat_u(s->range_mm, 12U) << 2) |
	       ((s->pings & 0xFFFFU) << 14);
}

void scan_st_unpack(uint32_t w, SCAN_ST *s)
{
	s->mode = w & 3U;
	s->range_mm = (int32_t)((w >> 2) & BP_MAX(12U));
	s->pings = (w >> 14) & 0xFFFFU;
}

/* ---- echo width -> 12-bit range (scan_core.h) ---------------------------- */
#define SCAN_MM_PER_US_X10000	1715U	/* 0.343 mm/us there and back: 0.1715 mm per us */

int32_t scan_range_mm(uint32_t width_us)
{
	uint32_t mm;

	if (width_us > 30000U) {
		return OS_INVALID;		/* over 5 m; also keeps the product in 32 bits */
	}
	mm = (width_us * SCAN_MM_PER_US_X10000) / 10000U;
	return (mm > (uint32_t)SCAN_RANGE_MAX_MM) ? OS_INVALID : (int32_t)mm;
}
