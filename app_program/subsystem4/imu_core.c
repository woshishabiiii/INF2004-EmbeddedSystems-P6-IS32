/*
 *	imu_core.c
 *	IMU and Terrain logic (see imu_core.h).
 *
 *	Every 10 ms:
 *	  1. calibrate (first cal_samples samples, car at rest and level);
 *	     then, whenever the car stands still waiting for START, keep
 *	     re-learning "level" from the floor it stands on;
 *	  2. impact check on the raw horizontal acceleration;
 *	  3. low-pass filter; remove the car's own acceleration;
 *	  4. pitch relative to the mounting, terrain state with hysteresis;
 *	  5. hump: accumulate height, report it when the car is level again;
 *	  6. motion class, reported when it has been stable for class_hold.
 */

#include "imu_core.h"
#include "imu_math.h"
#include "../bitpack.h"

#define IMU_Q			16	/* filters keep 1/16 mg */
#define MG_PER_MM_S2_X1000	102	/* 1 mm/s^2 = 0.102 mg */
#define ACC_LPF_DIV		4	/* encoder acceleration filter */
#define DS_MAX_MM		100	/* larger jumps of the odometer are glitches */
#define IMPACT_HOLD_MS		500U	/* class stays IMPACT this long */
#define RELEVEL_DIV		16	/* re-levelling: 1/16 of the difference per sample */

static int32_t iabs32(int32_t x)
{
	return (x < 0) ? -x : x;
}

void imu_core_init(IMU_CORE *c, const IMU_PARAMS *p)
{
	*c = (IMU_CORE){0};
	c->p = *p;
	if (c->p.pre_samples > IMU_PRE_MAX) {
		c->p.pre_samples = IMU_PRE_MAX;
	}
	if (c->p.pre_samples == 0U) {
		c->p.pre_samples = 1U;
	}
	c->terrain = TERRAIN_LEVEL;
	c->motion = MOTION_STATIONARY;
	c->candidate = MOTION_STATIONARY;
}

static void clear_pre(IMU_CORE *c)
{
	uint32_t i;

	for (i = 0U; i < IMU_PRE_MAX; i++) {
		c->pre[i] = 0;
	}
	c->pre_i = 0U;
	c->pre_sum = 0;
}

static void calibrate(IMU_CORE *c, int32_t rf, int32_t ru, int32_t rl)
{
	c->cal_sum[0] += rf;
	c->cal_sum[1] += ru;
	c->cal_sum[2] += rl;
	c->cal_n++;
	if (c->cal_n >= c->p.cal_samples) {
		int32_t n = (int32_t)c->cal_n;
		int32_t l0;

		c->f0 = c->cal_sum[0] / n;
		c->u0 = c->cal_sum[1] / n;
		l0 = c->cal_sum[2] / n;
		c->g0 = (int32_t)imu_isqrt((uint32_t)((c->f0 * c->f0) + (c->u0 * c->u0) + (l0 * l0)));
		c->base_cdeg = imu_atan2_cdeg(c->f0, c->u0);	/* mounting tilt */
		c->f = c->f0 * IMU_Q;
		c->u = c->u0 * IMU_Q;
		c->l = l0 * IMU_Q;
		c->calibrated = true;
	}
}

static void check_impact(IMU_CORE *c, int32_t rf, int32_t rl, const IMU_MOTION *m,
			 uint32_t now_ms, IMU_OUT *out)
{
	int32_t jump_f = iabs32(rf - (c->f / IMU_Q));	/* jolt against the filtered value */
	int32_t jump_l = iabs32(rl - (c->l / IMU_Q));
	int32_t jump = (jump_f > jump_l) ? jump_f : jump_l;
	int32_t speed = m->valid ? iabs32(m->speed_mm_s) : 0;

	if (c->in_hump || (c->terrain != TERRAIN_LEVEL)) {
		c->impact_pending = false;	/* hump edges jolt the car too */
		return;
	}
	if (!c->impact_pending && (jump > c->p.impact_mg)) {
		if (speed >= c->p.impact_min_speed) {
			c->impact_pending = true;
			c->impact_t_ms = now_ms;
			c->impact_speed = speed;
			c->impact_jump = jump;
		} else {
			c->bumps++;		/* a knock while (nearly) still */
		}
	}
	if (c->impact_pending) {
		if (m->valid && ((speed * 100) <= (c->impact_speed * c->p.impact_drop_pct))) {
			/* jolt AND the wheels slowed sharply: a collision */
			(void)evt_list_add(&out->ev, (uint8_t)EV_IMPACT, c->impact_jump, c->impact_speed);
			out->impact = true;
			c->impact_pending = false;
			c->impact_until_ms = now_ms + IMPACT_HOLD_MS;
		} else if ((now_ms - c->impact_t_ms) > c->p.impact_window_ms) {
			c->impact_pending = false;
			c->bumps++;		/* the car kept going: just a bump */
		} else {
			/* still deciding */
		}
	}
}

static void finish_hump(IMU_CORE *c, IMU_OUT *out)
{
	int32_t est_int = c->peak_um / 1000;
	int32_t est_wb = (c->p.wheelbase_mm * imu_sin_x1e4(c->pmax_cdeg)) / 10000;
	int32_t h = (est_int > est_wb) ? est_int : est_wb;

	if (h >= c->p.hump_min_mm) {
		c->last_hump_mm = h;
		if (h > c->max_hump_mm) {
			c->max_hump_mm = h;
		}
		c->humps++;
		(void)evt_list_add(&out->ev, (uint8_t)EV_HUMP, h, c->max_hump_mm);
	}
	c->in_hump = false;
	c->h_um = 0;
	c->peak_um = 0;
	c->pmax_cdeg = 0;
	clear_pre(c);
}

static void terrain_and_hump(IMU_CORE *c, int32_t dh_um, int32_t ds_mm, bool moving,
			     uint32_t now_ms, IMU_OUT *out)
{
	int32_t	p = c->pitch_cdeg;
	int32_t	ap = iabs32(p);
	int32_t	t = c->terrain;
	int32_t	dir = (p > c->p.climb_cdeg) ? 1 : ((p < -c->p.climb_cdeg) ? -1 : 0);

	/* A slope only counts once the car has TRAVELLED terrain_mm while
	   tilted that way: braking, starting or a knock tilts the reading
	   for a moment without the car going anywhere. */
	if ((dir != 0) && (dir == c->tilt_dir)) {
		c->tilt_mm += iabs32(ds_mm);
	} else {
		c->tilt_dir = dir;
		c->tilt_mm = 0;
	}
	if ((dir != 0) && (c->tilt_mm >= c->p.terrain_mm)) {
		t = (dir > 0) ? TERRAIN_CLIMBING : TERRAIN_DESCENDING;
	} else if ((t != TERRAIN_LEVEL) && (ap < c->p.level_cdeg)) {
		t = TERRAIN_LEVEL;		/* hysteresis: level again below level_cdeg */
	} else {
		/* no change */
	}
	if (t != c->terrain) {
		c->terrain = t;
		(void)evt_list_add(&out->ev, (uint8_t)EV_TERRAIN, t, p);
		if (t == TERRAIN_LEVEL) {
			c->level_since_ms = now_ms;
			c->level_mm = 0;
		}
	}
	c->level_mm += iabs32(ds_mm);

	/* hump */
	if (!c->in_hump && (t != TERRAIN_LEVEL)) {
		c->in_hump = true;
		c->h_um = (c->pre_sum > 0) ? c->pre_sum : 0;	/* rise before the threshold */
		c->peak_um = c->h_um;
		c->pmax_cdeg = moving ? ap : 0;
	}
	if (c->in_hump) {
		c->h_um += dh_um;
		if (c->h_um > c->peak_um) {
			c->peak_um = c->h_um;
		}
		if (moving && (ap > c->pmax_cdeg)) {
			c->pmax_cdeg = ap;		/* not while braking to a stop */
		}
		if ((t == TERRAIN_LEVEL) && ((c->level_mm >= c->p.hump_end_mm) ||
					     ((now_ms - c->level_since_ms) >= c->p.hump_end_ms))) {
			finish_hump(c, out);
		}
	} else {
		c->pre_sum += dh_um - c->pre[c->pre_i];
		c->pre[c->pre_i] = dh_um;
		c->pre_i = (c->pre_i + 1U) % c->p.pre_samples;
	}
}

/* Not on a run.  Slopes and humps are things the car DRIVES over: while
   it is carried, put down or waiting, none are measured (and any hump in
   progress is dropped, not reported).  While it stands still waiting for
   START, "level" is re-learned from the floor it stands on, so a car
   that was tilted during the power-up calibration (picked up, cable
   pulling) is level again before it starts. */
static void not_running(IMU_CORE *c, const IMU_MOTION *m, IMU_OUT *out)
{
	if (c->terrain != TERRAIN_LEVEL) {
		c->terrain = TERRAIN_LEVEL;
		(void)evt_list_add(&out->ev, (uint8_t)EV_TERRAIN, TERRAIN_LEVEL, c->pitch_cdeg);
	}
	c->in_hump = false;
	c->h_um = 0;
	c->peak_um = 0;
	c->pmax_cdeg = 0;
	c->tilt_mm = 0;
	c->tilt_dir = 0;
	clear_pre(c);
	if (m->waiting && (c->motion == MOTION_STATIONARY)) {
		int32_t level = imu_atan2_cdeg(c->f, c->u);	/* still: no acceleration to remove */

		c->base_cdeg += (level - c->base_cdeg) / RELEVEL_DIV;
	}
}

static void classify(IMU_CORE *c, int32_t rf, int32_t rl, const IMU_MOTION *m,
		     uint32_t now_ms, IMU_OUT *out)
{
	int32_t cls;
	int32_t vib = iabs32(rf - (c->f / IMU_Q)) + iabs32(rl - (c->l / IMU_Q));

	if ((int32_t)(c->impact_until_ms - now_ms) > 0) {
		cls = MOTION_IMPACT;
	} else if (c->terrain == TERRAIN_CLIMBING) {
		cls = MOTION_CLIMBING;
	} else if (c->terrain == TERRAIN_DESCENDING) {
		cls = MOTION_DESCENDING;
	} else if (m->valid && (iabs32(m->yaw_cdeg_s) > c->p.turn_cdeg_s)) {
		cls = MOTION_TURNING;
	} else if (m->valid && (c->acc_f > c->p.accel_mm_s2)) {
		cls = MOTION_ACCELERATING;
	} else if (m->valid && (c->acc_f < -c->p.accel_mm_s2)) {
		cls = MOTION_DECELERATING;
	} else if ((!m->valid || (iabs32(m->speed_mm_s) < c->p.still_speed)) &&
		   (vib < c->p.still_dev_mg)) {
		cls = MOTION_STATIONARY;
	} else {
		cls = MOTION_MOVING;
	}

	if (cls != c->candidate) {
		c->candidate = cls;
		c->cand_n = 0U;
	}
	c->cand_n++;
	if ((cls != c->motion) && ((c->cand_n >= c->p.class_hold) || (cls == MOTION_IMPACT))) {
		c->motion = cls;
		(void)evt_list_add(&out->ev, (uint8_t)EV_MOTION, cls, 0);
	}
}

void imu_core_step(IMU_CORE *c, const int32_t a_mg[3], bool valid,
		   const IMU_MOTION *m, uint32_t now_ms, IMU_OUT *out)
{
	int32_t rf, ru, rl, af, au, sin_x1e4, ds, dh_um = 0;

	evt_list_clear(&out->ev);
	out->fault = false;
	out->impact = false;

	if (!valid) {
		c->fails++;
		out->fault = (c->fails >= c->p.fail_limit);
		return;
	}
	c->fails = 0U;
	rf = a_mg[c->p.axis_fwd] * c->p.sign_fwd;
	ru = a_mg[c->p.axis_up] * c->p.sign_up;
	rl = a_mg[c->p.axis_lat] * c->p.sign_lat;

	/* 1. calibration */
	if (!c->calibrated) {
		calibrate(c, rf, ru, rl);
		return;
	}

	/* 2. impact, against the filter state BEFORE this sample */
	check_impact(c, rf, rl, m, now_ms, out);

	/* 3. filters; forward acceleration of the car removed */
	c->f += ((rf * IMU_Q) - c->f) / (1 << c->p.lpf_shift);
	c->u += ((ru * IMU_Q) - c->u) / (1 << c->p.lpf_shift);
	c->l += ((rl * IMU_Q) - c->l) / (1 << c->p.lpf_shift);
	if (m->valid) {
		c->acc_f += (m->accel_mm_s2 - c->acc_f) / ACC_LPF_DIV;
	}
	af = c->f - (((c->acc_f * MG_PER_MM_S2_X1000) / 1000) * IMU_Q);
	au = c->u;

	/* 4. pitch relative to how the board is mounted */
	c->pitch_cdeg = imu_atan2_cdeg(af, au) - c->base_cdeg;
	sin_x1e4 = imu_sin_x1e4(c->pitch_cdeg);

	/* 5. height change over the encoder travel since the last sample */
	ds = 0;
	if (m->valid) {
		if (c->have_dist && (iabs32(m->dist_mm - c->prev_dist) < DS_MAX_MM)) {
			ds = m->dist_mm - c->prev_dist;
			dh_um = (ds * sin_x1e4) / 10;	/* mm x 1000 um/mm x sin / 10 000 */
		}
		c->prev_dist = m->dist_mm;
		c->have_dist = true;
	}
	if (m->running) {
		terrain_and_hump(c, dh_um, ds, m->valid && (iabs32(m->speed_mm_s) >= c->p.still_speed),
				 now_ms, out);
	} else {
		not_running(c, m, out);
	}

	/* 6. motion class */
	classify(c, rf, rl, m, now_ms, out);
}

/* ---- packed status (imu_core.h) ----------------------------------------- */
#define ICS_PITCH_MIN	(-8192)
#define ICS_HEAD_MIN	(-32768)

void ic_status_pack(const IC_STATUS *s, IC_STATUS_PK *pk)
{
	pk->w[0] = (s->calibrated ? 1U : 0U) |
		   (s->ok ? 2U : 0U) |
		   (bp_sat_u(s->terrain, 2U) << 2) |
		   (bp_sat_u(s->motion, 3U) << 4) |
		   (bp_sat_s(s->pitch_cdeg, ICS_PITCH_MIN, 14U) << 7) |
		   (bp_sat_u(s->max_hump_mm, 8U) << 21);
	pk->w[1] = bp_sat_u(s->last_hump_mm, 8U) |
		   (bp_sat_u((int32_t)s->humps, 8U) << 8) |
		   (bp_sat_u(s->g0_mg, 12U) << 16);
	pk->w[2] = bp_sat_s(s->heading_cdeg, ICS_HEAD_MIN, 16U);
}

void ic_status_unpack(const IC_STATUS_PK *pk, IC_STATUS *s)
{
	uint32_t w0 = pk->w[0];
	uint32_t w1 = pk->w[1];

	s->calibrated = ((w0 & 1U) != 0U);
	s->ok = ((w0 & 2U) != 0U);
	s->terrain = (int32_t)((w0 >> 2) & BP_MAX(2U));
	s->motion = (int32_t)((w0 >> 4) & BP_MAX(3U));
	s->pitch_cdeg = bp_from_s((w0 >> 7) & BP_MAX(14U), ICS_PITCH_MIN);
	s->max_hump_mm = (int32_t)((w0 >> 21) & BP_MAX(8U));
	s->last_hump_mm = (int32_t)(w1 & BP_MAX(8U));
	s->humps = (w1 >> 8) & BP_MAX(8U);
	s->g0_mg = (int32_t)((w1 >> 16) & BP_MAX(12U));
	s->heading_cdeg = bp_from_s(pk->w[2] & BP_MAX(16U), ICS_HEAD_MIN);
}
