/*
 *	sim_world.c
 *	World geometry, car physics and sensor models for the host simulator.
 */

#include <math.h>
#include <stdlib.h>
#include "sim_world.h"
#include "../../app_program/subsystem3/code39.h"

#define PI		3.14159265358979
#define G_MM_S2		9806.65
#define LINE_HALF	9.0
#define BAR_GAP		20.0	/* line edge to barcode */
#define BAR_HEIGHT	40.0	/* bar length across the travel direction */
#define NARROW_MM	3.0
#define WIDE_MM		9.0

/* "true" motor model: different from the firmware's assumptions on purpose */
double g_db_true = 230.0;	/* permille before the wheel moves */
double g_kff_true = 1.70;	/* permille per mm/s */
double g_track_true = TRUE_TRACK_MM;
double g_counts_mm = TRUE_COUNTS_MM;
double g_right_eff = 0.94;
#define DB_TRUE		g_db_true
#define KFF_TRUE	g_kff_true
#define TAU_S		0.060	/* motor time constant */
#define TAU_BRAKE_S	0.025
#define TAU_COAST_S	0.120	/* free-wheeling gear motor: friction stops it */
#define RIGHT_EFF	g_right_eff	/* right motor 6 % weaker */
#define GRAV_LOAD	700.0	/* mm/s lost per unit sin(pitch) when climbing */
#define SERVO_DEG_S	400.0	/* SG90: about 0.15 s per 60 degrees */
#define IMU_TILT_DEG	1.5	/* the IMU board is mounted slightly nose-up */
#define SENSOR_GAP_MM	3.0	/* IR sensors above a flat floor */
#define SENSOR_MAX_MM	12.0	/* farther than this: no reflection = reads black */
#define US_MAX_MM	4000.0
#define US_SPECULAR_DEG	65.0	/* steeper than this: the echo misses the sensor */

static double rnd_normal(void)
{
	double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
	double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);

	return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
}

void w_init(WORLD *w)
{
	w->nseg = 0;
	w->nbar = 0;
	w->nhump = 0;
	w->nbox = 0;
}

void w_line(WORLD *w, double x1, double y1, double x2, double y2)
{
	if (w->nseg < W_MAX_SEGS) {
		W_SEG *s = &w->seg[w->nseg++];

		s->x1 = x1;
		s->y1 = y1;
		s->x2 = x2;
		s->y2 = y2;
	}
}

void w_arc(WORLD *w, double cx, double cy, double r, double a0_deg, double a1_deg)
{
	int n = (int)(fabs(a1_deg - a0_deg) / 3.0) + 1;
	int i;

	for (i = 0; i < n; i++) {
		double a = (a0_deg + ((a1_deg - a0_deg) * i / n)) * PI / 180.0;
		double b = (a0_deg + ((a1_deg - a0_deg) * (i + 1) / n)) * PI / 180.0;

		w_line(w, cx + r * cos(a), cy + r * sin(a), cx + r * cos(b), cy + r * sin(b));
	}
}

bool w_barcode(WORLD *w, double x, double y, double a_deg, char c, bool left)
{
	uint32_t	wd[CODE39_ELEMENTS];
	double		ux = cos(a_deg * PI / 180.0), uy = sin(a_deg * PI / 180.0);
	double		nx = left ? -uy : uy;		/* to the right of travel ... */
	double		ny = left ? ux : -ux;		/* ... or to the left */
	double		s = 0.0;
	uint32_t	i;

	if (!code39_encode(c, 3U, 9U, wd)) {
		return false;
	}
	for (i = 0U; i < CODE39_ELEMENTS; i++) {
		double len = (wd[i] == 3U) ? NARROW_MM : WIDE_MM;

		if (((i % 2U) == 0U) && (w->nbar < W_MAX_BARS)) {	/* even = bar */
			W_RECT	*r = &w->bar[w->nbar++];
			double	lat = LINE_HALF + BAR_GAP + (BAR_HEIGHT / 2.0);

			r->cx = x + ux * (s + len / 2.0) + nx * lat;
			r->cy = y + uy * (s + len / 2.0) + ny * lat;
			r->ux = ux;
			r->uy = uy;
			r->hl = len / 2.0;
			r->hw = BAR_HEIGHT / 2.0;
		}
		s += len;
	}
	return true;
}

void w_hump(WORLD *w, double x0, double len, double h)
{
	if (w->nhump < W_MAX_HUMPS) {
		w->hump[w->nhump].x0 = x0;
		w->hump[w->nhump].len = len;
		w->hump[w->nhump].h = h;
		w->nhump++;
	}
}

void w_box(WORLD *w, double x1, double y1, double x2, double y2, bool visible)
{
	if (w->nbox < W_MAX_BOXES) {
		W_BOX *b = &w->box[w->nbox++];

		b->x1 = (x1 < x2) ? x1 : x2;
		b->x2 = (x1 < x2) ? x2 : x1;
		b->y1 = (y1 < y2) ? y1 : y2;
		b->y2 = (y1 < y2) ? y2 : y1;
		b->visible = visible;
	}
}

static double seg_dist(const W_SEG *s, double px, double py)
{
	double dx = s->x2 - s->x1, dy = s->y2 - s->y1;
	double l2 = dx * dx + dy * dy;
	double t = (l2 > 0.0) ? (((px - s->x1) * dx + (py - s->y1) * dy) / l2) : 0.0;
	double qx, qy;

	if (t < 0.0) {
		t = 0.0;
	} else if (t > 1.0) {
		t = 1.0;
	}
	qx = s->x1 + t * dx;
	qy = s->y1 + t * dy;
	return hypot(px - qx, py - qy);
}

double w_line_dist(const WORLD *w, double px, double py)
{
	double	best = 1e9;
	int	i;

	for (i = 0; i < w->nseg; i++) {
		double d = seg_dist(&w->seg[i], px, py);

		if (d < best) {
			best = d;
		}
	}
	return best;
}

bool w_black(const WORLD *w, double px, double py)
{
	int i;

	if (w_line_dist(w, px, py) <= LINE_HALF) {
		return true;
	}
	for (i = 0; i < w->nbar; i++) {
		const W_RECT	*r = &w->bar[i];
		double		dx = px - r->cx, dy = py - r->cy;
		double		a = dx * r->ux + dy * r->uy;
		double		b = dx * r->uy - dy * r->ux;

		if ((fabs(a) <= r->hl) && (fabs(b) <= r->hw)) {
			return true;
		}
	}
	return false;
}

double w_height(const WORLD *w, double px, double py)
{
	double	h = 0.0;
	int	i;

	(void)py;
	for (i = 0; i < w->nhump; i++) {
		const W_HUMP *hp = &w->hump[i];

		if ((px >= hp->x0) && (px <= hp->x0 + hp->len)) {
			h += hp->h * (1.0 - cos(2.0 * PI * (px - hp->x0) / hp->len)) / 2.0;
		}
	}
	return h;
}

/* ---- car ---------------------------------------------------------------- */
void car_init(CAR *c, double x, double y, double th_deg)
{
	c->x = x;
	c->y = y;
	c->th = th_deg * PI / 180.0;
	c->v[0] = 0.0;
	c->v[1] = 0.0;
	c->cnt[0] = 0.0;
	c->cnt[1] = 0.0;
	c->duty[0] = 0.0;
	c->duty[1] = 0.0;
	c->servo_deg = 90.0;
	c->servo_cmd = 90.0;
	c->pitch = 0.0;
	c->a_long = 0.0;
	c->crashed = false;
	c->crash_count = 0;
	c->impact_ms = 0.0;
	c->odo = 0.0;
}

void car_point(const CAR *c, double fwd, double left, double *px, double *py)
{
	*px = c->x + fwd * cos(c->th) - left * sin(c->th);
	*py = c->y + fwd * sin(c->th) + left * cos(c->th);
}

static bool inside_box(const WORLD *w, double px, double py)
{
	int i;

	for (i = 0; i < w->nbox; i++) {
		const W_BOX *b = &w->box[i];

		if ((px >= b->x1) && (px <= b->x2) && (py >= b->y1) && (py <= b->y2)) {
			return true;
		}
	}
	return false;
}

static bool car_hits(const CAR *c, const WORLD *w)
{
	double f, l, px, py;

	for (f = -CAR_REAR; f <= CAR_FRONT; f += 10.0) {
		car_point(c, f, CAR_HALF_W, &px, &py);
		if (inside_box(w, px, py)) {
			return true;
		}
		car_point(c, f, -CAR_HALF_W, &px, &py);
		if (inside_box(w, px, py)) {
			return true;
		}
	}
	for (l = -CAR_HALF_W; l <= CAR_HALF_W; l += 10.0) {
		car_point(c, CAR_FRONT, l, &px, &py);
		if (inside_box(w, px, py)) {
			return true;
		}
		car_point(c, -CAR_REAR, l, &px, &py);
		if (inside_box(w, px, py)) {
			return true;
		}
	}
	return false;
}

void car_step(CAR *c, const WORLD *w, double dt)
{
	double	vm_old = (c->v[0] + c->v[1]) / 2.0;
	double	ox = c->x, oy = c->y, oth = c->th;
	double	vm, om, ax, ay, fx, fy;
	int	i;

	for (i = 0; i < 2; i++) {
		double d = c->duty[i];

		if (d == 0.0) {
			c->v[i] += (0.0 - c->v[i]) * dt / (c->coast ? TAU_COAST_S : TAU_BRAKE_S);
		} else {
			double mag = (fabs(d) - DB_TRUE) / KFF_TRUE;
			double vss;

			if (mag < 0.0) {
				mag = 0.0;
			}
			if (i == 1) {
				mag *= RIGHT_EFF;
			}
			vss = (d > 0.0) ? mag : -mag;
			vss -= GRAV_LOAD * sin(c->pitch);	/* uphill is harder */
			c->v[i] += (vss - c->v[i]) * dt / TAU_S;
		}
		c->cnt[i] += c->v[i] * dt * g_counts_mm;
	}
	vm = (c->v[0] + c->v[1]) / 2.0;
	om = (c->v[1] - c->v[0]) / g_track_true;
	c->a_long = 0.8 * c->a_long + 0.2 * ((vm - vm_old) / dt);
	c->th += om * dt;
	c->x += vm * cos(c->th) * dt;
	c->y += vm * sin(c->th) * dt;
	c->odo += fabs(vm) * dt;

	if (car_hits(c, w)) {
		/* blocked: the car does not move and the wheels stall */
		if (!c->crashed) {
			c->crash_count++;
			c->impact_ms = 30.0;
		}
		c->crashed = true;
		c->x = ox;
		c->y = oy;
		c->th = oth;
		for (i = 0; i < 2; i++) {
			c->cnt[i] -= c->v[i] * dt * g_counts_mm;
			c->v[i] = 0.0;
		}
	} else {
		c->crashed = false;
	}
	if (c->impact_ms > 0.0) {
		c->impact_ms -= dt * 1000.0;
	}

	/* servo slews toward its command */
	if (fabs(c->servo_cmd - c->servo_deg) <= SERVO_DEG_S * dt) {
		c->servo_deg = c->servo_cmd;
	} else {
		c->servo_deg += (c->servo_cmd > c->servo_deg) ? (SERVO_DEG_S * dt) : (-SERVO_DEG_S * dt);
	}

	/* pitch from the heights under the axle and the castor */
	car_point(c, 0.0, 0.0, &ax, &ay);
	car_point(c, CAR_CASTOR_FWD, 0.0, &fx, &fy);
	c->pitch = atan2(w_height(w, fx, fy) - w_height(w, ax, ay), CAR_CASTOR_FWD);
}

bool car_sensor_black(const CAR *c, const WORLD *w, double fwd, double left)
{
	double px, py, ax, ay, fx, fy, body, gap;

	car_point(c, fwd, left, &px, &py);
	car_point(c, 0.0, 0.0, &ax, &ay);
	car_point(c, CAR_CASTOR_FWD, 0.0, &fx, &fy);
	body = w_height(w, ax, ay) + (w_height(w, fx, fy) - w_height(w, ax, ay)) * fwd / CAR_CASTOR_FWD;
	gap = SENSOR_GAP_MM + body - w_height(w, px, py);
	if (gap > SENSOR_MAX_MM) {
		return true;			/* lifted: no reflection reads as black */
	}
	return w_black(w, px, py);
}

int32_t car_count(const CAR *c, int i)
{
	return (int32_t)floor(c->cnt[i]);
}

/* ---- ultrasonic ------------------------------------------------------------ */
static double ray_box(double sx, double sy, double dx, double dy, const W_BOX *b)
{
	double	best = 1e9;
	double	t, hx, hy;

	/* vertical edges x = x1 (normal -x) and x = x2 (normal +x) */
	if (fabs(dx) > 1e-9) {
		double xs[2] = {b->x1, b->x2};
		int k;

		for (k = 0; k < 2; k++) {
			t = (xs[k] - sx) / dx;
			hy = sy + t * dy;
			if ((t > 0.0) && (hy >= b->y1) && (hy <= b->y2) &&
			    (fabs(dx) >= cos(US_SPECULAR_DEG * PI / 180.0)) && (t < best)) {
				best = t;
			}
		}
	}
	if (fabs(dy) > 1e-9) {
		double ys[2] = {b->y1, b->y2};
		int k;

		for (k = 0; k < 2; k++) {
			t = (ys[k] - sy) / dy;
			hx = sx + t * dx;
			if ((t > 0.0) && (hx >= b->x1) && (hx <= b->x2) &&
			    (fabs(dy) >= cos(US_SPECULAR_DEG * PI / 180.0)) && (t < best)) {
				best = t;
			}
		}
	}
	return best;
}

double us_range(const CAR *c, const WORLD *w)
{
	double	sx, sy, best = 1e9, off;
	int	i;

	car_point(c, CAR_US_FWD, 0.0, &sx, &sy);
	for (off = -7.5; off <= 7.51; off += 2.5) {
		double a = c->th + ((c->servo_deg - 90.0 + off) * PI / 180.0);

		for (i = 0; i < w->nbox; i++) {
			double r;

			if (!w->box[i].visible) {
				continue;
			}
			r = ray_box(sx, sy, cos(a), sin(a), &w->box[i]);
			if (r < best) {
				best = r;
			}
		}
	}
	if (best > US_MAX_MM) {
		return -1.0;
	}
	return best + 2.0 * rnd_normal();
}

/* ---- accelerometer ---------------------------------------------------------- */
void imu_read(const CAR *c, double noise_mg, int32_t a[3])
{
	double	vm = (c->v[0] + c->v[1]) / 2.0;
	double	om = (c->v[1] - c->v[0]) / g_track_true;
	double	f = (G_MM_S2 * sin(c->pitch) + c->a_long) / G_MM_S2 * 1000.0;
	double	u = cos(c->pitch) * 1000.0;
	double	l = (vm * om) / G_MM_S2 * 1000.0;
	double	b = (IMU_TILT_DEG + c->imu_extra_deg) * PI / 180.0;
	double	fs, us;

	if (c->impact_ms > 0.0) {
		f -= 3000.0;			/* the jolt of a collision */
	}
	fs = f * cos(b) + u * sin(b);
	us = -f * sin(b) + u * cos(b);
	a[0] = (int32_t)lround(fs + noise_mg * rnd_normal());
	a[1] = (int32_t)lround(l + noise_mg * rnd_normal());
	a[2] = (int32_t)lround(us + noise_mg * rnd_normal());
}
