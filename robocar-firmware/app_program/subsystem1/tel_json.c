/*
 *	tel_json.c  (see tel_json.h)
 */

#include "tel_json.h"
#include "../subsystem4/imu_core.h"	/* imu_motion_name(), imu_terrain_name() */
#include "../subsystem5/vc_core.h"	/* vc_state_name(), vc_nav_name() */

#define DIGITS_MAX	11U		/* "-2147483648" */

/* ---- builder ---------------------------------------------------------------- */
static void put_c(TJ *j, char c)
{
	if (j->len < (j->cap - 1U)) {		/* keep room for the closing brace */
		j->buf[j->len] = c;
		j->len++;
	} else {
		j->ok = false;
	}
}

static void put_s(TJ *j, const char *s)
{
	while (*s != '\0') {
		put_c(j, *s);
		s++;
	}
}

static void put_u(TJ *j, uint32_t v)
{
	char		d[DIGITS_MAX];
	uint32_t	n = 0U;

	do {
		d[n] = (char)('0' + (v % 10U));
		n++;
		v /= 10U;
	} while ((v != 0U) && (n < DIGITS_MAX));
	while (n > 0U) {
		n--;
		put_c(j, d[n]);
	}
}

static void put_i(TJ *j, int32_t v)
{
	if (v < 0) {
		put_c(j, '-');
		put_u(j, (uint32_t)(-(v + 1)) + 1U);	/* safe for INT32_MIN */
	} else {
		put_u(j, (uint32_t)v);
	}
}

static void put_key(TJ *j, const char *key)
{
	if (!j->first) {
		put_c(j, ',');
	}
	j->first = false;
	put_c(j, '"');
	put_s(j, key);
	put_s(j, "\":");
}

void tj_begin(TJ *j, char *buf, uint32_t cap)
{
	j->buf = buf;
	j->cap = cap;
	j->len = 0U;
	j->ok = (cap >= 3U);
	j->first = true;
	if (j->ok) {
		put_c(j, '{');
	}
}

void tj_int(TJ *j, const char *key, int32_t v)
{
	put_key(j, key);
	put_i(j, v);
}

void tj_uint(TJ *j, const char *key, uint32_t v)
{
	put_key(j, key);
	put_u(j, v);
}

void tj_str(TJ *j, const char *key, const char *s)
{
	put_key(j, key);
	put_c(j, '"');
	while (*s != '\0') {
		if ((*s == '"') || (*s == '\\')) {
			put_c(j, '\\');
		}
		put_c(j, *s);
		s++;
	}
	put_c(j, '"');
}

void tj_char(TJ *j, const char *key, char c)
{
	char s[2];

	s[0] = c;
	s[1] = '\0';
	tj_str(j, key, s);
}

void tj_pair(TJ *j, const char *key, int32_t a, int32_t b)
{
	put_key(j, key);
	put_c(j, '[');
	put_i(j, a);
	put_c(j, ',');
	put_i(j, b);
	put_c(j, ']');
}

int32_t tj_end(TJ *j)
{
	if (!j->ok) {
		return -1;
	}
	j->buf[j->len] = '}';			/* room was kept for it */
	j->len++;
	return (int32_t)j->len;
}

/* ---- messages ------------------------------------------------------------- */
static const char *line_text(bool l, bool r)
{
	static const char * const txt[4] = {"WW", "WB", "BW", "BB"};	/* left first */

	return txt[(l ? 2U : 0U) + (r ? 1U : 0U)];
}

/* Keep a number within what the field can sensibly hold, so the message
   always fits in 200 bytes whatever the values (a garbage value shows as
   the limit instead of the whole message being lost). */
static int32_t lim(int32_t v, int32_t max)
{
	if (v > max) {
		return max;
	}
	if (v < -max) {
		return -max;
	}
	return v;
}

#define LIM4	9999		/* speeds, angles, heights */
#define LIM5	99999		/* ranges */
#define LIM7	9999999		/* counts and distances (5 km at 2 counts/mm) */

int32_t tel_fmt_telemetry(const TEL_DATA *d, char *buf, uint32_t cap)
{
	TJ j;

	tj_begin(&j, buf, cap);
	tj_uint(&j, "t", d->t_ms);
	tj_str(&j, "st", vc_state_name(d->state));
	tj_pair(&j, "v", lim(d->v[0], LIM4), lim(d->v[1], LIM4));
	tj_pair(&j, "enc", lim(d->enc[0], LIM7), lim(d->enc[1], LIM7));
	tj_int(&j, "d", lim(d->odo_mm, LIM7));
	tj_str(&j, "ln", line_text(d->line_l, d->line_r));
	tj_char(&j, "bc", d->barcode);
	tj_int(&j, "cmd", lim(d->pending, 9));
	tj_int(&j, "p", lim(d->pitch_cdeg / 10, LIM4));		/* tenths of a degree */
	tj_str(&j, "mo", imu_motion_name(d->motion));
	tj_int(&j, "hmax", lim(d->hmax_mm, LIM4));
	tj_int(&j, "rng", lim(d->range_mm, LIM5));
	tj_int(&j, "yaw", lim(d->yaw_cdeg_s / 100, LIM4));	/* deg/s */
	tj_uint(&j, "flt", d->faults & 0xFFFFU);
	return tj_end(&j);
}

int32_t tel_fmt_status(const TEL_DATA *d, char *buf, uint32_t cap)
{
	TJ j;

	tj_begin(&j, buf, cap);
	tj_uint(&j, "up", d->t_ms / 1000U);
	tj_str(&j, "st", vc_state_name(d->state));
	tj_int(&j, "why", d->why);
	tj_uint(&j, "flt", d->faults);
	tj_int(&j, "mcf", d->mc_fault);
	tj_int(&j, "imu", d->imu_ok ? 1 : 0);
	tj_int(&j, "hdg", d->heading_cdeg / 100);	/* degrees, uncalibrated */
	tj_uint(&j, "pings", d->pings);
	tj_int(&j, "wifi", d->wifi);
	tj_int(&j, "mq", d->mqtt_up ? 1 : 0);
	tj_uint(&j, "conn", d->mqtt_connects);
	tj_uint(&j, "txd", d->tx_drops);
	tj_uint(&j, "rxd", d->rx_drops);
	tj_uint(&j, "nerr", d->net_errors);
	return tj_end(&j);
}

int32_t tel_fmt_event(const APP_MSG *m, char *buf, uint32_t cap)
{
	TJ j;

	tj_begin(&j, buf, cap);
	tj_uint(&j, "t", m->t_ms);
	switch (m->type) {
	case EV_STATE:
		tj_str(&j, "ev", "state");
		tj_str(&j, "st", vc_state_name(m->p1));
		tj_int(&j, "why", m->p2);
		break;
	case EV_NAV:
		tj_str(&j, "ev", "nav");
		tj_str(&j, "what", vc_nav_name(m->p1));
		tj_int(&j, "v", m->p2);
		break;
	case EV_BARCODE:
		tj_str(&j, "ev", "barcode");
		tj_char(&j, "c", (char)m->p1);
		tj_int(&j, "cmd", m->p2);
		break;
	case EV_BARCODE_BAD:
		tj_str(&j, "ev", "barcode_bad");
		tj_int(&j, "why", m->p1);
		tj_int(&j, "n", m->p2);
		break;
	case EV_JUNCTION:
		tj_str(&j, "ev", "junction");
		tj_int(&j, "d", m->p1);
		break;
	case EV_LINE_LOST:
		tj_str(&j, "ev", "line_lost");
		tj_int(&j, "side", m->p1);
		break;
	case EV_TERRAIN:
		tj_str(&j, "ev", "terrain");
		tj_str(&j, "s", imu_terrain_name(m->p1));
		tj_int(&j, "p", m->p2 / 10);
		break;
	case EV_HUMP:
		tj_str(&j, "ev", "hump");
		tj_int(&j, "h", m->p1);
		tj_int(&j, "hmax", m->p2);
		break;
	case EV_MOTION:
		tj_str(&j, "ev", "motion");
		tj_str(&j, "mo", imu_motion_name(m->p1));
		break;
	case EV_IMPACT:
		tj_str(&j, "ev", "impact");
		tj_int(&j, "mg", m->p1);
		tj_int(&j, "v", m->p2);
		break;
	case EV_FAULT:
		tj_str(&j, "ev", "fault");
		tj_int(&j, "bits", m->p1);
		break;
	case EV_FAULT_CLEAR:
		tj_str(&j, "ev", "fault_clear");
		break;
	default:
		return 0;			/* not published */
	}
	return tj_end(&j);
}

int32_t tel_fmt_ack(uint32_t t_ms, uint32_t seq, bool ok, int32_t err, char *buf, uint32_t cap)
{
	TJ j;

	tj_begin(&j, buf, cap);
	tj_uint(&j, "t", t_ms);
	tj_str(&j, "ev", "cmd");
	tj_uint(&j, "seq", seq);
	tj_int(&j, "ok", ok ? 1 : 0);
	tj_int(&j, "err", err);
	return tj_end(&j);
}

int32_t tel_fmt_profile(uint32_t t_ms, bool valid, int32_t x_left, int32_t x_right,
			int32_t closest, int32_t closest_deg, int32_t clear_l, int32_t clear_r,
			uint32_t unknown_l, uint32_t unknown_r, char *buf, uint32_t cap)
{
	TJ j;

	tj_begin(&j, buf, cap);
	tj_uint(&j, "t", t_ms);
	tj_str(&j, "ev", "profile");
	tj_int(&j, "ok", valid ? 1 : 0);
	if (valid) {
		tj_int(&j, "xl", lim(x_left, LIM5));
		tj_int(&j, "xr", lim(x_right, LIM5));
		tj_int(&j, "w", lim(x_right - x_left, LIM5));
		tj_int(&j, "y", lim(closest, LIM5));
		tj_int(&j, "deg", lim(closest_deg, 999));
		tj_int(&j, "cl", lim(clear_l, LIM5));
		tj_int(&j, "cr", lim(clear_r, LIM5));
		tj_uint(&j, "ul", (unknown_l > 99U) ? 99U : unknown_l);
		tj_uint(&j, "ur", (unknown_r > 99U) ? 99U : unknown_r);
	}
	return tj_end(&j);
}

int32_t tel_fmt_summary(const TEL_SUMMARY *s, char *buf, uint32_t cap)
{
	TJ j;

	tj_begin(&j, buf, cap);
	tj_uint(&j, "t", s->t_ms);
	tj_str(&j, "ev", "summary");
	tj_uint(&j, "time", s->time_ms);
	tj_int(&j, "d", s->dist_mm);
	tj_int(&j, "hmax", s->hmax_mm);
	tj_uint(&j, "humps", s->humps);
	tj_uint(&j, "cmds", s->commands);
	tj_uint(&j, "obst", s->obstacles);
	tj_uint(&j, "bc_ok", s->reads_ok);
	tj_uint(&j, "bc_bad", s->reads_failed);
	return tj_end(&j);
}

static uint32_t ulim(uint32_t v, uint32_t max)
{
	return (v > max) ? max : v;
}

int32_t tel_fmt_timing(const TEL_TIMING *s, char *buf, uint32_t cap)
{
	static const char * const key[TEL_TIM_TASKS] = {"mc", "ln", "vc", "sc", "im", "sm", "tl"};
	TJ		j;
	uint32_t	i;

	tj_begin(&j, buf, cap);
	tj_uint(&j, "t", s->t_ms);
	tj_str(&j, "ev", "tim");
	for (i = 0U; i < TEL_TIM_TASKS; i++) {
		tj_uint(&j, key[i], ulim(s->max_us[i], (uint32_t)LIM5));
	}
	tj_uint(&j, "ov", ulim(s->overruns, (uint32_t)LIM5));
	tj_uint(&j, "irq", ulim(s->irq_per_s, (uint32_t)LIM5));
	tj_uint(&j, "irqm", ulim(s->irq_max_us, (uint32_t)LIM4));
	tj_uint(&j, "vq", ulim(s->vq_high, 99U));
	tj_uint(&j, "dr", ulim(s->dropped, (uint32_t)LIM5));
	tj_uint(&j, "sl", ulim((s->steer_lat_max_ms > 0) ? (uint32_t)s->steer_lat_max_ms : 0U, (uint32_t)LIM4));
	tj_uint(&j, "st", ulim((s->stop_lat_max_us > 0) ? (uint32_t)s->stop_lat_max_us : 0U, (uint32_t)LIM5));
	tj_uint(&j, "sd", ulim((s->stop_dist_mm > 0) ? (uint32_t)s->stop_dist_mm : 0U, 999U));
	tj_uint(&j, "rc", ulim(s->reconnect_ms, (uint32_t)LIM5));
	return tj_end(&j);
}
