/*
 *	msg_sizes.c
 *	Week 13 evidence (Change Requirement 1): message sizes before and
 *	after the minimum-bit change, measured with each version's own code.
 *	Built twice by measure.sh: with -DV12 against the V12 tree (JSON MQTT,
 *	unpacked records) and without it against this tree (binary, packed).
 *	The same values go into both, so the sizes compare like with like.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app_evt.h"
#include "subsystem2/mc_core.h"
#include "subsystem5/scan_core.h"
#include "subsystem5/os_profile.h"
#ifdef V12
#include "subsystem1/tel_json.h"
#define OUT_MAX		TEL_JSON_MAX
#else
#include "subsystem1/tel_bin.h"
#include "subsystem1/cmd_parse.h"
#define OUT_MAX		TEL_BIN_MAX
#endif

/* typical values on a run, and the longest text each field can produce */
static void fill(TEL_DATA *d, bool worst)
{
	memset(d, 0, sizeof(*d));
	d->t_ms = worst ? 4000000000U : 61234U;
	d->state = worst ? 13 : 2;
	d->why = worst ? 9 : 0;
	d->v[0] = worst ? -400 : 182;
	d->v[1] = worst ? -400 : 176;
	d->enc[0] = worst ? -2000000 : 18234;
	d->enc[1] = worst ? -2000000 : 18101;
	d->odo_mm = worst ? -1000000 : 2413;
	d->yaw_cdeg_s = worst ? -51100 : -350;
	d->line_l = true;
	d->line_r = false;
	d->barcode = worst ? '%' : 'A';
	d->pending = worst ? 4 : 0;
	d->pitch_cdeg = worst ? -8192 : 120;
	d->motion = worst ? 7 : 1;
	d->hmax_mm = worst ? 255 : 18;
	d->range_mm = worst ? 4095 : 734;
	d->faults = worst ? 0x63FFU : 0U;
	d->mc_fault = worst ? 7 : 0;
	d->imu_ok = true;
	d->heading_cdeg = worst ? -17999 : 9012;
	d->pings = worst ? 4000000000U : 912U;
	d->wifi = worst ? -3 : 3;
#ifdef V12
	d->mqtt_up = !worst;	/* V12 also sent the link state as its own field */
#endif
	d->mqtt_connects = worst ? 4000000000U : 1U;
	d->tx_drops = worst ? 4000000000U : 0U;
	d->rx_drops = worst ? 4000000000U : 0U;
	d->net_errors = worst ? 4000000000U : 0U;
#ifndef V12
	d->reconnect_ms = worst ? 262143U : 0U;
#endif
}

/* a run summary and a timing record, typical and longest */
static void fill_summary(TEL_SUMMARY *s, bool worst)
{
	memset(s, 0, sizeof(*s));
	s->t_ms = worst ? 4000000000U : 99000U;
	s->time_ms = worst ? 4000000000U : 95000U;
	s->dist_mm = worst ? -1000000 : 7340;
	s->hmax_mm = worst ? 255 : 18;
	s->humps = worst ? 255U : 2U;
	s->commands = worst ? 255U : 4U;
	s->obstacles = worst ? 127U : 1U;
	s->reads_ok = worst ? 255U : 4U;
	s->reads_failed = worst ? 255U : 0U;
}

static void fill_timing(TEL_TIMING *s, bool worst)
{
	static const uint32_t typ[TEL_TIM_TASKS] = {180U, 95U, 60U, 120U, 140U, 30U, 900U};
	uint32_t i;

	memset(s, 0, sizeof(*s));
	s->t_ms = worst ? 4000000000U : 65000U;
	for (i = 0U; i < TEL_TIM_TASKS; i++) {
		s->max_us[i] = worst ? 65535U : typ[i];
	}
	s->overruns = worst ? 65535U : 0U;
	s->irq_per_s = worst ? 65535U : 3300U;
	s->irq_max_us = worst ? 65535U : 12U;
	s->vq_high = worst ? 32U : 3U;
	s->dropped = worst ? 65535U : 0U;
	s->steer_lat_max_ms = worst ? 1023 : 2;
	s->stop_lat_max_us = worst ? 131071 : 450;
	s->stop_dist_mm = worst ? 1023 : 31;
	s->reconnect_ms = worst ? 262143U : 0U;
}

/* one obstacle profile, as the car measured it */
static const struct {
	int32_t x_left, x_right, closest, deg, clear_l, clear_r;
	uint32_t unknown_l, unknown_r, npoints;
} k_prof = {-120, 80, 240, 95, 300, 150, 0U, 0U, 25U};

#ifdef V12
static int32_t telemetry(const TEL_DATA *d, char *b) { return tel_fmt_telemetry(d, b, OUT_MAX); }
static int32_t status(const TEL_DATA *d, char *b) { return tel_fmt_status(d, b, OUT_MAX); }
static int32_t event(uint8_t type, int32_t p1, int32_t p2, char *b)
{
	APP_MSG m = {type, SRC_VC, 7U, 61234U, p1, p2};

	return tel_fmt_event(&m, b, OUT_MAX);
}
static int32_t ack(char *b) { return tel_fmt_ack(61234U, 16U, true, 0, b, OUT_MAX); }
static int32_t profile(char *b)
{
	return tel_fmt_profile(61234U, true, k_prof.x_left, k_prof.x_right, k_prof.closest, k_prof.deg,
			       k_prof.clear_l, k_prof.clear_r, k_prof.unknown_l, k_prof.unknown_r, b, OUT_MAX);
}
static int32_t summary(const TEL_SUMMARY *s, char *b) { return tel_fmt_summary(s, b, OUT_MAX); }
static int32_t timing(const TEL_TIMING *s, char *b) { return tel_fmt_timing(s, b, OUT_MAX); }
#else
static int32_t telemetry(const TEL_DATA *d, char *b) { return tel_bin_telemetry(d, (uint8_t *)b, OUT_MAX); }
static int32_t status(const TEL_DATA *d, char *b) { return tel_bin_status(d, (uint8_t *)b, OUT_MAX); }
static int32_t event(uint8_t type, int32_t p1, int32_t p2, char *b)
{
	APP_EVT e = {type, SRC_VC, 7U, 61234U, p1, p2};

	return tel_bin_event(&e, 61234U, (uint8_t *)b, OUT_MAX);
}
static int32_t ack(char *b) { return tel_bin_ack(61234U, 16U, true, 0, (uint8_t *)b, OUT_MAX); }
static int32_t profile(char *b)
{
	OS_PROFILE p;

	memset(&p, 0, sizeof(p));
	p.valid = true;
	p.x_left_mm = k_prof.x_left;
	p.x_right_mm = k_prof.x_right;
	p.width_mm = k_prof.x_right - k_prof.x_left;
	p.x_center_mm = (k_prof.x_left + k_prof.x_right) / 2;
	p.closest_mm = k_prof.closest;
	p.closest_deg = k_prof.deg;
	p.left_clear_mm = k_prof.clear_l;
	p.right_clear_mm = k_prof.clear_r;
	p.unknown_left = k_prof.unknown_l;
	p.unknown_right = k_prof.unknown_r;
	p.npoints = k_prof.npoints;
	return tel_bin_profile(61234U, &p, (uint8_t *)b, OUT_MAX);
}
static int32_t summary(const TEL_SUMMARY *s, char *b) { return tel_bin_summary(s, (uint8_t *)b, OUT_MAX); }
static int32_t timing(const TEL_TIMING *s, char *b) { return tel_bin_timing(s, (uint8_t *)b, OUT_MAX); }
#endif

int main(void)
{
	static const struct { uint8_t type; int32_t p1; int32_t p2; } ev[] = {
		{EV_LINE_LOST, 1, 0}, {EV_JUNCTION, 2413, 0}, {EV_BARCODE, 'A', 1},
		{EV_BARCODE_BAD, 3, 9}, {EV_FAULT, 0x001, 0}, {EV_FAULT_CLEAR, 0, 0},
		{EV_TERRAIN, 1, 520}, {EV_HUMP, 18, 18}, {EV_MOTION, 4, 0},
		{EV_IMPACT, 1600, 180}, {EV_STATE, 2, 0}, {EV_NAV, 2, 1}
	};
	char		b[512];
	TEL_DATA	d;
	uint32_t	i, n = 0U, sum = 0U, max = 0U;
	int32_t		len;

	printf("version: %s\n",
#ifdef V12
	       "V12 (JSON MQTT, unpacked records)"
#else
	       "min-bit (binary MQTT, packed records)"
#endif
	       );
	/* inter-task messages and shared records */
	printf("APP_MSG (vc_event_q, telemetry_q)      %3u B\n", (unsigned)sizeof(APP_MSG));
#ifdef V12
	printf("motion command (motion_cmd_q)          %3u B\n", (unsigned)sizeof(MC_CMD));
	printf("motion snapshot (under mtx_snapshot)   %3u B\n", (unsigned)sizeof(MC_SNAPSHOT));
	printf("scan request (scan_request_q)          %3u B\n", (unsigned)sizeof(SCAN_REQ));
	printf("obstacle profile (shared)              %3u B\n", (unsigned)sizeof(OS_PROFILE));
#else
	printf("motion command (motion_cmd_q)          %3u B\n", (unsigned)sizeof(MC_CMD_PK));
	printf("motion snapshot (under mtx_snapshot)   %3u B\n", (unsigned)sizeof(MC_SNAP_PK));
	printf("scan request (scan_request_q)          %3u B\n", (unsigned)sizeof(SCAN_REQ_PK));
	printf("obstacle profile (under mtx_profile)   %3u B\n", (unsigned)sizeof(OS_PROFILE_PK));
#endif
	/* MQTT */
	fill(&d, false);
	printf("MQTT telemetry, typical run            %3d B\n", (int)telemetry(&d, b));
	printf("MQTT status, typical run               %3d B\n", (int)status(&d, b));
	fill(&d, true);
	printf("MQTT telemetry, longest values         %3d B\n", (int)telemetry(&d, b));
	printf("MQTT status, longest values            %3d B\n", (int)status(&d, b));
	for (i = 0U; i < sizeof(ev) / sizeof(ev[0]); i++) {
		len = event(ev[i].type, ev[i].p1, ev[i].p2, b);
		if (len > 0) {
			n++;
			sum += (uint32_t)len;
			max = ((uint32_t)len > max) ? (uint32_t)len : max;
		}
	}
	printf("MQTT event, mean of %2u types           %3u B (longest %u B)\n", (unsigned)n,
	       (unsigned)((sum + (n / 2U)) / n), (unsigned)max);
	printf("MQTT command ack                       %3d B\n", (int)ack(b));
	printf("MQTT obstacle profile                  %3d B\n", (int)profile(b));
	{
		TEL_SUMMARY	s;
		TEL_TIMING	t;
		int32_t		typ, worst;

		fill_summary(&s, false);
		typ = summary(&s, b);
		fill_summary(&s, true);
		worst = summary(&s, b);
		printf("MQTT run summary                       %3d B (longest %d B)\n", (int)typ, (int)worst);
		fill_timing(&t, false);
		typ = timing(&t, b);
		fill_timing(&t, true);
		worst = timing(&t, b);
		printf("MQTT timing record (every 5 s)         %3d B (longest %d B)\n", (int)typ, (int)worst);
	}
#ifdef V12
	printf("MQTT command \"16 SET base_speed 220\"   %3u B (text, up to 64 B)\n",
	       (unsigned)strlen("16 SET base_speed 220"));
#else
	printf("MQTT command \"16 SET base_speed 220\"   %3u B (binary, always)\n", (unsigned)CMD_BYTES);
#endif
	return 0;
}
