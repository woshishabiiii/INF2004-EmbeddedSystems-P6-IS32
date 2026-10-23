/*
 *	tel_bin.c
 *	Binary MQTT records (see tel_bin.h for every layout).
 */

#include <string.h>
#include "tel_bin.h"
#include "../bitpack.h"
#include "../subsystem4/safety_core.h"	/* FLT_ALL, FLT_NO_ECHO, FLT_WIFI_DOWN */

#define T_BITS		24U	/* header time */
#define CHAR_MIN	32	/* ' ': characters ' '..'_' in 6 bits */

uint32_t tel_fault12(uint32_t flags)
{
	return (flags & FLT_ALL) |
	       (((flags & FLT_NO_ECHO) != 0U) ? 0x400U : 0U) |
	       (((flags & FLT_WIFI_DOWN) != 0U) ? 0x800U : 0U);
}

/* Header; false if the record does not fit */
static bool begin(BP_W *w, uint8_t *buf, uint32_t cap, uint32_t bytes, uint32_t type, uint32_t t_ms)
{
	if (cap < bytes) {
		return false;
	}
	memset(buf, 0, bytes);
	w->rec = buf;
	w->pos = 0U;
	bp_wraw(w, 4U, TEL_BIN_VERSION);
	bp_wraw(w, 4U, type);
	bp_wraw(w, T_BITS, t_ms);		/* low 24 bits: wraps */
	return true;
}

/* The record's length.  The last field must end in the record's last
   byte, so no record carries a spare byte; -1 if a layout and its size
   disagree (the TEL tests then fail). */
static int32_t finish(const BP_W *w, uint32_t bytes)
{
	return (((w->pos + 7U) / 8U) == bytes) ? (int32_t)bytes : -1;
}

int32_t tel_bin_telemetry(const TEL_DATA *d, uint8_t *buf, uint32_t cap)
{
	BP_W w;

	if (!begin(&w, buf, cap, TEL_BIN_TELEMETRY_BYTES, TEL_T_TELEMETRY, d->t_ms)) {
		return -1;
	}
	bp_wu(&w, 4U, d->state);
	bp_ws(&w, 10U, -512, d->v[0]);
	bp_ws(&w, 10U, -512, d->v[1]);
	bp_wraw(&w, 16U, (uint32_t)d->enc[0]);	/* low 16 bits, as the snapshot */
	bp_wraw(&w, 16U, (uint32_t)d->enc[1]);
	bp_wu(&w, 24U, d->odo_mm);
	bp_wraw(&w, 2U, (d->line_l ? 1U : 0U) | (d->line_r ? 2U : 0U));
	bp_ws(&w, 6U, CHAR_MIN, (int32_t)d->barcode);
	bp_wu(&w, 3U, d->pending);
	bp_ws(&w, 14U, -8192, d->pitch_cdeg);
	bp_wu(&w, 3U, d->motion);
	bp_wu(&w, 8U, d->hmax_mm);
	bp_wu(&w, 12U, d->range_mm);
	bp_ws(&w, 10U, -512, d->yaw_cdeg_s / 100);	/* deg/s */
	bp_wraw(&w, 12U, tel_fault12(d->faults));
	return finish(&w, TEL_BIN_TELEMETRY_BYTES);
}

int32_t tel_bin_status(const TEL_DATA *d, uint8_t *buf, uint32_t cap)
{
	BP_W w;

	if (!begin(&w, buf, cap, TEL_BIN_STATUS_BYTES, TEL_T_STATUS, d->t_ms)) {
		return -1;
	}
	bp_wu(&w, 4U, d->state);
	bp_wu(&w, 4U, d->why);
	bp_wraw(&w, 12U, tel_fault12(d->faults));
	bp_wraw(&w, 3U, (uint32_t)d->mc_fault);
	bp_wraw(&w, 1U, d->imu_ok ? 1U : 0U);
	bp_ws(&w, 9U, -256, d->heading_cdeg / 100);	/* degrees, uncalibrated */
	bp_wu(&w, 16U, (int32_t)d->pings);
	bp_ws(&w, 3U, -4, d->wifi);
	bp_wu(&w, 8U, (int32_t)d->mqtt_connects);
	bp_wu(&w, 16U, (int32_t)d->tx_drops);
	bp_wu(&w, 8U, (int32_t)d->rx_drops);
	bp_wu(&w, 8U, (int32_t)d->net_errors);
	bp_wu(&w, 18U, (int32_t)d->reconnect_ms);
	return finish(&w, TEL_BIN_STATUS_BYTES);
}

int32_t tel_bin_event(const APP_EVT *m, uint32_t t_ms, uint8_t *buf, uint32_t cap)
{
	BP_W	w;
	APP_MSG	pk;

	switch (m->type) {
	case EV_STATE:
	case EV_NAV:
	case EV_BARCODE:
	case EV_BARCODE_BAD:
	case EV_JUNCTION:
	case EV_LINE_LOST:
	case EV_TERRAIN:
	case EV_HUMP:
	case EV_MOTION:
	case EV_IMPACT:
	case EV_FAULT:
	case EV_FAULT_CLEAR:
		break;
	default:
		return 0;			/* not published */
	}
	if (!begin(&w, buf, cap, TEL_BIN_EVENT_BYTES, TEL_T_EVENT, t_ms)) {
		return -1;
	}
	app_msg_pack(m, &pk);			/* the payload exactly as in APP_MSG */
	bp_wraw(&w, 5U, m->type);
	bp_wraw(&w, 32U, pk.pl);
	return finish(&w, TEL_BIN_EVENT_BYTES);
}

int32_t tel_bin_ack(uint32_t t_ms, uint32_t seq, bool ok, int32_t err, uint8_t *buf, uint32_t cap)
{
	BP_W w;

	if (!begin(&w, buf, cap, TEL_BIN_ACK_BYTES, TEL_T_ACK, t_ms)) {
		return -1;
	}
	bp_wraw(&w, 8U, seq);
	bp_wraw(&w, 1U, ok ? 1U : 0U);
	bp_wu(&w, 3U, err);
	return finish(&w, TEL_BIN_ACK_BYTES);
}

int32_t tel_bin_profile(uint32_t t_ms, const OS_PROFILE *p, uint8_t *buf, uint32_t cap)
{
	BP_W		w;
	OS_PROFILE_PK	pk;
	uint32_t	i;

	if (!begin(&w, buf, cap, TEL_BIN_PROFILE_BYTES, TEL_T_PROFILE, t_ms)) {
		return -1;
	}
	os_profile_pack(p, &pk);
	for (i = 0U; i < OS_PROFILE_PK_BYTES; i++) {
		bp_wraw(&w, 8U, pk.b[i]);	/* header is whole bytes: a plain copy */
	}
	return finish(&w, TEL_BIN_PROFILE_BYTES);
}

int32_t tel_bin_summary(const TEL_SUMMARY *s, uint8_t *buf, uint32_t cap)
{
	BP_W w;

	if (!begin(&w, buf, cap, TEL_BIN_SUMMARY_BYTES, TEL_T_SUMMARY, s->t_ms)) {
		return -1;
	}
	bp_wu(&w, 24U, (int32_t)((s->time_ms > 0xFFFFFFU) ? 0xFFFFFFU : s->time_ms));
	bp_ws(&w, 25U, -16777216, s->dist_mm);
	bp_wu(&w, 8U, s->hmax_mm);
	bp_wu(&w, 8U, (int32_t)s->humps);
	bp_wu(&w, 8U, (int32_t)s->commands);
	bp_wu(&w, 7U, (int32_t)s->obstacles);
	bp_wu(&w, 8U, (int32_t)s->reads_ok);
	bp_wu(&w, 8U, (int32_t)s->reads_failed);
	return finish(&w, TEL_BIN_SUMMARY_BYTES);
}

/* An unsigned 32-bit count into n bits, saturating (bp_wu takes int32_t) */
static void put_count(BP_W *w, uint32_t n, uint32_t v)
{
	bp_wraw(w, n, (v > BP_MAX(n)) ? BP_MAX(n) : v);
}

int32_t tel_bin_timing(const TEL_TIMING *s, uint8_t *buf, uint32_t cap)
{
	BP_W		w;
	uint32_t	i;

	if (!begin(&w, buf, cap, TEL_BIN_TIMING_BYTES, TEL_T_TIMING, s->t_ms)) {
		return -1;
	}
	for (i = 0U; i < TEL_TIM_TASKS; i++) {
		put_count(&w, 16U, s->max_us[i]);
	}
	put_count(&w, 16U, s->overruns);
	put_count(&w, 16U, s->irq_per_s);
	put_count(&w, 16U, s->irq_max_us);
	put_count(&w, 6U, s->vq_high);
	put_count(&w, 16U, s->dropped);
	bp_wu(&w, 10U, s->steer_lat_max_ms);
	bp_wu(&w, 17U, s->stop_lat_max_us);
	bp_wu(&w, 10U, s->stop_dist_mm);
	put_count(&w, 18U, s->reconnect_ms);
	return finish(&w, TEL_BIN_TIMING_BYTES);
}
