/*
 *	tel_json.h
 *	Telemetry message formats (Buddy 1).  Pure C: no sprintf, no kernel
 *	calls, so the formats are unit-tested on a laptop (every message must
 *	stay within NET_PAYLOAD_MAX = 200 bytes; lwIP's MQTT buffer is 256).
 *
 *	Topics (prefix car/<group>/):
 *	  status     every 1 s   heartbeat: uptime, state, faults, link counters
 *	  telemetry  every 0.5 s state, wheel speeds, encoder counts, distance,
 *	                         line sensors, last barcode, pitch, motion, highest
 *	                         hump, forward range, turn rate, faults
 *	  event      on change   state changes, barcodes, junctions, humps,
 *	                         obstacle profiles, navigation decisions, faults,
 *	                         command acknowledgements, the run summary
 */

#ifndef TEL_JSON_H
#define TEL_JSON_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_evt.h"

#define TEL_JSON_MAX	200U

/* ---- a tiny bounded JSON object builder ---------------------------------- */
typedef struct {
	char		*buf;
	uint32_t	cap;
	uint32_t	len;
	bool		ok;		/* false once anything did not fit */
	bool		first;
} TJ;

void	tj_begin(TJ *j, char *buf, uint32_t cap);
void	tj_int(TJ *j, const char *key, int32_t v);
void	tj_uint(TJ *j, const char *key, uint32_t v);
void	tj_str(TJ *j, const char *key, const char *s);
void	tj_char(TJ *j, const char *key, char c);
void	tj_pair(TJ *j, const char *key, int32_t a, int32_t b);
int32_t	tj_end(TJ *j);		/* length, or -1 if it did not fit */

/* ---- everything the messages need, gathered by the Telemetry task -------- */
typedef struct {
	uint32_t	t_ms;
	int32_t		state;		/* VC_STATE */
	int32_t		why;
	int32_t		v[2];		/* wheel speeds mm/s */
	int32_t		enc[2];		/* encoder counts */
	int32_t		odo_mm;
	int32_t		yaw_cdeg_s;
	bool		line_l;
	bool		line_r;
	char		barcode;	/* last decoded, '-' if none */
	int32_t		pending;	/* LB_CMD_... */
	int32_t		pitch_cdeg;
	int32_t		motion;		/* MOTION_... */
	int32_t		hmax_mm;
	int32_t		range_mm;
	uint32_t	faults;		/* latched FLT_ bits */
	int32_t		mc_fault;	/* MC_FLT_... */
	bool		imu_ok;
	int32_t		heading_cdeg;
	uint32_t	pings;
	int32_t		wifi;		/* CYW43 link status */
	bool		mqtt_up;
	uint32_t	mqtt_connects;
	uint32_t	tx_drops;	/* messages dropped: mqtt_tx_mbf full */
	uint32_t	rx_drops;	/* commands dropped */
	uint32_t	net_errors;	/* commands too long, publishes refused */
} TEL_DATA;

/* Each returns the JSON length, or -1 if it would not fit in cap bytes */
int32_t	tel_fmt_telemetry(const TEL_DATA *d, char *buf, uint32_t cap);
int32_t	tel_fmt_status(const TEL_DATA *d, char *buf, uint32_t cap);

/* An event record (APP_MSG from telemetry_q).  Returns 0 for types that
   are not published. */
int32_t	tel_fmt_event(const APP_MSG *m, char *buf, uint32_t cap);

/* Command acknowledgement: seq, accepted, cmd_err_t */
int32_t	tel_fmt_ack(uint32_t t_ms, uint32_t seq, bool ok, int32_t err, char *buf, uint32_t cap);

/* Obstacle profile (EV_PROFILE_DATA) */
int32_t	tel_fmt_profile(uint32_t t_ms, bool valid, int32_t x_left, int32_t x_right,
			int32_t closest, int32_t closest_deg, int32_t clear_l, int32_t clear_r,
			uint32_t unknown_l, uint32_t unknown_r, char *buf, uint32_t cap);

/* Run summary (NAV_FINISH) */
typedef struct {
	uint32_t	t_ms;
	uint32_t	time_ms;	/* START to FINISH */
	int32_t		dist_mm;
	int32_t		hmax_mm;
	uint32_t	humps;
	uint32_t	commands;
	uint32_t	obstacles;
	uint32_t	reads_ok;
	uint32_t	reads_failed;
} TEL_SUMMARY;

int32_t	tel_fmt_summary(const TEL_SUMMARY *s, char *buf, uint32_t cap);

#endif /* TEL_JSON_H */
