/*
 *	tel_bin.h
 *	MQTT payloads as compact binary records (Buddy 1), replacing the JSON
 *	text of tel_json.c (Tribunal 1 change requirement: minimum-bit data,
 *	compact binary MQTT).  Pure C: no kernel calls, unit-tested on a
 *	laptop, and tools/mqtt_decode.py reads exactly these layouts.
 *
 *	Every record is a byte array written field by field with masks and
 *	shifts (bitpack.h): LSB first, so bit 0 of the record is bit 0 of
 *	byte 0, and multi-byte fields are little-endian.  No C structs and no
 *	C bit-fields go on the wire (their order and padding are up to the
 *	compiler).  A signed field is sent as value - min, so every field on
 *	the wire is unsigned; a value outside a field's range SATURATES.
 *	Every record is the fewest whole bytes that hold its fields: each
 *	packer checks that its last field ends in the record's last byte
 *	(finish() in tel_bin.c), so a size and its layout cannot disagree.
 *
 *	Common header (32 bits):
 *	  bits 0-3    version     TEL_BIN_VERSION; the decoder rejects others
 *	  bits 4-7    type        TEL_T_...
 *	  bits 8-31   t_ms        uptime in ms, low 24 bits (wraps every 4.66 h)
 *
 *	TEL_T_TELEMETRY  (topic telemetry, every 0.5 s)  182 bits = 23 B
 *	  state 4 | v left 10 (-512) | v right 10 (-512) | enc left 16 | enc right 16
 *	  (low 16 bits of each position) | odo_mm 24 | line 2 (bit 0 left on black,
 *	  bit 1 right) | barcode 6 (char - 32; '-' = none) | pending 3 (LB_CMD_) |
 *	  pitch_cdeg 14 (-8192) | motion 3 | hmax_mm 8 | range_mm 12 (4,095 = no
 *	  echo) | yaw deg/s 10 (-512) | faults 12
 *	TEL_T_STATUS     (topic status, every 1 s)        142 bits = 18 B
 *	  state 4 | why 4 | faults 12 | mc_fault 3 (MC_FLT_ bits) | imu_ok 1 |
 *	  heading deg 9 (-256) | pings 16 | wifi link 3 (-4) | connects 8 |
 *	  tx_drops 16 | rx_drops 8 | net_errors 8 |
 *	  reconnect_ms 18 (how long the last outage lasted, 0 = none yet)
 *	  MQTT up or down is the WIFI_DOWN bit of faults, not a field of its own.
 *	TEL_T_EVENT      (topic event)                     69 bits =  9 B
 *	  event type 5 | payload 32 (p1 and p2 exactly as in APP_MSG: app_evt.h)
 *	TEL_T_ACK        (topic event)                     44 bits =  6 B
 *	  seq 8 | accepted 1 | error 3 (cmd_err_t)
 *	TEL_T_PROFILE    (topic event)                    136 bits = 17 B
 *	  the 97-bit OS_PROFILE_PK (os_profile.h), copied as its 13 bytes
 *	TEL_T_SUMMARY    (topic event)                    128 bits = 16 B
 *	  time_ms 24 | dist_mm 25 (-16,777,216) | hmax_mm 8 | humps 8 | commands 8 |
 *	  obstacles 7 | barcodes ok 8 | barcodes failed 8
 *	TEL_T_TIMING     (topic event, every 5 s)         269 bits = 34 B
 *	  max_us 16 x 7 (motion, line, vc, scan, imu, safety, telemetry) |
 *	  overruns 16 | irq_per_s 16 | irq_max_us 16 | vq_high 6 | dropped 16 |
 *	  steer_lat_max_ms 10 | stop_lat_max_us 17 | stop_dist_mm 10 | reconnect_ms 18
 *
 *	faults (12 bits): bits 0-9 = the latched FLT_ALL bits, bit 10 = FLT_NO_ECHO,
 *	bit 11 = FLT_WIFI_DOWN (status bits, never latched): tel_fault12().
 */

#ifndef TEL_BIN_H
#define TEL_BIN_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_evt.h"
#include "../subsystem5/os_profile.h"

#define TEL_BIN_VERSION		1U

#define TEL_T_TELEMETRY		1U
#define TEL_T_STATUS		2U
#define TEL_T_EVENT		3U
#define TEL_T_ACK		4U
#define TEL_T_PROFILE		5U
#define TEL_T_SUMMARY		6U
#define TEL_T_TIMING		7U

#define TEL_BIN_TELEMETRY_BYTES	23U
#define TEL_BIN_STATUS_BYTES	18U
#define TEL_BIN_EVENT_BYTES	9U
#define TEL_BIN_ACK_BYTES	6U
#define TEL_BIN_PROFILE_BYTES	17U
#define TEL_BIN_SUMMARY_BYTES	16U
#define TEL_BIN_TIMING_BYTES	34U
#define TEL_BIN_MAX		34U	/* the largest record */

/* fault_flags bits -> the 12-bit wire field */
uint32_t tel_fault12(uint32_t flags);

/* ---- everything the periodic records need, gathered by the Telemetry task */
typedef struct {
	uint32_t	t_ms;
	int32_t		state;		/* VC_STATE */
	int32_t		why;
	int32_t		v[2];		/* wheel speeds mm/s */
	int32_t		enc[2];		/* encoder positions (low 16 bits are sent) */
	int32_t		odo_mm;
	int32_t		yaw_cdeg_s;
	bool		line_l;
	bool		line_r;
	char		barcode;	/* last decoded, '-' if none */
	int32_t		pending;	/* LB_CMD_... */
	int32_t		pitch_cdeg;
	int32_t		motion;		/* MOTION_... */
	int32_t		hmax_mm;
	int32_t		range_mm;	/* 4,095 = no echo */
	uint32_t	faults;		/* fault_flags bits: latched FLT_ALL + FLT_STATUS */
	int32_t		mc_fault;	/* MC_FLT_... bits */
	bool		imu_ok;
	int32_t		heading_cdeg;
	uint32_t	pings;
	int32_t		wifi;		/* CYW43 link status, -3..3 */
	uint32_t	mqtt_connects;
	uint32_t	tx_drops;	/* messages dropped: mqtt_tx_mbf full */
	uint32_t	rx_drops;	/* commands dropped */
	uint32_t	net_errors;	/* commands of the wrong size, publishes refused */
	uint32_t	reconnect_ms;	/* last loss -> connected again */
} TEL_DATA;

/* Each returns the record length in bytes, or -1 if cap is too small */
int32_t	tel_bin_telemetry(const TEL_DATA *d, uint8_t *buf, uint32_t cap);
int32_t	tel_bin_status(const TEL_DATA *d, uint8_t *buf, uint32_t cap);

/* An event record (unpacked from telemetry_q); t_ms = its full time.
   Returns 0 for types that are not published. */
int32_t	tel_bin_event(const APP_EVT *m, uint32_t t_ms, uint8_t *buf, uint32_t cap);

/* Command acknowledgement: seq, accepted, cmd_err_t */
int32_t	tel_bin_ack(uint32_t t_ms, uint32_t seq, bool ok, int32_t err, uint8_t *buf, uint32_t cap);

/* Obstacle profile (EV_PROFILE_DATA) */
int32_t	tel_bin_profile(uint32_t t_ms, const OS_PROFILE *p, uint8_t *buf, uint32_t cap);

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

int32_t	tel_bin_summary(const TEL_SUMMARY *s, uint8_t *buf, uint32_t cap);

/* Timing and queue health, every 5 s on the event topic: the measurements
   asked for by the design-review feedback (IS32) -- task run times, missed
   releases, the GPIO interrupt load, vc_event_q's peak backlog and drops,
   steering and stop latency, stopping distance and the reconnect time. */
#define TEL_TIM_TASKS	7U	/* motion, line, vc, scan, imu, safety, telemetry */

typedef struct {
	uint32_t	t_ms;
	uint32_t	max_us[TEL_TIM_TASKS];	/* longest run of each task, us */
	uint32_t	overruns;	/* releases missed, all tasks */
	uint32_t	irq_per_s;	/* GPIO interrupts per second, last period */
	uint32_t	irq_max_us;	/* longest GPIO interrupt handler run */
	uint32_t	vq_high;	/* most events waiting in vc_event_q */
	uint32_t	dropped;	/* non-safety events dropped */
	int32_t		steer_lat_max_ms;	/* line reading -> new wheel targets */
	int32_t		stop_lat_max_us;	/* stop request -> duty 0 */
	int32_t		stop_dist_mm;	/* last stop: CMD_STOP -> wheels stopped */
	uint32_t	reconnect_ms;	/* last MQTT reconnect, 0 = none yet */
} TEL_TIMING;

int32_t	tel_bin_timing(const TEL_TIMING *s, uint8_t *buf, uint32_t cap);

#endif /* TEL_BIN_H */
