/*
 *	subsystem1.h
 *	Subsystem 1 -- Telemetry and Command (Buddy 1).  PUBLIC interface.
 *
 *	This subsystem never calls cyw43, lwIP or MQTT functions: the kernel's
 *	cyw43_utk_task owns the radio and the network stack.  The two talk
 *	through mqtt_tx_mbf and mqtt_rx_mbf (net_bridge.c, called from
 *	lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c).
 *
 *	Other subsystems report to telemetry by sending one APP_MSG to
 *	telemetry_q with TMO_POL.  A full queue drops the record (and counts
 *	it): telemetry must never delay control.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "../app_msg.h"

#define S1_PRI_TELEMETRY	14	/* lowest application priority, core 0 */

IMPORT ID	telemetry_q;

/* Tunable values accepted by "SET" while the car is in READY or FINISHED
   (whitelist and limits: cmd_parse.c).  Each owner copies its values when
   START arrives.  PARAM_UNSET = never SET: the owner keeps its default. */
#define PARAM_BASE_SPEED	0U	/* Vehicle Controller */
#define PARAM_STOP_MM		1U	/* Vehicle Controller */
#define PARAM_KP_X1000		2U	/* Motion Control */
#define PARAM_KI_X1000		3U	/* Motion Control */
#define PARAM_DIST_CAL		4U	/* Motion Control */
#define PARAM_TURN_CAL		5U	/* Motion Control */
#define PARAM_UNSET		(-1)

IMPORT W	param_get(UINT index);

/* ---- private to subsystem 1 ---- */
#define TOPIC_STATUS		0U
#define TOPIC_TELEMETRY		1U
#define TOPIC_EVENT		2U	/* the last topic number */

#define NET_PAYLOAD_MAX		34U	/* bytes per MQTT message: TEL_BIN_MAX (tel_bin.h) */
#define NET_CMD_MAX		4U	/* bytes per command: CMD_BYTES (cmd_parse.h) */

IMPORT ID		mqtt_tx_mbf, mqtt_rx_mbf, tid_telemetry;
/* Written only by the network task (net_bridge.c), read by Telemetry.
   The counters stop at 255 (8 bits, as in the status record).  Whether
   the link is up is NOT a separate variable: it is the FLT_WIFI_DOWN bit
   in fault_flags (set while down, never latched). */
IMPORT volatile UB	net_rx_drops;	/* commands lost (buffer full or too long) */
IMPORT volatile UB	net_errors;	/* commands too long, or publishes refused by lwIP */
IMPORT volatile UB	net_connects;	/* MQTT connections made */
IMPORT volatile UW	net_reconnect_ms;	/* last loss -> connected again, ms (0 = none); 18 bits
					   on the wire (262 s), so a 32-bit word: one store */
IMPORT const char	*net_topic_name(UINT topic);
IMPORT void		telemetry_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM1_H */
