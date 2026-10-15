/*
 *	app_evt.h
 *	Event numbers and the 16-byte event record shared by every subsystem.
 *	Changed only by team agreement (like app.h).
 *
 *	Pure C: only <stdint.h> and <stdbool.h>, no kernel headers, so the
 *	logic modules (*_core.c) that use it also compile on a laptop for the
 *	host tests in testing/host/.
 */

#ifndef APP_EVT_H
#define APP_EVT_H

#include <stdint.h>
#include <stdbool.h>

/* One event: 16 bytes, plain data, no pointers.  Sent by value through
   the kernel message buffers vc_event_q and telemetry_q. */
typedef struct {
	uint8_t		type;		/* EV_... below */
	uint8_t		source;		/* SRC_... below */
	uint16_t	seq;		/* per-sender sequence number (wraps) */
	uint32_t	t_ms;		/* kernel time when the event was made */
	int32_t		p1;		/* payload; meaning depends on type */
	int32_t		p2;
} APP_MSG;

/* ---- event types: [p1, p2] ------------------------------------------- */
#define EV_LINE_STATUS	1U	/* [left on black 0/1, right on black 0/1]   S3 -> VC   */
#define EV_LINE_LOST	2U	/* [side last seen: -1 left, +1 right, 0]    S3 -> VC   */
#define EV_JUNCTION	3U	/* [odometer mm, -]                          S3 -> VC   */
#define EV_BARCODE	4U	/* [character, LB_CMD_...]                   S3 -> VC   */
#define EV_BARCODE_BAD	5U	/* [C39_... reason, elements captured]      S3 -> VC   */
#define EV_DONE		6U	/* [MC_MOVE_..., move id]                    S2 -> VC   */
#define EV_STOP_ACK	7U	/* [stop latency us, -]                      S2 -> VC   */
#define EV_FAULT	8U	/* [fault bits FLT_..., detail]              S4/S2 -> VC */
#define EV_FAULT_CLEAR	9U	/* [-, -] every fault cleared after RESTART   S4 -> VC   */
#define EV_ALERT	10U	/* [range mm, watch id] obstacle ahead      S5 scan -> VC */
#define EV_RANGE_ZONE	11U	/* [0 far / 1 near, watch id]               S5 scan -> VC */
#define EV_PROFILE	12U	/* [valid 0/1, request id]                  S5 scan -> VC */
#define EV_TERRAIN	13U	/* [TERRAIN_..., pitch centidegrees]         S4 -> VC   */
#define EV_HUMP		14U	/* [hump height mm, highest of the run mm]   S4 -> VC   */
#define EV_MOTION	15U	/* [MOTION_..., -] motion class changed      S4 -> telemetry */
#define EV_IMPACT	16U	/* [horizontal jump mg, speed before mm/s]   S4 -> VC   */
#define EV_BUTTON	17U	/* [BTN_START or BTN_STOP, -]                S4 -> VC   */
#define EV_COMMAND	18U	/* [CMD_..., value]                          S1 -> VC   */
#define EV_STATE	19U	/* [VC state, reason]                        VC -> telemetry */
#define EV_NAV		20U	/* [NAV_..., value] a navigation decision    VC -> telemetry */
#define EV_PROFILE_DATA	21U	/* [-, -] profile ready to publish       S5 scan -> telemetry */
#define EV_TYPE_MAX	21U

/* EV_COMMAND.p1: validated remote (MQTT) commands */
#define CMD_START	1
#define CMD_STOP	2
#define CMD_RESTART	3
#define CMD_FINISH	4

/* EV_BUTTON.p1: the Robo Pico user buttons (active low) */
#define BTN_START	20	/* GP20: START, or RESTART after a fault */
#define BTN_STOP	21	/* GP21: STOP */

/* Sources */
#define SRC_TELEMETRY	1U
#define SRC_MOTION	2U
#define SRC_LINE	3U
#define SRC_IMU		4U
#define SRC_SAFETY	5U
#define SRC_SCAN	6U
#define SRC_VC		7U

/* A short list of events produced by one step of a logic module.  The
   task that called the step sends them out afterwards. */
#define EVT_LIST_MAX	4U

typedef struct {
	uint32_t	n;
	APP_MSG		ev[EVT_LIST_MAX];
} EVT_LIST;

void	evt_list_clear(EVT_LIST *l);
bool	evt_list_add(EVT_LIST *l, uint8_t type, int32_t p1, int32_t p2);	/* false if full */

#endif /* APP_EVT_H */
