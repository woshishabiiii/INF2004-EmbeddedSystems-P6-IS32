/*
 *	app_evt.h
 *	Event numbers, the 8-byte packed event record (APP_MSG) shared by
 *	every subsystem, and its pack/unpack functions.  Changed only by team
 *	agreement (like app.h).  Owner: Buddy 5.
 *
 *	Pure C: only <stdint.h> and <stdbool.h>, no kernel headers, so the
 *	logic modules (*_core.c) that use it also compile on a laptop for the
 *	host tests in testing/host/.
 *
 *	Minimum-bit data representation (Tribunal 1 change requirement):
 *	  APP_MSG (in vc_event_q and telemetry_q) is 64 bits:
 *	    word 0  bits 0-4    type     5 bits  (21 event types)
 *	            bits 5-7    source   3 bits  (7 senders)
 *	            bits 8-15   seq      8 bits  per-sender sequence, wraps
 *	            bits 16-31  t_ms    16 bits  kernel ms, wraps every 65.5 s:
 *	                                         subtract only (bp_age16)
 *	    word 1  payload     32 bits  p1 in the low bits, p2 above it, each
 *	                                 with the width and offset of its event
 *	                                 type (s_layout[] in app_evt.c and the
 *	                                 table below); values outside the range
 *	                                 SATURATE, they never wrap.
 *	  Each slot of a kernel message buffer is 8 + 4 (header) = 12 B, was 20 B.
 *	  The logic modules work on the unpacked APP_EVT; only the tasks that
 *	  send and receive call app_msg_pack() / app_msg_unpack().
 */

#ifndef APP_EVT_H
#define APP_EVT_H

#include <stdint.h>
#include <stdbool.h>

/* ---- event types: [p1, p2]  (bits: p1 + p2) ------------------------------ */
#define EV_LINE_STATUS	1U	/* [left on black 0/1, right on black 0/1]  1+1   S3 -> VC   */
#define EV_LINE_LOST	2U	/* [side last seen: -1 left, +1 right, 0]   2     S3 -> VC   */
#define EV_JUNCTION	3U	/* [odometer mm 0..16.7 km, -]              24    S3 -> VC   */
#define EV_BARCODE	4U	/* [character ' '..'_', LB_CMD_...]         6+3   S3 -> VC   */
#define EV_BARCODE_BAD	5U	/* [C39_... or BC_REASON_OVERFLOW, elements] 7+6  S3 -> VC   */
#define EV_DONE		6U	/* [MC_MOVE_..., move id 1..255]            2+8   S2 -> VC   */
#define EV_STOP_ACK	7U	/* [stop latency us, stopping distance mm]  17+10 S2 -> VC   */
#define EV_FAULT	8U	/* [fault bits FLT_ALL, -]                  10    S4 -> VC   */
#define EV_FAULT_CLEAR	9U	/* [-, -] every fault cleared after RESTART  0    S4 -> VC   */
#define EV_ALERT	10U	/* [range mm, watch id 1..255]              12+8  S5 scan -> VC */
#define EV_RANGE_ZONE	11U	/* [0 far / 1 near, watch id]               1+8   S5 scan -> VC */
#define EV_PROFILE	12U	/* [valid 0/1, request id]                  1+8   S5 scan -> VC */
#define EV_TERRAIN	13U	/* [TERRAIN_..., pitch centidegrees +-81.9] 2+14  S4 -> VC   */
#define EV_HUMP		14U	/* [hump height mm, highest of the run mm]  8+8   S4 -> VC   */
#define EV_MOTION	15U	/* [MOTION_..., -] motion class changed     3     S4 -> telemetry */
#define EV_IMPACT	16U	/* [jump mg (magnitude), speed before mm/s] 12+9  S4 -> VC   */
#define EV_BUTTON	17U	/* [BTN_START or BTN_STOP, -] (stored - 20) 1     S4 -> VC   */
#define EV_COMMAND	18U	/* [CMD_..., -]                             3     S1 -> VC   */
#define EV_STATE	19U	/* [VC state, reason]                       4+4   VC -> telemetry */
#define EV_NAV		20U	/* [NAV_..., value +-16.7 km]               4+25  VC -> telemetry */
#define EV_PROFILE_DATA	21U	/* [-, -] profile ready to publish          0     S5 scan -> telemetry */
#define EV_TYPE_MAX	21U

/* EV_COMMAND.p1: validated remote (MQTT) commands */
#define CMD_START	1
#define CMD_STOP	2
#define CMD_RESTART	3
#define CMD_FINISH	4

/* EV_BUTTON.p1: the Robo Pico user buttons (active low) */
#define BTN_START	20	/* GP20: START, or RESTART after a fault */
#define BTN_STOP	21	/* GP21: STOP */

/* Sources (3 bits) */
#define SRC_TELEMETRY	1U
#define SRC_MOTION	2U
#define SRC_LINE	3U
#define SRC_IMU		4U
#define SRC_SAFETY	5U
#define SRC_SCAN	6U
#define SRC_VC		7U

/* Request and move ids carried in events: 8 bits, 1..255, 0 = none.
   Ids are only compared for equality, and at most a few are in flight. */
#define APP_ID_BITS	8U
#define APP_ID_MAX	255

static inline int32_t app_id_next(int32_t id)
{
	return ((id <= 0) || (id >= APP_ID_MAX)) ? 1 : (id + 1);
}

/* One event, unpacked: what the logic modules see */
typedef struct {
	uint8_t		type;		/* EV_... above */
	uint8_t		source;		/* SRC_... above */
	uint8_t		seq;		/* per-sender sequence number (wraps) */
	uint16_t	t_ms;		/* low 16 bits of the kernel time when the event was
					   made: only age = bp_age16(now, t_ms) is meaningful */
	int32_t		p1;		/* payload; meaning depends on type */
	int32_t		p2;
} APP_EVT;

/* One event, packed: what goes through the kernel message buffers */
typedef struct {
	uint32_t	hdr;		/* type | source << 5 | seq << 8 | t_ms << 16 */
	uint32_t	pl;		/* p1 | p2 << (width of p1) */
} APP_MSG;

void	app_msg_pack(const APP_EVT *e, APP_MSG *m);
void	app_msg_unpack(const APP_MSG *m, APP_EVT *e);

/* Bits of p1 and p2 for a type, and the offset (smallest value) of each
   (unit tests, decoder cross-check) */
uint32_t app_evt_bits(uint8_t type, uint32_t which);
int32_t	app_evt_min(uint8_t type, uint32_t which);

/* A short list of events produced by one step of a logic module.  The
   task that called the step sends them out afterwards. */
#define EVT_LIST_MAX	4U

typedef struct {
	uint32_t	n;
	APP_EVT		ev[EVT_LIST_MAX];
} EVT_LIST;

void	evt_list_clear(EVT_LIST *l);
bool	evt_list_add(EVT_LIST *l, uint8_t type, int32_t p1, int32_t p2);	/* false if full */

/* Queue-overflow policy for vc_event_q (design-review feedback IS32).
   A SAFETY event must reach Vehicle Controller: if the queue is full the
   sender raises FLT_QUEUE_OVF (Safe Stop).  Every other event only
   informs the mission: if it does not fit it is dropped and counted, and
   a newer event of the same kind (or the controller's own time limits)
   covers the loss.  Safety events: FAULT, FAULT_CLEAR, IMPACT, ALERT,
   STOP_ACK, BUTTON, COMMAND.
   Loss is detected by the drop counter and FLT_QUEUE_OVF, not by the
   8-bit sequence number, which only orders consecutive messages. */
bool	evt_is_safety(uint8_t type);

#endif /* APP_EVT_H */
