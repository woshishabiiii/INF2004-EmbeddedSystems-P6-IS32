/*
 *	app_msg.h
 *	Kernel side of the shared event format (team agreement, like app.h).
 *	The record itself and the event numbers are in app_evt.h, which has
 *	no kernel headers so the logic modules can include it too.
 */

#ifndef APP_MSG_H
#define APP_MSG_H

#include "app.h"
#include "app_evt.h"

/* Bytes of message-buffer space for n messages of size sz: the kernel
   stores a 4-byte header with each message and rounds every message up
   to a multiple of 4 bytes (a 23-byte record takes 24 + 4). */
#define MBF_BYTES(n, sz)	((n) * ((((sz) + 3U) & ~3U) + 4U))

/* Send one event to a message buffer WITHOUT waiting (TMO_POL).
   Fills in source, sequence number (*seq, then incremented, wraps at 256)
   and time, and packs it into the 8-byte APP_MSG (app_evt.h): a value
   outside its field's range is saturated, never wrapped.
   Returns E_OK, or the kernel error (E_TMOUT when the buffer is full). */
IMPORT ER app_send(ID mbf, UB source, UB *seq, UB type, W p1, W p2);

/* Same, for every event in a list made by a logic module.  Returns E_OK
   only if every send succeeded. */
IMPORT ER app_send_list(ID mbf, UB source, UB *seq, const EVT_LIST *l);

/* Every event in a list, with the queue-overflow policy of vc_event_q
   (evt_is_safety() in app_evt.h): a non-safety event that does not fit is
   dropped and counted (app_dropped()); *dropped (may be NULL) gets bit i
   set for each dropped event i.  Returns E_OK, or the kernel error if a
   SAFETY event could not be sent -- the caller then raises FLT_QUEUE_OVF. */
IMPORT ER app_send_events(ID mbf, UB source, UB *seq, const EVT_LIST *l, UW *dropped);

/* One event with the same policy.  Returns E_OK (sent, or a non-safety
   event dropped and counted -- *was_dropped tells which, may be NULL), or
   the kernel error for a safety event that could not be sent. */
IMPORT ER app_send_event(ID mbf, UB source, UB *seq, UB type, W p1, W p2, BOOL *was_dropped);

/* Non-safety events dropped so far (all senders) */
IMPORT UW app_dropped(void);

/* ---------------------------------------------------------------------
 *  Task timing (design-review feedback IS32: "estimate WCETs and show the
 *  5 ms and 20 ms deadlines hold").  Each task measures its own run time
 *  from wake-up to sleep with the 1 us timer and counts releases it
 *  missed.  One writer per slot, so no lock.  Published every 5 s by
 *  Telemetry ("tim" event) or printed on the console.
 * --------------------------------------------------------------------- */
#define TIM_MOTION	0U
#define TIM_LINE	1U
#define TIM_VC		2U
#define TIM_SCAN	3U
#define TIM_IMU		4U
#define TIM_SAFETY	5U
#define TIM_TELEMETRY	6U
#define TIM_SLOTS	7U

typedef struct {				/* 16 bits each, as in the timing record; stop at 65,535 */
	UH	max_us[TIM_SLOTS];	/* longest run, wake-up to sleep */
	UH	overruns[TIM_SLOTS];	/* releases missed (task still busy) */
} APP_TIMING;

IMPORT void app_task_time(UINT slot, UW start_us);	/* call just before sleeping */
IMPORT void app_task_overrun(UINT slot, UW missed);
IMPORT void app_timing_get(APP_TIMING *out);

#endif /* APP_MSG_H */
