/*
 *	subsystem5.h
 *	Subsystem 5 -- Obstacle Scan + Vehicle Controller (Buddy 5).
 *	PUBLIC interface.
 *
 *	Obstacle Scan owns the HC-SR04 (TRIG GP0, ECHO GP1) and the SG90 scan
 *	servo (GP12, PWM slice 6).  Vehicle Controller owns the mission state
 *	and is the ONLY sender of motion commands and scan requests.
 *
 *	vc_event_q is every subsystem's way to talk to Vehicle Controller.
 *	Send without waiting, with the overflow policy (app_msg.h):
 *	    er = app_send_event(vc_event_q, SRC_..., &seq, EV_..., p1, p2, NULL);
 *	or app_send_events() for a list; if er != E_OK (a SAFETY event did not
 *	fit) set FLT_QUEUE_OVF in fault_flags.  Other events that do not fit
 *	are dropped and counted.
 *
 *	Size (design-review feedback IS32): worst-case producers are line
 *	status 200/s (one per 5 ms sample, sent only on change), motion and
 *	Safety Monitor 50/s each (20 ms), Obstacle Scan 15/s (70 ms pings),
 *	commands a few per second -- under 400 events/s in total.  Vehicle
 *	Controller (core 1, priority 9) handles an event in about 350
 *	instructions of logic, so a backlog builds only while Motion Control
 *	or Line pre-empt it (well under 1 ms, at most a few events).  32 slots
 *	(640 B) is more than 10x that; VC_STATUS.vq_high logs the real peak.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../app_msg.h"
#include "vc_core.h"		/* VC_STATE, vc_state_name() */
#include "scan_core.h"		/* SCAN_..., OS_PROFILE */

#define S5_PRI_VC		9	/* core 1 */
#define S5_PRI_SCAN		10	/* core 0 */
#define VC_EVENT_SLOTS		32U

IMPORT ID	vc_event_q;

/* Latest Vehicle Controller state, for telemetry: shared as packed words
   (VC_ST in vc_core.h), unpacked by vc_get_status(). */
typedef VC_ST VC_STATUS;

/* Latest Obstacle Scan state, for telemetry: shared as one packed word
   (SCAN_ST in scan_core.h), unpacked by scan_get_status(). */
typedef SCAN_ST SCAN_STATUS;

IMPORT void	vc_get_status(VC_STATUS *out);
IMPORT void	scan_get_status(SCAN_STATUS *out);
/* The last profile: copies the packed profile under mtx_profile (2 ms
   timeout) and unpacks it.  Returns E_OK or a kernel error (then *out is
   not changed). */
IMPORT ER	scan_profile_get(OS_PROFILE *out);

/* ---- private to subsystem 5 ---- */
IMPORT ID	scan_request_q;
IMPORT ID	tid_scan, tid_vc, cyc_scan;
IMPORT ID	mtx_profile;
IMPORT ER	scan_request(UW mode, W stop_mm, W id);	/* VC -> Obstacle Scan, TMO_POL */
IMPORT void	scan_task(INT stacd, void *exinf);
IMPORT void	scan_cyclic_handler(void *exinf);
IMPORT void	vc_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM5_H */
