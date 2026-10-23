/*
 *	subsystem3.h
 *	Subsystem 3 -- Line and Barcode (Buddy 3).  Student ID: 2500337.
 *	PUBLIC interface.
 *
 *	Owns the three IR modules: left line DO (GP6), right line DO (GP7),
 *	barcode AO (GP26, ADC0) and barcode DO (GP27, edge interrupt).
 *
 *	Events sent to vc_event_q (app_evt.h):
 *	  EV_LINE_STATUS  p1 = left on black (0/1), p2 = right on black (0/1), on change
 *	  EV_LINE_LOST    p1 = side last seen: -1 left, +1 right, 0 unknown
 *	  EV_JUNCTION     p1 = odometer mm; both sensors on black
 *	  EV_BARCODE      p1 = character, p2 = LB_CMD_... (barcode_core.h)
 *	  EV_BARCODE_BAD  p1 = C39_... reason, p2 = elements captured
 *
 *	The line-following rule itself (line_core.h, ls_*) is called by
 *	Vehicle Controller with lb_steer_params.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../app_msg.h"
#include "line_core.h"
#include "barcode_core.h"

#define S3_PRI_LINE		8	/* core 1 */

/* The line-following rule's tuning (line_cfg.h), for Vehicle Controller */
IMPORT const LS_PARAMS	lb_steer_params;

/* Latest line and barcode state, for telemetry: shared as one packed
   word (LC_STATUS in line_core.h), unpacked by lb_get_status(). */
typedef LC_STATUS LB_STATUS;

IMPORT void	lb_get_status(LB_STATUS *out);

/* ---- private to subsystem 3 ---- */
IMPORT ID	tid_line;
IMPORT ID	cyc_line;
IMPORT void	line_task(INT stacd, void *exinf);
IMPORT void	line_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM3_H */
