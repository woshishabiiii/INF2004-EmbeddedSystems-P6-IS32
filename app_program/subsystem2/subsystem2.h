/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (Buddy 2).  Student ID: 2500518.
 *	PUBLIC interface: what the other subsystems may use.
 *
 *	Motion Control owns both motors (M1 = right: GP8/GP9, PWM slice 4;
 *	M2 = left: GP10/GP11, slice 5) and both encoders (left GP2/GP3, right
 *	GP16/GP17).  No other code touches those pins.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "../app_msg.h"
#include "mc_core.h"		/* MC_SNAPSHOT, MC_MOVE_..., MC_FLT_... */

#define S2_PRI_MOTION		6	/* highest application priority, core 1 */

/* stop_flags bits (event flag created by Motion Control) */
#define MC_CMD_STOP		0x01U	/* set / cleared by Vehicle Controller */
#define MC_FAULT_STOP		0x02U	/* set by Safety Monitor or Motion Control;
					   cleared only by Safety Monitor */

IMPORT ID	stop_flags;
IMPORT ID	tid_motion;

/* ---- Motion API --------------------------------------------------------
 * Each call only queues a command on motion_cmd_q (never waits) or sets a
 * stop bit; the Motion Control task does the work in its next 20 ms
 * period.  Returns E_OK, E_PAR for a bad argument, or E_TMOUT when the
 * queue is full.  A new move REPLACES the one in progress.
 *   id: any number the caller chooses; it comes back in EV_DONE.p2 so the
 *       caller knows which move finished (0 if it does not matter).
 * Vehicle Controller is the only caller of the move functions.  Safety
 * Monitor is the only caller of motion_fault_stop/motion_clear_fault_stop. */
IMPORT ER	motion_move_forward(W distance_mm, W speed_mm_s, W id);
IMPORT ER	motion_move_backward(W distance_mm, W speed_mm_s, W id);
IMPORT ER	motion_turn_left(W angle_deg, W id);	/* spin on the spot */
IMPORT ER	motion_turn_right(W angle_deg, W id);
IMPORT ER	motion_hold_speed(W left_mm_s, W right_mm_s);	/* until replaced */
IMPORT ER	motion_stop(void);		/* set CMD_STOP and wake Motion Control */
IMPORT ER	motion_clear_stop(void);	/* clear CMD_STOP */
IMPORT ER	motion_fault_stop(void);	/* set FAULT_STOP and wake Motion Control */
IMPORT ER	motion_clear_fault_stop(void);	/* clear FAULT_STOP */

/* Copy of the latest odometry snapshot (dist_snapshot).  Locks the mutex
   for the copy only, 2 ms timeout.  Returns E_OK or a kernel error. */
IMPORT ER	motion_get_snapshot(MC_SNAPSHOT *out);

/* ---- private to subsystem 2 ---- */
IMPORT ID	motion_cmd_q;
IMPORT ID	cyc_motion;
IMPORT ID	mtx_snapshot;
IMPORT void	motion_task(INT stacd, void *exinf);
IMPORT void	motion_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM2_H */
