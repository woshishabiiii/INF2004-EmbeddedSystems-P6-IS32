/*
 *	motion_api.c
 *	The motion API (subsystem2.h).  Each call only queues a command or
 *	sets a stop bit; the Motion Control task does the work.  Callers never
 *	touch the motor pins.
 */

#include "subsystem2.h"
#include "motion_cfg.h"

EXPORT volatile UW	mc_stop_req_us;		/* time of the last stop request (NFR3 log) */

LOCAL ER send_cmd(UB type, W a, W b, W id)
{
	MC_CMD cmd = {0};

	cmd.type = type;
	cmd.a = a;
	cmd.b = b;
	cmd.c = id;
	if (!mc_cmd_valid(&cmd, MC_MAX_SPEED)) {
		return E_PAR;
	}
	/* Never wait: a full queue is an error for the caller to handle */
	return tk_snd_mbf(motion_cmd_q, &cmd, (INT)sizeof(cmd), TMO_POL);
}

LOCAL ER wake_motion(void)
{
	ER er = tk_wup_tsk(tid_motion);		/* act now, not at the next tick */

	return (er == E_QOVR) ? E_OK : er;	/* a wake-up already pending is fine */
}

EXPORT ER motion_move_forward(W distance_mm, W speed_mm_s, W id)
{
	return (distance_mm > 0) ? send_cmd(MC_CMD_DISTANCE, distance_mm, speed_mm_s, id) : E_PAR;
}

EXPORT ER motion_move_backward(W distance_mm, W speed_mm_s, W id)
{
	return (distance_mm > 0) ? send_cmd(MC_CMD_DISTANCE, -distance_mm, speed_mm_s, id) : E_PAR;
}

EXPORT ER motion_turn_left(W angle_deg, W id)
{
	return (angle_deg > 0) ? send_cmd(MC_CMD_TURN, angle_deg, 0, id) : E_PAR;
}

EXPORT ER motion_turn_right(W angle_deg, W id)
{
	return (angle_deg > 0) ? send_cmd(MC_CMD_TURN, -angle_deg, 0, id) : E_PAR;
}

EXPORT ER motion_hold_speed(W left_mm_s, W right_mm_s)
{
	return send_cmd(MC_CMD_SPEED, left_mm_s, right_mm_s, 0);
}

EXPORT ER motion_stop(void)
{
	ER er;

	mc_stop_req_us = app_us();
	er = tk_set_flg(stop_flags, MC_CMD_STOP);
	return (er == E_OK) ? wake_motion() : er;
}

EXPORT ER motion_clear_stop(void)
{
	return tk_clr_flg(stop_flags, ~MC_CMD_STOP);	/* clears the bits that are 0 here */
}

EXPORT ER motion_fault_stop(void)
{
	ER er;

	mc_stop_req_us = app_us();
	er = tk_set_flg(stop_flags, MC_FAULT_STOP);
	return (er == E_OK) ? wake_motion() : er;
}

EXPORT ER motion_clear_fault_stop(void)
{
	return tk_clr_flg(stop_flags, ~MC_FAULT_STOP);
}
