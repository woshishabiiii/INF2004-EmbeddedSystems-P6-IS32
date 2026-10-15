/*
 *	task_safety.c
 *	Safety Monitor (Figure 3 of the design review), core 0, priority 7.
 *	The rules are in safety_core.c; this file is the RTOS side.
 *
 *	It waits on fault_flags with a 20 ms timeout, so it reacts to a fault
 *	at once AND still runs every 20 ms to read the buttons and check that
 *	every periodic task is alive.  It runs on core 0, so a runaway task on
 *	core 1 cannot stop it.
 */

#include "subsystem4.h"
#include "imu_cfg.h"
#include "../subsystem2/subsystem2.h"	/* motion_fault_stop() */
#include "../subsystem5/subsystem5.h"	/* vc_event_q */

LOCAL volatile UW	s_health_ms[HEALTH_SLOTS];
LOCAL SM_CORE		s_sm;
LOCAL UH		s_seq;

LOCAL const SM_PARAMS s_params = {
	{
		SM_HEALTH_MOTION_MS,	/* HEALTH_MOTION */
		SM_HEALTH_LINE_MS,	/* HEALTH_LINE */
		SM_HEALTH_IMU_MS,	/* HEALTH_IMU */
		SM_HEALTH_SCAN_MS,	/* HEALTH_SCAN */
		SM_HEALTH_VC_MS		/* HEALTH_VC */
	},
	SM_RESTART_POLLS		/* restart_polls */
};

EXPORT void health_stamp(UINT slot)
{
	if (slot < HEALTH_SLOTS) {
		s_health_ms[slot] = app_ms() | 1U;	/* one writer per slot: no lock; 0 = never */
	}
}

EXPORT BOOL safety_faulted(void)
{
	return s_sm.faulted ? TRUE : FALSE;
}

EXPORT UW safety_latched_bits(void)
{
	return s_sm.latched;
}

LOCAL void input_pull_up(UINT pin)
{
	(void)gpio_set_pin(pin, GPIO_MODE_IN);
	clr_w(GPIO(pin), GPIO_PDE);
	set_w(GPIO(pin), GPIO_PUE);
}

LOCAL void apply(const SM_OUT *out)
{
	if (out->latch) {
		(void)motion_fault_stop();		/* motors off first */
		tm_printf((UB*)"[SM] FAULT 0x%02x: FAULT_STOP latched\n", s_sm.latched);
	}
	if (out->release) {
		(void)motion_clear_fault_stop();
		tm_printf((UB*)"[SM] faults cleared: FAULT_STOP released\n");
	}
	if (out->clear_flags) {
		(void)tk_clr_flg(fault_flags, 0U);	/* clear every bit */
		tm_printf((UB*)"[SM] restart requested\n");
	}
	if (out->set_restart) {
		(void)tk_set_flg(fault_flags, FLT_RESTART_REQ);
	}
	if (out->raise != 0U) {
		(void)tk_set_flg(fault_flags, out->raise);
		tm_printf((UB*)"[SM] a task stopped running (health)\n");
	}
	if (app_send_list(vc_event_q, (UB)SRC_SAFETY, &s_seq, &out->ev) != E_OK) {
		/* Vehicle Controller's inbox is full.  The motors are already
		   stopped by FAULT_STOP; record the overflow as well. */
		(void)tk_set_flg(fault_flags, FLT_QUEUE_OVF);
	}
}

EXPORT void safety_task(INT stacd, void *exinf)
{
	SM_OUT	out;
	UW	age[HEALTH_SLOTS];
	bool	alive[HEALTH_SLOTS];

	(void)stacd;
	(void)exinf;

	input_pull_up(SM_BTN_START_PIN);
	input_pull_up(SM_BTN_STOP_PIN);
	sm_init(&s_sm, &s_params);

	while (1) {
		UINT	ptn = 0U;
		UW	now;
		UINT	i;
		ER	er;

		er = tk_wai_flg(fault_flags, sm_wait_pattern(&s_sm), TWF_ORW, &ptn, SM_PERIOD_MS);
		if (er == E_OK) {
			sm_on_flags(&s_sm, ptn, &out);
			apply(&out);
			if ((ptn & FLT_ALL) != 0U) {
				tm_printf((UB*)"[SM] fault bits 0x%02x\n", ptn & FLT_ALL);
			}
			continue;		/* buttons and health on the 20 ms timeout */
		}

		/* 20 ms timeout */
		now = app_ms();
		for (i = 0U; i < HEALTH_SLOTS; i++) {
			UW stamp = s_health_ms[i];

			alive[i] = (stamp != 0U);
			age[i] = now - stamp;
		}
		sm_on_poll(&s_sm, gpio_get_val(SM_BTN_START_PIN), gpio_get_val(SM_BTN_STOP_PIN),
			   age, alive, &out);
		apply(&out);
	}
}
