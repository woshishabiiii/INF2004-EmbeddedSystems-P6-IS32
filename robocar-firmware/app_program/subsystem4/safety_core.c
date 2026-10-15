/*
 *	safety_core.c
 *	Safety Monitor logic (see safety_core.h).
 */

#include "safety_core.h"

#define BTN_START_I	0U
#define BTN_STOP_I	1U

static void out_clear(SM_OUT *out)
{
	out->latch = false;
	out->release = false;
	out->clear_flags = false;
	out->set_restart = false;
	out->raise = 0U;
	evt_list_clear(&out->ev);
}

void sm_init(SM_CORE *s, const SM_PARAMS *p)
{
	*s = (SM_CORE){0};
	s->p = *p;
	s->prev[BTN_START_I] = SM_BTN_RELEASED;
	s->prev[BTN_STOP_I] = SM_BTN_RELEASED;
	s->stable[BTN_START_I] = SM_BTN_RELEASED;
	s->stable[BTN_STOP_I] = SM_BTN_RELEASED;
}

uint32_t sm_wait_pattern(const SM_CORE *s)
{
	return (s->faulted && (s->restart_left == 0U)) ?
	       FLT_RESTART_REQ : (FLT_ALL | FLT_RESTART_REQ);
}

void sm_on_flags(SM_CORE *s, uint32_t ptn, SM_OUT *out)
{
	uint32_t faults = ptn & FLT_ALL;

	out_clear(out);
	if ((faults != 0U) && !s->faulted) {
		/* 1. a new fault: latch FAULT_STOP at once */
		s->faulted = true;
		s->latched = faults;
		s->restart_left = 0U;
		out->latch = true;
		(void)evt_list_add(&out->ev, (uint8_t)EV_FAULT, (int32_t)faults, 0);
	} else if ((faults != 0U) && s->faulted) {
		/* 2. a fault is (still) present: a pending restart is cancelled */
		s->latched |= faults;
		s->restart_left = 0U;
	} else if ((ptn & FLT_RESTART_REQ) != 0U) {
		/* 3. RESTART: clear every bit; they must now stay clear */
		out->clear_flags = true;
		if (s->faulted) {
			s->restart_left = s->p.restart_polls;
			s->health_bad = 0U;	/* a still-silent task is reported again */
		}
	} else {
		/* nothing new */
	}
}

void sm_on_poll(SM_CORE *s, uint32_t btn_start, uint32_t btn_stop,
		const uint32_t age_ms[HEALTH_SLOTS], const bool alive[HEALTH_SLOTS],
		SM_OUT *out)
{
	uint32_t now_btn[2];
	uint32_t i;

	out_clear(out);

	/* 1. a restart counting down */
	if (s->restart_left > 0U) {
		s->restart_left--;
		if ((s->restart_left == 0U) && s->faulted) {
			s->faulted = false;
			s->latched = 0U;
			out->release = true;
			(void)evt_list_add(&out->ev, (uint8_t)EV_FAULT_CLEAR, 0, 0);
		}
	}

	/* 2. buttons: a new level must be read twice in a row */
	now_btn[BTN_START_I] = btn_start;
	now_btn[BTN_STOP_I] = btn_stop;
	for (i = 0U; i < 2U; i++) {
		if ((now_btn[i] == s->prev[i]) && (now_btn[i] != s->stable[i])) {
			s->stable[i] = now_btn[i];
			if (s->stable[i] != SM_BTN_RELEASED) {
				if (i == BTN_STOP_I) {
					(void)evt_list_add(&out->ev, (uint8_t)EV_BUTTON, BTN_STOP, 0);
				} else if (s->faulted) {
					out->set_restart = true;	/* START while faulted = RESTART */
				} else {
					(void)evt_list_add(&out->ev, (uint8_t)EV_BUTTON, BTN_START, 0);
				}
			}
		}
		s->prev[i] = now_btn[i];
	}

	/* 3. is every periodic task still running? */
	for (i = 0U; i < HEALTH_SLOTS; i++) {
		uint32_t bit = 1UL << i;

		if ((s->p.limit_ms[i] != 0U) && alive[i] && (age_ms[i] > s->p.limit_ms[i])) {
			if ((s->health_bad & bit) == 0U) {
				s->health_bad |= bit;
				out->raise |= FLT_HEALTH;
			}
		} else {
			s->health_bad &= ~bit;
		}
	}
}
