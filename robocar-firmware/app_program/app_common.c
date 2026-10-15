/*
 *	app_common.c
 *	Small helpers shared by every subsystem (team agreement, like app.h).
 */

#include "app_msg.h"

#define WATCHDOG_TICK		0x4005802CU	/* WATCHDOG_BASE + 0x2C */
#define WATCHDOG_TICK_ENABLE	(1U << 9)
#define CLK_REF_MHZ		12U		/* clk_ref runs from the 12 MHz crystal */
#define REG_ALIAS_CLR		0x3000U		/* write-1-to-clear alias of a register */

EXPORT void app_timer_init(void)
{
	/* The TIMER counts the watchdog tick: 12 MHz / 12 = one count per us */
	out_w(WATCHDOG_TICK, WATCHDOG_TICK_ENABLE | CLK_REF_MHZ);
	out_w(RESETS_RESET + REG_ALIAS_CLR, RESETS_RESET_TIMER);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER) == 0U) {
		/* a few clock cycles */
	}
}

EXPORT UW app_ms(void)
{
	SYSTIM now;

	(void)tk_get_tim(&now);
	return now.lo;
}

EXPORT UW app_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

EXPORT ER app_send(ID mbf, UB source, UH *seq, UB type, W p1, W p2)
{
	APP_MSG m;

	m.type = type;
	m.source = source;
	m.seq = *seq;
	m.t_ms = app_ms();
	m.p1 = p1;
	m.p2 = p2;
	*seq = (UH)(*seq + 1U);
	return tk_snd_mbf(mbf, &m, (INT)sizeof(m), TMO_POL);
}

EXPORT ER app_send_list(ID mbf, UB source, UH *seq, const EVT_LIST *l)
{
	ER	result = E_OK;
	UW	i;

	for (i = 0U; i < l->n; i++) {
		ER er = app_send(mbf, source, seq, l->ev[i].type, l->ev[i].p1, l->ev[i].p2);

		if (er != E_OK) {
			result = er;
		}
	}
	return result;
}

/* ---- queue-overflow policy ------------------------------------------------ */
#define DROP_SOURCES	8U		/* SRC_... are 1..7 */

LOCAL UW	s_dropped[DROP_SOURCES];	/* one writer per source task */

EXPORT ER app_send_event(ID mbf, UB source, UH *seq, UB type, W p1, W p2, BOOL *was_dropped)
{
	ER	er = app_send(mbf, source, seq, type, p1, p2);
	BOOL	drop = FALSE;

	if ((er != E_OK) && !evt_is_safety(type)) {
		drop = TRUE;			/* information only: drop and count */
		if (source < DROP_SOURCES) {
			s_dropped[source]++;
		}
		er = E_OK;
	}
	if (was_dropped != NULL) {
		*was_dropped = drop;
	}
	return er;				/* not E_OK: a safety event was lost */
}

EXPORT ER app_send_events(ID mbf, UB source, UH *seq, const EVT_LIST *l, UW *dropped)
{
	ER	result = E_OK;
	UW	mask = 0U;
	UW	i;

	for (i = 0U; i < l->n; i++) {
		BOOL	drop;
		ER	er = app_send_event(mbf, source, seq, l->ev[i].type, l->ev[i].p1, l->ev[i].p2, &drop);

		if (er != E_OK) {
			result = er;			/* must not be lost: caller raises a fault */
		}
		if (drop) {
			mask |= 1UL << i;
		}
	}
	if (dropped != NULL) {
		*dropped = mask;
	}
	return result;
}

EXPORT UW app_dropped(void)
{
	UW sum = 0U;
	UW i;

	for (i = 0U; i < DROP_SOURCES; i++) {
		sum += s_dropped[i];
	}
	return sum;
}

/* ---- task timing ---------------------------------------------------------- */
LOCAL APP_TIMING	s_timing;

EXPORT void app_task_time(UINT slot, UW start_us)
{
	UW dt = app_us() - start_us;

	if ((slot < TIM_SLOTS) && (dt > s_timing.max_us[slot])) {
		s_timing.max_us[slot] = dt;
	}
}

EXPORT void app_task_overrun(UINT slot, UW missed)
{
	if (slot < TIM_SLOTS) {
		s_timing.overruns[slot] += missed;
	}
}

EXPORT void app_timing_get(APP_TIMING *out)
{
	*out = s_timing;		/* word copies; each slot has one writer */
}
