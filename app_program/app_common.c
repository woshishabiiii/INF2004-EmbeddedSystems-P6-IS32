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
