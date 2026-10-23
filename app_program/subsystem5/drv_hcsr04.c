/*
 *	drv_hcsr04.c  (see drv_hcsr04.h)
 *
 *	range = echo width x speed of sound / 2
 *	      = width_us x 0.343 mm/us / 2 = width_us x 0.1715 mm
 *	(The datasheet's "us / 58 = cm" is the same thing.)
 *
 *	The ECHO interrupt runs on core 1 (the GPIO bank interrupt is enabled
 *	there); this task runs on core 0.  They share ONE word, s_echo,
 *	written only by the interrupt (drv_hcsr04.h).  The task used to clear
 *	a 'done' flag that the interrupt set: two cores writing the same
 *	variable.  Now every shared variable has one writer.
 */

#include "drv_hcsr04.h"
#include "../gpio_irq.h"
#include "scan_core.h"		/* scan_range_mm() */

#define HC_TRIG_US		10U	/* trigger pulse width */

/* Interrupt only (core 1) */
LOCAL BOOL		s_rose;		/* a rising edge is waiting for its fall */
LOCAL UW		s_t_rise;

/* Shared: written only by the interrupt */
LOCAL volatile UW	s_echo;		/* width | count << 15 */

/* Task only (core 0) */
LOCAL UW		s_t_trig_us;
LOCAL UW		s_last_trig_ms;
LOCAL BOOL		s_in_flight;
LOCAL UW		s_count_at_trig;	/* echo counter when the ping was fired */
LOCAL UW		s_width_us;

/* Interrupt context (gpio_irq.c) */
LOCAL void echo_edge(UINT pin, UW events, UW t_us)
{
	UINT level = (UINT)((in_w(GPIO_IN) >> pin) & 1U);

	if (((events & GPIO_EDGE_RISE) != 0U) && (level == 1U)) {
		s_t_rise = t_us;
		s_rose = TRUE;
	} else if (((events & GPIO_EDGE_FALL) != 0U) && (level == 0U) && s_rose) {
		UW old = s_echo;
		UW width = t_us - s_t_rise;
		UW count = ((old >> HC_W_COUNT_SHIFT) + 1U) & HC_W_COUNT_MASK;

		if (width > HC_W_WIDTH_MASK) {
			width = HC_W_WIDTH_MASK;	/* saturate: still "no echo" */
		}
		s_rose = FALSE;
		s_echo = width | (count << HC_W_COUNT_SHIFT);	/* one store */
	} else {
		/* stray edge */
	}
}

EXPORT ER hcsr04_init(void)
{
	(void)gpio_set_val(HC_TRIG_PIN, 0U);
	(void)gpio_set_pin(HC_TRIG_PIN, GPIO_MODE_OUT);
	(void)gpio_set_pin(HC_ECHO_PIN, GPIO_MODE_IN);
	clr_w(GPIO(HC_ECHO_PIN), GPIO_PUE);	/* keep the pull-down: idle low */
	set_w(GPIO(HC_ECHO_PIN), GPIO_PDE);
	s_last_trig_ms = app_ms() - HC_MIN_GAP_MS;
	return gpio_irq_attach(HC_ECHO_PIN, GPIO_EDGE_BOTH, echo_edge);
}

EXPORT BOOL hcsr04_echo_high(void)
{
	return (gpio_get_val(HC_ECHO_PIN) != 0U) ? TRUE : FALSE;
}

EXPORT ER hcsr04_trigger(void)
{
	UW t0;

	if (((app_ms() - s_last_trig_ms) < HC_MIN_GAP_MS) || hcsr04_echo_high()) {
		return E_OBJ;		/* too soon, or the last echo is still running */
	}
	s_count_at_trig = (s_echo >> HC_W_COUNT_SHIFT) & HC_W_COUNT_MASK;
	s_in_flight = TRUE;
	s_last_trig_ms = app_ms();

	(void)gpio_set_val(HC_TRIG_PIN, 1U);
	t0 = app_us();
	s_t_trig_us = t0;
	while ((app_us() - t0) <= HC_TRIG_US) {
		/* 10 us: the only busy-wait in the system */
	}
	(void)gpio_set_val(HC_TRIG_PIN, 0U);
	return E_OK;
}

EXPORT INT hcsr04_poll(UH *mm)
{
	UW echo;

	*mm = (UH)HC_RANGE_NONE;
	if (!s_in_flight) {
		return HC_IDLE;
	}
	echo = s_echo;				/* one load: width and count together */
	if (((echo >> HC_W_COUNT_SHIFT) & HC_W_COUNT_MASK) != s_count_at_trig) {
		s_in_flight = FALSE;
		s_width_us = echo & HC_W_WIDTH_MASK;
		*mm = (UH)scan_range_mm(s_width_us);	/* 12-bit, 4,095 = none */
		return (*mm == (UH)HC_RANGE_NONE) ? HC_NO_ECHO : HC_DONE;
	}
	if ((app_us() - s_t_trig_us) > HC_ECHO_TMO_US) {
		s_in_flight = FALSE;
		/* ECHO still high: the sensor fired but heard nothing back.
		   ECHO low and no completed echo: it never answered. */
		return hcsr04_echo_high() ? HC_NO_ECHO : HC_NO_RESPONSE;	/* no echo is NOT free space */
	}
	return HC_BUSY;
}

EXPORT UW hcsr04_last_width_us(void)
{
	return s_width_us;
}
