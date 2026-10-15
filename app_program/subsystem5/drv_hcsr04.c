/*
 *	drv_hcsr04.c  (see drv_hcsr04.h)
 *
 *	range = echo width x speed of sound / 2
 *	      = width_us x 0.343 mm/us / 2 = width_us x 0.1715 mm
 *	(The datasheet's "us / 58 = cm" is the same thing.)
 *
 *	The ECHO interrupt runs on core 1 (the GPIO bank interrupt is enabled
 *	there); this task runs on core 0.  They share only the volatile words
 *	below: the interrupt writes t_rise/t_fall and then sets done, and the
 *	task reads done before the times.
 */

#include "drv_hcsr04.h"
#include "../gpio_irq.h"

#define HC_TRIG_US		10U	/* trigger pulse width */
#define HC_MM_PER_US_X10000	1715U	/* 0.1715 mm per us */

LOCAL volatile BOOL	s_armed;	/* TRUE between trigger and falling edge */
LOCAL volatile BOOL	s_rose;
LOCAL volatile BOOL	s_done;
LOCAL volatile UW	s_t_rise;
LOCAL volatile UW	s_t_fall;
LOCAL UW		s_t_trig_us;
LOCAL UW		s_last_trig_ms;
LOCAL BOOL		s_in_flight;
LOCAL UW		s_width_us;

/* Interrupt context (gpio_irq.c) */
LOCAL void echo_edge(UINT pin, UW events, UW t_us)
{
	UINT level = (UINT)((in_w(GPIO_IN) >> pin) & 1U);

	if (!s_armed) {
		return;
	}
	if (((events & GPIO_EDGE_RISE) != 0U) && (level == 1U) && !s_rose) {
		s_t_rise = t_us;
		s_rose = TRUE;
	} else if (((events & GPIO_EDGE_FALL) != 0U) && (level == 0U) && s_rose) {
		s_t_fall = t_us;
		s_armed = FALSE;
		s_done = TRUE;			/* last: the times are already stored */
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
	s_rose = FALSE;
	s_done = FALSE;
	s_armed = TRUE;
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

EXPORT INT hcsr04_poll(W *mm)
{
	if (!s_in_flight) {
		return HC_IDLE;
	}
	if (s_done) {
		s_in_flight = FALSE;
		s_width_us = s_t_fall - s_t_rise;
		*mm = (W)((s_width_us * HC_MM_PER_US_X10000) / 10000U);
		return HC_DONE;
	}
	if ((app_us() - s_t_trig_us) > HC_ECHO_TMO_US) {
		s_in_flight = FALSE;
		s_armed = FALSE;
		return s_rose ? HC_NO_ECHO : HC_NO_RESPONSE;	/* no echo is NOT free space */
	}
	return HC_BUSY;
}

EXPORT UW hcsr04_last_width_us(void)
{
	return s_width_us;
}
