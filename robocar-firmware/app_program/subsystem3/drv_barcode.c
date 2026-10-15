/*
 *	drv_barcode.c  (see drv_barcode.h)
 */

#include "drv_barcode.h"
#include "../gpio_irq.h"

LOCAL BC_EDGE		s_buf[BC_BUF_SIZE];
LOCAL volatile UW	s_head;		/* written only by the interrupt */
LOCAL volatile UW	s_tail;		/* written only by the task */
LOCAL volatile UW	s_overflows;

LOCAL void bc_edge(UINT pin, UW events, UW t_us)
{
	UW next = (s_head + 1U) & (BC_BUF_SIZE - 1U);

	(void)events;
	if (next == s_tail) {
		s_overflows++;			/* full: drop, and count it */
		return;
	}
	s_buf[s_head].t_us = t_us;
	s_buf[s_head].level = (UB)((in_w(GPIO_IN) >> pin) & 1U);
	s_head = next;				/* publish after the data */
}

EXPORT ER barcode_init(void)
{
	(void)gpio_set_pin(BC_DO_PIN, GPIO_MODE_IN);
	clr_w(GPIO(BC_DO_PIN), GPIO_PDE | GPIO_PUE);
	s_head = 0U;
	s_tail = 0U;
	return gpio_irq_attach(BC_DO_PIN, GPIO_EDGE_BOTH, bc_edge);
}

EXPORT BOOL barcode_pop(BC_EDGE *out)
{
	if (s_tail == s_head) {
		return FALSE;
	}
	*out = s_buf[s_tail];
	s_tail = (s_tail + 1U) & (BC_BUF_SIZE - 1U);
	return TRUE;
}

EXPORT UW barcode_overflows(void)
{
	return s_overflows;
}
