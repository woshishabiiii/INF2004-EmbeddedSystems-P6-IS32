/*
 *	drv_barcode.c  (see drv_barcode.h)
 */

#include "drv_barcode.h"
#include "../gpio_irq.h"
#include "../bitpack.h"

BP_STATIC_ASSERT((BC_BUF_SIZE <= 256U) && ((BC_BUF_SIZE & (BC_BUF_SIZE - 1U)) == 0U), bc_buf_fits_8_bits);

LOCAL UW		s_buf[BC_BUF_SIZE];	/* 64 x 4 B: time | level << 31 */
LOCAL volatile UB	s_head;		/* written only by the interrupt */
LOCAL volatile UB	s_tail;		/* written only by the task */
LOCAL volatile UB	s_overflows;	/* written only by the interrupt; wraps, only
					   compared for a change (one per 5 ms task cycle) */

LOCAL void bc_edge(UINT pin, UW events, UW t_us)
{
	UB next = (UB)((s_head + 1U) & (BC_BUF_SIZE - 1U));

	(void)events;
	if (next == s_tail) {
		s_overflows = (UB)(s_overflows + 1U);	/* full: drop, and count it */
		return;
	}
	s_buf[s_head] = (t_us & BC_EDGE_TIME) |
			((((in_w(GPIO_IN) >> pin) & 1U) != 0U) ? BC_EDGE_LEVEL : 0U);
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

EXPORT BOOL barcode_pop(UW *edge)
{
	if (s_tail == s_head) {
		return FALSE;
	}
	*edge = s_buf[s_tail];
	s_tail = (UB)((s_tail + 1U) & (BC_BUF_SIZE - 1U));
	return TRUE;
}

EXPORT UW barcode_overflows(void)
{
	return s_overflows;
}
