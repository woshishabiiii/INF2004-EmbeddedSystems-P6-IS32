/*
 *	gpio_irq.c
 *	Shared GPIO edge-interrupt dispatcher (see gpio_irq.h).
 *
 *	RP2040 registers used (IO_BANK0, base 0x40014000):
 *	  INTR0..3        0x0F0  raw status; write 1 to clear an EDGE bit
 *	  PROC1_INTE0..3  0x130  enable, for interrupts delivered to core 1
 *	  PROC1_INTS0..3  0x150  status after masking, as seen by core 1
 *	(PROC0_INTE 0x100 / PROC0_INTS 0x120 in the single-core build, SMP=0,
 *	where every task and interrupt runs on core 0.)
 *	Each register covers 8 pins, 4 bits per pin:
 *	  bit 0 LEVEL_LOW, bit 1 LEVEL_HIGH, bit 2 EDGE_LOW, bit 3 EDGE_HIGH.
 *	Writes to base + 0x2000 set bits and base + 0x3000 clear bits
 *	atomically, so two cores can attach pins without a lock.
 */

#include "gpio_irq.h"

#if TK_SUPPORT_SMP
#define GPIO_IRQ_INTE_OFS	0x130U	/* PROC1_INTE0: delivered to core 1 */
#define GPIO_IRQ_INTS_OFS	0x150U	/* PROC1_INTS0 */
#else
#define GPIO_IRQ_INTE_OFS	0x100U	/* PROC0_INTE0: single-core build */
#define GPIO_IRQ_INTS_OFS	0x120U	/* PROC0_INTS0 */
#endif
#define IO_BANK0_INTR(i)	(IO_BANK0_BASE + 0x0F0U + ((UW)(i) * 4U))
#define IO_BANK0_P1_INTE(i)	(IO_BANK0_BASE + GPIO_IRQ_INTE_OFS + ((UW)(i) * 4U))
#define IO_BANK0_P1_INTS(i)	(IO_BANK0_BASE + GPIO_IRQ_INTS_OFS + ((UW)(i) * 4U))
#define REG_ALIAS_SET		0x2000U
#define REG_ALIAS_CLR		0x3000U

#define GPIO_IRQ_INTNO		13U	/* IO_IRQ_BANK0 in the RP2040 datasheet */
#define GPIO_IRQ_LEVEL		2	/* same level as the kernel's ADC and I2C drivers */
#define GPIO_IRQ_BANKS		4U
#define GPIO_IRQ_PROC_CORE1	2	/* tk_get_prc() counts cores from 1 */

LOCAL GPIO_EDGE_FN	s_fn[GPIO_NUM];
LOCAL BOOL		s_started = FALSE;
EXPORT volatile UW	gpio_irq_calls;
EXPORT volatile UW	gpio_irq_max_us;

LOCAL void gpio_bank_handler(UINT intno)
{
	UW	t0 = in_w(TIMER_TIMERAWL);	/* timestamp first */
	UW	bank;
	UW	dt;

	(void)intno;

	for (bank = 0U; bank < GPIO_IRQ_BANKS; bank++) {
		UW ints = in_w(IO_BANK0_P1_INTS(bank));

		if (ints != 0U) {
			UW k;

			/* Clear the edges we are about to handle.  An edge that
			   arrives after this write raises the interrupt again. */
			out_w(IO_BANK0_INTR(bank), ints);

			for (k = 0U; k < 8U; k++) {
				UW	ev = (ints >> (k * 4U)) & GPIO_EDGE_BOTH;
				UINT	pin = (UINT)((bank * 8U) + k);

				if ((ev != 0U) && (pin < GPIO_NUM) && (s_fn[pin] != NULL)) {
					s_fn[pin](pin, ev, t0);
				}
			}
		}
	}

	gpio_irq_calls++;
	dt = in_w(TIMER_TIMERAWL) - t0;
	if (dt > gpio_irq_max_us) {
		gpio_irq_max_us = dt;
	}
}

EXPORT ER gpio_irq_attach(UINT pin, UW edges, GPIO_EDGE_FN fn)
{
	UW	bank;
	UW	shift;

	if ((pin >= GPIO_NUM) || (fn == NULL) ||
	    ((edges & ~GPIO_EDGE_BOTH) != 0U) || (edges == 0U)) {
		return E_PAR;
	}
	bank = (UW)pin / 8U;
	shift = ((UW)pin % 8U) * 4U;

	s_fn[pin] = fn;
	out_w(IO_BANK0_INTR(bank), GPIO_EDGE_BOTH << shift);	/* drop stale edges */
	out_w(IO_BANK0_P1_INTE(bank) + REG_ALIAS_SET, edges << shift);
	return E_OK;
}

EXPORT ER gpio_irq_detach(UINT pin)
{
	UW	bank;
	UW	shift;

	if (pin >= GPIO_NUM) {
		return E_PAR;
	}
	bank = (UW)pin / 8U;
	shift = ((UW)pin % 8U) * 4U;
	out_w(IO_BANK0_P1_INTE(bank) + REG_ALIAS_CLR, 0xFU << shift);
	s_fn[pin] = NULL;
	return E_OK;
}

EXPORT ER gpio_irq_start(void)
{
	T_DINT	dint = {0};
	ER	er;

	if (s_started) {
		return E_OK;		/* already running: calling twice is harmless */
	}
#if TK_SUPPORT_SMP
	if (tk_get_prc() != GPIO_IRQ_PROC_CORE1) {
		return E_CTX;		/* must run on core 1 */
	}
#endif
	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)gpio_bank_handler;
	er = tk_def_int(GPIO_IRQ_INTNO, &dint);
	if (er < E_OK) {
		return er;
	}
	ClearInt(GPIO_IRQ_INTNO);
	EnableInt(GPIO_IRQ_INTNO, GPIO_IRQ_LEVEL);
	s_started = TRUE;
	return E_OK;
}

EXPORT UW gpio_funcsel(UINT pin)
{
	/* GPIOn_CTRL is at IO_BANK0 + 8 n + 4; FUNCSEL is bits 4:0 */
	return (pin < GPIO_NUM) ? (in_w(IO_BANK0_BASE + (8U * (UW)pin) + 4U) & 0x1FU) : 0x1FU;
}
