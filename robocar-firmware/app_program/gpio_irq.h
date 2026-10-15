/*
 *	gpio_irq.h
 *	Shared GPIO edge-interrupt dispatcher.  Changed only by team agreement
 *	(like app_main.c).  Maintained by Buddy 2.
 *
 *	Users: encoders GP2, GP3, GP16, GP17 (Motion Control), barcode DO GP27
 *	(Line and Barcode), ultrasonic ECHO GP1 (Obstacle Scan).  Motion
 *	Control calls gpio_irq_start() once at start-up, on core 1; the others
 *	only call gpio_irq_attach().
 *
 *	The RP2040 has ONE interrupt line for all GPIO pins (IO_IRQ_BANK0).
 *	This file owns it: each subsystem attaches a callback for its own
 *	pins, and the single handler calls the right callback with a 1 us
 *	timestamp taken on entry.
 *
 *	Rules for callbacks (they run in interrupt context):
 *	  - count, timestamp, store, and maybe tk_set_flg / tk_wup_tsk;
 *	  - never wait, never print, never take long (aim for a few us).
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include "app.h"

#define GPIO_EDGE_FALL	0x4U	/* high -> low */
#define GPIO_EDGE_RISE	0x8U	/* low -> high */
#define GPIO_EDGE_BOTH	(GPIO_EDGE_FALL | GPIO_EDGE_RISE)

/* pin:    the GPIO number that fired
   events: GPIO_EDGE_FALL and/or GPIO_EDGE_RISE (both if the pin toggled
           twice before the handler ran)
   t_us:   TIMERAWL (1 us) read when the handler started */
typedef void (*GPIO_EDGE_FN)(UINT pin, UW events, UW t_us);

/* Attach a callback and enable the chosen edges for one pin.  Configure
   the pin as an input (and its pulls) BEFORE calling this.  Safe to call
   from any task, on either core, before or after gpio_irq_start(). */
IMPORT ER gpio_irq_attach(UINT pin, UW edges, GPIO_EDGE_FN fn);

/* Stop interrupts from one pin. */
IMPORT ER gpio_irq_detach(UINT pin);

/* Register the bank handler with the kernel and enable it in the NVIC.
   The NVIC is per core, so call this ONCE, from a task that runs on
   core 1 (TP_PRC2).  Returns E_CTX if called on the wrong core.
   A second call does nothing and returns E_OK.  (Single-core build,
   SMP=0: everything runs on core 0 and the core-0 registers are used.) */
IMPORT ER gpio_irq_start(void);

/* Statistics for timing tests (read from tasks) */
IMPORT volatile UW gpio_irq_calls;	/* handler entries */
IMPORT volatile UW gpio_irq_max_us;	/* longest handler run, us */

/* Function selected on a pin (IO_BANK0 GPIOn_CTRL.FUNCSEL): 5 = SIO
   (plain GPIO), 2 = UART, 3 = I2C, 4 = PWM, 31 = none.  Used by the
   on-target tests to confirm that the kernel build left our pins alone
   (design-review feedback IS32: GP0/GP1, GP16, GP25). */
#define GPIO_FUNC_I2C	3U
#define GPIO_FUNC_SIO	5U
IMPORT UW gpio_funcsel(UINT pin);

#endif /* GPIO_IRQ_H */
