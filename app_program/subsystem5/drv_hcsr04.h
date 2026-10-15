/*
 *	drv_hcsr04.h
 *	HC-SR04 ultrasonic sensor at 3.3 V: TRIG = GP0, ECHO = GP1 (Grove 1).
 *	Non-blocking: hcsr04_trigger() fires a ping and returns at once; the
 *	shared GPIO interrupt timestamps both ECHO edges (1 us); the Obstacle
 *	Scan task collects the result with hcsr04_poll() on its next 10 ms
 *	step.  Called only from the Obstacle Scan task.
 */

#ifndef DRV_HCSR04_H
#define DRV_HCSR04_H

#include "../app.h"

#define HC_TRIG_PIN		0U
#define HC_ECHO_PIN		1U
#define HC_MIN_GAP_MS		60U	/* hard limit (datasheet: over 60 ms between
					   triggers); Obstacle Scan itself waits 70 ms */
#define HC_ECHO_TMO_US		30000U	/* 4 m round trip = 23 ms, plus margin */

/* hcsr04_poll() results */
#define HC_IDLE			0	/* no ping in flight */
#define HC_BUSY			1	/* waiting for the echo */
#define HC_DONE			2	/* *mm is the range */
#define HC_NO_ECHO		3	/* ECHO rose but did not fall within 30 ms: no echo heard */
#define HC_NO_RESPONSE		4	/* ECHO never rose: sensor missing or unpowered */

IMPORT ER	hcsr04_init(void);	/* pins, attach ECHO to the GPIO interrupt */

/* Fire one ping (10 us TRIG pulse).  E_OBJ: sooner than 60 ms after the
   last one, or ECHO is still high from the last ping. */
IMPORT ER	hcsr04_trigger(void);

IMPORT INT	hcsr04_poll(W *mm);
IMPORT BOOL	hcsr04_echo_high(void);
IMPORT UW	hcsr04_last_width_us(void);

#endif /* DRV_HCSR04_H */
