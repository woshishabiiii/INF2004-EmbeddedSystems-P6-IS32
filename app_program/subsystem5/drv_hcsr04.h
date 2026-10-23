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

/* Range: 12-bit unsigned, 4,095 = no echo.  Anything over 4,000 mm (the
   sensor's limit; an echo over ~23.3 ms) is no echo, which our code treats
   as unknown and never as free space. */
#define HC_RANGE_NONE		4095U	/* = OS_INVALID (scan_range_mm(), scan_core.c) */

/* hcsr04_poll() results */
#define HC_IDLE			0	/* no ping in flight */
#define HC_BUSY			1	/* waiting for the echo */
#define HC_DONE			2	/* *mm is the range, 0..4,000 */
#define HC_NO_ECHO		3	/* no echo heard: ECHO still high after 30 ms, or an
					   echo longer than 4,000 mm (*mm = HC_RANGE_NONE) */
#define HC_NO_RESPONSE		4	/* ECHO never rose: sensor missing or unpowered */

IMPORT ER	hcsr04_init(void);	/* pins, attach ECHO to the GPIO interrupt */

/* Fire one ping (10 us TRIG pulse).  E_OBJ: sooner than 60 ms after the
   last one, or ECHO is still high from the last ping. */
IMPORT ER	hcsr04_trigger(void);

IMPORT INT	hcsr04_poll(UH *mm);
IMPORT BOOL	hcsr04_echo_high(void);
IMPORT UW	hcsr04_last_width_us(void);

/* The echo word, written only by the ECHO interrupt (core 1) with one
   32-bit store per completed echo, read by Obstacle Scan (core 0) with one
   load.  The task never writes or clears it: it remembers the counter
   when it triggers and waits for it to change.
     bits 0-14   echo width, us (saturates at 32,767; 4 m = 23,324)
     bits 15-22  completed echoes, wraps (only "changed?" is asked)    */
#define HC_W_WIDTH_BITS		15U
#define HC_W_WIDTH_MASK		0x7FFFU
#define HC_W_COUNT_SHIFT	15U
#define HC_W_COUNT_MASK		0xFFU

#endif /* DRV_HCSR04_H */
