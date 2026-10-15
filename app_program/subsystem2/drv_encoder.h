/*
 *	drv_encoder.h
 *	Quadrature wheel encoders, counted in the shared GPIO interrupt.
 *	Left: A = GP2, B = GP3 (Grove 2).  Right: A = GP16, B = GP17 (Grove 4).
 *	Owned by Motion Control.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "../app.h"

#define ENC_LEFT	0U
#define ENC_RIGHT	1U

/* Configure the four pins and attach them to the shared GPIO interrupt.
   gpio_irq_start() must also have been called (from a core-1 task). */
IMPORT ER	encoder_init(void);

/* Signed count since start-up: + = forward after sign calibration.
   Reading a 32-bit word is atomic, so no lock is needed. */
IMPORT W	encoder_count(UINT side);

/* Transitions that skipped a state (both pins changed between two
   interrupts).  Should stay at 0; if it grows, edges are being lost. */
IMPORT UW	encoder_errors(UINT side);

#endif /* DRV_ENCODER_H */
