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

/* Both wheels' counts in one word: left = bits 0-15, right = bits 16-31,
   each a 16-bit count that wraps (+ = forward after sign calibration).
   Only the GPIO interrupt writes it, each half with a 16-bit store, so no
   read-modify-write; this is one 32-bit load, so both counts are from the
   same moment.  Motion Control turns it into 32-bit positions
   (mc_enc_update() in mc_core.c). */
IMPORT UW	encoder_word(void);

/* Per-wheel status byte, also written only by the interrupt:
     bits 0-1  last (A << 1) | B state
     bits 2-7  transitions that skipped a state (both pins changed between
               two interrupts), stops at 63.  Should stay at 0; if it
               grows, edges are being lost. */
#define ENC_STAT_STATE_MASK	0x03U
#define ENC_STAT_ERR_SHIFT	2U
#define ENC_STAT_ERR_MAX	63U
IMPORT UW	encoder_errors(UINT side);

#endif /* DRV_ENCODER_H */
