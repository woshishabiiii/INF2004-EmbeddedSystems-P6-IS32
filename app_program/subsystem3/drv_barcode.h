/*
 *	drv_barcode.h
 *	Barcode sensor DO on GP27: every edge is timestamped (1 us) by the
 *	shared GPIO interrupt and queued in edge_buf for the Line and Barcode
 *	task.  Single producer (interrupt) and single consumer (task), both on
 *	core 1, so no lock is needed: the interrupt is the only writer of the
 *	entries and of head, the task the only writer of tail.  Head and tail
 *	are separate bytes on purpose: packed into one word, each writer
 *	would need a read-modify-write and could undo the other's update.
 */

#ifndef DRV_BARCODE_H
#define DRV_BARCODE_H

#include "../app.h"

#define BC_DO_PIN	27U
#define BC_BUF_SIZE	64U	/* must be a power of two, at most 256 (8-bit head / tail) */

/* One edge, 4 B (was 8 B: UW time + UB level + 3 B padding):
     bits 0-30  TIMERAWL at the edge, us (wraps every 2^31 us = 35.8 min;
                only differences are used: bp_dt31() in bitpack.h)
     bit 31     pin level just after the edge
   A 9 mm bar at 120 mm/s lasts 75 ms, over the 65.5 ms a 16-bit us time
   could reach, so the time keeps 31 bits. */
#define BC_EDGE_LEVEL	0x80000000U
#define BC_EDGE_TIME	0x7FFFFFFFU

IMPORT ER	barcode_init(void);		/* pin + attach to gpio_irq */
IMPORT BOOL	barcode_pop(UW *edge);		/* FALSE if empty */
IMPORT UW	barcode_overflows(void);	/* 8-bit count that wraps: compare for a change */

#endif /* DRV_BARCODE_H */
