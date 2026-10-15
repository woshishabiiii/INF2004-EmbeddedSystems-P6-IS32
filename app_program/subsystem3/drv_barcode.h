/*
 *	drv_barcode.h
 *	Barcode sensor DO on GP27: every edge is timestamped (1 us) by the
 *	shared GPIO interrupt and queued in edge_buf for the Line and Barcode
 *	task.  Single producer (interrupt) and single consumer (task), both on
 *	core 1, so no lock is needed.
 */

#ifndef DRV_BARCODE_H
#define DRV_BARCODE_H

#include "../app.h"

#define BC_DO_PIN	27U
#define BC_BUF_SIZE	64U	/* must be a power of two */

typedef struct {
	UW	t_us;		/* TIMERAWL at the edge */
	UB	level;		/* pin level just after the edge */
} BC_EDGE;

IMPORT ER	barcode_init(void);		/* pin + attach to gpio_irq */
IMPORT BOOL	barcode_pop(BC_EDGE *out);	/* FALSE if empty */
IMPORT UW	barcode_overflows(void);

#endif /* DRV_BARCODE_H */
