/*
 *	drv_ir.h
 *	The two line sensors (digital DO only) and the barcode sensor's
 *	analogue output, used for calibration.  Called only from the Line and
 *	Barcode task.  (The barcode sensor's DO edges: drv_barcode.h.)
 */

#ifndef DRV_IR_H
#define DRV_IR_H

#include "../app.h"

/* Line sensor pins as inputs with NO pull (the modules drive the line) */
IMPORT ER	ir_init(void);

/* TRUE = that sensor sees black */
IMPORT void	ir_read_line(BOOL *left_black, BOOL *right_black);

/* Average, lowest and highest of n 12-bit readings (0..4095, in UH) of the barcode
   sensor's AO on ADC0 (GP26), about 1 ms apart.  Returns E_OK or a
   kernel error (for example when the ADC device is missing). */
#define IR_ADC_MASK	0x0FFFU		/* the RP2040 ADC is 12-bit */
IMPORT ER	ir_barcode_ao(UINT n, UH *mean, UH *lo, UH *hi);

#endif /* DRV_IR_H */
