/*
 *	drv_ir.c  (see drv_ir.h)
 *	TCRT5000 modules with an LM393 comparator: DO is a clean logic level
 *	set by the module's potentiometer; AO is the raw photo-transistor
 *	voltage (high over black with these modules -- measure it on your own modules).
 */

#include "drv_ir.h"
#include "line_cfg.h"

LOCAL void input_no_pull(UINT pin)
{
	(void)gpio_set_pin(pin, GPIO_MODE_IN);
	clr_w(GPIO(pin), GPIO_PDE | GPIO_PUE);	/* the reset default is pull-down */
}

EXPORT ER ir_init(void)
{
	input_no_pull(LB_LEFT_PIN);
	input_no_pull(LB_RIGHT_PIN);
	return E_OK;
}

EXPORT void ir_read_line(BOOL *left_black, BOOL *right_black)
{
	UW in = in_w(GPIO_IN);		/* both pins in one read */

	*left_black = (((in >> LB_LEFT_PIN) & 1U) == LB_BLACK_LEVEL) ? TRUE : FALSE;
	*right_black = (((in >> LB_RIGHT_PIN) & 1U) == LB_BLACK_LEVEL) ? TRUE : FALSE;
}

EXPORT ER ir_barcode_ao(UINT n, UH *mean, UH *lo, UH *hi)
{
	ID	dd;
	UW	v = 0U, sum = 0U;	/* v: the ADC driver's word; sum: working value */
	UH	a;
	UINT	i, got = 0U;
	SZ	asz;

	*mean = 0U;
	*lo = 0xFFFFU;
	*hi = 0U;
	dd = tk_opn_dev((UB*)"adca", TD_UPDATE);
	if (dd < E_OK) {
		return dd;
	}
	for (i = 0U; i < n; i++) {
		if (tk_srea_dev(dd, LB_BC_ADC_CH, &v, 1, &asz) >= E_OK) {
			a = (UH)(v & IR_ADC_MASK);	/* 12-bit reading */
			sum += a;
			got++;
			*lo = (a < *lo) ? a : *lo;
			*hi = (a > *hi) ? a : *hi;
		}
		(void)tk_dly_tsk(1);
	}
	(void)tk_cls_dev(dd, 0);
	if (got == 0U) {
		return E_IO;
	}
	*mean = (UH)(sum / got);
	return E_OK;
}
