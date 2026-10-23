/*
 *	subsystem3.c
 *	Subsystem 3 -- Line and Barcode: creates its cyclic handler, starts
 *	its task.
 */

#include "subsystem3.h"
#include "line_cfg.h"

EXPORT ID	tid_line;
EXPORT ID	cyc_line;

EXPORT const LS_PARAMS lb_steer_params = {
	LS_D_MIN,		/* d_min */
	LS_D_RATE,		/* d_rate */
	LS_RATE_MM,		/* rate_mm */
	LS_D_MAX,		/* d_max */
	LS_D_BIAS,		/* d_bias */
	LS_D_SEARCH,		/* d_search */
	LS_ESCAPE_MM,		/* escape_mm */
	LS_DRIFT_MM,		/* drift_mm */
	LB_LOST_MM,		/* lost_mm */
	LS_SLOW_PCT,		/* slow_pct */
	LS_START_SIDE		/* start_side */
};

EXPORT ER subsystem3_init(void)
{
	T_CCYC ccyc = {0};

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)line_cyclic_handler;
	ccyc.cyctim = LB_PERIOD_MS;
	ccyc.cycphs = LB_PERIOD_MS;
	cyc_line = tk_cre_cyc(&ccyc);
	return (cyc_line < E_OK) ? cyc_line : E_OK;
}

EXPORT ER subsystem3_start(void)
{
	/* Core 1, next to Motion Control and the GPIO interrupt that
	   timestamps the barcode edges */
	tid_line = app_start_task((FP)line_task, S3_PRI_LINE, TP_PRC2, "line");
	if (tid_line < E_OK) {
		return tid_line;
	}
	return tk_sta_cyc(cyc_line);
}
