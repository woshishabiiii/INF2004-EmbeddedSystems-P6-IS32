/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control: creates its objects, starts its task.
 */

#include "subsystem2.h"

#define MC_CMD_SLOTS	8U

EXPORT ID	stop_flags;
EXPORT ID	tid_motion;
EXPORT ID	motion_cmd_q;
EXPORT ID	cyc_motion;
EXPORT ID	mtx_snapshot;

EXPORT ER subsystem2_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};
	T_CMTX	cmtx = {0};
	T_CCYC	ccyc = {0};

	/* CMD_STOP starts SET: the car is held in READY until START */
	cflg.flgatr = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = MC_CMD_STOP;
	stop_flags = tk_cre_flg(&cflg);
	if (stop_flags < E_OK) {
		return stop_flags;
	}

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)MBF_BYTES(MC_CMD_SLOTS, sizeof(MC_CMD));
	cmbf.maxmsz = (INT)sizeof(MC_CMD);
	motion_cmd_q = tk_cre_mbf(&cmbf);
	if (motion_cmd_q < E_OK) {
		return motion_cmd_q;
	}

	cmtx.mtxatr = TA_INHERIT;		/* priority inheritance */
	mtx_snapshot = tk_cre_mtx(&cmtx);
	if (mtx_snapshot < E_OK) {
		return mtx_snapshot;
	}

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)motion_cyclic_handler;
	ccyc.cyctim = MC_PERIOD_MS;
	ccyc.cycphs = MC_PERIOD_MS;
	cyc_motion = tk_cre_cyc(&ccyc);
	return (cyc_motion < E_OK) ? cyc_motion : E_OK;
}

EXPORT ER subsystem2_start(void)
{
	/* Core 1 (TP_PRC2): no kernel service task runs there, and the
	   shared GPIO interrupt is enabled on core 1 by this task. */
	tid_motion = app_start_task((FP)motion_task, S2_PRI_MOTION, TP_PRC2, "motion");
	if (tid_motion < E_OK) {
		return tid_motion;
	}
	return tk_sta_cyc(cyc_motion);
}
