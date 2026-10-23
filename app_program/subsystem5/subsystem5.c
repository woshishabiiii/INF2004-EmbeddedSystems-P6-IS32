/*
 *	subsystem5.c
 *	Creates vc_event_q (Vehicle Controller's inbox), scan_request_q,
 *	mtx_profile and the scan cyclic handler; starts Vehicle Controller (core 1) and
 *	Obstacle Scan (core 0).
 */

#include "subsystem5.h"
#include "scan_cfg.h"

#define SCAN_REQ_SLOTS		4U

EXPORT ID	vc_event_q;
EXPORT ID	scan_request_q;
EXPORT ID	tid_scan, tid_vc, cyc_scan;
EXPORT ID	mtx_profile;

EXPORT ER subsystem5_init(void)
{
	T_CMBF	cmbf = {0};
	T_CCYC	ccyc = {0};
	T_CMTX	cmtx = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)MBF_BYTES(VC_EVENT_SLOTS, sizeof(APP_MSG));
	cmbf.maxmsz = (INT)sizeof(APP_MSG);
	vc_event_q = tk_cre_mbf(&cmbf);
	if (vc_event_q < E_OK) {
		return vc_event_q;
	}
	cmbf.bufsz  = (SZ)MBF_BYTES(SCAN_REQ_SLOTS, sizeof(SCAN_REQ_PK));
	cmbf.maxmsz = (INT)sizeof(SCAN_REQ_PK);
	scan_request_q = tk_cre_mbf(&cmbf);
	if (scan_request_q < E_OK) {
		return scan_request_q;
	}
	/* The packed profile (13 B) is copied in and out under this kernel
	   mutex: a copy takes several loads and stores, and the writer (core 0)
	   and the readers (core 1 and core 0) run at the same time.  Disabling
	   interrupts would not stop the other core. */
	cmtx.mtxatr = TA_INHERIT;		/* priority inheritance */
	mtx_profile = tk_cre_mtx(&cmtx);
	if (mtx_profile < E_OK) {
		return mtx_profile;
	}
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)scan_cyclic_handler;
	ccyc.cyctim = OS_PERIOD_MS;
	ccyc.cycphs = OS_PERIOD_MS;
	cyc_scan = tk_cre_cyc(&ccyc);
	return (cyc_scan < E_OK) ? cyc_scan : E_OK;
}

EXPORT ER subsystem5_start(void)
{
	tid_vc = app_start_task((FP)vc_task, S5_PRI_VC, TP_PRC2, "vc");
	if (tid_vc < E_OK) {
		return tid_vc;
	}
	tid_scan = app_start_task((FP)scan_task, S5_PRI_SCAN, TP_PRC1, "scan");
	if (tid_scan < E_OK) {
		return tid_scan;
	}
	return tk_sta_cyc(cyc_scan);
}
