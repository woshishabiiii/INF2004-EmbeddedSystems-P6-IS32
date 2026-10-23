/*
 *	subsystem4.c
 *	Subsystem 4 -- creates fault_flags and the IMU cyclic handler, starts
 *	the Safety Monitor and the IMU and Terrain task, both on core 0.
 */

#include "subsystem4.h"
#include "imu_cfg.h"

EXPORT ID	fault_flags;
EXPORT ID	tid_safety, tid_imu, cyc_imu;

EXPORT ER subsystem4_init(void)
{
	T_CFLG	cflg = {0};
	T_CCYC	ccyc = {0};

	cflg.flgatr = TA_TFIFO | TA_WMUL;
#if defined(TM_MQTT) && TM_MQTT
	cflg.iflgptn = FLT_WIFI_DOWN;		/* not connected yet: the network task clears it */
#else
	cflg.iflgptn = 0U;
#endif
	fault_flags = tk_cre_flg(&cflg);
	if (fault_flags < E_OK) {
		return fault_flags;
	}
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)imu_cyclic_handler;
	ccyc.cyctim = IMU_PERIOD_MS;
	ccyc.cycphs = IMU_PERIOD_MS;
	cyc_imu = tk_cre_cyc(&ccyc);
	return (cyc_imu < E_OK) ? cyc_imu : E_OK;
}

EXPORT ER subsystem4_start(void)
{
	/* Core 0: if a task on core 1 runs away, the monitor still runs */
	tid_safety = app_start_task((FP)safety_task, S4_PRI_SAFETY, TP_PRC1, "safety");
	if (tid_safety < E_OK) {
		return tid_safety;
	}
	tid_imu = app_start_task((FP)imu_task, S4_PRI_IMU, TP_PRC1, "imu");
	if (tid_imu < E_OK) {
		return tid_imu;
	}
	return tk_sta_cyc(cyc_imu);
}
