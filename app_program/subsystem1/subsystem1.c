/*
 *	subsystem1.c
 *	Creates telemetry_q and the two network bridge buffers, starts the
 *	Telemetry and Command task.
 */

#include "subsystem1.h"
#include "tel_bin.h"
#include "cmd_parse.h"
#include "../bitpack.h"

BP_STATIC_ASSERT(NET_PAYLOAD_MAX == TEL_BIN_MAX, payload_is_largest_record);
#define NET_TX_SLOTS	4U
BP_STATIC_ASSERT(NET_CMD_MAX == CMD_BYTES, command_is_4_bytes);

#define TEL_Q_SLOTS	16U
#define NET_RX_SLOTS	4U

EXPORT ID	telemetry_q, mqtt_tx_mbf, mqtt_rx_mbf, tid_telemetry;

LOCAL ID make_mbf(UW slots, UW maxmsz)
{
	T_CMBF cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)MBF_BYTES(slots, maxmsz);
	cmbf.maxmsz = (INT)maxmsz;
	return tk_cre_mbf(&cmbf);
}

EXPORT ER subsystem1_init(void)
{
	telemetry_q = make_mbf(TEL_Q_SLOTS, (UW)sizeof(APP_MSG));
	if (telemetry_q < E_OK) {
		return telemetry_q;
	}
	/* tx message = 1 byte topic number + binary record (35 B -> 36 + 4 B
	   per slot with the kernel's rounding: 160 B, was 820 B for JSON) */
	mqtt_tx_mbf = make_mbf(NET_TX_SLOTS, NET_PAYLOAD_MAX + 1U);
	if (mqtt_tx_mbf < E_OK) {
		return mqtt_tx_mbf;
	}
	mqtt_rx_mbf = make_mbf(NET_RX_SLOTS, NET_CMD_MAX);
	return (mqtt_rx_mbf < E_OK) ? mqtt_rx_mbf : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_telemetry = app_start_task((FP)telemetry_task, S1_PRI_TELEMETRY, TP_PRC1, "telemetry");
	return (tid_telemetry < E_OK) ? tid_telemetry : E_OK;
}
