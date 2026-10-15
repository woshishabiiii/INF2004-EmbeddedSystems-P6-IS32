/*
 *	subsystem1_test.c
 *	On-target tests for Telemetry and Command.  Built only with: make TEST=1
 */

#include "subsystem1.h"
#include "cmd_parse.h"
#include "tel_json.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
	CMD		c;
	uint32_t	seq = 0U;
	TEL_DATA	d = {0};
	char		json[NET_PAYLOAD_MAX];

	TEST_INIT();

	TEST_CHECK("S1-U01", (cmd_parse("1 START", 7U, &seq, true, &c) == CMD_OK) && (c.verb == 1),
		   "\"1 START\" accepted");
	TEST_CHECK("S1-U02", cmd_parse("1 STOP", 6U, &seq, true, &c) == CMD_ERR_REPEAT,
		   "repeated sequence number refused");
	TEST_CHECK("S1-U03", cmd_parse("2 JUMP", 6U, &seq, true, &c) == CMD_ERR_VERB,
		   "unknown verb refused");
	TEST_CHECK("S1-U04", cmd_parse("3 SET base_speed 999", 20U, &seq, true, &c) == CMD_ERR_RANGE,
		   "SET outside its range refused");
	TEST_CHECK("S1-U05", cmd_parse("4 SET base_speed 220", 20U, &seq, false, &c) == CMD_ERR_STATE,
		   "SET refused while running");
	TEST_CHECK("S1-U06", (cmd_parse("5 set stop_mm 300", 17U, &seq, true, &c) == CMD_OK) &&
		   (c.param == (int32_t)PARAM_STOP_MM) && (c.value == 300), "SET stop_mm 300 accepted");
	TEST_CHECK("S1-U07", param_get(PARAM_BASE_SPEED) == PARAM_UNSET, "parameters start unset");

	d.t_ms = 4000000000U;
	d.v[0] = -400;
	d.v[1] = -400;
	d.enc[0] = -2000000000;
	d.enc[1] = -2000000000;
	d.odo_mm = 2000000000;
	d.barcode = 'A';
	d.range_mm = 4000;
	d.faults = 0xFFU;
	TEST_CHECK("S1-U08", tel_fmt_telemetry(&d, json, NET_PAYLOAD_MAX) > 0,
		   "telemetry fits in 200 bytes with the largest values");
	TEST_CHECK("S1-U09", tel_fmt_status(&d, json, NET_PAYLOAD_MAX) > 0,
		   "heartbeat fits in 200 bytes");
	TEST_CHECK("S1-U10", (mqtt_tx_mbf > 0) && (mqtt_rx_mbf > 0) && (telemetry_q > 0),
		   "bridge buffers and telemetry_q exist");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
