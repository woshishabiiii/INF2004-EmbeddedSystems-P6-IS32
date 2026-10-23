/*
 *	subsystem1_test.c
 *	On-target tests for Telemetry and Command.  Built only with: make TEST=1
 */

#include "subsystem1.h"
#include "cmd_parse.h"
#include "tel_bin.h"
#include "../test_report.h"

#if APP_TEST

/* A 4-byte command, as the laptop sends it */
LOCAL const uint8_t *cmd(UB seq, W verb, W param, W value)
{
	static uint8_t	b[CMD_BYTES];
	CMD		c;

	c.seq = seq;
	c.verb = verb;
	c.param = param;
	c.value = value;
	cmd_encode(&c, b);
	return b;
}

EXPORT void subsystem1_test(void)
{
	CMD		c;
	uint32_t	seq = 0U;
	TEL_DATA	d = {0};
	UB		rec[NET_PAYLOAD_MAX];
	static const uint8_t bad_verb[CMD_BYTES] = {2U, 7U, 0U, 0U};	/* verb 7 */

	TEST_INIT();

	TEST_CHECK("S1-U01", (cmd_parse(cmd(1U, CMD_START, 0, 0), CMD_BYTES, &seq, true, &c) == CMD_OK) &&
		   (c.verb == 1), "4-byte START accepted");
	TEST_CHECK("S1-U02", cmd_parse(cmd(1U, CMD_STOP, 0, 0), CMD_BYTES, &seq, true, &c) == CMD_ERR_REPEAT,
		   "repeated sequence number refused");
	TEST_CHECK("S1-U03", cmd_parse(bad_verb, CMD_BYTES, &seq, true, &c) == CMD_ERR_VERB,
		   "unknown verb refused");
	TEST_CHECK("S1-U04", cmd_parse(cmd(3U, CMD_VERB_SET, (W)PARAM_BASE_SPEED, 999), CMD_BYTES, &seq, true, &c) ==
		   CMD_ERR_RANGE, "SET outside its range refused");
	TEST_CHECK("S1-U05", cmd_parse(cmd(4U, CMD_VERB_SET, (W)PARAM_BASE_SPEED, 220), CMD_BYTES, &seq, false, &c) ==
		   CMD_ERR_STATE, "SET refused while running");
	TEST_CHECK("S1-U06", (cmd_parse(cmd(5U, CMD_VERB_SET, (W)PARAM_STOP_MM, 300), CMD_BYTES, &seq, true, &c) ==
		   CMD_OK) && (c.param == (int32_t)PARAM_STOP_MM) && (c.value == 300), "SET stop_mm 300 accepted");
	TEST_CHECK("S1-U07", param_get(PARAM_BASE_SPEED) == PARAM_UNSET, "parameters start unset");

	d.t_ms = 4000000000U;
	d.v[0] = -400;
	d.v[1] = -400;
	d.enc[0] = -2000000000;
	d.enc[1] = -2000000000;
	d.odo_mm = 2000000000;
	d.barcode = 'A';
	d.range_mm = 4000;
	d.faults = 0xFFFFU;
	TEST_CHECK("S1-U08", tel_bin_telemetry(&d, rec, NET_PAYLOAD_MAX) == (W)TEL_BIN_TELEMETRY_BYTES,
		   "telemetry is a 23-byte binary record");
	TEST_CHECK("S1-U09", tel_bin_status(&d, rec, NET_PAYLOAD_MAX) == (W)TEL_BIN_STATUS_BYTES,
		   "status is an 18-byte binary record");
	TEST_CHECK("S1-U10", (mqtt_tx_mbf > 0) && (mqtt_rx_mbf > 0) && (telemetry_q > 0),
		   "bridge buffers and telemetry_q exist");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
