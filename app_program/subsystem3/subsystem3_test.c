/*
 *	subsystem3_test.c
 *	On-target tests for Line and Barcode.  Built only with: make TEST=1
 */

#include "subsystem3.h"
#include "line_cfg.h"
#include "../gpio_irq.h"
#include "../test_report.h"

#if APP_TEST

/* Feed the 29 widths of "*c*" into a capture as edges */
LOCAL bool capture(char c, uint32_t narrow, bool backwards, BC_RESULT *res)
{
	BC_CORE		b;
	BC_PARAMS	p = {LB_T_MIN_US, LB_T_END_MIN_MS * 1000U, LB_T_END_FACTOR, 1U};
	uint32_t	w[CODE39_ELEMENTS];
	uint32_t	t = 1000U, i;
	uint8_t		level = 1U;

	if (!code39_encode(c, narrow, narrow * 3U, w)) {
		return false;
	}
	bc_init(&b, &p);
	for (i = 0U; i < CODE39_ELEMENTS; i++) {
		bc_edge(&b, t, level);
		t += w[backwards ? (CODE39_ELEMENTS - 1U - i) : i];
		level ^= 1U;
	}
	bc_edge(&b, t, 0U);			/* end of the last bar */
	return bc_poll(&b, t + 1000000U, res);
}

EXPORT void subsystem3_test(void)
{
	BC_RESULT	r;
	LC_CORE		lc;
	LC_PARAMS	lp = {LB_JUNCTION_MS, LB_JUNCTION_MM, LB_T_LOST_MS};
	EVT_LIST	ev;
	LB_STATUS	st;

	TEST_INIT();

	TEST_CHECK("S3-U01", capture('A', 15000U, false, &r) && (r.result == C39_OK) && (r.c == 'A'),
		   "*A* decodes forwards (15 ms narrow bars)");
	TEST_CHECK("S3-U02", capture('A', 15000U, true, &r) && (r.result == C39_OK) && (r.c == 'A') &&
		   r.reversed, "*A* decodes backwards");
	TEST_CHECK("S3-U03", capture('Z', 4000U, false, &r) && (r.result == C39_OK) && (r.c == 'Z') &&
		   (bc_command_for(r.c) == LB_CMD_NONE), "*Z* decodes, no command");
	TEST_CHECK("S3-U04", (bc_command_for('A') == LB_CMD_TURN_LEFT) &&
		   (bc_command_for('B') == LB_CMD_TURN_RIGHT) && (bc_command_for('C') == LB_CMD_STRAIGHT) &&
		   (bc_command_for('D') == LB_CMD_U_TURN), "command table A-D");

	lc_init(&lc, &lp, 0U);
	evt_list_clear(&ev);
	lc_step(&lc, true, true, 0U, 0, &ev);
	lc_step(&lc, true, true, 20U, 10, &ev);
	TEST_CHECK("S3-U05", (ev.n == 2U) && (ev.ev[1].type == EV_JUNCTION),
		   "both black for 20 ms and 10 mm = junction");
	evt_list_clear(&ev);
	lc_step(&lc, false, false, 2000U, 10, &ev);
	TEST_CHECK("S3-U06", (ev.n == 2U) && (ev.ev[1].type == EV_LINE_LOST), "no black for T_lost = line lost");

	(void)tk_dly_tsk(20);
	lb_get_status(&st);
	TEST_CHECK("S3-U07", (st.reads_ok + st.reads_failed) == 0U, "no barcode read while standing still");

	/* I2C1's default pins are GP6/GP7: confirm the kernel did not start it */
	TEST_CHECK("S3-U08", (gpio_funcsel(6U) == GPIO_FUNC_SIO) && (gpio_funcsel(7U) == GPIO_FUNC_SIO) &&
		   (gpio_funcsel(27U) == GPIO_FUNC_SIO), "line and barcode DO pins are plain GPIO (I2C1 off)");

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
