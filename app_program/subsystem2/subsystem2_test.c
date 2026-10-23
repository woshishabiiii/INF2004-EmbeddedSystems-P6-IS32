/*
 *	subsystem2_test.c
 *	On-target tests for Motion Control.  Built only with: make TEST=1
 *	The car stays in READY (CMD_STOP set), so the motors never turn.
 */

#include "subsystem2.h"
#include "motion_cfg.h"
#include "../gpio_irq.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	MC_SNAPSHOT	a, b;
	MC_CMD		cmd = {0};
	T_RFLG		rflg;

	TEST_INIT();

	TEST_CHECK("S2-U01", motion_move_forward(0, 100, 1) == E_PAR, "zero distance refused");
	TEST_CHECK("S2-U02", motion_move_forward(100, MC_MAX_SPEED + 1, 1) == E_PAR,
		   "speed above the limit refused");
	TEST_CHECK("S2-U03", motion_turn_left(361, 1) == E_PAR, "turn over 360 degrees refused");
	cmd.type = MC_CMD_SPEED;
	cmd.a = 100;
	cmd.b = -100;
	TEST_CHECK("S2-U04", mc_cmd_valid(&cmd, MC_MAX_SPEED), "speed command accepted");

	TEST_CHECK("S2-U05", (tk_ref_flg(stop_flags, &rflg) == E_OK) &&
		   ((rflg.flgptn & MC_CMD_STOP) != 0U), "CMD_STOP is set at start-up (READY)");

	(void)motion_get_snapshot(&a);
	(void)tk_dly_tsk(100);
	(void)motion_get_snapshot(&b);
	TEST_CHECK("S2-U06", ((b.seq - a.seq) & 0xFFU) >= 4U, "control loop runs every 20 ms");
	TEST_CHECK("S2-U07", (b.duty[MC_LEFT] == 0) && (b.duty[MC_RIGHT] == 0),
		   "both motors braked while stopped");
	TEST_CHECK("S2-U08", MC_COUNTS_PER_90DEG > 0, "counts per 90 degrees is positive");

	/* Design-review feedback: confirm on the real kernel build that
	   nothing reconfigured our pins (GP16 in particular).  GP25 is the
	   radio's chip select: printed for the record, never claimed. */
	TEST_CHECK("S2-U09", (gpio_funcsel(2U) == GPIO_FUNC_SIO) && (gpio_funcsel(3U) == GPIO_FUNC_SIO) &&
		   (gpio_funcsel(16U) == GPIO_FUNC_SIO) && (gpio_funcsel(17U) == GPIO_FUNC_SIO),
		   "encoder pins GP2, GP3, GP16, GP17 are plain GPIO (SIO)");
	tm_printf((UB*)"[TEST] info: GP25 (radio chip select) FUNCSEL = %u\n", gpio_funcsel(25U));

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
