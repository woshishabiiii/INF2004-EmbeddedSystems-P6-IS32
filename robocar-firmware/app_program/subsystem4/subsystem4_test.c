/*
 *	subsystem4_test.c
 *	On-target tests for IMU and Terrain, and the Safety Monitor.
 *	Built only with: make TEST=1
 */

#include "subsystem4.h"
#include "imu_cfg.h"
#include "imu_math.h"
#include "../gpio_irq.h"
#include "../test_report.h"

#if APP_TEST

LOCAL bool near(W a, W b, W tol)
{
	return ((a - b) <= tol) && ((b - a) <= tol);
}

EXPORT void subsystem4_test(void)
{
	SM_CORE		sm;
	SM_PARAMS	sp = {{0U, 0U, 0U, 0U, 0U}, 2U};
	SM_OUT		out;
	UW		age[HEALTH_SLOTS] = {0U, 0U, 0U, 0U, 0U};
	bool		alive[HEALTH_SLOTS] = {false, false, false, false, false};
	IMU_STATUS	st;

	TEST_INIT();

	TEST_CHECK("S4-U01", near(imu_atan2_cdeg(1000, 1000), 4500, 2) &&
		   near(imu_atan2_cdeg(-500, 866), -3000, 2) && near(imu_atan2_cdeg(1, -1), 13500, 2),
		   "atan2 within 0.02 degree");
	TEST_CHECK("S4-U02", near(imu_sin_x1e4(3000), 5000, 3) && near(imu_sin_x1e4(-900), -1564, 3),
		   "sine table within 0.0003");

	sm_init(&sm, &sp);
	sm_on_flags(&sm, FLT_SCAN, &out);
	TEST_CHECK("S4-U03", out.latch && sm.faulted && (sm_wait_pattern(&sm) == FLT_RESTART_REQ),
		   "a fault latches FAULT_STOP; then only RESTART is waited for");
	sm_on_flags(&sm, FLT_RESTART_REQ, &out);
	TEST_CHECK("S4-U04", out.clear_flags && !out.release, "RESTART clears the flags, latch still on");
	sm_on_flags(&sm, FLT_SCAN, &out);
	sm_on_poll(&sm, 1U, 1U, age, alive, &out);
	sm_on_poll(&sm, 1U, 1U, age, alive, &out);
	TEST_CHECK("S4-U05", sm.faulted, "a fault raised again cancels the restart");
	sm_on_flags(&sm, FLT_RESTART_REQ, &out);
	sm_on_poll(&sm, 1U, 1U, age, alive, &out);
	sm_on_poll(&sm, 1U, 1U, age, alive, &out);
	TEST_CHECK("S4-U06", !sm.faulted && out.release && (out.ev.n == 1U) &&
		   (out.ev.ev[0].type == EV_FAULT_CLEAR), "faults clear for 2 polls: latch released");

	(void)tk_dly_tsk(1200);			/* calibration takes 1 s */
	imu_get_status(&st);
	if (IMU_REQUIRED != 0) {
		TEST_CHECK("S4-U07", st.ok && st.calibrated, "IMU answers and has calibrated");
		TEST_CHECK("S4-U08", (st.g0_mg > 900) && (st.g0_mg < 1100), "|g| at rest is 900..1100 mg");
	}

	/* I2C0 moved from GP8/GP9 (Motor 1) to GP4/GP5 by the team's kernel
	   configuration: confirm it on the board */
	TEST_CHECK("S4-U09", (gpio_funcsel(4U) == GPIO_FUNC_I2C) && (gpio_funcsel(5U) == GPIO_FUNC_I2C) &&
		   (gpio_funcsel(8U) != GPIO_FUNC_I2C) && (gpio_funcsel(9U) != GPIO_FUNC_I2C),
		   "I2C0 is on GP4/GP5, not on the Motor 1 pins");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
