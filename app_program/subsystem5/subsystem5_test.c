/*
 *	subsystem5_test.c
 *	On-target tests for Obstacle Scan and Vehicle Controller.
 *	Built only with: make TEST=1
 */

#include "subsystem5.h"
#include "scan_cfg.h"
#include "vc_cfg.h"
#include "../gpio_irq.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
	/* a 100 mm wide box 300 mm ahead, centred, wall-free sides */
	int32_t		ang[7] = {75, 80, 85, 90, 95, 100, 105};
	int32_t		rng[7] = {OS_INVALID, 306, 301, 300, 301, 306, OS_INVALID};
	int32_t		far[7] = {900, 306, 301, 300, 301, 306, 900};
	OS_PROFILE	p;
	VC_STATUS	vs;
	SCAN_STATUS	ss;

	TEST_INIT();

	os_build_profile(ang, far, 7U, OS_R_MAX_MM, OS_CORRIDOR_MM, &p);
	TEST_CHECK("S5-U01", p.valid && (p.closest_mm >= 290) && (p.closest_mm <= 300),
		   "box found 300 mm ahead");
	TEST_CHECK("S5-U02", (p.x_left_mm < -40) && (p.x_right_mm > 40) && (p.width_mm < 140),
		   "box edges either side of the centre line");
	os_build_profile(ang, rng, 7U, OS_R_MAX_MM, OS_CORRIDOR_MM, &p);
	TEST_CHECK("S5-U03", p.valid && (p.unknown_left == 1U) && (p.unknown_right == 1U),
		   "missing echoes counted as unknown, not free");
	TEST_CHECK("S5-U04", (os_sin1000(30) == 500) && (os_cos1000(120) == -500), "sin / cos table");

	vc_get_status(&vs);
	TEST_CHECK("S5-U05", (vs.state == (W)VC_READY) || (vs.state == (W)VC_STARTUP) ||
		   (vs.state == (W)VC_SAFE_STOP), "Vehicle Controller waits for START");
	scan_get_status(&ss);
	TEST_CHECK("S5-U06", ss.mode == (W)SCAN_IDLE, "no pinging until START");

	/* UART0's pins are GP0/GP1: confirm the kernel build left them to us */
	TEST_CHECK("S5-U07", (gpio_funcsel(0U) == GPIO_FUNC_SIO) && (gpio_funcsel(1U) == GPIO_FUNC_SIO),
		   "TRIG GP0 and ECHO GP1 are plain GPIO (UART0 not on them)");

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
