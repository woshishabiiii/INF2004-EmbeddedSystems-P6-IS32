/*
 *	app_main.c
 *	START HERE.  usermain() below is the program's entry point.
 *	Shared file: changed only by team agreement (TEAM.md).
 *
 *	How micro T-Kernel starts our code
 *	----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On the dual-core build the OTHER core may start a new task at once,
 *	so usermain() works in two phases:
 *	  1. every subsystem creates its kernel objects  (subsystemN_init)
 *	  2. every subsystem starts its tasks             (subsystemN_start)
 *	and then puts the initial task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	The autonomous car: seven application tasks (TEAM.md section 4)
 *	------------------------------------------------------------------
 *	  core 1: Motion Control      pri 6   subsystem2  20 ms + stop wake-up
 *	          Line and Barcode    pri 8   subsystem3  5 ms
 *	          Vehicle Controller  pri 9   subsystem5  events, 20 ms tick
 *	  core 0: Safety Monitor      pri 7   subsystem4  fault_flags, 20 ms
 *	          Obstacle Scan       pri 10  subsystem5  10 ms
 *	          IMU and Terrain     pri 11  subsystem4  10 ms
 *	          Telemetry, Command  pri 14  subsystem1  mqtt_rx_mbf, 100 ms
 *	The kernel's USB console (3) and WiFi/MQTT (4) tasks run on core 0.
 *
 *	Who talks to whom (TEAM.md section 5):
 *	  every task -> Vehicle Controller      vc_event_q    (APP_MSG, app_evt.h)
 *	  Vehicle Controller -> Motion Control  motion_cmd_q  (motion API, subsystem2.h)
 *	  Vehicle Controller -> Obstacle Scan   scan_request_q
 *	  anyone -> Motion Control              stop_flags    (CMD_STOP / FAULT_STOP)
 *	  anyone -> Safety Monitor              fault_flags   (FLT_... bits)
 *	  VC, IMU, Scan -> Telemetry            telemetry_q
 *	  Telemetry <-> kernel MQTT code        mqtt_tx_mbf / mqtt_rx_mbf
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return tid;
}

EXPORT INT usermain(void)
{
	T_RVER	ver;

	/* The 1 us TIMER timestamps encoder, barcode and echo edges */
	app_timer_init();

	tk_ref_ver(&ver);
	tm_printf((UB*)"\n=== Autonomous car on micro T-Kernel, Pico W"
		  " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
		  tk_get_prc() - 1);	/* tk_get_prc() counts from 1 */
#else
	tm_printf((UB*)"single-core build: every task runs on core 0\n");
#endif

	/* Phase 1: every subsystem creates its kernel objects */
	CHECK(subsystem1_init(), "subsystem1_init");
	CHECK(subsystem2_init(), "subsystem2_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem4_init(), "subsystem4_init");
	CHECK(subsystem5_init(), "subsystem5_init");

	/* Phase 2: every subsystem starts its tasks.  Motion Control first:
	   it brakes both motors and enables the shared GPIO interrupt. */
	CHECK(subsystem2_start(), "subsystem2_start");
	CHECK(subsystem4_start(), "subsystem4_start");
	CHECK(subsystem3_start(), "subsystem3_start");
	CHECK(subsystem5_start(), "subsystem5_start");
	CHECK(subsystem1_start(), "subsystem1_start");

	tm_printf((UB*)"all subsystems started\n\n");

#if APP_TEST
	/* Test build (make TEST=1): run every subsystem's tests in turn,
	   here in the initial task, while the system is running.  The car
	   stays in READY (motors braked) unless someone presses START. */
	tm_printf((UB*)"[TEST] === test run start ===\n");
	subsystem1_test();
	subsystem2_test();
	subsystem3_test();
	subsystem4_test();
	subsystem5_test();
	tm_printf((UB*)"[TEST] === test run end ===\n");
#endif

	/* Sleep forever.  Returning from usermain() would shut down. */
	(void)tk_slp_tsk(TMO_FEVR);
	return 0;
}
