/*
 *	bench.c
 *	Week 13 evidence (Change Requirement 1): how much CPU the data
 *	representation costs, before and after.  Each bench_...() function does
 *	one piece of work with typical values.  measure.sh builds this file for
 *	the Cortex-M0+ (the firmware's -O2 -mcpu=cortex-m0plus), once with
 *	-DV12 against the V12 tree and once against this tree, and
 *	count_instr.py runs every bench_...() in an ARM emulator and counts the
 *	instructions it executes.  Nothing here is flashed.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app_evt.h"
#include "subsystem1/cmd_parse.h"
#include "subsystem2/mc_core.h"
#include "subsystem3/line_core.h"
#include "subsystem4/imu_core.h"
#include "subsystem5/os_profile.h"
#include "subsystem5/scan_core.h"
#include "subsystem5/vc_core.h"
#ifdef V12
#include "subsystem1/tel_json.h"
#else
#include "subsystem1/tel_bin.h"
#endif

#define BENCH	__attribute__((noinline, used))

static volatile int32_t	g_sink;
static uint8_t		g_buf[256];
static TEL_DATA		g_d;

/* the same 12 event types msg_sizes.c uses */
static const struct { uint8_t type; int32_t p1; int32_t p2; } k_ev[] = {
	{EV_LINE_LOST, 1, 0}, {EV_JUNCTION, 2413, 0}, {EV_BARCODE, 'A', 1},
	{EV_BARCODE_BAD, 3, 9}, {EV_FAULT, 0x001, 0}, {EV_FAULT_CLEAR, 0, 0},
	{EV_TERRAIN, 1, 520}, {EV_HUMP, 18, 18}, {EV_MOTION, 4, 0},
	{EV_IMPACT, 1600, 180}, {EV_STATE, 2, 0}, {EV_NAV, 2, 1}
};
#define N_EV	(sizeof(k_ev) / sizeof(k_ev[0]))

#ifndef V12
void	bench_status_write_line(void);
void	bench_status_write_scan(void);
void	bench_status_write_imu(void);
void	bench_status_write_vc(void);
#endif

BENCH void bench_setup(void)
{
	memset(&g_d, 0, sizeof(g_d));
	g_d.t_ms = 61234U;
	g_d.state = 2;
	g_d.v[0] = 182;
	g_d.v[1] = 176;
	g_d.enc[0] = 18234;
	g_d.enc[1] = 18101;
	g_d.odo_mm = 2413;
	g_d.yaw_cdeg_s = -350;
	g_d.line_l = true;
	g_d.barcode = 'A';
	g_d.pitch_cdeg = 120;
	g_d.motion = 1;
	g_d.hmax_mm = 18;
	g_d.range_mm = 734;
	g_d.imu_ok = true;
	g_d.heading_cdeg = 9012;
	g_d.pings = 912U;
	g_d.wifi = 3;
#ifdef V12
	g_d.mqtt_up = true;	/* V12 also sent the link state as its own field */
#endif
	g_d.mqtt_connects = 1U;
#ifndef V12
	bench_status_write_line();	/* the readers below find real words */
	bench_status_write_scan();
	bench_status_write_imu();
	bench_status_write_vc();
#endif
}

/* ---- MQTT: one message each ------------------------------------------------ */
#ifdef V12
BENCH void bench_mqtt_telemetry(void) { g_sink = tel_fmt_telemetry(&g_d, (char *)g_buf, TEL_JSON_MAX); }
BENCH void bench_mqtt_status(void) { g_sink = tel_fmt_status(&g_d, (char *)g_buf, TEL_JSON_MAX); }

/* telemetry_q delivers the 16 B APP_MSG as it is: format it */
BENCH void bench_mqtt_events_x12(void)
{
	uint32_t i;

	for (i = 0U; i < N_EV; i++) {
		APP_MSG m = {k_ev[i].type, SRC_VC, 7U, 61234U, k_ev[i].p1, k_ev[i].p2};

		g_sink = tel_fmt_event(&m, (char *)g_buf, TEL_JSON_MAX);
	}
}

BENCH void bench_mqtt_command(void)
{
	static const char	text[] = "16 SET base_speed 220";
	uint32_t		last = 15U;
	CMD			c;

	g_sink = (int32_t)cmd_parse(text, (uint32_t)(sizeof(text) - 1U), &last, true, &c);
}
#else
BENCH void bench_mqtt_telemetry(void) { g_sink = tel_bin_telemetry(&g_d, g_buf, TEL_BIN_MAX); }
BENCH void bench_mqtt_status(void) { g_sink = tel_bin_status(&g_d, g_buf, TEL_BIN_MAX); }

/* telemetry_q delivers the 8 B APP_MSG packed: unpack it, then pack the record */
BENCH void bench_mqtt_events_x12(void)
{
	uint32_t i;

	for (i = 0U; i < N_EV; i++) {
		APP_EVT	e = {k_ev[i].type, SRC_VC, 7U, 61234U, k_ev[i].p1, k_ev[i].p2};
		APP_MSG	m;

		app_msg_pack(&e, &m);
		app_msg_unpack(&m, &e);
		g_sink = tel_bin_event(&e, 61234U, g_buf, TEL_BIN_MAX);
	}
}

BENCH void bench_mqtt_command(void)
{
	static const uint8_t	bytes[CMD_BYTES] = {0x10U, 0x05U, 0xDCU, 0x00U};	/* 16 SET base_speed 220 */
	uint32_t		last = 15U;
	CMD			c;

	g_sink = (int32_t)cmd_parse(bytes, CMD_BYTES, &last, true, &c);
}

/* ---- inter-task messages and shared records (new work, after only) -------- */
BENCH void bench_app_msg_x12(void)		/* every event: pack (sender) + unpack (receiver) */
{
	uint32_t i;

	for (i = 0U; i < N_EV; i++) {
		APP_EVT	e = {k_ev[i].type, SRC_VC, 7U, 61234U, k_ev[i].p1, k_ev[i].p2};
		APP_MSG	m;

		app_msg_pack(&e, &m);
		app_msg_unpack(&m, &e);
		g_sink = e.p1;
	}
}

BENCH void bench_motion_cmd(void)		/* every motion command: pack + unpack */
{
	MC_CMD		c = {MC_CMD_DISTANCE, {0U, 0U, 0U}, 600, 250, 17};
	MC_CMD_PK	pk;

	mc_cmd_pack(&c, &pk);
	mc_cmd_unpack(&pk, &c);
	g_sink = c.a;
}

static MC_SNAPSHOT	g_snap;
static MC_SNAP_PK	g_snap_pk;

BENCH void bench_snapshot_pack(void)	/* Motion Control, every 20 ms, outside the lock */
{
	g_snap.t_ms = 61234U;
	g_snap.speed[0] = 182;
	g_snap.speed[1] = 176;
	g_snap.count[0] = 18234;
	g_snap.count[1] = 18101;
	g_snap.odo_mm = 2413;
	g_snap.dist_mm = 1200;
	mc_snap_pack(&g_snap, &g_snap_pk);
	g_sink = (int32_t)g_snap_pk.w[0];
}

BENCH void bench_snapshot_unpack(void)	/* each reader, outside the lock */
{
	MC_SNAPSHOT s;

	mc_snap_unpack(&g_snap_pk, &s);
	g_sink = s.odo_mm;
}

BENCH void bench_snapshot_odo(void)	/* Line and VC: the odometer only, outside the lock */
{
	g_sink = mc_snap_odo(&g_snap_pk);
}

BENCH void bench_profile(void)		/* one obstacle profile: pack + unpack */
{
	OS_PROFILE	p, q;
	OS_PROFILE_PK	pk;

	memset(&p, 0, sizeof(p));
	p.valid = true;
	p.x_left_mm = -120;
	p.x_right_mm = 80;
	p.width_mm = 200;
	p.closest_mm = 240;
	p.closest_deg = 95;
	p.left_clear_mm = 300;
	p.right_clear_mm = 150;
	p.npoints = 25U;
	os_profile_pack(&p, &pk);
	os_profile_unpack(&pk, &q);
	g_sink = q.closest_mm;
}

/* ---- status words: each writer packs its record into its shared words ---- */
LC_STATUS		g_lc = {true, false, 'A', 3U, 1U, 4U};
IC_STATUS		g_ic = {true, true, 120, 1, 1, 18, 18, 1U, 9012, 1000};
VC_ST			g_vs = {2, 0, 0, 3U, 1U, 1000U, 0, 4U};
SCAN_ST			g_ss = {SCAN_WATCH, 734, 912U};
volatile uint32_t	g_lc_w, g_ss_w, g_ic_w[3], g_vs_w[3];

BENCH void bench_status_write_line(void) { g_lc_w = lc_status_pack(&g_lc); }	/* Line, every 5 ms */
BENCH void bench_status_write_scan(void) { g_ss_w = scan_st_pack(&g_ss); }	/* Scan, every 10 ms */

BENCH void bench_status_write_imu(void)	/* IMU, every 10 ms */
{
	IC_STATUS_PK pk;

	ic_status_pack(&g_ic, &pk);
	g_ic_w[0] = pk.w[0];
	g_ic_w[1] = pk.w[1];
	g_ic_w[2] = pk.w[2];
}

BENCH void bench_status_write_vc(void)	/* Vehicle Controller, every loop */
{
	VC_ST_PK pk;

	vc_st_pack(&g_vs, &pk);
	g_vs_w[0] = pk.w[0];
	g_vs_w[1] = pk.w[1];
	g_vs_w[2] = pk.w[2];
}

/* ... and each reader loads the words and unpacks them */
BENCH void bench_status_read_imu(void)	/* VC, every 20 ms tick */
{
	IC_STATUS_PK	pk;
	IC_STATUS	s;

	pk.w[0] = g_ic_w[0];
	pk.w[1] = g_ic_w[1];
	pk.w[2] = g_ic_w[2];
	ic_status_unpack(&pk, &s);
	g_sink = s.pitch_cdeg;
}

BENCH void bench_status_read_vc(void)	/* IMU, every 10 ms */
{
	VC_ST_PK	pk;
	VC_ST		s;

	pk.w[0] = g_vs_w[0];
	pk.w[1] = g_vs_w[1];
	pk.w[2] = g_vs_w[2];
	vc_st_unpack(&pk, &s);
	g_sink = s.state;
}

BENCH void bench_status_read_line(void)	/* Telemetry only */
{
	LC_STATUS s;

	lc_status_unpack(g_lc_w, &s);
	g_sink = s.last_char;
}

BENCH void bench_status_read_scan(void)	/* Telemetry only */
{
	SCAN_ST s;

	scan_st_unpack(g_ss_w, &s);
	g_sink = s.range_mm;
}
#endif

/* ---- V12: the getters copied the whole status struct (no lock) ------------
   Word structs the size of V12's IMU_STATUS (40 B), VC_STATUS (32 B), line
   status (24 B) and SCAN_STATUS (12 B); V12's writers stored the fields
   directly, so they have no "write" cost to compare. */
#ifdef V12
typedef struct { uint32_t w[10]; } V12_IMU_STATUS;
typedef struct { uint32_t w[8]; } V12_VC_STATUS;
typedef struct { uint32_t w[6]; } V12_LINE_STATUS;
typedef struct { uint32_t w[3]; } V12_SCAN_STATUS;
V12_IMU_STATUS		g_imu_src, g_imu_dst;
V12_VC_STATUS		g_vc_src, g_vc_dst;
V12_LINE_STATUS		g_line_src, g_line_dst;
V12_SCAN_STATUS		g_scan_src, g_scan_dst;

BENCH void bench_status_read_imu(void) { g_imu_dst = g_imu_src; }
BENCH void bench_status_read_vc(void) { g_vc_dst = g_vc_src; }
BENCH void bench_status_read_line(void) { g_line_dst = g_line_src; }
BENCH void bench_status_read_scan(void) { g_scan_dst = g_scan_src; }
#endif

/* ---- the copy made while holding mtx_snapshot (each reader) --------------- */
#ifdef V12
MC_SNAPSHOT	g_lock_src, g_lock_dst;		/* V12: the whole 88 B struct */
#else
MC_SNAP_PK	g_lock_src, g_lock_dst;		/* min-bit: the 36 B packed copy */
#endif

BENCH void bench_snapshot_copy_in_lock(void)
{
	g_lock_dst = g_lock_src;
}

/* never run: keeps every bench_...() and the linker happy */
int main(void)
{
	bench_setup();
	return 0;
}
