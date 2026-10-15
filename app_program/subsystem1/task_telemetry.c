/*
 *	task_telemetry.c
 *	Telemetry and Command task, core 0, priority 14 (the lowest).
 *	  - waits on mqtt_rx_mbf for up to 100 ms, so a remote STOP is
 *	    handled at once;
 *	  - validates commands (cmd_parse.c) and forwards accepted ones to
 *	    Vehicle Controller; SET values go into param_store;
 *	  - turns records from telemetry_q into event messages;
 *	  - every 500 ms formats telemetry and every 1 s a heartbeat.
 *	Messages go to mqtt_tx_mbf for the kernel's MQTT code to publish
 *	(MQTT=1 builds), or to the USB console (other builds).  It never
 *	calls cyw43, lwIP or MQTT functions; cyw43_utk_get_status() is the
 *	one read the template allows.
 */

#include "subsystem1.h"
#include "cmd_parse.h"
#include "tel_json.h"
#if defined(TM_WIFI_CYW43) && TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif
#include "../subsystem2/subsystem2.h"	/* motion_get_snapshot() */
#include "../subsystem3/subsystem3.h"	/* lb_get_status() */
#include "../subsystem4/subsystem4.h"	/* imu_get_status(), safety_latched_bits() */
#include "../subsystem5/subsystem5.h"	/* vc_event_q, vc_get_status(), scan_*() */

#define TEL_RX_TMO_MS		100

#if defined(TM_MQTT) && TM_MQTT
#define TEL_PERIOD_MS		500U	/* telemetry topic */
#define HB_PERIOD_MS		1000U	/* status topic (heartbeat) */
#define TEL_ECHO		0	/* MQTT build: messages are published */
#else
#define TEL_PERIOD_MS		1000U	/* no MQTT: print to the USB console */
#define HB_PERIOD_MS		5000U	/* ... less often, so other prints stay readable */
#define TEL_ECHO		1
#endif

LOCAL W		s_params[CMD_NPARAMS] = {
	PARAM_UNSET, PARAM_UNSET, PARAM_UNSET, PARAM_UNSET, PARAM_UNSET, PARAM_UNSET
};
LOCAL uint32_t	s_last_seq;
LOCAL UW	s_tx_drops;
LOCAL UH	s_seq;

EXPORT W param_get(UINT index)
{
	return (index < CMD_NPARAMS) ? s_params[index] : PARAM_UNSET;
}

/* Queue one JSON message for publishing (never waits), or print it */
LOCAL void publish(UB topic, const char *json, W len)
{
	UB	buf[NET_PAYLOAD_MAX + 1U];
	W	i;

	if ((len <= 0) || (len > (W)NET_PAYLOAD_MAX)) {
		s_tx_drops++;
		return;
	}
#if defined(TM_MQTT) && TM_MQTT
	buf[0] = topic;
	for (i = 0; i < len; i++) {
		buf[i + 1] = (UB)json[i];
	}
	if (tk_snd_mbf(mqtt_tx_mbf, buf, (INT)(len + 1), TMO_POL) != E_OK) {
		s_tx_drops++;			/* network slow or down: drop, count */
	}
#endif
#if TEL_ECHO
	for (i = 0; i < len; i++) {
		buf[i] = (UB)json[i];
	}
	buf[len] = 0U;
	tm_printf((UB*)"[TEL] %s %s\n", (const UB*)net_topic_name(topic), buf);
#endif
}

LOCAL void gather(TEL_DATA *d)
{
	MC_SNAPSHOT	snap = {0};
	LB_STATUS	lb;
	IMU_STATUS	imu;
	VC_STATUS	vc;
	SCAN_STATUS	sc;
#if defined(TM_WIFI_CYW43) && TM_WIFI_CYW43
	T_CYW43_UTK_STATUS st;
#endif

	(void)motion_get_snapshot(&snap);
	lb_get_status(&lb);
	imu_get_status(&imu);
	vc_get_status(&vc);
	scan_get_status(&sc);

	d->t_ms = app_ms();
	d->state = vc.state;
	d->why = vc.why;
	d->v[0] = snap.speed[MC_LEFT];
	d->v[1] = snap.speed[MC_RIGHT];
	d->enc[0] = snap.count[MC_LEFT];
	d->enc[1] = snap.count[MC_RIGHT];
	d->odo_mm = snap.odo_mm;
	d->yaw_cdeg_s = snap.yaw_cdeg_s;
	d->mc_fault = snap.fault_code;
	d->line_l = (lb.left_black != FALSE);
	d->line_r = (lb.right_black != FALSE);
	d->barcode = lb.last_char;
	d->pending = vc.pending;
	d->pitch_cdeg = imu.pitch_cdeg;
	d->motion = imu.motion;
	d->hmax_mm = imu.max_hump_mm;
	d->imu_ok = (imu.ok != FALSE);
	d->heading_cdeg = imu.heading_cdeg;
	d->range_mm = sc.range_mm;
	d->pings = sc.pings;
	d->faults = safety_latched_bits();
#if defined(TM_WIFI_CYW43) && TM_WIFI_CYW43
	cyw43_utk_get_status(&st);
	d->wifi = st.link_status;
#else
	d->wifi = 0;
#endif
	d->mqtt_up = (net_up != FALSE);
	d->mqtt_connects = net_connects;
	d->tx_drops = s_tx_drops;
	d->rx_drops = net_rx_drops;
	d->net_errors = net_errors;
}

LOCAL void handle_command(const UB *text, INT len)
{
	CMD		c;
	cmd_err_t	r;
	VC_STATUS	vc;
	char		json[NET_PAYLOAD_MAX];
	W		n;

	vc_get_status(&vc);
	r = cmd_parse((const char *)text, (uint32_t)len, &s_last_seq,
		      (vc.state == (W)VC_READY) || (vc.state == (W)VC_FINISHED), &c);
	if (r == CMD_OK) {
		if (c.verb == CMD_VERB_SET) {
			s_params[c.param] = c.value;
			tm_printf((UB*)"[TEL] SET %s = %d (used from the next START)\n",
				  (const UB*)cmd_params[c.param].name, c.value);
		} else if (app_send(vc_event_q, (UB)SRC_TELEMETRY, &s_seq, (UB)EV_COMMAND,
				    c.verb, c.value) != E_OK) {
			(void)tk_set_flg(fault_flags, FLT_QUEUE_OVF);
		} else {
			/* forwarded */
		}
	} else {
		tm_printf((UB*)"[TEL] command refused, error %d (seq %u)\n", (INT)r, c.seq);
	}
	n = tel_fmt_ack(app_ms(), c.seq, (r == CMD_OK), (int32_t)r, json, NET_PAYLOAD_MAX);
	publish((UB)TOPIC_EVENT, json, n);
}

LOCAL void take_record(const APP_MSG *m)
{
	char		json[NET_PAYLOAD_MAX];
	W		n;

	if (m->type == EV_PROFILE_DATA) {
		OS_PROFILE p;

		scan_profile_get(&p);
		n = tel_fmt_profile(m->t_ms, p.valid, p.x_left_mm, p.x_right_mm, p.closest_mm,
				    p.closest_deg, p.left_clear_mm, p.right_clear_mm,
				    p.unknown_left, p.unknown_right, json, NET_PAYLOAD_MAX);
	} else {
		n = tel_fmt_event(m, json, NET_PAYLOAD_MAX);
	}
	if (n != 0) {
		publish((UB)TOPIC_EVENT, json, n);
	}

	if ((m->type == EV_NAV) && (m->p1 == NAV_FINISH)) {
		TEL_SUMMARY	s;
		VC_STATUS	vc;
		IMU_STATUS	imu;
		LB_STATUS	lb;

		vc_get_status(&vc);
		imu_get_status(&imu);
		lb_get_status(&lb);
		s.t_ms = m->t_ms;
		s.time_ms = m->t_ms - vc.run_start_ms;
		s.dist_mm = m->p2;
		s.hmax_mm = imu.max_hump_mm;
		s.humps = imu.humps;
		s.commands = vc.commands_done;
		s.obstacles = vc.obstacles_passed;
		s.reads_ok = lb.reads_ok;
		s.reads_failed = lb.reads_failed;
		n = tel_fmt_summary(&s, json, NET_PAYLOAD_MAX);
		publish((UB)TOPIC_EVENT, json, n);
		tm_printf((UB*)"[TEL] RUN SUMMARY: %u ms, %d mm, highest hump %d mm\n",
			  s.time_ms, s.dist_mm, s.hmax_mm);
	}
}

EXPORT void telemetry_task(INT stacd, void *exinf)
{
	UB		rx[NET_CMD_MAX + 1U];
	char		json[NET_PAYLOAD_MAX];
	APP_MSG		m;
	TEL_DATA	d;
	UW		last_tel = 0U, last_hb = 0U, now;
	INT		len;
	W		n;

	(void)stacd;
	(void)exinf;

	while (1) {
		/* 1. Commands: wait up to 100 ms */
		len = tk_rcv_mbf(mqtt_rx_mbf, rx, TEL_RX_TMO_MS);
		if (len > 0) {
			handle_command(rx, len);
		}

		/* 2. Records from the other tasks, without waiting */
		while (tk_rcv_mbf(telemetry_q, &m, TMO_POL) == (INT)sizeof(m)) {
			take_record(&m);
		}

		/* 3. Periodic messages */
		now = app_ms();
		if ((now - last_tel) >= TEL_PERIOD_MS) {
			last_tel = now;
			gather(&d);
			n = tel_fmt_telemetry(&d, json, NET_PAYLOAD_MAX);
			publish((UB)TOPIC_TELEMETRY, json, n);
		}
		if ((now - last_hb) >= HB_PERIOD_MS) {
			last_hb = now;
			gather(&d);
			n = tel_fmt_status(&d, json, NET_PAYLOAD_MAX);
			publish((UB)TOPIC_STATUS, json, n);
		}
	}
}
