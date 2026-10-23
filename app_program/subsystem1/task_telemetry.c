/*
 *	task_telemetry.c
 *	Telemetry and Command task, core 0, priority 14 (the lowest).
 *	  - waits on mqtt_rx_mbf for up to 100 ms, so a remote STOP is
 *	    handled at once;
 *	  - validates 4-byte binary commands (cmd_parse.c) and forwards accepted ones to
 *	    Vehicle Controller; SET values go into param_store;
 *	  - turns records from telemetry_q into event messages;
 *	  - every 500 ms builds telemetry and every 1 s a heartbeat;
 *	  - every message is a compact binary record (tel_bin.h), decoded on
 *	    the laptop by tools/mqtt_decode.py;
 *	  - every 5 s a "tim" event: task run times, missed releases, GPIO
 *	    interrupt load, vc_event_q backlog and drops, steering and stop
 *	    latency, stopping distance, reconnect time (feedback IS32).
 *	Messages go to mqtt_tx_mbf for the kernel's MQTT code to publish
 *	(MQTT=1 builds), or to the USB console as hex (other builds).  It never
 *	calls cyw43, lwIP or MQTT functions; cyw43_utk_get_status() is the
 *	one read the template allows.
 */

#include "subsystem1.h"
#include "cmd_parse.h"
#include "tel_bin.h"
#if defined(TM_WIFI_CYW43) && TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif
#include "../subsystem2/subsystem2.h"	/* motion_get_snapshot() */
#include "../bitpack.h"
#include "../subsystem3/subsystem3.h"	/* lb_get_status() */
#include "../subsystem4/subsystem4.h"	/* imu_get_status(), safety_latched_bits() */
#include "../subsystem5/subsystem5.h"	/* vc_event_q, vc_get_status(), scan_*() */
#include "../gpio_irq.h"		/* gpio_irq_calls, gpio_irq_max_us */

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
#define TIM_PERIOD_MS		5000U	/* timing message ("tim" event) */

LOCAL W		s_params[CMD_NPARAMS] = {
	PARAM_UNSET, PARAM_UNSET, PARAM_UNSET, PARAM_UNSET, PARAM_UNSET, PARAM_UNSET
};
LOCAL uint32_t	s_last_seq;	/* 8-bit command seq, last accepted (0 after a reset) */
LOCAL UH	s_tx_drops;	/* 16 bits as in the status record: stops at 65,535 */
LOCAL UB	s_seq;		/* 8-bit APP_MSG sequence, wraps */

EXPORT W param_get(UINT index)
{
	return (index < CMD_NPARAMS) ? s_params[index] : PARAM_UNSET;
}

LOCAL void count_tx_drop(void)
{
	if (s_tx_drops < 0xFFFFU) {
		s_tx_drops++;
	}
}

/* One message into mqtt_tx_mbf: [topic number][payload] */
LOCAL void send_mbf(UB topic, const UB *data, W len)
{
#if defined(TM_MQTT) && TM_MQTT
	UB	buf[NET_PAYLOAD_MAX + 1U];
	W	i;

	buf[0] = topic;
	for (i = 0; i < len; i++) {
		buf[i + 1] = data[i];
	}
	if (tk_snd_mbf(mqtt_tx_mbf, buf, (INT)(len + 1), TMO_POL) != E_OK) {
		count_tx_drop();		/* network slow or down: drop, count */
	}
#else
	(void)topic;
	(void)data;
	(void)len;
#endif
}

/* Queue one binary record for publishing (never waits), or print it as hex */
LOCAL void publish(UB topic, const UB *rec, W len)
{
#if TEL_ECHO
	W	i;
#endif

	if ((len <= 0) || (len > (W)TEL_BIN_MAX)) {
		count_tx_drop();
		return;
	}
	send_mbf(topic, rec, len);
#if TEL_ECHO
	/* paste a line into: python3 tools/mqtt_decode.py --hex <bytes> */
	tm_printf((UB*)"[TEL] %s", (const UB*)net_topic_name(topic));
	for (i = 0; i < len; i++) {
		tm_printf((UB*)" %02x", (UINT)rec[i]);
	}
	tm_printf((UB*)"\n");
#endif
}

/* The non-latching status bits (FLT_NO_ECHO, FLT_WIFI_DOWN) as they are now */
LOCAL UW status_bits(void)
{
	T_RFLG rflg;

	return (tk_ref_flg(fault_flags, &rflg) == E_OK) ? (rflg.flgptn & FLT_STATUS) : 0U;
}

LOCAL void gather(TEL_DATA *d)
{
	MC_SNAPSHOT	snap = {0};
	LB_STATUS	lb;
	IMU_STATUS	imu;
	VC_STATUS	vc;
	SCAN_STATUS	sc;
	UW		st_bits;
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
	d->mc_fault = (int32_t)snap.fault_bits;
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
	st_bits = status_bits();
	d->faults = safety_latched_bits() | st_bits;	/* 12 bits on the wire */
#if defined(TM_WIFI_CYW43) && TM_WIFI_CYW43
	cyw43_utk_get_status(&st);
	d->wifi = st.link_status;
#else
	d->wifi = 0;
#endif
	d->mqtt_connects = net_connects;
	d->tx_drops = s_tx_drops;
	d->rx_drops = net_rx_drops;
	d->net_errors = net_errors;
	d->reconnect_ms = net_reconnect_ms;
}

LOCAL void handle_command(const UB *data, INT len)
{
	CMD		c;
	cmd_err_t	r;
	VC_STATUS	vc;
	UB		rec[NET_PAYLOAD_MAX];
	W		n;

	vc_get_status(&vc);
	r = cmd_parse(data, (uint32_t)len, &s_last_seq,
		      (vc.state == (W)VC_READY) || (vc.state == (W)VC_FINISHED) ||
		      (vc.state == (W)VC_HALTED), &c);
	if (r == CMD_OK) {
		if (c.verb == CMD_VERB_SET) {
			s_params[c.param] = c.value;
			tm_printf((UB*)"[TEL] SET %s = %d (used from the next START)\n",
				  (const UB*)cmd_params[c.param].name, c.value);
		} else if (app_send_event(vc_event_q, (UB)SRC_TELEMETRY, &s_seq, (UB)EV_COMMAND,
					  c.verb, c.value, NULL) != E_OK) {
			(void)tk_set_flg(fault_flags, FLT_QUEUE_OVF);	/* a command is a safety event */
		} else {
			/* forwarded */
		}
	} else {
		tm_printf((UB*)"[TEL] command refused, error %d (seq %u)\n", (INT)r, c.seq);
	}
	n = tel_bin_ack(app_ms(), c.seq, (r == CMD_OK), (int32_t)r, rec, NET_PAYLOAD_MAX);
	publish((UB)TOPIC_EVENT, rec, n);
}

LOCAL void take_record(const APP_EVT *m)
{
	UB	rec[NET_PAYLOAD_MAX];
	UW	t_ms = bp_expand16(app_ms(), m->t_ms);	/* t_ms is 16-bit: full time */
	W	n;

	if (m->type == EV_PROFILE_DATA) {
		OS_PROFILE p;

		n = 0;
		if (scan_profile_get(&p) == E_OK) {
			n = tel_bin_profile(t_ms, &p, rec, NET_PAYLOAD_MAX);
		}
	} else {
		n = tel_bin_event(m, t_ms, rec, NET_PAYLOAD_MAX);
	}
	if (n != 0) {
		publish((UB)TOPIC_EVENT, rec, n);
	}

	if ((m->type == EV_NAV) && (m->p1 == NAV_FINISH)) {
		TEL_SUMMARY	s;
		VC_STATUS	vc;
		IMU_STATUS	imu;
		LB_STATUS	lb;

		vc_get_status(&vc);
		imu_get_status(&imu);
		lb_get_status(&lb);
		s.t_ms = t_ms;
		s.time_ms = t_ms - vc.run_start_ms;	/* both 32-bit: a run can last over 65.5 s */
		s.dist_mm = m->p2;
		s.hmax_mm = imu.max_hump_mm;
		s.humps = imu.humps;
		s.commands = vc.commands_done;
		s.obstacles = vc.obstacles_passed;
		s.reads_ok = lb.reads_ok;
		s.reads_failed = lb.reads_failed;
		n = tel_bin_summary(&s, rec, NET_PAYLOAD_MAX);
		publish((UB)TOPIC_EVENT, rec, n);
		tm_printf((UB*)"[TEL] RUN SUMMARY: %u ms, %d mm, highest hump %d mm\n",
			  s.time_ms, s.dist_mm, s.hmax_mm);
	}
}

/* Every TIM_PERIOD_MS: the timing and queue-health measurements */
LOCAL void timing_message(UW period_ms)
{
	static UW	last_irq = 0U;
	TEL_TIMING	tm;
	APP_TIMING	at;
	MC_SNAPSHOT	snap = {0};
	VC_STATUS	vc;
	UB		rec[NET_PAYLOAD_MAX];
	UW		irq = gpio_irq_calls;
	UW		i, ov = 0U;

	app_timing_get(&at);
	(void)motion_get_snapshot(&snap);
	vc_get_status(&vc);
	tm.t_ms = app_ms();
	for (i = 0U; i < TEL_TIM_TASKS; i++) {
		tm.max_us[i] = at.max_us[i];
		ov += at.overruns[i];
	}
	tm.overruns = ov;
	tm.irq_per_s = ((irq - last_irq) * 1000U) / period_ms;
	last_irq = irq;
	tm.irq_max_us = gpio_irq_max_us;
	tm.vq_high = vc.vq_high;
	tm.dropped = app_dropped();
	tm.steer_lat_max_ms = snap.steer_lat_max_ms;
	tm.stop_lat_max_us = snap.stop_lat_max_us;
	tm.stop_dist_mm = snap.stop_dist_mm;
	tm.reconnect_ms = net_reconnect_ms;
	publish((UB)TOPIC_EVENT, rec, tel_bin_timing(&tm, rec, NET_PAYLOAD_MAX));
}

EXPORT void telemetry_task(INT stacd, void *exinf)
{
	UB		rx[NET_CMD_MAX];
	UB		rec[NET_PAYLOAD_MAX];
	APP_MSG		m;		/* packed, as it comes out of telemetry_q */
	APP_EVT		e;
	TEL_DATA	d;
	UW		last_tel = 0U, last_hb = 0U, last_tim = 0U, now;
	INT		len;
	W		n;

	(void)stacd;
	(void)exinf;
	while (1) {
		/* 1. Commands: wait up to 100 ms */
		UW	t0;

		len = tk_rcv_mbf(mqtt_rx_mbf, rx, TEL_RX_TMO_MS);
		t0 = app_us();
		if (len > 0) {
			handle_command(rx, len);
		}

		/* 2. Records from the other tasks, without waiting */
		while (tk_rcv_mbf(telemetry_q, &m, TMO_POL) == (INT)sizeof(m)) {
			app_msg_unpack(&m, &e);
			take_record(&e);
		}

		/* 3. Periodic messages */
		now = app_ms();
		if ((now - last_tel) >= TEL_PERIOD_MS) {
			last_tel = now;
			gather(&d);
			n = tel_bin_telemetry(&d, rec, NET_PAYLOAD_MAX);
			publish((UB)TOPIC_TELEMETRY, rec, n);
		}
		if ((now - last_hb) >= HB_PERIOD_MS) {
			last_hb = now;
			gather(&d);
			n = tel_bin_status(&d, rec, NET_PAYLOAD_MAX);
			publish((UB)TOPIC_STATUS, rec, n);
		}
		if ((now - last_tim) >= TIM_PERIOD_MS) {
			timing_message(now - last_tim);
			last_tim = now;
		}
		app_task_time(TIM_TELEMETRY, t0);
	}
}
