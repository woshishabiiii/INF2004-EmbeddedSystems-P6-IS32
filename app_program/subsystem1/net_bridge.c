/*
 *	net_bridge.c
 *	The application side of the MQTT bridge.  These functions are CALLED
 *	BY lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c, so they run inside
 *	the kernel's cyw43_utk_task -- the only task allowed to touch the
 *	network.  They only move bytes in and out of two message buffers,
 *	always with TMO_POL, so the network task never waits for us.
 *
 *	Their signatures use plain C types on purpose: mqtt_utk.c must not
 *	include tk/tkernel.h (see the comment at the top of that file).
 *	In a build without MQTT=1 nothing calls them; that is harmless.
 */

#include "subsystem1.h"
#include "net_bridge.h"
#include "../subsystem4/subsystem4.h"	/* fault_flags: FLT_WIFI_DOWN */

/* Topic prefix: replace g07 with your group number */
#define TOPIC_PREFIX	"car/g32/"

LOCAL const char * const s_topic[TOPIC_EVENT + 1U] = {
	TOPIC_PREFIX "status",
	TOPIC_PREFIX "telemetry",
	TOPIC_PREFIX "event",
};

EXPORT volatile UB	net_rx_drops;
EXPORT volatile UB	net_errors;
EXPORT volatile UB	net_connects;
EXPORT volatile UW	net_reconnect_ms;	/* NFR: reconnect time, measured */
LOCAL UW		s_down_ms;		/* when the connection was lost, 0 = up */

/* An 8-bit counter that stops at 255 (one writer: the network task) */
LOCAL void count8(volatile UB *c)
{
	if (*c < 0xFFU) {
		*c = (UB)(*c + 1U);
	}
}

/* Is the link up now?  The FLT_WIFI_DOWN bit, not a separate variable */
LOCAL BOOL link_is_up(void)
{
	T_RFLG rflg;

	return ((tk_ref_flg(fault_flags, &rflg) == E_OK) && ((rflg.flgptn & FLT_WIFI_DOWN) == 0U)) ? TRUE : FALSE;
}

EXPORT const char *net_topic_name(UINT topic)
{
	return (topic <= TOPIC_EVENT) ? s_topic[topic] : "?";
}

/* Next message to publish.  Returns the payload length, 0 if none. */
int net_bridge_take(const char **topic, unsigned char *payload, int max)
{
	UB	buf[NET_PAYLOAD_MAX + 1U];
	INT	len;
	INT	i;

	if ((mqtt_tx_mbf <= 0) || (max < (int)NET_PAYLOAD_MAX)) {
		return 0;
	}
	len = tk_rcv_mbf(mqtt_tx_mbf, buf, TMO_POL);
	if ((len < 2) || (buf[0] > TOPIC_EVENT)) {
		return 0;
	}
	*topic = s_topic[buf[0]];
	for (i = 1; i < len; i++) {
		payload[i - 1] = buf[i];
	}
	return (int)(len - 1);
}

/* A message arrived on the command topic: queue it for Telemetry */
void net_bridge_put(const unsigned char *data, int len)
{
	if ((mqtt_rx_mbf <= 0) || (len <= 0) || (len > (int)NET_CMD_MAX) ||
	    (tk_snd_mbf(mqtt_rx_mbf, data, (INT)len, TMO_POL) != E_OK)) {
		count8(&net_rx_drops);
	}
}

/* The command arrived but was too long, or a publish failed */
void net_bridge_dropped(void)
{
	count8(&net_errors);
}

/* MQTT connection state changes.  Also measures the reconnect time: from
   the moment the connection was lost to the moment it is back (WiFi
   re-join every 5 s and broker retry every 3 s in the kernel's code). */
void net_bridge_link(int connected)
{
	UW now = app_ms() | 1U;		/* 0 means "not lost" */

	if (connected != 0) {
		count8(&net_connects);
		(void)tk_clr_flg(fault_flags, ~(UINT)FLT_WIFI_DOWN);	/* status bit: never latches */
		if (s_down_ms != 0U) {
			net_reconnect_ms = now - s_down_ms;
			s_down_ms = 0U;
		}
	} else {
		if (link_is_up()) {
			s_down_ms = now;		/* it was up: the loss starts now */
		}
		(void)tk_set_flg(fault_flags, FLT_WIFI_DOWN);	/* reported, not latched */
	}
}

const char *net_bridge_cmd_topic(void)
{
	return TOPIC_PREFIX "cmd";
}
