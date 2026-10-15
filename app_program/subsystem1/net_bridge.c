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

/* Topic prefix: replace g07 with your group number */
#define TOPIC_PREFIX	"car/g07/"

LOCAL const char * const s_topic[3] = {
	TOPIC_PREFIX "status",
	TOPIC_PREFIX "telemetry",
	TOPIC_PREFIX "event",
};

EXPORT volatile UW	net_rx_drops;
EXPORT volatile UW	net_errors;
EXPORT volatile UW	net_connects;
EXPORT volatile BOOL	net_up;

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
		net_rx_drops++;
	}
}

/* The command arrived but was too long, or a publish failed */
void net_bridge_dropped(void)
{
	net_errors++;
}

/* MQTT connection state changes */
void net_bridge_link(int connected)
{
	if (connected != 0) {
		net_connects++;
		net_up = TRUE;
	} else {
		net_up = FALSE;
	}
}

const char *net_bridge_cmd_topic(void)
{
	return TOPIC_PREFIX "cmd";
}
