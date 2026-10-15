/*
 *	net_bridge.h
 *	The functions mqtt_utk.c calls (plain C types only: that file must
 *	not include the kernel headers, so it repeats these declarations).
 */

#ifndef NET_BRIDGE_H
#define NET_BRIDGE_H

int		net_bridge_take(const char **topic, unsigned char *payload, int max);
void		net_bridge_put(const unsigned char *data, int len);
void		net_bridge_dropped(void);
void		net_bridge_link(int connected);
const char	*net_bridge_cmd_topic(void);

#endif /* NET_BRIDGE_H */
