/*
 *	app_msg.h
 *	Kernel side of the shared event format (team agreement, like app.h).
 *	The record itself and the event numbers are in app_evt.h, which has
 *	no kernel headers so the logic modules can include it too.
 */

#ifndef APP_MSG_H
#define APP_MSG_H

#include "app.h"
#include "app_evt.h"

/* Bytes of message-buffer space for n messages of size sz: the kernel
   stores a 4-byte header with each message. */
#define MBF_BYTES(n, sz)	((n) * ((sz) + 4U))

/* Send one event to a message buffer WITHOUT waiting (TMO_POL).
   Fills in source, sequence number (*seq, then incremented) and time.
   Returns E_OK, or the kernel error (E_TMOUT when the buffer is full). */
IMPORT ER app_send(ID mbf, UB source, UH *seq, UB type, W p1, W p2);

/* Same, for every event in a list made by a logic module.  Returns E_OK
   only if every send succeeded. */
IMPORT ER app_send_list(ID mbf, UB source, UH *seq, const EVT_LIST *l);

#endif /* APP_MSG_H */
