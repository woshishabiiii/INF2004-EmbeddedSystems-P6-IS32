/*
 *	app_evt.c
 *	The event list used by the logic modules (see app_evt.h).  Pure C.
 */

#include "app_evt.h"

void evt_list_clear(EVT_LIST *l)
{
	l->n = 0U;
}

bool evt_list_add(EVT_LIST *l, uint8_t type, int32_t p1, int32_t p2)
{
	APP_MSG *m;

	if (l->n >= EVT_LIST_MAX) {
		return false;
	}
	m = &l->ev[l->n];
	m->type = type;
	m->source = 0U;		/* filled in by the sending task */
	m->seq = 0U;
	m->t_ms = 0U;
	m->p1 = p1;
	m->p2 = p2;
	l->n++;
	return true;
}
