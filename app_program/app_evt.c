/*
 *	app_evt.c
 *	The event list used by the logic modules, and the packed 8-byte
 *	event record (see app_evt.h).  Pure C.
 */

#include "app_evt.h"
#include "bitpack.h"

BP_STATIC_ASSERT(sizeof(APP_MSG) == 8U, app_msg_is_8_bytes);

#define HDR_TYPE_BITS	5U
#define HDR_SRC_BITS	3U
#define HDR_SEQ_BITS	8U
#define HDR_SRC_POS	5U
#define HDR_SEQ_POS	8U
#define HDR_T_POS	16U

/* How p1 and p2 are stored in the 32-bit payload: width and offset
   (stored = value - min).  raw = a bit mask: keep the low bits as they
   are instead of saturating.  Widths come from each value's valid range
   (app_evt.h lists them next to the event types). */
typedef struct {
	uint8_t		n1;
	int32_t		min1;
	uint8_t		n2;
	int32_t		min2;
	bool		raw1;
} EVT_LAYOUT;

static const EVT_LAYOUT s_layout[EV_TYPE_MAX + 1U] = {
	{ 0U,  0,	 0U, 0,			false},	/* 0: unused */
	{ 1U,  0,	 1U, 0,			false},	/* EV_LINE_STATUS */
	{ 2U, -1,	 0U, 0,			false},	/* EV_LINE_LOST */
	{24U,  0,	 0U, 0,			false},	/* EV_JUNCTION */
	{ 6U, 32,	 3U, 0,			false},	/* EV_BARCODE: ASCII 32..95 */
	{ 7U,  0,	 6U, 0,			false},	/* EV_BARCODE_BAD */
	{ 2U,  0,	 8U, 0,			false},	/* EV_DONE */
	{17U,  0,	10U, 0,			false},	/* EV_STOP_ACK */
	{10U,  0,	 0U, 0,			true},	/* EV_FAULT: FLT_ALL bit mask */
	{ 0U,  0,	 0U, 0,			false},	/* EV_FAULT_CLEAR */
	{12U,  0,	 8U, 0,			false},	/* EV_ALERT */
	{ 1U,  0,	 8U, 0,			false},	/* EV_RANGE_ZONE */
	{ 1U,  0,	 8U, 0,			false},	/* EV_PROFILE */
	{ 2U,  0,	14U, -8192,		false},	/* EV_TERRAIN */
	{ 8U,  0,	 8U, 0,			false},	/* EV_HUMP */
	{ 3U,  0,	 0U, 0,			false},	/* EV_MOTION */
	{12U,  0,	 9U, 0,			false},	/* EV_IMPACT: speed 0..511 (snapshot speed is +-511) */
	{ 1U, BTN_START, 0U, 0,		false},	/* EV_BUTTON: 0 = START (GP20), 1 = STOP (GP21) */
	{ 3U,  0,	 0U, 0,			false},	/* EV_COMMAND */
	{ 4U,  0,	 4U, 0,			false},	/* EV_STATE */
	{ 4U,  0,	25U, -16777216,		false},	/* EV_NAV: the widest value is a run distance (24 bits) */
	{ 0U,  0,	 0U, 0,			false},	/* EV_PROFILE_DATA */
};

static const EVT_LAYOUT *layout_of(uint8_t type)
{
	return &s_layout[(type <= EV_TYPE_MAX) ? type : 0U];
}

uint32_t app_evt_bits(uint8_t type, uint32_t which)
{
	const EVT_LAYOUT *l = layout_of(type);

	return (which == 1U) ? l->n1 : l->n2;
}

int32_t app_evt_min(uint8_t type, uint32_t which)
{
	const EVT_LAYOUT *l = layout_of(type);

	return (which == 1U) ? l->min1 : l->min2;
}

/* One field into the payload: saturated (or masked) and offset */
static uint32_t field_in(int32_t v, uint8_t n, int32_t min, bool raw)
{
	if (n == 0U) {
		return 0U;
	}
	if (raw) {
		return (uint32_t)v & BP_MAX(n);
	}
	return bp_sat_s(v, min, n);
}

static int32_t field_out(uint32_t f, uint8_t n, int32_t min)
{
	return (n == 0U) ? 0 : bp_from_s(f & BP_MAX(n), min);
}

void app_msg_pack(const APP_EVT *e, APP_MSG *m)
{
	const EVT_LAYOUT *l = layout_of(e->type);

	m->hdr = ((uint32_t)e->type & BP_MAX(HDR_TYPE_BITS)) |
		 (((uint32_t)e->source & BP_MAX(HDR_SRC_BITS)) << HDR_SRC_POS) |
		 ((uint32_t)e->seq << HDR_SEQ_POS) |
		 ((uint32_t)e->t_ms << HDR_T_POS);
	m->pl = field_in(e->p1, l->n1, l->min1, l->raw1);
	if (l->n2 != 0U) {
		m->pl |= field_in(e->p2, l->n2, l->min2, false) << l->n1;
	}
}

void app_msg_unpack(const APP_MSG *m, APP_EVT *e)
{
	const EVT_LAYOUT *l;

	e->type = (uint8_t)(m->hdr & BP_MAX(HDR_TYPE_BITS));
	e->source = (uint8_t)((m->hdr >> HDR_SRC_POS) & BP_MAX(HDR_SRC_BITS));
	e->seq = (uint8_t)((m->hdr >> HDR_SEQ_POS) & BP_MAX(HDR_SEQ_BITS));
	e->t_ms = (uint16_t)(m->hdr >> HDR_T_POS);
	l = layout_of(e->type);
	e->p1 = field_out(m->pl, l->n1, l->min1);
	e->p2 = (l->n2 == 0U) ? 0 : field_out(m->pl >> l->n1, l->n2, l->min2);
}

void evt_list_clear(EVT_LIST *l)
{
	l->n = 0U;
}

bool evt_list_add(EVT_LIST *l, uint8_t type, int32_t p1, int32_t p2)
{
	APP_EVT *m;

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

bool evt_is_safety(uint8_t type)
{
	switch (type) {
	case EV_FAULT:
	case EV_FAULT_CLEAR:
	case EV_IMPACT:
	case EV_ALERT:
	case EV_STOP_ACK:
	case EV_BUTTON:
	case EV_COMMAND:
		return true;
	default:
		return false;	/* line status, junctions, barcodes, DONE, profiles,
				   terrain, humps, ...: drop and count */
	}
}
