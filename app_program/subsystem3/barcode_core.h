/*
 *	barcode_core.h
 *	Barcode capture (Region B of Figure 5, Buddy 3).  Pure C.
 *
 *	The GPIO interrupt timestamps every edge of the barcode sensor (1 us).
 *	The Line and Barcode task feeds those edges in here with bc_edge() and
 *	calls bc_poll() every 5 ms.  A capture:
 *	  - starts at the first edge INTO a bar;
 *	  - drops an edge closer than t_min to the previous one, together
 *	    with that previous edge (a comparator "chatter" pair), and an
 *	    edge that leaves the level unchanged (two merged transitions);
 *	  - ends when the sensor has been quiet for T_end = max(t_end_min,
 *	    t_end_factor x the widest element so far);
 *	  - is then decoded by code39_decode(), forwards or backwards.
 */

#ifndef BARCODE_CORE_H
#define BARCODE_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "code39.h"

#define BC_MAX_EDGES	40U	/* a Code 39 "*X*" has 30; more = not a barcode */

/* EV_BARCODE_BAD.p1 when edges were lost because the interrupt's ring
   buffer overflowed: that capture is abandoned (a mission-level failure,
   not a fault -- the car carries on; design-review feedback IS32) */
#define BC_REASON_OVERFLOW	100

/* Navigation commands (EV_BARCODE.p2) */
#define LB_CMD_NONE		0	/* valid barcode, but no command (e.g. 'Z') */
#define LB_CMD_TURN_LEFT	1	/* 'A' */
#define LB_CMD_TURN_RIGHT	2	/* 'B' */
#define LB_CMD_STRAIGHT		3	/* 'C' */
#define LB_CMD_U_TURN		4	/* 'D' */

typedef struct {
	uint32_t	t_min_us;	/* shorter elements are chatter */
	uint32_t	t_end_min_us;	/* shortest quiet time that ends a capture */
	uint32_t	t_end_factor;	/* ... or this many times the widest element */
	uint8_t		bar_level;	/* sensor DO level over a black bar */
} BC_PARAMS;

typedef struct {
	c39_result_t	result;		/* C39_OK or why it failed */
	char		c;		/* decoded character (C39_OK only) */
	bool		reversed;	/* read back to front */
	uint32_t	elements;	/* widths captured (29 for a good read) */
	uint32_t	narrow_us;	/* narrowest element, for speed checks */
} BC_RESULT;

typedef struct {
	BC_PARAMS	p;
	uint32_t	t[BC_MAX_EDGES];	/* edge times of this capture */
	uint32_t	n;		/* edges stored */
	uint8_t		last_level;
	uint32_t	widest_us;
	bool		capturing;
	bool		broken;		/* too many edges */
	uint32_t	ok;		/* statistics for telemetry */
	uint32_t	failed;
} BC_CORE;

void	bc_init(BC_CORE *b, const BC_PARAMS *p);

/* One edge: time (us) and the sensor level just after it */
void	bc_edge(BC_CORE *b, uint32_t t_us, uint8_t level);

/* Call regularly.  Returns true when a capture has just ended; *res says
   what it was.  Captures of one or two elements (for example the sensor
   crossing a junction line) are ignored silently. */
bool	bc_poll(BC_CORE *b, uint32_t now_us, BC_RESULT *res);

/* Abandon any capture in progress */
void	bc_reset(BC_CORE *b);

/* The command table: character -> LB_CMD_... */
int32_t	bc_command_for(char c);

#endif /* BARCODE_CORE_H */
