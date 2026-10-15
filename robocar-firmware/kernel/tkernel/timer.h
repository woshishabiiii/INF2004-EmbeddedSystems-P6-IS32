/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.00.00
 *
 *    Copyright (C) 2006-2019 by Ken Sakamura.
 *    This software is distributed under the T-License 2.1.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2019/12/11.
 *
 *----------------------------------------------------------------------
 */

/*
 *	timer.h
 *	System Timer Module Definition
 */

#ifndef _TIMER_
#define _TIMER_

#include "longlong.h"

/*
 * SYSTIM internal expression and conversion
 */
typedef	D	LSYSTIM;	/* SYSTIM int. expression */

Inline LSYSTIM knl_toLSYSTIM( CONST SYSTIM *time )
{
	LSYSTIM		ltime;

	hilo_ll(ltime, time->hi, time->lo);

	return ltime;
}

Inline SYSTIM knl_toSYSTIM( LSYSTIM ltime )
{
	SYSTIM		time;

	ll_hilo(time.hi, time.lo, ltime);

	return time;
}

/*
 * Absolute time (can be considered the lower 32bit of SYSTIM)
 */
typedef	UW	ABSTIM;

#define ABSTIM_DIFF_MIN  (0x7FFFFFFF)

Inline BOOL knl_abstim_reached( ABSTIM curtim, ABSTIM evttim )
{
	return (ABSTIM)(curtim - evttim) <= (ABSTIM)ABSTIM_DIFF_MIN;
}

/*
 * Definition of timer event block 
 */
typedef void	(*CBACK)(void *);	/* Type of callback function */

typedef struct timer_event_block {
	QUEUE	queue;		/* Timer event queue */
	ABSTIM	time;		/* Event time */
	CBACK	callback;	/* Callback function */
	void	*arg;		/* Argument to be sent to callback function */
} TMEB;

/*
 * Current time (Software clock)
 */
IMPORT LSYSTIM	knl_current_time;	/* System operation time */
IMPORT LSYSTIM	knl_real_time_ofs;	/* Difference from actual time */

/*
 * Time-event queue
 */
IMPORT QUEUE	knl_timer_queue;

#if TK_SUPPORT_SMP
/* Phase-9 time-authority and callback-routing diagnostics. */
typedef struct {
	UW ticks;
	UW wrong_core_ticks;
	UW expired_events;
	UW max_expired_per_tick;
	UW handler_enqueued[TK_MAX_CORE];
	UW handler_executed[TK_MAX_CORE];
	UW callback_max_us[TK_MAX_CORE];
	UW callback_detached[TK_MAX_CORE];
	UW callback_reconciled[TK_MAX_CORE];
	UW callback_stale[TK_MAX_CORE];
} KNL_TIMER_DIAG;

IMPORT KNL_TIMER_DIAG knl_timer_diag;
IMPORT void knl_timehandler_initialize( void );
IMPORT void knl_timehandler_ipi( void );
IMPORT void knl_timer_get_diag( KNL_TIMER_DIAG *diag );
#endif

/*
 * Register time-event onto timer queue
 */
IMPORT void knl_timer_insert( TMEB *evt, TMO tmout, CBACK cback, void *arg );
IMPORT void knl_timer_insert_reltim( TMEB *event, RELTIM tmout, CBACK callback, void *arg );
IMPORT void knl_timer_insert_abs( TMEB *evt, ABSTIM time, CBACK cback, void *arg );

/*
 * Delete from time-event queue
 */
Inline void knl_timer_delete( TMEB *event )
{
	QueRemove(&event->queue);
}

#endif /* _TIMER_ */
