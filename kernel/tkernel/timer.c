/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.00.06A
 *
 *    Copyright (C) 2006-2023 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2023/03.
 *
 *----------------------------------------------------------------------
 */

/*
 *	timer.c
 *	Timer Control
 */

#include "kernel.h"
#include "timer.h"
#include "../sysdepend/sys_timer.h"

/*
 * Current time (Software clock)
 *	'current_time' shows the total operation time since
 *	operating system Starts. 'real_time_ofs' shows difference
 *	between the current time and the operating system clock
 *	(current_time). Do not change 'current_time' when setting
 *	time by 'set_tim()'. Set 'real_time_ofs' with the time  	 
 *   	difference between 'current_time' and setup time.
 *	Therefore 'current_time' does not affect with time change
 *	and it increases simply.
 */
Noinit(EXPORT LSYSTIM	knl_current_time);	/* System operation time */
Noinit(EXPORT LSYSTIM	knl_real_time_ofs);	/* Actual time - System operation time */

/* 
 * Timer event queue
 */
Noinit(EXPORT QUEUE	knl_timer_queue);

#if TK_SUPPORT_SMP
Noinit(EXPORT KNL_TIMER_DIAG knl_timer_diag);
#endif

/*
 * Start system timer
 */
EXPORT ER knl_timer_startup( void )
{
	knl_current_time = knl_real_time_ofs = uitoll(0);
	QueInit(&knl_timer_queue);
#if TK_SUPPORT_SMP
	knl_timer_diag.ticks = 0;
	knl_timer_diag.wrong_core_ticks = 0;
	knl_timer_diag.expired_events = 0;
	knl_timer_diag.max_expired_per_tick = 0;
	for ( UINT core = 0; core < TK_MAX_CORE; core++ ) {
		knl_timer_diag.handler_enqueued[core] = 0;
		knl_timer_diag.handler_executed[core] = 0;
		knl_timer_diag.callback_max_us[core] = 0;
		knl_timer_diag.callback_detached[core] = 0;
		knl_timer_diag.callback_reconciled[core] = 0;
		knl_timer_diag.callback_stale[core] = 0;
	}
	knl_timehandler_initialize();
#endif

	/* Start timer interrupt */
	knl_start_hw_timer();

	return E_OK;
}

#if USE_SHUTDOWN
/*
 * Stop system timer
 */
EXPORT void knl_timer_shutdown( void )
{
	knl_terminate_hw_timer();
}
#endif /* USE_SHUTDOWN */


/*
 * Insert timer event to timer event queue
 */
LOCAL void knl_enqueue_tmeb( TMEB *event )
{
	QUEUE	*q;
	ABSTIM	ofs = lltoul(knl_current_time) - ABSTIM_DIFF_MIN;

	for ( q = knl_timer_queue.next; q != &knl_timer_queue; q = q->next ) {
		if ( (ABSTIM)(event->time - ofs) < (ABSTIM)((((TMEB*)q)->time) - ofs) ) {
			break;
		}
	}
	QueInsert(&event->queue, q);
}

/*
 * Set timeout event
 *	Register the timer event 'event' onto the timer queue to
 *	start after the timeout 'tmout'. At timeout, start with the
 *	argument 'arg' on the callback function 'callback'.
 *	When 'tmout' is TMO_FEVR, do not register onto the timer
 *	queue, but initialize queue area in case 'timer_delete' 
 *	is called later.
 *
 *	"include/tk/typedef.h"
 *	typedef	W		TMO;
 *	typedef UW		RELTIM;
 *	#define TMO_FEVR	(-1)
 */
EXPORT void knl_timer_insert( TMEB *event, TMO tmout, CBACK callback, void *arg )
{
	event->callback = callback;
	event->arg = arg;

	if ( tmout == TMO_FEVR ) {
		QueInit(&event->queue);
	} else {
		/* To guarantee longer wait time specified by 'tmout',
		   add TIMER_PERIOD on wait time */
		event->time = lltoul(knl_current_time) + tmout + TIMER_PERIOD;
		knl_enqueue_tmeb(event);
	}
}

EXPORT void knl_timer_insert_reltim( TMEB *event, RELTIM tmout, CBACK callback, void *arg )
{
	event->callback = callback;
	event->arg = arg;

	/* To guarantee longer wait time specified by 'tmout',
	   add TIMER_PERIOD on wait time */
	event->time = lltoul(knl_current_time) + tmout + TIMER_PERIOD;
	knl_enqueue_tmeb(event);
}

/*
 * Set time specified event
 *	Register the timer event 'evt' onto the timer queue to start at the 
 *	(absolute) time 'time'.
 *	'time' is not an actual time. It is system operation time.
 */
EXPORT void knl_timer_insert_abs( TMEB *evt, ABSTIM time, CBACK cback, void *arg )
{
	evt->callback = cback;
	evt->arg = arg;
	evt->time = time;
	knl_enqueue_tmeb(evt);
}

/* ------------------------------------------------------------------------ */

/*
 * System timer interrupt handler
 *	This interrupt handler starts every TIMER_PERIOD millisecond 
 *	interval by hardware timer. Update the software clock and start the 
 *	timer event upon arriving at start time.
 */

EXPORT void knl_timer_handler( void )
{
	TMEB	*event;
	ABSTIM	cur;
#if TK_SUPPORT_SMP
	UW	expired = 0;
#endif

	knl_clear_hw_timer_interrupt();		/* Clear timer interrupt */

#if TK_SUPPORT_SMP
	/* SysTick is per-core on RP2040.  Only core 0 owns the global clock;
	   reject any accidentally enabled core-1 tick instead of double-counting. */
	if ( knl_current_core() != 0 ) {
		knl_timer_diag.wrong_core_ticks++;
		knl_end_of_hw_timer_interrupt();
		return;
	}
#endif

	BEGIN_CRITICAL_SECTION;
	knl_current_time = ll_add(knl_current_time, uitoll(TIMER_PERIOD));
	cur = lltoul(knl_current_time);
#if TK_SUPPORT_SMP
	knl_timer_diag.ticks++;
#endif

#if USE_DBGSPT && defined(USE_FUNC_TD_INF_TSK)
#if TK_SUPPORT_SMP
	/* One authoritative tick accounts for every processor's running task. */
	for ( UINT core = 0; core < TK_MAX_CORE; core++ ) {
		TCB *ctxtsk = *knl_ctxtsk_core_slot(core);
		if ( ctxtsk != NULL ) {
			if ( ctxtsk->sysmode > 0 ) ctxtsk->stime += TIMER_PERIOD;
			else ctxtsk->utime += TIMER_PERIOD;
		}
	}
#else
	if ( knl_ctxtsk != NULL ) {
		/* Task at execution */
		if ( knl_ctxtsk->sysmode > 0 ) {
			knl_ctxtsk->stime += TIMER_PERIOD;
		} else {
			knl_ctxtsk->utime += TIMER_PERIOD;
		}
	}
#endif
#endif

	/* Execute event that passed occurring time. */
	while ( !isQueEmpty(&knl_timer_queue) ) {
		event = (TMEB*)knl_timer_queue.next;

		if ( !knl_abstim_reached(cur, event->time) ) {
			break;
		}

		QueRemove(&event->queue);
#if TK_SUPPORT_SMP
		expired++;
#endif
		if ( event->callback != NULL ) {
			(*event->callback)(event->arg);
		}
	}

#if TK_SUPPORT_SMP
	knl_timer_diag.expired_events += expired;
	if ( expired > knl_timer_diag.max_expired_per_tick ) {
		knl_timer_diag.max_expired_per_tick = expired;
	}
#endif

	END_CRITICAL_SECTION;

	knl_end_of_hw_timer_interrupt();		/* Clear timer interrupt */
}

#if TK_SUPPORT_SMP
EXPORT void knl_timer_get_diag( KNL_TIMER_DIAG *diag )
{
	if ( diag == NULL ) return;
	BEGIN_CRITICAL_SECTION;
	*diag = knl_timer_diag;
	END_CRITICAL_SECTION;
}
#endif
