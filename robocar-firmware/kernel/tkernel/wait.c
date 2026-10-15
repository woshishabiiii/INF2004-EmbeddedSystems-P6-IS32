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
 *	wait.c
 *	Common Routine for Synchronization
 */

#include "kernel.h"
#include "wait.h"

/*
 * Releasing a task publishes it to a processor.
 *
 * 'wercd' points into the waiting task's own stack frame -- at the local the
 * blocking system call will return -- and that local already holds E_TMOUT,
 * written before the wait began. The official kernel therefore writes the
 * real result after knl_wait_release(), which is safe on one processor
 * because the released task cannot run until the releasing core dispatches.
 *
 * On two processors the release is the handover. knl_make_ready() publishes
 * the task to the other core's schedtsk slot, and that core -- idling in the
 * dispatcher, polling the slot -- can restore the context and run off the end
 * of the waiting call before this core has walked back up three frames to
 * write the error code. The task then returns the value it went to sleep
 * with: E_TMOUT, reported by a wakeup that in fact arrived on time.
 *
 * Nothing else the releaser writes is exposed this way. Every caller sets the
 * task's output values (winfo.flg.p_flgptn, winfo.mpf.p_blf, the rendezvous
 * fields) before releasing, and everything it touches afterwards is control
 * block state that the woken task can only reach through another system call,
 * which blocks on the kernel lock this core still holds. The error code is
 * the one value read with no lock at all, so it is the one that must be
 * written before the task becomes runnable.
 */

#if TK_SUPPORT_SMP
/*
 * A lower bound on how often the released task was already executing
 * elsewhere by the time the release returned -- that is, how often the old
 * order would have lost the error code.  A task can run and block again before
 * this sample, so zero is not evidence that no cross-core handoff happened.
 *
 * Kept after the fix rather than removed with it. It costs one comparison per
 * wakeup, it is the only direct evidence that the window is reachable on this
 * hardware rather than merely arguable, and a future reordering that reopens
 * it would otherwise fail silently again.
 */
EXPORT UW knl_smp_wake_raced;

LOCAL void knl_note_wake_race( TCB *tcb )
{
	UINT	core, self = knl_current_core();

	for ( core = 0; core < TK_MAX_CORE; core++ ) {
		if ( core == self ) continue;
		if ( *knl_ctxtsk_core_slot(core) == tcb ) {
			/* Already restored and running: with the old order,
			   this wakeup's error code arrived too late. */
			knl_smp_wake_raced++;
			return;
		}
	}
}
#else
#define knl_note_wake_race(tcb)		((void)0)
#endif

EXPORT void knl_wait_release_ok( TCB *tcb )
{
	*tcb->wercd = E_OK;
	knl_wait_release(tcb);
	knl_note_wake_race(tcb);
}

EXPORT void knl_wait_release_ok_ercd( TCB *tcb, ER ercd )
{
	*tcb->wercd = ercd;
	knl_wait_release(tcb);
	knl_note_wake_race(tcb);
}

EXPORT void knl_wait_release_ng( TCB *tcb, ER ercd )
{
	*tcb->wercd = ercd;
	knl_wait_release(tcb);
	/*
	 * The hook stays after the release: mtx_rel_wai() recomputes the lock
	 * holder's inherited priority from the head of the wait queue, and the
	 * task being released must already be off that queue when it does.
	 * It touches only mutex control block state, which is covered by the
	 * kernel lock this core holds.
	 */
	if ( tcb->wspec->rel_wai_hook != NULL ) {
		(*tcb->wspec->rel_wai_hook)(tcb);
	}
	knl_note_wake_race(tcb);
}

EXPORT void knl_wait_release_tmout( TCB *tcb )
{
	QueRemove(&tcb->tskque);
	knl_make_non_wait(tcb);
	if ( tcb->wspec->rel_wai_hook != NULL ) {
		(*tcb->wspec->rel_wai_hook)(tcb);
	}
}

/*
 * Change the active task state to wait state and connect to the
 * timer event queue.
 *	Normally, 'knl_ctxtsk' is in the RUN state, but when an interrupt
 *	occurs during executing system call, 'knl_ctxtsk' may become the
 *	other state by system call called in the interrupt handler.
 *	However, it does not be in WAIT state.
 *
 *	"include/tk/typedef.h"
 *	typedef	W		TMO;
 *	typedef UW		RELTIM;
 *	#define TMO_FEVR	(-1)
 */
EXPORT void knl_make_wait( TMO tmout, ATR atr )
{
	switch ( knl_ctxtsk->state ) {
	  case TS_READY:
		knl_make_non_ready(knl_ctxtsk);
		knl_ctxtsk->state = TS_WAIT;
		break;
	  case TS_SUSPEND:
		knl_ctxtsk->state = TS_WAITSUS;
		break;
	}
	knl_timer_insert(&knl_ctxtsk->wtmeb, tmout, (CBACK)knl_wait_release_tmout, knl_ctxtsk);
}

EXPORT void knl_make_wait_reltim( RELTIM tmout, ATR atr )
{
	switch ( knl_ctxtsk->state ) {
	  case TS_READY:
		knl_make_non_ready(knl_ctxtsk);
		knl_ctxtsk->state = TS_WAIT;
		break;
	  case TS_SUSPEND:
		knl_ctxtsk->state = TS_WAITSUS;
		break;
	}
	knl_timer_insert_reltim(&knl_ctxtsk->wtmeb, tmout, (CBACK)knl_wait_release_tmout, knl_ctxtsk);
}

/*
 * Release all tasks connected to the wait queue, and define it
 * as E_DLT error.
 */
EXPORT void knl_wait_delete( QUEUE *wait_queue )
{
	TCB	*tcb;

	while ( !isQueEmpty(wait_queue) ) {
		tcb = (TCB*)wait_queue->next;
		*tcb->wercd = E_DLT;	/* before the release; see above */
		knl_wait_release(tcb);
		knl_note_wake_race(tcb);
	}
}

/*
 * Get ID of the head task in the wait queue.
 */
EXPORT ID knl_wait_tskid( QUEUE *wait_queue )
{
	if ( isQueEmpty(wait_queue) ) {
		return 0;
	}

	return ((TCB*)wait_queue->next)->tskid;
}

/*
 * Change the active task state to wait state and connect to the timer wait 
 * queue and the object wait queue. Also set 'wid' in 'knl_ctxtsk'.
 */
EXPORT void knl_gcb_make_wait( GCB *gcb, TMO tmout )
{
	*knl_ctxtsk->wercd = E_TMOUT;
	if ( tmout != TMO_POL ) {
		knl_ctxtsk->wid = gcb->objid;
		knl_make_wait(tmout, gcb->objatr);
		if ( (gcb->objatr & TA_TPRI) != 0 ) {
			knl_queue_insert_tpri(knl_ctxtsk, &gcb->wait_queue);
		} else {
			QueInsert(&knl_ctxtsk->tskque, &gcb->wait_queue);
		}
	}
}

/*
 * When the task priority changes, adjust the task position at the wait queue.
 * It is called only if the object attribute TA_TPRI is specified.
 *
 */
EXPORT void knl_gcb_change_priority( GCB *gcb, TCB *tcb )
{
	QueRemove(&tcb->tskque);
	knl_queue_insert_tpri(tcb, &gcb->wait_queue);
}

/*
 * Search the first task of wait queue include "tcb" with target.
 * (Not insert "tcb" into wait queue.)
 *
 */
EXPORT TCB* knl_gcb_top_of_wait_queue( GCB *gcb, TCB *tcb )
{
	TCB	*q;

	if ( isQueEmpty(&gcb->wait_queue) ) {
		return tcb;
	}

	q = (TCB*)gcb->wait_queue.next;
	if ( (gcb->objatr & TA_TPRI) == 0 ) {
		return q;
	}

	return ( tcb->priority < q->priority )? tcb: q;
}
