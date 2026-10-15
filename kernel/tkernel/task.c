/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.00.01
 *
 *    Copyright (C) 2006-2020 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2020/05/29.
 *
 *----------------------------------------------------------------------
 */

/*
 *	task.c
 *	Task Control
 */

#define KNL_SMP_NO_SHORT_NAMES
#include "kernel.h"
#include "task.h"
#include "ready_queue.h"
#include "wait.h"
#include "check.h"

#include "../sysdepend/cpu_task.h"

/*
 * Per-core task dispatch disable state and execution control.
 *
 * One slot per processor: with two cores executing concurrently each has its
 * own current task, next task and dispatch-disable nesting.  Declared in
 * kernel/knlinc/smp.h, which also re-exposes them as current-core lvalues
 * under their original names for the rest of the kernel.
 */
/*
 * Canaries around the per-core scheduler slots.
 *
 * A fault showed knl_ctxtsk[0..1] and knl_schedtsk[0..1] -- four consecutive
 * words -- all holding the same TCB pointer, while the scheduler's own guards
 * reported no duplicate and no illegal assignment. No code writes those slots
 * except the dispatcher and the assignment, so the remaining explanation is a
 * write through corrupted links from a neighbour: knl_free_tcb, the free-TCB
 * queue head, sits immediately below them.
 *
 * These words are never written after initialisation. If one changes, the
 * corruption is real and its position says which side it came from.
 */
#define KNL_GUARD_PATTERN	0x5a5ac3c3UL
#define KNL_GUARD_SLOTS		2

/*
 * The guard words live inside the arrays rather than beside them. Separate
 * objects are sorted by the linker and landed nowhere useful; trailing
 * elements are guaranteed adjacent, and the array base -- which the
 * dispatcher assembly refers to by symbol -- is unchanged.
 */
Noinit(EXPORT TCB	*knl_ctxtsk[TK_MAX_CORE + KNL_GUARD_SLOTS]);
Noinit(EXPORT TCB	*knl_schedtsk[TK_MAX_CORE + KNL_GUARD_SLOTS]);
Noinit(EXPORT INT	knl_dispatch_disabled[TK_MAX_CORE + KNL_GUARD_SLOTS]);

EXPORT BOOL knl_guards_intact( void )
{
	UINT	i;

	for ( i = TK_MAX_CORE; i < TK_MAX_CORE + KNL_GUARD_SLOTS; i++ ) {
		if ( (UW)knl_ctxtsk[i] != KNL_GUARD_PATTERN ) return FALSE;
		if ( (UW)knl_schedtsk[i] != KNL_GUARD_PATTERN ) return FALSE;
		if ( (UW)knl_dispatch_disabled[i] != KNL_GUARD_PATTERN ) return FALSE;
	}
	return TRUE;
}

Noinit(EXPORT RDYQUE	knl_ready_queue);	/* Ready queue */

/*
 * Task control information
 */
Noinit(EXPORT TCB	knl_tcb_table[NUM_TSKID]);	/* Task control block */
Noinit(EXPORT QUEUE	knl_free_tcb);	/* FreeQue */

/*
 * Per-core idle contexts.
 *
 * The baseline dispatcher waited for work inside PendSV. That left the CPU in
 * Handler mode indefinitely and forced SysTick above PendSV so time could
 * advance. These private TCBs let an empty processor exception-return to a
 * real Thread-mode idle loop without consuming a public task ID or priority.
 */
#define KNL_IDLE_STACK_SIZE	TMP_STACK_SIZE
#define KNL_IDLE_STACK_WORDS	(KNL_IDLE_STACK_SIZE / sizeof(UW))
#define KNL_IDLE_STACK_PAINT	0x1d1e1d1eUL

Noinit(LOCAL UW	knl_idle_stack[TK_MAX_CORE][KNL_IDLE_STACK_WORDS]);
Noinit(LOCAL TCB	knl_idle_tcb[TK_MAX_CORE]);
Noinit(EXPORT TCB *knl_idletsk[TK_MAX_CORE]);

EXPORT volatile UW knl_idle_entries[TK_MAX_CORE];
EXPORT volatile UW knl_idle_loops[TK_MAX_CORE];
EXPORT volatile UW knl_idle_handler_entries[TK_MAX_CORE];

LOCAL void knl_idle_task( INT stacd, void *exinf )
{
	UINT core = knl_current_core();

	(void)exinf;
	if ( core >= TK_MAX_CORE || (UINT)stacd != core ) {
		for ( ;; ) {
			/* An idle context must never migrate. */
		}
	}

	knl_idle_entries[core]++;
	for ( ;; ) {
		if ( knl_get_ipsr() != 0 ) knl_idle_handler_entries[core]++;
		knl_idle_loops[core]++;
		if ( knl_lowpow_discnt == 0 ) {
			low_pow();
		}
	}
}

LOCAL void knl_idle_initialize( void )
{
	UINT core;
	INT i;

	for ( core = 0; core < TK_MAX_CORE; core++ ) {
		TCB *tcb = &knl_idle_tcb[core];

		for ( i = 0; i < (INT)KNL_IDLE_STACK_WORDS; i++ ) {
			knl_idle_stack[core][i] = KNL_IDLE_STACK_PAINT;
		}

		tcb->tskid = 0;
		tcb->exinf = (void *)(UW)core;
		tcb->tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC;
		tcb->task = (FP)knl_idle_task;
		tcb->sstksz = KNL_IDLE_STACK_SIZE;
		tcb->isysmode = 1;
		tcb->ipriority = NUM_TSKPRI - 1;
		tcb->assprc = 1UL << core;
		tcb->isstack = &knl_idle_stack[core][KNL_IDLE_STACK_WORDS];

		knl_make_dormant(tcb);
#if TK_SUPPORT_SMP
		knl_setup_context(tcb);
#endif
		knl_setup_stacd(tcb, (INT)core);
		tcb->state = TS_READY;
		knl_idletsk[core] = tcb;
		knl_idle_entries[core] = 0;
		knl_idle_loops[core] = 0;
		knl_idle_handler_entries[core] = 0;
	}
}

EXPORT UW knl_idle_stack_used( UINT core )
{
	INT i;

	if ( core >= TK_MAX_CORE ) return 0;
	for ( i = 0; i < (INT)KNL_IDLE_STACK_WORDS; i++ ) {
		if ( knl_idle_stack[core][i] != KNL_IDLE_STACK_PAINT ) break;
	}
	return KNL_IDLE_STACK_SIZE - (UW)i * sizeof(UW);
}

/*
 * TCB Initialization
 */
EXPORT ER knl_task_initialize( void )
{
	INT	i;
	TCB	*tcb;
	ID	tskid;

	/* Get system information */
	if ( NUM_TSKID < 1 ) {
		return E_SYS;
	}

	for ( i = TK_MAX_CORE; i < TK_MAX_CORE + KNL_GUARD_SLOTS; i++ ) {
		knl_ctxtsk[i] = (TCB *)KNL_GUARD_PATTERN;
		knl_schedtsk[i] = (TCB *)KNL_GUARD_PATTERN;
		knl_dispatch_disabled[i] = (INT)KNL_GUARD_PATTERN;
	}

	/* Initialize task execution control information (every core) */
	for ( i = 0; i < TK_MAX_CORE; i++ ) {
		knl_ctxtsk[i]  = NULL;
		knl_schedtsk[i] = NULL;
		knl_dispatch_disabled[i] = DDS_ENABLE;
	}
	knl_ready_queue_initialize(&knl_ready_queue);

	/* Register all TCBs onto FreeQue */
	QueInit(&knl_free_tcb);
	for ( tcb = knl_tcb_table, i = 0; i < NUM_TSKID; tcb++, i++ ) {
		tskid = ID_TSK(i);
		tcb->tskid = tskid;
		tcb->state = TS_NONEXIST;
#if USE_LEGACY_API && USE_RENDEZVOUS
		tcb->wrdvno = tskid;
#endif

		QueInsert(&tcb->tskque, &knl_free_tcb);
	}

	knl_idle_initialize();

	return E_OK;
}

/*
 * Prepare task execution.
 */
EXPORT void knl_make_dormant( TCB *tcb )
{
	/* Initialize variables which should be reset at DORMANT state */
	tcb->state	= TS_DORMANT;
	tcb->priority	= tcb->bpriority = tcb->ipriority;
	tcb->sysmode	= tcb->isysmode;
	tcb->wupcnt	= 0;
	tcb->suscnt	= 0;

	tcb->klockwait	= FALSE;
	tcb->klocked	= FALSE;
#if TK_SUPPORT_SMP
	tcb->stkfree	= FALSE;
#endif

#if USE_DBGSPT && defined(USE_FUNC_TD_INF_TSK)
	tcb->stime	= 0;
	tcb->utime	= 0;
#endif

	tcb->wercd = NULL;

#if USE_MUTEX == 1
	tcb->mtxlist	= NULL;
#endif

	/* Rebuilding a remote task's stack before its processor has completed the
	 * final save destroys live context.  SMP rebuilds at create/start, after
	 * ownership has been released; single-core keeps the original lifecycle. */
#if !TK_SUPPORT_SMP
	knl_setup_context(tcb);
#endif
}

/* ------------------------------------------------------------------------ */

/*
 * Set task to READY state.
 *	Update the task state and insert in the ready queue. If necessary, 
 *	update 'knl_schedtsk' and request to start task dispatcher. 
 */
EXPORT void knl_make_ready( TCB *tcb )
{
	tcb->state = TS_READY;
#if TK_SUPPORT_SMP
	(void)knl_ready_queue_insert(&knl_ready_queue, tcb);
	knl_smp_reschedule_all();
#else
	if ( knl_ready_queue_insert(&knl_ready_queue, tcb) ) {
		*knl_schedtsk_slot() = tcb;
	}
#endif
}

/*
 * Set task to non-executable state.
 *	Delete the task from the ready queue.
 *	If the deleted task is 'knl_schedtsk', set 'knl_schedtsk' to the
 *	highest priority task in the ready queue. 
 *	'tcb' task must be READY.
 */
EXPORT void knl_make_non_ready( TCB *tcb )
{
	knl_ready_queue_delete(&knl_ready_queue, tcb);
#if TK_SUPPORT_SMP
	knl_smp_reschedule_all();
#else
	if ( *knl_schedtsk_slot() == tcb ) {
		*knl_schedtsk_slot() = knl_ready_queue_top(&knl_ready_queue);
	}
#endif
}

/*
 * Change task priority.
 */
EXPORT void knl_change_task_priority( TCB *tcb, INT priority )
{
	INT	oldpri;

	if ( tcb->state == TS_READY ) {
		/*
		 * When deleting a task from the ready queue, 
		 * a value in the 'priority' field in TCB is needed. 
		 * Therefore you need to delete the task from the
		 * ready queue before changing 'tcb->priority.'
		 */
		knl_ready_queue_delete(&knl_ready_queue, tcb);
		tcb->priority = (UB)priority;
		knl_ready_queue_insert(&knl_ready_queue, tcb);
		knl_reschedule();
	} else {
		oldpri = tcb->priority;
		tcb->priority = (UB)priority;

		/* If the hook routine at the task priority change is defined,
		   execute it */
		if ( (tcb->state & TS_WAIT) != 0 && tcb->wspec->chg_pri_hook) {
			(*tcb->wspec->chg_pri_hook)(tcb, oldpri);
		}
	}
}

/*
 * Rotate ready queue.
 */
EXPORT void knl_rotate_ready_queue( INT priority )
{
	knl_ready_queue_rotate(&knl_ready_queue, priority);
	knl_reschedule();
}

/*
 * Rotate the ready queue including the highest priority task.
 */
EXPORT void knl_rotate_ready_queue_run( void )
{
	if ( *knl_schedtsk_slot() != NULL ) {
		knl_ready_queue_rotate(&knl_ready_queue,
				knl_ready_queue_top_priority(&knl_ready_queue));
		knl_reschedule();
	}
}

/* ------------------------------------------------------------------------ */
/*
 *	Debug support function
 */
#if USE_DBGSPT

#ifdef USE_FUNC_TD_RDY_QUE
/*
 * Refer ready queue
 */
SYSCALL INT td_rdy_que( PRI pri, ID list[], INT nent )
{
	QUEUE	*q, *tskque;
	INT	n = 0;

	CHECK_PRI(pri);

	BEGIN_DISABLE_INTERRUPT;
	tskque = &knl_ready_queue.tskque[int_priority(pri)];
	for ( q = tskque->next; q != tskque; q = q->next ) {
		if ( n++ < nent ) {
			*(list++) = ((TCB*)q)->tskid;
		}
	}
	END_DISABLE_INTERRUPT;

	return n;
}
#endif /* USE_FUNC_TD_RDY_QUE */

#endif /* USE_DBGSPT */
