/*
 *	smp_dispatch.c
 *	Cross-core dispatch and explicit task placement (Phase 6).
 *
 *	Phase 6 gives core 1 the dispatcher but not yet the scheduler.  A task
 *	is placed on a core explicitly, by moving it out of the ready queue and
 *	into that core's schedtsk slot under the kernel lock.  Phase 7 replaces
 *	the placement with the global ready-queue assignment; the dispatch
 *	mechanism proven here does not change.
 *
 *	Splitting it this way is deliberate.  The dispatcher and the scheduling
 *	policy fail in different ways -- the dispatcher corrupts contexts, the
 *	policy picks the wrong task -- and debugging them together on hardware
 *	means never knowing which one is wrong.
 *
 *	The invariant that matters here: a task must never be current on two
 *	cores.  Placement therefore removes the task from the ready queue, so
 *	the other core's reschedule cannot select it, and refuses outright if
 *	the other core is already running it.
 */

#define KNL_SMP_NO_SHORT_NAMES
#include "kernel.h"
#include "task.h"
#include "ready_queue.h"
#include "check.h"
#include "smp_lock.h"

#if TK_SUPPORT_SMP

#include "../sysdepend/cpu/rp2040/smp_rp2040.h"

/* Counters for the audit; a non-zero duplicate count is a hard failure. */
EXPORT UW	knl_smp_assign_count;
EXPORT UW	knl_smp_assign_refused;

/*
 * Ask a core to re-evaluate its schedtsk slot.  On the calling core this is
 * just a local PendSV; on the other core it is an IPI whose handler raises
 * PendSV there, because PendSV is a per-core exception and cannot be pended
 * remotely by writing another core's registers.
 */
EXPORT void knl_smp_dispatch_request( UINT core )
{
	if ( core == knl_current_core() ) {
		knl_dispatch();
	} else {
		knl_ipi_send(core, KNL_IPI_RESCHEDULE);
	}
}

/*
 * Place a READY task on a specific core.
 */
EXPORT ER knl_smp_assign_task( ID tskid, UINT core )
{
	TCB	*tcb;
	ER	ercd = E_OK;
	UINT	other;

	if ( core >= TK_MAX_CORE ) return E_PAR;
	if ( tskid < MIN_TSKID || tskid > MAX_TSKID ) return E_ID;

	BEGIN_CRITICAL_SECTION;

	tcb = get_tcb(tskid);
	other = ( core == 0 )? 1: 0;

	if ( tcb->state != TS_READY ) {
		ercd = E_OBJ;
		knl_smp_assign_refused++;
	} else if ( other < TK_MAX_CORE && knl_ctxtsk[other] == tcb ) {
		/* Already current on the other core: placing it here would put
		   one context on two cores. */
		ercd = E_OBJ;
		knl_smp_assign_refused++;
	} else {
		/* Take it out of the ready queue so the other core's
		   reschedule cannot also select it. */
		knl_ready_queue_delete(&knl_ready_queue, tcb);
		if ( other < TK_MAX_CORE && knl_schedtsk[other] == tcb ) {
			knl_schedtsk[other] = knl_ready_queue_top(&knl_ready_queue);
		}
		knl_schedtsk[core] = tcb;
		knl_smp_assign_count++;
		knl_smp_dispatch_request(core);
	}

	END_CRITICAL_SECTION;
	return ercd;
}

/*
 * Audit: how many tasks are current on more than one core.  Must be zero.
 */
EXPORT UINT knl_smp_dispatch_audit( void )
{
	UINT	i, j, dup = 0;

	BEGIN_CRITICAL_SECTION;
	for ( i = 0; i < TK_MAX_CORE; i++ ) {
		if ( knl_ctxtsk[i] == NULL ) continue;
		for ( j = i + 1; j < TK_MAX_CORE; j++ ) {
			if ( knl_ctxtsk[i] == knl_ctxtsk[j] ) dup++;
		}
	}
	END_CRITICAL_SECTION;
	return dup;
}

/* A task's stored affinity, so the harness can verify TA_ASSPRC actually took
   effect rather than inferring it from where the task happened to run. */
EXPORT UW knl_smp_task_assprc( ID tskid )
{
	if ( tskid < MIN_TSKID || tskid > MAX_TSKID ) return 0;
	return get_tcb(tskid)->assprc;
}

/* Which task each core is currently running, for the harness to print. */
EXPORT ID knl_smp_running_task( UINT core )
{
	TCB	*tcb;

	if ( core >= TK_MAX_CORE ) return 0;
	tcb = knl_ctxtsk[core];
	return ( tcb == NULL )? 0: tcb->tskid;
}

#endif /* TK_SUPPORT_SMP */
