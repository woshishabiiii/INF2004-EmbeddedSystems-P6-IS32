/*
 *	ready_queue_smp.c
 *	Global ready-queue assignment.
 *
 *	The single-core kernel only ever had to name one task: the head of the
 *	highest occupied priority. An SMP scheduler must name up to TK_MAX_CORE
 *	tasks at once, without placing one task on two processors and without
 *	violating any task's affinity.
 *
 *	Policy, unchanged from the ESP32-S3 port so the two remain comparable:
 *	one global ready queue, fixed priority, FIFO within a priority, static
 *	creation-time affinity, migration permitted, and a preference for
 *	leaving a task on the processor already running it.
 *
 *	This file is policy and contains nothing architecture-specific. The
 *	only hardware-dependent parts of scheduling -- reading the current core
 *	and poking the other one -- live behind knl_current_core() and the IPI.
 */

#define KNL_SMP_NO_SHORT_NAMES
#include "kernel.h"
#include "task.h"
#include "ready_queue.h"

#if TK_SUPPORT_SMP

IMPORT void knl_smp_dispatch_request( UINT core );
IMPORT volatile UW knl_smp_online_mask;

EXPORT UW knl_ready_queue_hist[TK_MAX_CORE];

/*
 * Illegal-assignment guard.
 *
 * A task must never be published to a processor its affinity excludes. The
 * consequence of getting this wrong is not a wrong scheduling decision but a
 * corrupt one: the target core restores a context that another core may still
 * own, and the failure surfaces later as a dispatcher branching through a
 * bogus saved return address.
 *
 * Rather than trust the matcher, check the result it produces. Refusing an
 * illegal selection costs one comparison per core per reschedule and turns a
 * silent corruption into a counter.
 */
EXPORT UW knl_smp_bad_assign;
/*
 * What publication last wrote, per core, and how many times.
 *
 * Everything checkable says schedtsk[0] cannot hold the pinned worker:
 * affinity is stored correctly, the mask can only yield core 1, the affinity
 * guard would reject it, and the slot guards are intact. Yet it does. Record
 * the value this function actually stores so the fault can be compared
 * against it: if the slot disagrees with the last publish, the write did not
 * come from here.
 */
EXPORT UW knl_smp_last_pub[TK_MAX_CORE];
EXPORT UW knl_smp_pub_count[TK_MAX_CORE];

EXPORT UW knl_smp_dup_candidate;	/* same task offered twice by the queue */
EXPORT UW knl_smp_dup_publish;		/* same task selected for two cores */
EXPORT UW knl_smp_bad_assign_task;
EXPORT UW knl_smp_bad_assign_core;

/*
 * Reject a task the scan has already offered.
 *
 * One task must never occupy two processors. A ready queue that lists a task
 * twice -- which a double wakeup produces -- would otherwise yield two
 * candidates that both match, and both cores would install the same context.
 * The damage appears far later as a dispatcher branching through a saved
 * return address that the other core has already consumed.
 */
LOCAL BOOL knl_already_candidate( TCB *candidate[], INT count, TCB *tcb )
{
	INT	i;

	for ( i = 0; i < count; i++ ) {
		if ( candidate[i] == tcb ) {
			knl_smp_dup_candidate++;
			return TRUE;
		}
	}
	return FALSE;
}

LOCAL UINT knl_popcount( UW bits )
{
	UINT	count = 0;

	while ( bits != 0 ) {
		bits &= bits - 1;
		count++;
	}
	return count;
}

/*
 * Which processors this task may be placed on right now.
 *
 * A processor whose current task has disabled dispatch is unavailable --
 * except to that very task, which is still entitled to keep running there.
 * Getting this wrong would let tk_dis_dsp() be defeated by migration.
 */
LOCAL UW knl_task_core_mask( TCB *tcb )
{
	/* Only processors that are actually running tasks.  Assigning to a core
	   that has not reached the dispatcher loses the task silently. */
	UW	enabled = ALL_CORE_BIT & knl_smp_online_mask;
	UINT	core;

	/*
	 * A task that is currently executing stays on the processor executing
	 * it, whatever its affinity would otherwise allow.
	 *
	 * Migrating a running task would require the losing core to have saved
	 * its context before the gaining core restores it, and nothing
	 * sequences those two events: the assignment publishes schedtsk and
	 * sends an IPI, and the target core may install the context while the
	 * original core is still running from it. The task would exist on two
	 * processors, which is the one invariant whose violation corrupts
	 * silently instead of faulting.
	 *
	 * This costs nothing in reachable placements. A task stops being
	 * current the moment its core dispatches away from it, and it is freely
	 * migratable from then on -- migration simply happens at a scheduling
	 * point rather than underneath a running task.
	 */
	for ( core = 0; core < TK_MAX_CORE; core++ ) {
		if ( knl_ctxtsk[core] == tcb ) {
			return (1UL << core) & enabled;
		}
	}

	/*
	 * Publication is ownership too.
	 *
	 * A dispatcher consumes schedtsk without taking the kernel lock.  If a
	 * task is present in one core's slot, that core may already have loaded
	 * the pointer and be about to restore it even when ctxtsk has not yet
	 * changed.  Moving the same task to another slot in that window can run
	 * one saved context on two processors.
	 *
	 * Keep a published task on its owning core until that slot is replaced.
	 * This also prevents the duplicate-publication guard below from solving
	 * the conflict by dropping the task entirely.  That old behaviour
	 * stranded a READY mutex owner: the matcher selected it for core 1, the
	 * stale core-0 publication caused it to be discarded, and a lower-
	 * priority task ran while the owner remained in the ready queue.
	 */
	for ( core = 0; core < TK_MAX_CORE; core++ ) {
		if ( knl_schedtsk[core] == tcb ) {
			return (1UL << core) & enabled;
		}
	}

	/* A processor whose current task disabled dispatch accepts no one
	   else; the task itself was already handled above. */
	for ( core = 0; core < TK_MAX_CORE; core++ ) {
		if ( knl_dispatch_disabled[core] >= DDS_DISABLE ) {
			enabled &= ~(1UL << core);
		}
	}
	return tcb->assprc & enabled & ALL_CORE_BIT;
}

/*
 * Hall's condition, reduced to the two-processor case: a candidate set is
 * feasible when the union of the processors its members may run on is at
 * least as large as the set itself.
 */
LOCAL BOOL knl_candidate_feasible( UW masks[], INT count, UW next )
{
	UW	union_bits = next;
	INT	i;

	if ( next == 0 ) return FALSE;
	for ( i = 0; i < count; i++ ) union_bits |= masks[i];
	knl_ready_queue_hist[count] = ((UW)(count + 1) << 16) | union_bits;
	return ( knl_popcount(union_bits) >= (UINT)(count + 1) )? TRUE: FALSE;
}

/*
 * Place the chosen candidates on processors.
 *
 * Order matters. Pinned tasks are placed first, because they have no
 * alternative and a migratable task placed greedily could take the only
 * processor a pinned task is allowed to use. A task already running on a
 * legal processor is then kept there, which avoids pointless migration and
 * the cache traffic that comes with it. Whatever remains fills the gaps.
 */
LOCAL void knl_assign_cores( TCB *out[], TCB *candidate[], UW masks[], INT count )
{
	INT	i, core;

	for ( core = 0; core < TK_MAX_CORE; core++ ) out[core] = NULL;

	for ( i = 0; i < count; i++ ) {			/* pinned first */
		if ( knl_popcount(masks[i]) != 1 ) continue;
		for ( core = 0; core < TK_MAX_CORE; core++ ) {
			if ( (masks[i] & (1UL << core)) != 0 && out[core] == NULL ) {
				out[core] = candidate[i];
				candidate[i] = NULL;
				break;
			}
		}
	}
	for ( i = 0; i < count; i++ ) {			/* stay where you are */
		if ( candidate[i] == NULL ) continue;
		for ( core = 0; core < TK_MAX_CORE; core++ ) {
			if ( out[core] == NULL && knl_ctxtsk[core] == candidate[i]
			  && (masks[i] & (1UL << core)) != 0 ) {
				out[core] = candidate[i];
				candidate[i] = NULL;
				break;
			}
		}
	}
	for ( i = 0; i < count; i++ ) {			/* fill the rest */
		if ( candidate[i] == NULL ) continue;
		for ( core = 0; core < TK_MAX_CORE; core++ ) {
			if ( out[core] == NULL && (masks[i] & (1UL << core)) != 0 ) {
				out[core] = candidate[i];
				candidate[i] = NULL;
				break;
			}
		}
	}
}

/*
 * Select one task per processor from the global ready queue.
 *
 * Object-lock holders are scanned first and across every priority, because
 * holding an object lock is a property of a task rather than of a single
 * slot: the legacy kernel gives such a task the highest-run privilege, and on
 * two processors several may hold locks on different objects at once.
 */
EXPORT void knl_ready_queue_assign( RDYQUE *rq, TCB *out[] )
{
	TCB	*candidate[TK_MAX_CORE];
	UW	masks[TK_MAX_CORE];
	INT	count = 0;
	INT	priority, i;
	QUEUE	*head, *q;
	BOOL	has_migratable = FALSE;

	for ( i = 0; i < TK_MAX_CORE; i++ ) {
		candidate[i] = NULL;
		masks[i] = 0;
		knl_ready_queue_hist[i] = 0;
	}

	for ( priority = 0; priority < NUM_TSKPRI && count < TK_MAX_CORE; priority++ ) {
		head = &rq->tskque[priority];
		for ( q = head->next; q != head && count < TK_MAX_CORE; q = q->next ) {
			TCB *owner = (TCB *)q;
			UW  mask;
			if ( !owner->klocked ) continue;
			if ( knl_already_candidate(candidate, count, owner) ) continue;
			mask = knl_task_core_mask(owner);
			if ( !knl_candidate_feasible(masks, count, mask) ) continue;
			candidate[count] = owner;
			masks[count] = mask;
			count++;
		}
	}

	for ( priority = 0; priority < NUM_TSKPRI && count < TK_MAX_CORE; priority++ ) {
		head = &rq->tskque[priority];
		for ( q = head->next; q != head && count < TK_MAX_CORE; q = q->next ) {
			TCB *tcb = (TCB *)q;
			UW  mask;
			if ( tcb->klocked ) continue;
			if ( knl_already_candidate(candidate, count, tcb) ) continue;
			mask = knl_task_core_mask(tcb);
			if ( !knl_candidate_feasible(masks, count, mask) ) continue;
			candidate[count] = tcb;
			masks[count] = mask;
			count++;
		}
	}

	for ( i = 0; i < count; i++ ) {
		if ( knl_popcount(masks[i]) > 1 ) has_migratable = TRUE;
	}
	(void)has_migratable;	/* the two-core matcher handles both cases */

	knl_assign_cores(out, candidate, masks, count);
	/* No "unchanged" flag is kept: the selection is recomputed on every
	   call.  The S3 port caches it, but that cache is only sound while a
	   single core mutates the queue, and here both do. */
}

/*
 * Recompute every processor's selection and publish it.
 *
 * Callers already hold the kernel lock, so the queue cannot move underneath
 * this. A processor whose selection changed is poked; the calling core pends
 * its own PendSV, and the other core is reached through the IPI, because
 * PendSV cannot be pended remotely.
 */
EXPORT void knl_smp_reschedule_all( void )
{
	TCB	*sel[TK_MAX_CORE];
	UINT	core, self = knl_current_core();

	knl_ready_queue_assign(&knl_ready_queue, sel);

	/*
	 * A task already published to another core must not be published here.
	 *
	 * The within-call check below catches sel[0] == sel[1]. It does not
	 * catch one call publishing a task to core 1 and a later call
	 * publishing the same task to core 0, which leaves both slots holding
	 * it and both dispatchers installing the same context.
	 */
	for ( core = 0; core < TK_MAX_CORE; core++ ) {
		UINT other;
		if ( sel[core] == NULL ) continue;
		for ( other = 0; other < TK_MAX_CORE; other++ ) {
			if ( other == core ) continue;
			if ( knl_schedtsk[other] == sel[core]
			  && sel[other] != sel[core] ) {
				knl_smp_dup_publish++;
				sel[core] = NULL;
				break;
			}
		}
	}

	/* Whatever the matcher produced, one task may not go to two cores. */
	for ( core = 1; core < TK_MAX_CORE; core++ ) {
		UINT prev;
		for ( prev = 0; prev < core; prev++ ) {
			if ( sel[core] != NULL && sel[core] == sel[prev] ) {
				knl_smp_dup_publish++;
				sel[core] = NULL;
			}
		}
	}

	for ( core = 0; core < TK_MAX_CORE; core++ ) {
		TCB *t = sel[core];
		if ( t != NULL && (t->assprc & (1UL << core)) == 0 ) {
			knl_smp_bad_assign++;
			knl_smp_bad_assign_task = (UW)t->tskid;
			knl_smp_bad_assign_core = core;
			sel[core] = NULL;	/* refuse to publish it */
		}
	}

	for ( core = 0; core < TK_MAX_CORE; core++ ) {
		if ( (knl_smp_online_mask & (1UL << core)) == 0 ) continue;
		if ( knl_schedtsk[core] == sel[core] ) continue;
		knl_schedtsk[core] = sel[core];
		knl_smp_last_pub[core] = (sel[core] == NULL)? 0: (UW)sel[core]->tskid;
		knl_smp_pub_count[core]++;
		if ( core != self ) {
			knl_smp_dispatch_request(core);
		}
	}
}

#endif /* TK_SUPPORT_SMP */
