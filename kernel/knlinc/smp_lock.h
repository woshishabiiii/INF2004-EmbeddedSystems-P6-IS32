/*
 *	smp_lock.h
 *	Recursive big kernel lock (internal interface).
 *
 *	Most official micro T-Kernel code assumes that masking local interrupts
 *	serialises access to kernel state.  That holds on one processor and
 *	fails on two: masking interrupts on core 0 says nothing about what core
 *	1 is doing.  This lock provides the missing cross-core exclusion for
 *	the initial global-scheduler design.
 *
 *	It is recursive per core, because kernel entry points nest and the
 *	official code takes critical sections inside critical sections.
 */

#ifndef _KERNEL_SMP_LOCK_
#define _KERNEL_SMP_LOCK_

#include <tk/typedef.h>
#include <sys/sysdef.h>

/* Diagnostics.  Hold and wait figures are microseconds from the 1 MHz timer,
   not cycles: Cortex-M0+ has no cycle counter.  They are useful for spotting
   outliers, not for costing an individual acquisition. */
typedef struct {
	UW	acquisitions;		/* uncontended + contended entries */
	UW	contentions;		/* entries that had to wait */
	UW	recursive_entries;	/* re-entries by the owning core */
	UW	max_depth;		/* deepest recursion observed */
	UW	max_hold_us;		/* longest time the lock was held */
	UW	max_wait_us;		/* longest time spent waiting for it */
	UW	detached_sections;	/* temporary full releases for user callbacks */
	UW	illegal_suspends;	/* invalid suspend/resume operations */
	UW	illegal_unlocks;	/* leave() without a matching enter() */
	UW	dead_ctxtsk_dispatches; /* killed task intercepted after BKL wait */
	UW	owner;			/* 0 = free, otherwise core + 1 */
	UW	depth[TK_MAX_CORE];	/* per-core recursion depth */
} KNL_LOCK_DIAG;

IMPORT void knl_kernel_lock_enter( void );
IMPORT void knl_kernel_lock_leave( void );
IMPORT void knl_kernel_lock_abandon( void );
/* Temporarily release every recursive level without restoring interrupts.
   The nonzero return value is the token required by resume(). */
IMPORT UW knl_kernel_lock_suspend( void );
IMPORT void knl_kernel_lock_resume( UW resume_depth );
IMPORT BOOL knl_kernel_lock_is_held( void );
IMPORT void knl_kernel_lock_audit( KNL_LOCK_DIAG *diag );
IMPORT void knl_kernel_lock_reset_stats( void );

#endif /* _KERNEL_SMP_LOCK_ */
