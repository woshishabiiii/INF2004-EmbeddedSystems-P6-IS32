/*
 *	smp.h
 *	Per-core kernel state.
 *
 *	The official kernel keeps the current task, the task to be executed,
 *	the dispatch-disable nesting and the task-independent nesting as
 *	singletons.  With two processors executing concurrently each needs its
 *	own, indexed by hardware core id.
 *
 *	The short-name macros below re-expose each array as a current-core
 *	lvalue, so the large majority of common kernel code -- which only ever
 *	means "the core I am running on" -- compiles unchanged.  A file that
 *	defines the storage, or that must address a specific core rather than
 *	its own, defines KNL_SMP_NO_SHORT_NAMES before including kernel.h and
 *	uses the explicit accessors.
 *
 *	In the single-core profile TK_MAX_CORE is 1 and knl_current_core()
 *	folds to a constant zero, so the arrays collapse and the generated code
 *	is equivalent to the original singletons.
 *
 *	This structure follows the ESP32-S3 micro T-Kernel SMP port; the
 *	mechanism for obtaining the core id is RP2040-specific.
 */

#ifndef _KERNEL_SMP_
#define _KERNEL_SMP_

#include <sys/sysdef.h>
#include <sys/profile.h>

typedef struct task_control_block TCB;

/* Two trailing guard elements each; see kernel/tkernel/task.c. */
IMPORT INT	knl_dispatch_disabled[TK_MAX_CORE + 2];
IMPORT TCB	*knl_ctxtsk[TK_MAX_CORE + 2];
IMPORT TCB	*knl_schedtsk[TK_MAX_CORE + 2];
IMPORT W	knl_taskindp[TK_MAX_CORE];

#if TK_SUPPORT_SMP
/* Route a detached cyclic/alarm callback to its selected processor. */
IMPORT void knl_smp_time_handler_request( UINT core );
#endif

#if TK_SUPPORT_SMP

/*
 * Hardware core id.  On RP2040 this is a single load from the SIO block,
 * which is core-local: the same address reads 0 on core 0 and 1 on core 1.
 * No lock or memory barrier is involved.
 */
Inline UINT knl_current_core( void )
{
	return (UINT)(*(_UW*)SIO_CPUID_ADDR);
}

#else

Inline UINT knl_current_core( void )
{
	return 0;
}

#endif /* TK_SUPPORT_SMP */

/* ------------------------------------------------------------------------ */
/*
 * Current-core slot accessors
 */

Inline INT *knl_dispatch_disabled_slot( void )
{
	return &knl_dispatch_disabled[knl_current_core()];
}

Inline TCB **knl_ctxtsk_slot( void )
{
	return &knl_ctxtsk[knl_current_core()];
}

Inline TCB **knl_schedtsk_slot( void )
{
	return &knl_schedtsk[knl_current_core()];
}

Inline W *knl_taskindp_slot( void )
{
	return &knl_taskindp[knl_current_core()];
}

/*
 * Explicit per-core accessors, for code that must address a core other than
 * its own.  Usable from files that keep the short names, which would
 * otherwise shadow the arrays.
 */

Inline TCB **knl_ctxtsk_core_slot( UINT core )
{
	return &knl_ctxtsk[core];
}

Inline TCB **knl_schedtsk_core_slot( UINT core )
{
	return &knl_schedtsk[core];
}

Inline INT *knl_dispatch_disabled_core_slot( UINT core )
{
	return &knl_dispatch_disabled[core];
}

#ifndef KNL_SMP_NO_SHORT_NAMES
#define knl_dispatch_disabled	(*knl_dispatch_disabled_slot())
#define knl_ctxtsk		(*knl_ctxtsk_slot())
#define knl_schedtsk		(*knl_schedtsk_slot())
#define knl_taskindp		(*knl_taskindp_slot())
#endif /* KNL_SMP_NO_SHORT_NAMES */

#endif /* _KERNEL_SMP_ */
