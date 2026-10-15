/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.0 BSP
 *
 *    Copyright (C) 2021-2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2022/11.
 *
 *----------------------------------------------------------------------
 */

/*
 *	cpu_status.h (ARMv6-M)
 *	CPU-Dependent Status Definition
 */

#ifndef _SYSDEPEND_CPU_CORE_STATUS_
#define _SYSDEPEND_CPU_CORE_STATUS_

#include <tk/syslib.h>
#include <sys/sysdef.h>

#include "sysdepend.h"
/* Per-core kernel state.  Included directly so these macros work in files
   that suppress the current-core short names. */
#include "../../../../knlinc/smp.h"
#include "../../../../knlinc/smp_lock.h"

/*
 * Start/End critical section
 *
 * Masking local interrupts is what the official kernel relies on to serialise
 * access to shared kernel state.  That is sufficient on one processor and
 * false on two: masking on core 0 says nothing about what core 1 is doing.
 * In the SMP profile the kernel lock supplies the missing cross-core
 * exclusion, and the whole kernel becomes one critical region.
 *
 * The lock is released before the dispatch check, because dispatching while
 * holding it would carry it into another task's context.
 *
 * Cost to be aware of: the lock keeps local interrupts masked for the whole
 * section, and a handler on one core that enters the kernel while the other
 * core holds the lock will spin until it is released.  Interrupt latency on
 * each core is therefore coupled to the other core's longest kernel section.
 * Bounding that is Phase 12's problem, and it is the X_i cross-core
 * interference term any timing claim has to account for.
 *
 * In the single-core profile these collapse to nothing and the original
 * behaviour is preserved exactly.
 */
#if TK_SUPPORT_SMP
#define KNL_KERNEL_LOCK_ENTER()	knl_kernel_lock_enter()
#define KNL_KERNEL_LOCK_LEAVE()	knl_kernel_lock_leave()
#else
#define KNL_KERNEL_LOCK_ENTER()
#define KNL_KERNEL_LOCK_LEAVE()
#endif

#define BEGIN_CRITICAL_SECTION	{ UINT _primask_ = disint();			\
				  KNL_KERNEL_LOCK_ENTER();
#define END_CRITICAL_SECTION	KNL_KERNEL_LOCK_LEAVE();				\
				if ( !isDI(_primask_)					\
				  && *knl_ctxtsk_slot() != *knl_schedtsk_slot()		\
				  && !*knl_dispatch_disabled_slot() ) {			\
					knl_dispatch();					\
				}							\
				set_primask(_primask_); }

/*
 * Start/End interrupt disable section
 */
#define BEGIN_DISABLE_INTERRUPT	{ UINT _basepri_ = disint();
#define END_DISABLE_INTERRUPT	set_primask(_basepri_); }

/*
 * Interrupt enable/disable
 *
 * The official kernel uses these to protect shared structures -- free-object
 * queues in particular -- on the assumption that masking local interrupts
 * excludes every other accessor. On two processors it excludes nothing, and
 * tk_cre_mpf() manipulating knl_free_mpfcb this way is what corrupted the
 * scheduler slots: the queue links were rewritten concurrently and a later
 * dispatch branched through the wreckage.
 *
 * In the SMP profile they therefore take the kernel lock, which masks local
 * interrupts as well and is recursive, so nesting inside an existing critical
 * section is harmless. Sites that never pair a release are safe because they
 * end in knl_force_dispatch(), which abandons the lock explicitly.
 *
 * DISABLE_INTERRUPT_LOCAL is the original meaning, for the few places that
 * want interrupt nesting control rather than mutual exclusion.
 */
#define ENABLE_INTERRUPT_LOCAL	{ set_primask(0); }
#define DISABLE_INTERRUPT_LOCAL	{ disint(); }

#if TK_SUPPORT_SMP
#define ENABLE_INTERRUPT	{ knl_kernel_lock_leave(); }
#define DISABLE_INTERRUPT	{ knl_kernel_lock_enter(); }
#else
#define ENABLE_INTERRUPT	{ set_primask(0); }
#define DISABLE_INTERRUPT	{ disint(); }
#endif

/*
 * Enable interrupt nesting
 *	Enable the interrupt that has a higher priority than 'level.'
 *		ARMv6M does not support CPU interrupt mask levels.
 */
#define ENABLE_INTERRUPT_UPTO(level)

/*
 *  Task-independent control
 *
 *  The nesting count is per core (declared in kernel/knlinc/smp.h): each
 *  processor is independently inside or outside a handler.
 */

/*
 * If it is the task-independent part, TRUE
 */
Inline BOOL knl_isTaskIndependent( void )
{
	return ( *knl_taskindp_slot() > 0 )? TRUE: FALSE;
}
/*
 * Move to/Restore task independent part
 */
Inline void knl_EnterTaskIndependent( void )
{
	(*knl_taskindp_slot())++;
}
Inline void knl_LeaveTaskIndependent( void )
{
	(*knl_taskindp_slot())--;
}

/*
 * Move to/Restore task independent part
 */
#define ENTER_TASK_INDEPENDENT	{ knl_EnterTaskIndependent(); }
#define LEAVE_TASK_INDEPENDENT	{ knl_LeaveTaskIndependent(); }

/* ----------------------------------------------------------------------- */
/*
 *	Check system state
 */

/*
 * When a system call is called from the task independent part, TRUE
 */
#define in_indp()	( knl_isTaskIndependent() || *knl_ctxtsk_slot() == NULL )

/*
 * When a system call is called during dispatch disable, TRUE
 * Also include the task independent part as during dispatch disable.
 */
#define in_ddsp()	( *knl_dispatch_disabled_slot()	\
			|| in_indp()			\
			|| isDI(get_primask()) )

/*
 * When a system call is called during CPU lock (interrupt disable), TRUE
 * Also include the task independent part as during CPU lock.
 */
#define in_loc()	( isDI(get_primask())		\
			|| in_indp() )

/*
 * When a system call is called during executing the quasi task part, TRUE
 * Valid only when in_indp() == FALSE because it is not discriminated from 
 * the task independent part. 
 */
#define in_qtsk()	( (*knl_ctxtsk_slot())->sysmode > (*knl_ctxtsk_slot())->isysmode )


#endif /* _SYSDEPEND_CPU_CORE_STATUS_ */
