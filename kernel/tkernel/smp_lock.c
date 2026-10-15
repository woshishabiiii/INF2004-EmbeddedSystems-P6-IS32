/*
 *	smp_lock.c
 *	Recursive big kernel lock, and the public SMP utility API.
 *
 *	This file is common policy.  Everything architecture-specific -- the
 *	atomics, the hardware spinlocks, the barrier and the timestamp -- comes
 *	from the knl_arch_* boundary in sysdepend, so porting this lock to
 *	another architecture means supplying those primitives, not rewriting
 *	the policy.  (The ESP32-S3 port's own inventory records mixing the two
 *	as its main portability defect; the boundary is drawn from the start
 *	here because ARMv6-M forces such different primitives that mixing them
 *	would not have survived contact.)
 *
 *	Structure of the lock:
 *
 *	  A hardware spinlock (SPINLOCK_KERNEL) is held only across the few
 *	  instructions that inspect and change the lock word.  The kernel lock
 *	  itself is that word, and is held for as long as the critical section
 *	  runs.  This keeps hardware-spinlock hold times to the handful of
 *	  instructions the datasheet asks for, while still allowing a kernel
 *	  section of arbitrary length.
 *
 *	  Local interrupts are masked from acquisition to final release.  Two
 *	  reasons: a hardware spinlock claim must not be interrupted by code on
 *	  the same core that claims the same lock, and an interrupt handler
 *	  that entered the kernel while its own core held the lock would
 *	  release it early on the way out.
 */

#define KNL_SMP_NO_SHORT_NAMES
#include "kernel.h"
#include "smp_lock.h"
#include "task.h"

#if TK_SUPPORT_SMP

#include <tk/smp.h>

/* Architecture boundary (kernel/sysdepend/cpu/rp2040/smp_atomic.c). */
IMPORT void knl_arch_memory_barrier( void );
IMPORT void knl_arch_cpu_relax( void );
IMPORT UW   knl_arch_time_us( void );
IMPORT void knl_arch_hwlock_claim( UINT n );
IMPORT void knl_arch_hwlock_release( UINT n );
IMPORT UW   knl_arch_atomic_fetch_add( volatile UW *addr, UW value );
IMPORT UW   knl_arch_atomic_xchg( volatile UW *addr, UW value );
IMPORT BOOL knl_arch_atomic_cmpxchg( volatile UW *addr, UW expected, UW desired );
IMPORT UW   knl_arch_atomic_cmpxchg_val( volatile UW *addr, UW expected, UW desired );
IMPORT UW   knl_arch_atomic_bitset( volatile UW *addr, UW mask );
IMPORT UW   knl_arch_atomic_bitclr( volatile UW *addr, UW mask );
IMPORT UW   knl_arch_atomic_load( volatile UW *addr );
IMPORT void knl_arch_atomic_store( volatile UW *addr, UW value );

#define SPINLOCK_KERNEL		0

/* ------------------------------------------------------------------------ */

LOCAL volatile UW	knl_lock_word;			/* 0 free, else core+1 */
LOCAL volatile UW	knl_lock_depth[TK_MAX_CORE];
LOCAL UINT		knl_lock_primask[TK_MAX_CORE];
LOCAL UW		knl_lock_acquired_us[TK_MAX_CORE];

LOCAL volatile UW	knl_lock_acquisitions;
LOCAL volatile UW	knl_lock_contentions;
LOCAL volatile UW	knl_lock_recursions;
LOCAL volatile UW	knl_lock_max_depth;
LOCAL volatile UW	knl_lock_max_hold_us;
LOCAL volatile UW	knl_lock_max_wait_us;
LOCAL volatile UW	knl_lock_detached_sections;
LOCAL volatile UW	knl_lock_illegal_suspends;
LOCAL volatile UW	knl_lock_illegal_unlocks;
LOCAL volatile UW	knl_lock_dead_ctxtsk_dispatches;

/* ------------------------------------------------------------------------ */

EXPORT void knl_kernel_lock_enter( void )
{
	UINT	saved;
	UINT	self = knl_current_core();
	UW	token = (UW)self + 1;
	UW	t_wait_start, waited;
	BOOL	contended = FALSE;

	saved = disint();

	/* Already ours?  Only the owning core can write its own token, so this
	   read needs no lock. */
	if ( knl_lock_word == token ) {
		knl_lock_depth[self]++;
		if ( knl_lock_depth[self] > knl_lock_max_depth ) {
			knl_lock_max_depth = knl_lock_depth[self];
		}
		knl_lock_recursions++;
		set_primask(saved);	/* still masked overall: depth > 1 */
		return;
	}

	t_wait_start = knl_arch_time_us();

	/*
	 * Test, then test-and-set.
	 *
	 * The unlocked read first matters: an aligned word read is indivisible,
	 * so a waiter can see the lock is busy without touching the hardware
	 * spinlock at all.  Claiming it on every retry -- as the first version
	 * of this loop did -- makes a waiter hammer the very lock the owner
	 * needs in order to release, and the datasheet gives core 0 the win on
	 * a simultaneous claim, so a spinning core 0 could delay core 1's
	 * release indefinitely.
	 */
	for ( ;; ) {
		if ( knl_lock_word == 0 ) {
			knl_arch_hwlock_claim(SPINLOCK_KERNEL);
			if ( knl_lock_word == 0 ) {
				knl_lock_word = token;
				knl_arch_hwlock_release(SPINLOCK_KERNEL);
				break;
			}
			knl_arch_hwlock_release(SPINLOCK_KERNEL);
		}
		contended = TRUE;
		knl_arch_cpu_relax();
	}

	waited = knl_arch_time_us() - t_wait_start;
	if ( waited > knl_lock_max_wait_us ) knl_lock_max_wait_us = waited;

	knl_lock_depth[self] = 1;
	knl_lock_primask[self] = saved;
	knl_lock_acquired_us[self] = knl_arch_time_us();

	knl_lock_acquisitions++;
	if ( contended ) knl_lock_contentions++;
	if ( knl_lock_max_depth < 1 ) knl_lock_max_depth = 1;

	/* A task terminated by the other processor while spinning here has local
	 * interrupts masked and cannot service the reschedule IPI.  Once it wins
	 * the BKL it must dispatch instead of entering a syscall as dead state. */
	{
		TCB *current = knl_ctxtsk[self];
		if ( current != NULL && knl_taskindp[self] == 0 &&
		     (current->state == TS_DORMANT || current->state == TS_NONEXIST) ) {
			knl_lock_dead_ctxtsk_dispatches++;
			knl_kernel_lock_leave();
			knl_force_dispatch();
			/* knl_force_dispatch() enables interrupts so PendSV can discard
			 * this context.  Never let a delayed exception return dead code to
			 * a caller which believes the BKL was acquired. */
			for ( ;; ) {
				__asm__ volatile ("nop");
			}
		}
	}

	/* Interrupts stay masked until the final leave(). */
}

EXPORT void knl_kernel_lock_leave( void )
{
	UINT	self = knl_current_core();
	UW	token = (UW)self + 1;
	UW	held;

	if ( knl_lock_word != token || knl_lock_depth[self] == 0 ) {
		knl_lock_illegal_unlocks++;
		return;
	}

	if ( --knl_lock_depth[self] > 0 ) {
		return;			/* still held by an outer section */
	}

	held = knl_arch_time_us() - knl_lock_acquired_us[self];
	if ( held > knl_lock_max_hold_us ) knl_lock_max_hold_us = held;

	knl_arch_hwlock_claim(SPINLOCK_KERNEL);
	knl_lock_word = 0;
	knl_arch_hwlock_release(SPINLOCK_KERNEL);

	set_primask(knl_lock_primask[self]);
}

/*
 * Release the lock unconditionally, whatever the recursion depth.
 *
 * For paths that abandon the current context rather than returning through
 * it: a task that exits, or any force-dispatch, never reaches the matching
 * END_CRITICAL_SECTION, so the lock it holds would stay held with nobody able
 * to release it. The other core then blocks forever on its next kernel call,
 * and this core continues with a depth that never returns to zero and a saved
 * interrupt state that is never restored.
 *
 * The interrupt state is deliberately not restored here: the caller is about
 * to enable interrupts itself so the dispatcher can run.
 */
EXPORT void knl_kernel_lock_abandon( void )
{
	UINT	self = knl_current_core();

	if ( knl_lock_word != (UW)self + 1 ) return;

	knl_lock_depth[self] = 0;

	knl_arch_hwlock_claim(SPINLOCK_KERNEL);
	knl_lock_word = 0;
	knl_arch_hwlock_release(SPINLOCK_KERNEL);
}

/*
 * Drop every recursive level while preserving the caller's local interrupt
 * mask.  Time-event handlers use this after their queue/state snapshot is
 * complete, so user code never extends the cross-core kernel-lock hold time.
 */
EXPORT UW knl_kernel_lock_suspend( void )
{
	UINT	self = knl_current_core();
	UW	depth = knl_lock_depth[self];
	UW	held;

	if ( depth == 0 || knl_lock_word != (UW)self + 1 ) {
		knl_lock_illegal_suspends++;
		return 0;
	}

	held = knl_arch_time_us() - knl_lock_acquired_us[self];
	if ( held > knl_lock_max_hold_us ) knl_lock_max_hold_us = held;

	knl_lock_depth[self] = 0;
	knl_lock_detached_sections++;

	knl_arch_hwlock_claim(SPINLOCK_KERNEL);
	knl_lock_word = 0;
	knl_arch_hwlock_release(SPINLOCK_KERNEL);

	/* Interrupts deliberately remain masked. */
	return depth;
}

EXPORT void knl_kernel_lock_resume( UW resume_depth )
{
	UINT	self = knl_current_core();

	if ( resume_depth == 0 || knl_kernel_lock_is_held() ) {
		knl_lock_illegal_suspends++;
		return;
	}

	knl_kernel_lock_enter();
	if ( resume_depth > 1 && knl_kernel_lock_is_held() ) {
		knl_lock_depth[self] = resume_depth;
		if ( resume_depth > knl_lock_max_depth ) {
			knl_lock_max_depth = resume_depth;
		}
	}
}

EXPORT BOOL knl_kernel_lock_is_held( void )
{
	return ( knl_lock_word == (UW)knl_current_core() + 1 )? TRUE: FALSE;
}

EXPORT void knl_kernel_lock_audit( KNL_LOCK_DIAG *diag )
{
	UINT	i;

	if ( diag == NULL ) return;

	diag->acquisitions      = knl_lock_acquisitions;
	diag->contentions       = knl_lock_contentions;
	diag->recursive_entries = knl_lock_recursions;
	diag->max_depth         = knl_lock_max_depth;
	diag->max_hold_us       = knl_lock_max_hold_us;
	diag->max_wait_us       = knl_lock_max_wait_us;
	diag->detached_sections = knl_lock_detached_sections;
	diag->illegal_suspends  = knl_lock_illegal_suspends;
	diag->illegal_unlocks   = knl_lock_illegal_unlocks;
	diag->dead_ctxtsk_dispatches = knl_lock_dead_ctxtsk_dispatches;
	diag->owner             = knl_lock_word;
	for ( i = 0; i < TK_MAX_CORE; i++ ) {
		diag->depth[i] = knl_lock_depth[i];
	}
}

EXPORT void knl_kernel_lock_reset_stats( void )
{
	knl_lock_acquisitions = 0;
	knl_lock_contentions = 0;
	knl_lock_recursions = 0;
	knl_lock_max_depth = 0;
	knl_lock_max_hold_us = 0;
	knl_lock_max_wait_us = 0;
	knl_lock_detached_sections = 0;
	knl_lock_illegal_suspends = 0;
	knl_lock_illegal_unlocks = 0;
	knl_lock_dead_ctxtsk_dispatches = 0;
}

/* ------------------------------------------------------------------------ */
/*
 * Public SMP utility API (SMP T-Kernel specification section 5.6).
 *
 * These spinlocks are software lock words built on the atomic primitives, not
 * the 32 SIO hardware spinlocks: applications must be able to create as many
 * as they need, and the hardware locks are a scarce kernel-reserved resource.
 */

EXPORT ER InitSpinLock( T_SPLOCK *lock )
{
	if ( lock == NULL ) return E_PAR;
	knl_arch_atomic_store((volatile UW *)lock, 0);
	return E_OK;
}

EXPORT ER SpinLock( T_SPLOCK *lock )
{
	if ( lock == NULL ) return E_PAR;
	while ( !knl_arch_atomic_cmpxchg((volatile UW *)lock, 0, 1) ) {
		knl_arch_cpu_relax();
	}
	return E_OK;
}

EXPORT ER SpinTryLock( T_SPLOCK *lock )
{
	if ( lock == NULL ) return E_PAR;
	return knl_arch_atomic_cmpxchg((volatile UW *)lock, 0, 1)? E_OK: E_BUSY;
}

EXPORT ER SpinUnlock( T_SPLOCK *lock )
{
	if ( lock == NULL ) return E_PAR;
	knl_arch_atomic_store((volatile UW *)lock, 0);
	return E_OK;
}

/*
 * Interrupt-preserving variants.  A spinlock taken in task context and also
 * taken by a handler on the same core must be held with interrupts masked, or
 * the handler deadlocks against the task it interrupted.
 */
EXPORT ER ISpinLock( T_SPLOCK *lock, UINT *intsts )
{
	if ( lock == NULL || intsts == NULL ) return E_PAR;
	*intsts = disint();
	while ( !knl_arch_atomic_cmpxchg((volatile UW *)lock, 0, 1) ) {
		knl_arch_cpu_relax();
	}
	return E_OK;
}

EXPORT ER ISpinTryLock( T_SPLOCK *lock, UINT *intsts )
{
	UINT	saved;

	if ( lock == NULL || intsts == NULL ) return E_PAR;
	saved = disint();
	if ( knl_arch_atomic_cmpxchg((volatile UW *)lock, 0, 1) ) {
		*intsts = saved;
		return E_OK;
	}
	set_primask(saved);
	return E_BUSY;
}

EXPORT ER ISpinUnlock( T_SPLOCK *lock, UINT intsts )
{
	if ( lock == NULL ) return E_PAR;
	knl_arch_atomic_store((volatile UW *)lock, 0);
	set_primask(intsts);
	return E_OK;
}

/* ------------------------------------------------------------------------ */
/*
 * Public atomics.
 */

EXPORT UW atomic_inc( UW *addr )
{
	return knl_arch_atomic_fetch_add((volatile UW *)addr, 1) + 1;
}

EXPORT UW atomic_dec( UW *addr )
{
	return knl_arch_atomic_fetch_add((volatile UW *)addr, (UW)-1) - 1;
}

EXPORT UW atomic_add( UW *addr, UW val )
{
	return knl_arch_atomic_fetch_add((volatile UW *)addr, val) + val;
}

EXPORT UW atomic_sub( UW *addr, UW val )
{
	return knl_arch_atomic_fetch_add((volatile UW *)addr, (UW)(-(W)val)) - val;
}

EXPORT UW atomic_xchg( UW *addr, UW val )
{
	return knl_arch_atomic_xchg((volatile UW *)addr, val);
}

EXPORT UW atomic_cmpxchg( UW *addr, UW val, UW cmp )
{
	/* One locked section: reading the old value separately would return
	   something the other core may already have replaced. */
	return knl_arch_atomic_cmpxchg_val((volatile UW *)addr, cmp, val);
}

EXPORT UW atomic_bitset( UW *addr, UW setptn )
{
	return knl_arch_atomic_bitset((volatile UW *)addr, setptn);
}

EXPORT UW atomic_bitclr( UW *addr, UW clrptn )
{
	return knl_arch_atomic_bitclr((volatile UW *)addr, clrptn);
}

/* The spelling is retained from the public SMP T-Kernel specification. */
EXPORT void mp_memory_barrir( void )
{
	knl_arch_memory_barrier();
}

#endif /* TK_SUPPORT_SMP */
