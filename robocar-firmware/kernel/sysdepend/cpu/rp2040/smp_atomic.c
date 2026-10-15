/*
 *	smp_atomic.c
 *	RP2040 architecture primitives for SMP (Phase 4).
 *
 *	Cortex-M0+ implements ARMv6-M, which has no LDREX/STREX and therefore
 *	no compare-and-swap of any kind.  Every atomic here is built from a SIO
 *	hardware spinlock held across a read-modify-write, with local interrupts
 *	masked for the duration.
 *
 *	The consequence is structural rather than cosmetic.  On an architecture
 *	with exclusives, an atomic is a cheap primitive from which locks are
 *	built; here an atomic *is* a lock acquisition, costing a spinlock claim
 *	and an interrupt-disable window.  Code that would reasonably spam
 *	atomics on such a machine must not do so here, and the kernel lock is
 *	deliberately not built by spinning on an atomic word.
 *
 *	Interrupts must be masked while a hardware spinlock is held: an
 *	interrupt on the same core that tried to claim the same lock would
 *	deadlock against itself, since these locks are not recursive.
 */

#include <sys/machine.h>
#ifdef CPU_RP2040

#define KNL_SMP_NO_SHORT_NAMES
#include "kernel.h"
#include "../../sysdepend.h"
#include "smp_rp2040.h"

#if TK_SUPPORT_SMP

/*
 * SIO hardware spinlocks (RP2040 datasheet 2.3.1.5).
 *
 * Reading SPINLOCKn attempts to claim it: a non-zero result means the claim
 * succeeded, zero means another core holds it.  Any write releases it.  Core
 * 0 wins a simultaneous claim.
 *
 * Allocation.  The kernel reserves the three below; the remainder are free
 * for applications.  This map exists so imported code cannot silently collide
 * with the kernel -- nothing in this port links the pico-sdk runtime, which
 * has its own conventions for these locks.
 *
 *	0	kernel lock word transitions (held for a few instructions)
 *	1	generic atomic read-modify-write
 *	2	IPI pending masks
 *	3-31	unreserved
 */
#define SPINLOCK_BASE_ADDR	(0xd0000000 + 0x100)
#define SPINLOCK(n)		(*(_UW*)(SPINLOCK_BASE_ADDR + ((n) * 4)))

#define SPINLOCK_KERNEL		0
#define SPINLOCK_ATOMIC		1
#define SPINLOCK_IPI		2

/* ------------------------------------------------------------------------ */

EXPORT void knl_arch_memory_barrier( void )
{
	__asm__ volatile ("dmb" ::: "memory");
}

EXPORT void knl_arch_cpu_relax( void )
{
	/* No yield hint worth issuing on M0+; the barrier keeps the spin from
	   being hoisted and keeps the re-read honest. */
	__asm__ volatile ("" ::: "memory");
}

/*
 * Free-running microsecond timer (TIMER block, TIMERAWL).
 *
 * Deliberately not called a cycle counter: Cortex-M0+ has no DWT cycle
 * counter, so this is 1 MHz wall time, not cycles.  Lock hold and wait
 * figures derived from it are microseconds with 1 us granularity, which is
 * coarse relative to a short critical section -- adequate for spotting
 * outliers, not for costing individual acquisitions.
 */
#define TIMER_RAWL	(*(_UW*)0x40054028)

EXPORT UW knl_arch_time_us( void )
{
	return TIMER_RAWL;
}

/* ------------------------------------------------------------------------ */
/*
 * Raw hardware spinlock claim/release.  Callers must already have local
 * interrupts masked.
 */
EXPORT void knl_arch_hwlock_claim( UINT n )
{
	while ( SPINLOCK(n) == 0 ) {
		knl_arch_cpu_relax();
	}
	knl_arch_memory_barrier();
}

EXPORT void knl_arch_hwlock_release( UINT n )
{
	knl_arch_memory_barrier();
	SPINLOCK(n) = 0;		/* any write releases */
}

/* ------------------------------------------------------------------------ */
/*
 * Atomic read-modify-write.
 *
 * Each of these masks interrupts, claims SPINLOCK_ATOMIC, performs the
 * update and releases.  They are correct for any aligned word in any memory
 * both cores can reach; there is no restriction to a particular RAM bank,
 * because exclusion comes from the lock rather than from the bus.
 */

#define ATOMIC_ENTER(saved)	{ (saved) = disint(); \
				  knl_arch_hwlock_claim(SPINLOCK_ATOMIC); }
#define ATOMIC_LEAVE(saved)	{ knl_arch_hwlock_release(SPINLOCK_ATOMIC); \
				  set_primask(saved); }

EXPORT UW knl_arch_atomic_load( volatile UW *addr )
{
	/* An aligned word load is indivisible on this bus; the barrier gives
	   it the ordering the callers expect. */
	UW v = *addr;
	knl_arch_memory_barrier();
	return v;
}

EXPORT void knl_arch_atomic_store( volatile UW *addr, UW value )
{
	knl_arch_memory_barrier();
	*addr = value;
	knl_arch_memory_barrier();
}

EXPORT UW knl_arch_atomic_fetch_add( volatile UW *addr, UW value )
{
	UINT	saved;
	UW	old;

	ATOMIC_ENTER(saved);
	old = *addr;
	*addr = old + value;
	ATOMIC_LEAVE(saved);
	return old;
}

EXPORT UW knl_arch_atomic_xchg( volatile UW *addr, UW value )
{
	UINT	saved;
	UW	old;

	ATOMIC_ENTER(saved);
	old = *addr;
	*addr = value;
	ATOMIC_LEAVE(saved);
	return old;
}

EXPORT BOOL knl_arch_atomic_cmpxchg( volatile UW *addr, UW expected, UW desired )
{
	UINT	saved;
	BOOL	ok;

	ATOMIC_ENTER(saved);
	ok = ( *addr == expected );
	if ( ok ) *addr = desired;
	ATOMIC_LEAVE(saved);
	return ok;
}

/*
 * Compare-and-exchange returning the previous value.  The comparison, the
 * store and the read of the old value must all happen inside one locked
 * section: doing the load separately would return a value another core may
 * already have replaced, which is exactly the race the caller is using this
 * to avoid.
 */
EXPORT UW knl_arch_atomic_cmpxchg_val( volatile UW *addr, UW expected, UW desired )
{
	UINT	saved;
	UW	old;

	ATOMIC_ENTER(saved);
	old = *addr;
	if ( old == expected ) *addr = desired;
	ATOMIC_LEAVE(saved);
	return old;
}

EXPORT UW knl_arch_atomic_bitset( volatile UW *addr, UW mask )
{
	UINT	saved;
	UW	old;

	ATOMIC_ENTER(saved);
	old = *addr;
	*addr = old | mask;
	ATOMIC_LEAVE(saved);
	return old;
}

EXPORT UW knl_arch_atomic_bitclr( volatile UW *addr, UW mask )
{
	UINT	saved;
	UW	old;

	ATOMIC_ENTER(saved);
	old = *addr;
	*addr = old & ~mask;
	ATOMIC_LEAVE(saved);
	return old;
}

/*
 * Atomically take the whole word and zero it.  This is the primitive the IPI
 * pending mask needs: a receiver must consume every reason bit raised so far
 * without losing one raised between the read and the clear.
 */
EXPORT UW knl_arch_atomic_take( volatile UW *addr )
{
	return knl_arch_atomic_xchg(addr, 0);
}

#endif /* TK_SUPPORT_SMP */
#endif /* CPU_RP2040 */
