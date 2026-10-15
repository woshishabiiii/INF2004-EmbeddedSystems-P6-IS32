/*
 *	smp.h
 *	SMP utility API (SMP T-Kernel specification section 5.6).
 *
 *	On one processor, masking interrupts is enough to keep another task or
 *	handler out of a shared structure.  On two it is not: masking on core 0
 *	says nothing about core 1.  Applications and shared services therefore
 *	need real cross-core primitives, which is what this header declares.
 *
 *	Cost note for RP2040.  Cortex-M0+ has no LDREX/STREX, so each atomic
 *	below is a hardware-spinlock claim and an interrupt-disable window, not
 *	a single instruction.  They are correct, but they are not cheap, and
 *	code ported from an architecture with exclusives should not assume
 *	otherwise.
 */

#ifndef __TK_SMP_H__
#define __TK_SMP_H__

#include <tk/typedef.h>
#include <tk/errno.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef volatile UW T_SPLOCK;

/* Lock words and atomic operands must be 32-bit aligned. */

IMPORT ER InitSpinLock( T_SPLOCK *lock );
IMPORT ER SpinLock( T_SPLOCK *lock );
IMPORT ER SpinTryLock( T_SPLOCK *lock );
IMPORT ER SpinUnlock( T_SPLOCK *lock );

/* Interrupt-preserving variants.  Required when a lock is taken both in task
   context and by a handler on the same core; otherwise the handler spins
   against the task it interrupted and neither proceeds. */
IMPORT ER ISpinLock( T_SPLOCK *lock, UINT *intsts );
IMPORT ER ISpinTryLock( T_SPLOCK *lock, UINT *intsts );
IMPORT ER ISpinUnlock( T_SPLOCK *lock, UINT intsts );

IMPORT UW atomic_inc( UW *addr );
IMPORT UW atomic_dec( UW *addr );
IMPORT UW atomic_add( UW *addr, UW val );
IMPORT UW atomic_sub( UW *addr, UW val );
IMPORT UW atomic_xchg( UW *addr, UW val );
IMPORT UW atomic_cmpxchg( UW *addr, UW val, UW cmp );
IMPORT UW atomic_bitset( UW *addr, UW setptn );
IMPORT UW atomic_bitclr( UW *addr, UW clrptn );

/* The spelling is retained from the public SMP T-Kernel specification. */
IMPORT void mp_memory_barrir( void );

#ifdef __cplusplus
}
#endif
#endif /* __TK_SMP_H__ */
