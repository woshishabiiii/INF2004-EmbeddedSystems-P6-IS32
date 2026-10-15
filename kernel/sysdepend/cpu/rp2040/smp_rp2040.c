/*
 *	smp_rp2040.c
 *	RP2040 dual-core SMP hardware layer (Phase 3: bring-up and IPI).
 */

#include <sys/machine.h>
#ifdef CPU_RP2040

#define KNL_SMP_NO_SHORT_NAMES
#include "kernel.h"
#include "../../sysdepend.h"
#include "smp_rp2040.h"
#include "smp_lock.h"
#include <tk/smp.h>

#if TK_SUPPORT_SMP

/* ------------------------------------------------------------------------ */
/*
 * SIO registers (RP2040 datasheet 2.3.1).  The SIO block is core-local for
 * CPUID and the FIFOs: the same addresses reach a different core's mailbox
 * depending on which core executes the access.
 */
#define SIO_BASE_ADDR	0xd0000000
#define SIO_CPUID	(*(_UW*)(SIO_BASE_ADDR + 0x000))
#define SIO_FIFO_ST	(*(_UW*)(SIO_BASE_ADDR + 0x050))
#define SIO_FIFO_WR	(*(_UW*)(SIO_BASE_ADDR + 0x054))
#define SIO_FIFO_RD	(*(_UW*)(SIO_BASE_ADDR + 0x058))

#define FIFO_ST_VLD	(1U << 0)	/* RX FIFO holds data */
#define FIFO_ST_RDY	(1U << 1)	/* TX FIFO has room */
#define FIFO_ST_WOF	(1U << 2)	/* sticky: wrote a full TX FIFO */
#define FIFO_ST_ROE	(1U << 3)	/* sticky: read an empty RX FIFO */

/*
 * The IPI must not be able to preempt a context switch.
 *
 * The NVIC defaults every external interrupt to priority 0, the highest,
 * while PendSV runs at 3. The dispatcher therefore ran with the FIFO
 * interrupt able to interrupt it, including in the window where a context is
 * half saved -- the outgoing task's stack pointer published but ctxtsk not yet
 * cleared. The handler then executes on the dispatcher's own stack, and its
 * return address is what every fault dump has shown sitting where a saved
 * EXC_RETURN belongs.
 *
 * FreeRTOS's RP2040 SMP port sets this interrupt to the lowest priority,
 * equal to PendSV, for the same reason. Match it.
 */
#define IRQ_PRIORITY_LOWEST	3

LOCAL void knl_set_irq_priority( UINT irq, UINT prio )
{
	UW	reg = NVIC_IPR((irq / 4) * 4);
	UINT	shift = (irq % 4) * 8;
	UW	v;

	v = in_w(reg);
	v &= ~(0xffUL << shift);
	v |= ((prio & 0x3UL) << 6) << shift;
	out_w(reg, v);
}

/* Architecture atomics (smp_atomic.c), declared before first use. */
IMPORT UW knl_arch_atomic_bitset( volatile UW *addr, UW mask );
IMPORT UW knl_arch_atomic_fetch_add( volatile UW *addr, UW value );
IMPORT UW knl_arch_atomic_take( volatile UW *addr );

/* The SIO FIFO interrupt is per core: core 0 takes IRQ 15, core 1 IRQ 16. */
#define SIO_IRQ_PROC0	15
#define SIO_IRQ_PROC1	16

/* ------------------------------------------------------------------------ */

EXPORT volatile UW knl_smp_core1_ready;
EXPORT volatile UW knl_smp_core1_heartbeat;
EXPORT volatile UW knl_smp_core1_coreid = 0xffffffffU;
EXPORT volatile UW knl_smp_ping_recv;
EXPORT volatile UW knl_smp_pong_recv;
EXPORT volatile UW knl_smp_fifo_errors;
EXPORT volatile UW knl_smp_fifo_wof[TK_MAX_CORE];
EXPORT volatile UW knl_smp_fifo_roe[TK_MAX_CORE];
EXPORT volatile UW knl_smp_shared_word;
EXPORT volatile UW knl_smp_launch_ok;

/*
 * Which processors the scheduler may place work on.
 *
 * Core 0 is online from reset; core 1 only once it has completed the bootrom
 * handshake and reached the dispatcher.  Without this the global assignment
 * happily hands tasks to a processor that is not running them, and the system
 * goes quiet with no fault of any kind -- the console task in particular is
 * simply never executed.  It also keeps reschedules from writing IPI
 * doorbells into the FIFO while the launch handshake is using it.
 */
EXPORT volatile UW knl_smp_online_mask = 1;

/* Handshake progress, so a failed launch says where it stopped rather than
   only that it did: highest sequence index reached, attempts consumed, and
   the last word core 1 echoed back. */
EXPORT volatile UW knl_smp_launch_seq;
EXPORT volatile UW knl_smp_launch_attempts;
EXPORT volatile UW knl_smp_launch_echo = 0xffffffffU;

/*
 * Work channel for core 1.
 *
 * The concurrency tests are meaningless unless both cores actually contend,
 * so core 0 posts a command and core 1 executes it in its park loop.  Command
 * and result are single-writer words in shared SRAM: core 0 writes cmd, core
 * 1 writes done.  No lock is needed for the handshake itself -- it is the
 * work the command performs that is under test.
 */
#define KNL_SMP_CMD_NONE	0
#define KNL_SMP_CMD_ATOMIC	1	/* atomic_inc the shared counter N times */
#define KNL_SMP_CMD_BKL		2	/* ++ a plain counter under the BKL N times */
#define KNL_SMP_CMD_SOAK	3	/* hold/release the BKL until told to stop */

EXPORT volatile UW knl_smp_cmd;
EXPORT volatile UW knl_smp_cmd_arg;
EXPORT volatile UW knl_smp_cmd_done;
EXPORT volatile UW knl_smp_atomic_counter;
EXPORT volatile UW knl_smp_plain_counter;
EXPORT volatile UW knl_smp_soak_stop;
EXPORT volatile UW knl_smp_soak_iters;

/* ------------------------------------------------------------------------ */
/*
 * Memory ordering.
 *
 * Cortex-M0+ is single-issue and in-order and the RP2040 bus fabric does not
 * reorder, so DMB is nearly free here; it is written explicitly anyway so the
 * intended ordering contract is visible rather than implied by the core's
 * simplicity.
 */
Inline void knl_smp_barrier( void )
{
	__asm__ volatile ("dmb" ::: "memory");
}

Inline void knl_smp_sev( void )
{
	__asm__ volatile ("sev" ::: "memory");
}

Inline void knl_smp_wfe( void )
{
	__asm__ volatile ("wfe" ::: "memory");
}

/* ------------------------------------------------------------------------ */
/*
 * FIFO primitives.  Neither waits without a bound on the other core beyond
 * the bootrom handshake, which is inherently a rendezvous.
 */

LOCAL void fifo_drain( void )
{
	while ( (SIO_FIFO_ST & FIFO_ST_VLD) != 0 ) {
		(void)SIO_FIFO_RD;
	}
	SIO_FIFO_ST = 0xffU;		/* clear sticky ROE/WOF */
}

/*
 * Record which sticky flag was seen and on which core.
 *
 * The Phase-4 run reported one FIFO error with no way to tell what it was:
 * a single counter, incremented non-atomically from both cores, in a phase
 * about atomicity.  Split it by flag and by core, and count with a real
 * atomic, so the next occurrence identifies itself.
 */
LOCAL void fifo_check_errors( void )
{
	UW	st = SIO_FIFO_ST;
	UINT	self = knl_current_core();

	if ( (st & (FIFO_ST_WOF | FIFO_ST_ROE)) == 0 ) return;

	if ( (st & FIFO_ST_WOF) != 0 ) {
		(void)knl_arch_atomic_fetch_add(&knl_smp_fifo_wof[self], 1);
	}
	if ( (st & FIFO_ST_ROE) != 0 ) {
		(void)knl_arch_atomic_fetch_add(&knl_smp_fifo_roe[self], 1);
	}
	(void)knl_arch_atomic_fetch_add(&knl_smp_fifo_errors, 1);

	SIO_FIFO_ST = 0xffU;		/* write any value clears them */
}

LOCAL BOOL fifo_push( UW value )
{
	INT spin;

	for ( spin = 0; spin < 1000000; spin++ ) {
		if ( (SIO_FIFO_ST & FIFO_ST_RDY) != 0 ) {
			SIO_FIFO_WR = value;
			knl_smp_barrier();
			knl_smp_sev();	/* wake a core parked in WFE */
			return TRUE;
		}
	}
	knl_smp_fifo_errors++;
	return FALSE;			/* peer not draining; do not hang */
}

LOCAL BOOL fifo_pop( UW *value )
{
	if ( (SIO_FIFO_ST & FIFO_ST_VLD) == 0 ) return FALSE;
	*value = SIO_FIFO_RD;
	return TRUE;
}

/* ------------------------------------------------------------------------ */
/*
 * IPI
 *
 * Reasons are coalesced into a per-target pending mask and the FIFO is used
 * only as a doorbell.  Phase 3 pushed reason words through the FIFO itself,
 * which meant a burst deeper than its eight entries was dropped; merging into
 * a mask instead makes delivery independent of FIFO depth, and repeated
 * requests for the same reason cost nothing.
 *
 * The mask is set with an atomic bit-set and consumed with an atomic
 * take-and-clear, so a reason raised between a receiver's read and its clear
 * is not lost.
 */

LOCAL volatile UW knl_ipi_pending[TK_MAX_CORE];

EXPORT void knl_smp_time_handler_request( UINT core )
{
	knl_ipi_send(core, KNL_IPI_TIME_HANDLER);
}

EXPORT void knl_ipi_send( UINT core, UW reasons )
{
	if ( core >= TK_MAX_CORE ) return;

	(void)knl_arch_atomic_bitset(&knl_ipi_pending[core], reasons);

	/*
	 * Doorbell.  Push only if there is room, and never spin: a full FIFO
	 * already means the receiver has an unread doorbell pending, and the
	 * reasons live in the mask regardless.  Spinning here would let a
	 * receiver that is merely slow stall the sender.
	 */
	if ( (SIO_FIFO_ST & FIFO_ST_RDY) != 0 ) {
		SIO_FIFO_WR = 0;
	}
	knl_smp_barrier();
	knl_smp_sev();
}

EXPORT UW knl_ipi_take( void )
{
	UW	value;

	/* Drain the doorbell so the interrupt deasserts, then take the mask. */
	while ( fifo_pop(&value) ) {
		/* payload unused */
	}
	fifo_check_errors();

	return knl_arch_atomic_take(&knl_ipi_pending[knl_current_core()]);
}

/* ------------------------------------------------------------------------ */
/*
 * Core 0's IPI handler: services PONG replies from core 1.
 */
LOCAL void knl_smp_core0_ipi( UINT intno )
{
	UW	reasons;

	(void)intno;

	reasons = knl_ipi_take();
	if ( (reasons & KNL_IPI_PONG) != 0 ) {
		knl_smp_pong_recv++;
	}
	if ( (reasons & KNL_IPI_RESCHEDULE) != 0 ) {
		knl_dispatch();
	}
	if ( (reasons & KNL_IPI_TIME_HANDLER) != 0 ) {
		knl_timehandler_ipi();
	}
}

/*
 * Core 1's IPI handler.
 *
 * PING service lives here rather than in a task, because core 1's park loop
 * is gone: it now runs the dispatcher, and whatever task it runs must not be
 * required to poll the IPI for the transport to work.
 *
 * RESCHEDULE raises PendSV on this core.  PendSV is a per-core exception, so
 * the other core cannot pend it directly -- the IPI exists precisely to turn
 * a remote request into a local one.
 */
LOCAL void knl_smp_core1_ipi( UINT intno )
{
	UW	reasons;

	(void)intno;

	reasons = knl_ipi_take();

	if ( (reasons & KNL_IPI_PING) != 0 ) {
		knl_smp_ping_recv++;
		knl_smp_shared_word = knl_smp_ping_recv;
		knl_smp_barrier();
		knl_ipi_send(0, KNL_IPI_PONG);
	}
	if ( (reasons & KNL_IPI_RESCHEDULE) != 0 ) {
		knl_dispatch();		/* pend PendSV on this core */
	}
	if ( (reasons & KNL_IPI_TIME_HANDLER) != 0 ) {
		knl_timehandler_ipi();
	}
}

EXPORT void knl_smp_core1_init( void )
{
	T_DINT	dint;

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)knl_smp_core1_ipi;
	(void)tk_def_int(SIO_IRQ_PROC1, &dint);

	knl_set_irq_priority(SIO_IRQ_PROC1, IRQ_PRIORITY_LOWEST);
	out_w(NVIC_ICPR(SIO_IRQ_PROC1), 1U << (SIO_IRQ_PROC1 % 32));
	out_w(NVIC_ISER(SIO_IRQ_PROC1), 1U << (SIO_IRQ_PROC1 % 32));
}

EXPORT void knl_smp_core0_init( void )
{
	T_DINT	dint;

	fifo_drain();

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)knl_smp_core0_ipi;
	(void)tk_def_int(SIO_IRQ_PROC0, &dint);

	/* NVIC_ISER/ICPR are macros over the armv6m register block; the NVIC is
	   per core, so this unmasks the FIFO interrupt on the calling core. */
	knl_set_irq_priority(SIO_IRQ_PROC0, IRQ_PRIORITY_LOWEST);
	out_w(NVIC_ICPR(SIO_IRQ_PROC0), 1U << (SIO_IRQ_PROC0 % 32));
	out_w(NVIC_ISER(SIO_IRQ_PROC0), 1U << (SIO_IRQ_PROC0 % 32));
}

/* ------------------------------------------------------------------------ */
/*
 * Core 1.
 *
 * Runs with its own stack in SRAM4/5 and its own NVIC, sharing the RAM vector
 * table with core 0 -- the table holds addresses of code, which is identical
 * for both cores; what differs is which interrupts each core unmasks.
 *
 * It deliberately touches no kernel state.  It polls its FIFO rather than
 * taking the SIO interrupt, so that in this phase core 1 never enters a
 * handler and never reaches any kernel code path.
 */
IMPORT const void (*vector_tbl[])();

LOCAL void knl_smp_core1_entry( void )
{
	/* Vector table and exception priorities: per core, and core 1 comes out
	   of the bootrom with neither configured. */
	*(_UW*)SCB_VTOR = (UW)exchdr_tbl;
	*(_UW*)SCB_SHPR2 = SCB_SHPR2_VAL;
	*(_UW*)SCB_SHPR3 = SCB_SHPR3_VAL;
	/* SysTick is a private peripheral on each Cortex-M0+.  Core 0 is the
	   sole timekeeper, so make core 1's disabled state explicit. */
	out_w(SYST_CSR, 0);
	out_w(SCB_ICSR, ICSR_PENDSTCLR);

	/*
	 * Clear this core's sticky FIFO flags before any error accounting
	 * starts.  Core 1 arrives from the bootrom, whose mailbox wait loop
	 * reads the FIFO and leaves ROE set; those flags describe the bootrom's
	 * activity, not the kernel's.
	 */
	fifo_drain();

	/* First point at which knl_current_core() returns anything but 0. */
	knl_smp_core1_coreid = knl_current_core();

	/* IPI receiver, so a reschedule request from core 0 can reach this
	   core's PendSV.  Registering it takes the kernel lock, which is safe:
	   core 0 finished kernel initialisation before launching us. */
	knl_smp_core1_init();

	knl_smp_barrier();
	knl_smp_core1_ready = 1;
	knl_smp_barrier();

	/*
	 * Enter the dispatcher.  This does not return: it pends PendSV and
	 * enables interrupts, and the dispatcher either switches to whatever
	 * task has been placed in this core's schedtsk slot or idles waiting
	 * for one.  Core 1 is a scheduled processor from here on, not a
	 * bespoke loop.
	 */
	knl_force_dispatch();

	for ( ;; ) {
		/* unreachable */
	}
}

/*
 * Bootrom launch handshake (datasheet 2.8.2 "Launching code on processor
 * core 1").  Core 1 sits in the bootrom waiting on WFE; this sends the six
 * word sequence 0, 0, 1, VTOR, SP, entry.  Any mismatch in the echo restarts
 * the sequence, which is how the bootrom resynchronises if core 1 was left in
 * an unknown state.
 */
EXPORT BOOL knl_smp_launch_core1( void )
{
	UW	cmd[6];
	UW	value;
	INT	seq, attempts;

	cmd[0] = 0;
	cmd[1] = 0;
	cmd[2] = 1;
	cmd[3] = (UW)exchdr_tbl;			/* VTOR */
	cmd[4] = (UW)KNL_CORE1_STACK_TOP;		/* initial SP */
	cmd[5] = (UW)knl_smp_core1_entry | 1U;		/* Thumb entry */

	/*
	 * The handshake is a rendezvous: core 1's bootrom echoes each word back
	 * into this core's RX FIFO, and knl_smp_launch_core1() must read those
	 * echoes itself.  If the SIO FIFO interrupt is enabled, its handler
	 * drains the FIFO first and every word appears unacknowledged, so the
	 * sequence never completes.  Mask it here rather than relying on the
	 * caller ordering it correctly.
	 */
	out_w(NVIC_ICER(SIO_IRQ_PROC0), 1U << (SIO_IRQ_PROC0 % 32));

	seq = 0;
	attempts = 0;
	while ( seq < 6 ) {
		if ( ++attempts > 100 ) return FALSE;

		if ( cmd[seq] == 0 ) {
			fifo_drain();
			knl_smp_sev();
		}

		if ( !fifo_push(cmd[seq]) ) return FALSE;

		/* Bounded wait for the echo: the bootrom replies promptly, and
		   hanging here would take the whole system down. */
		{
			INT spin;
			BOOL got = FALSE;
			for ( spin = 0; spin < 1000000 && !got; spin++ ) {
				got = fifo_pop(&value);
			}
			if ( !got ) return FALSE;
		}

		knl_smp_launch_echo = value;
		seq = ( cmd[seq] == value )? seq + 1: 0;
		if ( (UW)seq > knl_smp_launch_seq ) knl_smp_launch_seq = (UW)seq;
		knl_smp_launch_attempts = (UW)attempts;
	}

	/* Wait for core 1 to publish its ready flag, then let the scheduler
	   start using it.  Publishing this only on success means a failed
	   launch degrades to single-core operation rather than to silence. */
	{
		INT spin;
		for ( spin = 0; spin < 10000000; spin++ ) {
			knl_smp_barrier();
			if ( knl_smp_core1_ready != 0 ) {
				knl_smp_online_mask = ALL_CORE_BIT;
				knl_smp_barrier();
				return TRUE;
			}
		}
	}
	return FALSE;
}

#endif /* TK_SUPPORT_SMP */
#endif /* CPU_RP2040 */
