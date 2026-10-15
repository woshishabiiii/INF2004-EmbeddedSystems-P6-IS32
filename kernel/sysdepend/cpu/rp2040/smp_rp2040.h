/*
 *	smp_rp2040.h
 *	RP2040 dual-core SMP hardware layer.
 *
 *	Phase 3 scope: bring core 1 up and prove shared memory and an
 *	inter-processor interrupt path, WITHOUT letting core 1 touch any kernel
 *	state.  Core 1 parks in a loop that services only IPIs.  The scheduler
 *	arrives in Phase 6, after the kernel lock exists in Phase 4; until then
 *	core 1 executing kernel code would be unsafe, because shared kernel
 *	state is still protected only by local interrupt masking.
 *
 *	Compiled only in the SMP profile.
 */

#ifndef _SYSDEPEND_CPU_SMP_RP2040_H_
#define _SYSDEPEND_CPU_SMP_RP2040_H_

#include <sys/sysdef.h>

#if TK_SUPPORT_SMP

/*
 * IPI reason bits.
 *
 * Phase 3 carries these as literal words through the SIO FIFO.  The FIFO is
 * only eight entries deep per direction and a full write is dropped with a
 * sticky error flag, so Phase 4 replaces this with a coalesced pending mask
 * per core -- once atomics exist -- and uses the FIFO purely as a doorbell.
 * RESCHEDULE is defined now but unused: core 1 has no scheduler yet.
 */
#define KNL_IPI_RESCHEDULE	(1U << 0)	/* pend a local reschedule (Phase 6) */
#define KNL_IPI_PING		(1U << 1)	/* bring-up: request a PONG */
#define KNL_IPI_PONG		(1U << 2)	/* bring-up: PING acknowledged */
#define KNL_IPI_TIME_HANDLER	(1U << 3)	/* run queued cyclic/alarm callbacks */

/* Core 1 stack.  SRAM4 and SRAM5 are two 4 KiB banks above the striped main
   SRAM, outside the linker's RAM region, so nothing else is placed there.
   The datasheet suggests exactly this use: keeping a core's stack out of the
   striped region reduces bank contention with the other core. */
#define KNL_CORE1_STACK_BASE	0x20040000
#define KNL_CORE1_STACK_TOP	0x20042000

/* Launch core 1.  Returns TRUE once it has acknowledged the bootrom
   handshake and published its ready flag.  Safe to call once. */
IMPORT BOOL knl_smp_launch_core1( void );

/* Per-core IPI receiver setup; each runs on the core it serves, because the
   NVIC is per core. */
IMPORT void knl_smp_core0_init( void );
IMPORT void knl_smp_core1_init( void );

/* Send reasons to a core, and drain what this core has been sent. */
IMPORT void knl_ipi_send( UINT core, UW reasons );
IMPORT UW   knl_ipi_take( void );

/*
 * Bring-up diagnostics, read by the Phase 3 harness on core 0.
 */
IMPORT volatile UW knl_smp_core1_ready;		/* core 1 reached its park loop */
IMPORT volatile UW knl_smp_core1_heartbeat;	/* advanced by that loop */
IMPORT volatile UW knl_smp_core1_coreid;	/* knl_current_core() as core 1 sees it */
IMPORT volatile UW knl_smp_ping_recv;		/* PINGs serviced by core 1 */
IMPORT volatile UW knl_smp_pong_recv;		/* PONGs serviced by core 0 */
IMPORT volatile UW knl_smp_fifo_errors;		/* sticky ROE/WOF observed */
IMPORT volatile UW knl_smp_fifo_wof[TK_MAX_CORE];	/* wrote a full TX FIFO */
IMPORT volatile UW knl_smp_fifo_roe[TK_MAX_CORE];	/* read an empty RX FIFO */
IMPORT volatile UW knl_smp_shared_word;		/* written by core 1, read by core 0 */
IMPORT volatile UW knl_smp_launch_ok;
IMPORT volatile UW knl_smp_online_mask;	/* processors the scheduler may use */
IMPORT volatile UW knl_smp_launch_seq;		/* handshake words acknowledged */
IMPORT volatile UW knl_smp_launch_attempts;
IMPORT volatile UW knl_smp_launch_echo;		/* last word core 1 echoed */

/* Work channel: core 0 posts a command, core 1 runs it in its park loop, so
   the concurrency tests actually contend rather than testing one core. */
#define KNL_SMP_CMD_NONE	0
#define KNL_SMP_CMD_ATOMIC	1
#define KNL_SMP_CMD_BKL		2
#define KNL_SMP_CMD_SOAK	3

IMPORT volatile UW knl_smp_cmd;
IMPORT volatile UW knl_smp_cmd_arg;
IMPORT volatile UW knl_smp_cmd_done;
IMPORT volatile UW knl_smp_atomic_counter;
IMPORT volatile UW knl_smp_plain_counter;
IMPORT volatile UW knl_smp_soak_stop;
IMPORT volatile UW knl_smp_soak_iters;		/* core 1 handshake succeeded */

#endif /* TK_SUPPORT_SMP */
#endif /* _SYSDEPEND_CPU_SMP_RP2040_H_ */
