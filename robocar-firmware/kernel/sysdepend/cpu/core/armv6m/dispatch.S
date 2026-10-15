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

#include <sys/machine.h>
#ifdef CPU_CORE_ARMV6M
/*
 *	dispatch.S (ARMv6-M)
 *	Dispatcher
 */

#define	_in_asm_source_

#include <sys/machine.h>
#include <tk/errno.h>
#include <sys/sysdef.h>
#include <sys/knldef.h>

#include "offset.h"

/*
 * Load the address of this core's slot of a per-core array.
 *
 *	SMP_ADDR reg, symbol, tmp
 *
 * 'tmp' must be one of R0-R3.  Those are caller-saved and are already in the
 * exception frame, so clobbering them cannot damage the outgoing task's
 * context -- unlike R4-R11, which this routine saves and restores by hand.
 *
 * SIO CPUID is core-local and constant per core, so it is simply re-read at
 * each use rather than being held in a register across the context push.
 *
 * In the single-core profile this collapses to a plain address load.
 */
	.macro	SMP_ADDR reg, sym, tmp
	ldr	\reg, =Csym(\sym)
#if TK_SUPPORT_SMP
	ldr	\tmp, =SIO_CPUID_ADDR
	ldr	\tmp, [\tmp]			// core id
	lsls	\tmp, \tmp, #2			// * sizeof(void *)
	add	\reg, \reg, \tmp
#endif
	.endm

	.code 16
	.syntax unified
	.thumb

/* ------------------------------------------------------------------------ */
/*
 * Dispatcher
 *
 *	Contexts to save
 *	Save registers except for ssp to a stack. Save 'ssp' to TCB.
 *
 *	   High Address +---------------+
 *			| xpsr		|
 *			| pc		| Return address
 *			| lr		|
 *			| R12		|
 *			| R0-R3		|
 *			+---------------+ Save by Exception entry process.
 *			| R4 - R11	|
 *		ssp ->	| lr		|
 *	    Low Address +---------------+
 *
 */

	.text
	.align 2
	.thumb
	.thumb_func
	.globl Csym(knl_dispatch_entry)

Csym(knl_dispatch_entry):	
/*----------------- Start dispatch processing. -----------------*/
/*
 * knl_dispatch_disabled[], knl_ctxtsk[], knl_schedtsk[] and knl_tmp_stack[]
 * are per-core arrays, indexed by SIO CPUID.  Only R0-R3 are used to compute
 * that index: they are caller-saved and already in the exception frame, so
 * clobbering them is harmless, whereas R4-R11 belong to the outgoing task and
 * are saved by hand below.
 */
	SMP_ADDR r0, knl_dispatch_disabled, r2
	ldr	r1, =1
	str	r1, [r0]			// Dispatch disable

	SMP_ADDR r0, knl_ctxtsk, r2
	ldr	r1, [r0]			// R1 = ctxtsk
	cmp	r1, #0
	bne	l_dispatch_000

	ldr	r2, =(Csym(knl_tmp_stack) + TMP_STACK_SIZE)	// Set temporal stack
#if TK_SUPPORT_SMP
	ldr	r3, =SIO_CPUID_ADDR
	ldr	r3, [r3]			// R3 = core id
	ldr	r1, =TMP_STACK_SIZE
	muls	r3, r1, r3			// R3 = core id * TMP_STACK_SIZE
	add	r2, r2, r3			// this core's temporal stack
#endif
	mov	sp, r2
	b	l_dispatch_100

/*----------------- Save "ctxtsk" context. -----------------*/
l_dispatch_000: 
	push	{r4-r7}
	mov	r0, r8
	mov	r1, r9
	mov	r2, r10
	mov	r3, r11
	push	{r0-r3}
	push	{lr}

	SMP_ADDR r0, knl_ctxtsk, r2
	ldr	r1, [r0]			// R1 = ctxtsk
	mov	r2, sp
	str	r2, [r1, #TCB_tskctxb + CTXB_ssp]	// Save 'ssp' to TCB

#if TK_SUPPORT_SMP
	/* A task remains published in knl_ctxtsk[] until its complete context has
	 * been saved.  This is the ownership-release point used by remote task
	 * management, matching the RP2040 FreeRTOS SMP run-state handoff. */
	mov	r4, r1			// R4 = outgoing TCB (already saved)
#endif
	ldr	r2, =0
	str	r2, [r0]			// ctxtsk = NULL
#if TK_SUPPORT_SMP
	dmb					// publish saved context before release

	/* Reclamation must run after leaving the outgoing task's stack. */
	ldr	r2, =(Csym(knl_tmp_stack) + TMP_STACK_SIZE)
	ldr	r3, =SIO_CPUID_ADDR
	ldr	r3, [r3]			// R3 = core id
	ldr	r1, =TMP_STACK_SIZE
	muls	r3, r1, r3
	add	r2, r2, r3
	mov	sp, r2
	mov	r0, r4
	bl	Csym(knl_smp_dispatch_release)
#endif


/*----------------- Dispatch from "ctxtsk" to "schedtsk" -----------------*/
l_dispatch_100:
	SMP_ADDR r5, knl_schedtsk, r2		// R5 = &schedtsk[core]
	ldr	r2, =1				// Disable interruput
	msr	primask, r2

	ldr	r7, [r5]			// R7 = schedtsk
	cmp	r7, #0
	bne	l_dispatch_130

	/* No public task is runnable. Restore this processor's private idle
	 * context and exception-return to its Thread-mode power-saving loop. */
	SMP_ADDR r0, knl_idletsk, r2
	ldr	r7, [r0]

l_dispatch_130:					// Switch to 'schedtsk'
	/* Calls made while waiting for a runnable task may clobber R0. */
	SMP_ADDR r0, knl_ctxtsk, r2
	str	r7, [r0]			// ctxtsk = schedtsk
	ldr	r2, [r7, #TCB_tskctxb + CTXB_ssp]	// Restore 'ssp' from TCB
	mov	sp, r2

/*----------------- Restore "schedtsk" context. -----------------*/

	pop	{r0}
	mov	lr, r0
	pop	{r0-r3}
	mov	r11, r3
	mov	r10, r2
	mov	r9, r1
	mov	r8, r0
	pop	{r4-r7}

	SMP_ADDR r0, knl_dispatch_disabled, r2
	ldr	r1, =0
	str	r1, [r0]			// Dispatch enable

	msr	primask, r1			// Enable inperrupt

	bx	lr

#endif /* CPU_CORE_ARMV6M */
