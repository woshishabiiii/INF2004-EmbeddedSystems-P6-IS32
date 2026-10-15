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
 *	cpu_cntl.c (ARMv6-M)
 *	CPU-Dependent Control
 */
#define KNL_SMP_NO_SHORT_NAMES
#include "kernel.h"
#include "smp_lock.h"
#include "../../../sysdepend.h"

#include "cpu_task.h"


/* Temporal stack used when 'dispatch_to_schedtsk' is called.
   One per core: two processors can be between contexts at the same time, so
   they cannot share a scratch stack. */
Noinit(EXPORT UB knl_tmp_stack[TK_MAX_CORE][TMP_STACK_SIZE]);

/*
 * Paint the temporary stacks and report how deep they are actually used.
 *
 * The dispatcher runs on knl_tmp_stack whenever a core has no current task --
 * during task exit, task start, and while idling -- and interrupts are enabled
 * there. Anything an interrupt does then runs on this stack, and in the SMP
 * profile that now includes the whole global ready-queue assignment. This
 * turns "is 256 bytes still enough" from a guess into a measurement.
 */
#define TMP_STACK_PAINT		0xa5a5a5a5UL

EXPORT void knl_tmp_stack_paint( void )
{
	UINT	core;
	INT	i;

	for ( core = 0; core < TK_MAX_CORE; core++ ) {
		UW *p = (UW *)&knl_tmp_stack[core][0];
		for ( i = 0; i < (INT)(TMP_STACK_SIZE / sizeof(UW)); i++ ) {
			p[i] = TMP_STACK_PAINT;
		}
	}
}

EXPORT UW knl_tmp_stack_used( UINT core )
{
	UW	*p;
	INT	i;

	if ( core >= TK_MAX_CORE ) return 0;
	p = (UW *)&knl_tmp_stack[core][0];
	/* The stack grows down from the top, so scan up from the bottom for
	   the first word that is still painted. */
	for ( i = 0; i < (INT)(TMP_STACK_SIZE / sizeof(UW)); i++ ) {
		if ( p[i] != TMP_STACK_PAINT ) break;
	}
	return TMP_STACK_SIZE - (UW)i * sizeof(UW);
}

/* Task independent status, one nesting count per core (declared in smp.h) */
EXPORT	W	knl_taskindp[TK_MAX_CORE] = { 0 };

/* ------------------------------------------------------------------------ */
/*
 * Set task register contents (Used in tk_set_reg())
 */
EXPORT void knl_set_reg( TCB *tcb, CONST T_REGS *regs, CONST T_EIT *eit, CONST T_CREGS *cregs )
{
	SStackFrame	*ssp;
	INT	i;

#if USE_FPU
	UW		*tmpp;

	tmpp = (UW*)(( cregs != NULL )? cregs->ssp: tcb->tskctxb.ssp);
	if(tcb->tskatr & TA_FPU) {
		if(*tmpp & EXPRN_NO_FPU) {	/* FPU register is not saved */
			ssp = (SStackFrame*)tmpp;
		} else {		/* FPU register is saved */
			ssp = (SStackFrame*)(((SStackFrame_wFPU*)tmpp)->r_);
		}
	} else {
		ssp = (SStackFrame*)tmpp;
	}
#else
	ssp = (SStackFrame*)(( cregs != NULL )? cregs->ssp: tcb->tskctxb.ssp);
#endif
	
	if ( regs != NULL ) {
		for ( i = 0; i < 4; ++i ) {
			ssp->r[i] = regs->r[i];
		}
		for ( i = 4; i < 12; ++i){
			ssp->r_[i - 4] = regs->r[i];
		}
	}

	if ( eit != NULL ) {
		ssp->pc = eit->pc;
	}

	if ( cregs != NULL ) {
		tcb->tskctxb.ssp  = cregs->ssp;
	}
}


/* ------------------------------------------------------------------------ */
/*
 * Get task register contents (Used in tk_get_reg())
 */
EXPORT void knl_get_reg( TCB *tcb, T_REGS *regs, T_EIT *eit, T_CREGS *cregs )
{
	SStackFrame	*ssp;
	INT		i;

#if USE_FPU
	UW		*tmpp;

	tmpp = (UW*)tcb->tskctxb.ssp;
	if(tcb->tskatr & TA_FPU) {
		if(*tmpp & EXPRN_NO_FPU) {	/* FPU register is not saved */
			ssp = (SStackFrame*)tmpp;
		} else {		/* FPU register is saved */
			ssp = (SStackFrame*)&(((SStackFrame_wFPU*)tmpp)->exp_ret);
		}
	} else {
		ssp = (SStackFrame*)tmpp;
	}
#else
	ssp = (SStackFrame*)tcb->tskctxb.ssp;
#endif

	if ( regs != NULL ) {
		for ( i = 0; i < 4; ++i ) {
			regs->r[i] = ssp->r[i];
		}
		for ( i = 4; i < 12; ++i ){
			regs->r[i] = ssp->r_[i - 4];
		}
		regs->r[12] = ssp->ip;
		regs->lr = ssp->lr;
	}

	if ( eit != NULL ) {
		eit->pc       = ssp->pc;
		eit->taskmode = 0;
	}

	if ( cregs != NULL ) {
		cregs->ssp   = tcb->tskctxb.ssp;
	}
}

/* ----------------------------------------------------------------------- */
/*
 *	Task dispatcher startup
 */
EXPORT void knl_force_dispatch( void )
{
#if TK_SUPPORT_SMP
	/*
	 * This function does not return: it pends PendSV and enables
	 * interrupts, and the dispatcher discards the caller's context. Callers
	 * such as tk_ext_tsk() invoke it from inside a critical section, so the
	 * matching END_CRITICAL_SECTION never runs and the kernel lock would be
	 * held by a context that no longer exists. Release it here, where the
	 * abandonment actually happens, rather than relying on every caller to
	 * remember.
	 */
	knl_kernel_lock_abandon();
#endif
	*knl_dispatch_disabled_slot() = DDS_DISABLE_IMPLICIT;
#if !TK_SUPPORT_SMP
	*knl_ctxtsk_slot() = NULL;
#endif
	*(_UW*)SCB_ICSR = ICSR_PENDSVSET;	/* pendsv exception */
	set_primask(0);				/* Enable interrupt */
}

EXPORT void knl_dispatch( void )
{
	*(_UW*)SCB_ICSR = ICSR_PENDSVSET;	/* pendsv exception */
}

#endif /* CPU_CORE_ARMV6M */
