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
 *	exc_hdr.c (ARMv6-M)
 *	Exception handler
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <kernel.h>
#include "../../../sysdepend.h"

#if (USE_EXCEPTION_DBG_MSG && USE_TMONITOR)
	#define EXCEPTION_DBG_MSG(a)	tm_putstring((UB*)a)
#else
	#define EXCEPTION_DBG_MSG(a)
#endif

/*
 * NMI handler
 */
WEAK_FUNC EXPORT void NMI_Handler(void)
{
	EXCEPTION_DBG_MSG("NMI\n");
	while(1);
}

/*
 * Hard fault handler
 */
/*
 * Hard fault reporting.
 *
 * Three separate hypotheses about a fault in this port were wrong, each
 * costing a flash. The exception frame holds the address that faulted, so
 * capture it instead of guessing: a naked entry stub records the frame
 * pointer and tail-calls the reporter.
 *
 * ARMv6-M stacks r0-r3, r12, lr, pc, xpsr. Bit 2 of EXC_RETURN selects which
 * stack was in use, and this kernel runs everything on MSP, but both are
 * handled so the report stays correct if that ever changes.
 */
EXPORT volatile UW *knl_fault_frame;

IMPORT void knl_hard_fault_report(void);

__asm__(
"	.syntax unified			\n"
"	.thumb				\n"
"	.text				\n"
"	.align	2			\n"
"	.globl	HardFault_Handler	\n"
"	.thumb_func			\n"
"HardFault_Handler:			\n"
"	mov	r0, lr			\n"
"	movs	r1, #4			\n"
"	tst	r0, r1			\n"
"	beq	1f			\n"
"	mrs	r0, psp			\n"
"	b	2f			\n"
"1:	mrs	r0, msp			\n"
"2:	ldr	r1, =knl_fault_frame	\n"
"	str	r0, [r1]		\n"
"	b	knl_hard_fault_report	\n"
"	.ltorg				\n"
);

EXPORT void knl_hard_fault_report(void)
{
#if (USE_EXCEPTION_DBG_MSG  && USE_TMONITOR)

	ID		ctskid;
	volatile UW	*f = knl_fault_frame;

	if(knl_ctxtsk != NULL) {
		ctskid = knl_ctxtsk->tskid;
	} else {
		ctskid = 0;
	}
	/* Report the processor too: a faulting core halts below while the other
	   keeps running, so without it the log reads as if nothing happened. */
	tm_printf((UB*)"*** Hard fault ***  prc:%d ctxtsk:%d\n",
		  (INT)knl_current_core() + 1, ctskid);
#if TK_SUPPORT_SMP
	/*
	 * Both cores' scheduler slots, so the report distinguishes "the
	 * scheduler chose this" from "something wrote this". The affinity guard
	 * reports zero violations, yet the current task here is one pinned to
	 * the other core, and only these values separate the two explanations.
	 */
	{
		TCB *c0 = *knl_ctxtsk_core_slot(0);
		TCB *c1 = *knl_ctxtsk_core_slot(1);
		TCB *s0 = *knl_schedtsk_core_slot(0);
		TCB *s1 = *knl_schedtsk_core_slot(1);
		IMPORT BOOL knl_guards_intact(void);
		IMPORT UW knl_smp_last_pub[];
		IMPORT UW knl_smp_pub_count[];
		tm_printf((UB*)"    scheduler slot guards %s\n",
			  knl_guards_intact()? "intact": "SMASHED");
		tm_printf((UB*)"    ctxtsk[0]=%d ctxtsk[1]=%d schedtsk[0]=%d schedtsk[1]=%d\n",
			  (c0 == NULL)? 0: c0->tskid, (c1 == NULL)? 0: c1->tskid,
			  (s0 == NULL)? 0: s0->tskid, (s1 == NULL)? 0: s1->tskid);
		tm_printf((UB*)"    last publish: core0=%u core1=%u (counts %u/%u)\n",
			  knl_smp_last_pub[0], knl_smp_last_pub[1],
			  knl_smp_pub_count[0], knl_smp_pub_count[1]);
	}
#endif
	if(f != NULL) {
		tm_printf((UB*)"    pc=%08x lr=%08x psr=%08x\n", f[6], f[5], f[7]);
		tm_printf((UB*)"    r0=%08x r1=%08x r2=%08x r3=%08x r12=%08x\n",
			  f[0], f[1], f[2], f[3], f[4]);
	}
#endif
	while(1);
}

/*
 * Svcall
 */
WEAK_FUNC EXPORT void Svcall_Handler(void)
{
	EXCEPTION_DBG_MSG("SVCall\n");
	while(1);
}

/*
 * Default Handler
 */
WEAK_FUNC EXPORT void Default_Handler(void)
{
#if (USE_EXCEPTION_DBG_MSG  && USE_TMONITOR)
	INT	i;
	_UW	*icpr;

	icpr = (_UW*)NVIC_ICPR_BASE;

	EXCEPTION_DBG_MSG("Undefine Exceptio ICPR: ");
	for(i=0; i < 8; i++) {
		tm_printf((UB*)"%x ", *icpr++);
	}
	EXCEPTION_DBG_MSG("\n");
#endif
	while(1);
}

#endif /* CPU_CORE_ARMV6M */
