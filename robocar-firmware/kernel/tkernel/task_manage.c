/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.00.06
 *
 *    Copyright (C) 2006-2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2022/10.
 *
 *----------------------------------------------------------------------
 */

/*
 *	task_manage.c
 *	Task Management Function
 */

#include "kernel.h"
#include "wait.h"
#include "check.h"
#include <tm/tmonitor.h>

#include "../sysdepend/cpu_task.h"

/*
 * Create task
 */
SYSCALL ID tk_cre_tsk( CONST T_CTSK *pk_ctsk )
{
#if CHK_RSATR
	const ATR VALID_TSKATR = {	/* Valid value of task attribute */
		 TA_HLNG
		|TA_RNG3
		|TA_USERBUF
		|TA_COPS
#if TK_SUPPORT_SMP
		|TA_ASSPRC
#endif
#if USE_OBJECT_NAME
		|TA_DSNAME
#endif
	};
#endif
	TCB	*tcb;
	W	sstksz;
	void	*stack;
	ER	ercd;

	CHECK_RSATR(pk_ctsk->tskatr, VALID_TSKATR);
#if !USE_IMALLOC
	/* TA_USERBUF must be specified if configured in no Imalloc */
	CHECK_PAR((pk_ctsk->tskatr & TA_USERBUF) != 0);
#endif
	CHECK_PAR(pk_ctsk->stksz >= 0);
	CHECK_PRI(pk_ctsk->itskpri);

	if ( (pk_ctsk->tskatr & TA_USERBUF) != 0 ) {
		/* Use user buffer */
		sstksz = pk_ctsk->stksz;
		CHECK_PAR(sstksz >= MIN_SYS_STACK_SIZE);
		stack = pk_ctsk->bufptr;
	} else {
#if USE_IMALLOC
		/* Allocate system stack area */
		sstksz = pk_ctsk->stksz + DEFAULT_SYS_STKSZ;
		sstksz  = (sstksz  + 7) / 8 * 8;	/* Align to a multiple of 8 */
		stack = knl_Imalloc((UW)sstksz);
		if ( stack == NULL ) {
			return E_NOMEM;
		}
#endif
	}

	BEGIN_CRITICAL_SECTION;
	/* Get control block from FreeQue */
	tcb = (TCB*)QueRemoveNext(&knl_free_tcb);
	if ( tcb == NULL ) {
		ercd = E_LIMIT;
		goto error_exit;
	}

	/* Initialize control block */
	tcb->exinf     = pk_ctsk->exinf;
	tcb->tskatr    = pk_ctsk->tskatr;
#if TK_SUPPORT_SMP
	/* Affinity is fixed at creation.  Without TA_ASSPRC, or with an empty
	   or out-of-range mask, the task may run anywhere: a task that can run
	   on no processor would simply never be scheduled, which is a far worse
	   outcome than ignoring a malformed request. */
	if ( (pk_ctsk->tskatr & TA_ASSPRC) != 0
	  && (pk_ctsk->assprc & ALL_CORE_BIT) != 0 ) {
		tcb->assprc = pk_ctsk->assprc & ALL_CORE_BIT;
	} else {
		tcb->assprc = ALL_CORE_BIT;
	}
#endif
	tcb->task      = pk_ctsk->task;
	tcb->ipriority = (UB)int_priority(pk_ctsk->itskpri);
	tcb->sstksz    = sstksz;
#if USE_OBJECT_NAME
	if ( (pk_ctsk->tskatr & TA_DSNAME) != 0 ) {
		knl_strncpy((char*)tcb->name, (char*)pk_ctsk->dsname, OBJECT_NAME_LENGTH);
	}
#endif

	/* Set stack pointer */
	tcb->isstack = (VB*)stack + sstksz;

	/* Set initial value of task operation mode */
	tcb->isysmode = 1;
	tcb->sysmode  = 1;

	/* make it to DORMANT state */
	knl_make_dormant(tcb);
#if TK_SUPPORT_SMP
	knl_setup_context(tcb);
#endif

	ercd = tcb->tskid;

    error_exit:
	END_CRITICAL_SECTION;

#if USE_IMALLOC
	if ( (ercd < E_OK) && ((pk_ctsk->tskatr & TA_USERBUF) == 0) ) {
		knl_Ifree(stack);
	}
#endif

	return ercd;
}

/*
 * Task deletion
 *	Call from critical section
 */
LOCAL
void knl_del_tsk( TCB *tcb )
{
#if USE_IMALLOC
	if ( (tcb->tskatr & TA_USERBUF) == 0 ) {
		/* User buffer is not used */
		/* Free system stack */
		void *stack = (VB*)tcb->isstack - tcb->sstksz;
		knl_Ifree(stack);
	}
#endif

	/* Return control block to FreeQue */
	QueInsert(&tcb->tskque, &knl_free_tcb);
	tcb->state = TS_NONEXIST;
}

#if TK_SUPPORT_SMP
/* Dispatcher-side release diagnostics are per-core so their hot-path update
 * needs no cross-core read/modify/write. */
EXPORT UW knl_smp_context_releases[TK_MAX_CORE];
EXPORT UW knl_smp_deferred_reclaims;

EXPORT void knl_smp_dispatch_release( TCB *tcb )
{
	UINT core = knl_current_core();

	knl_smp_context_releases[core]++;
	if ( tcb == NULL || !tcb->stkfree ) return;

	/* We are off tcb's stack and ctxtsk was already cleared by assembly.
	 * Serialize allocator/free-queue mutation before returning its storage. */
	knl_kernel_lock_enter();
	if ( tcb->stkfree && !knl_is_ctxtsk(tcb) ) {
		tcb->stkfree = FALSE;
		knl_del_tsk(tcb);
		knl_smp_deferred_reclaims++;
	}
	knl_kernel_lock_leave();
}
#endif

#ifdef USE_FUNC_TK_DEL_TSK
/*
 * Delete task 
 */
SYSCALL ER tk_del_tsk( ID tskid )
{
	TCB	*tcb;
	TSTAT	state;
	ER	ercd = E_OK;

	CHECK_TSKID(tskid);
	CHECK_NONSELF(tskid);

	tcb = get_tcb(tskid);

	BEGIN_CRITICAL_SECTION;
	state = (TSTAT)tcb->state;
	if ( state != TS_DORMANT ) {
		ercd = ( state == TS_NONEXIST )? E_NOEXS: E_OBJ;
	} else {
#if TK_SUPPORT_SMP
		if ( knl_is_ctxtsk(tcb) ) {
			/* tk_ter_tsk() is logically complete before the remote final
			 * save.  Keep its stack and TCB reserved until that safe point. */
			tcb->stkfree = TRUE;
			tcb->state = TS_NONEXIST;
		} else
#endif
		{
		knl_del_tsk(tcb);
		}
	}
	END_CRITICAL_SECTION;

	return ercd;
}
#endif /* USE_FUNC_TK_DEL_TSK */

/* ------------------------------------------------------------------------ */

/*
 * Start task
 */
SYSCALL ER tk_sta_tsk( ID tskid, INT stacd )
{
	TCB	*tcb;
	TSTAT	state;
	ER	ercd = E_OK;

	CHECK_TSKID(tskid);
	CHECK_NONSELF(tskid);

	tcb = get_tcb(tskid);

	BEGIN_CRITICAL_SECTION;
	state = (TSTAT)tcb->state;
	if ( state != TS_DORMANT ) {
		ercd = ( state == TS_NONEXIST )? E_NOEXS: E_OBJ;
#if TK_SUPPORT_SMP
	} else if ( knl_is_ctxtsk(tcb) ) {
		/* The old incarnation still owns a live context. */
		ercd = E_OBJ;
	} else {
		knl_setup_context(tcb);
		knl_setup_stacd(tcb, stacd);
		knl_make_ready(tcb);
	}
#else
	} else {
		knl_setup_stacd(tcb, stacd);
		knl_make_ready(tcb);
	}
#endif
	END_CRITICAL_SECTION;

	return ercd;
}

/*
 * Task finalization
 *	Call from critical section
 */
LOCAL void knl_ter_tsk( TCB *tcb )
{
	TSTAT	state;

	state = (TSTAT)tcb->state;
	if ( state == TS_READY ) {
		knl_make_non_ready(tcb);

	} else if ( (state & TS_WAIT) != 0 ) {
		knl_wait_cancel(tcb);
		if ( tcb->wspec->rel_wai_hook != NULL ) {
			(*tcb->wspec->rel_wai_hook)(tcb);
		}
	}

#if USE_MUTEX == 1
	/* signal mutex */
	knl_signal_all_mutex(tcb);
#endif

	knl_cleanup_context(tcb);
}

#ifdef USE_FUNC_TK_EXT_TSK
/*
 * End its own task
 */
SYSCALL void tk_ext_tsk( void )
{
#ifdef DORMANT_STACK_SIZE
	/* To avoid destroying stack used in 'knl_make_dormant', 
	   allocate the dummy area on the stack. */
	volatile VB _dummy[DORMANT_STACK_SIZE];
#endif

	/* Check context error */
#if CHK_CTX2
	if ( in_indp() ) {
		SYSTEM_MESSAGE("tk_ext_tsk was called in the task independent\n");
		while(1);
		return;
	}
#endif
#if CHK_CTX1
	if ( in_ddsp() ) {
		SYSTEM_MESSAGE("tk_ext_tsk was called in the dispatch disabled\n");
	}
#endif

	DISABLE_INTERRUPT;
	knl_ter_tsk(knl_ctxtsk);
	knl_make_dormant(knl_ctxtsk);

	knl_force_dispatch();
	/* No return */

#ifdef DORMANT_STACK_SIZE
	/* Avoid WARNING (This code does not execute) */
	_dummy[0] = _dummy[0];
#endif
}
#endif /* USE_FUNC_TK_EXT_TSK */

#ifdef USE_FUNC_TK_EXD_TSK
/*
 * End and delete its own task
 */
SYSCALL void tk_exd_tsk( void )
{
	/* Check context error */
#if CHK_CTX2
	if ( in_indp() ) {
		SYSTEM_MESSAGE("tk_exd_tsk was called in the task independent\n");
		return;
	}
#endif
#if CHK_CTX1
	if ( in_ddsp() ) {
		SYSTEM_MESSAGE("tk_exd_tsk was called in the dispatch disabled\n");
	}
#endif

	DISABLE_INTERRUPT;
	knl_ter_tsk(knl_ctxtsk);
#if TK_SUPPORT_SMP
	knl_ctxtsk->stkfree = TRUE;
	knl_ctxtsk->state = TS_NONEXIST;
#else
	knl_del_tsk(knl_ctxtsk);
#endif

	knl_force_dispatch();
	/* No return */
}
#endif /* USE_FUNC_TK_EXD_TSK */

#ifdef USE_FUNC_TK_TER_TSK
/*
 * Termination of other task
 */
SYSCALL ER tk_ter_tsk( ID tskid )
{
	TCB	*tcb;
	TSTAT	state;
	ER	ercd = E_OK;

	CHECK_TSKID(tskid);
	CHECK_NONSELF(tskid);

	tcb = get_tcb(tskid);

	BEGIN_CRITICAL_SECTION;
	state = (TSTAT)tcb->state;
	if ( !knl_task_alive(state) ) {
		ercd = ( state == TS_NONEXIST )? E_NOEXS: E_OBJ;
	} else if ( tcb->klocked ) {
		/* Normally, it does not become this state.
		 * When the state is page-in wait in the virtual memory
		 * system and when trying to terminate any task,
		 * it becomes this state.
		 */
		ercd = E_OBJ;
	} else {
#if TK_SUPPORT_SMP
		INT core = knl_ctxtsk_core(tcb);
		if ( core >= 0 ) {
			/* Termination overrides the victim's dispatch-disable state: a
			 * dead task must be allowed to reach its final save. */
			*knl_dispatch_disabled_core_slot((UINT)core) = DDS_ENABLE;
		}
#endif
		knl_ter_tsk(tcb);
		knl_make_dormant(tcb);
	}
	END_CRITICAL_SECTION;

	return ercd;
}
#endif /* USE_FUNC_TK_TER_TSK */

/* ------------------------------------------------------------------------ */

#ifdef USE_FUNC_TK_CHG_PRI
/*
 * Change task priority
 */
SYSCALL ER tk_chg_pri( ID tskid, PRI tskpri )
{
	TCB	*tcb;
	INT	priority;
	ER	ercd;

	CHECK_TSKID_SELF(tskid);
	CHECK_PRI_INI(tskpri);

	tcb = get_tcb_self(tskid);

	BEGIN_CRITICAL_SECTION;
	if ( tcb->state == TS_NONEXIST ) {
		ercd = E_NOEXS;
		goto error_exit;
	}

	/* Conversion priority to internal expression */
	if ( tskpri == TPRI_INI ) {
		priority = tcb->ipriority;
	} else {
		priority = int_priority(tskpri);
	}

#if USE_MUTEX == 1
	/* Mutex priority change limit */
	ercd = knl_chg_pri_mutex(tcb, priority);
	if ( ercd < E_OK ) {
		goto error_exit;
	}

	tcb->bpriority = (UB)priority;
	priority = ercd;
#else
	tcb->bpriority = priority;
#endif

	/* Change priority */
	knl_change_task_priority(tcb, priority);

	ercd = E_OK;
    error_exit:
	END_CRITICAL_SECTION;

	return ercd;
}
#endif /* USE_FUNC_TK_CHG_PRI */

#ifdef USE_FUNC_TK_ROT_RDQ
/*
 * Rotate ready queue
 */
SYSCALL ER tk_rot_rdq( PRI tskpri )
{
	CHECK_PRI_RUN(tskpri);

	BEGIN_CRITICAL_SECTION;
	if ( tskpri == TPRI_RUN ) {
		if ( in_indp() ) {
			knl_rotate_ready_queue_run();
		} else {
			knl_rotate_ready_queue(knl_ctxtsk->priority);
		}
	} else {
		knl_rotate_ready_queue(int_priority(tskpri));
	}
	END_CRITICAL_SECTION;

	return E_OK;
}
#endif /* USE_FUNC_TK_ROT_RDQ */

/* ------------------------------------------------------------------------ */

#ifdef USE_FUNC_TK_GET_TID
/*
 * Refer task ID at execution
 */
SYSCALL ID tk_get_tid( void )
{
	return ( knl_ctxtsk == NULL )? 0: knl_ctxtsk->tskid;
}
#endif /* USE_FUNC_TK_GET_TID */

#ifdef USE_FUNC_TK_REF_TSK
/*
 * Refer task state
 */
SYSCALL ER tk_ref_tsk( ID tskid, T_RTSK *pk_rtsk )
{
	TCB	*tcb;
	TSTAT	state;
	ER	ercd = E_OK;

	CHECK_TSKID_SELF(tskid);

	tcb = get_tcb_self(tskid);

	knl_memset(pk_rtsk, 0, sizeof(*pk_rtsk));

	BEGIN_CRITICAL_SECTION;
	state = (TSTAT)tcb->state;
	if ( state == TS_NONEXIST ) {
		ercd = E_NOEXS;
	} else {
		if ( ( state == TS_READY ) &&
#if TK_SUPPORT_SMP
		     knl_is_ctxtsk(tcb) ) {
#else
		     ( tcb == knl_ctxtsk ) ) {
#endif
			pk_rtsk->tskstat = TTS_RUN;
		} else {
			pk_rtsk->tskstat = (UINT)state << 1;
		}
		if ( (state & TS_WAIT) != 0 ) {
			pk_rtsk->tskwait = tcb->wspec->tskwait;
			pk_rtsk->wid     = tcb->wid;
		}
		pk_rtsk->exinf     = tcb->exinf;
		pk_rtsk->tskpri    = ext_tskpri(tcb->priority);
		pk_rtsk->tskbpri   = ext_tskpri(tcb->bpriority);
		pk_rtsk->wupcnt    = tcb->wupcnt;
		pk_rtsk->suscnt    = tcb->suscnt;
	}
	END_CRITICAL_SECTION;

	return ercd;
}
#endif /* USE_FUNC_TK_REF_TSK */

/* ------------------------------------------------------------------------ */


#ifdef USE_FUNC_TK_REL_WAI
/*
 * Release wait
 */
SYSCALL ER tk_rel_wai( ID tskid )
{
	TCB	*tcb;
	TSTAT	state;
	ER	ercd = E_OK;

	CHECK_TSKID(tskid);

	tcb = get_tcb(tskid);

	BEGIN_CRITICAL_SECTION;
	state = (TSTAT)tcb->state;
	if ( (state & TS_WAIT) == 0 ) {
		ercd = ( state == TS_NONEXIST )? E_NOEXS: E_OBJ;
	} else {
		knl_wait_release_ng(tcb, E_RLWAI);
	}
	END_CRITICAL_SECTION;

	return ercd;
}
#endif /* USE_FUNC_TK_REL_WAI */

/* ------------------------------------------------------------------------ */
/*
 *	Debug support function
 */
#if USE_DBGSPT

#if USE_OBJECT_NAME
/*
 * Get object name from control block
 */
EXPORT ER knl_task_getname(ID id, UB **name)
{
	TCB	*tcb;
	ER	ercd = E_OK;

	CHECK_TSKID_SELF(id);

	BEGIN_DISABLE_INTERRUPT;
	tcb = get_tcb_self(id);
	if ( tcb->state == TS_NONEXIST ) {
		ercd = E_NOEXS;
		goto error_exit;
	}
	if ( (tcb->tskatr & TA_DSNAME) == 0 ) {
		ercd = E_OBJ;
		goto error_exit;
	}
	*name = tcb->name;

    error_exit:
	END_DISABLE_INTERRUPT;

	return ercd;
}
#endif /* USE_OBJECT_NAME */

#ifdef USE_FUNC_TD_LST_TSK
/*
 * Refer task usage state
 */
SYSCALL INT td_lst_tsk( ID list[], INT nent )
{
	TCB	*tcb, *end;
	INT	n = 0;

	BEGIN_DISABLE_INTERRUPT;
	end = knl_tcb_table + NUM_TSKID;
	for ( tcb = knl_tcb_table; tcb < end; tcb++ ) {
		if ( tcb->state == TS_NONEXIST ) {
			continue;
		}

		if ( n++ < nent ) {
			*list++ = tcb->tskid;
		}
	}
	END_DISABLE_INTERRUPT;

	return n;
}
#endif /* USE_FUNC_TD_LST_TSK */

#ifdef USE_FUNC_TD_REF_TSK
/*
 * Refer task state
 */
SYSCALL ER td_ref_tsk( ID tskid, TD_RTSK *pk_rtsk )
{
	TCB	*tcb;
	TSTAT	state;
	ER	ercd = E_OK;

	CHECK_TSKID_SELF(tskid);

	tcb = get_tcb_self(tskid);

	knl_memset(pk_rtsk, 0, sizeof(*pk_rtsk));

	BEGIN_DISABLE_INTERRUPT;
	state = (TSTAT)tcb->state;
	if ( state == TS_NONEXIST ) {
		ercd = E_NOEXS;
	} else {
		if ( ( state == TS_READY ) &&
#if TK_SUPPORT_SMP
		     knl_is_ctxtsk(tcb) ) {
#else
		     ( tcb == knl_ctxtsk ) ) {
#endif
			pk_rtsk->tskstat = TTS_RUN;
		} else {
			pk_rtsk->tskstat = (UINT)state << 1;
		}
		if ( (state & TS_WAIT) != 0 ) {
			pk_rtsk->tskwait = tcb->wspec->tskwait;
			pk_rtsk->wid     = tcb->wid;
		}
		pk_rtsk->exinf     = tcb->exinf;
		pk_rtsk->tskpri    = ext_tskpri(tcb->priority);
		pk_rtsk->tskbpri   = ext_tskpri(tcb->bpriority);
		pk_rtsk->wupcnt    = tcb->wupcnt;
		pk_rtsk->suscnt    = tcb->suscnt;

		pk_rtsk->task      = tcb->task;
		pk_rtsk->stksz     = tcb->sstksz;
		pk_rtsk->istack    = tcb->isstack;
	}
	END_DISABLE_INTERRUPT;

	return ercd;
}
#endif /* USE_FUNC_TD_REF_TSK */

#ifdef USE_FUNC_TD_INF_TSK
/*
 * Get task statistic information
 */
SYSCALL ER td_inf_tsk( ID tskid, TD_ITSK *pk_itsk, BOOL clr )
{
	TCB	*tcb;
	ER	ercd = E_OK;

	CHECK_TSKID_SELF(tskid);

	tcb = get_tcb_self(tskid);

	BEGIN_DISABLE_INTERRUPT;
	if ( tcb->state == TS_NONEXIST ) {
		ercd = E_NOEXS;
	} else {
		pk_itsk->stime = tcb->stime;
		pk_itsk->utime = tcb->utime;
		if ( clr ) {
			tcb->stime = 0;
			tcb->utime = 0;
		}
	}
	END_DISABLE_INTERRUPT;

	return ercd;
}
#endif /* USE_FUNC_TD_INF_TSK */

#endif /* USE_DBGSPT */
