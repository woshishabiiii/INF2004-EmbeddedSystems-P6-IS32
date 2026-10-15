/*
 *----------------------------------------------------------------------
 *    Device Driver for μT-Kernel 3.0
 *
 *    Copyright (C) 2020-2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2022/02.
 *
 *----------------------------------------------------------------------
 */

/*
 *	dev_ser.h
 *	Serial communication device driver
 *	Driver local definition
 */

#ifndef	__DEV_SER_H__
#define	__DEV_SER_H__

#include "../include/dev_ser.h"
#include "../common/drvif/msdrvif.h"
#include "ser_cnf.h"

/*----------------------------------------------------------------------*/
/* Hardware dependent definition
 */
#define DEVDEF_SER_PATH_(a)	#a
#define DEVDEF_SER_PATH(a)	DEVDEF_SER_PATH_(a)
#define DEVDEF_SER_SYSDEP()	DEVDEF_SER_PATH(sysdepend/TARGET_CPU_DIR/ser_sysdep.h)
#include DEVDEF_SER_SYSDEP()

/*----------------------------------------------------------------------
 * Communication data buffer
 */
typedef struct {
	UW	top;
	UW	tail;
#ifdef CPU_RP2040
	ID	wait_flgid;
#else
	ID	wait_tskid;
#endif
	UB	data[DEVCONF_SER_BUFFSIZE];
} T_SER_BUFF;

Inline	void buff_init(T_SER_BUFF *buff)
{
	buff->top = buff->tail = 0;
#ifdef CPU_RP2040
	(void)tk_clr_flg(buff->wait_flgid, 0);
#else
	buff->wait_tskid = 0;
#endif
}

/*----------------------------------------------------------------------
 * Device driver Control block
 */
typedef struct {
	UW	unit;		/* Unit No. */
	UINT	omode;		/* Open mode */

	/* Attribute data */
	ID	evtmbfid;	/* MBF ID for event notification */
	UW	com_mode;	/* Communication mode */
	UW	com_speed;	/* Communication Speed */
	UW	com_error;	/* Communication Error */
	TMO	snd_tmo;	/* Send timeout */
	TMO	rcv_tmo;	/* Receive timeout */

	/* Intrrupt */
	UINT	intno_rcv;	/* Receive interrupt number */
	UINT	intno_snd;	/* Send interrupt number */
	UINT	int_pri;	/* Interrupt priority */

	/* Communication data buffer */
	T_SER_BUFF	snd_buff;
	T_SER_BUFF	rcv_buff;

} T_SER_DCB;

/*----------------------------------------------------------------------
 * Serial device driver low-level interface
 */

/* The following functions are defined in the system common part. */
IMPORT BOOL dev_ser_get_snddat(UW unit, UW *data);	/* Get send data */
IMPORT void dev_ser_notify_rcv(UW unit, UW data);	/* Notification of receipt */
IMPORT void dev_ser_notify_err(UW unit, UW err);	/* Communication error notification */

/* The following functions are defined in the system-dependent part. */

/* Low-level device control */
IMPORT ER dev_ser_llctl(UW, INT, UW);		/* Low-Level device control */
IMPORT ER dev_ser_llinit(T_SER_DCB*);		/* Low-level device initialization */

#ifdef CPU_RP2040
/* RP2040 peripheral IRQs are owned by physical core 0.  These helpers mask
 * local interrupts and take a cross-core lock so task-side ring operations
 * cannot race the owner-core handler. */
IMPORT void dev_ser_buf_lock(UW unit, UINT *intsts);
IMPORT void dev_ser_buf_unlock(UW unit, UINT intsts);
#define DEV_SER_BUF_LOCK(p_dcb, intsts) \
	dev_ser_buf_lock((p_dcb)->unit, &(intsts))
#define DEV_SER_BUF_UNLOCK(p_dcb, intsts) \
	dev_ser_buf_unlock((p_dcb)->unit, (intsts))
#else
#define DEV_SER_BUF_LOCK(p_dcb, intsts) \
	DisableInt((p_dcb)->intno_rcv)
#define DEV_SER_BUF_UNLOCK(p_dcb, intsts) \
	EnableInt((p_dcb)->intno_rcv, (p_dcb)->int_pri)
#endif

/* Low-level device control command */
typedef enum {
	LLD_SER_MODE,		/* Set Communication mode */
	LLD_SER_SPEED,		/* Set Communication Speed */
	LLD_SER_START,		/* Start communication */
	LLD_SER_STOP,		/* Stop Start communication */
	LLD_SER_SEND,		/* Send data */
	LLD_SER_BREAK,		/* Send Break */
	LLD_SER_INTCTL,		/* Interrupt control */
} T_LLD_SER_CMD;

#endif		/* __DEV_SER_H__ */
