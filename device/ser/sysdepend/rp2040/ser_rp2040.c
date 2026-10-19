/*
 *----------------------------------------------------------------------
 *    Device Driver for μT-Kernel 3.0
 *
 *    Copyright (C) 2020-2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2022/11.
 *
 *----------------------------------------------------------------------
 */

#include <sys/machine.h>
#ifdef CPU_RP2040

#include <tk/tkernel.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif
#include "../../ser.h"
#include "../../../include/dev_def.h"
#if DEV_SER_ENABLE
/*
 *	ser_rp2040.c
 *	Serial communication device driver
 *	System dependent processing for RP2040
 */

/*----------------------------------------------------------------------
 * Device register base address
 */
const LOCAL UW ba[DEV_SER_UNITNM] = { UART0_BASE, UART1_BASE };

#define	UART_DR(x)	(ba[x] + UARTx_DR)	// Data Register
#define	UART_RSR(x)	(ba[x] + UARTx_RSR)	// Receive Status Register/Error Clear Register
#define	UART_FR(x)	(ba[x] + UARTx_FR)	// Flag Register

#define	UART_IBRD(x)	(ba[x] + UARTx_IBRD)	// Integer Baud Rate Register
#define	UART_FBRD(x)	(ba[x] + UARTx_FBRD)	// Fractional Baud Rate Register
#define	UART_LCR_H(x)	(ba[x] + UARTx_LCR_H)	// Line Control Register
#define	UART_CR(x)	(ba[x] + UARTx_CR)	// Control Register

#define	UART_IFLS(x)	(ba[x] + UARTx_IFLS)	// Interrupt FIFO Level Select Register
#define	UART_IMSC(x)	(ba[x] + UARTx_IMSC)	// Interrupt Mask Set/Clear Register
#define	UART_RIS(x)	(ba[x] + UARTx_RIS)	// Raw Interrupt Status Register
#define	UART_MIS(x)	(ba[x] + UARTx_MIS)	// Masked Interrupt Status Register
#define	UART_ICR(x)	(ba[x] + UARTx_ICR)	// Interrupt Clear Register

/*----------------------------------------------------------------------
 * Device data
*/
const LOCAL struct {
	UINT	intno;		// Interrupt number
	PRI	intpri;		// Interrupt priority
} ll_devdat[DEV_SER_UNITNM] = {
	{	/* UART0 */
		.intno		= INTNO_UART0,
		.intpri		= DEVCNF_UART0_INTPRI,
	},
	{	/* UART1 */
		.intno		= INTNO_UART1,
		.intpri		= DEVCNF_UART1_INTPRI,
	},
};

/*----------------------------------------------------------------------
 * Device low-level control data
*/
typedef struct {
	UW	mode;		// Serial mode
	UW	speed;		// Spped (bit rate)
} T_DEV_SER_LLDEVCB;

LOCAL T_DEV_SER_LLDEVCB		ll_devcb[DEV_SER_UNITNM];

#if TK_SUPPORT_SMP
LOCAL T_SPLOCK	ll_buf_lock[DEV_SER_UNITNM];
#endif

EXPORT void dev_ser_buf_lock(UW unit, UINT *intsts)
{
#if TK_SUPPORT_SMP
	(void)ISpinLock(&ll_buf_lock[unit], intsts);
#else
	DI(*intsts);
#endif
}

EXPORT void dev_ser_buf_unlock(UW unit, UINT intsts)
{
#if TK_SUPPORT_SMP
	(void)ISpinUnlock(&ll_buf_lock[unit], intsts);
#else
	(void)unit;
	EI(intsts);
#endif
}

/*----------------------------------------------------------------------
 * Interrupt handler
 */
void usart_inthdr( UINT intno)
{
	W	unit;
	UW	mis, data;
	UINT	intsts;

	if(intno == INTNO_UART0) {
		unit = DEV_SER_UNIT0;
	} else if(intno == INTNO_UART1) {
		unit = DEV_SER_UNIT1;
	} else {
		ClearInt(intno);
		return;
	}

	dev_ser_buf_lock((UW)unit, &intsts);

	mis = in_w(UART_MIS(unit));		// Get interrupt factor

	out_w(UART_ICR(unit), -1);		// Clear interrupt
	ClearInt(intno);

	/* Reception process */
	if( mis & UART_MIS_RX) {
		data = in_w(UART_DR(unit));
		if(data & UART_DR_ERR) {
			dev_ser_notify_err(unit, (data>>8)&0x0F);	/* Notify the main process of this error. */
		} else {
			dev_ser_notify_rcv(unit, data & UART_DR_DATA);	/* Notify the main process of data reception. */
		}
	}

	/* Transmission process */
	if( mis & UART_MIS_TX) {
		if( dev_ser_get_snddat(unit, &data)) {
			out_w(UART_DR(unit), data);
		} else {
			clr_w(UART_IMSC(unit), UART_IMSC_TXIM);	// Set TXI Interrupt mask.
		}
	}

	/* Break error handling */
	if( mis & UART_MIS_BE) {
		dev_ser_notify_err(unit, DEV_SER_ERR_BE);	/* Notify the main process of this error. */
	}
	dev_ser_buf_unlock((UW)unit, intsts);
}

/*----------------------------------------------------------------------
 * Set mode & Start communication
 */
LOCAL void start_com(UW unit, UW mode, UW speed)
{
	UW	ibrd, fbrd, rdiv;

	/* Baud rate setting */
	rdiv = (8 * CLK_PERI_FREQ / speed);
	ibrd = rdiv>>7; fbrd = 0;
	if(ibrd == 0) {
		ibrd = 1; 
	} else if(ibrd >= 65535) {
		ibrd = 65535;
	} else {
		fbrd = ((rdiv & 0x7f) + 1) / 2;
	}
	out_w(UART_IBRD(unit), ibrd);
	out_w(UART_FBRD(unit), fbrd);

	/* Communication data format setting */
	out_w(UART_LCR_H(unit), mode);		

	/* UART enabled */
	out_w(UART_CR(unit), UART_CR_RXE | UART_CR_TXE | UART_CR_UARTEN);
}

/*----------------------------------------------------------------------
 * Stop communication
 */
LOCAL void stop_com(UW unit)
{
	if(unit != DEVCNF_SER_DBGUN) {
		out_w(UART_CR(unit), UART_CR_RXE | UART_CR_TXE);	// UART disable.
	} else {	/* Used by T-Monitor */
		out_w(UART_CR(unit), UART_CR_DEBUG);
	}
}

/*----------------------------------------------------------------------
 * Low level device control
 */
EXPORT ER dev_ser_llctl( UW unit, INT cmd, UW parm)
{
	ER	err	= E_OK;

	switch(cmd) {
	case LLD_SER_MODE:	/* Set Communication mode */
		ll_devcb[unit].mode = parm;
		break;
	
	case LLD_SER_SPEED:	/* Set Communication Speed */
		ll_devcb[unit].speed = parm;
		break;
	
	case LLD_SER_START:	/* Start communication */
		out_w(UART_CR(unit), UART_CR_RXE | UART_CR_TXE);		// UART disable.
		out_w(UART_IMSC(unit), UART_IMSC_RXIM | UART_IMSC_BEIM);	// Clear RXI Interrupt mask.
		
		out_w(UART_ICR(unit), 0);					// Clear interrupt
		ClearInt(ll_devdat[unit].intno);
		start_com( unit, ll_devcb[unit].mode, ll_devcb[unit].speed);
		break;

	case LLD_SER_STOP:
		out_w(UART_IMSC(unit), 0);
		stop_com(unit);
		break;

	case LLD_SER_SEND:
		if(in_w(UART_FR(unit)) & UART_FR_TXFF) {
			err = E_BUSY;				// Transmit FIFO full.
		} else {
			out_w(UART_DR(unit), parm);		// Set Transmission data
			set_w(UART_IMSC(unit), UART_IMSC_TXIM);	// Clear TXI Interrupt mask.
		}
		break;

	case LLD_SER_BREAK:
		if(parm) {
			set_w(UART_LCR_H(unit), UART_LCR_H_BRK);	/* Send Break */
		} else {
			clr_w(UART_LCR_H(unit), UART_LCR_H_BRK);	/* Stop Break */
		}
		break;
	}

	return err;
}

/*----------------------------------------------------------------------
 * Device initialization
 */
EXPORT ER dev_ser_llinit( T_SER_DCB *p_dcb)
{
	const T_DINT	dint = {
		.intatr	= TA_HLNG,
		.inthdr	= usart_inthdr,
	};
	T_CFLG	cflg = {0};
	UW	unit;
	ER	err;
	unit = p_dcb->unit;

#if TK_SUPPORT_SMP
	(void)InitSpinLock(&ll_buf_lock[unit]);
#endif
	cflg.flgatr = TA_TFIFO;
	p_dcb->snd_buff.wait_flgid = tk_cre_flg(&cflg);
	if(p_dcb->snd_buff.wait_flgid < E_OK) {
		return p_dcb->snd_buff.wait_flgid;
	}
	p_dcb->rcv_buff.wait_flgid = tk_cre_flg(&cflg);
	if(p_dcb->rcv_buff.wait_flgid < E_OK) {
		err = p_dcb->rcv_buff.wait_flgid;
		(void)tk_del_flg(p_dcb->snd_buff.wait_flgid);
		return err;
	}

	/* UART device initialize */
	stop_com(unit);

	/* Device Control block Initizlize */
	p_dcb->intno_rcv = p_dcb->intno_snd = ll_devdat[unit].intno;
	p_dcb->int_pri = ll_devdat[unit].intpri;

	/* Interrupt handler definition */
	err = tk_def_int(ll_devdat[unit].intno, &dint);
	if(err == E_OK) {
		ClearInt(ll_devdat[unit].intno);
		/* llinit runs on physical core 0 before core 1 is launched.  Leave
		 * NVIC ownership here permanently and gate activity in UART_IMSC. */
		EnableInt(ll_devdat[unit].intno, ll_devdat[unit].intpri);
	} else {
		(void)tk_del_flg(p_dcb->rcv_buff.wait_flgid);
		(void)tk_del_flg(p_dcb->snd_buff.wait_flgid);
	}

	return err;
}

#endif		/* DEV_SER_ENABLE */
#endif		/* CPU_RP2040 */
