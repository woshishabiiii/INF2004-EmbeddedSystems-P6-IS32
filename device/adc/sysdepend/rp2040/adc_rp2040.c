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
#include <tm/tmonitor.h>

#include "../../adc.h"
#include "../../../include/dev_def.h"
#if DEV_ADC_ENABLE
/*
 *	dev_adc_rp2040.c
 *	A/D converter device driver
 *	System dependent processing for RP2040
 */

/*----------------------------------------------------------------------
 * Device control data
*/
LOCAL struct {
	ID	done_flgid;
	UW	*buf;
	SZ	asz;
	BOOL	active;
} ll_devcb;

#if TK_SUPPORT_SMP
LOCAL T_SPLOCK ll_lock;
#define ADC_LOCK(sts)      ((void)ISpinLock(&ll_lock, &(sts)))
#define ADC_UNLOCK(sts)    ((void)ISpinUnlock(&ll_lock, (sts)))
#else
#define ADC_LOCK(sts)      DI(sts)
#define ADC_UNLOCK(sts)    EI(sts)
#endif

/*----------------------------------------------------------------------
 * Interrupt handler
 */
void adc_inthdr( UINT intno)
{
	UINT	intsts;
	BOOL	done = FALSE;

	ADC_LOCK(intsts);
	if(ll_devcb.active && ll_devcb.buf != NULL) {
		*ll_devcb.buf = in_w(ADC_FIFO) & 0x0FFF;
		ll_devcb.asz = 1;
		ll_devcb.active = FALSE;
		done = TRUE;
	}
	out_w(ADC_INTE, 0);
	if(done) (void)tk_set_flg(ll_devcb.done_flgid, 1);
	ADC_UNLOCK(intsts);

	ClearInt(intno);
}

/*----------------------------------------------------------------------
 * A/D convert
 */
LOCAL UW adc_convert( INT ch, INT size, UW *buf )
{
	ER	err;
	UINT	intsts;
	UINT	flgptn;

	if(ch<0 || ch>(ADC_CH_NUM-1)) return E_PAR;
	if(size != 1) return E_PAR;

	while((in_w(ADC_CS)&ADC_CS_READY)==0);

	ADC_LOCK(intsts);
	(void)tk_clr_flg(ll_devcb.done_flgid, 0);
	ll_devcb.buf = buf;
	ll_devcb.asz = 0;
	ll_devcb.active = TRUE;

	out_w(ADC_INTE,1);					// Interrupt Enable
	out_w(ADC_CS, ch<<ADC_CS_AINSEL_POS|ADC_CS_EN);
	set_w(ADC_CS, ADC_CS_STRAT_ONCE);
	ADC_UNLOCK(intsts);

	err = tk_wai_flg(ll_devcb.done_flgid, 1,
			 TWF_ORW | TWF_BITCLR, &flgptn, DEVCNF_ADC_TMOSCAN);

	ADC_LOCK(intsts);
	if(err < E_OK && ll_devcb.active) {
		out_w(ADC_INTE, 0);
		ll_devcb.active = FALSE;
		ll_devcb.buf = NULL;
	}
	if(err == E_OK) err = (ll_devcb.asz == 1)? 1: E_IO;
	ADC_UNLOCK(intsts);

	return err;
}


/*----------------------------------------------------------------------
 * A/DC open
 */
LOCAL ER adc_open(void)
{
	ER	err = E_OK;

	out_w(ADC_DIV, ADC_DIV_INI);				// Clock divider

	out_w(ADC_FCS, 1<<ADC_FCS_THRESH_POS | ADC_FCS_EN);	// Set FIFO

	return err;
}

/*----------------------------------------------------------------------
 * A/DC close
 */
LOCAL void adc_close(void)
{
	out_w(ADC_INTE, 0);
}

/*----------------------------------------------------------------------
 * Low level device control
 */
EXPORT W dev_adc_llctl( UW unit, INT cmd, UW p1, UW p2, UW *pp)
{
	W	rtn	= (W)E_OK;

	switch(cmd) {
	case LLD_ADC_OPEN:	/* Open　A/DC */
		rtn = (W)adc_open();
		break;

	case LLD_ADC_CLOSE:	/* Close　A/DC */
		adc_close();
		break;
	
	case LLD_ADC_READ:	/* Read A/DC data */
		rtn = adc_convert( p1, p2, pp);
		break;
	
	case LLD_ADC_RSIZE:	/* Get read data size */
		rtn = 1;
		break;
	}
	
	return rtn;
}

/*----------------------------------------------------------------------
 * Device initialization
 */
EXPORT ER dev_adc_llinit( T_ADC_DCB *p_dcb)
{
	const T_DINT	dint = {
		.intatr	= TA_HLNG,
		.inthdr	= adc_inthdr
	};

	T_CFLG	cflg = {0};
	ER	err;

#if TK_SUPPORT_SMP
	(void)InitSpinLock(&ll_lock);
#endif
	cflg.flgatr = TA_TFIFO;
	ll_devcb.done_flgid = tk_cre_flg(&cflg);
	if(ll_devcb.done_flgid < E_OK) return ll_devcb.done_flgid;
	ll_devcb.buf = NULL;
	ll_devcb.asz = 0;
	ll_devcb.active = FALSE;

/* Release A/DC reset */
#if DEVCONF_ADC_REL_RESET
	clr_w(RESETS_RESET, RESETS_RESET_ADC);	// Release reset
#endif

/* Initialize analog input pins */
#if DEVCONF_ADC_PIN_INIT_0
	out_w(GPIO_CTRL(26),GPIO_CTRL_FUNCSEL_NULL);
	clr_w(GPIO(26), GPIO_IE|GPIO_PUE|GPIO_PDE);
#endif

#if DEVCONF_ADC_PIN_INIT_1
	out_w(GPIO_CTRL(27),GPIO_CTRL_FUNCSEL_NULL);
	clr_w(GPIO(27), GPIO_IE|GPIO_PUE|GPIO_PDE);
#endif

#if DEVCONF_ADC_PIN_INIT_2
	out_w(GPIO_CTRL(28),GPIO_CTRL_FUNCSEL_NULL);
	clr_w(GPIO(28), GPIO_IE|GPIO_PUE|GPIO_PDE);
#endif

#if DEVCONF_ADC_PIN_INIT_3
	out_w(GPIO_CTRL(29),GPIO_CTRL_FUNCSEL_NULL);
	clr_w(GPIO(29), GPIO_IE|GPIO_PUE|GPIO_PDE);
#endif

	/* Enable A/DC */
	set_w(ADC_CS, ADC_CS_EN);
	while(!(in_w(ADC_CS)&ADC_CS_READY));

	err = tk_def_int( INTNO_ADC, &dint);
	if(err == E_OK) {
		ClearInt(INTNO_ADC);
		EnableInt(INTNO_ADC, DEVCNF_ADC_INTPRI);
	} else {
		(void)tk_del_flg(ll_devcb.done_flgid);
	}

	return err;
}

#endif		/* DEV_ADC_ENABLE */
#endif		/* CPU_RP2040 */
