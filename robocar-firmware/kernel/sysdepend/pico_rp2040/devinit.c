/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.0 BSP
 *
 *    Copyright (C) 2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2022/11.
 *
 *----------------------------------------------------------------------
 */

#include <sys/machine.h>
#ifdef PICO_RP2040

/*
 *	devinit.c (RaspberryPi Pico)
 *	Device-Dependent Initialization
 */

#include <sys/sysdef.h>
#include <tm/tmonitor.h>
#include <tk/device.h>

#include <kernel.h>
#include "sysdepend.h"

/* ------------------------------------------------------------------------ */

/*
 * Initialization before micro T-Kernel starts
 */

EXPORT ER knl_init_device( void )
{
	return E_OK;
}

/* ------------------------------------------------------------------------ */
/*
 * Start processing after T-Kernel starts
 *	Called from the initial task contexts.
 */
#if USE_TMONITOR && TM_CONSOLE_USB_CDC
IMPORT ER tm_usb_console_start(void);
#endif

#if TM_WIFI_CYW43
IMPORT ER cyw43_utk_start(void);
#endif

#if TK_SUPPORT_SMP
#include "../cpu/rp2040/smp_rp2040.h"
#endif

EXPORT ER knl_start_device( void )
{
#if USE_SDEV_DRV	// Use sample driver
	ER	err;

	/* A/D Converter unit.0 "adca" */
	#if DEVCNF_USE_ADC
		err = dev_init_adc(0);
		if(err < E_OK) return err;
	#endif

	/* I2C unit.0 "iica" */
	#if DEVCNF_USE_IIC
		err = dev_init_i2c(0);
		if(err < E_OK) return err;
	#endif

	/* UART0 "sera" */
	#if DEVCNF_USE_SER
		err = dev_init_ser(0);
		if(err < E_OK) return err;
	#endif

#endif

#if TK_SUPPORT_SMP
	{	/* Measure how deep the dispatcher's temporary stacks are used. */
		IMPORT void knl_tmp_stack_paint(void);
		knl_tmp_stack_paint();
	}

	/* Bring core 1 up.  It parks in a loop servicing only IPIs and touches
	   no kernel state: shared kernel data is still guarded solely by local
	   interrupt masking, which does not exclude the other core.  Core 1
	   enters the scheduler in Phase 6, after the kernel lock exists. */
	knl_smp_launch_ok = knl_smp_launch_core1()? 1: 0;
	knl_smp_core0_init();
#endif

#if USE_TMONITOR && TM_CONSOLE_USB_CDC
	/* Start the USB CDC console.  This runs from the initial task with the
	   kernel fully up, which is the earliest point a service task can be
	   created, so every application gets the console without wiring it in
	   itself.  A failure here is not fatal: the UART mirror still works. */
	(void)tm_usb_console_start();
#endif

#if TM_WIFI_CYW43
	/* CYW43 state, PIO/DMA and radio pins are owned by a processor-1
	 * (physical core-0) service task. */
	(void)cyw43_utk_start();
#endif

	return E_OK;
}

#if USE_SHUTDOWN
/* ------------------------------------------------------------------------ */
/*
 * System finalization
 *	Called just before system shutdown.
 *	Execute finalization that must be done before system shutdown.
 */
EXPORT ER knl_finish_device( void )
{
	return E_OK;
}

#endif /* USE_SHUTDOWN */

#endif /* PICO_RP2040 */
