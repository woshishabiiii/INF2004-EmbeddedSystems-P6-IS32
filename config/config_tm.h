/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.00.04
 *
 *    Copyright (C) 2006-2021 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2021/05/17.
 *
 *----------------------------------------------------------------------
 */

/*
 *	config_tm.h
 *	T-Monitor Configuration Definition
 */

#ifndef __TM_CONFIG_H__
#define __TM_CONFIG_H__

/*---------------------------------------------------------------------- */
/* Select a communication port
 *      Select the communication port used by T-Monitor.
 *         1: Valid  0: Invalid  (Only one of them is valid)
 */
#define	TM_COM_SERIAL_DEV	(1)	/* Use serial communication device */
#define	TM_COM_NO_DEV		(0)	/* Do not use communication port */

/*---------------------------------------------------------------------- */
/* tm_printf() call setting
 *         1: Valid  0: Invalid
 */
#define	USE_TM_PRINTF		(1)	/* Use tm_printf() & tm_sprintf() calls */
#define	TM_OUTBUF_SZ		(0)	/* Output Buffer size in stack */

/*
 * Console sink selection (RaspberryPi Pico).
 *
 * TM_CONSOLE_USB_CDC is normally set by the build (CONSOLE=usb_cdc in
 * build_make/pico_rp2040.mk); these are the fallbacks.
 *
 * The UART sink is kept enabled alongside USB on purpose.  USB output needs
 * the scheduler and an enumerated host, so it can say nothing about early
 * boot, handler context, or a fault -- exactly the window in which the
 * Phase-0 startup HardFault had to be diagnosed by flashing GPIOs because no
 * console existed. UART0 costs two pins and covers that window.
 */
#ifndef TM_CONSOLE_USB_CDC
#define	TM_CONSOLE_USB_CDC	(0)	/* USB CDC-ACM console */
#endif
#ifndef TM_CONSOLE_UART
#define	TM_CONSOLE_UART		(1)	/* UART0 early/panic mirror */
#endif

#endif /* __TM_CONFIG_H__ */
