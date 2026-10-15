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

/*
 *    tm_com.c
 *    T-Monitor Communication low-level device driver (RaspberryPi Pico / Pico W)
 *
 *    Two sinks, because they are good at different things:
 *
 *      UART0   available from libtm_init() onward, needs no host, no
 *              enumeration and no scheduler.  This is the only channel that
 *              works during early boot, inside handlers, and after a fault,
 *              so it stays compiled in as the early/panic mirror.
 *
 *      USB CDC the console you actually read.  Needs the scheduler (its
 *              service task drains the ring) and a host that has enumerated
 *              the device, so it says nothing about early boot.
 *
 *    Output goes to both when both are enabled.  Neither waits indefinitely
 *    on external I/O: the UART wait is bounded and the USB path only appends
 *    under a short SMP ring lock.  This matters -- the original driver spun
 *    forever on a UART flag, which turns any console problem into an
 *    unexplained hang.
 */

#include <tk/typedef.h>
#include <sys/sysdef.h>

#if USE_TMONITOR
#include "../../libtm.h"

#ifdef PICO_RP2040

/* UART register definition (Use UART0) */
#define UART_BASE              0x40034000

#define UART_DR		(*(_UW*)(UART_BASE+0x000))
#define UART_FR		(*(_UW*)(UART_BASE+0x018))
#define UART_IBRD	(*(_UW*)(UART_BASE+0x024))
#define UART_FBRD	(*(_UW*)(UART_BASE+0x028))
#define UART_LCR_H	(*(_UW*)(UART_BASE+0x02C))
#define UART_CR		(*(_UW*)(UART_BASE+0x030))

#define	FR_TXFF		(1<<5)	// Transmit FIFO full
#define	FR_RXFE		(1<<4)	// Receive FIFO empty

/*
 * Bound on the wait for FIFO space, in loop iterations.  Generous for a
 * 115200 baud character (~87 us) yet still finite, so a UART that is
 * unclocked or held in reset costs a bounded delay and a dropped byte
 * instead of hanging the system.
 */
#define UART_TX_SPIN_LIMIT	200000

#if TM_CONSOLE_UART
LOCAL void uart_snd_dat(const UB* buf, INT size)
{
	UB	*b;
	UW	spin;

	for( b = (UB *)buf; size > 0; size--, b++ ){
		spin = UART_TX_SPIN_LIMIT;
		while( (UART_FR & FR_TXFF) != 0 ){
			if( --spin == 0 ) return;	/* give up, drop the rest */
		}
		UART_DR = *b;
	}
}
#endif /* TM_CONSOLE_UART */

#if TM_CONSOLE_USB_CDC
IMPORT void tm_usb_console_put(const UB *buf, INT size);
#endif

EXPORT	void	tm_snd_dat( const UB* buf, INT size )
{
#if TM_CONSOLE_USB_CDC
	tm_usb_console_put(buf, size);
#endif
#if TM_CONSOLE_UART
	uart_snd_dat(buf, size);
#endif
}

EXPORT	void	tm_rcv_dat( UB* buf, INT size )
{
#if TM_CONSOLE_UART
	for( ; size > 0; size--, buf++ ){
		while ( (UART_FR & FR_RXFE) != 0 );
		*buf = UART_DR & 0xff;
	}
#else
	/* Console input is not implemented on the USB-only configuration. */
	for( ; size > 0; size--, buf++ ){
		*buf = 0;
	}
#endif
}

EXPORT	void	tm_com_init(void)
{
#if TM_CONSOLE_UART
	UART_IBRD	= 67;			/* Baud rate setting */
	UART_FBRD	= 52;
	UART_LCR_H	= 0x70;			/* Communication data format setting */
	UART_CR 	= (1<<9)|(1<<8)|(1<<0);	/* Communication enabled */
#endif
	/* The USB side needs the scheduler, so it is started later from
	   knl_start_device(); nothing to do here. */
}

#endif /* PICO_RP2040 */
#endif /* USE_TMONITOR */
