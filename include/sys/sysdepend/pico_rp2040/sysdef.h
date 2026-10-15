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
 *	sysdef.h
 *
 *	System dependencies definition (RaspberryPi Pico depended)
 *	Included also from assembler program.
 */

#ifndef __SYS_SYSDEF_DEPEND_H__
#define __SYS_SYSDEF_DEPEND_H__


/* CPU-dependent definition */
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

/* ------------------------------------------------------------------------ */
/*
 * Clock control definition
 */

/* RCC register initial value */

/* Clock frequency 　*/
#define	KHz			(1000)
#define	MHz			(KHz*1000)

#define	SYSCLK			(125)		/* System clock */
#define	TMCLK_MHz		(125)
#define	TMCLK_KHz		(TMCLK_MHz*1000)

#define	XOSC_MHz		(12)
#define	XOSC_KHz		(XOSC_MHz*1000)
#define XOSC_STARTUP_DELAY	((XOSC_KHz + 128) / 256)

#define	CLK_PLL_SYS_FREQ	(TMCLK_MHz*MHz)
#define	CLK_PLL_USB_FREQ	(48*MHz)

#define	CLK_USB_FREQ		(48*MHz)
#define	CLK_ADC_FREQ		(48*MHz)
#define	CLK_RTC_FREQ		(46875)
#define	CLK_PERI_FREQ		(125*MHz)

#define	CLK_USB_SRC		0	// clksrc_pll_usb
#define	CLK_ADC_SRC		0	// clksrc_pll_usb
#define	CLK_RTC_SRC		0	// clksrc_pll_usb
#define	CLK_PERI_SRC		0	// clk_sys


/* ------------------------------------------------------------------------ */
/*
 * Board pin assignment
 *
 * Target board is the Raspberry Pi Pico W.  Its on-board LED is not on an
 * RP2040 pin at all: it hangs off the CYW43439 radio's own GPIO and needs the
 * full cyw43 driver to reach, so this port drives an external LED instead.
 *
 * The following RP2040 pins are wired to the CYW43439 on the Pico W and must
 * not be claimed as general-purpose I/O.  On the non-W Pico, GP25 is the LED
 * and these are free.
 *
 *	GP23	WL_ON	radio power enable (left low: radio held off)
 *	GP24	WL_D	radio SPI data
 *	GP25	WL_CS	radio chip select
 *	GP29	WL_CLK	radio SPI clock (also ADC3 / VSYS sense)
 *
 * GP0/GP1 carry UART0 (console copy + crash report) only when built with
 * "make CRASH_REPORT=1" or CONSOLE=uart.  By default the build defines
 * BOARD_UART0_FREE=1 and the console is on USB only, leaving them free.
 *
 * No other pin is configured at boot.  (The Pico W's on-board LED is on the
 * radio, so applications that want an LED wire an external one to a free
 * pin and set it up themselves with gpio_set_pin().)
 */
#ifndef BOARD_UART0_FREE
#define	BOARD_UART0_FREE	0	/* 1: boot leaves GP0/GP1 alone */
#endif

/*
 * Startup checkpoint diagnostic.
 *
 * Set BOARD_DIAG_GPIO_SCAN to 1 to flash a numbered checkpoint on
 * BOARD_DIAG_GPIO_MASK at each stage of startup, using nothing but register
 * writes and stack locals.  It works before the vector table is relocated,
 * before .data is loaded, before .bss is cleared and before any console
 * exists, which is precisely the window where no other instrumentation is
 * available.
 *
 * This located the unaligned .data load that HardFaulted every boot of the
 * stock port (see docs/evidence/phase0/).  It is retained, disabled, because
 * Phase 3 core-1 bring-up has the same problem: the second core comes up with
 * no console and no scheduler, so a GPIO checkpoint is the only signal
 * available.
 *
 * The mask covers GP2..GP22 and GP26..GP28: everything except the UART0
 * console (GP0/GP1) and the CYW43439 radio pins (GP23/24/25/29).
 */
#define	BOARD_DIAG_GPIO_SCAN	0
#define	BOARD_DIAG_GPIO_MASK	0x1C7FFFFCUL

/*
 * Maximum value of Power-saving mode switching prohibition request.
 * Use in tk_set_pow API.
 */
#define LOWPOW_LIMIT	0x7fff		/* Maximum number for disabling */

#endif /* __TK_SYSDEF_DEPEND_H__ */
