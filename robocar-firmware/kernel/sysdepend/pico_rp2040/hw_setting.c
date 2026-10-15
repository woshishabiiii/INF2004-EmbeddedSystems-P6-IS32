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
 *	hw_setting.c (RaspberryPi Pico)
 *	startup / shoutdown processing for hardware
 *	
 *	Pin function Setting (for RaspberryPi Pico W)
 *		P0   : UART0-TX	(only with make CRASH_REPORT=1 or CONSOLE=uart)
 *		P1   : UART0-RX
 *
 *	Only the console pins are set up at boot.  Every other GPIO is left
 *	in its reset state for the application (or a driver it enables) to
 *	configure.  GP23/24/25/29 belong to the CYW43439 radio on the Pico W
 *	and are deliberately left alone here; see the board sysdef.h.
 */

#include <kernel.h>
#include <tm/tmonitor.h>

#include "sysdepend.h"

/* 
 * Setup register data 
 */
typedef struct {
	UW	addr;
	UW	data;
} T_SETUP_REG;

/*
 * Release peripheral’s reset
 */
LOCAL const UW module_tbl[] = {
	RESETS_RESET_IO_BANK0,		/* IO_BANK0 */
	RESETS_RESET_PADS_BANK0,	/* PADS_BANK0 */
	RESETS_RESET_UART0,		/* UART0 */

#if TM_WIFI_CYW43
	/* The Pico W radio transport runs a PIO state machine with two DMA
	 * channels.  Unlike a pico-sdk application, micro T-Kernel performs its
	 * own selective peripheral reset release, so these blocks must be made
	 * live explicitly before the CYW43439 service starts. */
	RESETS_RESET_PIO0,		/* CYW43439 PIO-SPI */
	RESETS_RESET_PIO1,		/* PIO allocator may select either instance */
	RESETS_RESET_DMA,		/* CYW43439 PIO-SPI transfers */
#endif

#if USE_PTMR
	RESETS_RESET_PWM,		/* PWM */
#endif	/* USE_PTMR */

#if USE_SDEV_DRV	// Do not use sample device driver
	RESETS_RESET_ADC,		/* A/DC */
	RESETS_RESET_I2C0,		/* I2C0 */
#endif /* USE_SDEV_DRV */
	0
};

/* 
 * Setup pin functions Tadle
 */
LOCAL const T_SETUP_REG pinfnc_tbl[] = {
#if !BOARD_UART0_FREE
	/* P0,P1 : UART0 console mirror */
	{GPIO_CTRL(0),	GPIO_CTRL_FUNCSEL_UART},	/* P0 UART0-TX */
	{GPIO_CTRL(1),	GPIO_CTRL_FUNCSEL_UART},	/* P1 UART0-RX */
#endif

	/* ADC (GP26-28) and I2C pins are not set up here: their drivers
	   claim their own pins when configured to.  See config_device.h,
	   adc_cnf_sysdep.h and i2c_cnf_sysdep.h. */
	{0, 0}
};

#if BOARD_DIAG_GPIO_SCAN
/*
 * Phase-0 bring-up diagnostic.
 *
 * Runs at the end of knl_startup_hw(), which is the first thing the reset
 * handler calls -- before the vector table is copied, before .data is loaded
 * and before .bss is cleared.  Everything here is therefore register writes
 * and stack locals only.
 *
 * Emits three ~200 ms pulses on BOARD_DIAG_GPIO_MASK and leaves the pins
 * driven low.  Seeing these three pulses but not the application's continuous
 * blink means the board and the flash image are fine and the fault is in
 * kernel startup.
 */
/* Rough busy-wait.  The loop is about 10 cycles at 125 MHz; this only has to
   be legible to the eye, so it is not calibrated. */
#define DIAG_LOOPS_PER_MS	12500

LOCAL void diag_delay_ms(UW ms)
{
	volatile UW	i;
	UW		loops = ms * DIAG_LOOPS_PER_MS;

	for(i = 0; i < loops; i++) {
	}
}

LOCAL void diag_claim(void)
{
	UW	pin;

	for(pin = 0; pin < GPIO_NUM; pin++) {
		if((BOARD_DIAG_GPIO_MASK & (1UL<<pin)) == 0) continue;
		out_w(GPIO(pin), GPIO_DRIVE_4MA | GPIO_SHEMITT);  /* output, no pulls */
		out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
	}
	out_w(GPIO_OE_SET, BOARD_DIAG_GPIO_MASK);
}

LOCAL void diag_flash(UW on_ms, UW off_ms)
{
	out_w(GPIO_OUT_SET, BOARD_DIAG_GPIO_MASK);
	diag_delay_ms(on_ms);
	out_w(GPIO_OUT_CLR, BOARD_DIAG_GPIO_MASK);
	diag_delay_ms(off_ms);
}

/*
 * Emit checkpoint number 'n', then a long gap.
 *
 * Counted as one long flash per five plus one short flash per remaining
 * unit, so a two-digit ladder stays countable by eye:
 *
 *	3  = short short short
 *	5  = LONG
 *	7  = LONG short short
 *	8  = LONG short short short
 *
 * Callers may run before .data is loaded and .bss cleared, so this path
 * touches no globals -- register writes and stack locals only.
 */
EXPORT void knl_diag_pulse(INT n)
{
	INT	i;

	diag_claim();

	for(i = 0; i < n / 5; i++) {
		diag_flash(600, 300);
	}
	for(i = 0; i < n % 5; i++) {
		diag_flash(120, 280);
	}
	diag_delay_ms(1800);
}

/* Start-of-ladder marker: eight fast flashes, so the checkpoint sequence
   that follows has an unmistakable beginning. */
EXPORT void knl_diag_attention(void)
{
	INT	i;

	diag_claim();
	for(i = 0; i < 8; i++) {
		diag_flash(60, 60);
	}
	diag_delay_ms(1500);
}
#endif /* BOARD_DIAG_GPIO_SCAN */

/*
 * Startup Device
 */
EXPORT void knl_startup_hw(void)
{
	const T_SETUP_REG	*p;
	UW	rst;

	/* Startup System Clock */
	startup_clock(CLKATR_USB | CLKATR_ADC | CLKATR_RTC | CLKATR_PREI);

	for(INT i = 0; (rst = module_tbl[i]); i++) {
		set_w(RESETS_RESET, rst);
		clr_w(RESETS_RESET, rst);
		while((in_w(RESETS_RESET_DONE) & rst)==0);
	}

	for(p = pinfnc_tbl; p->addr != 0; p++) {
		out_w(p->addr, p->data);
	}

#if BOARD_DIAG_GPIO_SCAN
	knl_diag_attention();		/* start of ladder */
	knl_diag_pulse(1);		/* CP1: hardware startup complete */
#endif
}

#if USE_SHUTDOWN
/*
 * Shutdown device
 */
EXPORT void knl_shutdown_hw( void )
{
	disint();
	while(1);
}
#endif /* USE_SHUTDOWN */

/*
 * Re-start device
 *	mode = -1		reset and re-start	(Reset -> Boot -> Start)
 *	mode = -2		fast re-start		(Start)
 *	mode = -3		Normal re-start		(Boot -> Start)
 */
EXPORT ER knl_restart_hw( W mode )
{
	switch(mode) {
	case -1: /* Reset and re-start */
		SYSTEM_MESSAGE("\n<< SYSTEM RESET & RESTART >>\n");
		return E_NOSPT;
	case -2: /* fast re-start */
		SYSTEM_MESSAGE("\n<< SYSTEM FAST RESTART >>\n");
		return E_NOSPT;
	case -3: /* Normal re-start */
		SYSTEM_MESSAGE("\n<< SYSTEM RESTART >>\n");
		return E_NOSPT;
	default:
		return E_PAR;
	}
}


#endif /* PICO_RP2040 */
