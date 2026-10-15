/*
 *----------------------------------------------------------------------
 *    Device Driver for μT-Kernel 3.0
 *
 *    Copyright (C) 2022-2023 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2023/05.
 *
 *----------------------------------------------------------------------
 */

/*
 *	adc_cnf_sysdep.h 
 *	A/D converter device driver configuration file
 *		for RP2040
 */
#ifndef	__DEV_ADC_CNF_RP2040_H__
#define	__DEV_ADC_CNF_RP2040_H__

/* 
 * Release A/DC reset
 *	Enable when not performed in the OS initialization process.
 */
#define	DEVCONF_ADC_REL_RESET	FALSE

/*
 * Device control data
 */
#define	ADC_DIV_INI		0	// Clock divider

/* 
 * Initialize analog input pins
 *	The kernel does not set up any ADC pin at boot.  Set the flag to TRUE
 *	for each channel you use: the driver then makes that pin an analog
 *	input (digital input and pulls off) when it starts.  Record the pin
 *	in TEAM.md.
 *	  channel 0 = GP26, channel 1 = GP27, channel 2 = GP28
 *	  channel 3 = GP29 is the Pico W RADIO CLOCK: never set _3 to TRUE.
 */
#define	DEVCONF_ADC_PIN_INIT_0	TRUE	/* team: barcode sensor AO on GP26 */
#define	DEVCONF_ADC_PIN_INIT_1	FALSE
#define	DEVCONF_ADC_PIN_INIT_2	FALSE
#define	DEVCONF_ADC_PIN_INIT_3	FALSE

/* Interrupt t priority */
#define	DEVCNF_ADC_INTPRI	2

/* A/D conversion timeout time */
#define	DEVCNF_ADC_TMOSCAN	1000

#endif		/* __DEV_ADC_CNF_RP2040_H__ */
