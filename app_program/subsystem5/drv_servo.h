/*
 *	drv_servo.h
 *	SG90 scan servo on GP12 (Robo Pico servo port 1), PWM slice 6 A,
 *	50 Hz.  Called only from the Obstacle Scan task.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include "../app.h"

/* Calibrated pulse widths (measure them).  The SG90 nominally spans about
   500-2400 us; never drive it into its end stops.
   30 deg must point the sensor to the car's RIGHT and 150 deg to its LEFT
   (the scan geometry in os_profile.h).  If yours turns the other way,
   swap the two values. */
#define SERVO_US_AT_30DEG	2130	/* MEASURE */
#define SERVO_US_AT_150DEG	870	/* MEASURE */
#define SERVO_US_MIN		600	/* hard safety limits */
#define SERVO_US_MAX		2400

IMPORT ER	servo_init(void);		/* centre (90 deg) */
IMPORT void	servo_set_us(W pulse_us);	/* clamped to MIN..MAX */
IMPORT void	servo_set_deg(W deg);		/* 30..150, linear between the two calibration points */

#endif /* DRV_SERVO_H */
