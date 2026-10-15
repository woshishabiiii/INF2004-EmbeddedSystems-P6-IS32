/*
 *	drv_motor.h
 *	Robo Pico DC motor driver: two channels, 16 kHz PWM.
 *	Called ONLY from the Motion Control task (TEAM.md: owner of PWM
 *	slices 4 and 5 and pins GP8-GP11).
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "../app.h"

#define MOTOR_LEFT		0U	/* M2 terminal: GP10 (A), GP11 (B), slice 5 */
#define MOTOR_RIGHT		1U	/* M1 terminal: GP8 (A),  GP9 (B),  slice 4 */
#define MOTOR_DUTY_MAX		1000	/* duty is in permille: 1000 = 100 % */

/* Release the PWM block from reset, set slices 4 and 5 to 16 kHz and
   leave both motors braked (duty 0). */
IMPORT ER	motor_init(void);

/* duty: -1000..+1000 permille.  + = forward, - = backward, 0 = brake.
   Values outside the range are clamped. */
IMPORT void	motor_set(UINT side, W duty);

/* Both channels to duty 0 (A = B = low: brake). */
IMPORT void	motor_brake_all(void);

#endif /* DRV_MOTOR_H */
