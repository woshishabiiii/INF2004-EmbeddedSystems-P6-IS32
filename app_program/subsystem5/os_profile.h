/*
 *	os_profile.h
 *	Obstacle profiling (Stage 3, Buddy 5): turns one servo scan (angles
 *	and ranges) into the obstacle's position, width, closest point and the
 *	free gap on each side.  Pure C, unit-tested on a laptop.
 *
 *	Geometry, seen from the ultrasonic sensor:
 *	  angle 90 = straight ahead, < 90 = to the right, > 90 = to the left;
 *	  x = sideways position, mm (right +, left -);
 *	  y = forward distance, mm.
 *	The HC-SR04 beam is about 15 degrees wide, so an obstacle looks a
 *	little wider than it is: the profile errs on the safe side.
 */

#ifndef OS_PROFILE_H
#define OS_PROFILE_H

#include <stdint.h>
#include <stdbool.h>

#define OS_MAX_POINTS		64U
#define OS_INVALID		4095	/* range value for "no echo": unknown, NOT free.
					   Ranges are 12-bit unsigned (0..4,000 mm), so
					   "none" is the all-ones value, not -1. */
#define OS_CLEAR_OPEN		1000	/* clearance when nothing was seen on that side, mm */

typedef struct {
	bool		valid;		/* an obstacle was found */
	int32_t		x_left_mm;	/* left edge (most negative x) */
	int32_t		x_right_mm;	/* right edge */
	int32_t		width_mm;	/* x_right - x_left */
	int32_t		x_center_mm;	/* location: sideways ... */
	int32_t		closest_mm;	/* ... and forward distance of its nearest point */
	int32_t		closest_deg;	/* angle of that point */
	int32_t		left_clear_mm;	/* free gap beyond the left edge */
	int32_t		right_clear_mm;	/* free gap beyond the right edge */
	uint32_t	unknown_left;	/* no-echo readings beyond the left edge */
	uint32_t	unknown_right;	/* ... beyond the right edge */
	uint32_t	npoints;	/* valid readings used */
} OS_PROFILE;

/* The profile, packed (48 B -> 13 B), as Obstacle Scan shares it with
   Vehicle Controller and Telemetry under mtx_profile.  97 bits, LSB first
   (bitpack.h); signed values with an offset; values outside saturate.
     valid            1
     x_left_mm       11   -1024..1023  (|x| <= OS_R_MAX_MM = 800)
     x_right_mm      11   -1024..1023
     width_mm        11   0..2047
     x_center_mm     11   -1024..1023
     closest_mm      10   0..1023      (farther than 800 is background)
     closest_deg      7   30..157      (servo angles 30..150), stored - 30
     left_clear_mm   10   0..1023      (OS_CLEAR_OPEN = 1000)
     right_clear_mm  10   0..1023
     unknown_left     5   0..31        } a scan gives at most SCAN_SLOTS = 25
     unknown_right    5   0..31        } points (scan_core.h; scan_core.c
     npoints          5   0..31        } checks it when it compiles)        */
#define OS_PROFILE_PK_BYTES	13U
#define OS_PK_COUNT_BITS	5U	/* unknown_left, unknown_right, npoints */

typedef struct {
	uint8_t		b[OS_PROFILE_PK_BYTES];
} OS_PROFILE_PK;

void	os_profile_pack(const OS_PROFILE *p, OS_PROFILE_PK *pk);
void	os_profile_unpack(const OS_PROFILE_PK *pk, OS_PROFILE *p);

/* Points must be sorted by angle (ascending).  Ranges beyond r_max_mm are
   background, not obstacle; OS_INVALID ranges are unknown.
   The obstacle is grown from the nearest point inside the path corridor
   (|x| <= corridor_mm, the car's half-width plus margin), so a wall or a
   second object beside the path is not mistaken for the obstacle ahead;
   only if nothing lies in the corridor is the nearest point overall used. */
void	os_build_profile(const int32_t angle_deg[], const int32_t range_mm[],
			 uint32_t n, int32_t r_max_mm, int32_t corridor_mm, OS_PROFILE *p);

/* cos and sin of a whole-degree angle 0..180, x 1000 */
int32_t	os_cos1000(int32_t deg);
int32_t	os_sin1000(int32_t deg);

#endif /* OS_PROFILE_H */
