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
#define OS_INVALID		(-1)	/* range value for "no echo": unknown, NOT free */
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
