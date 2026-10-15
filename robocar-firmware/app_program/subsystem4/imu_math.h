/*
 *	imu_math.h
 *	Small integer maths helpers (Buddy 4).  The template does not link the
 *	C maths library and the RP2040 has no floating-point unit, so angles
 *	are integers in centidegrees (1/100 degree) and sines are x 10 000.
 *	Pure C: also compiles on a laptop for testing.
 */

#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>

/* Integer square root */
uint32_t imu_isqrt(uint32_t x);

/* atan2(y, x) in centidegrees, -18000..18000.  Error below 0.01 degree
   (table of 65 points with straight lines between them). */
int32_t imu_atan2_cdeg(int32_t y, int32_t x);

/* sin of an angle in centidegrees, x 10 000, for -9000..9000
   (table every 2 degrees, straight lines between them). */
int32_t imu_sin_x1e4(int32_t cdeg);

#endif /* IMU_MATH_H */
