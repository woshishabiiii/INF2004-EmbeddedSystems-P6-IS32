/*
 *	imu_math.c  (see imu_math.h)
 */

#include "imu_math.h"

#define ATAN_STEPS	64	/* table covers atan(0/64) .. atan(64/64) */
#define CDEG_90		9000
#define CDEG_180	18000
#define SIN_STEP_CDEG	200	/* the sine table has one entry per 2 degrees */

/* atan(i / 64) in centidegrees, i = 0..64 */
static const int16_t s_atan[ATAN_STEPS + 1] = {
	   0,   90,  179,  268,  358,  447,  536,  624,  713,  800,
	 888,  975, 1062, 1148, 1234, 1319, 1404, 1488, 1571, 1653,
	1735, 1817, 1897, 1977, 2056, 2134, 2211, 2287, 2363, 2438,
	2511, 2584, 2657, 2728, 2798, 2867, 2936, 3003, 3070, 3136,
	3201, 3264, 3327, 3390, 3451, 3511, 3571, 3629, 3687, 3744,
	3800, 3855, 3909, 3963, 4016, 4067, 4119, 4169, 4218, 4267,
	4315, 4363, 4409, 4455, 4500
};

/* sin(2 d degrees) x 10 000, d = 0..45 */
static const int16_t s_sin[46] = {
	    0,  349,  698, 1045, 1392, 1736, 2079, 2419, 2756, 3090,
	 3420, 3746, 4067, 4384, 4695, 5000, 5299, 5592, 5878, 6157,
	 6428, 6691, 6947, 7193, 7431, 7660, 7880, 8090, 8290, 8480,
	 8660, 8829, 8988, 9135, 9272, 9397, 9511, 9613, 9703, 9781,
	 9848, 9903, 9945, 9976, 9994, 10000
};

uint32_t imu_isqrt(uint32_t x)
{
	uint32_t r = 0U;
	uint32_t bit = 1UL << 30;

	while (bit > x) {
		bit >>= 2;
	}
	while (bit != 0U) {
		if (x >= (r + bit)) {
			x -= r + bit;
			r = (r >> 1) + bit;
		} else {
			r >>= 1;
		}
		bit >>= 2;
	}
	return r;
}

/* atan(n / d) for 0 <= n <= d, d > 0, in centidegrees */
static int32_t atan_unit(uint32_t n, uint32_t d)
{
	uint32_t q, i, f;

	/* z = n / d in 1/65536 steps; n, d fit in 31 bits, so scale d down
	   first if n << 16 would overflow */
	while (n > 0xFFFFU) {
		n >>= 1;
		d >>= 1;
	}
	if (d == 0U) {
		return CDEG_90 / 2;
	}
	q = (n << 16) / d;			/* 0..65536 */
	i = q >> 10;				/* table index, 0..64 */
	f = q & 0x3FFU;				/* fraction between entries, /1024 */
	if (i >= (uint32_t)ATAN_STEPS) {
		return s_atan[ATAN_STEPS];
	}
	return s_atan[i] + (int32_t)(((uint32_t)(s_atan[i + 1U] - s_atan[i]) * f) >> 10);
}

int32_t imu_atan2_cdeg(int32_t y, int32_t x)
{
	uint32_t ay = (uint32_t)((y < 0) ? -y : y);
	uint32_t ax = (uint32_t)((x < 0) ? -x : x);
	int32_t a;

	if ((ax == 0U) && (ay == 0U)) {
		return 0;
	}
	if (ay <= ax) {
		a = atan_unit(ay, ax);			/* 0..45 degrees */
	} else {
		a = CDEG_90 - atan_unit(ax, ay);	/* 45..90 degrees */
	}
	if (x < 0) {
		a = CDEG_180 - a;			/* second quadrant */
	}
	return (y < 0) ? -a : a;
}

int32_t imu_sin_x1e4(int32_t cdeg)
{
	int32_t neg = (cdeg < 0) ? 1 : 0;
	int32_t a = neg ? -cdeg : cdeg;
	int32_t i, f, v;

	if (a > CDEG_90) {
		a = CDEG_90;
	}
	i = a / SIN_STEP_CDEG;
	f = a % SIN_STEP_CDEG;
	if (i >= 45) {
		v = s_sin[45];
	} else {
		v = s_sin[i] + (((s_sin[i + 1] - s_sin[i]) * f) / SIN_STEP_CDEG);
	}
	return neg ? -v : v;
}
