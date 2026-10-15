/*
 *	os_profile.c  (see os_profile.h)
 */

#include "os_profile.h"

#define CLUSTER_DR_MM	80	/* neighbours closer than this in range = same object */
#define LOOKAHEAD_MM	300	/* only objects this far past the nearest point limit a gap */

/* sin(d) x 1000 for d = 0, 5, 10 ... 90 */
static const int16_t s_sin5[19] = {
	0, 87, 174, 259, 342, 423, 500, 574, 643, 707,
	766, 819, 866, 906, 940, 966, 985, 996, 1000
};

int32_t os_sin1000(int32_t deg)
{
	int32_t d = (deg > 90) ? (180 - deg) : deg;	/* sin(180 - d) = sin d */
	int32_t i, f;

	if (d < 0) {
		d = 0;
	}
	i = d / 5;
	f = d % 5;
	if (i >= 18) {
		return 1000;
	}
	return s_sin5[i] + (((s_sin5[i + 1] - s_sin5[i]) * f) / 5);	/* straight line between entries */
}

int32_t os_cos1000(int32_t deg)
{
	return (deg <= 90) ? os_sin1000(90 - deg) : -os_sin1000(deg - 90);
}

static bool same_object(int32_t r1, int32_t r2)
{
	int32_t d = r1 - r2;

	return (d < CLUSTER_DR_MM) && (d > -CLUSTER_DR_MM);
}

void os_build_profile(const int32_t angle_deg[], const int32_t range_mm[],
		      uint32_t n, int32_t r_max_mm, int32_t corridor_mm, OS_PROFILE *p)
{
	int32_t		x[OS_MAX_POINTS], y[OS_MAX_POINTS];
	bool		ok[OS_MAX_POINTS], in_obj[OS_MAX_POINTS];
	uint32_t	i, k = 0U, lo, hi;
	int32_t		ymin = INT32_MAX;
	int32_t		ymin_path = INT32_MAX;
	uint32_t	k_path = 0U;

	*p = (OS_PROFILE){0};
	if (n > OS_MAX_POINTS) {
		n = OS_MAX_POINTS;
	}
	for (i = 0U; i < n; i++) {
		ok[i] = (range_mm[i] != OS_INVALID) && (range_mm[i] >= 0) && (range_mm[i] <= r_max_mm);
		in_obj[i] = false;
		x[i] = (range_mm[i] * os_cos1000(angle_deg[i])) / 1000;
		y[i] = (range_mm[i] * os_sin1000(angle_deg[i])) / 1000;
		if (ok[i]) {
			p->npoints++;
			if (y[i] < ymin) {
				ymin = y[i];
				k = i;
			}
			if ((x[i] <= corridor_mm) && (x[i] >= -corridor_mm) && (y[i] < ymin_path)) {
				ymin_path = y[i];
				k_path = i;
			}
		}
	}
	if (p->npoints == 0U) {
		return;			/* nothing seen: NOT proof of a clear path */
	}
	if (ymin_path != INT32_MAX) {
		k = k_path;		/* the obstacle in the path, not beside it */
	}

	/* Grow the object from the nearest point while neighbours agree.  One
	   missing echo between two matching points does not split the object
	   (an angled face can scatter one ping): it would look narrower. */
	lo = k;
	hi = k;
	in_obj[k] = true;
	while (lo > 0U) {
		if (ok[lo - 1U] && same_object(range_mm[lo - 1U], range_mm[lo])) {
			lo--;
		} else if ((lo > 1U) && (range_mm[lo - 1U] == OS_INVALID) &&
			   ok[lo - 2U] && same_object(range_mm[lo - 2U], range_mm[lo])) {
			in_obj[lo - 1U] = true;
			lo -= 2U;
		} else {
			break;
		}
		in_obj[lo] = true;
	}
	while ((hi + 1U) < n) {
		if (ok[hi + 1U] && same_object(range_mm[hi + 1U], range_mm[hi])) {
			hi++;
		} else if (((hi + 2U) < n) && (range_mm[hi + 1U] == OS_INVALID) &&
			   ok[hi + 2U] && same_object(range_mm[hi + 2U], range_mm[hi])) {
			in_obj[hi + 1U] = true;
			hi += 2U;
		} else {
			break;
		}
		in_obj[hi] = true;
	}

	p->x_left_mm = INT32_MAX;
	p->x_right_mm = INT32_MIN;
	for (i = lo; i <= hi; i++) {
		if (!ok[i]) {
			continue;
		}
		if (x[i] < p->x_left_mm) {
			p->x_left_mm = x[i];
		}
		if (x[i] > p->x_right_mm) {
			p->x_right_mm = x[i];
		}
	}
	ymin = y[k];
	for (i = lo; i <= hi; i++) {
		if (ok[i] && (y[i] < ymin)) {
			ymin = y[i];
			k = i;
		}
	}
	p->width_mm = p->x_right_mm - p->x_left_mm;
	p->x_center_mm = (p->x_left_mm + p->x_right_mm) / 2;
	p->closest_mm = ymin;
	p->closest_deg = angle_deg[k];

	/* Clearance: nearest OTHER object beyond each edge, within the
	   lookahead.  Larger angles are to the left (x more negative). */
	p->left_clear_mm = OS_CLEAR_OPEN;
	p->right_clear_mm = OS_CLEAR_OPEN;
	for (i = 0U; i < n; i++) {
		if (in_obj[i]) {
			continue;
		}
		if (range_mm[i] == OS_INVALID) {
			if (i > hi) {
				p->unknown_left++;	/* no echo: we cannot tell */
			} else if (i < lo) {
				p->unknown_right++;
			} else {
				/* inside the object's angle range */
			}
			continue;
		}
		if (!ok[i] || (y[i] > (ymin + LOOKAHEAD_MM))) {
			continue;			/* background */
		}
		if ((x[i] < p->x_left_mm) && ((p->x_left_mm - x[i]) < p->left_clear_mm)) {
			p->left_clear_mm = p->x_left_mm - x[i];
		}
		if ((x[i] > p->x_right_mm) && ((x[i] - p->x_right_mm) < p->right_clear_mm)) {
			p->right_clear_mm = x[i] - p->x_right_mm;
		}
	}
	p->valid = true;
}
