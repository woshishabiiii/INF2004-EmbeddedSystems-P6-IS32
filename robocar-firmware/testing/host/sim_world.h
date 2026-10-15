/*
 *	sim_world.h
 *	The simulated world for the host simulator: track (black line
 *	segments), barcodes, humps, obstacles, and the car's physics and
 *	sensors.  Host-only code (doubles are fine here).
 */

#ifndef SIM_WORLD_H
#define SIM_WORLD_H

#include <stdint.h>
#include <stdbool.h>

#define W_MAX_SEGS	600
#define W_MAX_BARS	64
#define W_MAX_HUMPS	4
#define W_MAX_BOXES	6

typedef struct { double x1, y1, x2, y2; } W_SEG;		/* black line, 18 mm wide */
typedef struct { double cx, cy, ux, uy, hl, hw; } W_RECT;	/* centre, unit axis, half length/width */
typedef struct { double x0, len, h; } W_HUMP;			/* along x: raised cosine */
typedef struct { double x1, y1, x2, y2; bool visible; } W_BOX;	/* axis-aligned obstacle */

typedef struct {
	W_SEG	seg[W_MAX_SEGS];
	int	nseg;
	W_RECT	bar[W_MAX_BARS];
	int	nbar;
	W_HUMP	hump[W_MAX_HUMPS];
	int	nhump;
	W_BOX	box[W_MAX_BOXES];
	int	nbox;
} WORLD;

/* car geometry and "true" physical constants (deliberately a little
   different from the firmware's configured values) */
typedef struct {
	double	x, y, th;		/* axle centre, heading (rad, CCW from +x) */
	double	v[2];			/* wheel speeds mm/s, [0] left */
	double	cnt[2];			/* encoder counts (fractional) */
	double	duty[2];		/* permille, from the firmware */
	double	servo_deg;		/* actual servo angle */
	double	servo_cmd;
	double	pitch;			/* rad, nose up + */
	double	a_long;			/* mm/s^2 */
	bool	crashed;
	int	crash_count;
	double	impact_ms;		/* > 0: jolt still being felt */
	double	odo;
} CAR;

/* world building */
void	w_init(WORLD *w);
void	w_line(WORLD *w, double x1, double y1, double x2, double y2);
void	w_arc(WORLD *w, double cx, double cy, double r, double a0_deg, double a1_deg);
/* barcode "*c*" beside the straight from (x,y) heading a_deg, on the
   right of that heading (or the left: then a car driving the other way
   reads it back to front) */
bool	w_barcode(WORLD *w, double x, double y, double a_deg, char c, bool left);
void	w_hump(WORLD *w, double x0, double len, double h);
void	w_box(WORLD *w, double x1, double y1, double x2, double y2, bool visible);

bool	w_black(const WORLD *w, double px, double py);
double	w_height(const WORLD *w, double px, double py);
double	w_line_dist(const WORLD *w, double px, double py);	/* to the nearest segment centre line */

/* car */
void	car_init(CAR *c, double x, double y, double th_deg);
void	car_step(CAR *c, const WORLD *w, double dt_s);
void	car_point(const CAR *c, double fwd, double left, double *px, double *py);
bool	car_sensor_black(const CAR *c, const WORLD *w, double fwd, double left);
int32_t	car_count(const CAR *c, int i);

/* ultrasonic: range in mm from the sensor, or -1 when no echo comes back */
double	us_range(const CAR *c, const WORLD *w);

/* accelerometer reading in mg (X fwd, Y left, Z up of the IMU board) */
void	imu_read(const CAR *c, double noise_mg, int32_t a[3]);

/* geometry (mm) shared with the simulator */
#define CAR_LINE_FWD	60.0	/* line sensors ahead of the axle */
#define CAR_LINE_HALF	12.5	/* half the line sensor spacing */
#define CAR_BC_LEFT	(-42.0)	/* barcode sensor: 42 mm to the right */
#define CAR_US_FWD	80.0	/* ultrasonic ahead of the axle */
#define CAR_CASTOR_FWD	120.0	/* castor ahead of the axle */
#define CAR_FRONT	100.0	/* bumper ahead of the axle */
#define CAR_REAR	60.0	/* back behind the axle */
#define CAR_HALF_W	80.0	/* half the width */
#define TRUE_TRACK_MM	133.0	/* the firmware assumes 130 */
#define TRUE_COUNTS_MM	2.0	/* the firmware assumes 2000 / m */

/* the "true" car, changeable for robustness runs (sim -p) */
extern double g_db_true, g_kff_true, g_track_true, g_counts_mm, g_right_eff;

#endif
