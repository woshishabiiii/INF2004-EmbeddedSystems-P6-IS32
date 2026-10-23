/*
 *	bitpack.h
 *	Minimum-bit data representation (change requirement, Tribunal 1):
 *	the helpers every packed layout in the firmware is built from.
 *	Team agreement, like app.h.  Pure C (stdint only), header-only.
 *
 *	Rules for every packed field (Barr-C: no C bit-fields, whose order and
 *	padding are up to the compiler; explicit masks and shifts instead):
 *	  - a field holds only the bits its valid range needs;
 *	  - an unsigned field SATURATES at its largest value instead of
 *	    wrapping (5,000 mm in 12 bits must not become 904 mm);
 *	  - a signed field is stored with an offset, so every field on the
 *	    wire and in RAM is unsigned: stored = value - min;
 *	  - a time or counter field that wraps is only ever SUBTRACTED
 *	    (bp_age16(), bp_dt31()), never compared with "greater than".
 *
 *	Records of several fields are laid out LSB first in a byte array
 *	(bp_put / bp_get): bit 0 of the record is bit 0 of byte 0, so the
 *	order does not depend on the CPU or the compiler.  The laptop decoder
 *	(tools/mqtt_decode.py) reads the MQTT records the same way.
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <stdint.h>

/* Largest value of an n-bit unsigned field, n = 1..32 */
#define BP_MAX(n)	((uint32_t)((n) >= 32U ? 0xFFFFFFFFU : ((1UL << (n)) - 1UL)))

/* Unsigned value -> n-bit field, saturating (negative -> 0) */
static inline uint32_t bp_sat_u(int32_t v, uint32_t n)
{
	if (v <= 0) {
		return 0U;
	}
	return ((uint32_t)v > BP_MAX(n)) ? BP_MAX(n) : (uint32_t)v;
}

/* Signed value in [min, min + 2^n - 1] -> n-bit field (value - min),
   saturating at both ends */
static inline uint32_t bp_sat_s(int32_t v, int32_t min, uint32_t n)
{
	int64_t d = (int64_t)v - (int64_t)min;

	if (d <= 0) {
		return 0U;
	}
	return (d > (int64_t)BP_MAX(n)) ? BP_MAX(n) : (uint32_t)d;
}

/* n-bit field stored with offset min -> signed value */
static inline int32_t bp_from_s(uint32_t f, int32_t min)
{
	return (int32_t)((int64_t)f + (int64_t)min);
}

/* Write the low n bits (1..32) of v at bit position pos of the record.
   A byte at a time: at most 5 steps for a 32-bit field. */
static inline void bp_put(uint8_t *rec, uint32_t pos, uint32_t n, uint32_t v)
{
	while (n > 0U) {
		uint32_t	off = pos & 7U;
		uint32_t	take = ((8U - off) < n) ? (8U - off) : n;
		uint8_t		mask = (uint8_t)(BP_MAX(take) << off);

		rec[pos >> 3] = (uint8_t)((rec[pos >> 3] & (uint8_t)~mask) |
					  (uint8_t)((v << off) & mask));
		v = (take < 32U) ? (v >> take) : 0U;
		pos += take;
		n -= take;
	}
}

/* Read n bits (1..32) at bit position pos */
static inline uint32_t bp_get(const uint8_t *rec, uint32_t pos, uint32_t n)
{
	uint32_t v = 0U;
	uint32_t got = 0U;

	while (got < n) {
		uint32_t off = pos & 7U;
		uint32_t take = ((8U - off) < (n - got)) ? (8U - off) : (n - got);

		v |= ((uint32_t)(rec[pos >> 3] >> off) & BP_MAX(take)) << got;
		pos += take;
		got += take;
	}
	return v;
}

/* A record is written field by field with a cursor */
typedef struct {
	uint8_t		*rec;
	uint32_t	pos;		/* next free bit */
} BP_W;

typedef struct {
	const uint8_t	*rec;
	uint32_t	pos;		/* next bit to read */
} BP_R;

static inline void bp_wu(BP_W *w, uint32_t n, int32_t v)		/* unsigned, saturating */
{
	bp_put(w->rec, w->pos, n, bp_sat_u(v, n));
	w->pos += n;
}

static inline void bp_ws(BP_W *w, uint32_t n, int32_t min, int32_t v)	/* signed with offset */
{
	bp_put(w->rec, w->pos, n, bp_sat_s(v, min, n));
	w->pos += n;
}

static inline void bp_wraw(BP_W *w, uint32_t n, uint32_t v)	/* low n bits, no saturation */
{
	bp_put(w->rec, w->pos, n, v);
	w->pos += n;
}

static inline uint32_t bp_ru(BP_R *r, uint32_t n)
{
	uint32_t v = bp_get(r->rec, r->pos, n);

	r->pos += n;
	return v;
}

static inline int32_t bp_rs(BP_R *r, uint32_t n, int32_t min)
{
	return bp_from_s(bp_ru(r, n), min);
}

/* ---- wrap-safe time ------------------------------------------------------
 * 16-bit ms times (APP_MSG.t_ms, health stamps, the motion command's line
 * time) wrap every 65.536 s.  Only differences are used, and only for
 * intervals far shorter than that (latency logs: a few ms; the longest
 * health limit: 250 ms).  Never compare two of them with ">".          */
static inline uint32_t bp_age16(uint32_t now_ms, uint32_t then16)
{
	return (uint32_t)(uint16_t)((uint16_t)now_ms - (uint16_t)then16);
}

/* The full 32-bit ms time of a 16-bit stamp taken at most 65.5 s ago */
static inline uint32_t bp_expand16(uint32_t now_ms, uint32_t then16)
{
	return now_ms - bp_age16(now_ms, then16);
}

/* 31-bit us times (barcode edge buffer: bit 31 holds the sensor level).
   Mask BOTH times before subtracting and mask the difference: correct
   across the wrap (every 2^31 us = 35.8 min) and whatever the level bits. */
#define BP_T31_MASK	0x7FFFFFFFU

static inline uint32_t bp_dt31(uint32_t later, uint32_t earlier)
{
	return ((later & BP_T31_MASK) - (earlier & BP_T31_MASK)) & BP_T31_MASK;
}

/* Compile-time size check (C99: a negative array size fails the build) */
#define BP_STATIC_ASSERT(cond, name)	typedef char bp_assert_##name[(cond) ? 1 : -1]

#endif /* BITPACK_H */
