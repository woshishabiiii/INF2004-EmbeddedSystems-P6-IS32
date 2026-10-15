/*
 *	code39.c
 *	Code 39: every character is 9 elements (5 bars, 4 spaces) of which
 *	exactly 3 are wide.  A 9-bit pattern, first element = most
 *	significant bit, 1 = wide, identifies the character.  The table below
 *	is the standard Code 39 character set.
 */

#include "code39.h"

#define C39_CHARS	44U
#define C39_MIN_RATIO_X10	13U	/* narrowest wide >= 1.3 x widest narrow, or reject */

static const char	s_alphabet[C39_CHARS] =
	"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";

static const uint16_t	s_pattern[C39_CHARS] = {
	0x034U, 0x121U, 0x061U, 0x160U, 0x031U, 0x130U, 0x070U, 0x025U, 0x124U, 0x064U,	/* 0-9 */
	0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U, 0x00DU, 0x10CU, 0x04CU, 0x01CU,	/* A-J */
	0x103U, 0x043U, 0x142U, 0x013U, 0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U,	/* K-T */
	0x181U, 0x0C1U, 0x1C0U, 0x091U, 0x190U, 0x0D0U,					/* U-Z */
	0x085U, 0x184U, 0x0C4U, 0x0A8U, 0x0A2U, 0x08AU, 0x02AU,				/* - . space $ / + % */
	0x094U										/* '*' */
};

static uint32_t count_bits(uint32_t x)
{
	uint32_t n = 0U;

	while (x != 0U) {
		n += x & 1U;
		x >>= 1;
	}
	return n;
}

static bool lookup(uint32_t pattern, char *out)
{
	uint32_t i;

	if (count_bits(pattern) != 3U) {
		return false;			/* every character has exactly 3 wide */
	}
	for (i = 0U; i < C39_CHARS; i++) {
		if (s_pattern[i] == pattern) {
			*out = s_alphabet[i];
			return true;
		}
	}
	return false;
}

/* One character from its 9 widths.  Every Code 39 character has exactly
   three wide elements, so the three WIDEST are taken as wide.  This only
   compares elements of the same character (about 16 mm of barcode), so a
   change of speed across the barcode does not matter; the narrowest of
   the three must still be clearly wider than the widest of the other six. */
static c39_result_t decode_char(const uint32_t w[], char *out)
{
	uint32_t	i, k, pattern = 0U;
	uint32_t	min_wide = UINT32_MAX, max_narrow = 0U;
	uint32_t	rank;

	for (i = 0U; i < 9U; i++) {
		rank = 0U;			/* how many elements are wider than this one */
		for (k = 0U; k < 9U; k++) {
			if ((w[k] > w[i]) || ((w[k] == w[i]) && (k < i))) {
				rank++;
			}
		}
		if (rank < 3U) {
			pattern |= 1U << (8U - i);	/* one of the three widest */
			min_wide = (w[i] < min_wide) ? w[i] : min_wide;
		} else {
			max_narrow = (w[i] > max_narrow) ? w[i] : max_narrow;
		}
	}
	if ((max_narrow == 0U) || ((min_wide * 10U) < (max_narrow * C39_MIN_RATIO_X10))) {
		return C39_BAD_WIDTHS;		/* wide and narrow cannot be told apart */
	}
	return lookup(pattern, out) ? C39_OK : C39_BAD_CHAR;
}

static c39_result_t decode_once(const uint32_t w[], char *out_char)
{
	uint32_t	c;
	char		ch[3];
	c39_result_t	r;

	for (c = 0U; c < 3U; c++) {
		r = decode_char(&w[c * 10U], &ch[c]);	/* characters start at 0, 10, 20 */
		if (r != C39_OK) {
			return r;
		}
	}
	if ((ch[0] != '*') || (ch[2] != '*')) {
		return C39_BAD_DELIMITER;
	}
	*out_char = ch[1];
	return C39_OK;
}

c39_result_t code39_decode(const uint32_t widths[], uint32_t n,
			   char *out_char, bool *reversed)
{
	uint32_t	rev[CODE39_ELEMENTS];
	uint32_t	i;
	c39_result_t	r;

	if (n != CODE39_ELEMENTS) {
		return C39_BAD_COUNT;
	}
	r = decode_once(widths, out_char);
	if (r == C39_OK) {
		*reversed = false;
		return r;
	}
	/* The car may cross the barcode from the other end: try backwards.
	   ('*' read backwards is 'P', so a reversed read always fails the
	   delimiter check above instead of giving a wrong character.) */
	for (i = 0U; i < CODE39_ELEMENTS; i++) {
		rev[i] = widths[CODE39_ELEMENTS - 1U - i];
	}
	r = decode_once(rev, out_char);
	if (r == C39_OK) {
		*reversed = true;
	}
	return r;
}

bool code39_encode(char c, uint32_t narrow, uint32_t wide, uint32_t widths[])
{
	uint32_t	k, i, idx = C39_CHARS;
	uint32_t	pat[3];

	for (i = 0U; i < C39_CHARS; i++) {
		if (s_alphabet[i] == c) {
			idx = i;
		}
	}
	if ((idx == C39_CHARS) || (c == '*')) {
		return false;
	}
	pat[0] = 0x094U;
	pat[1] = s_pattern[idx];
	pat[2] = 0x094U;
	for (k = 0U; k < 3U; k++) {
		for (i = 0U; i < 9U; i++) {
			widths[(k * 10U) + i] = (((pat[k] >> (8U - i)) & 1U) != 0U) ? wide : narrow;
		}
		if (k < 2U) {
			widths[(k * 10U) + 9U] = narrow;	/* inter-character gap */
		}
	}
	return true;
}
