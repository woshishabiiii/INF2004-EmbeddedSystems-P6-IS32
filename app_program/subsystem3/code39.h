/*
 *	code39.h
 *	Code 39 decoder for one-character barcodes: start '*', data, stop '*'.
 *	Pure C (only <stdint.h>/<stdbool.h>), so the same file also compiles
 *	and runs on a laptop for unit testing.
 */

#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>
#include <stdbool.h>

/* 3 characters x 9 elements + 2 inter-character gaps */
#define CODE39_ELEMENTS		29U

typedef enum {
	C39_OK = 0,
	C39_BAD_COUNT,		/* not exactly 29 widths */
	C39_BAD_WIDTHS,		/* narrow and wide cannot be told apart */
	C39_BAD_CHAR,		/* a pattern that is not a Code 39 character */
	C39_BAD_DELIMITER	/* first or last character is not '*' */
} c39_result_t;

/*
 * widths: element durations in any unit (e.g. microseconds), in the order
 *         they were seen: bar, space, bar, ... (29 values, first = bar).
 * out_char: the data character on success.
 * reversed: true if the barcode was read back to front.
 */
c39_result_t code39_decode(const uint32_t widths[], uint32_t n,
			   char *out_char, bool *reversed);

/* Build the 29 widths of "*c*" with the given narrow and wide widths.
   Returns false if c is not encodable.  Used by the unit tests. */
bool code39_encode(char c, uint32_t narrow, uint32_t wide, uint32_t widths[]);

#endif /* CODE39_H */
