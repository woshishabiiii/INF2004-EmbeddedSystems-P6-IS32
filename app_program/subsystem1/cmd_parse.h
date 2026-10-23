/*
 *	cmd_parse.h
 *	Decodes and validates one MQTT command (Buddy 1).  Pure C (stdint and
 *	stdbool only), unit-tested on a laptop; tools/mqtt_decode.py encodes
 *	commands with the same layout.
 *
 *	Format: 4 bytes, little-endian, fields LSB first (bitpack.h), no text:
 *	  bits 0-7    seq     8 bits, wraps
 *	  bits 8-10   verb    3 bits: 1 START, 2 STOP, 3 RESTART, 4 FINISH, 5 SET
 *	  bits 11-13  param   3 bits: SET only, index into cmd_params[] (0..5)
 *	  bits 14-15  spare   must be 0 (a different layout is refused, not misread)
 *	  bits 16-31  value  16 bits unsigned: SET only (largest: ki_x1000 = 50,000)
 *	A command is new if its seq is 1 to 127 ahead of the last accepted one
 *	(mod 256); anything else is a repeat or a delayed copy and is refused.
 *	The car starts from 0 after a reset, so the first command is seq 1..127.
 *	(Version B: "seq must increase" breaks once an 8-bit seq wraps.)
 */

#ifndef CMD_PARSE_H
#define CMD_PARSE_H

#include <stdint.h>
#include <stdbool.h>

#define CMD_BYTES	4U
#define CMD_SEQ_WINDOW	127U	/* new = 1..127 ahead, mod 256 */

typedef enum {
	CMD_OK = 0,
	CMD_ERR_FORMAT,		/* not 4 bytes, or the spare bits are set */
	CMD_ERR_VERB,		/* unknown verb */
	CMD_ERR_REPEAT,		/* seq not 1..127 ahead of the last accepted */
	CMD_ERR_NAME,		/* SET of a parameter not in the whitelist */
	CMD_ERR_RANGE,		/* SET value outside its limits */
	CMD_ERR_STATE		/* SET while the car is running */
} cmd_err_t;

#define CMD_VERB_SET	5	/* verbs 1..4 are CMD_START..CMD_FINISH (app_evt.h) */

typedef struct {
	uint32_t	seq;		/* 0..255 */
	int32_t		verb;		/* CMD_START .. CMD_FINISH, or CMD_VERB_SET */
	int32_t		param;		/* index into cmd_params[], for SET */
	int32_t		value;
} CMD;

typedef struct {
	const char	*name;
	int32_t		min;
	int32_t		max;
} CMD_PARAM;

/* The tunable values (whitelist).  Order = parameter index (PARAM_... in
   subsystem1.h).  Each owner copies its values when START arrives. */
#define CMD_NPARAMS	6U
extern const CMD_PARAM cmd_params[CMD_NPARAMS];

/* last_seq: the last accepted seq (updated on success).
   settable: true while the car is not running (READY, FINISHED, HALTED). */
cmd_err_t cmd_parse(const uint8_t *data, uint32_t len, uint32_t *last_seq,
		    bool settable, CMD *out);

/* The 4 bytes for a command (laptop tools and tests; the car only decodes) */
void	cmd_encode(const CMD *c, uint8_t out[CMD_BYTES]);

#endif /* CMD_PARSE_H */
