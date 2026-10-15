/*
 *	cmd_parse.h
 *	Parses and validates one MQTT command payload (Buddy 1).  Pure C
 *	(stdint and stdbool only), unit-tested on a laptop.
 *
 *	Format (ASCII, single spaces):   <seq> <VERB> [<name> <value>]
 *	  "12 START"   "13 STOP"   "14 RESTART"   "15 FINISH"
 *	  "16 SET base_speed 220"
 *	Verbs may be upper or lower case.  seq (1, 2, 3 ...) must be larger
 *	than the last accepted one, so a repeated or delayed message is
 *	ignored; it starts again from 0 when the car restarts.
 */

#ifndef CMD_PARSE_H
#define CMD_PARSE_H

#include <stdint.h>
#include <stdbool.h>

#define CMD_MAX_LEN	64U

typedef enum {
	CMD_OK = 0,
	CMD_ERR_FORMAT,		/* not "<seq> <VERB> ..." */
	CMD_ERR_VERB,		/* unknown verb */
	CMD_ERR_REPEAT,		/* seq not newer than the last accepted */
	CMD_ERR_NAME,		/* SET of a name not in the whitelist */
	CMD_ERR_RANGE,		/* SET value outside its limits */
	CMD_ERR_STATE		/* SET while the car is running */
} cmd_err_t;

#define CMD_VERB_SET	5	/* verbs 1..4 are CMD_START..CMD_FINISH (app_evt.h) */

typedef struct {
	uint32_t	seq;
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
   settable: true while the car is not running (READY or FINISHED). */
cmd_err_t cmd_parse(const char *text, uint32_t len, uint32_t *last_seq,
		    bool settable, CMD *out);

#endif /* CMD_PARSE_H */
