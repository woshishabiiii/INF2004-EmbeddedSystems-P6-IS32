/*
 *	cmd_parse.c  (see cmd_parse.h)
 *	No sscanf/atoi: plain loops, every length checked.
 */

#include "cmd_parse.h"

#define CMD_SEQ_MAX	100000000	/* longer numbers are rejected */

const CMD_PARAM cmd_params[CMD_NPARAMS] = {
	{"base_speed",	 80,   400},	/* mm/s, line following */
	{"stop_mm",	150,   600},	/* obstacle ALERT distance */
	{"kp_x1000",	  0, 20000},	/* motion PI gains x 1000 */
	{"ki_x1000",	  0, 50000},
	{"dist_cal",	500,  1500},	/* 1000 x commanded / measured distance */
	{"turn_cal",	500,  1500},	/* 1000 x commanded / measured angle */
};

static char upper(char c)
{
	return ((c >= 'a') && (c <= 'z')) ? (char)(c - ('a' - 'A')) : c;
}

/* a (alen chars) equals the NUL-terminated b; fold = ignore case */
static bool str_eq(const char *a, uint32_t alen, const char *b, bool fold)
{
	uint32_t i;

	for (i = 0U; i < alen; i++) {
		if (b[i] == '\0') {
			return false;
		}
		if ((fold ? upper(a[i]) : a[i]) != b[i]) {
			return false;
		}
	}
	return b[alen] == '\0';
}

/* Next space-separated word: sets *w / *wl, returns the index after it */
static uint32_t next_word(const char *t, uint32_t len, uint32_t i, const char **w, uint32_t *wl)
{
	while ((i < len) && ((t[i] == ' ') || (t[i] == '\r') || (t[i] == '\n'))) {
		i++;
	}
	*w = &t[i];
	*wl = 0U;
	while ((i < len) && (t[i] != ' ') && (t[i] != '\r') && (t[i] != '\n')) {
		i++;
		(*wl)++;
	}
	return i;
}

static bool to_int(const char *w, uint32_t wl, int32_t *out)
{
	uint32_t	i = 0U;
	int32_t		v = 0;
	bool		neg = false;

	if (wl == 0U) {
		return false;
	}
	if (w[0] == '-') {
		neg = true;
		i = 1U;
		if (wl == 1U) {
			return false;
		}
	}
	for (; i < wl; i++) {
		if ((w[i] < '0') || (w[i] > '9') || (v > CMD_SEQ_MAX)) {
			return false;		/* not a digit, or far too long */
		}
		v = (v * 10) + (int32_t)(w[i] - '0');
	}
	*out = neg ? -v : v;
	return true;
}

cmd_err_t cmd_parse(const char *text, uint32_t len, uint32_t *last_seq,
		    bool settable, CMD *out)
{
	const char	*w;
	uint32_t	wl, i, p;
	int32_t		seq;

	out->seq = 0U;
	out->verb = 0;
	out->param = -1;
	out->value = 0;
	if ((len == 0U) || (len > CMD_MAX_LEN)) {
		return CMD_ERR_FORMAT;
	}
	i = next_word(text, len, 0U, &w, &wl);
	if (!to_int(w, wl, &seq) || (seq < 0)) {
		return CMD_ERR_FORMAT;
	}
	out->seq = (uint32_t)seq;

	i = next_word(text, len, i, &w, &wl);
	if (str_eq(w, wl, "START", true)) {
		out->verb = 1;
	} else if (str_eq(w, wl, "STOP", true)) {
		out->verb = 2;
	} else if (str_eq(w, wl, "RESTART", true)) {
		out->verb = 3;
	} else if (str_eq(w, wl, "FINISH", true)) {
		out->verb = 4;
	} else if (str_eq(w, wl, "SET", true)) {
		out->verb = CMD_VERB_SET;
	} else {
		return CMD_ERR_VERB;
	}

	if (out->verb == CMD_VERB_SET) {
		i = next_word(text, len, i, &w, &wl);
		for (p = 0U; p < CMD_NPARAMS; p++) {
			if (str_eq(w, wl, cmd_params[p].name, false)) {
				break;
			}
		}
		if (p == CMD_NPARAMS) {
			return CMD_ERR_NAME;
		}
		i = next_word(text, len, i, &w, &wl);
		if (!to_int(w, wl, &out->value)) {
			return CMD_ERR_FORMAT;
		}
		if ((out->value < cmd_params[p].min) || (out->value > cmd_params[p].max)) {
			return CMD_ERR_RANGE;
		}
		if (!settable) {
			return CMD_ERR_STATE;
		}
		out->param = (int32_t)p;
	}

	(void)next_word(text, len, i, &w, &wl);
	if (wl != 0U) {
		return CMD_ERR_FORMAT;			/* trailing words */
	}
	if (out->seq <= *last_seq) {
		return CMD_ERR_REPEAT;			/* checked last: only valid commands count */
	}
	*last_seq = out->seq;
	return CMD_OK;
}
