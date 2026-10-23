/*
 *	cmd_parse.c  (see cmd_parse.h)
 *	Masks and shifts only; every length checked.
 */

#include "cmd_parse.h"
#include "../bitpack.h"

#define CMD_VERB_POS	8U
#define CMD_PARAM_POS	11U
#define CMD_SPARE_POS	14U
#define CMD_VALUE_POS	16U

const CMD_PARAM cmd_params[CMD_NPARAMS] = {
	{"base_speed",	 80,   400},	/* mm/s, line following */
	{"stop_mm",	150,   600},	/* obstacle ALERT distance */
	{"kp_x1000",	  0, 20000},	/* motion PI gains x 1000 */
	{"ki_x1000",	  0, 50000},
	{"dist_cal",	500,  1500},	/* 1000 x commanded / measured distance */
	{"turn_cal",	500,  1500},	/* 1000 x commanded / measured angle */
};

cmd_err_t cmd_parse(const uint8_t *data, uint32_t len, uint32_t *last_seq,
		    bool settable, CMD *out)
{
	uint32_t w;
	uint32_t ahead;

	out->seq = 0U;
	out->verb = 0;
	out->param = -1;
	out->value = 0;
	if (len != CMD_BYTES) {
		return CMD_ERR_FORMAT;
	}
	w = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
	    ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);	/* little-endian */
	out->seq = w & BP_MAX(8U);
	if (((w >> CMD_SPARE_POS) & BP_MAX(2U)) != 0U) {
		return CMD_ERR_FORMAT;
	}
	out->verb = (int32_t)((w >> CMD_VERB_POS) & BP_MAX(3U));
	if ((out->verb < 1) || (out->verb > CMD_VERB_SET)) {
		return CMD_ERR_VERB;
	}
	if (out->verb == CMD_VERB_SET) {
		uint32_t p = (w >> CMD_PARAM_POS) & BP_MAX(3U);

		if (p >= CMD_NPARAMS) {
			return CMD_ERR_NAME;
		}
		out->value = (int32_t)(w >> CMD_VALUE_POS);
		if ((out->value < cmd_params[p].min) || (out->value > cmd_params[p].max)) {
			return CMD_ERR_RANGE;
		}
		if (!settable) {
			return CMD_ERR_STATE;
		}
		out->param = (int32_t)p;
	}

	/* 8-bit sequence, wrap-safe: new if 1..127 ahead (mod 256) */
	ahead = (out->seq - *last_seq) & BP_MAX(8U);
	if ((ahead == 0U) || (ahead > CMD_SEQ_WINDOW)) {
		return CMD_ERR_REPEAT;			/* checked last: only valid commands count */
	}
	*last_seq = out->seq;
	return CMD_OK;
}

void cmd_encode(const CMD *c, uint8_t out[CMD_BYTES])
{
	uint32_t w = (c->seq & BP_MAX(8U)) |
		     (((uint32_t)c->verb & BP_MAX(3U)) << CMD_VERB_POS);

	if (c->verb == CMD_VERB_SET) {
		w |= (((uint32_t)c->param & BP_MAX(3U)) << CMD_PARAM_POS) |
		     (bp_sat_u(c->value, 16U) << CMD_VALUE_POS);
	}
	out[0] = (uint8_t)w;
	out[1] = (uint8_t)(w >> 8);
	out[2] = (uint8_t)(w >> 16);
	out[3] = (uint8_t)(w >> 24);
}
