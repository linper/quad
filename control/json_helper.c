/**
 * @file json_helper.c
 * @brief Helper for partial json parsing
 * @author Linas Perkauskas
 * @date 2023-02-04
 */
#include <stdlib.h>

#include "mth.h"
#include "json_helper.h"

#define OP_TBL '{'
#define CL_TBL '}'
#define OP_ARR '['
#define CL_ARR ']'

int json_help_analyze(const char *str, struct json_status *st)
{
	int balance = 0;
	const char *s = str;

	st->start = NULL;
	st->inner = NULL;
	st->n = 0;
	st->rank = 0;

	while (*s) {
		switch (*s) {
		case OP_TBL:
		case OP_ARR:
			if (!balance && !st->start) {
				// First OP
				st->start = s;
			} else if (balance == 1 && !st->inner) {
				// Second OP
				st->inner = s;
			}
			balance++;
			st->rank = MAX(st->rank, balance);
			break;
		case CL_TBL:
		case CL_ARR:
			balance--;
			if (!balance && !st->n) {
				// Closing last array/table
				st->n = s - str + 1;
				goto end;
			}

			break;
		}

		s++;
	}
end:
	return !!(balance || !st->n || !st->start);
}

const char *json_help_to_start(const char *str)
{
	const char *s = str;
	while (*s) {
		if (*s == CL_ARR || *s == CL_TBL) {
			return s;
		}

		s++;
	}

	return NULL;
}

const char *json_help_step_out(const char *str)
{
	const char *s = str;
	while (*s) {
		s++;
		if (*s == CL_ARR || *s == CL_TBL) {
			return s;
		} else if (*s == OP_ARR || *s == OP_TBL) {
			return NULL;
		}
	}

	return NULL;
}

const char *json_help_step_in(const char *str)
{
	const char *s = str;
	while (*s) {
		s++;
		if (*s == OP_ARR || *s == OP_TBL) {
			return s;
		} else if (*s == CL_ARR || *s == CL_TBL) {
			return NULL;
		}
	}

	return NULL;
}
