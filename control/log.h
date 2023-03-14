/**
 * @file log.h
 * @brief Loging Interface
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#pragma once

#include <stdio.h>

#define ERR_NOT_FOUND "Not found\n"
#define ERR_MALLOC_FAIL "Failed to allocate memory\n"
#define ERR_ERROR "Unknown error\n"
#define ERR_INVALID_INPUT "Invalid input\n"
#define ERR_PARSE_FAIL "Failed to parse\n"
#define ERR_RANGE "Out of range\n"
#define ERR_IMPL "Not implemented\n"

#define NLS "%s\n"
#define NLD "%s\n"
#define NLU "%s\n"

#define HIT                                                                    \
	do {                                                                       \
		if (!quiet) {                                                          \
			fprintf(stdout, "HIT %s:%d\n", __func__, __LINE__);                \
			fflush(stdout);                                                    \
		}                                                                      \
	} while (0)

#define TRACE                                                                  \
	do {                                                                       \
		if (!quiet) {                                                          \
			print_trace(stdout);                                               \
			fflush(stdout);                                                    \
		}                                                                      \
	} while (0)

#define DBG(fmt, args...)                                                      \
	do {                                                                       \
		if (!quiet && debug) {                                                 \
			fprintf(stdout, "%s[%d]: " fmt, __func__, __LINE__, ##args);       \
			fflush(stdout);                                                    \
		}                                                                      \
	} while (0)

#define ERR(fmt, args...)                                                      \
	do {                                                                       \
		if (!quiet) {                                                          \
			fprintf(stderr, "ERROR %s[%d]: " fmt, __func__, __LINE__, ##args); \
			fflush(stderr);                                                    \
		}                                                                      \
	} while (0)

#define FATAL(fmt, args...)                                                    \
	do {                                                                       \
		fprintf(stderr, "FATAL %s[%d]: " fmt, __func__, __LINE__, ##args);     \
		print_trace(stderr);                                                   \
		fflush(stderr);                                                        \
		exit(EXIT_FAILURE);                                                    \
	} while (0)

#define HEX(data, n)                                                           \
	do {                                                                       \
		if (!quiet) {                                                          \
			fprintf(stdout, "HEX %s[%d]", __func__, __LINE__);                 \
			for (int i = 0; i < (int)n; i++)                                   \
				printf(":%02X", data[i]);                                      \
			printf("\n");                                                      \
			fflush(stdout);                                                    \
		}                                                                      \
	} while (0)

extern int quiet, debug;

void print_trace(FILE *stream);

