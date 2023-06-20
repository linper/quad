/**
 * @file log.c
 * @brief Loging Interface
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <execinfo.h>
#include <stdlib.h>

#include "log.h"

void print_trace(FILE *stream)
{
	void *array[16];
	char **strings;

	size_t size = backtrace(array, 10);
	strings = backtrace_symbols(array, size);
	if (strings != NULL) {
		fprintf(stream, "Obtained %zu stack frames.\n", size);
		for (size_t i = 0; i < size; i++)
			fprintf(stream, "%s\n", strings[i]);
	}

	free(strings);
}
