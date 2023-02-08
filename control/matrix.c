/**
 * @file matrix.c
 * @brief Implementation of matrix data structure and its interface
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <log.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "mth.h"
#include "matrix.h"

mat_t *create_matrix(uint8_t m, uint8_t n, bool zero)
{
	if (!m || !n) {
		return NULL;
	}

	mat_t *mat = malloc(sizeof(mat_t));
	if (!mat) {
		FATAL(ERR_MALLOC_FAIL);
	}

	if (zero) {
		mat->arr = calloc(n * m, sizeof(float));
	} else {
		mat->arr = malloc(n * m * sizeof(float));
	}

	if (!mat->arr) {
		FATAL(ERR_MALLOC_FAIL);
	}

	return mat;
}

mat_t *matrix_from_array(uint8_t m, uint8_t n, float *arr)
{
	if (!arr) {
		return NULL;
	}

	mat_t *mat = create_matrix(m, n, false);
	if (!mat) {
		return NULL;
	}

	memcpy(mat->arr, arr, n * m * sizeof(float));

	return mat;
}

int matrix_update_array(mat_t *mat, uint32_t l, float *arr)
{
	if (!mat || !arr || mat->n * mat->n != l) {
		return 1;
	}

	memcpy(mat->arr, arr, l * sizeof(float));

	return 0;
}

void free_matrix(mat_t *m)
{
	if (m) {
		free(m->arr);
		free(m);
	}
}

