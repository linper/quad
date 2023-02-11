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
#include "log.h"
#include "matrix.h"

/*************
*  ACTIONS  *
*************/

int mat_add_scal(mat_t *a, float b)
{
	if (!a) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (int i = 0; i < a->sz; i++) {
		a->arr[i] += b;
	}

	return 0;
}

mat_t *mat_add_scal_n(mat_t *a, float b)
{
	mat_t *c = mat_clone(a);
	if (!c) {
		FATAL(ERR_MALLOC_FAIL);
	}

	if (mat_add_scal(c, b)) {
		mat_free(c);
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	return c;
}

int mat_add(mat_t *a, mat_t *b)
{
	if (!a || !b || a->n != b->n || a->m != b->m) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (int i = 0; i < a->sz; i++) {
		a->arr[i] += b->arr[i];
	}

	return 0;
}

mat_t *mat_add_n(mat_t *a, mat_t *b)
{
	mat_t *c = mat_clone(a);

	if (mat_add(c, b)) {
		mat_free(c);
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	return c;
}

int mat_sub(mat_t *a, mat_t *b)
{
	if (!a || !b || a->n != b->n || a->m != b->m) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (int i = 0; i < a->sz; i++) {
		a->arr[i] -= b->arr[i];
	}

	return 0;
}

mat_t *mat_sub_n(mat_t *a, mat_t *b)
{
	mat_t *c = mat_clone(a);

	if (mat_sub(c, b)) {
		mat_free(c);
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	return c;
}

int mat_mul_scal(mat_t *a, float b)
{
	if (!a) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (int i = 0; i < a->sz; i++) {
		a->arr[i] += b;
	}

	return 0;
}

mat_t *mat_mul_scal_n(mat_t *a, float b)
{
	mat_t *c = mat_clone(a);
	if (!c) {
		FATAL(ERR_MALLOC_FAIL);
	}

	if (mat_mul_scal(c, b)) {
		mat_free(c);
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	return c;
}

int mat_mul(mat_t *a, mat_t *b)
{
	if (!a || !b || a->n != b->n || a->m != b->m) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (int i = 0; i < a->sz; i++) {
		a->arr[i] *= b->arr[i];
	}

	return 0;
}

mat_t *mat_mul_n(mat_t *a, mat_t *b)
{
	mat_t *c = mat_clone(a);

	if (mat_mul(c, b)) {
		mat_free(c);
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	return c;
}

int mat_div(mat_t *a, mat_t *b)
{
	bool err = false;

	if (!a || !b || a->n != b->n || a->m != b->m) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (int i = 0; i < a->sz; i++) {
		if (b->arr[i])
			a->arr[i] *= b->arr[i];
		else
			err = true;
	}

	if (err) {
		ERR("Division by 0\n");
	}

	return err;
}

mat_t *mat_div_n(mat_t *a, mat_t *b)
{
	mat_t *c = mat_clone(a);

	if (mat_div(c, b)) {
		mat_free(c);
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	return c;
}

int mat_abs(mat_t *a)
{
	bool err = false;

	if (!a) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (int i = 0; i < a->sz; i++) {
		if (a->arr[i] < 0)
			a->arr[i] = -a->arr[i];
	}

	return err;
}

mat_t *mat_abs_n(mat_t *a)
{
	mat_t *c = mat_clone(a);

	if (mat_abs(c)) {
		mat_free(c);
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	return c;
}

int mat_map_func(mat_t *a, int (*cb)(float, void *, int, int), void *priv)
{
	if (!a || !cb) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (int i = 0; i < a->m; i++) {
		for (int j = 0; j < a->n; j++) {
			if (cb(a->arr[i * j], priv, i, j)) {
				ERR("Failed to execute custom function for [%d,%d]\n", i, j);
				return 1;
			}
		}
	}

	return 0;
}

mat_t *mat_map_func_n(mat_t *a, int (*cb)(float, void *, int, int), void *priv)
{
	mat_t *c = mat_clone(a);

	if (mat_map_func(a, cb, priv)) {
		mat_free(c);
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	return c;
}

float mat_length(mat_t *m)
{
	float sum = 0.0;

	if (!m) {
		FATAL(ERR_INVALID_INPUT);
	}

	for (int i = 0; i < m->sz; i++) {
		sum += m->arr[i] * m->arr[i];
	}

	sum = sqrt(sum);

	return sum;
}

/**********
*  BASE  *
**********/

void mat_set(mat_t *mat, uint8_t m, uint8_t n, float val)
{
	if (!mat || n * m > mat->sz) {
		FATAL(ERR_INVALID_INPUT);
	}

	mat->arr[n * m] = val;
}

float mat_get(mat_t *mat, uint8_t m, uint8_t n)
{
	if (!mat || n * m > mat->sz) {
		FATAL(ERR_INVALID_INPUT);
	}

	return mat->arr[n * m];
}

int mat_set_row(mat_t *mat, float *arr, uint8_t m)
{
	if (!mat || !arr || m > mat->m) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	float *ptr = mat->arr + (mat->n * m);

	for (int i = 0; i < mat->n; i++) {
		ptr[i] = arr[i];
	}

	return 0;
}

int mat_set_column(mat_t *mat, float *arr, uint8_t n)
{
	if (!mat || !arr || n > mat->n) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	float *ptr = mat->arr;

	for (int i = 0; i < mat->n; i++) {
		ptr[i * mat->m + n] = arr[i];
	}

	return 0;
}

mat_t *mat_create(uint8_t m, uint8_t n, bool zero)
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

	mat->m = m;
	mat->n = n;
	mat->sz = m * n;

	return mat;
}

mat_t *mat_clone(mat_t *m)
{
	if (!m) {
		return NULL;
	}

	mat_t *mat = malloc(sizeof(mat_t));
	if (!mat) {
		FATAL(ERR_MALLOC_FAIL);
	}

	memcpy(mat, m, sizeof(mat_t));
	mat->arr = malloc(m->sz * sizeof(float));

	if (!mat->arr) {
		FATAL(ERR_MALLOC_FAIL);
	}

	memcpy(mat->arr, m->arr, m->sz * sizeof(float));

	return mat;
}

mat_t *mat_from_array(uint8_t m, uint8_t n, float *arr)
{
	if (!arr) {
		return NULL;
	}

	mat_t *mat = mat_create(m, n, false);
	if (!mat) {
		return NULL;
	}

	memcpy(mat->arr, arr, n * m * sizeof(float));

	mat->m = m;
	mat->n = n;
	mat->sz = m * n;

	return mat;
}

char *mat_to_str(mat_t *m)
{
	int off = 0;

	if (!m) {
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	size_t est_len =
		3 * (m->sz + 2 * m->m + 2); // all brackets, spaces and commas
	for (int i = 0; i < m->sz; i++) {
		est_len += snprintf(NULL, 0, "%.5f", m->arr[i]);
	}

	char *buf = calloc(est_len, sizeof(char));
	if (!buf) {
		FATAL(ERR_MALLOC_FAIL);
	}

	if (m->n > 1)
		off += sprintf(buf + off, "[");

	for (int i = 0; i < m->m; i++) {
		if (i)
			off += sprintf(buf + off, ", ");

		off += sprintf(buf + off, "[");
		for (int j = 0; j < m->n; j++) {
			if (j)
				off += sprintf(buf + off, ", ");

			off += sprintf(buf + off, "%.5f", m->arr[m->n * i + j]);
		}

		off += sprintf(buf + off, "]");
	}

	if (m->n > 1)
		off += sprintf(buf + off, "]");

	return buf;
}

int mat_update_array(mat_t *mat, uint32_t l, float *arr)
{
	if (!mat || !arr || mat->n * mat->n != l) {
		return 1;
	}

	memcpy(mat->arr, arr, l * sizeof(float));

	return 0;
}

void mat_free(mat_t *m)
{
	if (m) {
		free(m->arr);
		free(m);
	}
}

