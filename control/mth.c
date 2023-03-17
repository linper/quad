/**
 * @file mth.c
 * @brief Misc math stuff
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_vector_double.h>

#include <json-c/json_object.h>
#include <json-c/json_types.h>

#include <log.h>
#include <mth.h>

double area(double *p1, double *p2, double *p3)
{
	return fabs((p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) +
				 p3[0] * (p1[1] - p2[1])) /
				2.0);
}

double bound_data(double dt, double lo, double hi)
{
	if (dt < lo) {
		return lo;
	} else if (dt > hi) {
		return hi;
	} else {
		return dt;
	}
}

/************
*  Matrix  *
************/

gsl_matrix *matrix_from_array(unsigned n_rows, unsigned n_cols,
							  const double *arr)
{
	gsl_matrix *m;

	if (!arr) {
		return NULL;
	}

	m = gsl_matrix_calloc(n_rows, n_cols);
	if (!m) {
		return NULL;
	}

	for (unsigned i = 0; i < n_rows; i++) {
		memcpy(m->data + i * m->tda, arr + i * n_cols, n_cols * sizeof(double));
	}

	return m;
}

int matrix_update_array(gsl_matrix *m, unsigned n_rows, unsigned n_cols,
						const double *arr)
{
	if (!m || !arr) {
		return 1;
	}

	for (unsigned i = 0; i < n_rows; i++) {
		memcpy(m->data + i * m->tda, arr + i * n_cols, n_cols * sizeof(double));
	}

	return 0;
}

struct json_object *matrix_to_json(gsl_matrix *m)
{
	struct json_object *l1, *l2;

	if (!m) {
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	l1 = json_object_new_array_ext(m->size1);
	for (size_t i = 0; i < m->size1; i++) {
		l2 = json_object_new_array_ext(m->size2);
		for (size_t j = 0; j < m->size2; j++) {
			json_object_array_put_idx(
				l2, j, json_object_new_double(gsl_matrix_get(m, i, j)));
		}
		json_object_array_put_idx(l1, i, l2);
	}

	return l1;
}

/************
*  VECTOR  *
************/

gsl_vector *vector_from_array(unsigned n, const double *arr)
{
	gsl_vector *m;

	if (!arr) {
		return NULL;
	}

	m = gsl_vector_calloc(n);
	if (!m) {
		return NULL;
	}

	for (unsigned i = 0; i < n; i++) {
		m->data[i * m->stride] = arr[i];
	}

	return m;
}

int vector_update_array(gsl_vector *v, unsigned n, const double *arr)
{
	if (!v || !arr) {
		return 1;
	}

	for (unsigned i = 0; i < n; i++) {
		v->data[i * v->stride] = arr[i];
	}

	return 0;
}

double vector_length(gsl_vector *v)
{
	double sum = 0.0, e;

	if (!v) {
		return 0.0;
	}

	for (size_t i = 0; i < v->size; i++) {
		e = gsl_vector_get(v, i);
		sum += e * e;
	}

	sum = sqrt(sum);

	return sum;
}

gsl_vector *vector_sub_n(gsl_vector *a, gsl_vector *b)
{
	gsl_vector *m;

	if (!a || !b) {
		return NULL;
	}

	m = gsl_vector_calloc(a->size);
	if (!m) {
		return NULL;
	}

	gsl_vector_memcpy(m, a);

	gsl_vector_sub(m, b);

	return m;
}

gsl_vector *vector_add_n(gsl_vector *a, gsl_vector *b)
{
	gsl_vector *m;

	if (!a || !b) {
		return NULL;
	}

	m = gsl_vector_calloc(a->size);
	if (!m) {
		return NULL;
	}

	gsl_vector_memcpy(m, a);

	gsl_vector_add(m, b);

	return m;
}

gsl_vector *vector_mul_n(gsl_vector *a, gsl_vector *b)
{
	gsl_vector *m;

	if (!a || !b) {
		return NULL;
	}

	m = gsl_vector_calloc(a->size);
	if (!m) {
		return NULL;
	}

	gsl_vector_memcpy(m, a);

	gsl_vector_mul(m, b);

	return m;
}

gsl_vector *vector_div_n(gsl_vector *a, gsl_vector *b)
{
	gsl_vector *m;

	if (!a || !b) {
		return NULL;
	}

	m = gsl_vector_calloc(a->size);
	if (!m) {
		return NULL;
	}

	gsl_vector_memcpy(m, a);

	gsl_vector_div(m, b);

	return m;
}

struct json_object *vector_to_json(gsl_vector *v)
{
	struct json_object *l1;

	if (!v) {
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	l1 = json_object_new_array_ext(v->size);
	for (size_t i = 0; i < v->size; i++) {
		json_object_array_put_idx(l1, i,
								  json_object_new_double(gsl_vector_get(v, i)));
	}

	return l1;
}

/***********
*  BLOCK  *
***********/

gsl_block *block_from_array(unsigned n, const double *arr)
{
	gsl_block *m;

	if (!arr) {
		return NULL;
	}

	m = gsl_block_calloc(n);
	if (!m) {
		return NULL;
	}

	memcpy(m->data, arr, n * sizeof(double));

	return m;
}

int block_update_array(gsl_block *b, unsigned n, const double *arr)
{
	if (!b || !arr) {
		return 0;
	}

	memcpy(b->data, arr, n * sizeof(double));

	return 0;
}

struct json_object *block_to_json(gsl_block *b)
{
	struct json_object *l1;

	if (!b) {
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	l1 = json_object_new_array_ext(b->size);
	for (size_t i = 0; i < b->size; i++) {
		json_object_array_put_idx(l1, i, json_object_new_double(b->data[i]));
	}

	return l1;
}
