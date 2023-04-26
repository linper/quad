/**
 * @file mth.c
 * @brief Misc math stuff
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <gsl/gsl_block_double.h>
#include <stddef.h>
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

gsl_matrix *matrix_from_array(size_t n_rows, size_t n_cols, const double *arr)
{
	gsl_matrix *m;

	if (!arr) {
		return NULL;
	}

	m = gsl_matrix_calloc(n_rows, n_cols);
	if (!m) {
		return NULL;
	}

	for (size_t i = 0; i < n_rows; i++) {
		memcpy(m->data + i * m->tda, arr + i * n_cols, n_cols * sizeof(double));
	}

	return m;
}

int matrix_update_array(gsl_matrix *m, size_t n_rows, size_t n_cols,
						const double *arr)
{
	if (!m || !arr) {
		return 1;
	}

	for (size_t i = 0; i < n_rows; i++) {
		memcpy(m->data + i * m->tda, arr + i * n_cols, n_cols * sizeof(double));
	}

	return 0;
}

int matrix_copy_to(gsl_matrix *dst, gsl_matrix *src, size_t di, size_t dj,
				   size_t si, size_t sj, size_t ni, size_t nj)
{
	if (!dst || !src || dst->size1 < di + ni || dst->size2 < dj + nj ||
		src->size1 < si + ni || src->size2 < sj + nj) {
		return 1;
	}

	for (size_t i = 0; i < ni; i++) {
		memcpy(dst->data + (di + i) * dst->tda + dj,
			   src->data + (si + i) * src->tda + sj, nj * sizeof(double));
	}

	return 0;
}

gsl_matrix *matrix_dot(gsl_matrix *a, gsl_matrix *b)
{
	double ai, bi, temp;
	gsl_matrix *m;

	if (!a || !b || a->size2 != b->size1) {
		return NULL;
	}

	m = gsl_matrix_calloc(a->size1, b->size2);
	if (!m) {
		return NULL;
	}
	for (size_t i = 0; i < m->size1; i++) {
		for (size_t j = 0; j < m->size2; j++) {
			ai = gsl_matrix_get(a, i, 0);
			bi = gsl_matrix_get(b, 0, j);
			temp = ai * bi;
			for (size_t k = 1; k < a->size2; k++) {
				ai = gsl_matrix_get(a, i, k);
				bi = gsl_matrix_get(b, k, j);
				temp += ai * bi;
			}
			gsl_matrix_set(m, i, j, temp);
		}
	}

	return m;
}

gsl_vector *matrix_vector_dot(gsl_matrix *a, gsl_vector *b)
{
	double ai, bi, temp;
	gsl_vector *m;

	if (!a || !b || a->size2 != b->size) {
		return NULL;
	}

	m = gsl_vector_calloc(b->size);
	if (!m) {
		return NULL;
	}

	for (size_t i = 0; i < b->size; i++) {
		temp = 0.0;
		for (size_t j = 0; j < a->size2; j++) {
			bi = gsl_vector_get(b, j);
			ai = gsl_matrix_get(a, i, j);
			temp += ai * bi;
		}
		gsl_vector_set(m, i, temp);
	}

	return m;
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

void matrix_print(gsl_matrix *m)
{
	printf("[");
	for (size_t i = 0; i < m->size1; i++) {
		printf("[");
		for (size_t j = 0; j < m->size2; j++) {
			printf("%.4g%s", gsl_matrix_get(m, i, j),
				   j + 1 < m->size2 ? ", " : "");
		}
		printf("]%s", i + 1 < m->size1 ? ", " : "");
	}
	printf("]\n");
}

/************
*  VECTOR  *
************/

gsl_vector *vector_from_array(size_t n, const double *arr)
{
	gsl_vector *m;

	if (!arr) {
		return NULL;
	}

	m = gsl_vector_calloc(n);
	if (!m) {
		return NULL;
	}

	for (size_t i = 0; i < n; i++) {
		m->data[i * m->stride] = arr[i];
	}

	return m;
}

int vector_update_array(gsl_vector *v, size_t n, const double *arr)
{
	if (!v || !arr) {
		return 1;
	}

	for (size_t i = 0; i < n; i++) {
		v->data[i * v->stride] = arr[i];
	}

	return 0;
}

gsl_vector *vector_clone(gsl_vector *v)
{
	gsl_vector *c = gsl_vector_calloc(v->size);
	if (!v) {
		return NULL;
	}

	gsl_vector_memcpy(c, v);

	return c;
}

gsl_vector *vector_linspace(double start, double end, size_t n)
{
	double inc = 0;
	double cur = start;
	gsl_vector *v;

	if (!n) {
		return NULL;
	}

	v = gsl_vector_calloc(n);

	if (n > 1) {
		inc = (end - start) / (n - 1);
	}

	for (size_t i = 0; i < n; i++) {
		gsl_vector_set(v, i, cur);
		cur += inc;
	}

	return v;
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

void vector_set_length(gsl_vector *v, double l)
{
	double cur_l = vector_length(v);
	gsl_vector_scale(v, l / cur_l);
}

gsl_vector *vector_set_length_n(gsl_vector *v, double l)
{
	if (!v) {
		return NULL;
	}

	gsl_vector *nv = vector_clone(v);
	if (!nv) {
		return NULL;
	}

	vector_set_length(nv, l);

	return nv;
}

double vector3_cos(gsl_vector *a, gsl_vector *b)
{
	double la, lb, ret;

	if (!a || !b || a->size != b->size || a->size != 3) {
		return 0.0;
	}

	la = vector_length(a);
	lb = vector_length(b);

	if (!la || !lb) {
		return 0.0;
	}

	ret = vector_dot(a, b);

	return ret / (la * lb);
}

gsl_vector *vector3_cross(gsl_vector *va, gsl_vector *vb)
{
	if (!va || !vb || va->size != vb->size || va->size != 3) {
		return NULL;
	}

	double a0, a1, a2, b0, b1, b2;
	a0 = gsl_vector_get(va, 0);
	a1 = gsl_vector_get(va, 1);
	a2 = gsl_vector_get(va, 2);
	b0 = gsl_vector_get(vb, 0);
	b1 = gsl_vector_get(vb, 1);
	b2 = gsl_vector_get(vb, 2);

	double cross_arr[9] = {
		a1 * b2 - a2 * b1,
		a2 * b0 - a0 * b2,
		a0 * b1 - a1 * b0,
	};

	gsl_vector *res = vector_from_array(3, cross_arr);

	return res;
}

double vector_dot(gsl_vector *a, gsl_vector *b)
{
	double sum = 0.0;

	if (!a || !b || a->size != b->size) {
		return 0.0;
	}

	for (size_t i = 0; i < a->size; i++) {
		sum += gsl_vector_get(a, i) * gsl_vector_get(b, i);
	}

	return sum;
}

int vector_copy_to(gsl_vector *dst, gsl_vector *src, size_t di, size_t si,
				   size_t ni)
{
	{
		if (!dst || !src || dst->size < di + ni || src->size < si + ni) {
			return 1;
		}

		for (size_t i = 0; i < ni; i++) {
			gsl_vector_set(dst, di + i, gsl_vector_get(src, si + i));
		}

		return 0;
	}
}
gsl_matrix *mat_rot_from_2vec(gsl_vector *from, gsl_vector *to)
{
	double phi, rcos, rsin, a0, a1, a2;
	gsl_vector *axis_m;
	gsl_matrix *mat;

	phi = vector3_angle(from, to);
	axis_m = vector3_cross(from, to);
	vector_set_length(axis_m, 1.0);

	a0 = gsl_vector_get(axis_m, 0);
	a1 = gsl_vector_get(axis_m, 1);
	a2 = gsl_vector_get(axis_m, 2);

	rcos = cosf(phi);
	rsin = sinf(phi);

	double mat_arr[9] = {
		[0] = rcos + a0 * a0 * (1 - rcos),
		[1] = a2 * rsin + a1 * a0 * (1 - rcos),
		[2] = -a1 * rsin + a2 * a0 * (1 - rcos),

		[3] = -a2 * rsin + a0 * a1 * (1 - rcos),
		[4] = rcos + a1 * a1 * (1 - rcos),
		[5] = a0 * rsin + a2 * a1 * (1 - rcos),

		[6] = a1 * rsin + a0 * a2 * (1 - rcos),
		[7] = -a0 * rsin + a1 * a2 * (1 - rcos),
		[8] = rcos + a2 * a2 * (1 - rcos),
	};

	mat = matrix_from_array(3, 3, mat_arr);

	gsl_vector_free(axis_m);

	return mat;
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

gsl_vector *vector_scale_n(gsl_vector *v, const double l)
{
	gsl_vector *c = vector_clone(v);
	if (!c) {
		FATAL(ERR_MALLOC_FAIL);
	}

	gsl_vector_scale(c, l);

	return c;
}

gsl_vector *vector_add_constant_n(gsl_vector *v, const double l)
{
	gsl_vector *c = vector_clone(v);
	if (!c) {
		FATAL(ERR_MALLOC_FAIL);
	}

	gsl_vector_add_constant(c, l);

	return c;
}

void vector_print(gsl_vector *v)
{
	printf("[");
	for (size_t i = 0; i < v->size; i++) {
		printf("%.4g%s", v->data[i * v->stride], i + 1 < v->size ? ", " : "");
	}
	printf("]\n");
}

/***********
*  BLOCK  *
***********/

gsl_block *block_from_array(size_t n, const double *arr)
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

int block_update_array(gsl_block *b, size_t n, const double *arr)
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

gsl_block *block_linspace(double start, double end, size_t n)

{
	double inc = 0;
	double cur = start;
	gsl_block *b;

	if (!n) {
		return NULL;
	}

	b = gsl_block_calloc(n);

	if (n > 1) {
		inc = (end - start) / (n - 1);
	}

	for (size_t i = 0; i < n; i++) {
		b->data[i] = cur;
		cur += inc;
	}

	return b;
}

void block_expand(gsl_block *b)
{
	if (!b) {
		return;
	}

	b->data = realloc(b->data, 2 * b->size);
	if (!b->data) {
		FATAL(ERR_MALLOC_FAIL);
		return;
	}

	b->size *= 2;
}

void block_check_size(gsl_block *b, size_t sz)
{
	if (!b) {
		return;
	}

	size_t new_sz = b->size;

	while (new_sz < sz) {
		new_sz *= 2;
	}

	if (new_sz == b->size) {
		return;
	}

	b->data = realloc(b->data, new_sz);
	if (!b->data) {
		FATAL(ERR_MALLOC_FAIL);
		return;
	}

	b->size = new_sz;
}

void block_mul_constant_ex(gsl_block *b, double val, size_t n)
{
	for (size_t i = 0; i < n; ++i) {
		b->data[i] *= val;
	}
}

void block_add_constant_ex(gsl_block *b, double val, size_t n)
{
	for (size_t i = 0; i < n; ++i) {
		b->data[i] += val;
	}
}

void block_print(gsl_block *b)
{
	printf("[");
	for (size_t i = 0; i < b->size; i++) {
		printf("%.4g%s", b->data[i], i + 1 < b->size ? ", " : "");
	}
	printf("]\n");
}
