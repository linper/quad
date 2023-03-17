/**
 * @file matrix.c
 * @brief Implementation of matrix data structure and its interface
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include <json-c/json_object.h>
#include <json-c/json_types.h>

#include <log.h>
#include <stash.h>
#include "mth.h"
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

	for (size_t i = 0; i < a->sz; i++) {
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
	if (!a || !b || a->n_cols != b->n_cols || a->n_rows != b->n_rows) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (size_t i = 0; i < a->sz; i++) {
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
	if (!a || !b || a->n_cols != b->n_cols || a->n_rows != b->n_rows) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (size_t i = 0; i < a->sz; i++) {
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

	for (size_t i = 0; i < a->sz; i++) {
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
	if (!a || !b || a->n_cols != b->n_cols || a->n_rows != b->n_rows) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (size_t i = 0; i < a->sz; i++) {
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

	if (!a || !b || a->n_cols != b->n_cols || a->n_rows != b->n_rows) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (size_t i = 0; i < a->sz; i++) {
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

	for (size_t i = 0; i < a->sz; i++) {
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

	for (size_t i = 0; i < a->n_rows; i++) {
		for (size_t j = 0; j < a->n_cols; j++) {
			if (cb(a->arr[i * a->n_cols + j], priv, i, j)) {
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

	for (size_t i = 0; i < m->sz; i++) {
		sum += m->arr[i] * m->arr[i];
	}

	sum = sqrt(sum);

	return sum;
}

/**********
*  BASE  *
**********/

void mat_set(mat_t *mat, uint32_t m, uint32_t n, float val)
{
	if (!mat || n * m > mat->sz) {
		FATAL(ERR_INVALID_INPUT);
	}

	mat->arr[n * m] = val;
}

float mat_get(mat_t *mat, uint32_t m, uint32_t n)
{
	if (!mat || n * m > mat->sz) {
		FATAL(ERR_INVALID_INPUT);
	}

	return mat->arr[n * m];
}

mat_t *mat_get_row(mat_t *mat, uint32_t m)
{
	if (!mat || m > mat->n_rows) {
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	mat_t *ret = mat_create_ext(1, mat->n_cols, false, mat->stash);

	float *ptr = mat->arr + (mat->n_cols * m);

	for (size_t i = 0; i < mat->n_cols; i++) {
		ret->arr[i] = ptr[i];
	}

	return 0;
}

int mat_set_row(mat_t *mat, float *arr, uint32_t m)
{
	if (!mat || !arr || m > mat->n_rows) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	float *ptr = mat->arr + (mat->n_cols * m);

	for (size_t i = 0; i < mat->n_cols; i++) {
		ptr[i] = arr[i];
	}

	return 0;
}

int mat_set_column(mat_t *mat, float *arr, uint32_t n)
{
	if (!mat || !arr || n > mat->n_cols) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	float *ptr = mat->arr;

	for (size_t i = 0; i < mat->n_cols; i++) {
		ptr[i * mat->n_cols + n] = arr[i];
	}

	return 0;
}

int mat_map_mat(mat_t *dst, mat_t *src, uint32_t m, uint32_t n)
{
	if (!dst || !src || m + src->n_rows > dst->n_rows ||
		n + src->n_cols > dst->n_cols) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (size_t i = m, j = 0; i < m + dst->n_rows; i++, j++) {
		memcpy(dst->arr + (i * dst->n_cols) + n, src->arr + (j * src->n_cols),
			   src->n_cols);
	}

	return 0;
}

void mat_transpose(mat_t *m)
{
	float *arr, tmp;

	if (!m || m->is_view) {
		return;
	}

	if (m->n_cols == 1 || m->n_rows == 1) {
		tmp = m->n_cols;
		m->n_cols = m->n_rows;
		m->n_rows = tmp;

		return;
	}

	arr = malloc(m->sz);
	if (!arr) {
		FATAL(ERR_MALLOC_FAIL);
		return;
	}

	for (size_t i = 0; i < m->n_rows; i++) {
		for (size_t j = 0; j < m->n_cols; j++) {
			arr[j * m->n_rows + i] = m->arr[i * m->n_cols + j];
		}
	}

	tmp = m->n_cols;
	m->n_cols = m->n_rows;
	m->n_rows = tmp;

	free(m->arr);
	m->arr = arr;

	return;
}

mat_t *mat_dot(mat_t *a, mat_t *b)
{
	mat_t *m;

	if (!a || !b || a->n_cols != b->n_rows) {
		return NULL;
	}

	m = mat_create(a->n_rows, b->n_cols);
	if (!m) {
		return NULL;
	}

	for (size_t i = 0; i < a->n_rows; i++) {
		for (size_t j = 0; j < a->n_cols; j++) {
			for (size_t k = 0; k < b->n_rows; ++k) {
				for (size_t l = 0; l < b->n_cols; ++l) {
					m->arr[i * m->n_cols + l] +=
						a->arr[i * a->n_cols + j] * a->arr[k * b->n_cols + l];
				}
			}
		}
	}

	return m;
}

mat_t *mat_cross(mat_t *ma, mat_t *mb)
{
	if (!ma || !mb || !mat_dim_eq(ma, mb) || !mat_is_vec(ma) || ma->sz != 3) {
		return NULL;
	}

	float *a = ma->arr;
	float *b = mb->arr;

	float cross_arr[9] = {
		a[1] * b[2] - a[2] * b[1],
		a[2] * b[0] - a[0] * b[2],
		a[0] * b[1] - a[1] * b[0],
	};

	mat_t *res = mat_from_array(1, 3, cross_arr);

	return res;
}

mat_t *mat_identity(uint32_t edge)
{
	if (!edge) {
		return NULL;
	}

	mat_t *mat = mat_create_ext(edge, edge, true, NULL);
	if (!mat) {
		FATAL(ERR_MALLOC_FAIL);
	}

	for (size_t i = 0; i < edge; ++i) {
		mat->arr[i * edge + i] = 1.0;
	}

	return mat;
}

mat_t *mat_set_length_n(mat_t *m, float l)
{
	mat_t *c = mat_clone(m);
	if (!c) {
		FATAL(ERR_MALLOC_FAIL);
	}

	mat_set_length(c, l);

	return c;
}

mat_t *mat_rot_from_2vec(mat_t *from, mat_t *to)
{
	float phi, rcos, rsin, *axis;
	mat_t *from_, *to_, *axis_m, *mat;

	from_ = mat_set_length_n(from, 1.0);
	to_ = mat_set_length_n(to, 1.0);
	phi = mat_vec_angle(from_, to_);
	axis_m = mat_cross(from_, to_);
	mat_set_length(axis_m, 1.0);
	axis = axis_m->arr;

	rcos = cosf(phi);
	rsin = sinf(phi);

	float mat_arr[9] = {
		[0] = rcos + axis[0] * axis[0] * (1 - rcos),
		[1] = axis[2] * rsin + axis[1] * axis[0] * (1 - rcos),
		[2] = -axis[1] * rsin + axis[2] * axis[0] * (1 - rcos),

		[3] = -axis[2] * rsin + axis[0] * axis[1] * (1 - rcos),
		[4] = rcos + axis[1] * axis[1] * (1 - rcos),
		[5] = axis[0] * rsin + axis[2] * axis[1] * (1 - rcos),

		[6] = axis[1] * rsin + axis[0] * axis[2] * (1 - rcos),
		[7] = -axis[0] * rsin + axis[1] * axis[2] * (1 - rcos),
		[8] = rcos + axis[2] * axis[2] * (1 - rcos),
	};

	mat = mat_from_array(3, 3, mat_arr);

	mat_free(from_);
	mat_free(to_);
	mat_free(axis_m);

	return mat;
}

float mat_vec_cos(mat_t *a, mat_t *b)
{
	float la, lb, ret;
	mat_t *dot;

	la = mat_length(a);
	lb = mat_length(b);

	if (!la || !lb || !mat_dim_eq(a, b) || !mat_is_vec(a)) {
		return 0.0;
	} else {
		mat_transpose(b);
		dot = mat_dot(a, b);
		mat_transpose(b);

		if (!dot) {
			return 0.0;
		}

		if (dot->sz != 1) {
			mat_free(dot);
			return 0.0;
		}

		ret = dot->arr[0];
		mat_free(dot);

		return ret;
	}
}

int mat_reshape(mat_t *m, uint32_t n_rows, uint32_t n_cols)
{
	if (!m || n_cols * n_rows != m->sz) {
		return 1;
	}

	m->n_cols = n_cols;
	m->n_rows = n_rows;

	return 0;
}

mat_t *mat_akima(mat_t *X, mat_t *Y)
{
	mat_t *Xi, *Yi, *XM1, *XP1, *YM1, *YP1, *tmp1, *tmp2, *A, *base;

	/*Xi = X[1:len(X) - 1]*/
	/*Yi = Y[1:len(Y) - 1]*/
	/*XM1 = X[:len(X) - 2]*/
	/*XP1 = X[2:]*/
	/*YM1 = Y[:len(Y) - 2]*/
	/*YP1 = Y[2:]*/
	Xi = mat_view(X, 1, X->sz - 1);
	Yi = mat_view(Y, 1, Y->sz - 1);
	XM1 = mat_view(X, 0, X->sz - 2);
	YM1 = mat_view(Y, 0, Y->sz - 2);
	XP1 = mat_view(X, 2, X->sz);
	YP1 = mat_view(Y, 2, Y->sz);

	/*A = np.empty(shape=X.shape, dtype=X.dtype)*/
	/*A[1:len(A) - 1] = AM1 + Ai + AP1*/
	A = mat_create_ext(1, X->sz, true, X->stash);
	base = mat_view(A, 1, A->sz - 1);

	/*AM1 = (Xi - XP1) / ((XM1 - Xi) * (XM1 - XP1)) * YM1*/
	mat_add(base, Xi);
	mat_sub(base, XP1);
	tmp2 = mat_sub_n(XM1, Xi);
	tmp1 = mat_sub_n(XM1, XP1);
	mat_mul(tmp1, tmp2);
	mat_div(base, tmp1);
	mat_mul(base, YM1);
	mat_free(tmp1);
	mat_free(tmp2);

	/*Ai = (2 * Xi - XP1 - XM1) / ((Xi - XM1) * (Xi - XP1)) * Yi*/
	tmp2 = mat_sub_n(Xi, XM1);
	tmp1 = mat_sub_n(Xi, XP1);
	mat_mul(tmp1, tmp2);
	mat_add(tmp2, Xi);
	mat_sub(tmp2, XP1);
	mat_div(tmp2, tmp1);
	mat_mul(tmp2, Yi);
	mat_add(base, tmp2);
	mat_free(tmp1);
	mat_free(tmp2);

	/*AP1 = (Xi - XM1) / ((XP1 - XM1) * (XP1 - Xi)) * YP1*/
	tmp2 = mat_sub_n(XP1, XM1);
	tmp1 = mat_sub_n(XP1, Xi);
	mat_mul(tmp1, tmp2);
	mat_free(tmp2);
	tmp2 = mat_sub_n(Xi, XP1);
	mat_div(tmp2, tmp1);
	mat_mul(tmp2, YP1);
	mat_add(base, tmp2);
	mat_free(tmp1);
	mat_free(tmp2);

	/*A[0] = A[1]*/
	/*A[-1] = A[-2]*/
	A->arr[0] = A->arr[1];
	A->arr[A->sz - 1] = A->arr[A->sz- 2];

	mat_free(Xi);
	mat_free(Yi);
	mat_free(XM1);
	mat_free(XP1);
	mat_free(YM1);
	mat_free(YP1);
	mat_free(base);

	return A;
}

mat_t *mat_linspace(float start, float end, uint32_t n, stash_t *s)
{
	mat_t *mat;
	float inc = 0;
	float cur = start;

	if (!n) {
		return NULL;
	}

	mat = mat_create_ext(1, n, false, s);

	if (n > 1) {
		inc = (end - start) / (n - 1);
	}

	for (size_t i = 0; i < n; i++) {
		mat->arr[i] = cur;
		cur += inc;
	}

	return mat;
}

mat_t *mat_create_ext(uint32_t n_rows, uint32_t n_cols, bool zero, stash_t *s)
{
	mat_t *mat;

	if (!n_cols || !n_rows) {
		return NULL;
	}

	if (s) {
		mat = STASH_ALLOC(s, sizeof(mat_t));
	} else {
		mat = malloc(sizeof(mat_t));
		if (!mat) {
			FATAL(ERR_MALLOC_FAIL);
		}
	}

	if (s) {
		mat->arr = STASH_ALLOC(s, n_cols * n_rows * sizeof(float));
	} else if (zero) {
		mat->arr = calloc(n_cols * n_rows, sizeof(float));
	} else {
		mat->arr = malloc(n_cols * n_rows * sizeof(float));
	}

	if (!mat->arr) {
		FATAL(ERR_MALLOC_FAIL);
	}

	mat->n_rows = n_rows;
	mat->n_cols = n_cols;
	mat->sz = n_rows * n_cols;
	mat->stash = s;

	return mat;
}

mat_t *mat_clone(mat_t *m)
{
	mat_t *mat;

	if (!m) {
		return NULL;
	}

	if (m->stash) {
		mat = STASH_ALLOC(m->stash, sizeof(mat_t));
	} else {
		mat = malloc(sizeof(mat_t));
		if (!mat) {
			FATAL(ERR_MALLOC_FAIL);
		}
	}

	memcpy(mat, m, sizeof(mat_t));

	if (m->stash) {
		mat->arr = STASH_ALLOC(m->stash, m->sz * sizeof(float));
	} else {
		mat->arr = malloc(m->sz * sizeof(float));
		if (!mat->arr) {
			FATAL(ERR_MALLOC_FAIL);
		}
	}

	memcpy(mat->arr, m->arr, m->sz * sizeof(float));

	return mat;
}

mat_t *mat_slice(mat_t *m, uint32_t start, uint32_t end)
{
	mat_t *mat;

	if (!m) {
		return NULL;
	}

	if (m->stash) {
		mat = STASH_ALLOC(m->stash, sizeof(mat_t));
	} else {
		mat = malloc(sizeof(mat_t));
		if (!mat) {
			FATAL(ERR_MALLOC_FAIL);
		}
	}

	unsigned new_len = end - start;

	m->n_rows = 1;
	m->n_cols = new_len;
	m->sz = new_len;
	m->stash = mat->stash;

	if (m->stash) {
		mat->arr = STASH_ALLOC(m->stash, new_len * sizeof(float));
	} else {
		mat->arr = malloc(new_len * sizeof(float));
		if (!mat->arr) {
			FATAL(ERR_MALLOC_FAIL);
		}
	}

	memcpy(mat->arr, m->arr + start, new_len * sizeof(float));

	return mat;
}

mat_t *mat_view(mat_t *m, uint32_t start, uint32_t end)
{
	mat_t *mat;

	if (!m) {
		return NULL;
	}

	if (m->stash) {
		mat = STASH_ALLOC(m->stash, sizeof(mat_t));
	} else {
		mat = malloc(sizeof(mat_t));
		if (!mat) {
			FATAL(ERR_MALLOC_FAIL);
		}
	}

	unsigned new_len = end - start;

	m->n_rows = 1;
	m->n_cols = new_len;
	m->sz = new_len;
	m->stash = mat->stash;
	m->is_view = true;

	if (m->stash) {
		mat->arr = STASH_ALLOC(m->stash, new_len * sizeof(float));
	} else {
		mat->arr = malloc(new_len * sizeof(float));
		if (!mat->arr) {
			FATAL(ERR_MALLOC_FAIL);
		}
	}

	memcpy(mat->arr, m->arr + start, new_len * sizeof(float));

	return mat;
}

mat_t *mat_from_array_ext(uint32_t n_rows, uint32_t n_cols, const float *arr,
						  stash_t *s)
{
	if (!arr) {
		return NULL;
	}

	mat_t *mat = mat_create_ext(n_rows, n_cols, false, s);
	if (!mat) {
		return NULL;
	}

	memcpy(mat->arr, arr, n_cols * n_rows * sizeof(float));

	mat->n_rows = n_rows;
	mat->n_cols = n_cols;
	mat->sz = n_rows * n_cols;

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
		3 * (m->sz + 2 * m->n_rows + 2); // all brackets, spaces and commas
	for (size_t i = 0; i < m->sz; i++) {
		est_len += snprintf(NULL, 0, "%.5f", m->arr[i]);
	}

	char *buf = calloc(est_len, sizeof(char));
	if (!buf) {
		FATAL(ERR_MALLOC_FAIL);
	}

	if (m->n_cols > 1)
		off += sprintf(buf + off, "[");

	for (size_t i = 0; i < m->n_rows; i++) {
		if (i)
			off += sprintf(buf + off, ", ");

		off += sprintf(buf + off, "[");
		for (size_t j = 0; j < m->n_cols; j++) {
			if (j)
				off += sprintf(buf + off, ", ");

			off += sprintf(buf + off, "%.5f", m->arr[m->n_cols * i + j]);
		}

		off += sprintf(buf + off, "]");
	}

	if (m->n_cols > 1)
		off += sprintf(buf + off, "]");

	return buf;
}

struct json_object *mat_to_json(mat_t *m)
{
	struct json_object *l1, *l2;

	if (!m) {
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	bool single_dim = !!(m->n_cols == 1 || m->n_rows == 1);

	l1 = json_object_new_array_ext(m->n_rows);
	for (size_t i = 0; i < m->n_rows; i++) {
		if (single_dim) {
			for (size_t j = 0; j < m->n_cols; j++) {
				json_object_array_put_idx(
					l1, j, json_object_new_double(m->arr[m->n_cols * i + j]));
			}
		} else {
			l2 = json_object_new_array_ext(m->n_cols);
			for (size_t j = 0; j < m->n_cols; j++) {
				json_object_array_put_idx(
					l2, j, json_object_new_double(m->arr[m->n_cols * i + j]));
			}
			json_object_array_put_idx(l1, i, l2);
		}
	}

	return l1;
}

int mat_update_array(mat_t *mat, uint32_t l, float *arr)
{
	if (!mat || !arr || mat->n_cols * mat->n_rows != l) {
		return 1;
	}

	memcpy(mat->arr, arr, l * sizeof(float));

	return 0;
}

void mat_free(mat_t *m)
{
	if (m && !m->stash) {
		if (!m->is_view) {
			free(m->arr);
		}
		free(m);
	}
}

