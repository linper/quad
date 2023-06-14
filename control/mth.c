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

/****************
*  VEC BUFFER  *
****************/
void vbuf_free(vbuf_t *b)
{
	if (!b) {
		return;
	}

	gsl_block_free(b->block);
	free(b);
}

vbuf_t *vbuf_create(size_t cap)
{
	gsl_block *b = block_calloc(cap);
	vbuf_t *vb = calloc(1, sizeof(vbuf_t));
	if (!b || !vb) {
		FATAL(ERR_MALLOC_FAIL);
	}

	vb->block = b;

	return vb;
}

gsl_vector *vbuf_convert_to_vec(vbuf_t *b)
{
	if (!b) {
		return NULL;
	}

	gsl_vector *v = gsl_vector_alloc_from_block(b->block, 0, b->cnt, 1);
	if (!v) {
		FATAL(ERR_MALLOC_FAIL);
	}

	free(b);

	return v;
}

void vbuf_push(vbuf_t *b, double val)
{
	if (!b) {
		return;
	}

	if (b->cnt + 1 > b->block->size) {
		block_expand(b->block);
	}

	b->block->data[b->cnt++] = val;
}

double vbuf_pop(vbuf_t *b)
{
	if (!b || !b->cnt) {
		return NAN;
	}

	return b->block->data[--b->cnt];
}

double vbuf_get(vbuf_t *b, size_t index)
{
	if (!b || index > b->cnt) {
		return NAN;
	}

	return b->block->data[index];
}

int vbuf_set(vbuf_t *b, size_t index, double val)
{
	if (!b || index > b->cnt) {
		return 1;
	}

	b->block->data[index] = val;

	return 0;
}

int vbuf_insert(vbuf_t *b, size_t index, double val)
{
	if (!b || index > b->cnt) {
		return 1;
	}

	if (b->cnt + 1 > b->block->size) {
		block_expand(b->block);
	}

	memmove(b->block->data + index, b->block->data + index + 1,
			sizeof(double) * (b->cnt - index));

	b->block->data[b->cnt++] = val;

	return 0;
}

double vbuf_remove(vbuf_t *b, size_t index)
{
	if (!b || index > b->cnt - 1) {
		return NAN;
	}

	double val = b->block->data[index];

	memmove(b->block->data + index + 1, b->block->data + index,
			sizeof(double) * (b->cnt - index - 1));

	return val;
}

void vbuf_clear(vbuf_t *b)
{
	if (!b) {
		return;
	}

	memset(b->block->data, 0, sizeof(double) * b->block->size);
	b->cnt = 0;
}

/************
*  Matrix  *
************/

gsl_matrix *matrix_from_array(size_t n_rows, size_t n_cols, const double *arr)
{
	if (!arr) {
		return NULL;
	}

	gsl_matrix *m = matrix_calloc(n_rows, n_cols);

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

gsl_matrix *matrix_clone(gsl_matrix *m)
{
	if (!m) {
		return NULL;
	}
	gsl_matrix *c = matrix_calloc(m->size1, m->size2);

	gsl_matrix_memcpy(c, m);

	return c;
}

gsl_matrix *matrix_sub_n(gsl_matrix *a, gsl_matrix *b)
{
	if (!a || !b) {
		return NULL;
	}

	gsl_matrix *m = matrix_calloc(a->size1, a->size2);

	gsl_matrix_memcpy(m, a);

	gsl_matrix_sub(m, b);

	return m;
}

gsl_matrix *matrix_add_n(gsl_matrix *a, gsl_matrix *b)
{
	if (!a || !b) {
		return NULL;
	}

	gsl_matrix *m = matrix_calloc(a->size1, a->size2);

	gsl_matrix_memcpy(m, a);

	gsl_matrix_add(m, b);

	return m;
}

gsl_matrix *matrix_mul_n(gsl_matrix *a, gsl_matrix *b)
{
	if (!a || !b) {
		return NULL;
	}

	gsl_matrix *m = matrix_calloc(a->size1, a->size2);

	gsl_matrix_memcpy(m, a);

	gsl_matrix_mul_elements(m, b);

	return m;
}

gsl_matrix *matrix_div_n(gsl_matrix *a, gsl_matrix *b)
{
	if (!a || !b) {
		return NULL;
	}

	gsl_matrix *m = matrix_calloc(a->size1, a->size2);

	gsl_matrix_memcpy(m, a);

	gsl_matrix_div_elements(m, b);

	return m;
}

gsl_matrix *matrix_scale_n(gsl_matrix *v, const double l)
{
	if (!v) {
		return NULL;
	}

	gsl_matrix *c = matrix_clone(v);

	gsl_matrix_scale(c, l);

	return c;
}

gsl_matrix *matrix_add_constant_n(gsl_matrix *v, const double l)
{
	if (!v) {
		return NULL;
		;
	}

	gsl_matrix *c = matrix_clone(v);

	gsl_matrix_add_constant(c, l);

	return c;
}

gsl_matrix *matrix_linspace(gsl_vector *start, gsl_vector *end, size_t n)
{
	gsl_vector *cur, *inc;
	gsl_matrix *m;

	if (!n || !start || !end || start->size != end->size) {
		return NULL;
	}

	cur = vector_clone(start);
	inc = vector_clone(end);
	m = matrix_calloc(n, start->size);

	if (n > 1) {
		gsl_vector_sub(inc, start);
		gsl_vector_scale(inc, 1.0 / (n - 1));
	}

	for (size_t i = 0; i < n; i++) {
		for (size_t j = 0; j < start->size; j++) {
			gsl_matrix_set(m, i, j, gsl_vector_get(cur, j));
		}

		gsl_vector_add(cur, inc);
	}

	gsl_vector_free(cur);
	gsl_vector_free(inc);

	return m;
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

gsl_matrix *matrix_del_row_n(gsl_matrix *m, size_t idx)
{
	if (!m || m->size1 <= idx || m->size1 == 1) {
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	gsl_matrix *d = matrix_calloc(m->size1 - 1, m->size2);


	for (size_t i = 0, j = 0; i < m->size1; i++) {
		if (i != idx) {
			memcpy(d->data + d->tda * j, m->data + d->tda * i,
				   m->tda * sizeof(double));
			j++;
		}
	}

	return d;
}

int matrix_add_vec_rows(gsl_matrix *m, gsl_vector *v)
{
	if (!m || !v || m->size2 != v->size) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (size_t i = 0; i < m->size2; i++) {
		double val = gsl_vector_get(v, i);
		for (size_t j = 0; j < m->size1; j++) {
			*gsl_matrix_ptr(m, j, i) -= val;
		}
	}

	return 0;
}

int matrix_sub_vec_rows(gsl_matrix *m, gsl_vector *v)
{
	if (!m || !v || m->size2 != v->size) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	for (size_t i = 0; i < m->size2; i++) {
		double val = gsl_vector_get(v, i);
		for (size_t j = 0; j < m->size1; j++) {
			*gsl_matrix_ptr(m, j, i) -= val;
		}
	}

	return 0;
}

gsl_vector *matrix_sum_axis(gsl_matrix *m, int axis)
{
	gsl_vector *v;

	if (!m || axis < 0 || axis > 1) {
		return NULL;
	}

	if (!axis) {
		v = vector_calloc(m->size2);

		for (size_t i = 0; i < m->size1; i++) {
			for (size_t j = 0; j < m->size2; j++) {
				v->data[j * v->stride] += m->data[i * m->tda + j];
			}
		}

	} else {
		v = vector_calloc(m->size1);

		for (size_t i = 0; i < m->size1; i++) {
			for (size_t j = 0; j < m->size2; j++) {
				v->data[i * v->stride] += m->data[i * m->tda + j];
			}
		}
	}

	return v;
}

gsl_matrix *matrix_dot(gsl_matrix *a, gsl_matrix *b)
{
	double ai, bi, temp;
	gsl_matrix *m;

	if (!a || !b || a->size2 != b->size1) {
		return NULL;
	}

	m = matrix_calloc(a->size1, b->size2);
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
		ERR(ERR_INVALID_INPUT);
		return NULL;
	}

	m = vector_calloc(b->size);

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
	if (!n || !arr) {
		FATAL(ERR_INVALID_INPUT);
	}

	gsl_vector *v = vector_calloc(n);

	for (size_t i = 0; i < n; i++) {
		v->data[i * v->stride] = arr[i];
	}

	return v;
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
	if (!v) {
		FATAL(ERR_INVALID_INPUT);
	}

	gsl_vector *c = vector_calloc(v->size);
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

	v = vector_calloc(n);

	if (n > 1) {
		inc = (end - start) / (n - 1);
	}

	for (size_t i = 0; i < n; i++) {
		gsl_vector_set(v, i, cur);
		cur += inc;
	}

	return v;
}

double vector_dist(gsl_vector *from, gsl_vector *to)
{
	double sum = 0.0, e;

	if (!from || !to) {
		return 0.0;
	}

	for (size_t i = 0; i < from->size; i++) {
		e = gsl_vector_get(from, i) - gsl_vector_get(to, i);
		sum += e * e;
	}

	sum = sqrt(sum);

	return sum;
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

double vector_mean_range(gsl_vector *v, size_t off, size_t n)
{
	if (!v) {
		return NAN;
	}

	double sum = 0.0;
	for (size_t i = off; i < off + n; i++) {
		sum += v->data[v->stride * i];
	}

	double mean = sum / n;

	return mean;
}

double vector_std_range(gsl_vector *v, size_t off, size_t n)
{
	double sum, std, mean, val;

	if (!v) {
		return NAN;
	}

	mean = vector_mean_range(v, off, n);

	sum = 0.0;
	for (size_t i = off; i < off + n; i++) {
		val = mean - v->data[v->stride * i];
		val *= val;
		sum += val;
	}

	std = sum / n;
	std = sqrt(std);

	return std;
}

void vector_increments(gsl_vector *v)
{
	if (!v) {
		return;
	}

	for (size_t i = 0; i < v->size - 1; i++) {
		v->data[v->stride * i] =
			v->data[v->stride * (i + 1)] - v->data[v->stride * i];
	}

	v->data[v->stride * (v->size - 1)] = 0.0;
	v->size--;
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

gsl_vector *vector_project(gsl_vector *from, gsl_vector *to)
{
	double len = vector_dot(from, to);
	len /= vector_length(to);
	gsl_vector *v = vector_set_length_n(to, len);

	return v;
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
	if (!a || !b) {
		return NULL;
	}

	gsl_vector *m = vector_calloc(a->size);
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

	m = vector_calloc(a->size);

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

	m = vector_calloc(a->size);

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

	m = vector_calloc(a->size);

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

	m = block_calloc(n);
	if (!m) {
		FATAL(ERR_MALLOC_FAIL);
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

	b = block_calloc(n);
	if (!b) {
		FATAL(ERR_MALLOC_FAIL);
	}

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

/*********
*  MISC  *
*********/
static bool sq_roots(double a, double b, double c, double *r1, double *r2)
{
	double D = sqrt((b * b - 4 * a * c));

	if (D >= 0) {
		if (r1)
			*r1 = (-b + D) / (2 * a);
		if (r2)
			*r2 = (-b - D) / (2 * a);

		return true;
	} else {
		if (r1)
			*r1 = 0.0;
		if (r2)
			*r2 = 0.0;

		return false;
	}
}

bool ellipse_point_inside(double a, double b, double x, double y, double *val)
{
	double v = ((x * x) / (a * a)) + ((y * y) / (b * b));
	v = v * v;
	*val = v;

	return v <= 1.0;
}

bool ellipse_line_intersect(double a, double b, double k, double c, double *x1,
							double *y1, double *x2, double *y2)
{
	double ra, rb, rc, r1, r2;
	bool inter;
	ra = (k * k * a * a) + b * b;
	rb = 2 * k * c * (a * a);
	rc = ((c * c) - (b * b)) * (a * a);
	inter = sq_roots(ra, rb, rc, &r1, &r2);

	if (inter) {
		if (x1)
			*x1 = r1;
		if (y1)
			*y1 = k * r1 + c;
		if (x2)
			*x2 = r2;
		if (y2)
			*y2 = k * r2 + c;
	} else {
		if (x1)
			*x1 = 0.0;
		if (y1)
			*y1 = 0.0;
		if (x2)
			*x2 = 0.0;
		if (y2)
			*y2 = 0.0;
	}

	return inter;
}

void line_cof(double x1, double y1, double x2, double y2, double *k, double *b)
{
	double k_val, b_val;
	double fi = 0.0000001;

	k_val = (y2 - y1) / (x2 - x1 + fi);
	b_val = y1 - k_val * x1;
	if (!k_val)
		k_val = fi;

	*k = k_val;
	*b = b_val;
}

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

int centroid_of_polygon(gsl_vector *res, gsl_matrix *pts)
{
	if (!res || !pts) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	double crs_sum = 0.0;
	double *x_sum = gsl_vector_ptr(res, 0);
	double *y_sum = gsl_vector_ptr(res, 1);

	for (size_t i = 0; i < pts->size1; ++i) {
		size_t j = (i + 1) % pts->size1;
		double cross = gsl_matrix_get(pts, i, 0) * gsl_matrix_get(pts, j, 1) -
					   gsl_matrix_get(pts, j, 0) * gsl_matrix_get(pts, i, 1);

		crs_sum += cross;
		*x_sum +=
			(gsl_matrix_get(pts, i, 0) + gsl_matrix_get(pts, j, 0)) * cross;
		*y_sum +=
			(gsl_matrix_get(pts, i, 1) + gsl_matrix_get(pts, j, 1)) * cross;
	}

	double z = 1.0 / (3.0 * crs_sum);

	gsl_vector_scale(res, z);

	return 0;
}
