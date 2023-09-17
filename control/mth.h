/**
 * @file mth.h
 * @brief Misc math stuff
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#pragma once

#include <stdbool.h>
#include <math.h>

#include <gsl/gsl_block_double.h>
#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_vector_double.h>

#include "log.h"

#define MIN(a, b) ((a) <= (b) ? (a) : (b))
#define MAX(a, b) ((a) >= (b) ? (a) : (b))
#define XOR(a, b) ((a) ^ (b))

/****************
*  VEC BUFFER  *
****************/
typedef struct vec_buffer {
	gsl_block *block;
	size_t cnt;
} vbuf_t;

void vbuf_free(vbuf_t *b);
vbuf_t *vbuf_create(size_t cap);
gsl_vector *vbuf_convert_to_vec(vbuf_t *b);
void vbuf_push(vbuf_t *b, double val);
double vbuf_pop(vbuf_t *b);
double vbuf_get(vbuf_t *b, size_t index);
int vbuf_set(vbuf_t *b, size_t index, double val);
int vbuf_insert(vbuf_t *b, size_t index, double val);
double vbuf_remove(vbuf_t *b, size_t index);
void vbuf_clear(vbuf_t *b);

/************
*  MATRIX  *
************/
gsl_matrix *matrix_from_array(size_t n_rows, size_t n_cols, const double *arr);
int matrix_update_array(const gsl_matrix *m, size_t n_rows, size_t n_cols,
						const double *arr);
gsl_matrix *matrix_del_row_n(gsl_matrix *m, size_t idx);
int matrix_fill_row(gsl_matrix *m, size_t idx, double val);
int matrix_fill_col(gsl_matrix *m, size_t idx, double val);
int matrix_add_vec_rows(gsl_matrix *m, gsl_vector *v);
int matrix_sub_vec_rows(gsl_matrix *m, gsl_vector *v);
gsl_vector *matrix_sum_axis(gsl_matrix *m, int axis);
gsl_vector *matrix_mean_axis(gsl_matrix *m, int axis);
gsl_matrix *matrix_clone(gsl_matrix *m);
gsl_matrix *matrix_sub_n(gsl_matrix *a, gsl_matrix *b);
gsl_matrix *matrix_add_n(gsl_matrix *a, gsl_matrix *b);
gsl_matrix *matrix_mul_n(gsl_matrix *a, gsl_matrix *b);
gsl_matrix *matrix_div_n(gsl_matrix *a, gsl_matrix *b);
gsl_matrix *matrix_scale_n(gsl_matrix *v, const double l);
gsl_matrix *matrix_add_constant_n(gsl_matrix *v, const double l);
gsl_matrix *matrix_linspace(gsl_vector *start, gsl_vector *end, size_t n);
gsl_matrix *matrix_dot(gsl_matrix *a, gsl_matrix *b);
gsl_vector *matrix_vector_dot(gsl_matrix *a, gsl_vector *b);
int matrix_copy_to(const gsl_matrix *dst, const gsl_matrix *src, size_t di,
				   size_t dj, size_t si, size_t sj, size_t ni, size_t nj);
struct json_object *matrix_to_json(gsl_matrix *m);
void matrix_print(gsl_matrix *m);

/************
*  VECTOR  *
************/
gsl_vector *vector_from_array(size_t n, const double *arr);
int vector_update_array(gsl_vector *v, size_t n, const double *arr);
double vector_dist(gsl_vector *from, gsl_vector *to);
double vector_length(gsl_vector *v);
void vector_set_length(gsl_vector *v, double l);
double vector_mean_range(gsl_vector *v, size_t off, size_t n);
double vector_std_range(gsl_vector *v, size_t off, size_t n);
void vector_increments(gsl_vector *v);
gsl_vector *vector_set_length_n(gsl_vector *v, double l);
gsl_vector *vector3_cross(gsl_vector *va, gsl_vector *vb);
double vector3_cos(gsl_vector *a, gsl_vector *b);
double vector_dot(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_project(gsl_vector *from, gsl_vector *to);
int vector_copy_to(gsl_vector *dst, gsl_vector *src, size_t di, size_t si,
				   size_t ni);
gsl_vector *vector_clone(gsl_vector *v);
gsl_vector *vector_linspace(double start, double end, size_t n);
gsl_matrix *mat_rot_from_2vec(gsl_vector *from, gsl_vector *to);
void matrix_increments(gsl_matrix *m);
void vector_sub_ddim(gsl_vector *a, gsl_vector *b);
void vector_add_ddim(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_sub_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_add_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_mul_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_div_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_scale_n(gsl_vector *v, const double l);
gsl_vector *vector_add_constant_n(gsl_vector *v, const double l);
struct json_object *vector_to_json(gsl_vector *v);
void vector_print(gsl_vector *v);

/***********
*  BLOCK  *
***********/
gsl_block *block_from_array(size_t n, const double *arr);
int block_update_array(gsl_block *b, size_t n, const double *arr);
struct json_object *block_to_json(gsl_block *b);
gsl_block *block_linspace(double start, double end, size_t n);
void block_expand(gsl_block *b);
void block_check_size(gsl_block *b, size_t sz);
void block_mul_constant_ex(gsl_block *b, double val, size_t n);
void block_add_constant_ex(gsl_block *b, double val, size_t n);
void block_print(gsl_block *b);

/*********
*  MISC  *
*********/
void line_cof(double x1, double y1, double x2, double y2, double *k, double *b);
bool ellipse_point_inside(double a, double b, double x, double y, double *val);
bool ellipse_line_intersect(double a, double b, double k, double c, double *x1,
							double *y1, double *x2, double *y2);
double area(const double *p1, const double *p2, const double *p3);
bool is_inside_triangle(const gsl_vector *pt, gsl_matrix *trig,
						double trig_acale, double *cof);
double bound_data(double dt, double lo, double hi);
int centroid_of_polygon(gsl_vector *res, gsl_matrix *pts);

/************
*  INLINE  *
************/
static inline bool ellipse_line_intersect2(double a, double b, double k,
										   double c, gsl_vector *p1,
										   gsl_vector *p2)
{
	return ellipse_line_intersect(a, b, k, c, gsl_vector_ptr(p1, 0),
								  gsl_vector_ptr(p1, 1), gsl_vector_ptr(p2, 0),
								  gsl_vector_ptr(p2, 1));
}

static inline gsl_matrix *matrix_calloc(size_t n1, size_t n2)
{
	gsl_matrix *m = gsl_matrix_calloc(n1, n2);
	if (!m)
		FATAL(ERR_PARSE_FAIL);
	return m;
}

static inline int matrix_copy_to_origin(const gsl_matrix *dst,
										const gsl_matrix *src)
{
	return matrix_copy_to(dst, src, 0, 0, 0, 0, src->size1, src->size2);
}

static inline gsl_vector *vector_calloc(size_t n)
{
	gsl_vector *v = gsl_vector_calloc(n);
	if (!v)
		FATAL(ERR_PARSE_FAIL);
	return v;
}

static inline double vector_mean(gsl_vector *v)
{
	return vector_mean_range(v, 0, v->size);
}

static inline double vector_std(gsl_vector *v)
{
	return vector_std_range(v, 0, v->size);
}

static inline double vector3_angle(gsl_vector *a, gsl_vector *b)
{
	return acos(vector3_cos(a, b));
}

static inline int vector_copy_to_origin(gsl_vector *dst, gsl_vector *src)
{
	return vector_copy_to(dst, src, 0, 0, src->size);
}

static inline gsl_block *block_calloc(size_t n)
{
	gsl_block *b = gsl_block_calloc(n);
	if (!b)
		FATAL(ERR_PARSE_FAIL);
	return b;
}

static inline void block_mul_constant(gsl_block *b, double val)
{
	return block_mul_constant_ex(b, val, b->size);
}

static inline void block_add_constant(gsl_block *b, double val)
{
	return block_add_constant_ex(b, val, b->size);
}

#define vector_foreach(e_, vec_)                                               \
	do {                                                                       \
		double e_ = vec_->data[0];                                             \
		for (size_t i = 0; i < vec_->size;                                     \
			 i++, e_ = vec_->data[i * vec_->stride])                           \
	} while (0);

/**
* container_of - cast a member of a structure out to the containing structure
* @ptr: the pointer to the member.
* @type: the type of the container struct this is embedded in.
* @member: the name of the member within the struct.
*
*/
#define container_of(ptr, type, member)                                        \
	({                                                                         \
		const typeof(((type *)0)->member) *__mptr = (ptr);                     \
		(type *)((char *)__mptr - offsetof(type, member));                     \
	})
