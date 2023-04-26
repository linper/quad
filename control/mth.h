/**
 * @file mth.h
 * @brief Misc math stuff
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#pragma once

#include <gsl/gsl_block_double.h>
#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_vector_double.h>

#include <math.h>

#define MIN(a, b) ((a) <= (b) ? (a) : (b))
#define MAX(a, b) ((a) >= (b) ? (a) : (b))
#define XOR(a, b) ((a) ^ (b))

double area(double *p1, double *p2, double *p3);

double bound_data(double dt, double lo, double hi);

gsl_matrix *matrix_from_array(size_t n_rows, size_t n_cols, const double *arr);
int matrix_update_array(gsl_matrix *m, size_t n_rows, size_t n_cols,
						const double *arr);
gsl_matrix *matrix_dot(gsl_matrix *a, gsl_matrix *b);
gsl_vector *matrix_vector_dot(gsl_matrix *a, gsl_vector *b);
int matrix_copy_to(gsl_matrix *dst, gsl_matrix *src, size_t di, size_t dj,
				   size_t si, size_t sj, size_t ni, size_t nj);
struct json_object *matrix_to_json(gsl_matrix *m);
void matrix_print(gsl_matrix *m);

gsl_vector *vector_from_array(size_t n, const double *arr);
int vector_update_array(gsl_vector *v, size_t n, const double *arr);
double vector_length(gsl_vector *v);
void vector_set_length(gsl_vector *v, double l);
gsl_vector *vector_set_length_n(gsl_vector *v, double l);
gsl_vector *vector3_cross(gsl_vector *va, gsl_vector *vb);
double vector3_cos(gsl_vector *a, gsl_vector *b);
double vector_dot(gsl_vector *a, gsl_vector *b);
int vector_copy_to(gsl_vector *dst, gsl_vector *src, size_t di, size_t si,
				   size_t ni);
gsl_vector *vector_clone(gsl_vector *v);
gsl_vector *vector_linspace(double start, double end, size_t n);
gsl_matrix *mat_rot_from_2vec(gsl_vector *from, gsl_vector *to);
gsl_vector *vector_sub_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_add_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_mul_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_div_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_scale_n(gsl_vector *v, const double l);
gsl_vector *vector_add_constant_n(gsl_vector *v, const double l);
struct json_object *vector_to_json(gsl_vector *v);
void vector_print(gsl_vector *v);

gsl_block *block_from_array(size_t n, const double *arr);
int block_update_array(gsl_block *b, size_t n, const double *arr);
struct json_object *block_to_json(gsl_block *b);
gsl_block *block_linspace(double start, double end, size_t n);
void block_expand(gsl_block *b);
void block_check_size(gsl_block *b, size_t sz);
void block_mul_constant_ex(gsl_block *b, double val, size_t n);
void block_add_constant_ex(gsl_block *b, double val, size_t n);
void block_print(gsl_block *b);

/************
*  INLINE  *
************/

static inline int matrix_copy_to_origin(gsl_matrix *dst, gsl_matrix *src)
{
	return matrix_copy_to(dst, src, 0, 0, 0, 0, src->size1, src->size2);
}

static inline double vector3_angle(gsl_vector *a, gsl_vector *b)
{
	return acos(vector3_cos(a, b));
}

static inline int vector_copy_to_origin(gsl_vector *dst, gsl_vector *src)
{
	return vector_copy_to(dst, src, 0, 0, src->size);
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
