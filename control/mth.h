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

gsl_matrix *matrix_from_array(unsigned n_rows, unsigned n_cols,
							  const double *arr);
int matrix_update_array(gsl_matrix *m, unsigned n_rows, unsigned n_cols,
						const double *arr);
struct json_object *matrix_to_json(gsl_matrix *m);

double vector_length(gsl_vector *v);
gsl_vector *vector_from_array(unsigned n, const double *arr);
int vector_update_array(gsl_vector *v, unsigned n, const double *arr);
gsl_vector *vector_sub_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_add_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_mul_n(gsl_vector *a, gsl_vector *b);
gsl_vector *vector_div_n(gsl_vector *a, gsl_vector *b);
struct json_object *vector_to_json(gsl_vector *v);

gsl_block *block_from_array(unsigned n, const double *arr);
int block_update_array(gsl_block *b, unsigned n, const double *arr);
struct json_object *block_to_json(gsl_block *b);
