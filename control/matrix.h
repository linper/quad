/**
 * @file matrix.h
 * @brief Description of floating point matrix data structure and its interface
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <json-c/json.h>

#include "stash.h"
#include <string.h>

/**
 * @brief Struct that holds all the matrix data.
 */
typedef struct matrix {
	uint32_t n_rows; ///< 	Number of rows
	uint32_t n_cols; ///< 	Number of columns
	size_t sz; ///< 		Total size of matric array (m * n)
	float *arr; ///< 		Inner floating point array
	stash_t *stash; ///< 	Stash struct to store internal memory (opt)
	bool is_view; ///< 		Is view
} mat_t;

/*************
*  ACTIONS  *
*************/

/**
 * @brief Adds `float` to each member of matrix.
 * @param[in, out] 	*a 	Result matrix
 * @param[in] 		*b 	Argument to modify matrix
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_add_scal(mat_t *a, float b);

/**
 * @brief Adds `float` to each member of matrix. Returns new matix
 * @param[in, out] 	*a 	Base matrix
 * @param[in] 		*b 	Argument to add to matrix members
 * @return Sum of matrix and scalar in newly created matrix or NULL on failure.
 */
mat_t *mat_add_scal_n(mat_t *a, float b);

/**
 * @brief Adds two maricies. Dimensions of matrices must match.  
 * @param[in, out] 	*a 	First/result matrix
 * @param[in] 		*b 	Second/argument matrix
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_add(mat_t *a, mat_t *b);

/**
 * @brief Adds two maricies. Returns new matix. Dimensions of matrices must match.  
 * @param[in] *a 	First matrix
 * @param[in] *b 	Second matrix
 * @return Sum of matrices in newly created matrix or NULL on failure.
 */
mat_t *mat_add_n(mat_t *a, mat_t *b);

/**
 * @brief Subtracts `float` from each member of matrix.
 * @param[in, out] 	*a 	Result matrix
 * @param[in] 		*b 	Argument to modify matrix
 * @return 0 on sucess, 1 - otherwise.
 */
static inline int mat_sub_scal(mat_t *a, float b)
{
	return mat_add_scal(a, -b);
}

/**
 * @brief Subtracts `float` from each member of matrix. Returns new matix.
 * @param[in, out] 	*a 	Base matrix
 * @param[in] 		*b 	Argument to subtract from matrix members
 * @return Differnce of matrix and scalar in newly created matrix or NULL on failure.
 */
static inline mat_t *mat_sub_scal_n(mat_t *a, float b)
{
	return mat_add_scal_n(a, -b);
}

/**
 * @brief Difference of  two maricies. Dimensions of matrices must match.  
 * @param[in, out] 	*a 	First/result matrix
 * @param[in] 		*b 	Second/argument matrix
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_sub(mat_t *a, mat_t *b);

/**
 * @brief Difference of two maricies. Returns new matix. Dimensions of matrices must match.  
 * @param[in] *a 	First matrix
 * @param[in] *b 	Second matrix
 * @return Difference of matrices in newly created matrix or NULL on failure.
 */
mat_t *mat_sub_n(mat_t *a, mat_t *b);

/**
 * @brief Multiplies each member of matrix by`float`.
 * @param[in, out] 	*a 	Result matrix
 * @param[in] 		*b 	Argument to modify matrix
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_mul_scal(mat_t *a, float b);

/**
 * @brief Multiplies each member of matrix by`float`. Returns new matix
 * @param[in, out] 	*a 	Base matrix
 * @param[in] 		*b 	Argument to multiply to matrix members
 * @return Multiplication result of matrix and scalar in newly created matrix or NULL on failure.
 */
mat_t *mat_mul_scal_n(mat_t *a, float b);

/**
 * @brief Multiply ELEMENTWISE two maricies. Dimensions of matrices must match.  
 * @param[in, out] 	*a 	First/result matrix
 * @param[in] 		*b 	Second/argument matrix
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_mul(mat_t *a, mat_t *b);

/**
 * @brief Multiply ELEMENTWISE two maricies. Returns new matix. Dimensions of matrices must match.  
 * @param[in] *a 	First matrix
 * @param[in] *b 	Second matrix
 * @return Multiplication result of matrices in newly created matrix or NULL on failure.
 */
mat_t *mat_mul_n(mat_t *a, mat_t *b);

/**
 * @brief Divides matrix elements by `float`.
 * @param[in, out] 	*a 	Result matrix
 * @param[in] 		*b 	Argument to modify matrix
 * @return 0 on sucess, 1 - otherwise.
 */
static inline int mat_div_scal(mat_t *a, float b)
{
	if (b)
		return mat_mul_scal(a, 1.0 / b);
	else
		return 1;
}

/**
 * @brief Divides matrix elements by `float`. Returns new matix.
 * @param[in, out] 	*a 	Base matrix
 * @param[in] 		*b 	Argument to divide from matrix members by
 * @return Division result of matrix and scalar in newly created matrix or NULL on failure.
 */
static inline mat_t *mat_div_scal_n(mat_t *a, float b)
{
	if (b)
		return mat_mul_scal_n(a, 1.0 / b);
	else
		return NULL;
}

/**
 * @brief Elemtwise division result of two maricies. Dimensions of matrices must match.  
 * @param[in, out] 	*a 	First/result matrix
 * @param[in] 		*b 	Second/argument matrix
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_sub(mat_t *a, mat_t *b);

/**
 * @brief Elementwise division resultof two maricies. Returns new matix. Dimensions of matrices must match.  
 * @param[in] *a 	First matrix
 * @param[in] *b 	Second matrix
 * @return Elemtwise division result of matrices in newly created matrix or NULL on failure.
 */
mat_t *mat_sub_n(mat_t *a, mat_t *b);

/**
 * @brief Reduces matrix values to absolute.  
 * @param[in, out] 	*a 	Matrix to modify.
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_abs(mat_t *a);

/**
 * @brief Reduces matrix values to absolute.  
 * @param[in] *a 	Reference matrix.
 * @return newly created matrix with absolute values of matrix `a` or NULL on failure.
 */
mat_t *mat_abs_n(mat_t *a);

/**
 * @brief Function that executes iven function for a/ll matrix elements.  
 * @param[in] *a 		Reference matrix.
 * @param[in] *cb 		Callback to execute.
 * Callback header:
 * 		Element value
 * 		User pointer
 * 		Element row
 * 		Element column
 * @param[in] *priv 	Reference matrix.
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_map_func(mat_t *a, int (*cb)(float, void *, int, int), void *priv);

/**
 * @brief Function that executes iven function for all matrix elements.  
 * @param[in] *a 		Reference matrix.
 * @param[in] *cb 		Callback to execute.
 * Callback header:
 * 		Element value
 * 		User pointer
 * 		Element row
 * 		Element column
 * @param[in] *priv 	Reference matrix.
 * @return New matrix or NULL on failure.
 */
mat_t *mat_map_func_n(mat_t *a, int (*cb)(float, void *, int, int), void *priv);

/**
 * @brief Returns sum of squered elements. Fatal on error;
 * @param[in] 	*m 	Matrix instance.
 * @return Sum of squared elements.
 */
float mat_length(mat_t *m);

/**********
*  BASE  *
**********/

/**
 * @brief Sets value to en elemet. Fatal on error
 * @param[in] 	*mat 	Matrix instance.
 * @param[in] 	m 		Row index.
 * @param[in] 	n 		Column index.
 * @param[in] 	val 	Value to set.
 * @return Nothing.
 */
void mat_set(mat_t *mat, uint32_t m, uint32_t n, float val);

/**
 * @brief Gets value to en elemet. Fatal on error
 * @param[in] 	*mat 	Matrix instance.
 * @param[in] 	m 		Row index.
 * @param[in] 	n 		Column index.
 * @return Element value.
 */
float mat_get(mat_t *mat, uint32_t m, uint32_t n);

/**
 * @brief Gets and allocates row from a matrix. Retreived row is
 * stashed if base matrix is also stashed.
 * @param[in] 	*mat 	Matrix instance.
 * @param[in] 	m 		Row index.
 * @return Element value.
 */
mat_t *mat_get_row(mat_t *mat, uint32_t m);

/**
 * @brief Sets whole row of values of the matrix. 
 * It is assumed that array length is equal to the 
 * length of the matrix row.
 * @param[in, out] 	*mat 	Matrix instance.
 * @param[in] 		*arr 	Array of values to set to matrix row.
 * @param[in] 		m 		Row index.
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_set_row(mat_t *mat, float *arr, uint32_t m);

/**
 * @brief Sets whole column of values of the matrix.
 * It is assumed that array length is equal to the 
 * length of the matrix column.
 * @param[in, out] 	*mat 	Matrix instance.
 * @param[in] 		*arr 	Array of values to set to matrix column.
 * @param[in] 		n 		Column index.
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_set_column(mat_t *mat, float *arr, uint32_t n);

/**
 * @brief Map one matrix's values onto other. 
 * @param[in, out] 	*dst 	Matrix to map values onto.
 * @param[in] 		*src 	Matrix to map values from.
 * @param[in] 		im 		Row index to start mapping from.
 * @param[in] 		n 		Column index to start maping from.
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_map_mat(mat_t *dst, mat_t *src, uint32_t m, uint32_t n);

/**
 * @brief Function that transposes matrix. 
 * @param[in, out] 	*m 	Matrix to transpose.
 * @return Nothing.
 */
void mat_transpose(mat_t *m);

/**
 * @brief Dot procuct of two matrices. 
 * @param[int] 	*a 	First matrix.
 * @param[int] 	*b 	Second matrix.
 * @return new matrix, or NULL on failure.
 */
mat_t *mat_dot(mat_t *a, mat_t *b);

/**
 * @brief Dot procuct of two vectors of size 3. 
 * @param[int] 	*a 	First vector.
 * @param[int] 	*b 	Second vector.
 * @return new vector, or NULL on failure.
 */
mat_t *mat_cross(mat_t *ma, mat_t *mb);

/**
 * @brief Allocates identity for matrix. 
 * @param[in] edge 		Number of rows and columns
 * @return Newly created matrix struct pointer or NULL on failure.
 */
mat_t *mat_identity(uint32_t edge);

/**
 * @brief Streatches matrix/vector to specified length. 
 * @param[in] *m 	Matrix to modify.
 * @param[in] l 	New length of matrix.
 * @return Nothing.
 */
static inline void mat_set_length(mat_t *m, float l)
{
	mat_mul_scal(m, l / mat_length(m));
}

/**
 * @brief Creates new matrix of specified length base ongiven matrix. 
 * @param[in] *m 	Base matrix.
 * @param[in] l 	Length of new matrix.
 * @return New matrix of NULL on failure.
 */
mat_t *mat_set_length_n(mat_t *m, float l);

/**
 * @brief Creates rotation matrix from 2 vectors. 
 * @param[in] *from 	Starting vector.
 * @param[in] *to 		End vector.
 * @return New vecotr of NULL on failure.
 */
mat_t *mat_rot_from_2vec(mat_t *from, mat_t *to);

/** @brief calculates cosine between 2 vecotrs. */
float mat_vec_cos(mat_t *a, mat_t *b);

/** @brief calculates cosine between 2 vecotrs. */
static inline float mat_vec_angle(mat_t *a, mat_t *b)
{
	return acosf(mat_vec_cos(a, b));
}

int mat_reshape(mat_t *m, uint32_t n_rows, uint32_t n_cols);
mat_t *mat_akima(mat_t *X, mat_t *Y);
mat_t *mat_linspace(float start, float end, uint32_t n, stash_t *s);

/** @brief Checks if 2 matrices have equal dimensions. */
static inline bool mat_dim_eq(mat_t *a, mat_t *b)
{
	return (a && b && a->n_cols == b->n_cols && a->n_rows == b->n_rows);
}

/** @brief Checks if 2 matrices have transposed dimensions. */
static inline bool mat_dim_tr(mat_t *a, mat_t *b)
{
	return (a && b && a->n_cols == b->n_rows && a->n_rows == b->n_cols);
}

/** @brief Checks if matix can be treated as a vector. */
static inline bool mat_is_vec(mat_t *a)
{
	return (a && (a->n_cols == 1 || a->n_rows == 1));
}

/**
 * @brief Allocates memory for matrix. 
 * @param[in] n_rows 		Number of rows
 * @param[in] n_cols 		Number of columns
 * @param[in] zero 			Do zero out memory
 * @return Newly created matrix struct pointer or NULL on failure.
 */
mat_t *mat_create_ext(uint32_t n_rows, uint32_t n_cols, bool zero, stash_t *s);

/**
 * @brief Fills matriix with values. I.e. memset.
 * @return Nothing
 * */
static inline void mat_fill(mat_t *m, float val)
{
	if (m) {
		for (size_t i = 0; i < m->sz; i++) {
			m->arr[i] = val;
		}
	}
}

static inline mat_t *mat_create(uint32_t n_rows, uint32_t n_cols)
{
	return mat_create_ext(n_rows, n_cols, true, NULL);
}

/**
 * @brief Clones given `mat_t` struct
 * @param[in] *m 	Martix to clone
 * @return Newly cloned matrix struct pointer or NULL on failure.
 */
mat_t *mat_clone(mat_t *m);
mat_t *mat_slice(mat_t *m, uint32_t start, uint32_t end);
mat_t *mat_view(mat_t *m, uint32_t start, uint32_t end);

/**
 * @brief Allocates memory for matrix, copies to provided data to it. 
 * @param[in] n_rows 		Number of rows. n * m must be equal to 
 * provided array size
 * @param[in] n_cols 		Number of columns. n * m must be equal to 
 * provided array size
 * @param[in] *arr 			Input array to copy
 * @param[in] *s 			Stash for internal memory (opt)
 * @return Newly created matrix struct pointer or NULL on failure.
 */
mat_t *mat_from_array_ext(uint32_t n_rows, uint32_t n_cols, const float *arr,
						  stash_t *s);

static inline mat_t *mat_from_array(uint32_t n_rows, uint32_t n_cols,
									const float *arr)
{
	return mat_from_array_ext(n_rows, n_cols, arr, NULL);
}

/**
 * @brief Returns stringified matrix inside allocates string buffer.
 * @param[in] 	*m 	Matrix to update.
 * @return Allocated string buffer of NULL on failure.
 */
char *mat_to_str(mat_t *m);

/**
 * @brief Returns serialized matrix inside `json_object` struct.
 * @param[in] 	*m 	Matrix to serialize.
 * @return Allocated `json_object` or NULL on failure.
 */
struct json_object *mat_to_json(mat_t *m);

/**
 * @brief Updates internal array with given one.
 * @param[in, out] 	*mat 	Matrix to update. 		
 * @param[in] 		l 		Size of provided arrau. MUST (n * m == l)
 * @param[in] 		*arr 	Input array to copy
 * @return Newly created matric struct pointer or NULL on failure.
 */
int mat_update_array(mat_t *mat, uint32_t l, float *arr);

/**
 * @brief Frees Matrix struct.
 * @param[in] 	*m 			Matrix struct pointer
 * @return nothing
 */
void mat_free(mat_t *m);

