/**
 * @file matrix.h
 * @brief Description of floating point matrix data structure and its interface
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>

#include <json-c/json.h>

/**
 * @brief Struct that holds all the matrix data.
 */
typedef struct matrix {
	u_int8_t m; ///< 		Number of rows
	u_int8_t n; ///< 		Number of columns
	u_int16_t sz; ///< 		Total size of matric array (m * n)
	float *arr; ///< 		Inner floating point array
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
inline int mat_sub_scal(mat_t *a, float b)
{
	return mat_add_scal(a, -b);
}

/**
 * @brief Subtracts `float` from each member of matrix. Returns new matix.
 * @param[in, out] 	*a 	Base matrix
 * @param[in] 		*b 	Argument to subtract from matrix members
 * @return Differnce of matrix and scalar in newly created matrix or NULL on failure.
 */
inline mat_t *mat_sub_scal_n(mat_t *a, float b)
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
inline int mat_div_scal(mat_t *a, float b)
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
inline mat_t *mat_div_scal_n(mat_t *a, float b)
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
 * @brief Function that executes iven function for all matrix elements.  
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
mat_t *mat_map_func_n(mat_t *a, int (*cb)(float, void *, int, int),
						 void *priv);

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
void mat_set(mat_t *mat, uint8_t m, uint8_t n, float val);

/**
 * @brief Gets value to en elemet. Fatal on error
 * @param[in] 	*mat 	Matrix instance.
 * @param[in] 	m 		Row index.
 * @param[in] 	n 		Column index.
 * @return Element value.
 */
float mat_get(mat_t *mat, uint8_t m, uint8_t n);

/**
 * @brief Sets whole row of values of the matrix. 
 * It is assumed that array length is equal to the 
 * length of the matrix row.
 * @param[in, out] 	*mat 	Matrix instance.
 * @param[in] 		*arr 	Array of values to set to matrix row.
 * @param[in] 		m 		Row index.
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_set_row(mat_t *mat, float *arr, uint8_t m);

/**
 * @brief Sets whole column of values of the matrix.
 * It is assumed that array length is equal to the 
 * length of the matrix column.
 * @param[in, out] 	*mat 	Matrix instance.
 * @param[in] 		*arr 	Array of values to set to matrix column.
 * @param[in] 		n 		Column index.
 * @return 0 on sucess, 1 - otherwise.
 */
int mat_set_column(mat_t *mat, float *arr, uint8_t n);

/**
 * @brief Allocates memory for matrix. 
 * @param[in] m 			Number of rows
 * @param[in] n 			Number of columns
 * @param[in] zero 			Do zero out memory
 * @return Newly created matrix struct pointer or NULL on failure.
 */
mat_t *mat_create(uint8_t m, uint8_t n, bool zero);

/**
 * @brief Clones given `mat_t` struct
 * @param[in] *m 	Martix to clone
 * @return Newly cloned matrix struct pointer or NULL on failure.
 */
mat_t *mat_clone(mat_t *m);

/**
 * @brief Allocates memory for matrix, copies to provided data to it. 
 * @param[in] m 			Number of rows. n * m must be equal to 
 * provided array size
 * @param[in] n 			Number of columns. n * m must be equal to 
 * provided array size
 * @param[in] *arr 			Input array to copy
 * @return Newly created matrix struct pointer or NULL on failure.
 */
mat_t *mat_from_array(uint8_t m, uint8_t n, float *arr);

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

