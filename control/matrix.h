/**
 * @file matrix.h
 * @brief Description of floating point matrix data structure and its interface
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>
#include <sys/types.h>

/**
 * @brief Struct that holds all the matrix data.
 */
typedef struct matrix {
	u_int8_t m; ///< 		Number of rows
	u_int8_t n; ///< 		Number of columns
	u_int16_t sz; ///< 		Total size of matric array (m * n)
	float *arr; ///< 		Inner floating point array
} mat_t;

/**
 * @brief Allocates memory for matrix. 
 * @param[in] m 			Number of rows
 * @param[in] n 			Number of columns
 * @param[in] zero 			Do zero out memory
 * @return Newly created matric struct pointer or NULL on failure.
 */
mat_t *create_matrix(uint8_t m, uint8_t n, bool zero);

/**
 * @brief Allocates memory for matrix, copies to provided data to it. 
 * @param[in] m 			Number of rows. n * m must be equal to 
 * provided array size
 * @param[in] n 			Number of columns. n * m must be equal to 
 * provided array size
 * @param[in] *arr 			Input array to copy
 * @return Newly created matric struct pointer or NULL on failure.
 */
mat_t *matrix_from_array(uint8_t m, uint8_t n, float *arr);

/**
 * @brief Updates internal array with given one.
 * @param[in, out] 	*mat 	Matrix to update. 		
 * @param[in] 		l 		Size of provided arrau. MUST (n * m == l)
 * @param[in] 		*arr 	Input array to copy
 * @return Newly created matric struct pointer or NULL on failure.
 */
int matrix_update_array(mat_t *mat, uint32_t l, float *arr);

/**
 * @brief Frees Matrix struct.
 * @param[in] 	*m 			Matrix struct pointer
 * @return nothing
 */
void free_matrix(mat_t *m);

#endif // MATRIX_H
