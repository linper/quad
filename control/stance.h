/**
 * @file stance.h
 * @brief Globl movement plan
 * @author Linas Perkauskas
 * @date 2023-05-05
 */

#pragma once

#include <gsl/gsl_matrix_double.h>

void get_movement(gsl_vector *dir, gsl_matrix *pts);
