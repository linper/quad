/**
 * @file stance.h
 * @brief Globl movement plan
 * @author Linas Perkauskas
 * @date 2023-05-05
 */

#pragma once

#include <stdbool.h>

#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>

#include "glist.h"

struct stance;
struct movement;

typedef struct stance {
	struct stance *next;
	struct stance *prev;
	gsl_matrix *pts; // (4 x 2)
	gsl_matrix *orig_pts; // (4 x 2)
	gsl_matrix *def_pts; // (4 x 2)
	gsl_vector *com; // (2)
	gsl_vector *bcom; // (2)
	gsl_vector *com_off; // (2)
	gsl_vector *dir; // (2)
	size_t pidx; // leg idx to move to reach this stance
	double min_loss;
	gsl_matrix *bpts; // (3 x 2)
	gsl_matrix *mod_pts; // (4 x 2)
	gsl_vector *mod_com; // (2)
	/*gsl_vector *mod_bcom; // (2)*/
} stance_t;

typedef struct movement {
	bool need_sched; ///< 			Is movement resceduling needed
	struct stance *st;
	size_t n;
	double loss;
	gsl_matrix *target_path; // Inteded movement path
	gsl_matrix *target_dirs; // Inteded movement directions
	gsl_vector *dir; //
	gsl_matrix *coms; // (n, 2)
	gsl_matrix *bcoms; // (n, 2)
	gsl_matrix *path; // (n, 2)
	gsl_spline *x_spl;
	gsl_spline *y_spl;
	gsl_interp_accel *acc;
} movement_t;

void movement_free(movement_t *self);
void get_movement(gsl_matrix *pts, gsl_matrix *dirs, bool resched);
