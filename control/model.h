/**
 * @file model.h
 * @brief Description of two main datastructures that moded consists of (quad/model and leg).
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#pragma once

//#include "json_helper.h"
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#include <gsl/gsl_block_double.h>
#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_matrix_double.h>

#include "matrix.h"
#include "fsm.h"
#include "plan.h"
#include "pidc.h"

#define N_LEGS 4
#define NAME_LEN 16

typedef struct leg leg_t;

/**
 * @brief Struct that describes sensor data and data derived from it.
 */
typedef struct sens {
	double avg_leg_h; ///< 		Average leg height
	double abs_std_leg_h;
	bool h_hit[4]; ///< 		Does leg hard hit
	bool s_hit[4]; ///< 		Does leg hard hit
	double damp_dst[4]; ///< 	1 - normalized damp
	double walk_h[4]; ///<
	gsl_matrix *balance; ///< 	Balanciing offsets (4 x 3)
	gsl_block *touch_f; ///< 	Force that legs apply to ground (1 x 4)
	gsl_block *damp; ///< 		Leg dampener distance (1 x 4) SHOULD NOT BE NEEDED
	gsl_vector *bf_vec; ///< 	Base force vector (1 x 3)
	gsl_matrix *bfo_mat; ///< 	Base frame orientation matrix (3 x 3)
	int tf_type; ///< 			Some force ??? type
	gsl_vector *tf_pos; ///< 	Some force ??? position ? (1 x 3)
	//gsl_matrix *s_center; ///< 		FOR THE VIEW (1 x 3)
	//gsl_matrix *to_s_closest; ///< 	FOR THE VIEW (1 x 3)
} sens_t;

/**
 * @brief Struct that describes leg of the robot.
 */
typedef struct leg {
	int idx; ///< 				NON-CW index of leg
	char name[NAME_LEN]; ///< 	Name of leg
	plan_t plan; ///< 			Leg movement plan.
	fsm_t *fsm; ///< 			FSM of the leg.
	pidc_t balance_pid; ///< 	Balancing PID controler.
	pidc_t touch_pid; ///< 		Touch/contact PID controler.
	bool bal; ///< 				Is this leg is currentry balanced
	leg_t *next; ///< 			Next non-cw leg
	leg_t *cw_next; ///< 		Next cw leg
	gsl_vector *angles; ///< 	Calculated joint angles
	gsl_vector *pos; ///< 		Current position of leg (1 x 3)
	gsl_vector *def_pos; ///< 	Default position of leg (1 x 3)
	gsl_vector *base_off; ///< 	Shoulder offset from COM (1 x 3)
	gsl_vector *dir; ///< 		Direction multipliers (1 x 3)
	gsl_vector *joint_lims; ///< 	Leg joint angle limits (1 x 6)
} leg_t;

/**
 * @brief Struct that describes base of robot.
 */
typedef struct model {
	leg_t *legs[N_LEGS]; ///< 		Leg pointer array
	leg_t *cw_legs[N_LEGS]; ///< 	Clockwise leg ptr array(1st front-left)
	sens_t *sens; ///< 				Sensor info
	gsl_matrix *angles; ///< 		Calculated joint angles (4 x 3)

	/***************
	*  CONSTANTS  *
	***************/

	double max_dip; ///< 		How low can legs decend
	double min_dip; ///< 		How high can legs ascend
	double max_walk_h; ///< 		How low can legs decend
	double min_walk_h; ///< 		How high can legs ascend
	double leg_tar_h;
	double t_rad; ///< 			Touch radius
	double link_len; ///< 		Length of single leg link
	double soft_hit_thr; ///< 	Treshold for soft hit
	int cw[4]; ///< 			Clock-wise seg sequence
} model_t;

/**
 * @brief Allocates memory for `sens_t` struct inside `g_model` and fills it with data provided by `json_struct` struct .
 * @param[in] 	*j 		`json_object` struct as data	
 * @return 0 - on success, 1 - on failure.
 */
int set_sens_from_json(struct json_object *j);

/**
 * @brief Allocates memory for model and fills it with data prpvided by json `desc`.
 * Updates global `g_model` struct.
 * @param[in] 	*j 		`json_object` struct as data	
 * retreived form `sim` program.
 * @return 0 - on success, 1 - on failure.
 */
int model_from_json(struct json_object *j);

/**
 * @brief Calculates next step according to sensor data and currents state.
 * @return Nothing.
 */
void model_step();

/**
 * @brief Frees `g_mod` model.
 * @param[in] 	*mod 	Model struct to free.
 * @return Nothing
 */
void free_model(model_t *mod);

extern model_t *g_model;

