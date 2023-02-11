/**
 * @file model.h
 * @brief Description of two main datastructures that moded consists of (quad/model and leg).
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#pragma once

#include "json_helper.h"
#include <stdint.h>
#include <sys/types.h>

#include "matrix.h"

#define N_LEGS 4
#define NAME_LEN 16

typedef struct leg leg_t;

/**
 * @brief Struct that describes sensor data and data derived from it.
 */
typedef struct sens {
	float avg_leg_h; ///< 		Average leg height
	mat_t *touch_f; ///< 		Force that legs apply to ground (4 x 1)
	mat_t *damp; ///< 			Leg dampener distance (4 x 1) SHOULD NOT BE NEEDED
	mat_t *bf_vec; ///< 		Base force vector (3 x 1)
	mat_t *bfo_mat; ///< 		Base frame orientation matrix (3 x 3)
	int tf_type; ///< 			Some force ??? type
	mat_t *tf_pos; ///< 		Some force ??? position ? (3 x 1)
} sens_t;

/**
 * @brief Struct that describes leg of the robot.
 */
typedef struct leg {
	leg_t *next; ///< 			Next non-cw leg
	leg_t *cw_next; ///< 		Next cw leg
	char name[NAME_LEN]; ///< 	Name of leg
	float angles[3]; ///< 		Calculated joint angles
	int	 idx; ///< 				NON-CW index of leg
	mat_t *pos; ///< 			Current position of leg (3 x 1)
	mat_t *def_pos; ///< 		Default position of leg (3 x 1)
	mat_t *base_off; ///< 		Shoulder offset from COM (3 x 1)
	mat_t *dir; ///< 			Direction multipliers (3 x 1)
	mat_t *joint_lims; ///< 	Leg joint angle limits (6 x 1)
} leg_t;

/**
 * @brief Struct that describes base of robot.
 */
typedef struct model {
	leg_t *legs[N_LEGS];
	leg_t *cw_legs[N_LEGS];
	sens_t *sens; ///< 		Sensor info
	mat_t *angles; ///< 	Calculated joint angles

	/***************
	*  CONSTANTS  *
	***************/

	float max_dip; ///< 	How low can legs decend
	float leg_tar_h;
	float t_rad; ///< 		Touch radius
	float link_len; ///< 	Length of single leg link
	int cw[4]; ///< 		Clock-wise seg sequence
} model_t;

/**
 * @brief Allocates memory for `sens_t` struct inside `g_model` and fills it with data provided by `json_struct` struct .
 * @param[in] 	*j 		`json_object` struct as data	
 * @return 0 - on success, 1 - on failure.
 */
int set_sens_from_json(struct json_object *j);

/**
 * @brief Updates `sens_t` struct inside `g_model` and fills it with data provided by `json_struct` struct .
 * @param[in] 	*j 		`json_object` struct as data	
 * @return 0 - on success, 1 - on failure.
 */
int update_sens_from_json(struct json_object *j);

/**
 * @brief Allocates memory for model and fills it with data prpvided by json `desc`.
 * Updates global `g_model` struct.
 * @param[in] 	*j 		`json_object` struct as data	
 * retreived form `sim` program.
 * @return 0 - on success, 1 - on failure.
 */
int model_from_json(struct json_object *j);

/**
 * @brief Calculates joint angles for model.
 * @return Nothing.
 */
void model_get_angles();
/**
 * @brief Frees `g_mod` model.
 * @param[in] 	*mod 	Model struct to free.
 * @return Nothing
 */
void free_model(model_t *mod);

extern model_t *g_model;

