/**
 * @file model.h
 * @brief Description of two main datastructures that moded consists of (quad/model and leg).
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#ifndef MODEL_H
#define MODEL_H

#include <stdint.h>
#include <sys/types.h>

#include "matrix.h"

#define N_LEGS 4

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
	char name[16]; ///< 		Name of leg
	u_int8_t idx; ///< 			NON-CW index of leg
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
	sens_t *sens;

	/***************
	*  CONSTANTS  *
	***************/

	float max_dip; ///< 	How low can legs decend
	float leg_tar_h;
	float t_rad; ///< 		Touch radius
	int cw[4]; ///< 		Clock-wise seg sequence
} model_t;

/**
 * @brief Allocates memory for `sens_t` struct inside `g_model` and fills it with data prpvided by json `desc`.
 * @param[in] desc 		NULL terminated model description in json format.
 * @return 0 - on success, 1 - on failure.
 */
int set_sens_from_json(const char *desc);

/**
 * @brief Overrides data inside  for `sens_t` struct in `g_model` and fills it with data prpvided by json `desc`.
 * @param[in] desc 		NULL terminated model description in json format.
 * @return 0 - on success, 1 - on failure.
 */
int update_sens_from_json(const char *desc);

/**
 * @brief Allocates memory for model and fills it with data prpvided by json `desc`.
 * Updates global `g_model` struct.
 * @param[in] desc 		NULL terminated model description in json format. This should be
 * retreived form `sim` program.
 * @return 0 - on success, 1 - on failure.
 */
int model_from_json(const char *desc);

/**
 * @brief Frees `g_mod` model.
 * @param[in] 	*mod 	Model struct to free.
 * @return nothing
 */
void free_model(model_t *mod);

extern model_t *g_model;

#endif // MODEL_H
