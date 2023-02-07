
/**
 * @file model.c
 * @brief Implementation of two main datastructures that moded consists of (quad/model and leg).
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <json_helper.h>
#include <log.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "matrix.h"
#include "model.h"

model_t *g_model = NULL;

/**
 * @brief Frees `sens_t` struct.
 * @return nothing
 */
static void free_sens(sens_t *s)
{
	if (s) {
		free_matrix(s->bf_vec);
		free_matrix(s->bfo_mat);
		free_matrix(s->damp);
		free_matrix(s->tf_pos);
		free_matrix(s->touch_f);
		free(s);
	}
}

/**
 * @brief Frees `leg_t` struct.
 * @return nothing
 */
static void free_leg(leg_t *l)
{
	if (l) {
		free_matrix(l->pos);
		free_matrix(l->def_pos);
		free_matrix(l->base_off);
		free_matrix(l->dir);
		free_matrix(l->joint_lims);
		free(l);
	}
}

/**
 * Allocates sensor data struct and places into `g_model`
 * @return Allocated sensor data struct pointer or NULL on failure.
 */
static sens_t *alloc_sens()
{
	sens_t *s = calloc(1, sizeof(sens_t));
	if (!s) {
		FATAL(ERR_MALLOC_FAIL);
	}

	s->touch_f = create_matrix(4, 1, true);
	s->damp = create_matrix(4, 1, true);
	s->bf_vec = create_matrix(3, 1, true);
	s->bfo_mat = create_matrix(3, 3, true);
	s->tf_pos = create_matrix(3, 1, true);

	if (!s->touch_f || !s->damp || !s->bf_vec || !s->bfo_mat || !s->tf_pos) {
		ERR(ERR_PARSE_FAIL);
		free_sens(s);
		return NULL;
	}

	return s;
}

int set_sens_from_json(const char *desc)
{
	int ret;
	float tf[4], d[4], bv[3], bm[9], tfp[3];
	bool has_sens;
	sens_t *s;

	if (!desc || !g_model) {
		return 1;
	}

	has_sens = !!g_model->sens;

	if (!has_sens) {
		g_model->sens = alloc_sens();
		if (!g_model->sens) {
			return 1;
		}
	}

	s = g_model->sens;

	// Parse model data
	ret = sscanf(
		desc,
		"{\"avg_leg_h\": %f, \"touch_force\": [%f, %f, %f, %f], \"damp\": [%f, %f, %f, %f], \"bf_vec\": [%f, %f, %f], \"bfo_mat\": [[%f, %f, %f], [%f, %f, %f], [%f, %f, %f]], \"t_force\": {\"type\": %f, \"pos\": [%f, %f, %f]}}",
		&s->avg_leg_h, &tf[0], &tf[1], &tf[2], &tf[4], &d[0], &d[1], &d[2],
		&d[2], &d[3], &bv[0], &bv[1], &bv[2], &bm[0], &bm[1], &bm[2], &bm[3],
		&bm[4], &bm[5], &bm[6], &bm[7], &bm[8], &tfp[0], &tfp[1], &tfp[2]);
	if (ret != 25) {
		ERR(ERR_PARSE_FAIL);
		free_sens(s);
		return 1;
	}

	if (!has_sens) {
		s->touch_f = matrix_from_array(4, 1, tf);
		s->damp = matrix_from_array(4, 1, d);
		s->bf_vec = matrix_from_array(3, 1, bv);
		s->bfo_mat = matrix_from_array(3, 3, bm);
		s->tf_pos = matrix_from_array(3, 1, tfp);

		if (!s->touch_f || !s->damp || !s->bf_vec || !s->bfo_mat ||
			!s->tf_pos) {
			ERR(ERR_PARSE_FAIL);
			free_sens(s);
		}
	} else {
		ret |= matrix_update_array(s->touch_f, 4, tf);
		ret |= matrix_update_array(s->damp, 4, d);
		ret |= matrix_update_array(s->bf_vec, 3, bv);
		ret |= matrix_update_array(s->bfo_mat, 9, bm);
		ret |= matrix_update_array(s->tf_pos, 3, tfp);

		if (ret) {
			ERR(ERR_PARSE_FAIL);
			free_sens(s);
		}
	}

	return 0;
}

/**
 * @brief Allocates memory for `leg_t` struct and fills it with data prpvided by json `desc`.
 * @param[in] desc 		NULL terminated model description in json format.
 * @return 0 - on success, 1 - on failure.
 */
static leg_t *leg_from_json(const char *desc)
{
	leg_t *l;
	int ret;
	float p[3], dp[3], bo[3], d[3], jl[6];

	if (!desc) {
		return NULL;
	}

	l = calloc(1, sizeof(leg_t));
	if (!l) {
		FATAL(ERR_MALLOC_FAIL);
	}
	// Parse model data
	ret = sscanf(
		desc,
		"{\"name\": \"%15[^\"]\", \"idx\": %hhu, \"pos\": [%f, %f, %f], \"def_pos\": [%f, %f, %f], \"base_off\": [%f, %f, %f], \"dir\": [%f, %f, %f], \"joint_lims\": [%f, %f, %f, %f, %f, %f]}",
		l->name, &l->idx, &p[0], &p[1], &p[2], &dp[0], &dp[1], &dp[2], &bo[0],
		&bo[1], &bo[2], &d[0], &d[1], &d[2], &jl[0], &jl[1], &jl[2], &jl[3],
		&jl[4], &jl[5]);
	if (ret != 20) {
		ERR(ERR_PARSE_FAIL);
		free(l);
		return NULL;
	}

	l->pos = matrix_from_array(3, 1, p);
	l->def_pos = matrix_from_array(3, 1, dp);
	l->base_off = matrix_from_array(3, 1, bo);
	l->dir = matrix_from_array(3, 1, d);
	l->joint_lims = matrix_from_array(6, 1, jl);

	if (!l->pos || !l->def_pos || !l->base_off || !l->dir || !l->joint_lims) {
		ERR(ERR_PARSE_FAIL);
		free_leg(l);
	}

	return l;
}

int model_from_json(const char *desc)
{
	struct json_status st;
	int ret;
	const char *ptr, *legs_ptr;
	model_t *mod;
	leg_t *l;

	if (!desc) {
		return 1;
	}

	ptr = json_help_step_in(desc);
	if (!ptr) {
		ERR(ERR_PARSE_FAIL);
		return 1;
	}

	if (json_help_analyze(ptr, &st)) {
		ERR(ERR_PARSE_FAIL);
		return 1;
	}

	// save pointer to leg data for later
	legs_ptr = st.inner;

	ptr = st.start + st.n;

	if (json_help_analyze(ptr, &st)) {
		ERR(ERR_PARSE_FAIL);
		return 1;
	}

	mod = calloc(1, sizeof(model_t));
	if (!mod) {
		FATAL(ERR_MALLOC_FAIL);
	}

	// Parse model data
	ret = sscanf(
		st.start,
		"{\"max_dip\": %f, \"leg_tar_h\": %f, \"t_rad\": %f, \"cw\": [%d, %d, %d, %d]}",
		&mod->max_dip, &mod->leg_tar_h, &mod->t_rad, &mod->cw[0], &mod->cw[1],
		&mod->cw[2], &mod->cw[3]);
	if (ret != 7) {
		ERR(ERR_PARSE_FAIL);
		free(mod);
		return 1;
	}

	// Parse legs
	for (int i = 0; i < N_LEGS; i++) {
		if (json_help_analyze(legs_ptr, &st)) {
			ERR(ERR_PARSE_FAIL);
			free_model(mod);
			return 1;
		}

		l = leg_from_json(st.start);
		if (!l) {
			ERR(ERR_PARSE_FAIL);
			free(mod);
			return 1;
		}

		legs_ptr = st.start + st.n;

		mod->legs[l->idx] = l;
		mod->cw_legs[mod->cw[l->idx]] = l;
	}

	// Setup "next" legs
	for (int i = 0; i < N_LEGS; i++) {
		mod->legs[i]->next = mod->legs[(i + 1) % N_LEGS];
		mod->cw_legs[i]->cw_next = mod->cw_legs[(i + 1) % N_LEGS];
	}

	// Free previous model
	free_model(g_model);
	g_model = mod;

	return 0;
}

void free_model(model_t *mod)
{
	if (mod) {
		for (int i = 0; i < N_LEGS; i++) {
			free_leg(mod->legs[i]);
		}

		free(mod);
	}
}

