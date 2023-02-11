
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

#include "mth.h"
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
		mat_free(s->bf_vec);
		mat_free(s->bfo_mat);
		mat_free(s->damp);
		mat_free(s->tf_pos);
		mat_free(s->touch_f);
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
		mat_free(l->pos);
		mat_free(l->def_pos);
		mat_free(l->base_off);
		mat_free(l->dir);
		mat_free(l->joint_lims);
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

	s->touch_f = mat_create(1, 4, true);
	s->damp = mat_create(1, 4, true);
	s->bf_vec = mat_create(1, 3, true);
	s->bfo_mat = mat_create(3, 3, true);
	s->tf_pos = mat_create(1, 3, true);

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
		s->touch_f = mat_from_array(1, 4, tf);
		s->damp = mat_from_array(1, 4, d);
		s->bf_vec = mat_from_array(1, 3, bv);
		s->bfo_mat = mat_from_array(3, 3, bm);
		s->tf_pos = mat_from_array(1, 3, tfp);

		if (!s->touch_f || !s->damp || !s->bf_vec || !s->bfo_mat ||
			!s->tf_pos) {
			ERR(ERR_PARSE_FAIL);
			free_sens(s);
		}
	} else {
		ret |= mat_update_array(s->touch_f, 4, tf);
		ret |= mat_update_array(s->damp, 4, d);
		ret |= mat_update_array(s->bf_vec, 3, bv);
		ret |= mat_update_array(s->bfo_mat, 9, bm);
		ret |= mat_update_array(s->tf_pos, 3, tfp);

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

	l->pos = mat_from_array(1, 3, p);
	l->def_pos = mat_from_array(1, 3, dp);
	l->base_off = mat_from_array(1, 3, bo);
	l->dir = mat_from_array(1, 3, d);
	l->joint_lims = mat_from_array(1, 6, jl);

	if (!l->pos || !l->def_pos || !l->base_off || !l->dir || !l->joint_lims) {
		ERR(ERR_PARSE_FAIL);
		free_leg(l);
	}

	return l;
}

/**
 * @brief Allocates memory for `leg_t` struct and fills it with data prpvided by json `desc`.
 * @param[in] desc 		NULL terminated model description in json format.
 * @return Void.
 */
static void leg_get_angles(leg_t *l)
{
	float *dir, *lims, *target, gama, leg_len, max_leg_len, min_leg_len, e,
		alpha, theta, phi, beta;
	mat_t *tg_mat;

	tg_mat = mat_sub_n(l->pos, l->base_off);
	target = tg_mat->arr;
	lims = l->joint_lims->arr;
	dir = l->dir->arr;

	gama = atan(-target[1] / target[2]);
	if (lims[0] > gama) {
		target[1] = -target[2] * tanf(gama);
	} else if (lims[1] < gama) {
		gama = lims[1];
		target[1] = -target[2] * tanf(gama);
	}
	// Substract shoulder position
	target[1] -= 0.03 * sin(gama) + dir[2] * 0.01 * cos(gama);
	target[2] -= -0.03 * cos(gama) + dir[2] * 0.01 * sin(gama);

	leg_len = mat_length(tg_mat);
	max_leg_len = g_model->link_len * sqrt(5 - 4 * cos(fabs(lims[5])));
	min_leg_len = g_model->link_len * sqrt(5 - 4 * cos(fabs(lims[4])));

	if (max_leg_len > leg_len && leg_len > min_leg_len) { // middle
		e = acos(1.25 - ((leg_len * leg_len) /
						 (4 * g_model->link_len * g_model->link_len)));
		alpha = dir[0] * (M_PI - e);
	} else if (leg_len >= max_leg_len) { // outer
		alpha = lims[4];
	} else { // inner
		alpha = lims[5];
	}

	phi = asin(target[0] / leg_len);
	theta =
		asin(bound_data((g_model->link_len * sin(alpha)) / leg_len, -1.0, 1.0));

	if (lims[2] <= -phi - theta && -phi - theta <= lims[3]) {
		beta = -phi + -theta; // VII
	} else if (lims[2] > -phi - theta) {
		beta = lims[2]; // VIII
	} else {
		beta = lims[3];
	}

	l->angles[0] = alpha;
	l->angles[1] = beta;
	l->angles[2] = gama;

	mat_free(tg_mat);
}

void model_get_angles()
{
	leg_t *l;

	for (int i = 0; i < N_LEGS; i++) {
		l = g_model->legs[i];
		leg_get_angles(l);
		mat_set_row(g_model->angles, l->angles, i);
	}
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
		"{\"max_dip\": %f, \"leg_tar_h\": %f, \"t_rad\": %f, \"cw\": [%d, %d, %d, %d], \"link_len\": %f}",
		&mod->max_dip, &mod->leg_tar_h, &mod->t_rad, &mod->cw[0], &mod->cw[1],
		&mod->cw[2], &mod->cw[3], &mod->link_len);
	if (ret != 8) {
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
			free_model(mod);
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

	mod->angles = mat_create(4, 3, true);
	if (!mod->angles) {
		ERR(ERR_ERROR);
		free_model(mod);
		return 1;
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

		mat_free(mod->angles);
		free(mod);
	}
}

