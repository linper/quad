
/**
 * @file model.c
 * @brief Implementation of two main datastructures that moded consists of (quad/model and leg).
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <json-c/json_types.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <json-c/json_object.h>
#include <json-c/json_tokener.h>
#include <json-c/json.h>

#include "log.h"
#include "mth.h"
#include "matrix.h"
#include "model.h"
#include "json_helper.h"

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

int set_sens_from_json(struct json_object *j)
{
	float tf[4], d[4], bv[3], bm[9], tfp[3];
	struct json_object *tmp, *tmp2, *arr;
	int ret = 0;
	bool has_sens;
	sens_t *s;

	if (!j || !g_model) {
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

	// Avg_leg_h
	if (!json_object_object_get_ex(j, "avg_leg_h", &tmp)) {
		goto err;
	}

	s->avg_leg_h = json_object_get_double(tmp);

	// Touch_force
	if (!json_object_object_get_ex(j, "touch_force", &arr)) {
		goto err;
	}

	for (int i = 0; i < 4; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		tf[i] = json_object_get_int(tmp);
	}

	// Damp
	if (!json_object_object_get_ex(j, "damp", &arr)) {
		goto err;
	}

	for (int i = 0; i < 3; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		d[i] = json_object_get_double(tmp);
	}

	// Bf_vvec
	if (!json_object_object_get_ex(j, "bf_vec", &arr)) {
		goto err;
	}

	for (int i = 0; i < 3; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		bv[i] = json_object_get_double(tmp);
	}

	// Bfo_mat
	if (!json_object_object_get_ex(j, "bfo_mat", &arr)) {
		goto err;
	}

	for (int i = 0; i < 3; i++) {
		if (!(tmp2 = json_object_array_get_idx(arr, i)))
			goto err;
		for (int j = 0; j < 3; j++) {
			if (!(tmp = json_object_array_get_idx(tmp2, j)))
				goto err;

			bm[i * 3 + j] = json_object_get_double(tmp);
		}
	}

	// T_force
	if (!json_object_object_get_ex(j, "t_force", &tmp2)) {
		goto err;
	}

	// T_force: Type
	if (!json_object_object_get_ex(tmp2, "type", &tmp)) {
		goto err;
	}

	s->tf_type = json_object_get_double(tmp);

	// T_force: Pos
	if (!json_object_object_get_ex(tmp2, "pos", &arr)) {
		goto err;
	}

	for (int i = 0; i < 3; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		tfp[i] = json_object_get_double(tmp);
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

err:
	ERR(ERR_PARSE_FAIL);
	free_sens(s);
	return 1;
}

/**
 * @brief Allocates memory for `leg_t` struct and fills it with data prpvided by json `desc`.
 * @param[in] desc 		NULL terminated model description in json format.
 * @return 0 - on success, 1 - on failure.
 */
static leg_t *leg_from_json(struct json_object *j)
{
	leg_t *l = NULL;
	float p[3], dp[3], bo[3], d[3], jl[6];
	struct json_object *tmp, *arr;

	if (!j) {
		return NULL;
	}

	l = calloc(1, sizeof(leg_t));
	if (!l) {
		FATAL(ERR_MALLOC_FAIL);
	}

	// Name
	if (!json_object_object_get_ex(j, "name", &tmp)) {
		goto err;
	}

	strncpy(l->name, json_object_get_string(tmp), NAME_LEN);

	// Index
	if (!json_object_object_get_ex(j, "idx", &tmp)) {
		goto err;
	}

	l->idx = json_object_get_int(tmp);

	// Position
	if (!json_object_object_get_ex(j, "pos", &arr)) {
		goto err;
	}

	for (int i = 0; i < 3; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		p[i] = json_object_get_double(tmp);
	}

	// Default position
	if (!json_object_object_get_ex(j, "def_pos", &arr)) {
		goto err;
	}

	for (int i = 0; i < 3; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		dp[i] = json_object_get_double(tmp);
	}

	// Base offset
	if (!json_object_object_get_ex(j, "base_off", &arr)) {
		goto err;
	}

	for (int i = 0; i < 3; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		bo[i] = json_object_get_double(tmp);
	}

	// Dir
	if (!json_object_object_get_ex(j, "dir", &arr)) {
		goto err;
	}

	for (int i = 0; i < 3; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		d[i] = json_object_get_double(tmp);
	}

	// Joint_limit
	if (!json_object_object_get_ex(j, "joint_lims", &arr)) {
		goto err;
	}

	for (int i = 0; i < 6; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		jl[i] = json_object_get_double(tmp);
	}

	l->pos = mat_from_array(1, 3, p);
	l->def_pos = mat_from_array(1, 3, dp);
	l->base_off = mat_from_array(1, 3, bo);
	l->dir = mat_from_array(1, 3, d);
	l->joint_lims = mat_from_array(1, 6, jl);

	if (!l->pos || !l->def_pos || !l->base_off || !l->dir || !l->joint_lims) {
		goto err;
	}

	return l;
err:
	ERR(ERR_PARSE_FAIL);
	free(l);

	return NULL;
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

int model_from_json(struct json_object *j)
{
	model_t *mod;
	leg_t *l;
	struct json_object *tmp, *tmp2, *arr;

	if (!j) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	mod = calloc(1, sizeof(model_t));
	if (!mod) {
		FATAL(ERR_MALLOC_FAIL);
	}

	printf("MODEL:%s\n", json_object_get_string(j));

	// Base
	if (!json_object_object_get_ex(j, "base", &tmp2)) {
		goto err;
	}

	// Max_dip
	if (!json_object_object_get_ex(tmp2, "max_dip", &tmp)) {
		goto err;
	}

	mod->max_dip = json_object_get_double(tmp);

	// Leg_tar_h
	if (!json_object_object_get_ex(tmp2, "leg_tar_h", &tmp)) {
		goto err;
	}

	mod->leg_tar_h = json_object_get_double(tmp);

	// T_rad
	if (!json_object_object_get_ex(tmp2, "t_rad", &tmp)) {
		goto err;
	}

	mod->t_rad = json_object_get_double(tmp);

	// Link_len
	if (!json_object_object_get_ex(tmp2, "link_len", &tmp)) {
		goto err;
	}

	mod->link_len = json_object_get_double(tmp);

	// Max_dip
	if (!json_object_object_get_ex(tmp2, "max_dip", &tmp)) {
		goto err;
	}

	mod->max_dip = json_object_get_double(tmp);

	// Leg_tar_h
	if (!json_object_object_get_ex(tmp2, "leg_tar_h", &tmp)) {
		goto err;
	}

	mod->leg_tar_h = json_object_get_double(tmp);

	// T_rad
	if (!json_object_object_get_ex(tmp2, "t_rad", &tmp)) {
		goto err;
	}

	mod->t_rad = json_object_get_double(tmp);

	// CW
	if (!json_object_object_get_ex(tmp2, "cw", &arr)) {
		goto err;
	}

	for (int i = 0; i < 4; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		mod->cw[i] = json_object_get_int(tmp);
	}

	// Link_len
	if (!json_object_object_get_ex(tmp2, "link_len", &tmp)) {
		goto err;
	}

	mod->link_len = json_object_get_double(tmp);

	// Legs
	if (!json_object_object_get_ex(j, "legs", &arr)) {
		goto err;
	}

	for (int i = 0; i < N_LEGS; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		l = leg_from_json(tmp);
		if (!l) {
			goto err;
		}

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
		goto err;
	}

	// Free previous model
	free_model(g_model);
	g_model = mod;
	return 0;

err:
	json_object_put(j);
	free_model(g_model);
	return 1;
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

