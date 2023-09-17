
/**
 * @file model.c
 * @brief Implementation of two main datastructures that moded consists of (quad/model and leg).
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <balance.h>
#include <gsl/gsl_vector_double.h>
#include <json-c/json_types.h>
#include <plan.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <gsl/gsl_block_double.h>
#include <gsl/gsl_matrix_double.h>

#include <json-c/json_object.h>
#include <json-c/json_tokener.h>
#include <json-c/json.h>

#include "log.h"
#include "mth.h"
#include "model.h"
#include "stance.h"
#include "json_helper.h"

model_t *g_model = NULL;

/**
 * @brief Frees `sens_t` struct.
 * @return nothing
 */
static void sens_free(sens_t *s)
{
	if (s) {
		gsl_block_free(s->balance);
		gsl_block_free(s->touch_f);
		gsl_block_free(s->damp);
		gsl_vector_free(s->bf_vec);
		gsl_matrix_free(s->bfo_mat);
		gsl_vector_free(s->tf_pos);
		/*mat_free(s->s_center);*/
		/*mat_free(s->to_s_closest);*/
		free(s);
	}
}

/**
 * @brief Frees `leg_t` struct.
 * @return nothing
 */
static void leg_free(leg_t *l)
{
	if (l) {
		gsl_vector_free(l->pos);
		gsl_vector_free(l->def_pos);
		gsl_vector_free(l->base_off);
		gsl_vector_free(l->dir);
		gsl_vector_free(l->joint_lims);
		gsl_vector_free(l->angles);
		plan_free(&l->plan);
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

	s->balance = block_calloc(4);
	s->bfo_mat = matrix_calloc(3, 3);
	s->touch_f = block_calloc(4);
	s->damp = block_calloc(4);
	s->bf_vec = vector_calloc(3);
	s->tf_pos = vector_calloc(3);

	return s;
}

static void set_additional_sens()
{
	double sum = 0.0;
	size_t cnt = 0;
	leg_t *l;
	sens_t *s = g_model->sens;

	/* average balanced leg height*/
	for (size_t i = 0; i < N_LEGS; ++i) {
		l = g_model->legs[i];
		if (l->bal) {
			sum += gsl_vector_get(l->pos, 2);
			cnt++;
		}
	}

	if (cnt) {
		s->avg_leg_h = sum / cnt;
	} else {
		s->avg_leg_h = g_model->max_dip;
	}

	/* absolute std of balanced leg heights*/
	sum = 0.0;
	cnt = 0;
	for (size_t i = 0; i < N_LEGS; ++i) {
		l = g_model->legs[i];
		if (l->bal) {
			sum += fabs(gsl_vector_get(l->pos, 2) - s->avg_leg_h);
			cnt++;
		}
	}

	if (cnt) {
		s->abs_std_leg_h = sum / cnt;
	} else {
		s->abs_std_leg_h = 0.0;
	}
}

int set_sens_from_json(struct json_object *js)
{
	double tf[4] = { 0 }, d[4] = { 0 }, bv[3] = { 0 }, bm[9] = { 0 },
		   tfp[3] = { 0 };
	struct json_object *rsp, *tmp, *tmp2, *arr;
	int ret = 0;
	sens_t *s;

	if (!js || !g_model) {
		return 1;
	}

	if (!g_model->sens) {
		g_model->sens = alloc_sens();
		if (!g_model->sens) {
			return 1;
		}
	}

	s = g_model->sens;

	// Response
	if (!json_object_object_get_ex(js, "rsp", &rsp)) {
		goto err;
	}

	/*
	// Avg_leg_h
	if (!json_object_object_get_ex(rsp, "avg_leg_h", &tmp)) {
		goto err;
	}

	s->avg_leg_h = json_object_get_double(tmp);

	// Abs_std_leg_h
	if (!json_object_object_get_ex(rsp, "abs_std_leg_h", &tmp)) {
		goto err;
	}

	s->abs_std_leg_h = json_object_get_double(tmp);
	*/

	// Touch_force
	if (!json_object_object_get_ex(rsp, "touch_force", &arr)) {
		goto err;
	}

	for (int i = 0; i < 4; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		tf[i] = json_object_get_double(tmp);
	}

	// Damp
	if (!json_object_object_get_ex(rsp, "damp", &arr)) {
		goto err;
	}

	for (int i = 0; i < 4; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		d[i] = json_object_get_double(tmp);
	}

	// Bf_vvec
	if (!json_object_object_get_ex(rsp, "bf_vec", &arr)) {
		goto err;
	}

	for (int i = 0; i < 3; i++) {
		if (!(tmp = json_object_array_get_idx(arr, i)))
			goto err;

		bv[i] = json_object_get_double(tmp);
	}

	// Bfo_mat
	if (!json_object_object_get_ex(rsp, "bfo_mat", &arr)) {
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
	if (!json_object_object_get_ex(rsp, "t_force", &tmp2)) {
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

	ret |= block_update_array(s->touch_f, 4, tf);
	ret |= block_update_array(s->damp, 4, d);
	ret |= vector_update_array(s->bf_vec, 3, bv);
	ret |= matrix_update_array(s->bfo_mat, 3, 3, bm);
	ret |= vector_update_array(s->tf_pos, 3, tfp);

	if (ret) {
		goto err;
	}

	set_additional_sens();

	return 0;

err:
	ERR(ERR_PARSE_FAIL);
	sens_free(s);
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
	double p[3], dp[3], bo[3], d[3], jl[6];
	struct json_object *tmp, *arr;
	int res = 0;

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

	/*pidc_set(&l->balance_pid, 0.06, 0.0, 0.0005, 1.0 / 240);*/
	/*pidc_set(&l->touch_pid, 0.35, 0.0, 0.0, 1.0 / 240);*/

	pidc_set(&l->balance_pid, 0.27, -0.00, 0.0005, 1.0 / 48);
	pidc_set(&l->touch_pid, 0.45, 0.0, 0.0, 1.0 / 48);

	l->bal = true;
	l->pos = vector_from_array(3, p);
	l->def_pos = vector_from_array(3, dp);
	l->base_off = vector_from_array(3, bo);
	l->dir = vector_from_array(3, d);
	l->joint_lims = vector_from_array(6, jl);
	l->angles = vector_calloc(3);
	res |= plan_new(&l->plan);

	if (res) {
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
	double gama, leg_len, max_leg_len, min_leg_len, alpha, theta, phi, beta;
	gsl_vector *target, *lims, *dir;

	target = vector_sub_n(l->pos, l->base_off);
	lims = l->joint_lims;
	dir = l->dir;

	gama = atan(-gsl_vector_get(target, 1) / gsl_vector_get(target, 2));
	if (gsl_vector_get(lims, 0) > gama) {
		gama = gsl_vector_get(lims, 0);
		gsl_vector_set(target, 1, -gsl_vector_get(target, 2) * tanf(gama));
	} else if (gsl_vector_get(lims, 1) < gama) {
		gama = gsl_vector_get(lims, 1);
		gsl_vector_set(target, 1, -gsl_vector_get(target, 2) * tanf(gama));
	}
	// Substract shoulder position
	gsl_vector_set(
		target, 1,
		gsl_vector_get(target, 1) -
			(0.03 * sin(gama) + gsl_vector_get(dir, 2) * 0.01 * cos(gama)));
	gsl_vector_set(
		target, 2,
		gsl_vector_get(target, 2) -
			(-0.03 * cos(gama) + gsl_vector_get(dir, 2) * 0.01 * sin(gama)));

	leg_len = vector_length(target);
	max_leg_len =
		g_model->link_len * sqrt(5 - 4 * cos(fabs(gsl_vector_get(lims, 5))));
	min_leg_len =
		g_model->link_len * sqrt(5 - 4 * cos(fabs(gsl_vector_get(lims, 4))));

	if (max_leg_len > leg_len && leg_len > min_leg_len) { // middle
		double e = acos(1.25 - ((leg_len * leg_len) /
								(4 * g_model->link_len * g_model->link_len)));
		alpha = gsl_vector_get(dir, 0) * (M_PI - e);
	} else if (leg_len >= max_leg_len) { // outer
		alpha = gsl_vector_get(lims, 4);
	} else { // inner
		alpha = gsl_vector_get(lims, 5);
	}

	phi = asin(gsl_vector_get(target, 0) / leg_len);
	theta =
		asin(bound_data((g_model->link_len * sin(alpha)) / leg_len, -1.0, 1.0));

	if (gsl_vector_get(lims, 2) <= -phi - theta &&
		-phi - theta <= gsl_vector_get(lims, 3)) {
		beta = -phi + -theta; // VII
	} else if (gsl_vector_get(lims, 2) > -phi - theta) {
		beta = gsl_vector_get(lims, 2); // VIII
	} else {
		beta = gsl_vector_get(lims, 3);
	}

	gsl_vector_set(l->angles, 0, alpha);
	gsl_vector_set(l->angles, 1, beta);
	gsl_vector_set(l->angles, 2, gama);

	gsl_vector_free(target);
}

static void model_get_angles()
{
	for (int i = 0; i < N_LEGS; i++) {
		leg_t *l = g_model->legs[i];
		leg_get_angles(l);
		gsl_matrix_set_row(g_model->angles, i, l->angles);
	}
}

void model_step()
{
	leg_t *l;

	calc_balance();

resched:
	if (g_model->move && g_model->move->need_sched) {
		get_movement(NULL, NULL, true);
	}

	for (int i = 0; i < N_LEGS; ++i) {
		l = g_model->legs[i];
		plan_step(&l->plan);
	}

	if (g_model->move && g_model->move->need_sched) {
		goto resched;
	}

	model_get_angles();
}

int model_from_json(struct json_object *j)
{
	model_t *mod;
	leg_t *l;
	struct json_object *rsp, *tmp, *tmp2, *arr;

	if (!j) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	mod = calloc(1, sizeof(model_t));
	if (!mod) {
		FATAL(ERR_MALLOC_FAIL);
	}

	DBG("MODEL:%s\n", json_object_get_string(j));

	// Response
	if (!json_object_object_get_ex(j, "rsp", &rsp)) {
		goto err;
	}

	// Base
	if (!json_object_object_get_ex(rsp, "base", &tmp2)) {
		goto err;
	}

	// Min_walk_h
	if (!json_object_object_get_ex(tmp2, "min_walk_h", &tmp)) {
		goto err;
	}

	mod->min_walk_h = json_object_get_double(tmp);

	// Max_walk_h
	if (!json_object_object_get_ex(tmp2, "max_walk_h", &tmp)) {
		goto err;
	}

	mod->max_walk_h = json_object_get_double(tmp);

	// Min_dip
	if (!json_object_object_get_ex(tmp2, "min_dip", &tmp)) {
		goto err;
	}

	mod->min_dip = json_object_get_double(tmp);

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

	// Soft hit treshold
	if (!json_object_object_get_ex(tmp2, "soft_hit_thr", &tmp)) {
		goto err;
	}

	mod->soft_hit_thr = json_object_get_double(tmp);

	// mass
	if (!json_object_object_get_ex(tmp2, "mass", &tmp)) {
		goto err;
	}

	mod->mass = json_object_get_double(tmp);

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
	if (!json_object_object_get_ex(rsp, "legs", &arr)) {
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

	// Legs excentricity
	mod->leg_exc = (gsl_vector_get(mod->legs[3]->def_pos, 0) -
					gsl_vector_get(mod->legs[1]->def_pos, 0)) /
				   (gsl_vector_get(mod->legs[0]->def_pos, 1) -
					gsl_vector_get(mod->legs[1]->def_pos, 1));

	// Setup "next" legs
	for (int i = 0; i < N_LEGS; i++) {
		mod->legs[i]->next = mod->legs[(i + 1) % N_LEGS];
		mod->cw_legs[i]->cw_next = mod->cw_legs[(i + 1) % N_LEGS];
	}

	mod->angles = matrix_calloc(4, 3);

	// Free previous model
	model_free(g_model);
	g_model = mod;
	return 0;

err:
	json_object_put(j);
	model_free(g_model);
	return 1;
}

void model_free(model_t *mod)
{
	if (mod) {
		for (int i = 0; i < N_LEGS; i++) {
			leg_free(mod->legs[i]);
		}

		movement_free(mod->move);
		gsl_matrix_free(mod->angles);
		sens_free(g_model->sens);
		free(mod);
	}
}

