/**
 * @file balance.c``
 * @brief Some math for blancing
 * @author Linas Perkauskas
 * @date 2023-02-26
 */

#include <stdlib.h>
#include <math.h>

#include <model.h>
#include <log.h>
#include <mth.h>
#include <matrix.h>
#include <balance.h>
#include <pidc.h>

enum balalance_attrs {
	BA_HEIGHt,
	BA_LEAN,
	BA_IMP_INV,
	BA_TOUCH,
	BA_DROP,
	BA_MAX
};

void get_lean_diff(mat_t *ld)
{
	float fw_arr[3] = { 0.0, 0.0, -1.0 };
	mat_t *force_r_mat, *leg_pos_mod, *forward;
	sens_t *s = g_model->sens;
	leg_t *l;

	forward = mat_from_array(1, 3, fw_arr);
	force_r_mat = mat_rot_from_2vec(forward, s->bf_vec);

	for (int i = 0; i < N_LEGS; i++) {
		l = g_model->legs[i];
		if (s->damp->arr[i] < g_model->soft_hit_thr) {
			ld->arr[i] = s->avg_leg_h = l->pos->arr[2];
		} else {
			leg_pos_mod = mat_dot(l->pos, force_r_mat);
			mat_sub(leg_pos_mod, l->pos);
			ld->arr[i] = leg_pos_mod->arr[2];
		}
	}

	mat_free(forward);
}

void get_imp_diff(mat_t *id)
{
	float x_min, x_max, x_diff, x_lever, y_min, y_max, y_diff, y_lever, *pos;
	sens_t *s = g_model->sens;

	x_min = s->tf_pos->arr[0];
	x_max = s->tf_pos->arr[0];
	y_min = s->tf_pos->arr[1];
	y_max = s->tf_pos->arr[1];

	for (int i = 0; i < N_LEGS; i++) {
		pos = g_model->legs[i]->pos->arr;
		if (pos[0] > x_max)
			x_max = pos[0];
		if (pos[0] < x_min)
			x_min = pos[0];
		if (pos[1] > y_max)
			y_max = pos[1];
		if (pos[1] < y_min)
			y_min = pos[1];
	}

	x_diff = x_max - x_min;
	y_diff = y_max - y_min;

	for (int i = 0; i < N_LEGS; i++) {
		pos = g_model->legs[i]->pos->arr;
		x_lever = fabsf(pos[0] - s->tf_pos->arr[0]) / x_diff;
		y_lever = fabsf(pos[1] - s->tf_pos->arr[1]) / y_diff;
		id->arr[i] = x_lever * y_lever;
	}
}

static void get_bal_base(mat_t *res, float cof)
{
	mat_t *bfo_mat, *height_mat, *leg_pos_mod;
	leg_t *l;
	sens_t *s = g_model->sens;

	res = mat_create(1, 4);
	bfo_mat = mat_create(4, 4);
	leg_pos_mod = mat_create(1, 4);
	height_mat = mat_identity(4);

	// making shift matrix in z axis
	mat_set(height_mat, 2, 3, g_model->leg_tar_h - s->avg_leg_h);

	for (int i = 0; i < N_LEGS; i++) {
		l = g_model->legs[i];
		if (s->damp->arr[i] < g_model->soft_hit_thr) {
			res->arr[i] = cof * (s->avg_leg_h - l->pos->arr[2]);
			continue;
		}

		mat_fill(leg_pos_mod, 1.0);
		mat_map_mat(leg_pos_mod, l->pos, 0, 0);

		leg_pos_mod = mat_dot(bfo_mat, leg_pos_mod);
		mat_transpose(leg_pos_mod);
		leg_pos_mod = mat_dot(height_mat, leg_pos_mod);

		res->arr[i] = (leg_pos_mod->arr[2] - l->pos->arr[2]) * cof;
	}

	mat_free(bfo_mat);
	mat_free(height_mat);
	mat_free(leg_pos_mod);
}

void get_touch_diff(mat_t *dd, mat_t *td)
{
	float adj;
	bool s_hits;
	sens_t *s = g_model->sens;
	const float ADJUST_SENS_COF = 0.05;

	mat_fill(dd, 0.0);
	mat_fill(td, 0.0);

	for (int i = 0; i < N_LEGS; i++) {
		s_hits = s->damp->arr[i] / g_model->t_rad > g_model->soft_hit_thr;
		adj = s->damp->arr[i] - g_model->t_rad;

		if (s_hits && fabsf(g_model->t_rad) * ADJUST_SENS_COF < fabsf(adj)) {
			td->arr[i] = adj;
		} else if (!s_hits) {
			dd->arr[i] = -g_model->t_rad;
		}
	}
}

int calc_balance()
{
	const float TOUCH_COF = 0.2;
	const float BASE_PART_COF = 1.0;
	sens_t *s = g_model->sens;

	mat_t *base_part, *drop_diff, *touch_diff, *lean_diff, *height_diff,
		*imp_diff;

	base_part = mat_create(1, N_LEGS);
	drop_diff = mat_create(1, N_LEGS);
	touch_diff = mat_create(1, N_LEGS);
	lean_diff = mat_create(1, N_LEGS);
	height_diff = mat_create(1, N_LEGS);
	imp_diff = mat_create(1, N_LEGS);

	get_bal_base(base_part, BASE_PART_COF);
	get_touch_diff(drop_diff, touch_diff);
	get_lean_diff(lean_diff);
	get_imp_diff(imp_diff);
	mat_fill(height_diff, g_model->leg_tar_h - s->avg_leg_h);

	mat_fill(s->balance, 0.0);

	for (int i = 0; i < N_LEGS; i++) {
		s->balance->arr[i] =
			pidc_eval(&g_model->legs[i]->balance_pid, 0.0, base_part->arr[i]);
	}

	mat_add(drop_diff, touch_diff);
	mat_add_scal(drop_diff, TOUCH_COF);
	mat_add(drop_diff, s->balance);

	for (int i = 0; i < N_LEGS; i++) {
		if (imp_diff->arr[i] > 0.33) {
			s->balance->arr[i] =
				pidc_eval(&g_model->legs[i]->touch_pid, s->balance->arr[i],
						  s->balance->arr[i]);

		} else {
			s->balance->arr[i] =
				pidc_eval(&g_model->legs[i]->touch_pid, s->balance->arr[i],
						  drop_diff->arr[i]);
		}
	}

	mat_free(base_part);
	mat_free(drop_diff);
	mat_free(touch_diff);
	mat_free(lean_diff);
	mat_free(height_diff);
	mat_free(imp_diff);

	return 0;
}

float get_walk_height(float step_dist, float abs_adj_h)
{
	float aah_part, sd_part, val, res;

	aah_part = (-0.00088512 / (abs_adj_h + 0.0081818)) - 0.091818;
	sd_part = (-0.018857 / (step_dist + 0.17143)) - 0.09;

	val = MAX(aah_part, sd_part);
	res = bound_data(val, g_model->min_walk_h, g_model->max_walk_h);

	return res;
}
