/**
 * @file balance.c
 * @brief Some math for blancing
 * @author Linas Perkauskas
 * @date 2023-02-26
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_block_double.h>
#include <gsl/gsl_matrix_double.h>

#include <log.h>
#include <model.h>
#include <mth.h>
#include <balance.h>
#include <pidc.h>

#define G_ACC 10.0

void get_imp_diff(gsl_block *id)
{
	double x_min, x_max, x_diff, y_min, y_max, y_diff, x, y;
	sens_t *s = g_model->sens;

	x_min = gsl_vector_get(s->tf_pos, 0);
	x_max = x_min;
	y_min = gsl_vector_get(s->tf_pos, 1);
	y_max = x_min;

	for (int i = 0; i < N_LEGS; i++) {
		if (!g_model->legs[i]->bal)
			continue;

		x = gsl_vector_get(g_model->legs[i]->pos, 0);
		y = gsl_vector_get(g_model->legs[i]->pos, 1);

		if (x > x_max)
			x_max = x;
		if (x < x_min)
			x_min = x;
		if (y > y_max)
			y_max = y;
		if (y < y_min)
			y_min = y;
	}

	x_diff = x_max - x_min;
	y_diff = y_max - y_min;

	for (int i = 0; i < N_LEGS; i++) {
		double x_lever, y_lever;
		x_lever = fabs(gsl_vector_get(g_model->legs[i]->pos, 0) -
					   gsl_vector_get(s->tf_pos, 0)) /
				  x_diff;
		y_lever = fabs(gsl_vector_get(g_model->legs[i]->pos, 1) -
					   gsl_vector_get(s->tf_pos, 1)) /
				  y_diff;
		id->data[i] = x_lever * y_lever;
	}
}

static void get_shock_diff(gsl_block *sd)
{
	const double SHOCK_SENS_COF = 3.5;
	double weight = g_model->mass * G_ACC;
	sens_t *s = g_model->sens;

	for (int i = 0; i < N_LEGS; i++) {
		if (s->touch_f->data[i] > weight) {
			sd->data[i] = SHOCK_SENS_COF * g_model->t_rad *
						  ((s->touch_f->data[i] / weight) - 1);
		} else {
			sd->data[i] = 0;
		}
	}
}

static void get_bal_base(gsl_block *res, double cof)
{
	double gh_diff, lh_diff, lh_mod;
	gsl_matrix *bfo_mat, *height_mat;
	gsl_vector *leg_pos_mod;
	sens_t *s = g_model->sens;

	bfo_mat = matrix_calloc(4, 4);
	gsl_matrix_set_identity(bfo_mat);
	matrix_copy_to_origin(bfo_mat, s->bfo_mat);

	leg_pos_mod = vector_calloc(4);
	height_mat = matrix_calloc(4, 4);
	gsl_matrix_set_identity(height_mat);

	// making shift matrix in z axis
	gsl_matrix_set(height_mat, 2, 3, (g_model->leg_tar_h - s->avg_leg_h));
	gh_diff = g_model->leg_tar_h - s->avg_leg_h;

	for (int i = 0; i < N_LEGS; i++) {
		leg_t *l = g_model->legs[i];
		if (s->damp->data[i] < g_model->soft_hit_thr) {
			res->data[i] = cof * (s->avg_leg_h - gsl_vector_get(l->pos, 2));
			continue;
		}

		vector_copy_to_origin(leg_pos_mod, l->pos);
		gsl_vector_set(leg_pos_mod, 3, 1.0);

		gsl_vector *lpm_tmp = matrix_vector_dot(bfo_mat, leg_pos_mod);
		lh_mod = gsl_vector_get(lpm_tmp, 2);
		gsl_vector_free(lpm_tmp);
		lh_mod += gh_diff;
		lh_diff = (lh_mod - gsl_vector_get(l->pos, 2)) * cof;

		res->data[i] = lh_diff;
	}

	gsl_matrix_free(bfo_mat);
	gsl_matrix_free(height_mat);
	gsl_vector_free(leg_pos_mod);
}

static void get_touch_diff(gsl_block *dd, gsl_block *td)
{
	sens_t *s = g_model->sens;
	const double ADJUST_SENS_COF = 0.05;

	for (int i = 0; i < N_LEGS; i++) {
		bool s_hits = s->damp->data[i] > g_model->soft_hit_thr;
		double adj = g_model->t_rad * (1 - s->damp->data[i]);

		if (s_hits && ADJUST_SENS_COF < (1 - s->damp->data[i])) {
			td->data[i] = -adj;
		} else if (!s_hits) {
			dd->data[i] = -g_model->t_rad;
		}
	}
}

int calc_balance()
{
	const double TOUCH_COF = 0.2;
	const double BASE_PART_COF = 1.0;
	sens_t *s = g_model->sens;

	gsl_block *imp_diff, *base_part, *touch_diff, *drop_diff, *shock_diff;

	base_part = block_calloc(N_LEGS);
	drop_diff = block_calloc(N_LEGS);
	touch_diff = block_calloc(N_LEGS);
	shock_diff = block_calloc(N_LEGS);
	imp_diff = block_calloc(N_LEGS);

	get_bal_base(base_part, BASE_PART_COF);
	get_touch_diff(drop_diff, touch_diff);
	get_imp_diff(imp_diff);
	get_shock_diff(shock_diff);

	memset(s->balance->data, 0, s->balance->size * sizeof(double));

	for (int i = 0; i < N_LEGS; i++) {
		s->balance->data[i] =
			pidc_eval(&g_model->legs[i]->balance_pid, 0.0,
					  base_part->data[i] + shock_diff->data[i]);
	}

	for (int i = 0; i < N_LEGS; i++) {
		double *cur = s->balance->data + i;
		if (imp_diff->data[i] > 0.33) {
			*cur += pidc_eval(&g_model->legs[i]->touch_pid, *cur, *cur);
		} else {
			*cur += pidc_eval(&g_model->legs[i]->touch_pid, *cur,
							  *cur + TOUCH_COF * touch_diff->data[i] +
								  drop_diff->data[i]);
		}
	}

	gsl_block_free(base_part);
	gsl_block_free(drop_diff);
	gsl_block_free(touch_diff);
	gsl_block_free(imp_diff);
	gsl_block_free(shock_diff);

	return 0;
}

double get_walk_height(double step_dist, double abs_adj_h)
{
	double aah_part, sd_part, val, res;

	aah_part = (-0.00088512 / (abs_adj_h + 0.0081818)) - 0.091818;
	sd_part = (-0.018857 / (step_dist + 0.17143)) - 0.09;

	val = MAX(aah_part, sd_part);
	res = bound_data(val, g_model->min_walk_h, g_model->max_walk_h);

	return res;
}
