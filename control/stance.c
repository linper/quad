/**
 * @file stance.c
 * @brief Globl movement plan
 * @author Linas Perkauskas
 * @date 2023-05-05
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <mth.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include <json-c/json_object.h>
#include <json-c/json_types.h>

#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_sort_vector_double.h>
#include <gsl/gsl_block_double.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>

#include <log.h>
#include <model.h>
#include <stance.h>

#define LEG_ELLIPSE_A 0.21
#define LEG_ELLIPSE_B 0.18

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
	gsl_matrix *barrier; // (2 x 2) possible barrier to cross to reach this stance
	gsl_matrix *bpts; // (3 x 2)
	gsl_matrix *mod_pts; // (4 x 2)
	gsl_vector *mod_com; // (2)
	gsl_vector *mod_bcom; // (2)
} stance_t;

typedef struct movement {
	struct stance *st;
	size_t n;
	double loss;
	gsl_matrix *coms; // (n, 2)
	gsl_matrix *bcoms; // (n, 2)
	gsl_matrix *path; // (n, 2)
	gsl_spline *x_spl;
	gsl_spline *y_spl;
	gsl_interp_accel *acc;
} movement_t;

static stance_t *stance_optimize(stance_t *self, size_t n);
static void stance_plot(stance_t *self);
static size_t plot_id = 0;

static void stance_free(stance_t *self, bool all)
{
	if (!self) {
		return;
	}
	stance_t *next = self->next;
	gsl_matrix_free(self->bpts);
	gsl_matrix_free(self->pts);
	gsl_matrix_free(self->orig_pts);
	gsl_matrix_free(self->def_pts);
	gsl_vector_free(self->com);
	gsl_vector_free(self->dir);
	gsl_vector_free(self->bcom);
	gsl_vector_free(self->com_off);
	gsl_matrix_free(self->mod_pts);
	gsl_vector_free(self->mod_com);
	gsl_vector_free(self->mod_bcom);
	free(self);

	if (all && next) {
		stance_free(next, true);
	}
}

static void movement_free(movement_t *self)
{
	if (!self) {
		return;
	}

	gsl_matrix_free(self->bcoms);
	gsl_matrix_free(self->coms);
	gsl_matrix_free(self->path);
	gsl_spline_free(self->x_spl);
	gsl_spline_free(self->y_spl);
	gsl_interp_accel_free(self->acc);
	stance_free(self->st, true);
	free(self);
}

static stance_t *stance_create_from_model(gsl_vector *dir, gsl_vector *com)
{
	stance_t *st = calloc(1, sizeof(stance_t));
	if (!st) {
		FATAL(ERR_MALLOC_FAIL);
	}

	/*st->bpts = matrix_calloc(N_LEGS - 1, 2);*/
	st->pts = matrix_calloc(N_LEGS, 2);
	st->mod_pts = matrix_calloc(N_LEGS, 2);
	st->def_pts = matrix_calloc(N_LEGS, 2);
	st->bcom = vector_calloc(2);
	st->mod_bcom = vector_calloc(2);
	st->com_off = vector_calloc(2);
	st->com = vector_calloc(2);
	st->mod_com = vector_calloc(2);

	for (size_t i = 0; i < N_LEGS; i++) {
		for (size_t j = 0; j < 2; j++) {
			gsl_matrix_set(st->pts, i, j, g_model->cw_legs[i]->pos->data[j]);
			gsl_matrix_set(st->def_pts, i, j,
						   g_model->cw_legs[i]->def_pos->data[j]);
		}
	}

	vector_copy_to(st->com, com, 0, 0, 2);
	st->orig_pts = matrix_clone(st->pts);
	st->dir = vector_clone(dir);

	st->pidx = 0;
	st->min_loss = -1;

	return st;
}

static stance_t *stance_create(gsl_matrix *pos, gsl_matrix *def_pos,
							   gsl_vector *dir, gsl_vector *com)
{
	stance_t *st = calloc(1, sizeof(stance_t));
	if (!st) {
		FATAL(ERR_MALLOC_FAIL);
	}

	/*st->bpts = matrix_calloc(N_LEGS - 1, 2);*/
	st->pts = matrix_clone(pos);
	st->mod_pts = matrix_clone(pos);
	st->orig_pts = matrix_clone(pos);
	st->def_pts = matrix_clone(def_pos);
	st->com = vector_clone(com);
	st->mod_com = vector_clone(com);
	st->dir = vector_clone(dir);
	st->bcom = vector_calloc(2);
	st->mod_bcom = vector_calloc(2);
	st->com_off = vector_calloc(2);

	st->pidx = 0;
	st->min_loss = -1;

	return st;
}

static stance_t *stance_append_create(stance_t *s)
{
	// Get last stance
	while (s->next) {
		s = s->next;
	}

	stance_t *st = stance_create(s->pts, s->def_pts, s->dir, s->com);
	s->next = st;
	st->prev = s;

	return st;
}

static movement_t *create_movement(size_t n, gsl_vector *dir, gsl_vector *com)
{
	stance_t *ss;

	movement_t *mv = calloc(1, sizeof(movement_t));
	if (!mv) {
		FATAL(ERR_MALLOC_FAIL);
	}

	mv->acc = gsl_interp_accel_alloc();
	if (!mv->acc) {
		FATAL(ERR_MALLOC_FAIL);
	}

	mv->st = stance_create_from_model(dir, com);

	ss = mv->st;
	mv->n = n + 1;
	stance_optimize(mv->st, n);
	while (ss->next) {
		(void)stance_plot;
		/*stance_plot(ss->next);*/
		ss = ss->next;
	}

	return mv;
}

static void calc_bcom(gsl_matrix *pts, int pidx, gsl_vector *ans)
{
	gsl_matrix *bcom_pts = matrix_del_row_n(pts, pidx);
	centroid_of_polygon(ans, bcom_pts);
	gsl_matrix_free(bcom_pts);
}

static gsl_matrix *guess_start_pos(gsl_vector_view off, gsl_vector *d, size_t n)
{
	double k, b;
	bool inter;

	gsl_vector *p1, *p2;
	gsl_matrix *full_pts, *pts;

	p1 = vector_calloc(2);
	p2 = vector_calloc(2);
	pts = matrix_calloc(n, 2);

	line_cof(0.0, 0.0, gsl_vector_get(d, 0), gsl_vector_get(d, 0), &k, &b);
	inter =
		ellipse_line_intersect(LEG_ELLIPSE_A, LEG_ELLIPSE_B, k, b,
							   gsl_vector_ptr(p1, 0), gsl_vector_ptr(p1, 1),
							   gsl_vector_ptr(p2, 0), gsl_vector_ptr(p2, 1));
	if (!inter) {
		gsl_vector_free(p1);
		gsl_vector_free(p2);

		return NULL;
	}

	gsl_vector_add(p1, &off.vector);
	gsl_vector_add(p2, &off.vector);
	full_pts = matrix_linspace(p1, p2, n + 2);
	matrix_copy_to(pts, full_pts, 0, 0, 1, 0, n, 2);

	gsl_matrix_free(full_pts);
	gsl_vector_free(p1);
	gsl_vector_free(p2);

	return pts;
}

static void eval_st_support_shape(stance_t *st, double *val)
{
	double pts[N_LEGS][2] = { 0 };
	double pair1[2], pair2[2];
	double surface, perimeter = 0.0;
	gsl_vector_view p1, p2;

	for (size_t i = 0; i < N_LEGS; i++) {
		// clockwise points adjusted by default leg excentricity
		p1 = gsl_matrix_row(st->pts, i);
		p2 = gsl_matrix_row(st->pts, (i + 1) / N_LEGS);
		pts[i][0] = gsl_vector_get(&p1.vector, 0) * g_model->leg_exc;
		pts[i][1] = gsl_vector_get(&p1.vector, 1) * 1.0;

		perimeter += vector_dist(&p1.vector, &p2.vector);
	}

	pair1[0] = area(pts[0], pts[1], pts[2]);
	pair1[1] = area(pts[2], pts[3], pts[0]);

	pair2[0] = area(pts[1], pts[2], pts[3]);
	pair2[1] = area(pts[3], pts[0], pts[1]);

	surface = (pair1[0] + pair1[1] + pair2[0] + pair2[1]) / 2;

	if (val) {
		int max_ps_coaf = 16;
		*val = 1 - ((max_ps_coaf * surface) / (perimeter * perimeter));
	}
}

static void eval_st_avg_move_dir(stance_t *st, double *val)
{
	gsl_vector *avg_pos_diff, *apd_vec;

	double R = MAX(LEG_ELLIPSE_A, LEG_ELLIPSE_B);

	avg_pos_diff = matrix_sum_axis(st->pts, 0);
	apd_vec = vector_project(avg_pos_diff, st->dir);

	double sign = XOR(apd_vec->data[0] > 0, st->dir->data[0] > 0) ? -1.0 : 1.0;

	double ap_part = 1 - (R + (sign * vector_length(apd_vec) / (2 * R)));

	gsl_vector_free(avg_pos_diff);
	gsl_vector_free(apd_vec);

	if (val)
		*val = ap_part;
}

static void eval_st_sequence(stance_t *st, double *avg_val, double *edge_val)
{
	double edge_retr_vals[N_LEGS] = { 0 };
	double max_d = 0.0, ins_val, lox, loy, k, b, edge_part, avg_part;
	gsl_matrix *off;
	gsl_vector_view loff;
	gsl_vector *pf, *p1, *p2, *dl_vec;
	vbuf_t *dists_left;

	dists_left = vbuf_create(8);
	off = matrix_sub_n(st->pts, st->def_pts);
	p1 = vector_calloc(2);
	p2 = vector_calloc(2);

	for (size_t i = 0; i < st->pts->size1; i++) {
		// leg offset from default position
		lox = gsl_matrix_get(off, i, 0);
		loy = gsl_matrix_get(off, i, 1);
		loff = gsl_matrix_row(off, i);
		// checking if leg is inside area restricted by ellipse
		bool inside = ellipse_point_inside(LEG_ELLIPSE_A, LEG_ELLIPSE_B, lox,
										   loy, &ins_val);
		if (!inside) {
			// unlikely
			edge_retr_vals[i] = 1.0;
			vbuf_push(dists_left, 0.0);
			continue;
		}

		// gets line coafitients of leg movenet according to its default position
		line_cof(lox, loy, lox + gsl_vector_get(st->dir, 0),
				 loy + gsl_vector_get(st->dir, 1), &k, &b);
		// gets points where leg will cross ellipse restricted area if direction is maintained
		ellipse_line_intersect2(LEG_ELLIPSE_A, LEG_ELLIPSE_B, k, b, p1, p2);

		// checking if perpendicular vectors pointing in same direction
		if (!XOR(p1->data[0] > p2->data[0], st->dir->data[0] > 0)) {
			pf = p1;
		} else {
			pf = p2;
		}

		// how much leg point is away from middle
		edge_retr_vals[i] = 1 - sqrt(1 - ins_val * ins_val);

		// getting distance till restricted area
		double dl = vector_dist(pf, &loff.vector);
		vbuf_push(dists_left, dl);

		// get maximum distance from p1 to p2 among legs
		double full_d = vector_dist(p1, p2);

		if (max_d < full_d) {
			max_d = full_d;
		}
	}

	gsl_matrix_free(off);
	gsl_vector_free(p1);
	gsl_vector_free(p2);

	edge_part = 0.0;
	for (size_t i = 0; i < N_LEGS; i++) {
		edge_part += edge_retr_vals[i];
	}

	if (dists_left->cnt <= 1) {
		avg_part = 1.0;
		goto end;
	}

	// adding outer values for widest p1/p2 pair
	vbuf_push(dists_left, 0.0);
	vbuf_push(dists_left, max_d);

	dl_vec = vbuf_convert_to_vec(dists_left);

	gsl_sort_vector(dl_vec);

	vector_increments(dl_vec);
	vector_increments(dl_vec);

	avg_part = 2 * vector_std(dl_vec) / max_d;

	gsl_vector_free(dl_vec);

end:
	if (edge_val)
		*edge_val = edge_part;
	if (avg_val)
		*avg_val = avg_part;
}

static void eval_st_min_prox(stance_t *st, double *val)
{
	double px, py, min_dist = DBL_MAX;

	px = gsl_matrix_get(st->pts, st->pidx, 0);
	py = gsl_matrix_get(st->pts, st->pidx, 1);

	for (size_t i = 0; i < N_LEGS; i++) {
		double x = gsl_matrix_get(st->orig_pts, i, 0);
		double y = gsl_matrix_get(st->orig_pts, i, 1);
		double dist = sqrt((px - x) * (px - x) + (py - y) * (py - y));

		if (dist < min_dist) {
			min_dist = dist;
		}
	}

	if (val) {
		const double k = -30.0, r = 0.006;
		*val = bound_data(k * min_dist - k * r + 1, 0.0, 1.0);
	}
}

static double stance_loss(stance_t *st)
{
	const double PROX_C = 0.1;
	const double AVG_C = 0.6;
	const double EDGE_C = 0.5;
	const double AP_DIR_C = 0.35;
	const double SUP_TRI_C = 0.15;

	double val, prox_part, avg_part, apd_part, edge_part, sup_shp_part;

	eval_st_min_prox(st, &prox_part);
	prox_part *= PROX_C;

	eval_st_sequence(st, &avg_part, &edge_part);
	avg_part *= AVG_C;
	edge_part *= EDGE_C;

	eval_st_avg_move_dir(st, &apd_part);
	apd_part *= AP_DIR_C;

	eval_st_support_shape(st, &sup_shp_part);
	sup_shp_part *= SUP_TRI_C;
	(void)sup_shp_part;

	/*val = prox_part + avg_part + edge_part + sup_shp_part + apd_part;*/
	val = prox_part + avg_part + edge_part + apd_part;
	/*val = prox_part + sup_shp_part;*/

	return val;
}
static int stance_grad(stance_t *st, gsl_matrix *spts, gsl_matrix *grad,
					   gsl_vector *losses)
{
	const double DELTA = 1e-5;

	gsl_matrix *pts_save;
	double var_loss;

	pts_save = matrix_clone(st->pts);
	/*grad = matrix_calloc(spts->size1, spts->size2);*/

	for (size_t i = 0; i < spts->size1; i++) {
		/*matrix_copy_to_origin(st->pts, st->pts);*/
		matrix_copy_to(st->pts, spts, st->pidx, 0, i, 0, 1, st->pts->size2);
		double base_loss = stance_loss(st);
		gsl_vector_set(losses, i, base_loss);
		for (size_t j = 0; j < st->pts->size2; j++) {
			*gsl_matrix_ptr(st->pts, st->pidx, j) += DELTA;

			var_loss = stance_loss(st);
			gsl_matrix_set(grad, i, j, (var_loss - base_loss) / DELTA);

			// restoring
			*gsl_matrix_ptr(st->pts, st->pidx, j) -= DELTA;
		}

		/*matrix_copy_to(grad, st->pts, i, 0, st->pidx, 0, 1, grad->size2);*/
		matrix_copy_to_origin(st->pts, pts_save);
	}

	/*matrix_copy_to_origin(st->pts, pts_save);*/

	return 0;
}

static int get_optimal_step(stance_t *st, gsl_vector *pt, double *loss)
{
	const double MAX_ST_GD_ITER = 25;
	const double EPSILON = 1e-3;
	const double N_START_PTS = 6;
	const double ALPHA = 0.01;

	gsl_matrix *spts, *grad;
	gsl_vector_view row, res_row;
	gsl_vector *losses, *prev_losses;

	row = gsl_matrix_row(st->def_pts, st->pidx);
	spts = guess_start_pos(row, st->dir, N_START_PTS);
	if (!spts) {
		return 0;
	}

	grad = matrix_calloc(N_START_PTS, 2);
	losses = vector_calloc(N_START_PTS);
	gsl_vector_set_all(losses, DBL_MAX);
	prev_losses = vector_clone(losses);

	for (size_t i = 0; i < MAX_ST_GD_ITER; i++) {
		stance_grad(st, spts, grad, losses);
		gsl_matrix_scale(grad, ALPHA);
		gsl_matrix_sub(spts, grad);

		gsl_vector_sub(prev_losses, losses); // diff losses
		if (EPSILON * ALPHA > gsl_vector_max(prev_losses)) {
			break;
		}

		vector_copy_to_origin(prev_losses, losses);
	}

	size_t min_idx = gsl_vector_min_index(losses);

	if (loss) {
		*loss = gsl_vector_get(losses, min_idx);
	}

	if (pt) {
		res_row = gsl_matrix_row(spts, min_idx);
		vector_copy_to_origin(pt, &res_row.vector);
	}

	return 0;
}

static stance_t *stance_optimize(stance_t *self, size_t n)
{
	double loss;
	size_t min_idx = 0;

	gsl_vector *min_pt, *pt;

	min_pt = vector_calloc(2);
	pt = vector_calloc(2);

	/*calc_bcom(self->pts, -1, self->bcom);*/
	/*centroid_of_polygon(self->bcom, self->pts);*/
	vector_copy_to_origin(self->bcom, self->com);

	for (size_t ni = 0; ni < n; ni++) {
		// Add
		self = stance_append_create(self);
		/*min_pts = matrix_clone(self->pts);*/
		double min_loss = DBL_MAX;

		for (size_t li = 0; li < N_LEGS; li++) {
			self->pidx = li;
			get_optimal_step(self, pt, &loss);
			if (loss < min_loss) {
				vector_copy_to_origin(min_pt, pt);
				min_loss = loss;
				min_idx = li;
			}
		}
		self->pidx = min_idx;
		self->min_loss = loss;
		gsl_matrix_set_row(self->pts, min_idx, min_pt);

		printf("point:");
		vector_print(pt);
		printf("loss:%.4g pidx:%zu\n", loss, min_idx);

		gsl_vector *com_off = vector_calloc(2);
		centroid_of_polygon(com_off, self->pts);
		calc_bcom(self->pts, self->pidx, self->bcom);

		matrix_sub_vec_rows(self->pts, com_off);
		gsl_vector_set_all(self->com, 0.0);
		gsl_vector_free(com_off);
	}

	gsl_vector_free(min_pt);
	gsl_vector_free(pt);

	return self;
}

static void eval_mv_com_diff(movement_t *mv, double *dist_val, double *std_val)
{
	double straight_dist, full_dist = 0, dist_res, std_res;
	gsl_vector_view v1, v2;

	gsl_vector *dists = vector_calloc(mv->path->size1 - 1);

	v1 = gsl_matrix_row(mv->path, 0);
	v2 = gsl_matrix_row(mv->path, mv->path->size1 - 1);
	straight_dist = vector_dist(&v1.vector, &v2.vector);

	for (size_t i = 0; i < mv->path->size1 - 1; i++) {
		v1 = gsl_matrix_row(mv->path, i);
		v2 = gsl_matrix_row(mv->path, i + 1);

		double d = vector_dist(&v1.vector, &v2.vector);
		dists->data[i] = d;
		full_dist += d;
	}

	dist_res = (full_dist / straight_dist) - 1.0;
	std_res = vector_std(dists);
	gsl_vector_free(dists);

	if (dist_val)
		*dist_val = dist_res;

	if (std_val)
		*std_val = std_res;
}

static void eval_mv_ins_trig(movement_t *mv, double *val)
{
	stance_t *s = mv->st;
	const double PREV_COF = 0.4;
	const double CUR_COF = 0.6;
	const double TRIG_SCALE = 0.9;
	double c, cof_sum = 0.0;
	size_t cnt = 0, i = 1;

	while (s && (s = s->next)) {
		gsl_vector_view v_cur = gsl_matrix_row(mv->path, i);
		is_inside_triangle(&v_cur.vector, s->bpts, TRIG_SCALE, &c);
		/*printf("cur tr cof:%.5g\n", c);*/
		cof_sum += c * CUR_COF;

		gsl_vector_view v_prev = gsl_matrix_row(mv->path, i - 1);
		is_inside_triangle(&v_prev.vector, s->bpts, TRIG_SCALE, &c);
		cof_sum += c * PREV_COF;
		/*printf("prev tr cof:%.5g\n", c);*/
		cnt++;
		i++;
	}

	cof_sum /= cnt;
	cof_sum -= 1.0;

	if (val)
		*val = cof_sum;
}

static double movement_loss(movement_t *mv)
{
	const double STRAIGHTNESS_C = 0.25;
	const double INSIDE_TRIG_C = 0.7;
	const double STD_DISTS_C = 1.5;

	double val, stdd_part = 0, len_part = 0, trig_part = 0;

	eval_mv_com_diff(mv, &len_part, &stdd_part);
	len_part *= STRAIGHTNESS_C;
	stdd_part *= STD_DISTS_C;

	eval_mv_ins_trig(mv, &trig_part);
	trig_part *= INSIDE_TRIG_C;

	val = len_part + stdd_part + trig_part;

	return val;
}

static int movement_grad(movement_t *mv, gsl_matrix *grad, double *loss)
{
	const double DELTA = 1e-5;

	double var_loss;

	*loss = movement_loss(mv);
	for (size_t i = 1; i < grad->size1 - 1; i++) {
		for (size_t j = 0; j < grad->size2; j++) {
			*gsl_matrix_ptr(mv->path, i, j) += DELTA;

			var_loss = movement_loss(mv);
			gsl_matrix_set(grad, i, j, (var_loss - *loss) / DELTA);

			// restoring
			*gsl_matrix_ptr(mv->path, i, j) -= DELTA;
		}
	}

	return 0;
}

static int movement_optimize(movement_t *mv)
{
	const double MAX_MV_GD_ITER = 500;
	const double ALPHA = 0.005;
	/*const double ALPHA = 0.05;*/
	const double EPSILON = 1e-3;

	gsl_matrix *grad;
	double loss = DBL_MAX, prev_loss = DBL_MAX;

	grad = matrix_calloc(mv->n, 2);

	for (size_t iter = 0; iter < MAX_MV_GD_ITER; iter++) {
		movement_grad(mv, grad, &loss);
		printf("mov iter:%zu loss:%.6g\n", iter, loss);

		if (EPSILON * ALPHA > prev_loss - loss) {
			break;
		}

		gsl_matrix_scale(grad, ALPHA);
		gsl_matrix_sub(mv->path, grad);

		prev_loss = loss;
	}

	return 0;
}

static void stance_plot(stance_t *self)
{
	const double N_EDGE = 72;
	const double INC = 0.01;

	gsl_matrix *heat = matrix_calloc(N_EDGE, N_EDGE);

	gsl_matrix *pts_cp = matrix_clone(self->pts);
	/*gsl_matrix *dpts_cp = matrix_clone(self->def_pts);*/

	for (int i = 0, y = N_EDGE / 2 - N_EDGE; i < N_EDGE; i++, y++) {
		for (int j = 0, x = N_EDGE / 2 - N_EDGE; j < N_EDGE; j++, x++) {
			self->pts->data[self->pidx * self->pts->tda + 0] = INC * x;
			self->pts->data[self->pidx * self->pts->tda + 1] = INC * y;

			double loss = stance_loss(self);
			gsl_matrix_set(heat, i, j, loss);
		}
	}

	self->pts->data[self->pidx * self->pts->tda + 0] =
		pts_cp->data[self->pidx * self->pts->tda + 0];
	self->pts->data[self->pidx * self->pts->tda + 1] =
		pts_cp->data[self->pidx * self->pts->tda + 1];

	double loss = 0.0;
	gsl_vector *pt = vector_calloc(2);
	get_optimal_step(self, pt, &loss);

	json_object *base_j, *entry_j, *arr_j;

	base_j = json_object_new_array_ext(2);

	entry_j = json_object_new_object();
	json_object_object_add(entry_j, "type", json_object_new_string("heat"));
	json_object_object_add(entry_j, "data", matrix_to_json(heat));
	json_object_object_add(entry_j, "tick", json_object_new_double(INC));
	json_object_array_put_idx(base_j, 0, entry_j);

	entry_j = json_object_new_object();
	json_object_object_add(entry_j, "type", json_object_new_string("scatter"));
	arr_j = json_object_new_array_ext(1);
	json_object_array_put_idx(arr_j, 0, json_object_new_double(pt->data[1]));
	json_object_object_add(entry_j, "x", arr_j);
	arr_j = json_object_new_array_ext(1);
	json_object_array_put_idx(arr_j, 0, json_object_new_double(-pt->data[0]));
	json_object_object_add(entry_j, "y", arr_j);
	char *loss_str;
	asprintf(&loss_str, "loss(%.3g,%.3g):%.4g", pt->data[0], pt->data[1], loss);
	json_object_object_add(entry_j, "label", json_object_new_string(loss_str));
	free(loss_str);
	json_object_array_put_idx(base_j, 1, entry_j);

	FILE *fp;
	const char *plot_str = json_object_to_json_string(base_j);
	char buf[128] = { 0 };

	sprintf(buf, "./plot/heat_%zu.json", plot_id);

	fp = fopen(buf, "w+");
	if (!fp) {
		FATAL("Failed to open json file\n");
	}

	fprintf(fp, "%s\n", plot_str);
	fclose(fp);

	json_object_put(base_j);
	gsl_vector_free(pt);
	gsl_matrix_free(pts_cp);
	gsl_matrix_free(heat);
	/*gsl_matrix_free(dpts_cp);*/

	sprintf(buf, "python ../misc/plot.py ./plot/heat_%zu.json", plot_id++);
	system(buf);
}

static void movement_plot(movement_t *self)
{
	const size_t n_interp = 100;
	stance_t *st = self->st;
	size_t n = 1, i = 0;
	json_object *base_j, *entry_j;

	while ((st = st->next))
		n++;

	base_j = json_object_new_array_ext(2 * self->n + 4);

	st = self->st;
	while (st) {
		if (st->bpts) {
			entry_j = json_object_new_object();
			json_object_object_add(entry_j, "type",
								   json_object_new_string("ring"));
			json_object_object_add(entry_j, "tdata", matrix_to_json(st->bpts));
			char *label;
			asprintf(&label, "bal:%ld", i / 2);
			json_object_object_add(entry_j, "label",
								   json_object_new_string(label));
			free(label);
			json_object_object_add(entry_j, "linestyle",
								   json_object_new_string("dashed"));
			json_object_array_put_idx(base_j, i++, entry_j);
		}

		entry_j = json_object_new_object();
		json_object_object_add(entry_j, "type", json_object_new_string("ring"));
		json_object_object_add(entry_j, "tdata", matrix_to_json(st->mod_pts));
		char *label;
		asprintf(&label, "stance:%ld", i / 2);
		json_object_object_add(entry_j, "label", json_object_new_string(label));
		free(label);
		json_object_array_put_idx(base_j, i++, entry_j);

		st = st->next;
	}

	entry_j = json_object_new_object();
	json_object_object_add(entry_j, "type", json_object_new_string("plot"));
	json_object_object_add(entry_j, "tdata", matrix_to_json(self->coms));
	json_object_object_add(entry_j, "label", json_object_new_string("coms"));
	json_object_object_add(entry_j, "marker", json_object_new_string("o"));
	json_object_array_put_idx(base_j, i++, entry_j);

	entry_j = json_object_new_object();
	json_object_object_add(entry_j, "type", json_object_new_string("plot"));
	json_object_object_add(entry_j, "tdata", matrix_to_json(self->bcoms));
	json_object_object_add(entry_j, "label", json_object_new_string("bcoms"));
	json_object_object_add(entry_j, "marker", json_object_new_string("o"));
	json_object_array_put_idx(base_j, i++, entry_j);

	entry_j = json_object_new_object();
	json_object_object_add(entry_j, "type", json_object_new_string("plot"));
	json_object_object_add(entry_j, "tdata", matrix_to_json(self->path));
	json_object_object_add(entry_j, "label", json_object_new_string("path"));
	json_object_object_add(entry_j, "marker", json_object_new_string("o"));
	json_object_array_put_idx(base_j, i++, entry_j);

	gsl_matrix *path_spl = matrix_calloc(n_interp, 2);
	double di = (double)(self->n - 1) / n_interp;
	for (size_t j = 0; j < n_interp; j++) {
		path_spl->data[j * path_spl->tda] =
			gsl_spline_eval(self->x_spl, j * di, self->acc);
		path_spl->data[j * path_spl->tda + 1] =
			gsl_spline_eval(self->y_spl, j * di, self->acc);
	}
	entry_j = json_object_new_object();
	json_object_object_add(entry_j, "type", json_object_new_string("plot"));
	json_object_object_add(entry_j, "tdata", matrix_to_json(path_spl));
	json_object_object_add(entry_j, "label",
						   json_object_new_string("path_spl"));
	json_object_array_put_idx(base_j, i++, entry_j);
	gsl_matrix_free(path_spl);

	FILE *fp;
	const char *plot_str = json_object_to_json_string(base_j);
	char buf[128] = { 0 };

	sprintf(buf, "./plot/move_%zu.json", plot_id);

	fp = fopen(buf, "w+");
	if (!fp) {
		FATAL("Failed to open json file\n");
	}

	fprintf(fp, "%s\n", plot_str);
	fclose(fp);

	json_object_put(base_j);

	sprintf(buf, "python ../misc/plot.py ./plot/move_%zu.json", plot_id++);
	system(buf);
}

static void compose_stances(movement_t *mv)
{
	size_t ci = 0, bi = 0;
	stance_t *st = mv->st;

	/*for (size_t i = 0; i < N_LEGS; i++) {*/
	/*st->pidx = i;*/
	/*stance_plot(s);*/
	/*}*/

	gsl_vector *com_off_sum = vector_calloc(2);

	gsl_matrix *path = matrix_calloc(mv->n, 2);
	gsl_matrix *mod_coms = matrix_calloc(mv->n, 2);

	while (st) {
		matrix_copy_to_origin(st->mod_pts, st->pts);
		vector_copy_to_origin(st->mod_bcom, st->bcom);
		vector_copy_to_origin(st->mod_com, st->com);

		matrix_sub_vec_rows(st->mod_pts, com_off_sum);

		gsl_vector_view vc = gsl_matrix_row(mod_coms, ci++);
		centroid_of_polygon(&vc.vector, st->mod_pts);
		gsl_vector_view bvc = gsl_matrix_row(path, bi++);
		if (st->prev) {
			calc_bcom(st->mod_pts, st->pidx, &bvc.vector);
			st->bpts = matrix_del_row_n(st->mod_pts, st->pidx);
		} else {
			vector_copy_to_origin(&bvc.vector, &vc.vector);
		}

		if (st->next) {
			int not_pidx = !st->next->pidx;
			gsl_vector_view np_nrow = gsl_matrix_row(st->next->pts, not_pidx);
			gsl_vector_add(com_off_sum, &np_nrow.vector);
			gsl_vector_view np_row = gsl_matrix_row(st->pts, not_pidx);
			gsl_vector_sub(com_off_sum, &np_row.vector);
		}

		st = st->next;
	}

	gsl_vector_free(com_off_sum);

	/*mv->path = path;*/
	mv->path = matrix_clone(mod_coms);
	mv->coms = matrix_clone(mod_coms);
	/*mv->bcoms = matrix_clone(path);*/
	mv->bcoms = path;
}

static void detail_path(movement_t *mv)
{
	gsl_block *data_x, *data_y, *tpts;

	data_x = block_calloc(mv->n);
	data_y = block_calloc(mv->n);
	tpts = block_calloc(mv->n);

	for (size_t i = 0; i < mv->path->size1; i++) {
		data_x->data[i] = mv->path->data[i * mv->path->tda];
		data_y->data[i] = mv->path->data[i * mv->path->tda + 1];
		tpts->data[i] = i;
	}

	mv->x_spl = gsl_spline_alloc(gsl_interp_akima, mv->n);
	mv->y_spl = gsl_spline_alloc(gsl_interp_akima, mv->n);

	gsl_spline_init(mv->x_spl, tpts->data, data_x->data, mv->n);
	gsl_spline_init(mv->y_spl, tpts->data, data_y->data, mv->n);

	gsl_block_free(data_x);
	gsl_block_free(data_y);
	gsl_block_free(tpts);
}

void get_movement(gsl_vector *dir, gsl_vector *pt)
{
	/*size_t n_st = 5;*/
	size_t n_st = 7;
	movement_t *mv = create_movement(n_st, dir, pt);
	compose_stances(mv);
	movement_optimize(mv);
	detail_path(mv);
	movement_plot(mv);
	movement_free(mv);
}
