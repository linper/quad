/**
 * @file stance.c
 * @brief Globl movement plan
 * @author Linas Perkauskas
 * @date 2023-05-05
 */

#include <mth.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
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

#include <glist.h>

#include <log.h>
#include <model.h>
#include <stance.h>

#define LEG_ELLIPSE_A 0.21
#define LEG_ELLIPSE_B 0.18

struct stance;

typedef struct stance {
	struct stance *next;
	struct stance *prev;
	gsl_matrix *pts; // (4 x 2)
	gsl_matrix *orig_pts; // (4 x 2)
	gsl_matrix *def_pts; // (4 x 2)
	gsl_vector *com; // (2)
	gsl_vector *bal_com; // (2)
	gsl_vector *com_off; // (2)
	gsl_vector *dir; // (2)
	size_t pidx;
	double min_loss;
} stance_t;

static void stance_free(stance_t *self)
{
	gsl_matrix_free(self->pts);
	gsl_matrix_free(self->orig_pts);
	gsl_matrix_free(self->def_pts);
	gsl_vector_free(self->com);
	gsl_vector_free(self->dir);
	gsl_vector_free(self->bal_com);
	gsl_vector_free(self->com_off);
	free(self);
}

static stance_t *stance_create_from_model(gsl_vector *dir, gsl_vector *com)
{
	stance_t *st = calloc(1, sizeof(stance_t));
	if (!st) {
		FATAL(ERR_MALLOC_FAIL);
	}

	st->pts = gsl_matrix_calloc(N_LEGS, 2);
	st->def_pts = gsl_matrix_calloc(N_LEGS, 2);
	st->bal_com = gsl_vector_calloc(2);
	st->com_off = gsl_vector_calloc(2);
	st->com = gsl_vector_calloc(2);

	if (!st->pts || !st->def_pts || !st->bal_com || !st->com_off || !st->com) {
		FATAL(ERR_MALLOC_FAIL);
	}

	for (size_t i = 0; i < N_LEGS; i++) {
		for (size_t j = 0; j < 2; j++) {
			gsl_matrix_set(st->pts, i, j, g_model->legs[i]->pos->data[j]);
			gsl_matrix_set(st->def_pts, i, j,
						   g_model->legs[i]->def_pos->data[j]);
		}
	}

	vector_copy_to(st->com, com, 0, 0, 2);
	st->orig_pts = matrix_clone(st->pts);
	st->dir = vector_clone(dir);

	st->pidx = -1;
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

	st->pts = matrix_clone(pos);
	st->orig_pts = matrix_clone(pos);
	st->def_pts = matrix_clone(def_pos);
	st->com = vector_clone(com);
	st->dir = vector_clone(dir);
	st->bal_com = gsl_vector_calloc(2);
	st->com_off = gsl_vector_calloc(2);

	st->pidx = -1;
	st->min_loss = -1;

	if (!st->bal_com || !st->com_off) {
		FATAL(ERR_MALLOC_FAIL);
	}

	return st;
}

static void calc_coms(stance_t *st)
{
	gsl_matrix *com_off_pts = gsl_matrix_calloc(N_LEGS, 2);
	gsl_matrix *bal_com_pts = gsl_matrix_calloc(N_LEGS - 1, 2);
	if (!com_off_pts || !bal_com_pts) {
		FATAL(ERR_MALLOC_FAIL);
	}

	for (size_t i = 0; i < N_LEGS; i++) {
		matrix_copy_to(com_off_pts, st->pts, g_model->cw[i], 0, i, 0, 0, 2);
	}

	size_t bcpc = 0;
	for (size_t i = 0; i < N_LEGS; i++) {
		if (i != st->pidx) {
			matrix_copy_to(com_off_pts, st->pts, g_model->cw[i], 0, bcpc++, 0,
						   0, 2);
		}
	}

	centroid_of_polygon(st->com_off, com_off_pts);
	// No need to roder legs clockwise, because 3 points are alway - triangle
	centroid_of_polygon(st->bal_com, bal_com_pts);

	gsl_matrix_free(com_off_pts);
	gsl_matrix_free(bal_com_pts);
}

static gsl_matrix *guess_start_pos(gsl_vector_view off, gsl_vector *d, size_t n)
{
	double k, b;
	bool inter;

	gsl_vector *p1, *p2;
	gsl_matrix *full_pts, *pts;

	p1 = gsl_vector_calloc(2);
	p2 = gsl_vector_calloc(2);
	pts = gsl_matrix_calloc(n, 2);
	if (!p1 || !p2 || !pts) {
		FATAL(ERR_MALLOC_FAIL);
	}

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

static void eval_support_shape(stance_t *st, double *val)
{
	int max_ps_coaf = 16;
	int cw[N_LEGS] = { 0, 1, 3, 2 };
	double cw_pts[N_LEGS][2] = { 0 };
	double pair1[2], pair2[2];
	double surface, perimeter = 0.0;
	gsl_vector_view p1, p2;

	for (size_t i = 0; i < N_LEGS; i++) {
		// clockwise points adjusted by default leg excentricity
		p1 = gsl_matrix_row(st->pts, cw[i]);
		p2 = gsl_matrix_row(st->pts, (cw[i] + 1) / N_LEGS);
		cw_pts[i][0] = gsl_vector_get(&p1.vector, 0) * g_model->leg_exc;
		cw_pts[i][1] = gsl_vector_get(&p1.vector, 1) * 1.0;

		perimeter += vector_dist(&p1.vector, &p2.vector);
	}

	pair1[0] = area(cw_pts[0], cw_pts[1], cw_pts[2]);
	pair1[1] = area(cw_pts[2], cw_pts[3], cw_pts[0]);

	pair2[0] = area(cw_pts[1], cw_pts[2], cw_pts[3]);
	pair2[1] = area(cw_pts[3], cw_pts[0], cw_pts[1]);

	surface = (pair1[0] + pair1[1] + pair2[0] + pair2[1]) / 2;

	if (val)
		*val = 1 - ((max_ps_coaf * surface) / (perimeter * perimeter));
}

static void eval_avg_move_dir(stance_t *st, double *val)
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

static void eval_sequence(stance_t *st, double *avg_val, double *edge_val)
{
	double edge_retr_vals[N_LEGS] = { 0 };
	double max_d = 0.0, ins_val, lox, loy, k, b, edge_part, avg_part;
	gsl_matrix *off;
	gsl_vector_view loff;
	gsl_vector *pf, *p1, *p2, *dl_vec;
	vbuf_t *dists_left;

	dists_left = vbuf_create(8);
	off = matrix_sub_n(st->pts, st->def_pts);
	p1 = gsl_vector_calloc(2);
	p2 = gsl_vector_calloc(2);
	if (!p1 || !p2) {
		FATAL(ERR_MALLOC_FAIL);
	}

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

static void eval_min_prox(stance_t *st, double *val)
{
	const double k = -30.0, r = 0.006;
	double px, py, x, y, dist, min_dist = DBL_MAX;

	px = gsl_matrix_get(st->pts, st->pidx, 0);
	py = gsl_matrix_get(st->pts, st->pidx, 1);

	for (size_t i = 0; i < N_LEGS; i++) {
		x = gsl_matrix_get(st->orig_pts, i, 0);
		y = gsl_matrix_get(st->orig_pts, i, 1);
		dist = sqrt((px - x) * (px - x) + (py - y) * (py - y));

		if (dist < min_dist) {
			min_dist = dist;
		}
	}

	if (val)
		*val = bound_data(k * min_dist - k * r + 1, 0.0, 1.0);
}

static double stance_loss(stance_t *st)
{
#define PROX_C 0.1
#define AVG_C 0.6
#define EDGE_C 0.5
#define AP_DIR_C 0.3
#define SUP_TRI_C 0.2

	double val, prox_part, avg_part, apd_part, edge_part, sup_shp_part;

	eval_min_prox(st, &prox_part);
	prox_part *= PROX_C;

	eval_sequence(st, &avg_part, &edge_part);
	avg_part *= AVG_C;
	edge_part *= EDGE_C;

	eval_avg_move_dir(st, &apd_part);
	apd_part *= AP_DIR_C;

	eval_support_shape(st, &sup_shp_part);
	sup_shp_part *= SUP_TRI_C;

	val = prox_part + avg_part + edge_part + sup_shp_part + apd_part;
	/*val = avg_part + prox_part;*/
	/*val = avg_part;*/
	/*val = edge_part;*/

	return val;
}

static int stance_grad(stance_t *st, gsl_matrix *spts, gsl_matrix *pts_mod,
					   gsl_vector *losses)
{
#define DELTA 1e-5

	gsl_matrix *grad, *pts_save;
	double base_loss, var_loss;

	pts_save = matrix_clone(st->pts);
	grad = gsl_matrix_calloc(spts->size1, spts->size2);
	if (!grad) {
		FATAL(ERR_MALLOC_FAIL);
	}

	for (size_t i = 0; i < spts->size1; i++) {
		matrix_copy_to_origin(st->pts, st->pts);
		matrix_copy_to(st->pts, spts, st->pidx, 0, i, 0, 1, st->pts->size2);
		base_loss = stance_loss(st);
		gsl_vector_set(losses, i, base_loss);
		for (size_t j = 0; j < st->pts->size2; j++) {
			*gsl_matrix_ptr(st->pts, st->pidx, j) += DELTA;

			var_loss = stance_loss(st);
			gsl_matrix_set(grad, i, j, (var_loss - base_loss) / DELTA);

			// restoring
			*gsl_matrix_ptr(st->pts, st->pidx, j) -= DELTA;
		}

		matrix_copy_to(pts_mod, st->pts, i, 0, st->pidx, 0, 1, pts_mod->size2);
	}

	matrix_copy_to_origin(st->pts, pts_save);

	return 0;
}

static int get_optimal_step(stance_t *st, gsl_vector *pt, double *loss)
{
#define MAX_GD_ITER 25
#define EPSILON 1e-3
#define ALPHA 0.01
#define N_START_PTS 6

	gsl_matrix *spts, *pts_mod;
	gsl_vector_view row, res_row;
	gsl_vector *losses, *prev_losses;

	row = gsl_matrix_row(st->def_pts, st->pidx);
	spts = guess_start_pos(row, st->dir, N_START_PTS);
	if (!spts) {
		return 0;
	}

	pts_mod = gsl_matrix_calloc(N_START_PTS, 2);
	losses = gsl_vector_calloc(N_START_PTS);
	gsl_vector_set_all(losses, DBL_MAX);
	prev_losses = vector_clone(losses);
	if (!pts_mod || !losses) {
		FATAL(ERR_MALLOC_FAIL);
	}

	for (size_t i = 0; i < MAX_GD_ITER; i++) {
		stance_grad(st, spts, pts_mod, losses);
		gsl_matrix_scale(pts_mod, ALPHA);
		gsl_matrix_sub(spts, pts_mod);

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

static stance_t *optimize(stance_t *self, size_t n)
{
	double min_loss, loss;
	size_t min_idx = 0;

	gsl_vector *min_pt, *pt;
	stance_t *en, *new_stc;

	en = self;
	min_pt = gsl_vector_alloc(2);
	pt = gsl_vector_alloc(2);
	if (!min_pt || !pt) {
		FATAL(ERR_MALLOC_FAIL);
	}

	// Get last stance
	while (en->next) {
		en = en->next;
	}

	for (size_t ni = 0; ni < n; ni++) {
		// Add
		new_stc = stance_create(en->pts, en->def_pts, en->dir, en->com);
		en->next = new_stc;
		new_stc->prev = en;
		en = new_stc;

		/*min_pts = matrix_clone(en->pts);*/

		min_loss = DBL_MAX;

		for (size_t li = 0; li < N_LEGS; li++) {
			en->pidx = li;
			get_optimal_step(en, pt, &loss);
			if (loss < min_loss) {
				vector_copy_to_origin(min_pt, pt);
				min_loss = loss;
				min_idx = li;
			}
		}
		en->pidx = min_idx;
		en->min_loss = loss;
		gsl_matrix_set_row(en->pts, min_idx, min_pt);

		printf("point:");
		vector_print(pt);
		printf("loss:%.4g pidx:%ld\n", loss, min_idx);

		calc_coms(en);

		matrix_sub_vec_rows(en->pts, en->com_off);
		gsl_vector_set_all(en->com, 0.0);
	}

	gsl_vector_free(min_pt);
	gsl_vector_free(pt);

	return en;
}

static void stance_plot(stance_t *self)
{
#define N_EDGE 64
#define INC 0.01

	gsl_matrix *heat = gsl_matrix_calloc(N_EDGE, N_EDGE);

	gsl_matrix *pts_cp = matrix_clone(self->pts);
	gsl_matrix *dpts_cp = matrix_clone(self->def_pts);

	for (int i = 0, y = N_EDGE / 2 - N_EDGE; i < N_EDGE; i++, y++) {
		for (int j = 0, x = N_EDGE / 2 - N_EDGE; j < N_EDGE; j++, x++) {
			self->pts->data[self->pidx * self->pts->tda + 0] += INC * x;
			self->pts->data[self->pidx * self->pts->tda + 1] += INC * y;

			double loss = stance_loss(self);
			gsl_matrix_set(heat, i, j, loss);

			self->pts->data[self->pidx * self->pts->tda + 0] -= INC * x;
			self->pts->data[self->pidx * self->pts->tda + 1] -= INC * y;
		}
	}

	json_object *base_j;

	base_j = json_object_new_object();
	json_object_object_add(base_j, "type", json_object_new_string("heat"));
	json_object_object_add(base_j, "data", matrix_to_json(heat));

	FILE *fp;
	const char *plot_str = json_object_to_json_string(base_j);

	fp = fopen("./plot/heat.json", "w+");
	if (!fp) {
		FATAL("Failed to open json file\n");
	}

	fprintf(fp, "%s\n", plot_str);
	fclose(fp);

	json_object_put(base_j);
	gsl_matrix_free(pts_cp);
	gsl_matrix_free(heat);
	gsl_matrix_free(dpts_cp);

	system("python ../misc/plot.py ./plot/heat.json");
}

void get_movement(gsl_vector *dir, gsl_vector *pt)
{
	(void)optimize;
	stance_t *s = stance_create_from_model(dir, pt);
	/*optimize(s, 1);*/
	for (size_t i = 0; i < N_LEGS; i++) {
		s->pidx = i;
		stance_plot(s);
	}

	stance_free(s);
}
