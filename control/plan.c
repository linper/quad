/**
 * @file plan.c
 * @brief Individual leg movement plan
 * @author Linas Perkauskas
 * @date 2023-02-25
 */

#include <gsl/gsl_block_double.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>
#include <mth.h>
#include <stance.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <gsl/gsl_vector_double.h>

#include "log.h"
#include <fsm.h>
#include <plan.h>
#include <balance.h>
#include <glist.h>
/*#include <stash.h>*/
#include <model.h>

#define MIN_N_INTERP 5
#define START_N_STEPS 4096

enum lfsm_act {
	LFSMA_NO_ACT,
	LFSMA_END,
	LFSMA_HIT,
	LFSMA_STOP,
	LFSMA_MAX,
};

/*
static double get_d2_speed(dpt_t *d1, dpt_t *d2)
{
	double vel, S;
	gsl_vector *d;

	d = vector_sub_n(d2->pos, d1->pos);
	S = vector_length(d);
	vel = S / (d2->ts - d1->ts);

	return vel;
}
*/

static int gradual_vfunc(plan_t *p, dpt_t *d1, dpt_t *d2, double dist_st,
						 gsl_block **dists_p, gsl_block **vels_ps_p)
{
	gsl_block *dists, *vels_ps;
	double prev_v = 0.0, vel;
	double tm_sum = dist_st;

	if (!p || !d1 || !d2 || !dists_p || !vels_ps_p) {
		return 1;
	}

	vels_ps = block_linspace(d1->vps, d2->vps, d2->ts - d1->ts + 1);
	dists = gsl_block_calloc(d2->ts - d1->ts + 1);

	for (size_t i = 0; i < vels_ps->size; i++) {
		vel = vels_ps->data[i];
		tm_sum += (vel + prev_v) / 2;
		prev_v = vel;
		/*prev_v = vel + prev_v;*/
		dists->data[i] = tm_sum;
	}

	*dists_p = dists;
	*vels_ps_p = vels_ps;

	return 0;
}

static int variable_vfunc(plan_t *p, dpt_t *d1, dpt_t *d2, double dist_st,
						  gsl_block **dists_p, gsl_block **vels_ps_p)
{
	/*gsl_vector *tmp_m;*/
	gsl_block *dists, *vels_ps;
	double S_dif, sp_top, inc, sum = 0.0, tm_sum = 0.0, prev_v = 0.0, vel;
	int t_dt;

	if (!p || !d1 || !d2 || !dists_p || !vels_ps_p) {
		return 1;
	}

	gradual_vfunc(p, d1, d2, dist_st, &dists, &vels_ps);
	/*tmp_m = vector_sub_n(d2->pos, d1->pos);*/

	/*S_dif = vector_length(tmp_m);*/
	S_dif = (d2->ts - d1->ts) - (dists->data[dists->size - 1] - dists->data[0]);
	/*gsl_vector_free(tmp_m);*/

	t_dt = d2->ts - d1->ts;
	sp_top = 2 * S_dif / t_dt;
	inc = sp_top / (vels_ps->size - 1);

	for (size_t i = 0; i < vels_ps->size; i++) {
		vels_ps->data[i] += sum;

		if (i > vels_ps->size / 2) {
			sum -= inc;
		} else {
			sum += inc;
		}
	}

	for (size_t i = 0; i < vels_ps->size; i++) {
		vel = vels_ps->data[i];
		tm_sum += (vel + prev_v) / 2;
		prev_v = vel;
		dists->data[i] += tm_sum;
	}

	*dists_p = dists;
	*vels_ps_p = vels_ps;

	return 0;
}

static void plan_adjust_z(plan_t *self, double z)
{
	/*double cur_z = *self->cur->z;*/

	double mod_z = *self->cur->z + gsl_vector_get(self->adj, 2) + z;
	/*printf("%g:%g:%g::%g\n", *self->cur->z, gsl_vector_get(self->adj, 2), z,*/
	/*mod_z);*/

	if (mod_z < g_model->max_dip) {
		mod_z = g_model->max_dip - mod_z;
	} else if (mod_z > g_model->min_dip) {
		mod_z = g_model->min_dip - mod_z;
	} else {
		mod_z = z;
	}

	/*printf("modz:%g\n", mod_z);*/

	*gsl_vector_ptr(self->adj, 2) += mod_z;
}

/* @brief Puts new creates new step in cir. buffer, and returns it's pointer */
static dpt_t *dpt_hist_step(plan_t *p)
{
	int new_idx = (p->h_idx + 1) % DPT_CBUF_SIZE;
	dpt_t *d = p->hist[new_idx];
	p->h_idx = new_idx;

	d->ts = p->dists->data[p->next_i];
	d->vps = p->vels->data[p->next_i];
	/*printf("eval:%g\n", d->ts);*/
	gsl_vector_set(d->pos, 0, gsl_spline_eval(p->x_spl, d->ts, p->acc));
	gsl_vector_set(d->pos, 1, gsl_spline_eval(p->y_spl, d->ts, p->acc));
	gsl_vector_set(d->pos, 2, gsl_spline_eval(p->z_spl, d->ts, p->acc));

	d->x = gsl_vector_ptr(d->pos, 0);
	d->y = gsl_vector_ptr(d->pos, 1);
	d->z = gsl_vector_ptr(d->pos, 2);

	return d;
}

static dpt_t *dpt_hist_repeat(plan_t *p)
{
	int new_idx = (p->h_idx + 1) % DPT_CBUF_SIZE;
	dpt_t *d_old = p->hist[p->h_idx];
	dpt_t *d_new = p->hist[new_idx];

	d_new->ts = d_old->ts + 1;
	d_new->vps = d_old->vps;
	/*printf("repeat eval:%g\n", d_new->ts);*/
	gsl_vector_set(d_new->pos, 0, gsl_vector_get(d_old->pos, 0));
	gsl_vector_set(d_new->pos, 1, gsl_vector_get(d_old->pos, 1));
	gsl_vector_set(d_new->pos, 2, gsl_vector_get(d_old->pos, 2));

	d_new->x = gsl_vector_ptr(d_new->pos, 0);
	d_new->y = gsl_vector_ptr(d_new->pos, 1);
	d_new->z = gsl_vector_ptr(d_new->pos, 2);

	p->h_idx = new_idx;

	return d_new;
}

static dpt_t *dpt_hist_get(plan_t *p, int idx)
{
	int h_idx = (p->h_idx + DPT_CBUF_SIZE + idx) % DPT_CBUF_SIZE;

	return p->hist[h_idx];
}

static void plan_make_steps(plan_t *self)
{
	double *pts_data, *tpts, *vps, *data_x, *data_y, *data_z, off;
	double epsilon = 0.0001; //small constant for float rounding
	size_t n, num_steps, cur_num_steps = 0;
	dpt_t *e, *e_next;
	gsl_block *vels, *dists;

	n = self->pts->count;
	pts_data = malloc(5 * n * sizeof(double));
	if (!pts_data) {
		FATAL(ERR_MALLOC_FAIL);
	}

	tpts = pts_data;
	vps = pts_data + n;
	data_x = pts_data + 2 * n;
	data_y = pts_data + 3 * n;
	data_z = pts_data + 4 * n;

	for (size_t i = 0; i < n; i++) {
		e = (dpt_t *)glist_get(self->pts, i);
		tpts[i] = e->ts;
		vps[i] = e->vps;
		data_x[i] = *e->x;
		data_y[i] = *e->y;
		data_z[i] = *e->z;
	}

	num_steps = (size_t)round(tpts[n - 1] - tpts[0]);
	block_check_size(self->dists, 2 * num_steps);
	block_check_size(self->vels, 2 * num_steps);

	off = tpts[0];
	tpts[0] -= epsilon;
	tpts[n - 1] += epsilon;

	for (size_t i = 0; i < n - 1; i++) {
		e = (dpt_t *)self->pts->array[i];
		e_next = (dpt_t *)self->pts->array[i + 1];

		variable_vfunc(self, e, e_next, off, &dists, &vels);
		off = dists->data[dists->size - 1];
		if (i) {
			memcpy(self->dists->data + cur_num_steps, dists->data + 1,
				   (dists->size - 1) * sizeof(double));
			memcpy(self->vels->data + cur_num_steps, vels->data + 1,
				   (vels->size - 1) * sizeof(double));
			cur_num_steps += dists->size - 1;
		} else {
			memcpy(self->dists->data, dists->data,
				   dists->size * sizeof(double));
			memcpy(self->vels->data, vels->data, vels->size * sizeof(double));
			cur_num_steps += dists->size;
		}

		gsl_block_free(dists);
		gsl_block_free(vels);
	}

	double st0 = self->dists->data[0];
	double fi0 = self->dists->data[cur_num_steps - 1];
	double st1 = ((dpt_t *)self->pts->array[0])->ts;
	double fi1 = ((dpt_t *)self->pts->array[n - 1])->ts;
	double dist_mul = (fi1 - st1) / (fi0 - st0);

	block_add_constant_ex(self->dists, -st0, cur_num_steps);
	block_mul_constant_ex(self->dists, dist_mul, cur_num_steps);
	block_add_constant_ex(self->dists, st1, cur_num_steps);

	gsl_interp_accel_reset(self->acc);
	gsl_spline_free(self->x_spl);
	gsl_spline_free(self->y_spl);
	gsl_spline_free(self->z_spl);

	self->x_spl = gsl_spline_alloc(gsl_interp_akima, n);
	self->y_spl = gsl_spline_alloc(gsl_interp_akima, n);
	self->z_spl = gsl_spline_alloc(gsl_interp_akima, n);

	gsl_spline_init(self->x_spl, tpts, data_x, n);
	gsl_spline_init(self->y_spl, tpts, data_y, n);
	gsl_spline_init(self->z_spl, tpts, data_z, n);

	self->n_steps = cur_num_steps;

	free(pts_data);
}

static void plan_next(plan_t *self, bool repeat)
{
	if (!repeat) {
		self->cur = dpt_hist_step(self);
	} else {
		self->cur = dpt_hist_repeat(self);
	}

	leg_t *l = container_of(self, leg_t, plan);
	gsl_vector_set_all(l->pos, 0.0);
	gsl_vector_add(l->pos, self->cur->pos);
	gsl_vector_add(l->pos, self->adj);

	if (!repeat) {
		self->next_i++;
	}
}

static void plan_reset(plan_t *self)
{
	glist_clear(self->raw_pts);
	glist_clear(self->pts);
	dpt_free(self->target);

	self->target = NULL;
	self->need_plan = true;
	self->next_i = 0;
}

static void plan_make(plan_t *self, enum lfsm_state state, glist_t *pts)
{
	fsm_reset(self->fsm);
	plan_reset(self);
	glist_copy_to(pts, self->raw_pts);
	dpt_t *pt = glist_get(pts, 0);
	self->target = dpt_clone(pt);

	fsm_set(self->fsm, state);
}

static int act_ascending(fsm_t *fsm)
{
	const double TOP_TM_C = 0.4;
	const double MID_PT_HT_C = 0.75;
	const double END_OF_C = 0.5;
	const double MID_OF_C = 0.25;

	leg_t *l = (leg_t *)fsm->priv;
	plan_t *p = &l->plan;
	dpt_t *pre, *post;

	double dx, dy, step_dist, end_ts, *walk_h;
	dpt_t *start, *end, *mid;

	if (p->need_plan) {
		DBG("%s: Ascending\n", l->name);
		l->bal = false;
		walk_h = g_model->sens->walk_h;

		pre = dpt_clone(dpt_hist_get(p, -1));
		pre->vps = 1.0;

		start = dpt_clone(dpt_hist_get(p, 0));
		start->vps = 1.0;

		/*start = dpt_new(p->cur->pos->arr, p->cur->vel->arr, 0, 0.0, p->stash);*/
		end_ts = start->ts + TOP_TM_C * (p->target->ts - start->ts);
		dx = *p->target->x - *start->x;
		dy = *p->target->y - *start->y;
		step_dist = sqrtf(dx * dx + dy * dy);

		walk_h[l->idx] =
			get_walk_height(step_dist, g_model->sens->abs_std_leg_h);

		double end_pos[3] = {
			*start->x + END_OF_C * (*p->target->x - *start->x),
			*start->y + END_OF_C * (*p->target->y - *start->y), walk_h[l->idx]
		};

		end = dpt_new(end_pos, end_ts, 1.0);

		post = dpt_clone(end);
		post->ts += 1.0;

		double mid_pos[3] = {
			*start->x + MID_OF_C * (*p->target->x - *start->x),
			*start->y + MID_OF_C * (*p->target->y - *start->y),
			*start->z + MID_PT_HT_C * (walk_h[l->idx] - *start->z)
		};

		mid = dpt_new(mid_pos, (start->ts + end_ts) / 2, 1.0);

		/*start->vps = 1.0; // maybe not needed*/
		/*mid->vps = 1.0;*/
		/*end->vps = 1.0;*/

		/*start->vps = 0.0; // maybe not needed*/
		/*mid->vps = get_d2_speed(start, mid);*/
		/*end->vps = get_d2_speed(mid, end);*/

		glist_push(p->pts, pre);
		glist_push(p->pts, start);
		glist_push(p->pts, mid);
		glist_push(p->pts, end);
		glist_push(p->pts, post);

		/*fill_diffs(p, p->pts);*/
		plan_make_steps(p);
		p->need_plan = false;
	}

	if (p->next_i == p->n_steps) {
		/* TODO: commenting these may break something <26-02-23, yourname> */
		/* double wh = self.info.walk_h*/
		dpt_t *c_tg = dpt_clone(p->target);
		plan_reset(p);
		p->target = c_tg;
		/*self.info.walk_h = wh*/
		fsm_next(fsm, LFSMA_END);
	} else {
		plan_next(p, false);
	}

	return 0;
}

static int act_descending(fsm_t *fsm)
{
	const double HT_C = 0.2;
	const double OF_C = 0.96;
	const double TM_C = 0.7;

	leg_t *l = (leg_t *)fsm->priv;
	plan_t *p = &l->plan;
	double mid_ts;
	dpt_t *start, *end, *mid, *pre, *post;

	if (p->need_plan) {
		DBG("%s: Descending\n", l->name);
		l->bal = false;

		pre = dpt_clone(dpt_hist_get(p, -1));

		start = dpt_clone(dpt_hist_get(p, 0));
		end = dpt_new2(p->target->pos, p->target->ts, 1.0);

		post = dpt_clone(end);
		post->ts += 1.0;

		mid_ts = roundf(start->ts + TM_C * (p->target->ts - start->ts));

		double mid_pos[3] = { *start->x + OF_C * (*p->target->x - *start->x),
							  *start->y + OF_C * (*p->target->y - *start->y),
							  *end->z + HT_C * (g_model->sens->walk_h[l->idx] -
												*end->z) };

		mid = dpt_new(mid_pos, mid_ts, 1.0);
		/*mid->vps = get_d2_speed(start, mid);*/

		glist_push(p->pts, pre);
		glist_push(p->pts, start);
		glist_push(p->pts, mid);
		glist_push(p->pts, end);
		glist_push(p->pts, post);

		/*fill_diffs(p, p->pts);*/

		plan_make_steps(p);

		p->need_plan = false;
	}

	if (g_model->sens->touch_f->data[l->idx] > 0.0) {
		plan_reset(p);
		fsm_next(fsm, LFSMA_HIT);
	} else if (p->next_i == p->n_steps) {
		plan_reset(p);
		fsm_next(fsm, LFSMA_END);
	} else {
		plan_next(p, false);
	}

	return 0;
}

static int act_traversing(fsm_t *fsm)
{
	leg_t *l = (leg_t *)fsm->priv;
	plan_t *p = &l->plan;
	dpt_t *d, *dc;

	if (p->need_plan) {
		DBG("%s: Traversing\n", l->name);
		l->bal = true;

		for (int i = MIN(-MIN_N_INTERP + p->raw_pts->count + 1, -1); i <= 0;
			 i++) {
			d = dpt_hist_get(p, i);
			dc = dpt_clone(d);
			glist_push(p->pts, dc);
		}

		for (size_t i = 0; i < p->raw_pts->count; i++) {
			dc = dpt_clone((dpt_t *)p->raw_pts->array[i]);
			glist_push(p->pts, dc);
		}

		plan_make_steps(p);
		p->need_plan = false;
	}

	if (p->next_i == p->n_steps) {
		plan_reset(p);
		fsm_next(fsm, LFSMA_END);
	} else {
		plan_adjust_z(p, g_model->sens->balance->data[l->idx]);
		plan_next(p, false);
	}

	return 0;
}

static int act_stoped(fsm_t *fsm)
{
	leg_t *l = (leg_t *)fsm->priv;
	plan_t *p = &l->plan;
	l->bal = false;

	if (p->need_plan) {
		DBG("%s: Stoping\n", l->name);
		p->need_plan = false;
		l->bal = true;
	}

	return 0;
}

static int act_pending(fsm_t *fsm)
{
	leg_t *l = (leg_t *)fsm->priv;
	plan_t *p = &l->plan;

	if (p->need_plan) {
		DBG("%s: Pending\n", l->name);
		p->need_plan = false;
		l->bal = true;
	}

	plan_adjust_z(p, g_model->sens->balance->data[l->idx]);
	plan_next(p, true);

	return 0;
}

static fsm_func lfsm_funcs[LFSMS_MAX] = {
	[LFSMS_STP] = act_stoped,	  [LFSMS_ASC] = act_ascending,
	[LFSMS_DSC] = act_descending, [LFSMS_TRV] = act_traversing,
	[LFSMS_PND] = act_pending,
};

static unsigned lfsm_states[LFSMA_MAX][LFSMS_MAX] = {
	[LFSMA_NO_ACT] = {
		[LFSMS_STP]=LFSMS_STP,
		[LFSMS_ASC]=LFSMS_ASC,
		[LFSMS_DSC]=LFSMS_DSC,
		[LFSMS_TRV]=LFSMS_TRV,
		[LFSMS_PND]=LFSMS_PND,
	},           
	[LFSMA_END] = {
		[LFSMS_STP]=LFSMS_PND,
		[LFSMS_ASC]=LFSMS_DSC,
		[LFSMS_DSC]=LFSMS_PND,
		[LFSMS_TRV]=LFSMS_PND,
		[LFSMS_PND]=LFSMS_PND,
	},
	[LFSMA_HIT] = {
		[LFSMS_STP]=LFSMS_STP,
		[LFSMS_ASC]=LFSMS_ASC,
		[LFSMS_DSC]=LFSMS_PND,
		[LFSMS_TRV]=LFSMS_TRV,
		[LFSMS_PND]=LFSMS_PND,
	},
	[LFSMA_STOP] = {
		[LFSMS_STP]=LFSMS_STP,
		[LFSMS_ASC]=LFSMS_STP,
		[LFSMS_DSC]=LFSMS_STP,
		[LFSMS_TRV]=LFSMS_STP,
		[LFSMS_PND]=LFSMS_STP,
	},
};

dpt_t *dpt_new(const double pos[3], double ts, double vps)
{
	dpt_t *p;

	p = calloc(1, sizeof(dpt_t));
	if (!p) {
		FATAL(ERR_MALLOC_FAIL);
	}

	p->pos = vector_from_array(3, pos);
	if (!p->pos) {
		FATAL(ERR_MALLOC_FAIL);
	}

	p->x = gsl_vector_ptr(p->pos, 0);
	p->y = gsl_vector_ptr(p->pos, 1);
	p->z = gsl_vector_ptr(p->pos, 2);

	p->ts = ts;
	p->vps = vps;

	return p;
}

dpt_t *dpt_new2(gsl_vector *pos, double ts, double vps)
{
	dpt_t *p;

	p = calloc(1, sizeof(dpt_t));
	if (!p) {
		FATAL(ERR_MALLOC_FAIL);
	}

	p->pos = vector_clone(pos);
	if (!p->pos) {
		FATAL(ERR_MALLOC_FAIL);
	}

	p->x = gsl_vector_ptr(p->pos, 0);
	p->y = gsl_vector_ptr(p->pos, 1);
	p->z = gsl_vector_ptr(p->pos, 2);

	p->ts = ts;
	p->vps = vps;

	return p;
}

dpt_t *dpt_new3(gsl_vector *pos, double ts, double vps)
{
	dpt_t *p;

	p = calloc(1, sizeof(dpt_t));
	if (!p) {
		FATAL(ERR_MALLOC_FAIL);
	}

	p->pos = pos;

	p->x = gsl_vector_ptr(p->pos, 0);
	p->y = gsl_vector_ptr(p->pos, 1);
	p->z = gsl_vector_ptr(p->pos, 2);

	p->ts = ts;
	p->vps = vps;

	return p;
}

/*
dpt_t *dpt_from_spl(plan_t *p, double ts, double vps)
{
	double arr[3] = { 0 };
	dpt_t *d;

	if (!p || !p->x_spl || !p->y_spl || !p->z_spl) {
		return NULL;
	}

	arr[0] = gsl_spline_eval(p->x_spl, ts, p->acc);
	arr[1] = gsl_spline_eval(p->y_spl, ts, p->acc);
	arr[2] = gsl_spline_eval(p->z_spl, ts, p->acc);
	d = dpt_new(arr, ts, vps);

	return d;
}
*/

dpt_t *dpt_clone(dpt_t *d)
{
	dpt_t *p;

	p = calloc(1, sizeof(dpt_t));
	if (!p) {
		FATAL(ERR_MALLOC_FAIL);
	}

	p->pos = vector_clone(d->pos);
	if (!p->pos) {
		FATAL(ERR_MALLOC_FAIL);
	}

	p->x = gsl_vector_ptr(p->pos, 0);
	p->y = gsl_vector_ptr(p->pos, 1);
	p->z = gsl_vector_ptr(p->pos, 2);

	p->ts = d->ts;
	p->vps = d->vps;

	return p;
}

void dpt_free(dpt_t *d)
{
	if (d) {
		gsl_vector_free(d->pos);
		free(d);
	}
}

int plan_new(plan_t *p)
{
	dpt_t *st_p;
	leg_t *l = container_of(p, leg_t, plan);

	for (int i = 0; i < DPT_CBUF_SIZE; i++) {
		p->hist[i] = dpt_new2(l->def_pos, i, 1.0);
	}
	p->h_idx = DPT_CBUF_SIZE - 1;

	st_p = p->hist[p->h_idx];

	p->need_plan = true;
	p->cur = st_p;
	p->target = NULL;
	p->raw_pts = glist_new(8);
	glist_set_free_cb(p->raw_pts, (void (*)(void *))dpt_free);
	p->pts = glist_new(16);
	glist_set_free_cb(p->pts, (void (*)(void *))dpt_free);
	p->adj = gsl_vector_calloc(3);

	p->dists = gsl_block_calloc(START_N_STEPS);
	p->vels = gsl_block_calloc(START_N_STEPS);

	p->acc = gsl_interp_accel_alloc();

	p->fsm =
		fsm_new((unsigned *)lfsm_states, lfsm_funcs, LFSMS_MAX, LFSMA_MAX, l);

	if (!p->cur || !p->raw_pts || !p->pts || !p->adj || !p->dists || !p->vels ||
		!p->acc || !p->fsm) {
		return 1;
	}

	return 0;
}

void plan_free(plan_t *p)
{
	if (!p)
		return;

	for (int i = 0; i < DPT_CBUF_SIZE; i++) {
		dpt_free(p->hist[i]);
	}

	gsl_block_free(p->dists);
	gsl_block_free(p->vels);
	glist_free(p->raw_pts);
	glist_free(p->pts);
	fsm_free(p->fsm);
	/*glist_free(p->targets);*/
	/*glist_free(p->steps);*/

	gsl_vector_free(p->adj);

	/*stash_free(p->stash);*/

	gsl_spline_free(p->x_spl);
	gsl_spline_free(p->y_spl);
	gsl_spline_free(p->z_spl);
	gsl_interp_accel_free(p->acc);
}

void plan_step(plan_t *self)
{
	if (fsm_execute(self->fsm)) {
		ERR("FSM execute failed\n");
	}

	/*plan_compensate(self);*/
}

void plan_make_movement(plan_t *self, double *pts, size_t n_pts, bool do_lift)
{
	/*(void)self;*/
	/*(void)pts;*/
	/*(void)n_pts;*/
	(void)do_lift;

	double tsum;
	dpt_t *p, *pp;
	glist_t *lst;

	lst = glist_new(n_pts);
	pp = dpt_hist_get(self, 0);
	tsum = pp->ts;

	if (do_lift) {
		p = dpt_new(pts + 3 * (n_pts - 1), tsum + 50, 1.0);
		glist_push(lst, p);
		plan_make(self, LFSMS_ASC, lst);
	} else {
		for (size_t i = 0; i < n_pts; ++i) {
			tsum += 50;
			p = dpt_new(pts, tsum, 1.0);
			glist_push(lst, p);
			pp = p;
			pts += 3;
		}
		plan_make(self, LFSMS_TRV, lst);
	}

	glist_free_shallow(lst);
}
