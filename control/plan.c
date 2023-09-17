/**
 * @file plan.c
 * @brief Individual leg movement plan
 * @author Linas Perkauskas
 * @date 2023-02-25
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_block_double.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>

#include "log.h"
#include <mth.h>
#include <fsm.h>
#include <plan.h>
#include <balance.h>
#include <glist.h>
#include <model.h>
#include <stance.h>

#define MIN_N_INTERP 5
#define START_N_STEPS 4096

enum lfsm_act {
	LFSMA_NO_ACT,
	LFSMA_END,
	LFSMA_HIT,
	LFSMA_STOP,
	LFSMA_MAX,
};

static int gradual_vfunc(dpt_t *d1, dpt_t *d2, double dist_st,
						 gsl_block **dists_p, gsl_block **vels_ps_p)
{
	gsl_block *dists, *vels_ps;
	double prev_v = 0.0;
	double tm_sum = dist_st;

	if (!d1 || !d2 || !dists_p || !vels_ps_p) {
		return 1;
	}

	vels_ps = block_linspace(d1->vps, d2->vps, d2->ts - d1->ts + 1);
	dists = block_calloc(d2->ts - d1->ts + 1);

	for (size_t i = 0; i < vels_ps->size; i++) {
		double vel = vels_ps->data[i];
		tm_sum += (vel + prev_v) / 2;
		prev_v = vel;
		dists->data[i] = tm_sum;
	}

	*dists_p = dists;
	*vels_ps_p = vels_ps;

	return 0;
}

static int variable_vfunc(dpt_t *d1, dpt_t *d2, double dist_st,
						  gsl_block **dists_p, gsl_block **vels_ps_p)
{
	gsl_block *dists, *vels_ps;
	double S_dif, sp_top, inc, sum = 0.0, tm_sum = 0.0, prev_v = 0.0;
	int t_dt;

	if (!d1 || !d2 || !dists_p || !vels_ps_p) {
		return 1;
	}

	gradual_vfunc(d1, d2, dist_st, &dists, &vels_ps);

	S_dif = (d2->ts - d1->ts) - (dists->data[dists->size - 1] - dists->data[0]);

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
		double vel = vels_ps->data[i];
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
	double *pts_data, *tpts, *data_x, *data_y, *data_z, off;
	double epsilon = 0.0001; //small constant for float rounding
	size_t n, num_steps, cur_num_steps = 0;
	gsl_block *vels, *dists;

	n = self->pts->count;
	pts_data = malloc(5 * n * sizeof(double));
	if (!pts_data) {
		FATAL(ERR_MALLOC_FAIL);
	}

	tpts = pts_data;
	data_x = pts_data + 2 * n;
	data_y = pts_data + 3 * n;
	data_z = pts_data + 4 * n;

	for (size_t i = 0; i < n; i++) {
		dpt_t *e = (dpt_t *)glist_get(self->pts, i);
		tpts[i] = e->ts;
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
		dpt_t *e = (dpt_t *)self->pts->array[i];
		dpt_t *e_next = (dpt_t *)self->pts->array[i + 1];

		variable_vfunc(e, e_next, off, &dists, &vels);
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

	/*block_print(self->dists);*/

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

static void plan_resched(plan_t *self)
{
	glist_clear(self->raw_pts);
	glist_clear(self->pts);

	self->need_plan = true;
	g_model->move->need_sched = true;
	self->next_i = 0;
}

static void plan_next_stage(plan_t *self)
{
	if (!self->raw_pts->count) {
		plan_resched(self);
		return;
	}

	glist_clear(self->pts);
	fsm_set(self->fsm,
			((dpt_t *)glist_get(self->raw_pts, 0))->up ? LFSMS_ASC : LFSMS_TRV);

	self->need_plan = true;
	self->next_i = 0;
}

static int act_ascending(fsm_t *fsm)
{
	const double TOP_TM_C = 0.4;
	const double MID_PT_HT_C = 0.75;
	const double END_OF_C = 0.5;
	const double MID_OF_C = 0.25;

	leg_t *l = (leg_t *)fsm->priv;
	plan_t *p = &l->plan;

	if (p->need_plan) {
		dpt_t *start, *end, *mid;
		double dx, dy, step_dist, end_ts, *walk_h;

		DBG("%s: Ascending\n", l->name);
		l->bal = false;
		p->up = true;

		walk_h = g_model->sens->walk_h;

		dpt_t *pre = dpt_clone(dpt_hist_get(p, -1));
		pre->vps = 1.0;

		start = dpt_clone(dpt_hist_get(p, 0));
		start->vps = 1.0;

		dpt_t *target = glist_get(p->raw_pts, 0);
		end_ts = start->ts + TOP_TM_C * (target->ts - start->ts);
		dx = *target->x - *start->x;
		dy = *target->y - *start->y;
		step_dist = sqrtf(dx * dx + dy * dy);

		walk_h[l->idx] =
			get_walk_height(step_dist, g_model->sens->abs_std_leg_h);

		double end_pos[3] = { *start->x + END_OF_C * (*target->x - *start->x),
							  *start->y + END_OF_C * (*target->y - *start->y),
							  walk_h[l->idx] };

		end = dpt_new(end_pos, end_ts, 1.0, false);

		dpt_t *post = dpt_clone(end);
		post->ts += 1.0;

		double mid_pos[3] = { *start->x + MID_OF_C * (*target->x - *start->x),
							  *start->y + MID_OF_C * (*target->y - *start->y),
							  *start->z +
								  MID_PT_HT_C * (walk_h[l->idx] - *start->z) };

		mid = dpt_new(mid_pos, (start->ts + end_ts) / 2, 1.0, false);

		glist_push(p->pts, pre);
		glist_push(p->pts, start);
		glist_push(p->pts, mid);
		glist_push(p->pts, end);
		glist_push(p->pts, post);

		plan_make_steps(p);
		p->next_i = 2;
		p->need_plan = false;
	}

	if (p->next_i == p->n_steps) {
		/* TODO: commenting these may break something <26-02-23, yourname> */
		/* double wh = self.info.walk_h*/
		fsm_next(fsm, LFSMA_END);
		glist_clear(p->pts);
		p->need_plan = true;
		p->next_i = 0;
		/*self.info.walk_h = wh*/
		fsm_execute(fsm);
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

	if (p->need_plan) {
		dpt_t *start, *end, *mid, *pre, *post;
		DBG("%s: Descending\n", l->name);
		l->bal = false;
		p->up = true;

		pre = dpt_clone(dpt_hist_get(p, -1));

		start = dpt_clone(dpt_hist_get(p, 0));
		end = glist_remove(p->raw_pts, 0);

		post = dpt_clone(end);
		post->ts += 1.0;

		double mid_ts = roundf(start->ts + TM_C * (end->ts - start->ts));

		double mid_pos[3] = { *start->x + OF_C * (*end->x - *start->x),
							  *start->y + OF_C * (*end->y - *start->y),
							  *end->z + HT_C * (g_model->sens->walk_h[l->idx] -
												*end->z) };

		mid = dpt_new(mid_pos, mid_ts, 1.0, false);

		glist_push(p->pts, pre);
		glist_push(p->pts, start);
		glist_push(p->pts, mid);
		glist_push(p->pts, end);
		glist_push(p->pts, post);

		plan_make_steps(p);
		p->next_i = 2;
		p->need_plan = false;
	}

	if (g_model->sens->touch_f->data[l->idx] > 0.0) {
		plan_resched(p);
		fsm_next(fsm, LFSMA_HIT);
		fsm_execute(fsm);
	} else if (p->next_i == p->n_steps) {
		fsm_next(fsm, LFSMA_END);
		plan_next_stage(p);
		fsm_execute(fsm);
	} else {
		plan_next(p, false);
	}

	return 0;
}

static int act_traversing(fsm_t *fsm)
{
	leg_t *l = (leg_t *)fsm->priv;
	plan_t *p = &l->plan;

	if (p->need_plan) {
		DBG("%s: Traversing\n", l->name);
		l->bal = true;
		p->up = false;

		int trv_cnt = 0, skip_cnt = 0;

		for (size_t i = 0; i < p->raw_pts->count; i++) {
			const dpt_t *d = glist_get(p->raw_pts, i);
			if (d->up) {
				break;
			}
			trv_cnt++;
		}

		for (int i = MIN(-MIN_N_INTERP + trv_cnt + 1, -1); i <= 0; i++) {
			dpt_t *d = dpt_hist_get(p, i);
			dpt_t *dc = dpt_clone(d);
			glist_push(p->pts, dc);
			skip_cnt++;
		}

		if (glist_move_n_to(p->raw_pts, p->pts, 0, p->pts->count, trv_cnt)) {
			ERR(ERR_INVALID_INPUT);
			return 1;
		}

		plan_make_steps(p);
		p->next_i = skip_cnt;
		p->need_plan = false;
	}

	if (p->next_i == p->n_steps) {
		fsm_next(fsm, LFSMA_END);
		plan_next_stage(p);
		fsm_execute(fsm);
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
		p->up = false;
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
		p->up = false;
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

dpt_t *dpt_new(const double pos[3], double ts, double vps, bool is_up)
{
	dpt_t *p;

	p = calloc(1, sizeof(dpt_t));
	if (!p) {
		FATAL(ERR_MALLOC_FAIL);
	}

	p->pos = vector_from_array(3, pos);

	p->x = gsl_vector_ptr(p->pos, 0);
	p->y = gsl_vector_ptr(p->pos, 1);
	p->z = gsl_vector_ptr(p->pos, 2);

	p->ts = ts;
	p->vps = vps;
	p->up = is_up;

	return p;
}

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
	p->up = d->up;

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
		p->hist[i] = dpt_new(l->def_pos->data, i, 1.0, false);
	}
	p->h_idx = DPT_CBUF_SIZE - 1;

	st_p = p->hist[p->h_idx];

	p->need_plan = true;
	p->cur = st_p;
	p->raw_pts = glist_new(8);
	glist_set_free_cb(p->raw_pts, (void (*)(void *))dpt_free);
	p->pts = glist_new(16);
	glist_set_free_cb(p->pts, (void (*)(void *))dpt_free);
	p->adj = vector_calloc(3);

	p->dists = block_calloc(START_N_STEPS);
	p->vels = block_calloc(START_N_STEPS);

	p->acc = gsl_interp_accel_alloc();

	p->fsm =
		fsm_new((unsigned *)lfsm_states, lfsm_funcs, LFSMS_MAX, LFSMA_MAX, l);

	if (!p->cur || !p->raw_pts || !p->pts || !p->acc || !p->fsm) {
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

	gsl_vector_free(p->adj);

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
}

void plan_make_movement(plan_t *self, glist_t *lst)
{
	double init_ts = dpt_hist_get(self, 0)->ts;
	glist_clear(self->raw_pts);

	glist_foreach (dpt_t *e, lst) {
		e->ts += init_ts;
		e->vps = 1.0;
	}

	glist_copy_to(lst, self->raw_pts);
	fsm_set(self->fsm,
			((dpt_t *)glist_get(lst, 0))->up ? LFSMS_ASC : LFSMS_TRV);
	plan_next_stage(self);
}
