/**
 * @file plan.c
 * @brief Individual leg movement plan
 * @author Linas Perkauskas
 * @date 2023-02-25
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdlib.h>
#include <math.h>

#include "log.h"
#include <fsm.h>
#include <plan.h>
#include <balance.h>
#include <glist.h>
#include <matrix.h>
#include <model.h>
#include <stash.h>

/**
* container_of - cast a member of a structure out to the containing structure
* @ptr: the pointer to the member.
* @type: the type of the container struct this is embedded in.
* @member: the name of the member within the struct.
*
*/
#define container_of(ptr, type, member)                                        \
	({                                                                         \
		const typeof(((type *)0)->member) *__mptr = (ptr);                     \
		(type *)((char *)__mptr - offsetof(type, member));                     \
	})

enum lfsm_act {
	LFSMA_NO_ACT,
	LFSMA_END,
	LFSMA_HIT,
	LFSMA_STOP,
	LFSMA_MAX,
};

enum lfsm_state {
	LFSMS_STP,
	LFSMS_ASC,
	LFSMS_DSC,
	LFSMS_TRV,
	LFSMS_PND,
	LFSMS_MAX,
};

static int fill_diffs(plan_t *p, glist_t *lst)
{
	mat_t *time_steps, *data_x, *data_y, *data_z, *dx, *dy, *dz;
	dpt_t *d;
	float vel[3];

	time_steps = mat_create_ext(1, lst->count, false, p->stash);
	data_x = mat_create_ext(1, lst->count, false, p->stash);
	data_y = mat_create_ext(1, lst->count, false, p->stash);
	data_z = mat_create_ext(1, lst->count, false, p->stash);

	for (size_t i = 0; i < lst->count; i++) {
		d = lst->array[i];
		time_steps->arr[i] = d->ts;
		data_x->arr[i] = d->pos->arr[0];
		data_y->arr[i] = d->pos->arr[1];
		data_z->arr[i] = d->pos->arr[2];
	}

	dx = mat_akima(time_steps, data_x);
	dy = mat_akima(time_steps, data_y);
	dz = mat_akima(time_steps, data_z);

	for (size_t i = 0; i < lst->count; i++) {
		d = lst->array[i];
		memcpy(vel, d->vel, 3 * sizeof(float));

		if (vel[0] == NAN) {
			vel[0] = dx->arr[i];
		}

		if (vel[1] == NAN) {
			vel[1] = dy->arr[i];
		}

		if (vel[2] == NAN) {
			vel[2] = dz->arr[i];
		}

		dpt_set_vel(d, vel);
	}

	return 0;
}

static float get_d2_speed(dpt_t *d1, dpt_t *d2)
{
	float vel, S;
	mat_t *d;

	d = mat_sub_n(d2->pos, d1->pos);
	S = mat_length(d);
	vel = S / (d2->ts - d1->ts);

	return vel;
}

static int gradual_vfunc(plan_t *p, dpt_t *d1, dpt_t *d2, mat_t **dists_p,
						 mat_t **vels_ps_p)
{
	mat_t *dists, *vels_ps;
	float tm_sum = 0.0, prev_v = 0.0;

	if (!p || !d1 || !d2 || !dists_p || !vels_ps_p) {
		return 1;
	}

	vels_ps = mat_linspace(d1->vps, d2->vps, d2->ts - d1->ts + 1, p->stash);
	dists = mat_create_ext(1, d2->ts - d1->ts + 1, true, p->stash);

	for (size_t i = 0; i < vels_ps->sz; i++) {
		tm_sum += (vels_ps->arr[i] + prev_v) / 2;
		prev_v = vels_ps->arr[i] + prev_v;
		dists->arr[i] = tm_sum;
	}

	*dists_p = dists;
	*vels_ps_p = vels_ps;

	return 0;
}

static int variable_vfunc(plan_t *p, dpt_t *d1, dpt_t *d2, mat_t **dists_p,
						  mat_t **vels_ps_p)
{
	mat_t *dists, *vels_ps, *tmp_m;
	float S_dif, sp_top, inc, sum = 0.0, tm_sum = 0.0, prev_v = 0.0;
	int t_dt;

	if (!p || !d1 || !d2 || !dists_p || !vels_ps_p) {
		return 1;
	}

	gradual_vfunc(p, d1, d2, &dists, &vels_ps);
	tmp_m = mat_sub_n(d2->pos, d1->pos);

	S_dif = mat_length(tmp_m);
	S_dif -= dists->arr[dists->sz - 1];
	mat_free(tmp_m);

	t_dt = d2->ts - d1->ts;
	sp_top = 2 * S_dif / t_dt;
	inc = sp_top / (vels_ps->sz - 1);

	for (size_t i = 0; i < vels_ps->sz; i++) {
		vels_ps->arr[i] += sum;
		if (i > vels_ps->sz / 2) {
			sum -= inc;
		} else {
			sum += inc;
		}
	}

	for (size_t i = 0; i < vels_ps->sz; i++) {
		tm_sum += (vels_ps->arr[i] + prev_v) / 2;
		prev_v = vels_ps->arr[i] + prev_v;
		dists->arr[i] += tm_sum;
	}

	*dists_p = dists;
	*vels_ps_p = vels_ps;

	return 0;
}

static int act_ascending(fsm_t *fsm)
{
	const float TOP_TM_C = 0.4;
	const float MID_PT_HT_C = 0.75;
	const float END_OF_C = 0.5;
	const float MID_OF_C = 0.25;

	leg_t *l = (leg_t *)fsm->priv;
	plan_t *p = &l->plan;

	float dx, dy, step_dist, end_ts, end_vel[3] = { NAN, NAN, 0.0 },
									 unset[3] = { NAN, NAN, NAN }, *walk_h;
	dpt_t *start, *end, *mid;

	if (p->need_plan) {
		DBG("%s: Ascending\n", l->name);
		l->bal = false;
		walk_h = g_model->sens->walk_h;

		start = dpt_new(p->cur->pos->arr, p->cur->vel->arr, 0, 0.0, p->stash);
		end_ts = (int)floorf(TOP_TM_C * (p->target->ts - start->ts));
		dx = *p->target->x - *start->x;
		dy = *p->target->y - *start->y;
		step_dist = sqrtf(dx * dx + dy * dy);

		walk_h[l->idx] =
			get_walk_height(step_dist, g_model->sens->abs_std_leg_h);

		float end_pos[3] = { *start->x + END_OF_C * (*p->target->x - *start->x),
							 *start->y + END_OF_C * (*p->target->y - *start->y),
							 walk_h[l->idx] };
		end = dpt_new(end_pos, end_vel, p->target->ts, 0.0, p->stash);

		float mid_pos[3] = { *start->x + MID_OF_C * (*p->target->x - *start->x),
							 *start->y + MID_OF_C * (*p->target->y - *start->y),
							 *start->z +
								 MID_PT_HT_C * (walk_h[l->idx] - *start->z) };

		mid = dpt_new(mid_pos, unset, (int)end_ts / 2, 0.0, p->stash);

		start->vps = 0.0; // maybe not needed
		mid->vps = get_d2_speed(start, mid);
		end->vps = get_d2_speed(mid, end);

		glist_push(p->pts, start);
		glist_push(p->pts, mid);
		glist_push(p->pts, end);

		fill_diffs(p, p->pts);

		plan_make_steps(p);

		p->need_plan = false;
	}

	if (p->cur_i <= p->steps->count) {
		plan_step_one(p);
	} else {
		/* TODO: commenting these may break something <26-02-23, yourname> */
		/* float wh = self.info.walk_h*/
		plan_reset(p);
		/*self.info.walk_h = wh*/
	}

	return 0;
}

static int act_descending(fsm_t *fsm)
{
	const float HT_C = 0.2;
	const float OF_C = 0.96;
	const float TM_C = 0.7;

	leg_t *l = (leg_t *)fsm->priv;
	plan_t *p = &l->plan;
	float mid_ts, end_vel[3] = { 0.0, NAN, NAN }, unset[3] = { NAN, NAN, NAN };
	dpt_t *start, *end, *mid;

	if (p->need_plan) {
		DBG("%s: Descending\n", l->name);
		l->bal = false;

		start = dpt_new(p->cur->pos->arr, p->cur->vel->arr, p->cur->ts,
						p->cur->vps, p->stash);
		end =
			dpt_new(p->target->pos->arr, end_vel, p->target->ts, 0.0, p->stash);

		mid_ts = roundf(start->ts + TM_C * (p->target->ts - start->ts));

		float mid_pos[3] = { *start->x + OF_C * (*p->target->x - *start->x),
							 *start->y + OF_C * (*p->target->y - *start->y),
							 *end->z + HT_C * (g_model->sens->walk_h[l->idx] -
											   *end->z) };

		mid = dpt_new(mid_pos, unset, mid_ts, 0.0, p->stash);
		mid->vps = get_d2_speed(start, mid);

		glist_push(p->pts, start);
		glist_push(p->pts, mid);
		glist_push(p->pts, end);

		p->vfunc = &variable_vfunc;

		fill_diffs(p, p->pts);

		plan_make_steps(p);

		p->need_plan = false;
	}

	if (g_model->sens->touch_f->arr[l->idx] > 0.0) {
		plan_reset(p);
		fsm_next(fsm, LFSMA_HIT);
	} else if (p->cur_i <= p->steps->count) {
		plan_step_one(p);
	} else {
		plan_reset(p);
		fsm_next(fsm, LFSMA_END);
	}

	return 0;
}

static int act_traversing(fsm_t *fsm)
{
	leg_t *l = (leg_t *)fsm->priv;
	plan_t *p = &l->plan;
	float empty[3] = { 0 };

	if (p->need_plan) {
		DBG("%s: Traversing\n", l->name);
		l->bal = true;

		dpt_t *start =
			dpt_new(p->cur->pos->arr, empty, 0, p->cur->vps, p->stash);
		dpt_t *last = (dpt_t *)glist_get(p->raw_pts, p->raw_pts->count - 1);
		mat_fill(last->vel, 0.0);

		glist_push(p->pts, start);
		glist_extend(p->pts, p->raw_pts);

		fill_diffs(p, p->pts);
		plan_make_steps(p);

		p->need_plan = false;
	}

	if (p->cur_i == p->steps->count) {
		plan_reset(p);
		fsm_next(fsm, LFSMA_END);
	} else {
		mat_t *balance = mat_get_row(g_model->sens->balance, l->idx);
		plan_adjust(p, balance);
		mat_free(balance);
		plan_step_zero(p);

		mat_free(balance);
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

	mat_t *balance = mat_get_row(g_model->sens->balance, l->idx);
	plan_adjust(p, balance);
	mat_free(balance);
	plan_step_zero(p);

	mat_free(balance);

	return 0;
}

static fsm_func lfsm_funcs[LFSMS_MAX] = {
	[LFSMS_STP] = act_stoped,	  [LFSMS_ASC] = act_ascending,
	[LFSMS_DSC] = act_descending, [LFSMS_TRV] = act_traversing,
	[LFSMS_PND] = act_pending,
};

static uint32_t lfsm_states[LFSMA_MAX][LFSMS_MAX] = {
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

dpt_t *dpt_new(const float pos[3], const float vel[3], unsigned ts, float vps,
			   stash_t *stash)
{
	dpt_t *p;

	if (stash) {
		p = STASH_ALLOC(stash, sizeof(dpt_t));
	} else {
		p = calloc(1, sizeof(dpt_t));
		if (!p) {
			FATAL(ERR_MALLOC_FAIL);
		}
	}

	p->pos = mat_from_array_ext(1, 3, pos, stash);
	p->vel = mat_from_array_ext(1, 3, vel, stash);
	p->x = &p->pos->arr[0];
	p->y = &p->pos->arr[1];
	p->z = &p->pos->arr[2];
	p->dx = &p->vel->arr[0];
	p->dy = &p->vel->arr[1];
	p->dz = &p->vel->arr[2];

	p->ts = ts;
	p->vps = vps;
	p->stash = stash;

	return p;
}

plan_t *plan_new(plan_t *p)
{
	dpt_t *st_p;
	stash_t *s;
	float empty[3] = { 0 };
	leg_t *l = container_of(p, leg_t, plan);

	s = stash_new();
	st_p = dpt_new(l->pos->arr, empty, 0, 0.0, s);

	p->stash = s;
	p->need_plan = true;
	p->cur = st_p;
	p->target = st_p;
	p->vfunc = NULL;
	p->raw_pts = glist_new_ext(8, s, GLF_ELEM_ST);
	p->pts = glist_new_ext(16, s, GLF_ELEM_ST);
	p->steps = glist_new_ext(2048, s, GLF_ELEM_ST);
	p->targets = glist_new_ext(8, s, GLF_ELEM_ST);
	p->adj = mat_from_array_ext(1, 3, empty, s);
	p->stash = s;

	glist_push(p->targets, st_p);
	glist_push(p->targets, st_p);
	glist_push(p->targets, st_p);

	l->fsm =
		fsm_new((uint32_t *)lfsm_states, lfsm_funcs, LFSMS_MAX, LFSMA_MAX, l);

	return p;
}

void plan_free(plan_t *p)
{
	if (!p)
		return;

	glist_free(p->raw_pts);
	glist_free(p->pts);
	glist_free(p->targets);
	glist_free(p->steps);

	mat_free(p->adj);

	stash_free(p->stash);
}

void plan_reset(plan_t *self)
{
	glist_clear_shallow(self->raw_pts);
	glist_clear_shallow(self->pts);
	glist_clear_shallow(self->steps);
	stash_clear(self->stash);

	self->need_plan = true;
	self->vfunc = NULL;
	self->cur_i = 0;
}

void plan_adjust(plan_t *self, mat_t *adj)
{
	mat_t *new_adj = mat_add_n(self->adj, adj);

	float cur_z = *self->cur->z;
	float d_adj = new_adj->arr[2] + cur_z;

	if (d_adj < g_model->max_dip) {
		new_adj->arr[2] = g_model->max_dip - cur_z;
	} else {
		new_adj->arr[2] = g_model->min_dip - cur_z;
	}

	self->adj = new_adj;
}

void plan_compensate(plan_t *self)
{
	mat_add_n(self->cur->pos, self->adj);
	mat_fill(self->adj, 0.0);
}

void plan_step_one(plan_t *self)
{
	leg_t *l = container_of(self, leg_t, plan);
	self->cur_i++;
	self->cur = glist_get(self->steps, self->cur_i);
	mat_fill(l->pos, 0.0);
	mat_add(l->pos, self->cur->pos);
	mat_add(l->pos, self->adj);
}

void plan_step_zero(plan_t *self)
{
	leg_t *l = container_of(self, leg_t, plan);
	mat_fill(l->pos, 0.0);
	mat_add(l->pos, self->cur->pos);
	mat_add(l->pos, self->adj);
}

void plan_make_steps(plan_t *self)
{
	(void)self;
	/*mat_t *tpts, *data_x, *data_y, *data_z, *data_dx, *data_dy, *data_dz, *dx,*/
		/**dy, *dz, *ts_connected;*/
	/*dpt_t *d;*/
	/*float vel[3];*/

	/*tpts = mat_create_ext(1, self->pts->count, false, self->stash);*/
	/*data_x = mat_create_ext(1, self->pts->count, false, self->stash);*/
	/*data_y = mat_create_ext(1, self->pts->count, false, self->stash);*/
	/*data_z = mat_create_ext(1, self->pts->count, false, self->stash);*/
	/*data_dx = mat_create_ext(1, self->pts->count, false, self->stash);*/
	/*data_dy = mat_create_ext(1, self->pts->count, false, self->stash);*/
	/*data_dz = mat_create_ext(1, self->pts->count, false, self->stash);*/

	/*for (size_t i = 0; i < self->pts->count; i++) {*/
		/*d = self->pts->array[i];*/
		/*tpts->arr[i] = d->ts;*/
		/*data_x->arr[i] = d->pos->arr[0];*/
		/*data_y->arr[i] = d->pos->arr[1];*/
		/*data_z->arr[i] = d->pos->arr[2];*/
		/*data_x->arr[i] = d->vel->arr[0];*/
		/*data_y->arr[i] = d->vel->arr[1];*/
		/*data_z->arr[i] = d->vel->arr[2];*/
	/*}*/

	/*ts_connected =*/
		/*mat_linspace(tpts->arr[0] + 1, tpts->arr[tpts->sz - 1],*/
					 /*(uint32_t)(tpts->arr[tpts->sz - 1] - tpts->arr[0]),*/
					 /*self->stash);*/

	/*dx = mat_akima(time_steps, data_x);*/
	/*dy = mat_akima(time_steps, data_y);*/
	/*dz = mat_akima(time_steps, data_z);*/
}
/*
def plan_steps(points, speed_func):
    tpts = np.array([i.ts for i in points])
    data_x = np.array([i.pos[0] for i in points])
    data_y = np.array([i.pos[1] for i in points])
    data_z = np.array([i.pos[2] for i in points])
    data_dx = np.array([i.vel[0] for i in points])
    data_dy = np.array([i.vel[1] for i in points])
    data_dz = np.array([i.vel[2] for i in points])

    ts_connected = np.array(
        list(range(tpts[0] + 1, tpts[-1] + 1)))
    time_steps, time_steps_chunked, vels_ps = connect_times(
        points, speed_func)

    spline_x = connect_splines(data_x, data_dx, time_steps_chunked)
    spline_y = connect_splines(data_y, data_dy, time_steps_chunked)
    spline_z = connect_splines(data_z, data_dz, time_steps_chunked)

    dx = akima(np.array(time_steps), np.array(spline_x))
    dy = akima(np.array(time_steps), np.array(spline_y))
    dz = akima(np.array(time_steps), np.array(spline_z))

    steps = [DestPoint(np.array([x, y, z]), np.array([dx_, dy_, dz_]), ts=ts, vel_ps=v_ps) for x, y, z, ts, dx_, dy_, dz_, v_ps in zip(
        spline_x, spline_y, spline_z, ts_connected, dx, dy, dz, vels_ps)]
    steps.reverse()
    # print(f"steps len:{len(steps)}")
    return steps
*/
