/**
 * @file plan.h
 * @brief Individual leg movement plan
 * @author Linas Perkauskas
 * @date 2023-02-25
 */

#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "glist.h"
#include "matrix.h"
#include "stash.h"

struct leg_plan;

typedef struct dst_pt {
	uint32_t ts;
	float vps;
	mat_t *pos;
	mat_t *vel;
	float *x;
	float *y;
	float *z;
	float *dx;
	float *dy;
	float *dz;
	stash_t *stash;
} dpt_t;

dpt_t *dpt_new(const float pos[3], const float vel[3], uint32_t ts, float vps,
			   stash_t *stash);

static inline void dpt_set_pos(dpt_t *p, float pos[3])
{
	if (p)
		mat_update_array(p->pos, 3, pos);
}

static inline void dpt_set_vel(dpt_t *p, float vel[3])
{
	if (p)
		mat_update_array(p->vel, 3, vel);
}

typedef int (*speed_func)(struct leg_plan *, dpt_t *, dpt_t *, mat_t **,
						  mat_t **);

typedef struct leg_plan {
	bool need_plan; ///< 		Is replan needed
	dpt_t *target; ///< 		Current target dest point ptr
	dpt_t *cur; ///< 			Current step dest point ptr
	uint32_t cur_i; ///< 		Current step dest point idx

	glist_t *raw_pts; ///< 		Input pts to interpolate
	glist_t *pts; ///< 			Intermediate pts derived from raw_pts
	glist_t *steps; ///< 		Interpolated pts
	glist_t *targets; ///< 		End interpolation points

	mat_t *adj; ///< 			Offset from path (due to alancing)
	stash_t *stash; ///< 		Plan stash
	speed_func vfunc;
} plan_t;

plan_t *plan_new(plan_t *p);

void plan_free(plan_t *p);

void plan_reset(plan_t *self);

void plan_adjust(plan_t *self, mat_t *adj);

void plan_compensate(plan_t *self);

void plan_step_one(plan_t *self);

void plan_step_zero(plan_t *self);

void plan_make_steps(plan_t *self);

