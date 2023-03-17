/**
 * @file plan.h
 * @brief Individual leg movement plan
 * @author Linas Perkauskas
 * @date 2023-02-25
 */

#pragma once

#include <gsl/gsl_vector_double.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "glist.h"
#include "mth.h"
#include "stash.h"

struct leg_plan;

typedef struct dst_pt {
	uint32_t ts;
	double vps;
	gsl_vector *pos;
	gsl_vector *vel;
	double *x;
	double *y;
	double *z;
	double *dx;
	double *dy;
	double *dz;
	stash_t *stash;
} dpt_t;

dpt_t *dpt_new(const double pos[3], const double vel[3], uint32_t ts,
			   double vps, stash_t *stash);

static inline void dpt_set_pos(dpt_t *p, double pos[3])
{
	if (p)
		vector_update_array(p->pos, 3, pos);
}

static inline void dpt_set_vel(dpt_t *p, double vel[3])
{
	if (p)
		vector_update_array(p->vel, 3, vel);
}

typedef int (*speed_func)(struct leg_plan *, dpt_t *, dpt_t *, gsl_vector **,
						  gsl_vector **);

typedef struct leg_plan {
	bool need_plan; ///< 		Is replan needed
	dpt_t *target; ///< 		Current target dest point ptr
	dpt_t *cur; ///< 			Current step dest point ptr
	uint32_t cur_i; ///< 		Current step dest point idx

	glist_t *raw_pts; ///< 		Input pts to interpolate
	glist_t *pts; ///< 			Intermediate pts derived from raw_pts
	glist_t *steps; ///< 		Interpolated pts
	glist_t *targets; ///< 		End interpolation points

	gsl_vector *adj; ///< 			Offset from path (due to balancing)
	stash_t *stash; ///< 		Plan stash
	speed_func vfunc;
} plan_t;

plan_t *plan_new(plan_t *p);

void plan_free(plan_t *p);

void plan_reset(plan_t *self);

void plan_adjust(plan_t *self, gsl_vector *adj);

void plan_compensate(plan_t *self);

void plan_step_one(plan_t *self);

void plan_step_zero(plan_t *self);

void plan_make_steps(plan_t *self);

