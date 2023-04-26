/**
 * @file plan.h
 * @brief Individual leg movement plan
 * @author Linas Perkauskas
 * @date 2023-02-25
 */

#pragma once

#include <gsl/gsl_interp.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <gsl/gsl_math.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_vector_double.h>

#include "glist.h"
#include "fsm.h"
#include "mth.h"
//#include "stash.h"

#define N_INTERP 16
#define DPT_CBUF_SIZE 8

enum lfsm_state {
	LFSMS_STP,
	LFSMS_ASC,
	LFSMS_DSC,
	LFSMS_TRV,
	LFSMS_PND,
	LFSMS_MAX,
};

struct leg_plan;

typedef struct dst_pt {
	double ts;
	double vps;
	gsl_vector *pos;
	//gsl_vector *vel;
	double *x;
	double *y;
	double *z;
	//double *dx;
	//double *dy;
	//double *dz;
	//stash_t *stash;
} dpt_t;

void dpt_free(dpt_t *d);
dpt_t *dpt_new(const double pos[3], double ts, double vps);
//dpt_t *dpt_new(const double pos[3], const double vel[3], size_t ts, double vps);
dpt_t *dpt_new2(gsl_vector *pos, double ts, double vps);
dpt_t *dpt_new3(gsl_vector *pos, double ts, double vps);
//dpt_t *dpt_from_spl(struct leg_plan *p, double ts, double vps);
dpt_t *dpt_clone(dpt_t *d);
//dpt_t *dpt_hist_step(struct leg_plan *p);
//dpt_t *dpt_hist_get(struct leg_plan *p, int idx);

static inline void dpt_set_pos(dpt_t *p, double pos[3])
{
	if (p)
		vector_update_array(p->pos, 3, pos);
}

//static inline void dpt_set_vel(dpt_t *p, double vel[3])
//{
//if (p)
//vector_update_array(p->vel, 3, vel);
//}

typedef int (*speed_func)(struct leg_plan *, dpt_t *, dpt_t *, gsl_vector **,
						  gsl_vector **);

typedef struct leg_plan {
	fsm_t *fsm; ///< 					FSM of the leg.
	bool need_plan; ///< 				Is replan needed
	double cur_ts; ///< 				Current time step
	//double *ts_arr;
	//double *vels_arr;
	dpt_t *target; ///< 				Current target dest point ptr
	dpt_t *cur; ///< 					Current step dest point ptr
	dpt_t *hist[DPT_CBUF_SIZE]; ///< 	Steps history circ. buf
	size_t h_idx; ///< 					History index
	size_t next_i; ///< 				Next step dest point idx

	glist_t *raw_pts; ///< 				Input pts to interpolate
	glist_t *pts; ///< 					Intermediate pts derived from raw_pts
	//glist_t *steps; ///< 				Interpolated pts
	//glist_t *targets; ///< 				End interpolation points

	gsl_spline *x_spl;
	gsl_spline *y_spl;
	gsl_spline *z_spl;
	gsl_interp_accel *acc;
	size_t n_steps;
	gsl_block *dists;
	gsl_block *vels;

	gsl_vector *adj; ///< 		Offset from path (due to balancing)
} plan_t;

int plan_new(plan_t *p);

void plan_free(plan_t *p);

//void plan_reset(plan_t *self);

//void plan_adjust(plan_t *self, gsl_vector *adj);

//void plan_compensate(plan_t *self);

void plan_step(plan_t *self);

//void plan_next(plan_t *self, bool repeat);

//void plan_make_steps(plan_t *self);

//void plan_blob_check(plan_t *self, size_t n);

void plan_make_movement(plan_t *self, double *pts, size_t n_pts, bool do_lift);
