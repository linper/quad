/**
 * @file fsm.h
 * @brief Finite state machine
 * @author Linas Perkauskas
 * @date 2023-02-25
 */

#pragma once

#include <stdio.h>
#include <stdint.h>

typedef struct fsm fsm_t;

typedef int (*fsm_func)(struct fsm *);

/**
 * @brief Struct that describes FSM. 
 */
typedef struct fsm {
	void *priv; ///< 			Private data to be passed to fsm functions
	uint32_t n_act; ///< 		Number of actions
	uint32_t n_state; ///< 		Number of states
	uint32_t cur; ///< 			Current sate
	uint32_t *s_map; ///< 		State matrix (n_act x n_state). (Static)
	fsm_func *f_map; ///< 		Func map to be called for each state. (Static)
} fsm_t;

/**
 * @brief Allocates new FSM instance.
 * @param[in] 	*s_map 		State matrix (n_state x n_act). Default 
 * state must be 0. States and actionss nust be consequtive.
 * @param[in] 	*f_map 		Function map. Must be length of `n_state`.
 * @param[in] 	n_state 	Total number of states.
 * @param[in] 	n_act 		Total number of actions.
 * @param[in] 	*priv 		Private data to be used in fsm state-functions.
 */
fsm_t *fsm_new(uint32_t *s_map, fsm_func *f_map, uint32_t n_state,
			   uint32_t n_act, void *priv);

void fsm_free(fsm_t *self);

void fsm_next(fsm_t *self, uint32_t act);

void fsm_set(fsm_t *self, uint32_t state);

void fsm_reset(fsm_t *self);

int fsm_execute(fsm_t *self);
