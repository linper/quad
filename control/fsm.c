/**
 * @file fsm.c
 * @brief Finite state machine
 * @author Linas Perkauskas
 * @date 2023-02-25
 */

#include <stdlib.h>

#include "log.h"
#include "fsm.h"

fsm_t *fsm_new(uint32_t *s_map, fsm_func *f_map, uint32_t n_state,
			   uint32_t n_act, void *priv)
{
	if (!s_map || !f_map || !n_state || !n_act) {
		return NULL;
	}

	fsm_t *f = calloc(1, sizeof(fsm_t));
	if (!f) {
		FATAL(ERR_MALLOC_FAIL);
	}

	f->f_map = f_map;
	f->s_map = s_map;
	f->n_act = n_act;
	f->n_state = n_state;
	f->priv = priv;

	return f;
}

void fsm_free(fsm_t *self)
{
	if (!self) {
		return;
	}

	free(self);
}

void fsm_next(fsm_t *self, uint32_t act)
{
	if (!self || act >= self->n_act) {
		return;
	}

	self->cur = self->s_map[self->n_state * act + self->cur];

	if (self->cur >= self->n_state) {
		self->cur = 0;
	}
}

void fsm_set(fsm_t *self, uint32_t state)
{
	if (!self || state >= self->n_state) {
		return;
	}

	self->cur = state;
}

void fsm_reset(fsm_t *self)
{
	if (!self) {
		return;
	}

	self->cur = 0;
}

int fsm_execute(fsm_t *self)
{
	if (!self) {
		return 1;
	}

	int res = self->f_map[self->cur](self);

	return !!res;
}

