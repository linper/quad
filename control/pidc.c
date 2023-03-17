/**
 * @file pidc.c
 * @brief PID controler
 * @author Linas Perkauskas
 * @date 2023-03-02
 */

#include <stdlib.h>

#include <log.h>
#include <pidc.h>

void pidc_set(pidc_t *self, double p, double i, double d, double dt)
{
	if (!self) {
		FATAL(ERR_INVALID_INPUT);
		return;
	}

	self->p = p;
	self->i = i;
	self->d = d;
	self->dt = dt;
	self->it = 0.0;
	self->perr = 0.0;
}

double pidc_eval(pidc_t *self, double cur, double dest)
{
	double err, P, I, D, val;

	err = dest - cur;
	P = self->p * err;
	I = self->it + self->i * err * self->dt;
	D = self->d * (err - self->perr) / self->dt;

	self->it = I;
	self->perr = err;

	val = P + I + D;

	return val;
}
