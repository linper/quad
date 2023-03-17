/**
 * @file pidc.h
 * @brief PID controler
 * @author Linas Perkauskas
 * @date 2023-03-02
 */

#pragma once

typedef struct pidc {
	double p;
	double i;
	double d;
	double dt;
	double it;
	double perr;
} pidc_t;

void pidc_set(pidc_t *self, double p, double i, double d, double dt);

double pidc_eval(pidc_t *self, double cur, double dest);
