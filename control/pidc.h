/**
 * @file pidc.h
 * @brief PID controler
 * @author Linas Perkauskas
 * @date 2023-03-02
 */

#pragma once

typedef struct pidc {
	float p;
	float i;
	float d;
	float dt;
	float it;
	float perr;
} pidc_t;

void pidc_set(pidc_t *self, float p, float i, float d, float dt);

float pidc_eval(pidc_t *self, float cur, float dest);
