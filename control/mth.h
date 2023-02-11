/**
 * @file mth.h
 * @brief Misc math stuff
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#pragma once

#include <math.h>

#define MIN(a, b) ((a) <= (b) ? (a) : (b))
#define MAX(a, b) ((a) >= (b) ? (a) : (b))
#define XOR(a, b) ((a) ^ (b))

float area(float *p1, float *p2, float *p3);

inline float bound_data(float dt, float lo, float hi)
{
	if (dt < lo) {
		return lo;
	} else if (dt > hi) {
		return hi;
	} else {
		return dt;
	}
}

