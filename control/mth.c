/**
 * @file mth.c
 * @brief Misc math stuff
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "mth.h"

float area(float *p1, float *p2, float *p3)
{
	return fabs((p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) +
				 p3[0] * (p1[1] - p2[1])) /
				2.0);
}

