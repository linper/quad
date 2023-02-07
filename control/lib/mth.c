

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "mth.h"

void prompt(int num)
{
	printf("The number %d was entered", num);
}

float area(float *p1, float *p2, float *p3)
{
	return fabs((p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) +
				 p3[0] * (p1[1] - p2[1])) /
				2.0);
}

bool xor (bool a, bool b) { return a ^ b; }
