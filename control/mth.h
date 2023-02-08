/**
 * @file mth.h
 * @brief Misc math stuff
 * @author Linas Perkauskas
 * @date 2023-02-04
 */

#ifndef MTH_H
#define MTH_H

#define MIN(a, b) ((a) <= (b) ? (a) : (b))
#define MAX(a, b) ((a) >= (b) ? (a) : (b))
#define XOR(a, b) ((a) ^ (b))

float area(float *p1, float *p2, float *p3);

#endif // MTH_H
