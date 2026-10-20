#ifndef MATMUL_H
#define MATMUL_H
#include <math.h>
#include <stdint.h>
#include <ti/utils/mathutils/mathutils.h>

void matMul(const int16_t *inputMatrix1, const int rows1, const int columns1, const int16_t *inputMatrix2, const int rows2, const int columns2, int16_t *outputMatrix);

#endif
