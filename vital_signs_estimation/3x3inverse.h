#ifndef _3X3INVERSE_H
#define _3X3INVERSE_H

#include <math.h>
#include <stdint.h>
#include <ti/utils/mathutils/mathutils.h>

void matrix3x3inverse(const int16_t *inputMatrix, int16_t *outputMatrix);

#endif
