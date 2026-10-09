#ifndef ELLIPTICFILTER_H
#define ELLIPTICFILTER_H

#include <stdbool.h>

typedef struct { // Struct containing inputs and outputs of two filters
    float z1[2];
    float z2[2];
} EllipticFilterState;

int ellipticFilter(const float *phase, float *filteredPhase, bool characteristic, EllipticFilterState *state);

#endif
