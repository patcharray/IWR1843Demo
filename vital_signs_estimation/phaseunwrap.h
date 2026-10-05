#ifndef PHASEUNWRAP_H
#define PHASEUNWRAP_H

#include <math.h>

typedef struct{
    float prevRaw;
    float offset;
    int initialized;
} phaseUnwrapState;

float phaseUnwrap(float *rawPhase, phaseUnwrapState *state);

#endif
