#ifndef PHASEUNWRAP_H
#define PHASEUNWRAP_H

#include <math.h>


typedef struct{      // struct for saving the previous values of raw phase fo comparison 
    float prevRaw;   // and accumulation of a global phase offset
    float offset;
    int initialized;
} phaseUnwrapState;

float phaseUnwrap(float *rawPhase, phaseUnwrapState *state);

#endif
