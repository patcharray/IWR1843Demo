#ifndef DCPHASEREMOVE_H
#define DCPHASEREMOVE_H

typedef struct{
    float runningMean;
    int initialized;
} phaseRemoveState;

float DCPhaseRemove(float *inputPhase, phaseRemoveState *state);

#endif
