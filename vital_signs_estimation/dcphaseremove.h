#ifndef DCPHASEREMOVE_H
#define DCPHASEREMOVE_H

typedef struct{         // struct for saving the current mean 
    float runningMean;
    int initialized;
} phaseRemoveState;

float DCPhaseRemove(float *inputPhase, phaseRemoveState *state);

#endif
