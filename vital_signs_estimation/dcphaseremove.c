#include <dcphaseremove.h>
/*
void DCPhaseRemove(float *phase, int range){
    float sum = 0;
    float mean = 0;
    int i;

    for(i = 0; i < range; i++){
        sum += phase[i];
    }

    mean = (float) sum / range;

    for(i = 0; i < range; i++){
        phase[i] -= mean;
    }
}
*/


float DCPhaseRemove(float *inputPhase, phaseRemoveState *state){
    if (!state->initialized){
        state->runningMean = *inputPhase;
        state->initialized = 1;
        return 0.0f;
    } else {
        state->runningMean += 0.01 * (*inputPhase - state->runningMean);
        return *inputPhase - state->runningMean;
    }
}
