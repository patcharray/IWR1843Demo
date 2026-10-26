#include <dcphaseremove.h>

/***************************************************************************************************************
 *                                           DESCRIPTION 
 * Removes the DC offset along phase axis in real time via running mean.
 *
 ***************************************************************************************************************/

float DCPhaseRemove(float *inputPhase, phaseRemoveState *state){
    if (!state->initialized){
        state->runningMean = *inputPhase;
        state->initialized = 1;
        return 0.0f;
    } else {
        state->runningMean += 0.01 * (*inputPhase - state->runningMean); // 0.01 is the steepness factor 
        return *inputPhase - state->runningMean;                         // Can be adjusted, but 0.01 works well
    }
}
