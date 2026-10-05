#include <phaseunwrap.h>

/*
void phaseUnwrap(float *phase, int range){
    int i;
    for (i = 0; i < range - 1; i++){
        if (phase[i + 1] - phase[i] > (float) M_PI){
            phase[i+1] -= 2 * (float) M_PI;
        } 
        else if (phase[i + 1] - phase[i] < -(float) M_PI){
            phase[i + 1] += 2 * (float) M_PI;
        }
        else {
        continue;
        }
    }
}
*/

float phaseUnwrap(float *rawPhase, phaseUnwrapState *state){

    if(!state->initialized){
        state->prevRaw = *rawPhase;
        state->offset = 0.0f;
        state->initialized = 1;
    } else {
        float diff = *rawPhase - state->prevRaw;
        if (diff > (float) M_PI){
            state->offset += 2 * (float) M_PI;
        } else if (diff < -(float) M_PI){
            state->offset -=  2 * (float) M_PI;
        }
        state->prevRaw = *rawPhase;
    }
    return *rawPhase - state->offset;
}
