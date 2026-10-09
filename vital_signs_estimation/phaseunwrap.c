#include <phaseunwrap.h>

/***************************************************************************************************************
 *                                          DESCRIPTION
 * After atan2sp() phase can only take values in a range (-pi, pi], but the physical displacement of a chest 
 * can be greater than wavelength/4, therefore the abrupt changes (jumps) in phase may occur. If the difference
 * between two neighbouring phase samples is greater than abs(pi), the algorithm adds/subtracts 2*pi and the
 * offset accumulated by the previous samples.
 * For example: phase[i] = 0.8pi, phase[i+1] = -0.9pi
 *              --> phase[i+1] - phase[i] = -1.7pi < -pi
 *              --> phase[i] += 2*pi = 1.1*pi
 *
 ***************************************************************************************************************/

float phaseUnwrap(float *rawPhase, phaseUnwrapState *state){

    if(!state->initialized){
        state->prevRaw = *rawPhase; // the first phase sample is saved in a struct, nothing to compare with
        state->offset = 0.0f;
        state->initialized = 1;
    } else {
        float diff = *rawPhase - state->prevRaw;
        if (diff > (float) M_PI){
            state->offset += 2 * (float) M_PI;
        } else if (diff < -(float) M_PI){
            state->offset -=  2 * (float) M_PI;
        }
        state->prevRaw = *rawPhase; // the raw sample is saved in a struct for comparison with next raw sample
    }
    return *rawPhase - state->offset;
}
