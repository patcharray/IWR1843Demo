#include <ellipticFilter.h>

/* 
 * Second-order section coefficients derived for elliptic filter using scipy.signal.ellip() function
 */
static const float sosBreathing[12] = {
    0.03801253f, -0.03652015f, 0.03801253f, 1.0f, -1.77809882f, 0.85359061f,
    1.0f,        -1.9997177f,  1.0f,        1.0f, -1.95749091f, 0.96216252f
};

static const float sosHeartbeat[12] = {
    0.09457149f,  0.1264823f,   0.09457149f, 1.0f, -0.63600174f, 0.69454343f,
    1.0f,        -1.97255535f,  1.0f,        1.0f, -1.55940984f, 0.81818437f
};

/***************************************************************************************************************
 *                                           DESCRIPTION 
 * Direct-Form II Transposed 2nd Order Biquad Elliptic (Cauer) Filter
 *                  | Breathing  | Heartbeat| 
 *    Bandwidth     | 0.1-0.5 Hz | 0.8-2 Hz |
 *   Max. Ripple    |    2dB     |    2dB   |
 * Min. Attenuation |    30dB    |    30dB  |
 *
 * Direct Form II allows the sequential connection of filters, so the output of first filter is connected to
 * the input of the second 
 ***************************************************************************************************************/
 
int ellipticFilter(const float *phase, float *filteredPhase, bool characteristic, EllipticFilterState *state){
    const float *sos = characteristic ? sosBreathing : sosHeartbeat;
    float x = *phase;
    int section;

    for (section = 0; section < 2; section++){
        const float *c = &sos[section * 6];
        float b0 = c[0];
        float b1 = c[1];
        float b2 = c[2];
        float a1 = c[4];
        float a2 = c[5];
        float y  = b0 * x + state->z1[section];

        state->z1[section] = b1 * x - a1 * y + state->z2[section];
        state->z2[section] = b2 * x - a2 * y;
        x = y;
    }

    *filteredPhase = x;
    return 1;
}
