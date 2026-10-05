#include <ellipticFilter.h>


int ellipticFilter(const float *phase, float *filteredPhase, bool characteristic, EllipticFilterState *state){

    static const float sosBreathing[12] = {0.03801253f, -0.03652015f, 0.03801253f, 1,       -1.77809882f, 0.85359061f,
                                            1,           -1.9997177f,  1,           1,       -1.95749091f, 0.96216252f};

    static const float sosHeartbeat[12] =  {0.09457149f, 0.1264823f,   0.09457149f, 1,       -0.63600174f, 0.69454343f,
                                             1,           -1.97255535f, 1,           1,       -1.55940984f, 0.81818437f};

    float hb1[5];
    float ha1[5];
    float hb2[5];
    float ha2[5];
    float xPair[2];

    const float *sos = characteristic ? sosBreathing : sosHeartbeat;

    if(!state->havePendingInput){
        state->pendingInput = *phase;
        state->havePendingInput = 1;
    } else {
        xPair[0] = state->pendingInput;
        xPair[1] = *phase;
        state->havePendingInput = 0;

        hb1[0] = sos[0];
        hb1[1] = sos[1];
        hb1[2] = sos[2];
        hb1[3] = 0;
        hb1[4] = 0;

        ha1[0] = 0;
        ha1[1] = sos[4];
        ha1[2] = sos[5];
        ha1[3] = 0;
        ha1[4] = 0;

        hb2[0] = sos[6];
        hb2[1] = sos[7];
        hb2[2] = sos[8];
        hb2[3] = 0;
        hb2[4] = 0;

        ha2[0] = 0;
        ha2[1] = sos[10];
        ha2[2] = sos[11];
        ha2[3] = 0;
        ha2[4] = 0;

        DSPF_sp_iir(state->y1Sec1, xPair, state->y2Sec1, hb1, ha1, NR);
        DSPF_sp_iir(state->y1Sec2, state->y2Sec1, state->y2Sec2, hb2, ha2, NR);

        state->outQueue[0] = state->y2Sec2[0];
        state->outQueue[1] = state->y2Sec2[1];
        state->outQueueLen = 2;

    }

    if (state->outQueueLen > 0){
        *filteredPhase = state->outQueue[2 - state->outQueueLen];
        state->outQueueLen--;
        return 1;
    }

    return 0;

}
