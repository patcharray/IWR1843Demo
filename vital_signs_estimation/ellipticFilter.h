#ifndef ELLIPTICFILTER_H
#define ELLIPTICFILTER_H

#include <stdbool.h>
#include <ti/dsplib/src/DSPF_sp_iir/DSPF_sp_iir.h>
#include <string.h>

#define NR 2

typedef struct {
    float y1Sec1[NR + 4];
    float y1Sec2[NR + 4];
    float y2Sec1[NR + 4];
    float y2Sec2[NR + 4];

    float x1[NR + 4];
    float x2[NR + 4];

    float pendingInput;
    int   havePendingInput;

    float outQueue[2];
    int   outQueueLen;

    float pad;
} EllipticFilterState;

int ellipticFilter(const float *phase, float *filteredPhase, bool characteristic, EllipticFilterState *state);

#endif
