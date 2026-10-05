#ifndef VIBRATIONFFT_H
#define VIBRATIONFFT_H

#include <ti/dsplib/src/DSPF_sp_fftSPxSP/DSPF_sp_fftSPxSP.h>
#include <math.h>
#include <string.h>

/* See note in dcphasecomp.h: clear TRUE/FALSE around mathlib, then restore. */
#undef TRUE
#undef FALSE
#include <ti/mathlib/mathlib.h>
#undef TRUE
#undef FALSE
#define TRUE  1U
#define FALSE 0U

#define N 256

typedef struct{
    int initialized;
    float twiddle[2 * N];
} twiddleState;

typedef struct{
    float sampleBuffer[2 * N];
    int sampleCount;
    int chosenBin;
}   vibrationFFTState;

void gen_twiddle_fft_sp(float *w, int n, twiddleState *state);
void vibrationFFT(float newSample, float *w, float *outputSpectrum, vibrationFFTState *state);

#endif
