#ifndef VIBRATIONFFT_H
#define VIBRATIONFFT_H

#include <ti/dsplib/src/DSPF_sp_fftSPxSP/DSPF_sp_fftSPxSP.h>
#include <math.h>
#include <string.h>

/* 
 * See note in dcphasecomp.h: clear TRUE/FALSE around mathlib, then restore
 */
#undef TRUE
#undef FALSE
#include <ti/mathlib/mathlib.h>
#undef TRUE
#undef FALSE
#define TRUE  1U
#define FALSE 0U

#define N 256 // Number of samples


typedef struct{           // Struct containing twiddle initialization
    float twiddle[2 * N];
    int initialized;
} twiddleState;

typedef struct{                // Struct containing FFT buffer and bin, 
    float sampleBuffer[2 * N]; // for which the vibration (doppler) FFT
    int sampleCount;           // will execute
    int chosenBin;
}   vibrationFFTState;

void gen_twiddle_fft_sp(float *w, int n, twiddleState *state);
void vibrationFFT(float newSample, float *w, float *outputSpectrum, vibrationFFTState *state);

#endif
