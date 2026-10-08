#include <vibrationfft.h>

/* 
 * DSPF_sp_fftSPxSP() returns complex spectrum, thus fftOutput serves as the 
 * intermediate stage
 */
#pragma DATA_SECTION(fftOutput, ".l3ram");
#pragma DATA_ALIGN(fftOutput, 8);
static float fftOutput[2 * N];

/* 
 * DSPF_sp_fftSPxSP() uses its "input" as in-place scratch: it writes its butterfly
 * results back over the input for every stage but the last. So, every time the
 * function executes, the input vector is copied into fftScratch
 */
#pragma DATA_SECTION(fftScratch, ".l3ram");
#pragma DATA_ALIGN(fftScratch, 8);
static float fftScratch[2 * N];

/*
 * Radix-4 returns the coefficients in reverse order, the bit-reversal table (brev)
 * reorders them
 */
#pragma DATA_ALIGN(brev, 8);
static unsigned char brev[64] = {
    0x0, 0x20, 0x10, 0x30, 0x8, 0x28, 0x18, 0x38,
    0x4, 0x24, 0x14, 0x34, 0xc, 0x2c, 0x1c, 0x3c,
    0x2, 0x22, 0x12, 0x32, 0xa, 0x2a, 0x1a, 0x3a,
    0x6, 0x26, 0x16, 0x36, 0xe, 0x2e, 0x1e, 0x3e,
    0x1, 0x21, 0x11, 0x31, 0x9, 0x29, 0x19, 0x39,
    0x5, 0x25, 0x15, 0x35, 0xd, 0x2d, 0x1d, 0x3d,
    0x3, 0x23, 0x13, 0x33, 0xb, 0x2b, 0x1b, 0x3b,
    0x7, 0x27, 0x17, 0x37, 0xf, 0x2f, 0x1f, 0x3f
};

/*
 * Taken from ti/dsplib_c674x_3_4_0_0/examples/fft_sp_ex/fft_example_sp.c
 */
void gen_twiddle_fft_sp(float *w, int n, twiddleState *state)
{
    if (!state->initialized){
        state->initialized = 1;
        int i, j, k;
        double theta1, theta2, theta3;
        const double PI = 3.141592654;
        for (j = 1, k = 0; j <= n >> 2; j = j << 2) {
            for (i = 0; i < n >> 2; i += j) {
                theta1 = 2*PI*i/n; w[k]   = cos(theta1); w[k+1] = sin(theta1);
                theta2 = 4*PI*i/n; w[k+2] = cos(theta2); w[k+3] = sin(theta2);
                theta3 = 6*PI*i/n; w[k+4] = cos(theta3); w[k+5] = sin(theta3);
                k += 6;
            }
        }
    }

}

/***************************************************************************************************************
 *                                           DESCRIPTION 
 * Complex Radix-4 FFT over the samples of the chosen bin. Runs every sample, removing the oldest sample, 
 * shifting other samples and appending new sample. Needs to fill its sample buffer first in order to execute.
 *
 ***************************************************************************************************************/

void vibrationFFT(float newSample, float *w, float *outputSpectrum, vibrationFFTState *state){
    if (state->sampleCount < N){
        state->sampleBuffer[2 * state->sampleCount] = newSample; // Fills the buffer
        state->sampleBuffer[2 * state->sampleCount + 1] = 0.0f;
        state->sampleCount++;
    } else {
        memmove(state->sampleBuffer, state->sampleBuffer + 2, (N - 1) * 2 * sizeof(float)); // Shift N-1 samples
        state->sampleBuffer[2 * (N - 1)] = newSample; // Appends new sample
        state->sampleBuffer[2 * (N - 1) + 1] = 0.0f;
        int i;
        memcpy(fftScratch, state->sampleBuffer, 2 * N * sizeof(float)); // Copies the buffer (see fftScratch description)
        DSPF_sp_fftSPxSP(N, fftScratch, w, fftOutput, brev, 4, 0, N);
        for (i = 0; i < N; i++){ // Takes magnitude of each sample (see fftOutput description)
            outputSpectrum[i] = sqrtsp(fftOutput[2 * i] * fftOutput[2 * i] + fftOutput[2 * i + 1] * fftOutput[2 * i + 1]);
        }

    }
}
