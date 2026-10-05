#include <vibrationfft.h>

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
int bufferState(float newSample){

    sampleBuffer[2*sampleCount]     = newSample;
    sampleBuffer[2*sampleCount + 1] = 0.0f;
    sampleCount++;

    if (sampleCount < N){
        return 0;
    }
    else {
        sampleCount = 0;
        return 1;
    }
}
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

/*
void vibrationFFT(float newSample, const float *w, float *outputSpectrum){
    if (bufferState(newSample)){
        float fftOutput [2 * N];
        DSPF_sp_fftSPxSP(N, sampleBuffer, w, fftOutput, brev, 4, 0, N);
        int i; 
        for (i = 0; i < N; i++){
            outputSpectrum[i] = sqrtsp(fftOutput[2*i]*fftOutput[2*i] + fftOutput[2*i+1]*fftOutput[2*i+1]);

        }
    }

}
*/

void vibrationFFT(float newSample, float *w, float *outputSpectrum, vibrationFFTState *state){
    if (state->sampleCount < N){
        state->sampleBuffer[2 * state->sampleCount] = newSample;
        state->sampleBuffer[2 * state->sampleCount + 1] = 0.0f;
        state->sampleCount++;
    } else {
        memmove(state->sampleBuffer, state->sampleBuffer + 2, (N - 1) * 2 * sizeof(float));
        state->sampleBuffer[2 * (N - 1)] = newSample;
        state->sampleBuffer[2 * (N - 1) + 1] = 0.0f;
        float fftOutput [2 * N];
        int i;
        DSPF_sp_fftSPxSP(N, state->sampleBuffer, w, fftOutput, brev, 4, 0, N);
        for (i = 0; i < N; i++){
            outputSpectrum[i] = sqrtsp(fftOutput[2 * i] * fftOutput[2 * i] + fftOutput[2 * i + 1] * fftOutput[2 * i + 1]);
        }

    }
}
