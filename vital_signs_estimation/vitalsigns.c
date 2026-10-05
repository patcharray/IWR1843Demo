/*
                        DETAILED DESCRIPTION OF THE FULL PROCESSING CHAIN
1. RANGE FFT
2. DC PHASE COMPENSATION (GETS CALLED EACH FRAME)
3. PHASE UNWRAPPING (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS (ACROSS ALL SAMPLES))
4. DC PHASE REMOVAL (USES MOVING-AVERAGE, THUS ALSO GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS (ACROSS ALL SAMPLES))
5. FILTERING (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS (ACROSS ALL SAMPLES))
6. PEAK DETECTION (GETS CALLED EACH FRAME)
7. VIBRATION FFT (GETS CALLED ONCE ENOUGH SAMPLES HAVE BEEN ACCUMMULATED ALONG SLOW-TIME AXIS FOR THE CHOSEN BIN)
8. PEAK DETECTION ()
*/
#include <vitalsigns.h>

#define MAX_RANGE 64
#define sampFreq 10

/*
#pragma DATA_SECTION(phase, ".l3ram");
#pragma DATA_SECTION(unwrapState, ".l3ram");
#pragma DATA_SECTION(removeState, ".l3ram");
#pragma DATA_SECTION(phaseBreathing, ".l3ram");
#pragma DATA_SECTION(breathingState, ".l3ram");
#pragma DATA_SECTION(phaseHeartbeat, ".l3ram");
#pragma DATA_SECTION(heartbeatState, ".l3ram");
#pragma DATA_SECTION(wState, ".l3ram");
#pragma DATA_SECTION(breathingFFTState, ".l3ram");
#pragma DATA_SECTION(heartbeatFFTState, ".l3ram");
#pragma DATA_SECTION(samplesBreathing, ".l3ram");
#pragma DATA_SECTION(samplesHeartbeat, ".l3ram");
*/

static float phase [MAX_RANGE];
static phaseUnwrapState unwrapState[MAX_RANGE];
static phaseRemoveState removeState[MAX_RANGE];
static float phaseBreathing [MAX_RANGE];
static EllipticFilterState breathingState[MAX_RANGE];
static float phaseHeartbeat [MAX_RANGE];
static EllipticFilterState heartbeatState[MAX_RANGE];
static twiddleState wState;
static vibrationFFTState breathingFFTState;
static vibrationFFTState heartbeatFFTState;
static float samplesBreathing[N];
static float samplesHeartbeat[N];


void vitalSigns(cmplx16ImRe_t *radarCube, uint16_t range, float *breathingRate, float *heartRate){
    int i;
    int indexBreathing, indexHeartbeat;

    DSPhaseComp(radarCube, range);
    

    for (i = 0; i < range; i++){
        phase[i] = atan2sp(radarCube[i].imag, radarCube[i].real);
    }

    for (i = 0; i < range; i++){
        phase[i] = phaseUnwrap(&phase[i], &unwrapState[i]);
    }

    for (i = 0; i < range; i++){
        phase[i] = DCPhaseRemove(&phase[i], &removeState[i]);
    }

    for (i = 0; i < range; i++){
        ellipticFilter(&phase[i], &phaseBreathing[i], true, &breathingState[i]);
        ellipticFilter(&phase[i], &phaseHeartbeat[i], false, &heartbeatState[i]);
    }


    indexBreathing = DSPF_sp_maxidx(phaseBreathing, range);
    indexHeartbeat = DSPF_sp_maxidx(phaseHeartbeat, range);
    

    gen_twiddle_fft_sp(wState.twiddle, N, &wState);

    if (breathingFFTState.sampleCount == 0) {
        breathingFFTState.chosenBin = indexBreathing;   
        heartbeatFFTState.chosenBin = indexHeartbeat;
    }
    vibrationFFT(phaseBreathing[breathingFFTState.chosenBin], wState.twiddle, samplesBreathing, &breathingFFTState);

    vibrationFFT(phaseHeartbeat[heartbeatFFTState.chosenBin], wState.twiddle, samplesHeartbeat, &heartbeatFFTState);



    *breathingRate = (((float) DSPF_sp_maxidx(samplesBreathing, N) * sampFreq) / N) * 60;
    *heartRate = (((float) DSPF_sp_maxidx(samplesHeartbeat, N) * sampFreq) / N) * 60;

}
