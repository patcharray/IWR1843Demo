/***************************************************************************************************************
 *                                           DESCRIPTION 
 * The signal processing chain for breathing rate and heartrate detection/estimation for stationary targets.
 * 
 * 1. RANGE FFT
 * 2. PHASE EXTRACTION (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS (ACROSS ALL SAMPLES))
 *    Chest and heart movements are too small to cause a substantial magnitude change, therefore the extreme 
 *    sensitivity of phase is exploited in order to detect them. 
 *
 * 2. PHASE UNWRAPPING (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS (ACROSS ALL SAMPLES))
 *    After the unwrapping the phase can only take values in a range (-pi, pi], but the physical displacement 
 *    of the chest can be greater than wavelength/4, therefore abrupt changes (jumps) in phase may occur.
 *
 * 3. DC PHASE REMOVAL (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS (ACROSS ALL SAMPLES))
 *    After the unwrapping, the sum of all sampled phases does not equal zero -> DC element is present in all
 *    samples. To mitigate it, the running mean is calculated and subtracted in real time, thus saving 
 *    memory
 *
 * 4. PHASE DIFFERENCING (EXACT FIRST-ORDER HIGH-PASS; REMOVES THE SLOW DRIFT THAT THE BANDPASS TRANSITION BAND CANNOT)
 *    The measured phase carries a drift tens of times larger than the chest displacement -> unwrap random-walk
 *    and real target motion. Functions as an exact first-order high-pass
 *
 * 5. FILTERING (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS (ACROSS ALL SAMPLES))
 *
 *
 * 6. PEAK DETECTION (GETS CALLED EACH FRAME)
 *
 *
 * 7. VIBRATION FFT (GETS CALLED ONCE ENOUGH SAMPLES HAVE BEEN ACCUMMULATED ALONG SLOW-TIME AXIS FOR THE CHOSEN BIN)
 *
 *
 * 8. PEAK DETECTION ()
 *
 *
 * 
 ***************************************************************************************************************/
#include <vitalsigns.h>

#define MAX_RANGE 64
#define sampFreq 10


#define BREATH_BIN_MIN 3
#define BREATH_BIN_MAX 13
#define HEART_BIN_MIN  17
#define HEART_BIN_MAX  77

/* 
 * The chain has to settle before any of it means anything:
 *  - DCPhaseRemove is an EMA with alpha = 0.01, i.e. tau ~ 10 s;
 *  - the breathing SOS cascade has a pole radius of sqrt(0.96216) and needs
 *    ~26 s to settle.
 * Feeding the vibration FFT before that just buries the band in transients
 */
/* Diagnostic mode: instead of rates, stream the DC-removed unwrapped phase at
 * the locked range bin (float 1) and the locked bin index itself (float 2), so
 * the slow-time signal can be logged and analysed off-target. Set to 0 for
 * normal operation. */
#define VITALSIGNS_DIAG 0

#if VITALSIGNS_DIAG
#define WARMUP_FRAMES  150             /* 15 s -- just the EMA, not the SOS */
#else
#define WARMUP_FRAMES  300             /* 30 s at 10 Hz */
#endif

/* Afterwards, accumulate filtered energy per range bin for this long and pick
 * the strongest bin ONCE. The bin must not move after that: the FFT window is
 * 256 samples of slow time, so re-selecting per frame splices unrelated bins
 * into one window and destroys the coherence the FFT depends on. */
#if VITALSIGNS_DIAG
#define SELECT_FRAMES  50              /* 5 s at 10 Hz */
#else
#define SELECT_FRAMES  100             /* 10 s at 10 Hz */
#endif

/* Window of range bins the subject is expected to occupy.
 *
 * Bin spacing follows from profileCfg: freqSlopeConst 100 MHz/us over an ADC
 * window of 64/7.2 MHz = 8.889 us gives a 888.9 MHz sweep, so
 *     dr = c / (2 * 888.9 MHz) = 0.169 m per bin
 * and the 64 bins span 10.8 m. (The comment block at the top of
 * profile_2d.cfg claims 0.042 m/bin; that is stale and wrong by 4x.)
 *
 * Bins 0..2 are dominated by TX->RX leakage -- large, nearly static phase that
 * would always win an energy contest. Beyond ~2 m there is nothing to measure
 * in a room, and letting the selector roam that far is how it locked onto bin
 * 26 (4.4 m) and bin 62 (10.5 m) on earlier runs. Widen RANGE_BIN_MAX if the
 * subject sits further away. */
#define RANGE_BIN_MIN  3     /* ~0.51 m */
#define RANGE_BIN_MAX  8    /* ~1.42 m */

/* Median window over the reported rate.
 *
 * The slow-time window holds 256 samples and advances by one per frame, so
 * successive spectra are nearly identical and a one-frame jump in the argmax
 * is not a change in the signal -- it is two bins sitting nearly level with
 * noise deciding between them. A median rejects those isolated flips outright
 * while still passing a genuine step in rate, which an average would smear.
 * Odd length so the median is a real sample; 15 frames costs 0.75 s of lag. */
#define RATE_MEDIAN_LEN 15

/* Reported in place of a rate so the host can tell *why* there is no rate yet,
 * and so a stale flash image is immediately obvious (it reports 0.0 instead).
 * During VS_STATE_FILLING the heart-rate slot carries the locked range bin. */
#define VS_STATE_WARMUP    (-1.0f)   /* filters still settling */
#define VS_STATE_SELECTING (-2.0f)   /* accumulating per-bin energy */
#define VS_STATE_FILLING   (-3.0f)   /* slow-time window not full yet */
#define VS_STATE_NONFINITE (-4.0f)   /* in-band spectrum has a NaN/Inf */
#define VS_STATE_SILENT    (-5.0f)   /* in-band spectrum identically zero */

/* Temporary: locate where a NaN/Inf first enters the chain. Once a stage goes
 * non-finite the IIR state latches it forever, so only the FIRST occurrence
 * tells us anything -- hence the latch. Reports -(100 + stage) in the breathing
 * slot and the offending range bin in the heart slot. Set to 0 to remove. */
#define VITALSIGNS_TRACE_NONFINITE 0

#define VS_STAGE_ATAN2      1
#define VS_STAGE_UNWRAP     2
#define VS_STAGE_DCREMOVE   3
#define VS_STAGE_DIFF       4
#define VS_STAGE_FILT_BR    5
#define VS_STAGE_FILT_HR    6
#define VS_STAGE_FFT_IN     7
#define VS_STAGE_SPECTRUM   8


#pragma DATA_SECTION(phase, ".l3ram");
#pragma DATA_SECTION(unwrapState, ".l3ram");
#pragma DATA_SECTION(removeState, ".l3ram");
#pragma DATA_SECTION(prevPhase, ".l3ram");
#pragma DATA_SECTION(phaseBreathing, ".l3ram");
#pragma DATA_SECTION(breathingState, ".l3ram");
#pragma DATA_SECTION(phaseHeartbeat, ".l3ram");
#pragma DATA_SECTION(heartbeatState, ".l3ram");
#pragma DATA_SECTION(energyBreathing, ".l3ram");
// #pragma DATA_SECTION(energyHeartbeat, ".l3ram");
#pragma DATA_SECTION(wState, ".l3ram");
#pragma DATA_SECTION(breathingFFTState, ".l3ram");
#pragma DATA_SECTION(heartbeatFFTState, ".l3ram");
#pragma DATA_SECTION(samplesBreathing, ".l3ram");
#pragma DATA_SECTION(samplesHeartbeat, ".l3ram");
#pragma DATA_SECTION(breathHistory, ".l3ram");
#pragma DATA_SECTION(heartHistory, ".l3ram");


static float phase[MAX_RANGE];
static phaseUnwrapState unwrapState[MAX_RANGE];
static phaseRemoveState removeState[MAX_RANGE];
static float prevPhase[MAX_RANGE];
static float phaseBreathing[MAX_RANGE];
#pragma DATA_ALIGN(breathingState, 8);
static EllipticFilterState breathingState[MAX_RANGE];
static float phaseHeartbeat[MAX_RANGE];
#pragma DATA_ALIGN(heartbeatState, 8);
static EllipticFilterState heartbeatState[MAX_RANGE];
static float energyBreathing[MAX_RANGE];
// static float energyHeartbeat[MAX_RANGE];
#pragma DATA_ALIGN(wState, 8);
static twiddleState wState;
#pragma DATA_ALIGN(breathingFFTState, 8);
static vibrationFFTState breathingFFTState;
#pragma DATA_ALIGN(heartbeatFFTState, 8);
static vibrationFFTState heartbeatFFTState;
static float samplesBreathing[N];
static float samplesHeartbeat[N];
static float breathHistory[RATE_MEDIAN_LEN];
static float heartHistory[RATE_MEDIAN_LEN];
static int rateHistoryLen;
static int rateHistoryIdx;
static int frameCount;
static int binLocked;
#if VITALSIGNS_TRACE_NONFINITE
static int faultStage;
static int faultBin;
#endif

void vitalSignsInit(void){
    memset(phase, 0, sizeof(phase));
    memset(unwrapState, 0, sizeof(unwrapState));
    memset(removeState, 0, sizeof(removeState));
    memset(prevPhase, 0, sizeof(prevPhase));
    memset(phaseBreathing, 0, sizeof(phaseBreathing));
    memset(phaseHeartbeat, 0, sizeof(phaseHeartbeat));
    memset(breathingState, 0, sizeof(breathingState));
    memset(heartbeatState, 0, sizeof(heartbeatState));
    memset(energyBreathing, 0, sizeof(energyBreathing));
    // memset(energyHeartbeat, 0, sizeof(energyHeartbeat));
    memset(&wState, 0, sizeof(wState));
    memset(&breathingFFTState, 0, sizeof(breathingFFTState));
    memset(&heartbeatFFTState, 0, sizeof(heartbeatFFTState));
    memset(samplesBreathing, 0, sizeof(samplesBreathing));
    memset(samplesHeartbeat, 0, sizeof(samplesHeartbeat));
    memset(breathHistory, 0, sizeof(breathHistory));
    memset(heartHistory, 0, sizeof(heartHistory));
    rateHistoryLen = 0;
    rateHistoryIdx = 0;
    frameCount = 0;
    binLocked  = 0;
#if VITALSIGNS_TRACE_NONFINITE
    faultStage = 0;
    faultBin   = -1;
#endif
}



#if VITALSIGNS_TRACE_NONFINITE
/* Index of the first NaN or Inf in v[0..n), else -1. NaN is detected via the
 * one value that compares unequal to itself; Inf via magnitude. */
static int firstNonFinite(const float *v, int n){
    int i;
    for (i = 0; i < n; i++){
        float f = v[i];
        if (f != f){
            return i;
        }
        if ((f > 3.0e38f) || (f < -3.0e38f)){
            return i;
        }
    }
    return -1;
}

/* Largest magnitude in v[0..n), or the NaN itself if one is present, so the
 * host sees either a real scale or a NaN rather than a bucketed verdict. */
static float maxAbsOrNan(const float *v, int n){
    int i;
    float m = 0.0f;
    for (i = 0; i < n; i++){
        float f = v[i];
        if (f != f){
            return f;
        }
        if (f < 0.0f){
            f = -f;
        }
        if (f > m){
            m = f;
        }
    }
    return m;
}

/* Latch only the earliest fault: later stages are downstream consequences. */
static void traceStage(int stage, const float *v, int n){
    int idx;
    if (faultStage != 0){
        return;
    }
    idx = firstNonFinite(v, n);
    if (idx >= 0){
        faultStage = stage;
        faultBin   = idx;
    }
}
#endif

/* Median of the first n entries. Insertion sort on a copy: n is 15, so this is
 * cheaper and clearer than anything asymptotically better. */
static float medianOf(const float *history, int n){
    float tmp[RATE_MEDIAN_LEN];
    int i, j;

    for (i = 0; i < n; i++){
        tmp[i] = history[i];
    }
    for (i = 1; i < n; i++){
        float v = tmp[i];
        for (j = i - 1; (j >= 0) && (tmp[j] > v); j--){
            tmp[j + 1] = tmp[j];
        }
        tmp[j + 1] = v;
    }
    return tmp[n / 2];
}


/* Peak bin -> rate in units per minute, but first separate a genuine peak from
 * the two degenerate cases that both collapse onto the band's lowest bin: a
 * spectrum poisoned by NaN/Inf, and one that is identically zero. Without this
 * the two are indistinguishable from a real reading at the band edge. */
static float bandRate(const float *spectrum, int lo, int hi){
    int i;
    int nonFinite = 0;
    int nonZero   = 0;

    for (i = lo; i <= hi; i++){
        float v = spectrum[i];
        if (v != v){                 /* NaN: the only value unequal to itself */
            nonFinite = 1;
        } else if ((v > 3.0e38f) || (v < -3.0e38f)){
            nonFinite = 1;
        } else if (v != 0.0f){
            nonZero = 1;
        }
    }

    if (nonFinite){
        return VS_STATE_NONFINITE;
    }
    if (!nonZero){
        return VS_STATE_SILENT;
    }
    return (((float) findMaxBin(spectrum, lo, hi) * sampFreq) / N) * 60;
}


void vitalSigns(cmplx16ImRe_t *radarCube, uint16_t range, float *breathingRate, float *heartRate){
    int i;
#if !VITALSIGNS_DIAG
    int spectrumValid;
    float instBreathing;
    float instHeart;
#endif


    /*
     * Phase extraction from range FFT
     */
    for (i = 0; i < range; i++){
        phase[i] = atan2sp(radarCube[i].imag, radarCube[i].real);
    }

#if VITALSIGNS_TRACE_NONFINITE
    traceStage(VS_STAGE_ATAN2, phase, range);
#endif


    /*
     * Phase unwrapping (to learn more see phaseunwrap.c and phaseunwrap.h)
     */
    for (i = 0; i < range; i++){
        phase[i] = phaseUnwrap(&phase[i], &unwrapState[i]);
    }

#if VITALSIGNS_TRACE_NONFINITE
    traceStage(VS_STAGE_UNWRAP, phase, range);
#endif


    /*
     * Removal of DC offset (to learn more see dcphaseremove.c and dcphaseremove.h)
     */
    for (i = 0; i < range; i++){
        phase[i] = DCPhaseRemove(&phase[i], &removeState[i]);
    }

#if VITALSIGNS_TRACE_NONFINITE
    traceStage(VS_STAGE_DCREMOVE, phase, range);
#endif

    /*
     * Differencing the slow-time phase. The measured phase carries a drift
     * tens of times larger than the chest displacement -> unwrap random-walk
     * and real target motion. 
     */
    for (i = 0; i < range; i++){
        float current = phase[i];
        phase[i]      = current - prevPhase[i];
        prevPhase[i]  = current;
    }

#if VITALSIGNS_TRACE_NONFINITE
    traceStage(VS_STAGE_DIFF, phase, range);
#endif


    /*
     * 2nd order elliptic (Cauer) bandpass filters along phase axis that filter
     * breathing and heartbreat respectfully
     * (implementation and detailed description in ellipticFilter.c and ellipticFilter.h)  
     */
    for (i = 0; i < range; i++){
        ellipticFilter(&phase[i], &phaseBreathing[i], true, &breathingState[i]);
        ellipticFilter(&phase[i], &phaseHeartbeat[i], false, &heartbeatState[i]);
    }

#if VITALSIGNS_TRACE_NONFINITE
    traceStage(VS_STAGE_FILT_BR, phaseBreathing, range);
    traceStage(VS_STAGE_FILT_HR, phaseHeartbeat, range);
    if (faultStage != 0){
        *breathingRate = (float) -(100 + faultStage);
        *heartRate     = (float) faultBin;
        return;
    }
#endif


    /*
     * Generates "twiddle" factors needed for Radix-4 FFTs 
     */
    gen_twiddle_fft_sp(wState.twiddle, N, &wState);

    frameCount++;

    if (frameCount <= WARMUP_FRAMES){     // Filters still settling - report nothing rather than a transient
        *breathingRate = VS_STATE_WARMUP;
        *heartRate     = VS_STATE_WARMUP;
        return;
    }

    if (!binLocked){
        for (i = RANGE_BIN_MIN; i < range; i++){ // Phase can be negative -> abs()^2 to find the bin with biggest magnitude
            energyBreathing[i] += phaseBreathing[i] * phaseBreathing[i];
            // energyHeartbeat[i] += phaseHeartbeat[i] * phaseHeartbeat[i];
        }

        if (frameCount < (WARMUP_FRAMES + SELECT_FRAMES)){
            *breathingRate = VS_STATE_SELECTING;
            *heartRate     = VS_STATE_SELECTING;
            return;
        }

        int hi = (RANGE_BIN_MAX < (range - 1)) ? RANGE_BIN_MAX : (range - 1); // Never let the window run 
                                                                              // past the configured bin count

        breathingFFTState.chosenBin = findMaxBin(energyBreathing, RANGE_BIN_MIN, hi);
        heartbeatFFTState.chosenBin = breathingFFTState.chosenBin;
        // heartbeatFFTState.chosenBin = findMaxBin(energyHeartbeat, RANGE_BIN_MIN, hi);
        binLocked = 1;
    }

#if VITALSIGNS_DIAG
    *breathingRate = phase[breathingFFTState.chosenBin];
    *heartRate     = (float) breathingFFTState.chosenBin;
#else

    /* 
     * vibrationFFT only transforms once the window is already full on entry,
     * so this has to be sampled before the call - otherwise the frame that
     * completes the window reports a peak from an all-zero spectrum
     */
    spectrumValid = (breathingFFTState.sampleCount == N);

#if VITALSIGNS_TRACE_NONFINITE
    traceStage(VS_STAGE_FFT_IN, breathingFFTState.sampleBuffer, 2 * N);
#endif

    vibrationFFT(phaseBreathing[breathingFFTState.chosenBin], wState.twiddle, samplesBreathing, &breathingFFTState);
    vibrationFFT(phaseHeartbeat[heartbeatFFTState.chosenBin], wState.twiddle, samplesHeartbeat, &heartbeatFFTState);

#if VITALSIGNS_TRACE_NONFINITE
    traceStage(VS_STAGE_SPECTRUM, samplesBreathing, N);
    if (faultStage != 0){
        if (faultStage == VS_STAGE_SPECTRUM){
            /* Every upstream stage is finite, so report the scale of the two
             * things the FFT actually consumes: the twiddle table (should be
             * <= 1.0) and the slow-time window (should be a few radians). A
             * NaN or a huge value in either names the culprit outright. */
            *breathingRate = maxAbsOrNan(wState.twiddle, 2 * N);
            *heartRate     = maxAbsOrNan(breathingFFTState.sampleBuffer, 2 * N);
        } else {
            *breathingRate = (float) -(100 + faultStage);
            *heartRate     = (float) faultBin;
        }
        return;
    }
#endif

    if (!spectrumValid){
        /* 
         * The 256-sample slow-time window is not full yet. Pass the locked bin
         * up - it is the one thing about the chain that cannot be
         * inferred from the rates
         */
        *breathingRate = VS_STATE_FILLING;
        *heartRate     = (float) breathingFFTState.chosenBin;
        return;
    }

    
    /*
     * Filters out garbage and out of bounds vibrationFFT values, 
     * returns breathingRate/heartRate in beats per minute
     */
    instBreathing = bandRate(samplesBreathing, BREATH_BIN_MIN, BREATH_BIN_MAX);
    instHeart     = bandRate(samplesHeartbeat, HEART_BIN_MIN, HEART_BIN_MAX);

    if ((instBreathing < 0.0f) || (instHeart < 0.0f)){
        /* 
         * A faulty spectrum reports itself; it must not enter the history,
         * where it would be accouted for as a normal one
         */
        *breathingRate = instBreathing;
        *heartRate     = instHeart;
        return;
    }


    /*
     * Store the last 15 values of breathingRate and heartRate in cache
     */
    breathHistory[rateHistoryIdx] = instBreathing;
    heartHistory[rateHistoryIdx]  = instHeart;
    rateHistoryIdx = (rateHistoryIdx + 1) % RATE_MEDIAN_LEN;
    if (rateHistoryLen < RATE_MEDIAN_LEN){
        rateHistoryLen++;
    }

    /*
     * To suppress outliers, the median of last 15 values is taken
     */
    *breathingRate = medianOf(breathHistory, rateHistoryLen);
    *heartRate     = medianOf(heartHistory, rateHistoryLen);
#endif
}

