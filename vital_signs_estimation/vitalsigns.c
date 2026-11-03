/***************************************************************************************************************
 *                                           DESCRIPTION 
 * The signal processing chain for breathing rate and heartrate detection/estimation for stationary targets.
0 * Begins to stream results ~18 seconds after startup every 100ms. That implies some limitations in detection
 * and tracking ability. For the specs of chirp/frame configuration, see profiles/profile_2d.cfg
 * 
 * 1. RANGE FFT
 *    
 *
 * 2. PHASE EXTRACTION (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS ACROSS ALL SAMPLES)
 *    Chest and heart movements are too small to cause a substantial magnitude change, therefore the extreme 
 *    sensitivity of phase is exploited in order to detect them. 
 *
 * 2. PHASE UNWRAPPING (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS ACROSS ALL SAMPLES)
 *    After the unwrapping the phase can only take values in a range (-pi, pi], but the physical displacement 
 *    of the chest can be greater than wavelength/4, therefore abrupt changes (jumps) in phase may occur.
 *
 * 3. DC PHASE REMOVAL (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS ACROSS ALL SAMPLES)
 *    After the unwrapping, the sum of all sampled phases does not equal zero -> DC element is present in all
 *    samples. To mitigate it, the running mean is calculated and subtracted in real time, thus saving 
 *    memory
 *
 * 4. PHASE DIFFERENCING (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS ACROSS ALL SAMPLES)
 *    The measured phase carries a drift tens of times larger than the chest displacement -> unwrap random-walk
 *    and real target motion. Functions as an exact first-order high-pass that greatly supresses lower bins 
 *
 * 5. FILTERING (GETS CALLED EACH FRAME FOR EACH RANGE BIN ALONG SLOW-TIME AXIS ACROSS ALL SAMPLES)
 *    The phase samples get filtered by elliptic bandpass filter tuned to the frequency of heartbeat, in order to 
 *    get rid of the high-frequency noise
 *
 * 6. PEAK DETECTION (GETS CALLED ONCE AFTER THE WARMUP FOR EACH RANGE BIN ALONG SLOW-TIME AXIS FOR THE FIRST SAMPLE)
 *    Because of the limitations of the hardware, not the whole range, but only the bin with the largest 
 *    magnitude after the filtering is picked for further processing. Once the bin is chosen, it cannot be changed,
 *    so if the position of the target has changed, the board has to be restarted.
 *    
 * 7. VIBRATION FFT (GETS CALLED AFTER THE SETTLING TIME ALONG FAST-TIME(SAMPLE) AXIS EVERY NEW SAMPLE)
 *    Transforms the samples of the chosen bin and outputs them in a frequency(beats per second)-magnitude ratio
 *
 * 8. PEAK DETECTION (GETS CALLED AFTER THE SETTLING TIME ALONG FAST-TIME(SAMPLE) AXIS EVERY NEW SAMPLE)
 *    To find the target, the bin with the largest magnitude is chosen and multiplied by 60 (beats per minute)
 *
 * 9. MEDIAN (GETS CALLED AFTER THE SETTLING TIME ALONG FAST-TIME(SAMPLE) AXIS EVERY NEW SAMPLE)
 *    To minimize the impact of outliers (extraordinary high and low values), the last 15 values are stored in 
 *    a circular buffer, insertion sorted and median is taken
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
 * The heartbeat bandpass filter needs ~5s to settle
 * Feeding the vibration FFT before that just buries the band in transients
 */
#define WARMUP_FRAMES  50             /* 5 s at 10 Hz */


/* 
 * Afterwards, accumulate filtered energy per range bin for this long and pick
 * the strongest bin ONCE. The bin must not move after that: the FFT window is
 * 128 real samples and 128 zeros of slow time, so re-selecting per frame splices unrelated bins
 * into one window and destroys the coherence the FFT depends on 
 */
#define SELECT_FRAMES  100             /* 10 s at 10 Hz */


/* 
 * Window of range bins the subject is expected to occupy.
 * Bin spacing follows from profileCfg: freqSlopeConst 67 MHz/us over an ADC
 * window of 64/10 MHz = 6.4 us gives a 428.8 MHz sweep, so
 *     dr = c / (2 * 428.8 MHz) = 0.35 m per bin
 * and the 64 bins span 22.4 m. 
 *
 * Bins 0 and 1 are dominated by TX->RX leakage -> large, nearly static phase that
 * would always win an energy contest. Beyond ~2 m there is nothing to measure
 * in a room. Widen RANGE_BIN_MAX if the subject sits further away 
 */
#define RANGE_BIN_MIN  2     // 0.7 m
#define RANGE_BIN_MAX  4     // 1.4 m

/* 
 * Median window over the reported rate.
 *
 * The slow-time window holds 128 real and 128 zero samples and advances by one per frame, so
 * successive spectra are nearly identical and a one-frame jump in the argmax
 * is not a change in the signal -> it is two bins sitting nearly level with
 * noise deciding between them. A median rejects those isolated flips outright
 * while still passing a genuine step in rate, which an average would smear.
 * Odd length so the median is a real sample; 15 frames costs 0.75 s of lag 
 */
#define RATE_MEDIAN_LEN 15

/* 
 * Reported in place of a rate so the host can tell *why* there is no rate yet,
 * and so a stale flash image is immediately obvious (it reports 0.0 instead).
 * During VS_STATE_FILLING the heart-rate slot carries the locked range bin 
 */
#define VS_STATE_WARMUP    (-1.0f)   // filters still settling 
#define VS_STATE_SELECTING (-2.0f)   // accumulating per-bin energy 
#define VS_STATE_FILLING   (-3.0f)   // slow-time window not full yet 
#define VS_STATE_NONFINITE (-4.0f)   // in-band spectrum has a NaN/Inf 
#define VS_STATE_SILENT    (-5.0f)   // in-band spectrum identically zero 
#define VS_STATE_NODETECT  (-6.0f)   // no subject in front of the sensor 


#define PRESENCE_ALPHA      0.02f
#define PRESENCE_ENERGY_MIN 4e-4f


#pragma DATA_SECTION(phase, ".l3ram");
#pragma DATA_SECTION(unwrapState, ".l3ram");
#pragma DATA_SECTION(removeState, ".l3ram");
#pragma DATA_SECTION(prevPhase, ".l3ram");
//#pragma DATA_SECTION(phaseBreathing, ".l3ram");
//#pragma DATA_SECTION(breathingState, ".l3ram");
#pragma DATA_SECTION(phaseHeartbeat, ".l3ram");
#pragma DATA_SECTION(heartbeatState, ".l3ram");
//#pragma DATA_SECTION(energyBreathing, ".l3ram");
// #pragma DATA_SECTION(energyHeartbeat, ".l3ram");      // the heartbeat signal is weak, so the bin localization  
#pragma DATA_SECTION(wState, ".l3ram");                  // has been put solely on breathing signal
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
//static float phaseBreathing[MAX_RANGE];
//#pragma DATA_ALIGN(breathingState, 8);
//static EllipticFilterState breathingState[MAX_RANGE];
static float phaseHeartbeat[MAX_RANGE];
#pragma DATA_ALIGN(heartbeatState, 8);
static EllipticFilterState heartbeatState[MAX_RANGE];
//static float energyBreathing[MAX_RANGE];
// static float energyHeartbeat[MAX_RANGE];
#pragma DATA_ALIGN(wState, 8);
static twiddleState wState;
#pragma DATA_ALIGN(breathingFFTState, 8);
static vibrationFFTState breathingFFTState;
#pragma DATA_ALIGN(heartbeatFFTState, 8);
static vibrationFFTState heartbeatFFTState;
static float samplesBreathing[N_ZERO_PAD];
static float samplesHeartbeat[N_ZERO_PAD];
static float breathHistory[RATE_MEDIAN_LEN];
static float heartHistory[RATE_MEDIAN_LEN];
#if !VITALSIGNS_DIAG
static int rateHistoryLen;
static int rateHistoryIdx;
#endif
static float presenceEnergy;
static int frameCount;
//static int binLocked;


void vitalSignsInit(void){
    memset(phase, 0, sizeof(phase));
    memset(unwrapState, 0, sizeof(unwrapState));
    memset(removeState, 0, sizeof(removeState));
    memset(prevPhase, 0, sizeof(prevPhase));
    //memset(phaseBreathing, 0, sizeof(phaseBreathing));
    memset(phaseHeartbeat, 0, sizeof(phaseHeartbeat));
    //memset(breathingState, 0, sizeof(breathingState));
    memset(heartbeatState, 0, sizeof(heartbeatState));
    //memset(energyBreathing, 0, sizeof(energyBreathing));
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
    presenceEnergy = 0.0f;
    frameCount = 0;
    //binLocked  = 0;
    vibrationFFTInit();
}


/* 
 * Median of the first n entries. Insertion sort on a copy
 */
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


/* 
 * Peak bin -> rate in units per minute, but first separate a genuine peak from
 * the two degenerate cases that both collapse onto the band's lowest bin: a
 * spectrum poisoned by NaN/Inf, and one that is identically zero. Without this
 * the two are indistinguishable from a real reading at the band edge 
 */
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
    return (((float) findMaxBin(spectrum, lo, hi) * sampFreq) / N_ZERO_PAD) * 60;
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
    

    /*
     * Phase unwrapping (to learn more see phaseunwrap.c and phaseunwrap.h)
     */
    for (i = 0; i < range; i++){
        phase[i] = phaseUnwrap(&phase[i], &unwrapState[i]);
    }


    /*
     * Removal of DC offset (to learn more see dcphaseremove.c and dcphaseremove.h)
     */
    for (i = 0; i < range; i++){
        phase[i] = DCPhaseRemove(&phase[i], &removeState[i]);
    }


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


    /*
     * 2nd order elliptic (Cauer) bandpass filter along phase axis that filters heartbeat
     * (implementation and detailed description in ellipticFilter.c and ellipticFilter.h)  
     */
    for (i = 0; i < range; i++){
        ellipticFilter(&phase[i], &phaseHeartbeat[i], false, &heartbeatState[i]);
    }


    /*
     * Generates "twiddle" factors needed for Radix-4 FFTs 
     */
    gen_twiddle_fft_sp(wState.twiddle, N_ZERO_PAD, &wState);

    frameCount++;

    if (frameCount <= WARMUP_FRAMES){     // Filters still settling - report nothing rather than a transient
        *breathingRate = VS_STATE_WARMUP;
        *heartRate     = VS_STATE_WARMUP;
        return;
    }

    /*
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

     */

    /* 
     * vibrationFFT only transforms once the window is already full on entry,
     * so this has to be sampled before the call - otherwise the frame that
     * completes the window reports a peak from an all-zero spectrum
     */
    spectrumValid = (breathingFFTState.sampleCount == N);


    vibrationFFT(phase[2], wState.twiddle, samplesBreathing, &breathingFFTState);
    vibrationFFT(phaseHeartbeat[2], wState.twiddle, samplesHeartbeat, &heartbeatFFTState);


    if (!spectrumValid){
        /* 
         * The 128-sample slow-time window is not full yet. Pass the locked bin
         * up - it is the one thing about the chain that cannot be
         * inferred from the rates
         */
        *breathingRate = VS_STATE_FILLING;
        *heartRate     = VS_STATE_FILLING;
        return;
    }


    /*
     * Mean-square of the filtered signal at the locked bin. Settles long
     * before the 128-frame window fills, so it needs no warm-up of its own
     */
    {
        float v = phase[2];
        presenceEnergy += PRESENCE_ALPHA * ((v * v) - presenceEnergy);
    }

    if (presenceEnergy < PRESENCE_ENERGY_MIN){
        /* 
         * Static clutter only -> nobody there. Reporting a peak from noise
         * would be indistinguishable from a real reading 
         */
        *breathingRate = VS_STATE_NODETECT;
        *heartRate     = VS_STATE_NODETECT;
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
         * where it would be accounted for as a normal one
         */
        *breathingRate = instBreathing;
        *heartRate     = instHeart;
        return;
    }


    /*
     * Store the last 15 values of breathingRate and heartRate in a buffer
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

}

