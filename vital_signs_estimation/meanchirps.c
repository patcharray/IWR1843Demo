#include <meanchirps.h>

/***************************************************************************************************************
 *                                           DESCRIPTION 
 * Collects all chirps in one frame, adds them and divides by number of chirps in a single frame.
 * Basically a mean value, suppresses noise -> raises SNR. 
 *
 ***************************************************************************************************************/

void meanChirps(cmplx16ImRe_t *radarCube, uint16_t range, uint16_t numChirps, cmplx16ImRe_t *mean){
    int bin, chirp;
    int32_t sumReal, sumImag;

    for (bin = 0; bin < range; bin ++){
        sumReal = 0;
        sumImag = 0;
        for(chirp = 0; chirp < numChirps; chirp++){
            sumReal += radarCube[chirp * range + bin].real;
            sumImag += radarCube[chirp * range + bin].imag;
        }
        mean[bin].real = (int16_t)(sumReal / numChirps);
        mean[bin].imag = (int16_t)(sumImag / numChirps);
    }
}
