#include <converttocmplx.h>

void convertToComplex(float *input, int range, cmplx16ImRe_t *output){
    //convert to int16_t minimizing quantization losses and then cmplx[i].real = int16_t[i] and cmplx[i].imag = 0

    /*
    since phase can take any values in interval [-pi, pi] and int16_t has a range of [-32768, 32767], the simplest way to convert float 
    to int16_t is to just multiply the values in array by 10^4, since   pi < 3.15 -> 10^4 * pi < 10^4 * 3.15 < 32767. Some precision
    loss is unavoidable
    */

    int i;

    for (i = 0; i < range; i++){
        output[i].real = (int16_t) (10000 * input[i]);
        output[i].imag = 0;
    }

}
