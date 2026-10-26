#include <findmaxbin.h>


 /***************************************************************************************************************
 *                                           DESCRIPTION 
 * Locates and outputs the bin with a peak value on a given interval. NaN-safe: a NaN anywhere in the band 
 * makes every `>` comparison false, which would otherwise silently pin the result at minBin. Tracking the 
 * running maximum in a separate variable lets NaNs be skipped instead.
 *
 ***************************************************************************************************************/
 
int findMaxBin(const float *input, const int minBin, const int maxBin){
    int i;
    int maxIdx = minBin;
    float max = input[minBin];

    for (i = minBin; i < (maxBin + 1); i++){
        if (input[i] > max){
            max = input[i];
            maxIdx = i;
        }
    }
    return maxIdx;
}
