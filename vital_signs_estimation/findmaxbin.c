#include <findmaxbin.h>




int findMaxBin(const float *input, const int minBin, const int maxBin){
    int i;
    int maxIdx = minBin;
    for (i = minBin; i < (maxBin + 1); i++){
        if (input[i] > input[maxIdx]){
            maxIdx = i;
        }
    }
    return maxIdx;
}
