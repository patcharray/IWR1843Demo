#include <3x3inverse.h>

void matrix3x3inverse(const int16_t *inputMatrix, int16_t *outputMatrix){

    double val;

    int32_t det = inputMatrix[0] * (inputMatrix[4] * inputMatrix[8] - inputMatrix[5] * inputMatrix[7]) +
                  inputMatrix[1] * (inputMatrix[3] * inputMatrix[8] - inputMatrix[5] * inputMatrix[6]) +
                  inputMatrix[2] * (inputMatrix[3] * inputMatrix[7] - inputMatrix[4] * inputMatrix[6]);
    
    if (!det){
        int i;
        for (i = 0; i < 9; i++){
            outputMatrix[i] = 0;
        }
        return;
    } 

    double invdet = (double) 1 / det;

    val = round((inputMatrix[4] * inputMatrix[8] - inputMatrix[5] * inputMatrix[7]) * invdet);
    MATHUTILS_SATURATE16(val);
    outputMatrix[0] = (int16_t) val;

    val = round((inputMatrix[2] * inputMatrix[7] - inputMatrix[1] * inputMatrix[8]) * invdet);
    MATHUTILS_SATURATE16(val);
    outputMatrix[1] = (int16_t) val;

    val = round((inputMatrix[1] * inputMatrix[5] - inputMatrix[2] * inputMatrix[4]) * invdet);
    MATHUTILS_SATURATE16(val);
    outputMatrix[2] = (int16_t) val;

    val = round((inputMatrix[5] * inputMatrix[6] - inputMatrix[3] * inputMatrix[8]) * invdet);
    MATHUTILS_SATURATE16(val);
    outputMatrix[3] = (int16_t) val;

    val = round((inputMatrix[0] * inputMatrix[8] - inputMatrix[6] * inputMatrix[2]) * invdet);
    MATHUTILS_SATURATE16(val);
    outputMatrix[4] = (int16_t) val;

    val = round((inputMatrix[3] * inputMatrix[2] - inputMatrix[0] * inputMatrix[5]) * invdet);
    MATHUTILS_SATURATE16(val);
    outputMatrix[5] = (int16_t) val;

    val = round((inputMatrix[3] * inputMatrix[7] - inputMatrix[6] * inputMatrix[4]) * invdet);
    MATHUTILS_SATURATE16(val);
    outputMatrix[6] = (int16_t) val;

    val = round((inputMatrix[6] * inputMatrix[1] - inputMatrix[0] * inputMatrix[7]) * invdet);
    MATHUTILS_SATURATE16(val);
    outputMatrix[7] = (int16_t) val;

    val = round((inputMatrix[0] * inputMatrix[4] - inputMatrix[1] * inputMatrix[3]) * invdet);
    MATHUTILS_SATURATE16(val);
    outputMatrix[8] = (int16_t) val;

}
