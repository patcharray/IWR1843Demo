#include <matmul.h>

void matMul(const int16_t *inputMatrix1, const int rows1, const int columns1, const int16_t *inputMatrix2, const int rows2, const int columns2, int16_t *outputMatrix){

    int i, j, k;

    int32_t a;

    if (columns1 != rows2){
        
    }

    for(i = 0; i < rows1; i++){
        for(j = 0; j < columns2; j++){
            a = 0;
            for(k = 0; k < rows2; k++){
                a += inputMatrix1[i*columns1 + k] * inputMatrix2[k*columns2 + j];
            }
            MATHUTILS_SATURATE16(a);
            outputMatrix[i*columns2 + j] = a;
        }
    }
}
