#include <dcphasecomp.h>

#define DCPHASECOMP_MAX_RANGE 64

static int16_t AMatrixBuf[DCPHASECOMP_MAX_RANGE * 3];
static int16_t bBuf[DCPHASECOMP_MAX_RANGE];
static int16_t inputMatrixTransBuf[DCPHASECOMP_MAX_RANGE * 3];

void createAMatrix(cmplx16ImRe_t *cube, int range, int16_t *outputMatrix){
    int i;

    for(i = 0; i < range; i++){
        outputMatrix[i * 3] = (int16_t) 1;
        outputMatrix[i * 3 + 1] = (-2) * cube[i].real;
        outputMatrix[i * 3 + 2] = (-2) * cube[i].imag;
    }
}

void moorePenrosePseudoinverse(int16_t *inputMatrix, int range, int16_t *outputMatrix){
    int16_t *inputMatrixTrans = inputMatrixTransBuf;
    int16_t productMatrix[3 * 3];
    int16_t productInverse[3 * 3];

    transpose(inputMatrix, range, 3, inputMatrixTrans);
    matMul(inputMatrixTrans, 3, range, inputMatrix, range, 3,  productMatrix);
    matrix3x3inverse(productMatrix, productInverse);
    matMul(productInverse, 3, 3, inputMatrixTrans, 3, range, outputMatrix);
}

void getAbs(cmplx16ImRe_t *inputVector, int range, int16_t *magnitude){
    int i;

    for (i = 0; i < range; i++){
        magnitude[i] = - sqrtsp(inputVector[i].real * inputVector[i].real + inputVector[i].imag * inputVector[i].imag);
    }
}

void getYVector(int16_t *inputMatrix, int16_t *b, int16_t *Y){
    matMul(inputMatrix, 3, 3, b, 3, 1, Y);
    Y[2] = - Y[2];
}

void getCenter(int16_t *Y, cmplx16ImRe_t *X){
    X->real = Y[1];
    X->imag = Y[2];
}

void subtractCenter(cmplx16ImRe_t *cube, int range, cmplx16ImRe_t *X){
    int i;

    for (i = 0; i < range; i++){
        cube[i].real -= X->real;
        cube[i].imag -= X->imag;
    }
}

void DSPhaseComp(cmplx16ImRe_t *radarCube, int range){
    int16_t *AMatrix = AMatrixBuf;
    int16_t *b = bBuf;
    int16_t Y[3];
    cmplx16ImRe_t X;

    createAMatrix(radarCube, range, AMatrix);
    moorePenrosePseudoinverse(AMatrix, range, AMatrix);
    getAbs(radarCube, range, b);
    getYVector(AMatrix, b, Y);
    getCenter(Y, &X);
    subtractCenter(radarCube, range, &X);
}
