#include <transpose.h>

void transpose(const int16_t *inputMatrix, int rows, int columns, int16_t *outputMatrix)
{
    int i, j;
    for (i = 0; i < rows; i++) {
        for (j = 0; j < columns; j++) {
            outputMatrix[j * rows + i] = inputMatrix[i * columns + j];
        }
    }
}
