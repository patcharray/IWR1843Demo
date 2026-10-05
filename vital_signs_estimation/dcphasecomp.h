#ifndef DCPHASECOMP_H
#define DCPHASECOMP_H

#include <transpose.h>
#include <3x3inverse.h>
#include <matmul.h>
#include <ti/common/sys_types.h>

/* mathlib's common.h redefines TRUE/FALSE (as ((t_bool)true)/((t_bool)false))
 * which clashes with xdc/std.h's 1U/0U. Same meaning, different spelling, but
 * the compiler flags it and --emit_warnings_as_errors makes it fatal. Clear
 * them around the include, then restore the xdc spelling, so the result is the
 * same no matter which header pulls mathlib in first. */
#undef TRUE
#undef FALSE
#include <ti/mathlib/mathlib.h>
#undef TRUE
#undef FALSE
#define TRUE  1U
#define FALSE 0U

#include <DSP_mat_mul.h>


void createAMatrix(cmplx16ImRe_t *cube, int range, int16_t *outputMatrix);
void moorePenrosePseudoinverse(int16_t *inputMatrix, int range, int16_t *outputMatrix);
void getAbs(cmplx16ImRe_t *inputVector, int range, int16_t *abs);
void getYVector(int16_t *inputMatrix, int16_t *b, int16_t *Y);
void getCenter(int16_t *Y, cmplx16ImRe_t *X);
void subtractCenter(cmplx16ImRe_t *cube, int range, cmplx16ImRe_t *X);
void DSPhaseComp(cmplx16ImRe_t *radarCube, int range);

#endif
