#ifndef VITALSIGNS_H
#define VITALSIGNS_H

#include <dcphasecomp.h>
#include <phaseunwrap.h>
#include <dcphaseremove.h>
#include <ellipticFilter.h>
#include <converttocmplx.h>
#include <vibrationfft.h>
#include <findmaxbin.h>
#include <ti/common/sys_types.h>
#include <string.h>

/* See note in dcphasecomp.h: clear TRUE/FALSE around mathlib, then restore. */
#undef TRUE
#undef FALSE
#include <ti/mathlib/mathlib.h>
#undef TRUE
#undef FALSE
#define TRUE  1U
#define FALSE 0U

#include <ti/dsplib/src/DSPF_sp_fftSPxSP/DSPF_sp_fftSPxSP.h>
#include <ti/dsplib/src/DSPF_sp_maxidx/DSPF_sp_maxidx.h>
#include <math.h>

void vitalSignsInit(void);
void vitalSigns(cmplx16ImRe_t *radarCube, uint16_t range, float *breathingRate, float *heartRate);

#endif
