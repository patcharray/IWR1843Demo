#ifndef MEANCHIRPS_H
#define MEANCHIRPS_H

#include <ti/common/sys_types.h>

void meanChirps(cmplx16ImRe_t *radarCube, uint16_t range, uint16_t numChirps, cmplx16ImRe_t *mean);

#endif
