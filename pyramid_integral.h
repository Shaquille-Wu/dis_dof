#ifndef __PYRAMID_INTEGRAL_H__
#define __PYRAMID_INTEGRAL_H__

#include <dis_types.h>

void  pyramid_integral_iu8_ou32(DIS_PYRAMID const* A,
                                DIS_PYRAMID*       B);

void  pyramid_integral_sqr_iu8_ou64(DIS_PYRAMID const* A,
                                    DIS_PYRAMID*       B);

#endif