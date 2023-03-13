#ifndef __PYRAMID_ADD_SUB_H__
#define __PYRAMID_ADD_SUB_H__

#include <dis_types.h>

void  pyramid_add_iu8_ou16(DIS_PYRAMID const* A,
                           DIS_PYRAMID const* B,
                           DIS_PYRAMID*       C);

void  pyramid_sub_iu8_os16(DIS_PYRAMID const* A,
                           DIS_PYRAMID const* B,
                           DIS_PYRAMID*       C);
#endif