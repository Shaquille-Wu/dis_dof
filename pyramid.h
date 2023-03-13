#ifndef __PYRAMID_H__
#define __PYRAMID_H__

#include <dis_types.h>

#ifdef __cplusplus
extern "C"{
#endif

int  create_pyramid(int            src_width, 
                    int            src_height, 
                    int            pixel_type,
                    int            pyramid_level,
                    int            pad,
                    DIS_PYRAMID*   pyramid);

int  create_pyramid_with_size(unsigned int const*    src_width, 
                              unsigned int const*    src_height, 
                              int                    pixel_type,
                              int                    pyramid_level,
                              int                    pad,
                              DIS_PYRAMID*           pyramid);

int  build_gray_pyramid(unsigned char const*  src_image,
                        int                   src_line_size,
                        int                   padding_zero,
                        DIS_PYRAMID*          pyramid);

int  sub_pyramid(DIS_PYRAMID const* A,
                 DIS_PYRAMID const* B,
                 DIS_PYRAMID*       C);

int  add_pyramid(DIS_PYRAMID const* A,
                 DIS_PYRAMID const* B,
                 DIS_PYRAMID*       C);

int  grad_xy_pyramid(DIS_PYRAMID const* A,
                     DIS_PYRAMID*       B);

int  integral_pyramid(DIS_PYRAMID const* A,
                      DIS_PYRAMID*       B);

int  integral_sqr_pyramid(DIS_PYRAMID const* A,
                          DIS_PYRAMID*       B);

int  destroy_pyramid(DIS_PYRAMID*  pyramid);

#ifdef __cplusplus
}
#endif

#endif