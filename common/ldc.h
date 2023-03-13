#ifndef _LDC_H
#define _LDC_H

#include <srv_def.h>
#include <srv_types.h>

#ifdef __cplusplus
extern "C"{
#endif

float lut_lookup_floating(float const*  lut, 
                          float         inval, 
                          int           indMax, 
                          float         stepInv, 
                          int*          status);

int distort_2_undistort(LensDistortionCorrection const* ldc, 
                        float                           point_in[2], 
                        float                           point_out[2], 
                        float*                          rdSq_out);

int undistort_2_distort(LensDistortionCorrection const* ldc, 
                        float                           point_in[2], 
                        float                           point_out[2]);


#ifdef __cplusplus
}
#endif

#endif