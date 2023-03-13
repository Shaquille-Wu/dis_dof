#ifndef _LDC_K_FIT_H
#define _LDC_K_FIT_H

#ifdef __cplusplus
extern "C"{
#endif

int   fit_ldc_k(float const* ldc_lut, 
                int          lut_max,
                float        ldc_lut_step_inv,
                float        focal_length,
                float*       k);   //k0 - k4, 5 k

#ifdef __cplusplus
}
#endif

#endif