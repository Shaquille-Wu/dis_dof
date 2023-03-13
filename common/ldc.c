#include "ldc.h"
#include <math.h>

float lut_lookup_floating(float const* lut, float inval, int indMax, float stepInv, int *status)
{
  *status = 0;
  float ind = inval * stepInv;
  if (ind >= (float)indMax){
    *status = -1;
    return lut[indMax];
  }

  int   N       = (int)ind;
  float indMinN = ind - (float)N;
  return (1.0f - indMinN)*lut[N] + indMinN * lut[N + 1];
}

int distort_2_undistort(LensDistortionCorrection const* ldc, 
                        float                           point_in[2], 
                        float                           point_out[2], 
                        float*                          rdSq_out)
{
  int   status = 0;
  float diffX, diffY;
  float ruDivRd;

  diffX = point_in[0] - ldc->distCenterX;
  diffY = point_in[1] - ldc->distCenterY;
  *rdSq_out = diffX*diffX + diffY*diffY;

  ruDivRd      = lut_lookup_floating(ldc->lut_d2u, 
                                     *rdSq_out, 
                                     ldc->lut_d2u_indMax, 
                                     ldc->lut_d2u_stepInv, 
                                     &status);
  point_out[0] = diffX * ruDivRd + ldc->distCenterX;
  point_out[1] = diffY * ruDivRd + ldc->distCenterY;

  return status;
}

int undistort_2_distort(LensDistortionCorrection const* ldc, 
                        float                           point_in[2], 
                        float                           point_out[2])
{
  int status = 0;
  float diffX, diffY;
  float lut_in_val;
  float lut_out_val;

  diffX = point_in[0] - ldc->distCenterX;
  diffY = point_in[1] - ldc->distCenterY;

  float ru;
  ru = sqrtf(diffX*diffX + diffY*diffY);
  lut_in_val  = atanf(ru*ldc->distFocalLengthInv);

  lut_out_val = lut_lookup_floating(ldc->lut_u2d, lut_in_val, ldc->lut_u2d_indMax, ldc->lut_u2d_stepInv, &status);

  if (ru==0){
    point_out[0] = ldc->distCenterX;
    point_out[1] = ldc->distCenterY;
  }else{
    point_out[0] = lut_out_val * diffX / ru + ldc->distCenterX;
    point_out[1] = lut_out_val * diffY / ru + ldc->distCenterY;
  }

  return status;
}
