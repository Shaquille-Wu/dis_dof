#ifndef _SRV_MATH_H
#define _SRV_MATH_H

#include <math.h>

#define  SEVX_EPS_F64    1e-12f

#ifdef __cplusplus
extern "C"{
#endif

static inline int mat_inv(double const*  src, 
                          double*        dst, 
                          unsigned int   size,
                          unsigned int   stride_size){
  unsigned int i = 0, j = 0, k = 0;
  unsigned int mat_size = size * stride_size;
  double mat_buf[size * stride_size];
  double max = 0.0, tmp = 0.0;
  for(i = 0 ; i < mat_size ; i ++){
    mat_buf[i] = src[i];
  }
  // set out matrix to identity
  for(i = 0 ; i < size ; i ++) {
    dst[i * stride_size + i] = 1.0;
  }
  for(i = 0 ; i < size ; i ++) {       // col
    max = mat_buf[(i * stride_size) + i];
    k   = i;
    for (j = i + 1; j < size; j++){   //row
      if (fabs(mat_buf[(j * stride_size) + i]) > fabs(max)){
          max = mat_buf[(j * stride_size) + i];
          k   = j;
      }
    }

    if (k != i){
      for (j = 0; j < size; j++){
        unsigned int idx1 = (i * stride_size) + j;
        unsigned int idx2 = (k * stride_size) + j;
        tmp           = mat_buf[idx1];
        mat_buf[idx1] = mat_buf[idx2];
        mat_buf[idx2] = tmp;

        tmp           = dst[idx1];
        dst[idx1]     = dst[idx2];
        dst[idx2]     = tmp;
      }
    }
    if (fabs(mat_buf[i * stride_size + i]) <= SEVX_EPS_F64){
      return -1;  //singular
    }
    tmp = mat_buf[i * stride_size + i];
    for (j = 0; j < size; j++){    //rows
      unsigned int idx = i * stride_size + j;
      mat_buf[idx] = mat_buf[idx] / tmp;
      dst[idx]     = dst[idx] / tmp;
    }
    for (j = 0; j < size; j++){  //rows
      if (j != i){
        tmp = mat_buf[j * stride_size + i];
        for (k = 0; k < size; k++) {  //rows
            unsigned int idx1 = j * stride_size + k;
            unsigned int idx2 = i * stride_size + k;
            mat_buf[idx1]     = mat_buf[idx1] - mat_buf[idx2] * tmp;
            dst[idx1]         = dst[idx1] - dst[idx2] * tmp;
        }
      }
    }
  }
  return 0;
}

static inline void mat_mul_3x3(float const* A, float const* B, float* C){
  C[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
  C[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
  C[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
  C[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
  C[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
  C[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
  C[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
  C[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
  C[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
};

static inline void mat_mul_d(double const*  A, 
                             double const*  B, 
                             unsigned int   M, 
                             unsigned int   N, 
                             unsigned int   K, 
                             double*        C){
  for(unsigned int m = 0 ; m < M ; m ++){
    for(unsigned int k = 0 ; k < K ; k ++){
      double sum = 0.0;
      for(unsigned int n = 0 ; n < N ; n ++){
        sum += A[m * N + n] * B[n * K + k];
      }
      C[m * K + k] = sum;
    }
  }
}

static inline void mat_mul(float const*  A, 
                           float const*  B, 
                           unsigned int  M, 
                           unsigned int  N, 
                           unsigned int  K, 
                           float*        C){
  for(unsigned int m = 0 ; m < M ; m ++){
    for(unsigned int k = 0 ; k < K ; k ++){
      float sum = 0.0;
      for(unsigned int n = 0 ; n < N ; n ++){
        sum += A[m * N + n] * B[n * K + k];
      }
      C[m * K + k] = sum;
    }
  }
}

static inline double norm_vec3_d(double const* v){
  double res = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  res = sqrt(res);
  return res;
}

static inline double norm_vec3_f(float const* v){
  float res = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  res = sqrtf(res);
  return res;
}

static inline double norm_2_vec_d(double const* v, int len){
  double res = 0.0;
  for(int i = 0 ; i < len ; i ++){
    res += v[i] * v[i];
  }
  return res;
}

static inline void mat_transpose_d(double const* A, int M, int N, double* AT){
  for(int i = 0 ; i < M ; i ++){
    for(int j = 0 ; j < N ; j ++){
      AT[j * M + i] = A[i * N + j];
    }
  }
}

#ifdef __cplusplus
}
#endif

#endif