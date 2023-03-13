#include "ldc_k_fit.h"
#include <math.h>
#include "align_mem.h"
#include <string.h>
#include <time.h>
#include <stdio.h>
#include "srv_math.h"

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

static void generate_uniq_rand(int len, int* hash_table, int* out){
  int    i          = 0 ;
  int    idx        = 0 ;
  int    remain_cnt = len ;
  int    last_val   = 0;
  int    tmp        = 0;

  if(len <= 0)
    return ;

  if(1 == len){
    out[0] = 0;
  }

  for(i = 0 ; i < len ; i ++){
    hash_table[i] = i;
  }

  for(i = 0 ; i < len ; i ++){
    if(remain_cnt > 1){
      idx                         = rand() % (remain_cnt - 1);
      last_val                    = hash_table[remain_cnt - 1];
      tmp                         = hash_table[idx];
      hash_table[idx]             = last_val;
      hash_table[remain_cnt - 1]  = tmp;
      out[i]                      = tmp;
    }else{
      out[i] = hash_table[0];
    }
  }

}

int   fit_ldc_k(float const* ldc_lut, 
                int          lut_max,
                float        ldc_lut_step_inv,
                float        focal_length,
                float*       k){
  double    input_max         = M_PI * 0.5;
  int       fit_samples       = 1024;
  unsigned char* work_buf     = (unsigned char*)alloc_mem_align(
                                                         2 * fit_samples * sizeof(double) + 
                                                         fit_samples * sizeof(int));
  double*   input_samples     = (double*)work_buf;
  double*   input_samples_y   = input_samples + fit_samples;
  int*      valid_idx         = (int*)(input_samples_y + fit_samples);
  int       i                 = 0;
  int       j                 = 0;
  int       fit_valid         = 0;
  for(i = 0 ; i < fit_samples ; i ++){
    input_samples[i] = (((double)i) * input_max) / ((double)fit_samples);
  }

  for(i = 0 ; i < fit_samples ; i ++){
    double  ind = input_samples[i] * (double)ldc_lut_step_inv;
    if (ind < lut_max){
      int    N                   = (int)ind;
      double indMinN             = ind - (double)N;
      valid_idx[fit_valid]       = i;
      input_samples_y[fit_valid] = (1.0 - indMinN) * ldc_lut[N] + indMinN * ldc_lut[N + 1];
      input_samples_y[fit_valid] = input_samples_y[fit_valid] / (double)focal_length;
      fit_valid ++;
    }
  }
  if(fit_valid < 20){
    free_mem_align(work_buf);
    return -1;
  }

  double sum_x[4 * 4] = {
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
  };
  double sum_x_inv[4 * 4] = {
    0.0
  };
  double sum_y[4] = {
    0.0, 0.0, 0.0, 0.0
  };

  for(i = 0 ; i < fit_valid ; i ++){
    double cur_sample = input_samples[valid_idx[i]];
    double theta      = cur_sample;
    double theta_2    = cur_sample * cur_sample;
    double theta_3    = theta   * theta_2;
    double theta_5    = theta_3 * theta_2;
    double theta_7    = theta_5 * theta_2;
    double theta_9    = theta_7 * theta_2;

    sum_x[0] += theta_3 * theta_3;
    sum_x[1] += theta_5 * theta_3;
    sum_x[2] += theta_7 * theta_3;
    sum_x[3] += theta_9 * theta_3;

    sum_x[4] += theta_3 * theta_5;
    sum_x[5] += theta_5 * theta_5;
    sum_x[6] += theta_7 * theta_5;
    sum_x[7] += theta_9 * theta_5;

    sum_x[8]  += theta_3 * theta_7;
    sum_x[9]  += theta_5 * theta_7;
    sum_x[10] += theta_7 * theta_7;
    sum_x[11] += theta_9 * theta_7;

    sum_x[12] += theta_3 * theta_9;
    sum_x[13] += theta_5 * theta_9;
    sum_x[14] += theta_7 * theta_9;
    sum_x[15] += theta_9 * theta_9;

    sum_y[0]  += (input_samples_y[i] * theta_3 - theta * theta_3);
    sum_y[1]  += (input_samples_y[i] * theta_5 - theta * theta_5);
    sum_y[2]  += (input_samples_y[i] * theta_7 - theta * theta_7);
    sum_y[3]  += (input_samples_y[i] * theta_9 - theta * theta_9);
  }
  mat_inv(sum_x, sum_x_inv, 4, 4);

  k[0] = 1.0f;
  for(i = 0 ; i < 4 ; i ++){
    printf("[ %.6f, %.6f, %.6f, %.6f ],\n", sum_x[4 * i], sum_x[4 * i + 1], sum_x[4 * i + 2], sum_x[4 * i + 3]);
    k[i + 1] = sum_x_inv[4 * i]     * sum_y[0] +
               sum_x_inv[4 * i + 1] * sum_y[1] +
               sum_x_inv[4 * i + 2] * sum_y[2] +
               sum_x_inv[4 * i + 3] * sum_y[3];

  }
  free_mem_align(work_buf);
}