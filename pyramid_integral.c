#include "pyramid_integral.h"
#include <string.h>

void  pyramid_integral_iu8_ou32(DIS_PYRAMID const* A,
                                DIS_PYRAMID*       B){
  memset(B->mem, 0, B->total_buf_size);
  for(int l = 0 ; l < A->level ; l ++){
    unsigned int          src_line_size = A->line_size[l];
    unsigned char const*  src_ptr       = A->buf[l] + 
                                     A->pad * src_line_size +
                                     A->pad;
    unsigned int     width         = A->width[l];
    unsigned int     height        = A->height[l];
    unsigned int     dst_line_size = B->line_size[l] >> 2;   
    unsigned int*    dst_ptr       = (unsigned int*)(B->buf[l] + 
                                      B->pad * B->line_size[l] +
                                      B->pad * 4);
    int i = 0, j = 0;
    unsigned int  cur_int = 0;
    unsigned int  cur_val = 0;
    for (j = 0; j < width ; j++){
      cur_val   += src_ptr[j];
      cur_int    = cur_val;
      dst_ptr[j] = cur_int;
    }
    cur_int = 0;
    cur_val = 0;
    for (i = 1; i < height ; i++){
      for (j = 0; j < width ; j++){
        cur_val += src_ptr[i * src_line_size + j] ;
        cur_int  = cur_val + dst_ptr[(i - 1) * dst_line_size + j];
        dst_ptr[i * dst_line_size + j] = cur_int;
      }
      cur_val = 0;
      cur_int = 0;
    }
  }
}

void  pyramid_integral_sqr_iu8_ou64(DIS_PYRAMID const* A,
                                    DIS_PYRAMID*       B){
  memset(B->mem, 0, B->total_buf_size);
  for(int l = 0 ; l < A->level ; l ++){
    unsigned int          src_line_size = A->line_size[l];
    unsigned char const*  src_ptr       = A->buf[l] + 
                                     A->pad * src_line_size +
                                     A->pad;
    unsigned int         width         = A->width[l];
    unsigned int         height        = A->height[l];
    unsigned int         dst_line_size = B->line_size[l] >> 3;   
    unsigned long long*  dst_ptr       = (unsigned long long*)(B->buf[l] + 
                                          B->pad * B->line_size[l] +
                                          B->pad * 8);
    int i = 0, j = 0;
    unsigned long long  cur_int = 0;
    unsigned long long  cur_val = 0;
    for (j = 0; j < width ; j++){
      unsigned int cur_src = src_ptr[j];
      cur_val   += (unsigned long long)(cur_src * cur_src);
      cur_int    = cur_val;
      dst_ptr[j] = cur_int;
    }
    cur_int = 0;
    cur_val = 0;
    for (i = 1; i < height ; i++){
      for (j = 0; j < width ; j++){
        unsigned int cur_src = src_ptr[i * src_line_size + j];
        cur_val += (unsigned long long)(cur_src * cur_src) ;
        cur_int  = cur_val + dst_ptr[(i - 1) * dst_line_size + j];
        dst_ptr[i * dst_line_size + j] = cur_int;
      }
      cur_val = 0;
      cur_int = 0;
    }
  }
}
