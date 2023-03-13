#include "pyramid_add_sub.h"

void  pyramid_add_iu8_ou16(DIS_PYRAMID const* A,
                           DIS_PYRAMID const* B,
                           DIS_PYRAMID*       C){
  unsigned int  l = 0, i = 0, j = 0;
  for(l = 0 ; l < A->level ; l ++){
    unsigned char const*  A_ptr        = A->buf[l];
    unsigned char const*  B_ptr        = B->buf[l];
    unsigned short int*   C_ptr        = (unsigned short int*)(C->buf[l]);
    unsigned int          cur_width    = A->width[l];
    unsigned int          cur_height   = A->height[l];
    unsigned int          A_line_size  = A->line_size[l];
    unsigned int          B_line_size  = B->line_size[l];
    unsigned int          C_line_size  = C->line_size[l];
    unsigned int          C_line_size2 = (C_line_size >> 1);
    if((A_line_size == B_line_size) && (2 * A_line_size == C_line_size)){
      unsigned int  AB_img_size = A_line_size * (2 * A->pad + A->height[l]);
      for(i = 0 ; i < AB_img_size ; i ++){
        C_ptr[i] = (unsigned short)(A_ptr[i]) + (unsigned short)(B_ptr[i]);
      }
    }else{
      for(i = 0 ; i < cur_height ; i ++){
        for(j = 0 ; j < cur_width ; j ++){
          C_ptr[j] = (unsigned short)(A_ptr[j]) + (unsigned short)(B_ptr[j]);
        }
        A_ptr += A_line_size;
        B_ptr += B_line_size;
        C_ptr += C_line_size2;
      }
    }
  }
}

void  pyramid_sub_iu8_os16(DIS_PYRAMID const* A,
                           DIS_PYRAMID const* B,
                           DIS_PYRAMID*       C){
  unsigned int  l = 0, i = 0, j = 0;
  for(l = 0 ; l < A->level ; l ++){
    unsigned char const*  A_ptr        = A->buf[l];
    unsigned char const*  B_ptr        = B->buf[l];
    short int*            C_ptr        = (short int*)(C->buf[l]);
    unsigned int          cur_width    = A->width[l];
    unsigned int          cur_height   = A->height[l];
    unsigned int          A_line_size  = A->line_size[l];
    unsigned int          B_line_size  = B->line_size[l];
    unsigned int          C_line_size  = C->line_size[l];
    unsigned int          C_line_size2 = (C_line_size >> 1);
    if((A_line_size == B_line_size) && (2 * A_line_size == C_line_size)){
      unsigned int  AB_img_size = A_line_size * (2 * A->pad + A->height[l]);
      for(i = 0 ; i < AB_img_size ; i ++){
        C_ptr[i] = (short)(A_ptr[i]) - (short)(B_ptr[i]);
      }
    }else{
      for(i = 0 ; i < cur_height ; i ++){
        for(j = 0 ; j < cur_width ; j ++){
          C_ptr[j] = (short)(A_ptr[j]) - (short)(B_ptr[j]);
        }
        A_ptr += A_line_size;
        B_ptr += B_line_size;
        C_ptr += C_line_size2;
      }
    }
  }
}